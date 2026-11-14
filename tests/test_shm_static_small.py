"""Near-ready static gravity recipe, diagnostics, and PSD candidate export."""
import csv
import json
import os
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from casbot_calib.field_gravity import write_static_diagnostics
from casbot_calib.pipeline.native import execute_with_start_replan, native_snapshot, open_mock_context
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.shm_data import NAMES, ROOT
from casbot_calib.shm_ready import ready_position
from casbot_calib.stages.dynamics import ApproachDisplacement, analyse_h1, load_h1_recipe, plan_h1

BIN = Path(os.environ.get('CASBOT_SHM_TEST_BIN', ROOT / 'build/humble/cpp/src/shm'))


def test_default_h1_small_stays_in_twenty_degree_shoulder_range(tmp_path):
    robot_cfg = load_robot_config()
    recipe = load_h1_recipe()
    assert recipe['mode'] == 'small'
    assert recipe['small']['target_offset_deg'] == 15.0 and recipe['small']['approach_offset_deg'] == 5.0
    m = plan_h1(ready_position(), robot_cfg, recipe, tmp_path / 'plan')
    assert len(m['trials']) == 8
    assert json.loads((tmp_path / 'plan/collision_reference.json').read_text())['passed']
    for row in m['static_motions']:
        assert row['labels'].count('static_measure') == 1
        states = np.array(row['states'])
        np.testing.assert_allclose(states[:, 0, 1], np.deg2rad(20.0), atol=1e-15)
        np.testing.assert_allclose(states[:, 0, 8], -np.deg2rad(20.0), atol=1e-15)
        assert np.max(np.abs(states[:, 0, [0, 7]])) <= np.deg2rad(20.0) * (1 + 1e-12)
        assert len(np.flatnonzero(np.ptp(states[:, 0, :], axis=0) > 1e-10)) == 1


def test_static_diagnostics_use_actual_q_and_keep_pair_pose_difference(tmp_path):
    rows = []
    for sign, q, tau in ((-1, 0.09, 0.2), (1, 0.1, 0.3)):
        rows.append(dict(
            name=str(sign),
            conditions=dict(batch='train', pose='p', approach=sign, target_q_rad=[0.5] * 14),
            q=np.full(14, q),
            torque=np.full(14, tau),
        ))
    summary = write_static_diagnostics(rows, SimpleNamespace(gravity=lambda q: 2 * q), tmp_path, NAMES)
    value = json.loads((tmp_path / 'static_diagnostics.json').read_text())
    assert summary['available_windows'] == 2 and summary['paired_targets'] == 1
    np.testing.assert_allclose(value['windows'][1]['nominal_gravity_nm'], 0.2)
    np.testing.assert_allclose(value['pairs'][0]['q_plus_minus_rad'], 0.01)
    np.testing.assert_allclose(value['pairs'][0]['residual_half_difference_nm'], 0.04)
    with (tmp_path / 'static_comparison.csv').open() as f:
        assert len(list(csv.DictReader(f))) == 28


def test_small_static_rejects_expanded_scope_without_clipping():
    c = load_h1_recipe()
    c['small']['target_offset_deg'] = 18.0
    c['small']['approach_offset_deg'] = 5.0
    with pytest.raises(ValueError, match='20°'):
        load_h1_recipe(c)


def test_observed_approach_is_not_replaced_by_reference_or_velocity():
    metadata = dict(labels=['paired_approach'], knots_s=[0, 2], states=[[np.zeros(14)], [np.full(14, 0.1)]])
    span = ApproachDisplacement(dict(begin_s=1), metadata)
    for ns in range(1_000_000_000, 3_000_000_000, 1_000_000):
        q = np.full(14, 0.0 if ns < 2_950_000_000 else 0.001)
        span.add(dict(observation_time_ns=ns, clock_segment=0, time_ambiguous=False, q=q), ns)
    result = span.result()
    assert result['available']
    np.testing.assert_allclose(result['reference_displacement_rad'], 0.1)
    np.testing.assert_allclose(result['observed_displacement_rad'], 0.001)
    assert result['first_rows'] == result['last_rows'] == 50


@pytest.mark.skipif(not (BIN / 'shm_executor').exists(), reason='build native physics')
def test_eight_windows_physics_fit_and_solver_unavailable_report(tmp_path, monkeypatch):
    native = BIN / 'shm_executor'
    robot_cfg = load_robot_config(dict(initial_hold_s=0.05, final_hold_s=0.05, anchor_transition_s=0.1))
    recipe = load_h1_recipe(dict(
        mode='small',
        ready_pose=True,
        small=dict(
            joints=['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint'],
            target_offset_deg=0.1,
            approach_offset_deg=0.03,
            timing=dict(initial_hold_s=0.05, minimum_transition_s=0.1, minimum_paired_approach_s=0.1, settle_s=0.05, measure_s=0.1, final_hold_s=0.05),
            planning_envelope=dict(speed_rad_s=0.05, acceleration_rad_s2=0.1, jerk_rad_s3=1.0),
        ),
    ))
    with open_mock_context(tmp_path / 'stage', native, robot_cfg) as mock_ctx:
        q0 = native_snapshot(native, mock_ctx.endpoint, robot_cfg, tmp_path / 'start', peer=mock_ctx.process)
        plan_dir = tmp_path / 'plan'
        m = plan_h1(q0, robot_cfg, recipe, plan_dir)
        assert len(m['trials']) == 8
        bundle = json.loads((plan_dir / 'reference.json').read_text())
        markers = bundle['markers']
        assert len(markers) == 19 and all(b['t_ns'] > a['t_ns'] for a, b in zip(markers, markers[1:]))
        code, _, _, _, _ = execute_with_start_replan(
            initial_plan_dir=plan_dir,
            initial_plan_meta=m,
            replan_fn=lambda q, d: plan_h1(q, robot_cfg, recipe, d),
            capture_root=tmp_path / 'capture',
            executor=native,
            endpoint=mock_ctx.endpoint,
            robot_config=robot_cfg,
            hand_background=mock_ctx.background,
            peer=mock_ctx.process,
        )
    assert code == 0

    out = tmp_path / 'analysis'
    report = analyse_h1(plan_dir, tmp_path / 'capture', out)
    assert report['execution_completed'] and report['analysis_available'] and report['fit_succeeded']
    assert report['static_diagnostics']['available_windows'] == 8 and report['static_diagnostics']['paired_targets'] == 4
    assert (out / 'candidate.urdf').is_file()
    assert json.loads((out / 'urdf_reload.json').read_text())['passed']
    windows = json.loads((out / 'static_windows.json').read_text())
    assert all(w['paired_approach']['available'] for w in windows)
    assert report['fit']['rank'] < report['fit']['columns']
    with (out / 'static_comparison.csv').open() as f:
        assert len(list(csv.DictReader(f))) == 112

    from casbot_calib import rigid_identification
    monkeypatch.setattr(rigid_identification, 'fit_rigid', lambda *a, **kw: (_ for _ in ()).throw(RuntimeError('test solver failure')))
    unavailable = analyse_h1(plan_dir, tmp_path / 'capture', tmp_path / 'unavailable')
    assert unavailable['execution_completed'] and unavailable['collection_complete'] and not unavailable['analysis_available']
    assert not unavailable['fit_succeeded'] and 'test solver failure' in unavailable['analysis_unavailable_reason']
    assert unavailable['static_diagnostics']['available_windows'] == 8
