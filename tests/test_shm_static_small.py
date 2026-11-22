"""Near-ready static gravity recipe, diagnostics, and PSD candidate export."""
import csv
import json
from types import SimpleNamespace
import numpy as np
import pytest
from casbot_calib.field_gravity import write_static_diagnostics
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.shm_data import NAMES, ROOT
from casbot_calib.shm_ready import ready_position
from casbot_calib.stages.dynamics import ApproachDisplacement, analyse_h1, load_h1_recipe, plan_h1




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


def test_solver_unavailable_report_from_synthetic_capture(tmp_path, monkeypatch):
    from test_wave2b_h2_h3 import _write_synthetic_capture, _reconstruct_bpoly_from_reference
    from casbot_calib.model_runtime import model_for
    robot_cfg = load_robot_config(dict(initial_hold_s=.01, final_hold_s=.01, anchor_transition_s=.1))
    recipe = load_h1_recipe(dict(mode='small', ready_pose=False, small=dict(
        parallel_arms=False, direct_pose_chain=False,
        joints=['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint'],
        target_offset_deg=.1, approach_offset_deg=.03,
        timing=dict(initial_hold_s=.01, minimum_transition_s=.1, minimum_paired_approach_s=.1,
                    settle_s=.05, measure_s=.1, final_hold_s=.01))))
    plan_dir = tmp_path / 'plan'
    plan_h1(ready_position(), robot_cfg, recipe, plan_dir)
    bundle = json.loads((plan_dir / 'reference.json').read_text())
    poly = _reconstruct_bpoly_from_reference(bundle)
    engine, _ = model_for(robot_cfg)
    _write_synthetic_capture(tmp_path / 'capture', bundle, poly,
        lambda t: poly(t, nu=1), lambda t, q, v: engine.gravity_batch(q), robot_cfg)
    from casbot_calib import rigid_identification
    monkeypatch.setattr(rigid_identification, 'fit_rigid', lambda *a, **kw: (_ for _ in ()).throw(RuntimeError('test solver failure')))
    unavailable = analyse_h1(plan_dir, tmp_path / 'capture', tmp_path / 'unavailable')
    assert unavailable['execution_completed'] and unavailable['collection_complete'] and not unavailable['analysis_available']
    assert not unavailable['fit_succeeded'] and 'test solver failure' in unavailable['analysis_unavailable_reason']
    assert unavailable['static_diagnostics']['available_windows'] == 8
