"""Preparation exception is bounded to its prefix; hardware is never accessed."""
import csv
import json
import os
from pathlib import Path
import struct
import subprocess
import numpy as np
import pytest
from casbot_calib.model_runtime import model_for, urdf_for
from casbot_calib.pipeline.native import execute_with_start_replan, native_snapshot, open_mock_context
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.shm_data import ROOT
from casbot_calib.shm_fieldwork import records
from casbot_calib.shm_ready import handover_fraction, preparation, ready_position
from casbot_calib.stages.dynamics import analyse_h1, load_h1_recipe, plan_h1

BIN = Path(os.environ.get('CASBOT_SHM_TEST_BIN', ROOT / 'build/humble/cpp/src/shm'))
HANGING = np.array([
    -0.037817, 0.10047065, -0.132868, -0.016, 0.023154, 0.045726, -0.12043,
    -0.014166, -0.11496665, -0.12538533, -0.054911, 0.18399067, 0.03161545, -0.05161767,
])


@pytest.fixture
def native():
    if not (BIN / 'shm_executor').is_file():
        pytest.skip('Build native SHM first')
    return BIN / 'shm_executor'


def hanging_mock_config(tmp_path, robot_cfg, *, fault_after_commands=None):
    engine, _ = model_for(robot_cfg)
    cfg = json.loads((ROOT / 'configs/shm/h1_mock.json').read_text())
    cfg['urdf'] = str(urdf_for(robot_cfg))
    cfg['initial_q'] = HANGING.tolist()
    cfg['initial_fields']['position'] = HANGING.tolist()
    cfg['initial_fields']['effort'] = engine.gravity(HANGING).tolist()
    if fault_after_commands is not None:
        cfg['fault_after_commands'] = fault_after_commands
    fixture = tmp_path / ('hanging_fault.json' if fault_after_commands else 'hanging.json')
    fixture.write_text(json.dumps(cfg))
    return fixture


def test_preparation_uses_urdf_and_velocity_not_collision():
    c = load_robot_config()
    engine, _ = model_for(c)
    motion, proof = preparation(HANGING, engine)
    assert proof['passed'] and proof['collision_check'] == 'skipped_user_authorized_preparation_only'
    assert proof['reference_velocity']['maximum_rad_s'] <= 0.15 * (1 + 1e-9)
    np.testing.assert_array_equal(motion.states[-1][0], ready_position())
    np.testing.assert_array_equal(handover_fraction([0, proof['move_end_ns']], proof), 0.0)
    assert handover_fraction(proof['handover_end_ns'], proof) == 1.0
    bad = HANGING.copy()
    bad[0] = engine.model.joints[0].upper + 0.1
    with pytest.raises(ValueError, match='URDF'):
        preparation(bad, engine)


def test_ready_prefix_allows_known_hanging_start_but_unprepared_path_refuses(tmp_path):
    robot_cfg = load_robot_config()
    no_ready = load_h1_recipe(dict(mode='smoke', ready_pose=False, smoke_jog=dict(joint_id=12, amplitude_deg=1.0)))
    with pytest.raises(ValueError, match='碰撞'):
        plan_h1(HANGING, robot_cfg, no_ready, tmp_path / 'refused')

    with_ready = load_h1_recipe(dict(mode='smoke', ready_pose=True, smoke_jog=dict(joint_id=12, amplitude_deg=1.0)))
    root = tmp_path / 'ready'
    m = plan_h1(HANGING, robot_cfg, with_ready, root)
    p = m['preparation']
    bundle = json.loads((root / 'reference.json').read_text())
    formal = json.loads((root / 'calibration_motion.json').read_text())
    motion = json.loads((root / 'motion.json').read_text())
    proof = json.loads((root / 'collision_reference.json').read_text())
    assert proof['passed'] and proof['scope'] == 'formal_calibration_reference_only'
    assert bundle['purpose'] == 'shm_calib_ready_then_collision_checked'
    k = bundle['knots_ns'].index(p['calibration_begin_ns'])
    np.testing.assert_array_equal(bundle['states'][k], formal['states'][0])
    np.testing.assert_array_equal(bundle['states'][k + 1:], formal['states'][1:])
    assert np.all(np.asarray(bundle['states'][k])[1:] == 0)
    assert motion['conditions']['comparison_window_s'][0] > p['calibration_begin_ns'] * 1e-9
    assert all(not any(row['value']) for row in bundle['effort'] if row['t_ns'] <= p['move_end_ns'])
    assert any(any(row['value']) for row in bundle['effort'] if row['t_ns'] >= p['handover_end_ns'])


def test_native_rejects_bad_schedule_before_opening_any_channel(tmp_path, native):
    from casbot_calib.shm_reference import jog_bundle
    b = jog_bundle([0.0] * 14, 12, 0.01, 1)
    b['pd_schedule'] = [dict(t_ns=0, kp=[501.0] * 14, kd=[2.0] * 14)]
    path = tmp_path / 'bad.json'
    path.write_text(json.dumps(b))
    r = subprocess.run([str(native.with_name('shm_reference_check')), str(path)], capture_output=True, text=True)
    assert r.returncode != 0 and 'Scheduled PD' in r.stderr
    b['pd_schedule'][0].update(t_ns=1, kp=[20.0] * 14)
    path.write_text(json.dumps(b))
    r = subprocess.run([str(native.with_name('shm_reference_check')), str(path)], capture_output=True, text=True)
    assert r.returncode != 0 and 'schedule times' in r.stderr


def test_one_enable_cycle_ready_then_h1_physical_capture_and_analysis(tmp_path, native):
    robot_cfg = load_robot_config(dict(kp=30.0, kd=3.0))
    recipe = load_h1_recipe(dict(mode='smoke', ready_pose=True, smoke_jog=dict(joint_id=12, amplitude_deg=1.0)))
    mock_cfg = hanging_mock_config(tmp_path, robot_cfg)

    with open_mock_context(tmp_path / 'stage', native, robot_cfg, mock_cfg) as mock_ctx:
        q0 = native_snapshot(native, mock_ctx.endpoint, robot_cfg, tmp_path / 'snapshot', peer=mock_ctx.process)
        plan_dir = tmp_path / 'plan'
        m = plan_h1(q0, robot_cfg, recipe, plan_dir)
        code, summary, eff_plan, _, child = execute_with_start_replan(
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
    assert code == 0 and summary['execution_completed']

    with (child / 'native/events.csv').open() as f:
        names = [r['event'] for r in csv.DictReader(f)]
    assert names.count('enabling_at_start') == 1
    assert names.index('ready_begin') < names.index('ready_field_handover_begin') < names.index('h1_calibration_begin') < names.index('exiting_disable')

    with (child / 'native/commands.csv').open() as f:
        commands = list(csv.DictReader(f))
    stream = [r for r in commands if r['phase'] == '3']
    origin = int(stream[0]['send_mono_ns'])
    assert [int(r['phase']) for r in commands] == sorted(int(r['phase']) for r in commands)
    early = [r for r in stream if int(r['send_mono_ns']) - origin < m['preparation']['move_end_ns']]
    assert early and all(float(r['kp0']) == 20 and float(r['kd0']) == 2 and all(float(r['effort' + str(j)]) == 0 for j in range(14)) for r in early)
    late = [r for r in stream if int(r['send_mono_ns']) - origin >= m['preparation']['handover_end_ns']]
    assert late and all(float(r['kp0']) == 30 and float(r['kd0']) == 3 for r in late)
    middle_ns = (m['preparation']['move_end_ns'] + m['preparation']['handover_end_ns']) // 2
    middle = min(stream, key=lambda r: abs(int(r['send_mono_ns']) - origin - middle_ns))
    assert abs(float(middle['kp0']) - 25) < 0.2 and abs(float(middle['kd0']) - 2.5) < 0.11

    report = analyse_h1(eff_plan, tmp_path / 'capture', tmp_path / 'analysis')
    assert report['execution_completed'] and report['analysis_available']
    collision = json.loads((tmp_path / 'analysis/collision_received_interpolant.json').read_text())
    assert collision['passed'] and collision['excluded_rows'] > 0 and collision['collision_checked_rows'] > 1000
    assert report['calibration_begin_s'] == m['preparation']['calibration_begin_ns'] * 1e-9


def test_formal_collision_is_not_covered_by_preparation_exception(tmp_path):
    robot_cfg = load_robot_config()
    recipe = dict(mode='smoke', ready_pose=True, smoke_jog=dict(joint_id=12, amplitude_deg=1.0, anchor_rad=[0.0] * 14))
    with pytest.raises(ValueError, match='碰撞'):
        plan_h1(HANGING, robot_cfg, recipe, tmp_path / 'bad_formal')


def test_static_windows_shift_past_preparation_prefix(tmp_path):
    robot_cfg = load_robot_config(dict(initial_hold_s=0.05, final_hold_s=0.05, anchor_transition_s=0.1))
    recipe = dict(
        mode='small',
        ready_pose=True,
        small=dict(
            joints=['left_shoulder_pitch_joint'],
            target_offset_deg=5.0,
            approach_offset_deg=2.0,
            timing=dict(initial_hold_s=0.05, minimum_transition_s=0.1, minimum_paired_approach_s=0.1, settle_s=0.05, measure_s=0.2, final_hold_s=0.05),
            planning_envelope=dict(speed_rad_s=0.05, acceleration_rad_s2=0.1, jerk_rad_s3=1.0),
        ),
    )
    m = plan_h1(HANGING, robot_cfg, recipe, tmp_path / 'static')
    offset = m['preparation']['calibration_begin_ns'] * 1e-9
    assert len(m['trials']) == 4 and all(t['measure_begin_s'] > offset for t in m['trials'])
    assert m['trials'][0]['measure_end_s'] - m['trials'][0]['measure_begin_s'] == pytest.approx(0.2)
    assert json.loads((tmp_path / 'static/collision_reference.json').read_text())['passed']


def test_ten_degree_jog_retains_collision_and_slow_reference(tmp_path):
    from casbot_calib.staged_motion import derivative_bounds
    from scipy.interpolate import BPoly

    robot_cfg = load_robot_config()
    recipe = dict(mode='smoke', ready_pose=True, smoke_jog=dict(joint_id=12, amplitude_deg=10.0))
    m = plan_h1(HANGING, robot_cfg, recipe, tmp_path / 'ten_deg')
    motion = json.loads((tmp_path / 'ten_deg/calibration_motion.json').read_text())
    assert m['recipe']['smoke_jog']['amplitude_deg'] == 10.0 and m['duration_ns'] > 40_000_000_000
    assert json.loads((tmp_path / 'ten_deg/collision_reference.json').read_text())['passed']
    assert derivative_bounds(BPoly.from_derivatives(motion['knots_s'], np.asarray(motion['states']))).max() <= 0.05 * (1 + 1e-9)
    q = np.asarray(motion['states'])[:, 0, 0]
    assert q.min() == pytest.approx(-np.deg2rad(10)) and q.max() == pytest.approx(np.deg2rad(10))
    with pytest.raises(ValueError, match='10'):
        load_h1_recipe(dict(mode='smoke', smoke_jog=dict(joint_id=12, amplitude_deg=10.1)))


def test_preparation_fault_disables_without_entering_formal_reference(tmp_path, native):
    robot_cfg = load_robot_config()
    recipe = load_h1_recipe(dict(mode='smoke', ready_pose=True, smoke_jog=dict(joint_id=12, amplitude_deg=1.0)))
    mock_cfg = hanging_mock_config(tmp_path, robot_cfg, fault_after_commands=180)

    with open_mock_context(tmp_path / 'stage', native, robot_cfg, mock_cfg) as mock_ctx:
        q0 = native_snapshot(native, mock_ctx.endpoint, robot_cfg, tmp_path / 'snapshot', peer=mock_ctx.process)
        plan_dir = tmp_path / 'fault_plan'
        m = plan_h1(q0, robot_cfg, recipe, plan_dir)
        code, result, _, _, child = execute_with_start_replan(
            initial_plan_dir=plan_dir,
            initial_plan_meta=m,
            replan_fn=lambda q, d: plan_h1(q, robot_cfg, recipe, d),
            capture_root=tmp_path / 'fault',
            executor=native,
            endpoint=mock_ctx.endpoint,
            robot_config=robot_cfg,
            hand_background=mock_ctx.background,
            peer=mock_ctx.process,
        )
    assert code != 0 and len(result['attempts']) == 1 and not result['execution_completed']
    native_result = json.loads((child / 'result.json').read_text())['native']
    assert 'drive_error' in native_result['reason'] and native_result['disable_ack'] == '0'
    raw = list(records(child / 'native/commands.bin', 752))
    assert raw and all(struct.unpack_from('<H', raw[-1], 8 + 12 + j * 18)[0] == 6 for j in range(12, 26))
    assert all(struct.unpack_from('<H', row, 8 + 12 + 12 * 18)[0] != 0x80 for row in raw)
    with (child / 'native/events.csv').open() as f:
        assert 'h1_calibration_begin' not in [r['event'] for r in csv.DictReader(f)]
