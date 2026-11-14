"""SHM H1 safety, execution, start-replan, and fault-abort contracts on private physics."""
import csv
import json
import os
from pathlib import Path
import shutil
import sys
import numpy as np
import pytest
from casbot_calib import shm_fieldwork as fw
from casbot_calib.pipeline.native import (
    execute_with_start_replan,
    native_execute_once,
    native_snapshot,
    open_mock_context,
)
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.shm_data import ROOT
from casbot_calib.shm_reference import jog_bundle
from casbot_calib.stages.dynamics import TimeMean, analyse_h1, load_h1_recipe, plan_h1

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='POSIX SHM only')
NATIVE = Path(os.environ.get('CASBOT_SHM_TEST_BIN', ROOT / 'build/humble/cpp/src/shm')) / 'shm_executor'


@pytest.fixture
def fast_robot():
    return load_robot_config(dict(initial_hold_s=0.05, final_hold_s=0.05, anchor_transition_s=0.1))


@pytest.fixture
def smoke_recipe():
    return load_h1_recipe(dict(mode='smoke', ready_pose=False, smoke_jog=dict(joint_id=12, amplitude_deg=0.1)))


def test_h1_reference_keeps_continuous_collision_proof_and_explicit_gravity(tmp_path, fast_robot, smoke_recipe):
    cfg = json.loads((ROOT / 'configs/shm/h1_mock.json').read_text())
    plan_dir = tmp_path / 'plan'
    m = plan_h1(cfg['initial_q'], fast_robot, smoke_recipe, plan_dir)
    reference = json.loads((plan_dir / 'reference.json').read_text())
    assert json.loads((plan_dir / 'collision_reference.json').read_text())['passed']
    assert m['robot_config']['transport'] == 'shm_mc'
    assert reference['origin'] == 'absolute'
    assert reference['purpose'] == 'shm_calib_collision_checked'
    assert any(any(abs(t) > 0 for t in r['value']) for r in reference['effort'])
    assert all(isinstance(t, int) for t in reference['knots_ns'])


def test_collision_refusal_at_all_zero_hanging_pose(tmp_path, fast_robot, smoke_recipe):
    # All-zero pose without ready prefix has known shoulder/body clearance < 10mm.
    with pytest.raises(ValueError, match='碰撞'):
        plan_h1(np.zeros(14), fast_robot, smoke_recipe, tmp_path / 'refused')


def test_time_mean_equalizes_duplicate_groups_and_refuses_jumps():
    def compute(repeats):
        mean = TimeMean()
        for t, count in zip((0, 10, 20), repeats):
            for _ in range(count):
                mean.add(dict(observation_time_ns=t, clock_segment=0, time_ambiguous=False), np.array([t, t * t], float))
        return mean.result(), mean

    a, _ = compute((1, 1, 1))
    b, mean = compute((1, 4, 2))
    np.testing.assert_array_equal(a, b)
    assert mean.groups == 3 and mean.rows == 7

    mean = TimeMean()
    mean.add(dict(observation_time_ns=0, clock_segment=0, time_ambiguous=False), np.array([1.0]))
    mean.add(dict(observation_time_ns=10, clock_segment=1, time_ambiguous=True), np.array([2.0]))
    assert mean.result() is None


def test_only_zero_write_no_fault_start_mismatch_can_replan(tmp_path, fast_robot, smoke_recipe):
    if not NATIVE.exists():
        pytest.skip('Build SHM physics executables')
    calls = []
    with open_mock_context(tmp_path / 'stage', NATIVE, fast_robot) as mock_ctx:
        snap_dir = tmp_path / 'snapshot'
        q0 = native_snapshot(NATIVE, mock_ctx.endpoint, fast_robot, snap_dir, peer=mock_ctx.process)
        plan_dir = tmp_path / 'plan'
        m = plan_h1(q0, fast_robot, smoke_recipe, plan_dir)

        def flaky_execute(executor, endpoint, robot_config, reference_path, hand_background, output_dir, **kwargs):
            calls.append(Path(reference_path))
            if len(calls) == 1:
                output_dir = Path(output_dir)
                (output_dir / 'native').mkdir(parents=True)
                shutil.copyfile(snap_dir / 'native/status.bin', output_dir / 'native/status.bin')
                fake = dict(
                    execution_completed=False,
                    returncode=1,
                    native=dict(
                        commands_sent='0',
                        first_shm_write_mono_ns='0',
                        reason='Reference start differs from fresh measured position; no commands sent',
                    ),
                )
                (output_dir / 'result.json').write_text(json.dumps(fake))
                return 1, fake
            return native_execute_once(executor, endpoint, robot_config, reference_path, hand_background, output_dir, **kwargs)

        code, summary, _, _, _ = execute_with_start_replan(
            initial_plan_dir=plan_dir,
            initial_plan_meta=m,
            replan_fn=lambda q, d: plan_h1(q, fast_robot, smoke_recipe, d),
            capture_root=tmp_path / 'refresh',
            executor=NATIVE,
            endpoint=mock_ctx.endpoint,
            robot_config=fast_robot,
            hand_background=mock_ctx.background,
            peer=mock_ctx.process,
            execute_once_fn=flaky_execute,
        )
    assert code == 0 and len(summary['attempts']) == 2 and summary['attempts'][0]['reason'] == 'start_replan'
    assert summary['effective_plan'] is not None
    report = analyse_h1(plan_dir, tmp_path / 'refresh', tmp_path / 'after_refresh')
    assert report['execution_completed'] and report['analysis_available']
    assert report['observation_time_origin_ns'] > report['first_shm_write_mono_ns']


def test_fault_during_execution_ends_collection_without_replan(tmp_path, fast_robot, smoke_recipe):
    if not NATIVE.exists():
        pytest.skip('Build SHM physics executables')
    cfg = json.loads((ROOT / 'configs/shm/h1_mock.json').read_text())
    cfg['urdf'] = str((ROOT / 'configs/shm' / cfg['urdf']).resolve())
    cfg['fault_after_commands'] = 80
    mock_file = tmp_path / 'fault.json'
    mock_file.write_text(json.dumps(cfg))

    with open_mock_context(tmp_path / 'stage', NATIVE, fast_robot, mock_file) as mock_ctx:
        q0 = native_snapshot(NATIVE, mock_ctx.endpoint, fast_robot, tmp_path / 'snapshot', peer=mock_ctx.process)
        plan_dir = tmp_path / 'plan'
        m = plan_h1(q0, fast_robot, smoke_recipe, plan_dir)
        code, summary, _, _, _ = execute_with_start_replan(
            initial_plan_dir=plan_dir,
            initial_plan_meta=m,
            replan_fn=lambda q, d: plan_h1(q, fast_robot, smoke_recipe, d),
            capture_root=tmp_path / 'fault',
            executor=NATIVE,
            endpoint=mock_ctx.endpoint,
            robot_config=fast_robot,
            hand_background=mock_ctx.background,
            peer=mock_ctx.process,
        )
    assert code != 0 and len(summary['attempts']) == 1 and not summary['execution_completed']
    assert 'drive_error' in summary['attempts'][0]['reason']
    with pytest.raises(ValueError, match='未完成'):
        analyse_h1(plan_dir, tmp_path / 'fault', tmp_path / 'not_a_success')


def test_native_observed_velocity_ends_and_records_trigger(tmp_path):
    if not NATIVE.exists():
        pytest.skip('Build SHM physics executables')
    cfg = json.loads((ROOT / 'configs/shm/h1_mock.json').read_text())
    bundle = jog_bundle(cfg['initial_q'], 12, 0.001, 1)
    for row in bundle['states']:
        row[0] = cfg['initial_q'][:]
    bundle.update(purpose='shm_calib_collision_checked', observed_velocity_limit_rad_s=0.00001)
    reference = tmp_path / 'reference.json'
    reference.write_text(json.dumps(bundle))
    out = tmp_path / 'velocity_fault'
    assert fw.main([
        'execute', str(reference), '--executor', str(NATIVE), '--mock',
        '--mock-config', str(ROOT / 'configs/shm/h1_mock.json'), '--output', str(out), '--execute',
    ]) != 0
    report = json.loads((out / 'result.json').read_text())
    assert report['native']['reason'].startswith('observed_velocity_limit_slot_')
    assert report['native']['disable_ack'] == '1' and not report['execution_completed']
    assert report['observed_limits']['actions']['end_control'] > 0
    with (out / 'native/observed_limits.csv').open() as f:
        limits = list(csv.DictReader(f))
    with (out / 'native/feedback.csv').open() as f:
        feedback = list(csv.DictReader(f))
    assert any(row['receive_mono_ns'] == limits[0]['receive_mono_ns'] for row in feedback)
    with (out / 'native/commands.csv').open() as f:
        commands = list(csv.DictReader(f))
    assert all(int(row['phase']) in (0, 5) for row in commands)
