"""Real-path planning/analysis contracts, without ROS or a physical device."""
import csv
import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib.data import write_json
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.static_field import paired_motions
from casbot_calib.hardware_field import load_profile, seal, check_seal, prepare_field, prepare_jog, analyse_field, read_feedback, output_outside
from casbot_calib.vendor_identity import sha256

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def profile(tmp_path):
    c = yaml.safe_load((ROOT / 'configs/hardware_field.yaml').read_text())
    c['collision_scene'] = str(ROOT / 'configs/collision_scene.yaml')
    c['anchor_transition_s'] = .1; c['initial_hold_s'] = c['final_hold_s'] = .05
    path = tmp_path / 'hardware.yaml'; path.write_text(yaml.safe_dump(c))
    return path


def make_inputs(root):
    """Small explicitly synthetic reference fixture; not full preset coverage."""
    e, source = nominal_dual_arm(); c = yaml.safe_load((ROOT / 'configs/static_field_nominal.yaml').read_text())
    c['approach_offset_rad'] = .001
    c['timing'] = dict(initial_hold_s=.05, minimum_transition_s=.1, minimum_paired_approach_s=.05,
        settle_s=.1, measure_s=.2, final_hold_s=.05)
    preset = root / 'preset'; preset.mkdir(); rows = []; order = []
    for k, split in enumerate(('train', 'validation')):
        q = np.array(c['home_rad']); q[0] += (.01 if k == 0 else -.01)
        row = dict(id=f'left_fixture_{k}', arm='left', status='selected', split=split,
            q_rad=q.tolist(), family='single_axis', active_joints=[0], offsets_deg=[1 if k == 0 else -1])
        rows.append(row)
        for m in paired_motions(row, c, split=split):
            folder = preset / 'plans' / m.name; folder.mkdir(parents=True)
            write_json(folder / 'motion.json', m.metadata()); order.append(m.name)
    write_json(preset / 'config.json', c); write_json(preset / 'candidates.json', rows)
    write_json(preset / 'scene.json', yaml.safe_load((ROOT / 'configs/collision_scene.yaml').read_text()))
    write_json(preset / 'report.json', dict(passed=True, source_model_sha256=source['sha256'],
        joint_names=[j.name for j in e.model.joints], planned_execution_order=order, scope='unit-test protocol fixture'))
    seal(preset)
    snap = root / 'snapshot'; (snap / 'execution').mkdir(parents=True)
    write_json(snap / 'result.json', dict(passed=True, snapshot_only=True))
    with (snap / 'execution/snapshot.csv').open('w', newline='') as stream:
        w = csv.writer(stream); w.writerow(['receive_ns', *['q'+str(j) for j in range(14)], *['v'+str(j) for j in range(14)]])
        w.writerow([123456789, *c['home_rad'], *([0.] * 14)])
    seal(snap)
    return preset, snap


@pytest.fixture
def planned(tmp_path, profile, monkeypatch):
    import casbot_calib.hardware_field as field
    binary = tmp_path / 'binary'; binary.write_text('test-executor')
    monkeypatch.setattr(field, 'executor_binary', lambda: (binary, {'test': True}))
    preset, snapshot = make_inputs(tmp_path)
    out = tmp_path / 'plan'; report = prepare_field(profile, preset, snapshot, out)
    return out, report


@pytest.mark.parametrize('key,value', [('stop_service', '/invented_stop'), ('command_hz', 1000),
    ('maximum_velocity_rad_s', -1), ('feedback_type', 'wrong'), ('effort_gain', 0)])
def test_field_profile_rejects_ambiguous_inputs(profile, key, value):
    c = yaml.safe_load(profile.read_text()); c[key] = value; profile.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError): load_profile(profile)


def test_actual_start_full_return_roles_and_input_seals(planned):
    root, report = planned; check_seal(root)
    data = np.genfromtxt(root / 'position_plan.csv', delimiter=',', skip_header=1)
    np.testing.assert_allclose(data[0, 1:15], report['observed_start_rad'], atol=1e-12)
    np.testing.assert_allclose(data[-1, 1:15], data[0, 1:15], atol=1e-12)
    assert len(report['trials']) == 4 and report['command_period_ns'] == 2000000
    assert report['hardware_command_path_implemented'] and not report['physical_motion_qualified']
    assert not report['tracking_tube_certified']
    with (root / 'position_plan.csv').open('a') as f: f.write('changed\n')
    with pytest.raises(ValueError, match='changed'): check_seal(root)


def make_capture(plan, report, root):
    from scipy.interpolate import BPoly
    (root / 'execution').mkdir(parents=True)
    with np.load(plan / 'reference.npz') as d: curve = BPoly(d['coefficients'], d['breakpoints'])
    times = np.arange(0., curve.x[-1], .005); q = curve(times); v = curve(times, nu=1)
    e, _ = nominal_dual_arm(); torque = e.gravity_batch(q)
    origin = 9_100_000_000_000_001  # above exact float64 integer range
    with (root / 'execution/feedback.csv').open('w', newline='') as stream:
        w = csv.writer(stream); w.writerow(['receive_ns', *['q'+str(j) for j in range(14)],
            *['v'+str(j) for j in range(14)], 'publish_stamp_ns', *['effort_raw'+str(j) for j in range(14)]])
        for t, p, speed, force in zip(times, q, v, torque):
            w.writerow([origin + round(t*1e9), *p, *speed, 1000 + round(t*1e9), *force])
    with (root / 'execution/commands.csv').open('w', newline='') as stream:
        w = csv.writer(stream); w.writerow(['planned_index', 'planned_ns', 'publish_ns', 'phase'])
        w.writerow([0, origin, origin, 'stream'])
    write_json(root / 'result.json', dict(passed=True))
    write_json(root / 'request.json', dict(config=report['config'], executor_sha256=report['executor_sha256'],
        source_plan_csv_sha256=sha256(plan / 'position_plan.csv')))
    seal(root)
    return origin


def test_callback_windows_fit_and_export_do_not_claim_acquisition_or_physical_acceptance(planned, tmp_path):
    plan, metadata = planned; capture = tmp_path / 'capture'; origin = make_capture(plan, metadata, capture)
    raw = read_feedback(capture / 'execution/feedback.csv')
    assert raw['receive_ns'].dtype == np.int64 and raw['receive_ns'][0] == origin
    assert 'sample_ns' not in raw
    output = tmp_path / 'analysis'; report = analyse_field(plan, capture, output)
    assert report['passed'] and report['fit_succeeded'] and report['commissioning_validation_passed']
    assert not report['hardware_calibration_accepted'] and not report['sample_time_known']
    assert not report['torque_scale_verified'] and not report['inertias_identified']
    assert np.max(report['validation_rmse_nm']) < 1e-5
    windows = json.loads((output / 'static_windows.json').read_text())
    assert all(r['independent_acquisition_count'] is None for r in windows)
    assert (output / 'candidate_exploratory.urdf').is_file()
    assert json.loads((output / 'urdf_reload.json').read_text())['passed']


def test_different_executed_plan_is_rejected_before_fitting(planned, tmp_path):
    plan, report = planned; capture = tmp_path / 'capture'; make_capture(plan, report, capture)
    p = capture / 'request.json'; d = json.loads(p.read_text()); d['source_plan_csv_sha256'] = 'other'; write_json(p, d); seal(capture)
    with pytest.raises(ValueError, match='different position plan'): analyse_field(plan, capture, tmp_path / 'analysis')


def test_jog_uses_measured_start_one_joint_and_never_mechanical_zero_claim(planned, profile, tmp_path):
    plan, _ = planned
    root = tmp_path / 'jog'
    report = prepare_jog(profile, tmp_path / 'snapshot', root, 'left_shoulder_pitch_joint', .5)
    assert report['stage'] == 'position' and report['trials'] == []
    data = np.genfromtxt(root / 'position_plan.csv', delimiter=',', skip_header=1)
    assert np.ptp(data[:, 1]) == pytest.approx(np.deg2rad(1.), abs=1e-5)
    np.testing.assert_allclose(np.ptp(data[:, 2:15], axis=0), 0, atol=1e-12)
    capture = tmp_path / 'jog_capture'; make_capture(root, report, capture)
    result = analyse_field(root, capture, tmp_path / 'jog_analysis')
    assert result['passed'] and not result['physical_direction_verified'] and not result['mechanical_zero_calibrated']


def test_review_cannot_mutate_inputs_and_unlisted_files_are_rejected(planned):
    plan, _ = planned
    with pytest.raises(ValueError, match='outside'): output_outside(plan / 'nested', plan)
    (plan / 'unlisted').write_text('extra')
    with pytest.raises(ValueError, match='coverage'): check_seal(plan)


def test_changed_ros_domain_is_rejected_before_executor_start(planned, profile, tmp_path):
    from casbot_calib.hardware_field import execute_field
    snapshot = tmp_path / 'snapshot'; write_json(snapshot / 'request.json', dict(domain_id=21)); seal(snapshot)
    plan = tmp_path / 'bound_jog'; prepare_jog(profile, snapshot, plan, 'left_shoulder_pitch_joint')
    with pytest.raises(ValueError, match='same ROS domain'): execute_field(plan, tmp_path / 'run', 22)
    assert not (tmp_path / 'run').exists()


def test_sdk_ignoring_small_jog_cannot_pass_only_because_tracking_guard_is_larger(planned, profile, tmp_path):
    plan = tmp_path / 'ignored_jog'; metadata = prepare_jog(profile, tmp_path / 'snapshot', plan, 'left_shoulder_pitch_joint', .5)
    capture = tmp_path / 'ignored_capture'; make_capture(plan, metadata, capture)
    path = capture / 'execution/feedback.csv'
    with path.open() as stream: rows = list(csv.DictReader(stream))
    names = list(rows[0]); e, _ = nominal_dual_arm(); home = metadata['observed_start_rad']; effort = e.gravity(np.array(home))
    for row in rows:
        for j in range(14): row['q'+str(j)] = str(home[j]); row['v'+str(j)] = '0'; row['effort_raw'+str(j)] = str(effort[j])
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=names); writer.writeheader(); writer.writerows(rows)
    seal(capture)
    report = analyse_field(plan, capture, tmp_path / 'ignored_analysis')
    assert not report['passed'] and not report['motion_observed']


def test_position_diagnostics_keep_missing_effort_optional(tmp_path):
    path = tmp_path / 'feedback.csv'
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream); writer.writerow(['receive_ns', *['q'+str(j) for j in range(14)],
            *['v'+str(j) for j in range(14)], 'publish_stamp_ns', *['effort_raw'+str(j) for j in range(14)]])
        writer.writerow([1234, *([0.] * 28), 1000, *(['nan'] * 14)])
    result = read_feedback(path, require_effort=False)
    assert np.isnan(result['effort_raw']).all()
    with pytest.raises(ValueError, match='invalid'): read_feedback(path)
