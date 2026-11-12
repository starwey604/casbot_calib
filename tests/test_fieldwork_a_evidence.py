import csv
import json
from pathlib import Path
import numpy as np
import pytest
from casbot_calib.fieldwork_a_evidence import cadence, table, find_recordings, window_a, imu_summary
from casbot_calib.fieldwork_cli import parser
from test_hardware_probe import fixture, rotor


def test_cadence_keeps_integer_ns_and_reports_backwards_clock():
    origin = 10**18
    x = cadence(np.array([origin, origin+2000000, origin+4000000], dtype=np.int64))
    assert x['hz'] == 500 and x['interval_ms']['p99'] == 2
    assert cadence([5, 5, 4])['nonincreasing_intervals'] == 2
    assert cadence([])['hz'] is None


def test_saved_exports_are_not_double_counted(tmp_path):
    for name in ('recording', 'capture/execution'):
        d = tmp_path/'controls/c1/runs/r1'/name; d.mkdir(parents=True)
        for f in ('commands.csv', 'feedback.csv'):
            (d/f).write_text('a\n1\n')
    assert find_recordings(tmp_path) == [tmp_path/'controls/c1/runs/r1/recording']


def test_partial_csv_is_not_silently_fitted(tmp_path):
    path = tmp_path/'a.csv'; path.write_text('a,b\n1,2\n3\n')
    with pytest.raises(ValueError, match='不完整'):
        table(path)


def test_original_a_and_torque_budget_on_known_rotor(monkeypatch):
    import casbot_calib.model_runtime as runtime
    monkeypatch.setattr(runtime, 'model_for', lambda config: (rotor(), None))
    obs, t, _ = fixture()
    result = window_a(obs, t, [1, 3], 4, dict(gravity_world=[0., 0., 0.], effort_gain=1., effort_zero_nm=0.), 30.)
    assert result['available'] and abs(result['baseline']['inertia_kg_m2']/5e-5-1) < .02
    assert len(result['perturbations']) == 6
    assert result['maximum_extra_5ms_inertia_change_relative'] > .1
    assert result['signal']['torque_rms_budget_for_10_percent_J_nm'] > 0
    assert not result['preprocessing']['sample_time_known']
    original = obs['effort_raw'].copy()
    # Same diagnostic uses force from public observations, not command effort.
    obs['effort_raw'] *= 2
    scaled = window_a(obs, t, [1, 3], 4, dict(gravity_world=[0., 0., 0.], effort_gain=1., effort_zero_nm=0.), 30.)
    assert scaled['baseline']['inertia_kg_m2'] == pytest.approx(2*result['baseline']['inertia_kg_m2'], rel=1e-8)
    np.testing.assert_array_equal(obs['effort_raw'], original*2)


def test_static_and_missing_force_lead_to_specific_next_measurement(monkeypatch):
    import casbot_calib.model_runtime as runtime
    monkeypatch.setattr(runtime, 'model_for', lambda config: (rotor(), None))
    obs, t, _ = fixture(); p = dict(gravity_world=[0., 0., 0.], effort_gain=1., effort_zero_nm=0.)
    obs['effort_raw'][1500, 4] = np.nan
    with pytest.raises(ValueError, match='缺失'):
        window_a(obs, t, [1, 3], 4, p, 30)
    obs['effort_raw'][:] = 0; obs['q'][:] = 0; obs['dq'][:] = 0
    with pytest.raises(ValueError, match='不可分离'):
        window_a(obs, t, [1, 3], 4, p, 30)


def test_imu_covariance_missing_sentinel_and_frame_retained(tmp_path):
    data = []
    for k in range(3):
        data.append(dict(channel='imu', receive_ns=10**15+k*10000000, clock_id='same', message=dict(
            header=dict(frame_id='torso'), angular_velocity=[1., k, 3.], angular_velocity_covariance=[-1.]+[0.]*8,
            linear_acceleration=[0., k, 9.81], linear_acceleration_covariance=[0.]*9)))
    path = tmp_path/'packets.jsonl'; path.write_text(''.join(json.dumps(r)+'\n' for r in data))
    r = imu_summary(tmp_path)[0]
    assert r['frame_ids'] == ['torso'] and r['receive']['hz'] == 100
    assert 'angular_velocity' not in r['channels']
    assert r['channels']['linear_acceleration']['valid_rows'] == 3


def test_cli_is_readonly_and_imu_is_optional():
    a = parser().parse_args(['a-evidence', '--source', 'saved', '--output', 'report', '--window', '8', '10'])
    assert a.joint == 13 and a.window == [8, 10]
    a = parser().parse_args(['observe', '--domain-id', '1', '--scope', 'hardware_read_only', '--output', 'report', '--imu-topic', '/motion/imu'])
    assert a.imu_topic == '/motion/imu'


def test_imu_projection_exposes_body_force_contamination_and_collinearity():
    from casbot_calib.fieldwork_a_evidence import imu_projection
    t = np.arange(0, 2, .002); w = 2*np.pi*5
    a = np.sin(w*t); velocity = -np.cos(w*t)/w
    body = .4*a+np.sin(2*np.pi*3.5*t)
    dq = np.zeros((len(t), 14)); ddq = dq.copy(); dq[:, 13] = velocity; ddq[:, 13] = a
    problem = dict(time_s=t, joint=13, dq=dq, ddq=ddq,
        target=.00027*a+.01*velocity+.0005*body, projected_gravity_nm=np.zeros(len(t)),
        preprocessing=dict(frequency_band_hz=[0, 15]))
    values = np.zeros((len(t), 6)); values[:, 3] = body
    result = imu_projection(problem, (t, values))
    assert result['available'] and result['maximum_inertia_change_relative'] > .1
    central = result['cases'][1]
    assert central['inertia_kg_m2'] == pytest.approx(.00027, rel=.01)
    values[:, 3] = a
    result = imu_projection(problem, (t, values))
    # At zero offset a body regressor is indistinguishable from the inertia term.
    assert not result['available'] and 'unavailable' in result['cases'][1]


def test_saved_workspace_cli_uses_sibling_imu_without_touching_inputs(tmp_path):
    import os
    import subprocess
    import sys
    import yaml
    source = tmp_path/'saved'; run = source/'controls/c1/runs/r1'
    recording = run/'recording'; recording.mkdir(parents=True)
    protocol = source/'protocol'; protocol.mkdir()
    profile = yaml.safe_load(Path('configs/hardware_field.yaml').read_text())
    profile['gravity_world'] = [0., 0., 0.]
    (protocol/'profile.yaml').write_text(yaml.safe_dump(profile))
    (protocol/'session.json').write_text(json.dumps(dict(schema_version='casbot_fieldwork_workspace_v1')))
    raw = run/'capture/raw_state/raw'; raw.mkdir(parents=True)
    clock = dict(clock_id='linux_monotonic:analytic_fixture')
    (raw/'recorder.json').write_text(json.dumps(clock))
    t = np.arange(0., 4., .001); w = 2*np.pi*5; origin = 10**15
    q = .18*t+.004*np.sin(w*t); v = .18+.004*w*np.cos(w*t); a = -.004*w*w*np.sin(w*t)
    with (recording/'feedback.csv').open('w') as stream:
        out = csv.writer(stream); out.writerow(['receive_ns', *[k+str(j) for k in ('q','v','effort_raw') for j in range(14)]])
        for i, ti in enumerate(t):
            vals = np.zeros((3, 14)); vals[:, 13] = [q[i], v[i], .00027*a[i]+.01*v[i]+.005]
            out.writerow([origin+round(ti*1e9), *vals.ravel()])
    with (recording/'commands.csv').open('w') as stream:
        out = csv.writer(stream); out.writerow(['planned_ns', 'publish_ns'])
        for i in range(0, len(t), 2):out.writerow([origin+round(t[i]*1e9)]*2)
    iraw = source/'observations/o1/capture/raw'; iraw.mkdir(parents=True)
    (iraw/'recorder.json').write_text(json.dumps(clock))
    with (iraw/'packets.jsonl').open('w') as stream:
        for ti in t[::5]:
            stream.write(json.dumps(dict(channel='imu', receive_ns=origin+round(ti*1e9), message=dict(
                header=dict(frame_id='torso'), angular_velocity=[0.,0.,0.], angular_velocity_covariance=[0.]*9,
                linear_acceleration=[0.,0.,9.81], linear_acceleration_covariance=[0.]*9)))+'\n')
    before = {p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()}
    output = tmp_path/'report'
    process = subprocess.run([sys.executable, '-m', 'casbot_calib.fieldwork_cli', 'a-evidence',
        '--source', str(run), '--output', str(output), '--window', '1', '3', '--joint', '13'], capture_output=True, text=True)
    assert process.returncode == 0, process.stdout+process.stderr
    result = json.loads((output/'report.json').read_text()); row = result['records'][0]
    assert row['sender']['hz'] == 500 and row['receiver']['hz'] == 1000
    assert row['A']['baseline']['inertia_kg_m2'] == pytest.approx(.00027, rel=.01)
    assert row['A']['body_motion']['available']
    assert row['A']['body_motion']['maximum_inertia_change_relative'] < 1e-8
    assert row['imu_path'] == str(iraw/'packets.jsonl')
    assert result['automatic_calibration_acceptance'] is False
    after = {p.relative_to(source):p.read_bytes() for p in source.rglob('*') if p.is_file()}
    assert before == after
