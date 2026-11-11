import copy
import json
import os
from pathlib import Path

import pytest

from casbot_calib.fieldwork import JsonTail, LiveObservation, collect
from casbot_calib.fieldwork_analysis import (ConnectionMonitor, analyse_capture, position_errors,
    receive_rates, save_report, write_json)
from casbot_calib.vendor_data import load_config, raw_fingerprints


@pytest.fixture
def config():
    return load_config(Path(__file__).resolve().parents[1]/'configs/vendor_s1.yaml')


def packet(config, ns, *, velocity=None):
    names = ['waist_yaw_joint'] + list(reversed(config['joint_names'])) + ['left_thumb_proximal_joint']
    return dict(channel='joint', receive_sequence=ns, receive_ns=ns, type=config['joint_type'],
        message=dict(name=names, position=[.3]+[.01*i for i in range(14)]+[.2],
                     velocity=[] if velocity is None else velocity, effort=[], header=dict(sec=0, nanosec=0)))


def session_fixture(root, config, *, command='observe', times=(1000000000, 1100000000, 1200000000)):
    root.mkdir()
    (root/'capture/raw').mkdir(parents=True)
    (root/'events').mkdir()
    session = dict(schema_version='casbot_fieldwork_session_v1', run_id='fixture', created_utc='2026-11-11T00:00:00Z',
        command=command, config=config, timeout_ns=250000000, domain_id=180, scope='localhost_contract_mock', target_label='test fixture')
    write_json(root/'session.json', session)
    write_json(root/'environment.json', dict(ros_distro='humble', recorder_available=True))
    write_json(root/'capture/session.json', dict(config=config, domain_id=180, connection_scope='localhost_contract_mock'))
    write_json(root/'capture/raw/recorder.json', dict(clock_id='fixture_clock'))
    write_json(root/'capture/raw/summary.json', dict(start_ns=900000000, end_ns=times[-1]+10000000, reason='duration_complete'))
    (root/'capture/raw/cdr.bin').write_bytes(b'unit_test_placeholder_not_ros')
    (root/'capture/raw/publishers.jsonl').write_text('')
    (root/'capture/raw/packets.jsonl').write_text(''.join(json.dumps(packet(config, t))+'\n' for t in times))
    endpoint = dict(gid='first', type=config['joint_type'], qos_reliability=1, node='state_fixture')
    discovery = dict(observed_ns=1000000000, services={}, topics={}, monitored_topics={config['joint_topic']: [endpoint]})
    (root/'capture/raw/discovery.jsonl').write_text(json.dumps(discovery)+'\n')
    seal_fixture(root)
    return session


def seal_fixture(root):
    write_json(root/'capture/capture.json', dict(completed=True, returncode=0, source_sha256=raw_fingerprints(root/'capture')))


def check(report, name):
    return next(c for c in report['checks'] if c['item_id'] == name)


def test_full_axes_and_unknown_fields_preserved(tmp_path, config):
    root = tmp_path/'capture'
    session = session_fixture(root, config)
    report = analyse_capture(root, session)
    assert report['status'] == 'passed'
    assert len(report['channels']['joint']['axes']) == 16
    assert report['channels']['joint']['optional_unavailable'] == dict(velocity=3, effort=3)
    assert all(axis['span_rad'] == 0 for axis in report['channels']['joint']['axes'])
    assert report['channels']['joint']['receive']['count'] == 3  # identical values are not deduplicated
    assert report['unavailable']['actual_sample_hz'] is None
    assert not report['motion_allowed'] and not report['physical_stop_verified']


@pytest.mark.parametrize('change,error', [('duplicate', 'invalid_or_duplicate_names'), ('missing', 'missing_arm_axes'),
                                        ('empty', 'unavailable_position'), ('nan', 'unavailable_position')])
def test_bad_position_is_not_connection_success(config, change, error):
    p = packet(config, 1)
    if change == 'duplicate': p['message']['name'][0] = p['message']['name'][1]
    if change == 'missing': p['message']['name'][1] = 'unexpected'
    if change == 'empty': p['message']['position'] = []
    if change == 'nan': p['message']['position'][0] = None
    original = copy.deepcopy(p)
    assert error in position_errors(p, config['joint_names'])
    assert original == p


def test_polling_detection_does_not_invent_sampling_or_recovery(config):
    events = []
    monitor = ConnectionMonitor(config['joint_names'], 250, events.append)
    monitor.packet(packet(config, 1000), 1010)
    monitor.tick(1300)
    monitor.tick(1350)
    bad = packet(config, 1400); bad['message']['position'] = []
    monitor.packet(bad, 1410)
    monitor.packet(packet(config, 1500), 1520)
    assert [e['event'] for e in events] == ['first_valid_feedback', 'valid_feedback_timeout', 'valid_feedback_resumed']
    assert events[1]['observed_ns'] == 1300 and events[2]['gap_ns'] == 500


def test_initial_timeout_is_detected_once_before_first_frame(config):
    events = []
    monitor = ConnectionMonitor(config['joint_names'], 250, events.append)
    monitor.tick(0); monitor.tick(300); monitor.tick(600)
    assert len(events) == 1 and events[0]['event'] == 'initial_feedback_timeout'


def test_multiline_interactive_input_does_not_leave_q_in_text_buffer(tmp_path, config):
    from casbot_calib.fieldwork_analysis import json_lines
    (tmp_path/'events').mkdir()
    read_fd, write_fd = os.pipe()
    try:
        with os.fdopen(read_fd, 'r') as stream:
            live = LiveObservation(tmp_path, dict(config=config, timeout_ns=100), lambda event: None,
                                   interactive=True, input_stream=stream)
            os.write(write_fd, b'n first\nq\n')
            with pytest.raises(KeyboardInterrupt): live.poll()
        assert [e['event'] for e in json_lines(tmp_path/'events/operator.jsonl')] == ['operator_note', 'operator_finish']
    finally:
        os.close(write_fd)


def test_callback_failure_terminates_owned_recorder(tmp_path, config, monkeypatch):
    import casbot_calib.vendor_capture as vendor
    prefix = tmp_path/'install'
    (prefix/'crb_ros_msg/lib').mkdir(parents=True)
    (prefix/'crb_ros_msg/lib/fixture.so').touch()
    binary = prefix/'casbot_calib_vendor/lib/casbot_calib_vendor/state_recorder'
    binary.parent.mkdir(parents=True); binary.touch()
    write_json(prefix/'vendor-source.json', dict(interfaces={}))
    source = tmp_path/'config.yaml'; source.write_text('fixture')
    monkeypatch.setenv('CASBOT_VENDOR_INSTALL', str(prefix))
    monkeypatch.setattr(vendor, 'load_config', lambda path: config)
    class Process:
        code = None
        terminated = False
        def poll(self): return self.code
        def terminate(self): self.terminated = True; self.code = -15
        def wait(self, timeout=None): return self.code
    process = Process()
    monkeypatch.setattr(vendor.subprocess, 'Popen', lambda *args, **kwargs: process)
    def broken(): raise ValueError('frontend broke')
    with pytest.raises(ValueError, match='frontend broke'):
        vendor.capture(source, tmp_path/'capture', duration_s=1, domain_id=1,
                       scope='hardware_read_only', poll_callback=broken)
    assert process.terminated


def test_reconnect_requires_real_gap_and_restored_valid_frames(tmp_path, config):
    root = tmp_path/'capture'
    session = session_fixture(root, config, command='reconnect', times=(1000000000, 1100000000, 1800000000, 1900000000))
    report = analyse_capture(root, session)
    assert report['status'] == 'passed' and report['gaps'][0]['gap_ns'] == 700000000
    session['timeout_ns'] = 900000000
    assert check(analyse_capture(root, session), 'recovery')['status'] == 'blocked'


def test_no_recovery_when_capture_ends_stale(tmp_path, config):
    root = tmp_path/'capture'
    session = session_fixture(root, config, command='reconnect')
    write_json(root/'capture/raw/summary.json', dict(end_ns=2000000000, reason='duration_complete'))
    seal_fixture(root)
    report = analyse_capture(root, session)
    assert report['stale_at_end'] and check(report, 'recovery')['status'] == 'blocked'


@pytest.mark.parametrize('kind,diagnosis', [('none', 'publisher_not_discovered'), ('type', 'message_type_mismatch'),
    ('qos', 'qos_reliability_incompatible'), ('silent', 'publisher_discovered_no_data')])
def test_discovery_distinguishes_supported_causes(tmp_path, config, kind, diagnosis):
    root = tmp_path/'capture'
    session = session_fixture(root, config)
    (root/'capture/raw/packets.jsonl').write_text('')
    path = root/'capture/raw/discovery.jsonl'
    row = json.loads(path.read_text())
    endpoints = row['monitored_topics'][config['joint_topic']]
    if kind == 'none': endpoints.clear()
    if kind == 'type': endpoints[0]['type'] = 'std_msgs/msg/String'
    if kind == 'qos': endpoints[0]['qos_reliability'] = 2
    path.write_text(json.dumps(row)+'\n')
    seal_fixture(root)
    report = analyse_capture(root, session)
    assert report['connection_diagnosis'] == diagnosis
    assert not report['automated_checks_passed']


def test_multiple_publishers_require_target_confirmation(tmp_path, config):
    root = tmp_path/'capture'
    session = session_fixture(root, config)
    path = root/'capture/raw/discovery.jsonl'
    row = json.loads(path.read_text())
    row['monitored_topics'][config['joint_topic']].append(dict(type=config['joint_type'], qos_reliability=1, gid='second'))
    path.write_text(json.dumps(row)+'\n'); seal_fixture(root)
    assert check(analyse_capture(root, session), 'discovery')['status'] == 'blocked'


def test_reanalysis_rejects_source_mutation_and_domain_mismatch(tmp_path, config):
    root = tmp_path/'capture'
    session = session_fixture(root, config)
    session['domain_id'] = 181
    assert check(analyse_capture(root, session), 'evidence')['status'] == 'failed'
    session['domain_id'] = 180
    with (root/'capture/raw/packets.jsonl').open('a') as stream: stream.write('{"tail":')
    report = analyse_capture(root, session)
    assert report['evidence_errors'] and report['status'] == 'failed'


def test_json_tail_retains_partial_line(tmp_path):
    path = tmp_path/'tail.jsonl'
    path.write_text('{"a":1}\n{"b":')
    tail = JsonTail(path)
    assert tail.read() == [dict(a=1)]
    assert tail.read() == []
    with path.open('a') as stream: stream.write('2}\n')
    assert tail.read() == [dict(b=2)]


def test_report_is_offline_and_does_not_overwrite_source(tmp_path, config):
    root = tmp_path/'capture'
    session_fixture(root, config)
    before = (root/'capture/raw/packets.jsonl').read_bytes()
    save_report(root)
    out = tmp_path/'analysis'
    report = save_report(root, out)
    assert report['status'] == 'passed' and 'waist_yaw_joint' in (out/'report.md').read_text()
    assert before == (root/'capture/raw/packets.jsonl').read_bytes()
    with pytest.raises(FileExistsError): save_report(root)
    with pytest.raises(ValueError): save_report(root, root/'analysis')


@pytest.mark.parametrize('domain,duration,scope', [(None, 2., 'hardware_read_only'), (233, 2., 'hardware_read_only'),
    (1, float('nan'), 'hardware_read_only'), (1, 1., 'localhost_contract_mock')])
def test_invalid_or_unsafe_request_fails_before_creating_session(tmp_path, domain, duration, scope):
    with pytest.raises(ValueError):
        collect('configs/vendor_s1.yaml', tmp_path/'out', command='observe', domain_id=domain,
                scope=scope, duration_s=duration)
    assert not (tmp_path/'out').exists()


def test_rates_use_local_times_and_reject_backward_clock():
    assert receive_rates([1000000000, 1002000000, 1004000000])['hz'] == 500
    assert receive_rates([2, 1])['hz'] is None
    assert receive_rates([1])['hz'] is None


def test_cli_requires_explicit_domain_scope_and_has_no_motion_commands():
    from casbot_calib.fieldwork_cli import parser
    for args in (['observe', '--output', 'unused'], ['execute'], ['observe', '--domain-id', '0', '--output', 'unused']):
        with pytest.raises(SystemExit): parser().parse_args(args)


def test_collect_error_still_delivers_report(tmp_path, config, monkeypatch):
    import casbot_calib.vendor_capture as vendor
    monkeypatch.setattr(vendor, 'capture', lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError('fixture launch failure')))
    root = tmp_path/'out'
    report = collect('configs/vendor_s1.yaml', root, command='preflight', domain_id=1,
                     scope='hardware_read_only', duration_s=1)
    assert report['status'] == 'failed'
    assert 'fixture launch failure' in (root/'capture_error.json').read_text()


def test_rich_output_does_not_interpret_robot_names_as_markup(tmp_path, config):
    from io import StringIO
    from rich.console import Console
    from casbot_calib.fieldwork_cli import Terminal
    root = tmp_path/'capture'
    session = session_fixture(root, config)
    terminal = Terminal()
    stream = StringIO()
    terminal.console = Console(file=stream, width=120, color_system=None)
    terminal.report(analyse_capture(root, session), root)
    assert '现场只读检查' in stream.getvalue() and 'waist_yaw_joint' in stream.getvalue()
