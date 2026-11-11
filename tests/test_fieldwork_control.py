"""Software contracts only; synthetic sealed fixtures never imply physical reviews."""
import csv
import json
from pathlib import Path
import time

import numpy as np
import pytest
import yaml

from casbot_calib import hardware_field as field
from casbot_calib.fieldwork_control import Session, create_session, heartbeat_check, operate
from casbot_calib.hardware_batch import atomic_json, event
from casbot_calib.vendor_identity import sha256

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def executor(tmp_path, monkeypatch):
    binary = tmp_path/'executor'; binary.write_text('unit test only')
    monkeypatch.setattr(field, 'executor_binary', lambda: (binary, dict(unit_fixture=True)))


@pytest.fixture
def control(tmp_path, executor):
    from casbot_calib.sdk_connection import mock_connection
    config = yaml.safe_load((ROOT/'configs/hardware_field.yaml').read_text())
    config.update(collision_scene=str(ROOT/'configs/collision_scene.yaml'), anchor_transition_s=.5,
                  initial_hold_s=.1, final_hold_s=.1,
                  connection=mock_connection('/casbot_calib_s2_control_fixture', 181, 'a'*64))
    path = tmp_path/'profile.yaml'; path.write_text(yaml.safe_dump(config))
    root = tmp_path/'session'
    create_session(path, root, 181, 'localhost_contract_mock')
    return root


def snapshot(root):
    directory = root/'snapshots/fixture'; (directory/'execution').mkdir(parents=True)
    config = Session(root).config
    q = yaml.safe_load((ROOT/'configs/sdk_mock.yaml').read_text())['initial_q']
    atomic_json(directory/'result.json', dict(passed=True, snapshot_only=True))
    atomic_json(directory/'request.json', dict(config=config, domain_id=181))
    with (directory/'execution/snapshot.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['receive_ns', *['q'+str(i) for i in range(14)], *['v'+str(i) for i in range(14)]])
        writer.writerow([100, *q, *([0]*14)])
    field.seal(directory); event(root, 'snapshot_finished', snapshot='snapshots/fixture')
    return 'snapshots/fixture'


def fake_attempt(root, name, joint):
    """Explicit unit-test evidence; not SDK acquisition or human observation."""
    directory = root/'attempts'/name
    for part in ('capture', 'analysis', 'plan'):
        (directory/part).mkdir(parents=True)
    atomic_json(directory/'capture/result.json', dict(passed=True, source='unit_fixture'))
    atomic_json(directory/'analysis/report.json', dict(passed=True, source='unit_fixture'))
    atomic_json(directory/'plan/plan.json', dict(passed=True, source='unit_fixture'))
    for part in ('capture', 'analysis', 'plan'): field.seal(directory/part)
    atomic_json(directory/'input/request.json', dict(kind='jog', joints=[joint], snapshot='snapshots/fixture',
        protocol_sha256=sha256(root/'protocol/manifest.json'), plan_manifest_sha256=sha256(directory/'plan/manifest.json')))
    field.seal(directory/'input')
    atomic_json(directory/'outcome.json', dict(passed=True))
    return 'attempts/'+name


def confirm(root, name, joint):
    attempt = fake_attempt(root, name, joint)
    return Session(root).review(attempt, 'unit fixture', 'synthetic '+joint, True, True, 'not physical evidence')


def test_short_is_blocked_in_backend_before_any_io(control):
    with pytest.raises(ValueError, match='14'):
        Session(control).plan('short', 'not_read', [Session(control).names[0]], .5)


def test_all_14_unique_bound_reviews_are_required(control):
    names = Session(control).names
    for i, name in enumerate(names[:13]): confirm(control, str(i), name)
    with pytest.raises(ValueError, match='14'): Session(control).require_all_directions()
    confirm(control, 'last', names[13])
    assert len(Session(control).require_all_directions()) == 14
    path = control/'attempts/last/capture/result.json'
    path.write_text('{}')
    with pytest.raises(ValueError, match='changed'): Session(control).require_all_directions()


def test_negative_physical_review_terminates_motion(control):
    joint = Session(control).names[0]
    attempt = fake_attempt(control, 'negative', joint)
    Session(control).review(attempt, 'observer', 'wrong joint observed', False, False, 'unit counterexample')
    assert Session(control).status()['blocked_reason']
    with pytest.raises(ValueError, match='终止'): Session(control).assert_motion()


def test_hardware_conditions_are_not_inferred_from_profile(tmp_path, executor):
    root = tmp_path/'hardware'
    create_session(ROOT/'configs/hardware_field.yaml', root, 42, 'hardware_position_stream')
    assert not Session(root).status()['site_conditions_ready']
    with pytest.raises(ValueError, match='证据'): Session(root).assert_motion()


def test_unknown_site_conditions_do_not_enable_motion(tmp_path):
    with pytest.raises(ValueError, match='结论和证据'):
        create_session(ROOT/'configs/hardware_field.yaml', tmp_path/'hardware', 42, 'hardware_position_stream',
                       ROOT/'fieldwork/profiles/site_conditions.example.yaml')


@pytest.mark.parametrize('behavior', ['unknown', 'release', 'default_zero'])
def test_unsupported_non_target_behavior_cannot_be_described_as_hold(tmp_path, behavior):
    from casbot_calib.fieldwork_control import SITE_ITEMS
    proof = tmp_path/'site.txt'; proof.write_text('unit-test declaration, not physical proof')
    site = {key: dict(status='site_confirmed', conclusion='unit declaration', evidence=['site.txt']) for key in SITE_ITEMS}
    site['mode_and_non_target_hold']['non_target_behavior'] = behavior
    path = tmp_path/'site.yaml'; path.write_text(yaml.safe_dump(site))
    with pytest.raises(ValueError, match='非目标轴保持'):
        create_session(ROOT/'configs/hardware_field.yaml', tmp_path/'session', 42, 'hardware_position_stream', path)


def test_confirmed_site_evidence_is_copied_but_not_physical_qualification(tmp_path, executor):
    from casbot_calib.fieldwork_control import SITE_ITEMS
    proof = tmp_path/'site.txt'; proof.write_text('unit fixture only')
    site = {key: dict(status='site_confirmed', conclusion='unit declaration', evidence=['site.txt']) for key in SITE_ITEMS}
    site['mode_and_non_target_hold']['non_target_behavior'] = 'hold_prior_target_output'
    path = tmp_path/'site.yaml'; path.write_text(yaml.safe_dump(site))
    root = tmp_path/'session'
    status = create_session(ROOT/'configs/hardware_field.yaml', root, 42, 'hardware_position_stream', path)
    assert status['site_conditions_ready'] and not status['physical_motion_qualified']
    assert (root/'protocol/site_evidence/emergency_stop_and_support/0_site.txt').read_text() == 'unit fixture only'


def test_controlled_snapshot_interrupt_clears_old_snapshot_without_sdk_recovery(control, monkeypatch):
    snapshot(control)
    monkeypatch.setattr(field, 'run_executor', lambda *args, **kwargs:
                        dict(passed=False, fault='interrupted', supervisor_interruption='frontend_closed'))
    with pytest.raises(ValueError, match='受控中断'): Session(control).snapshot()
    status = Session(control).status()
    assert status['snapshot'] is None and status['blocked_reason'] is None


def test_completed_plan_is_never_reexecuted(control):
    relative = fake_attempt(control, 'completed', Session(control).names[0])
    with pytest.raises(ValueError, match='一次'): Session(control).execute(relative, cancel_check=lambda: None)


def test_report_keeps_rejected_plans_and_remains_available_after_fault(control):
    with pytest.raises(ValueError, match='14'):
        operate(control, 'plan', dict(kind='short', snapshot='missing', joints=[Session(control).names[0]], amplitude_deg=.5))
    event(control, 'terminal_fault', reason='unit fixture')
    result = operate(control, 'report', {})
    report = json.loads((control/result['report_dir']/'report.json').read_text())
    assert report['attempts'][0]['state'] == '参考拒绝/未下发'
    assert report['session']['blocked_reason'] and not report['physical_stop_verified']


def test_closed_return_path_and_nonselected_axes_unchanged(control):
    names = Session(control).names
    for i, name in enumerate(names): confirm(control, str(i), name)
    snap = snapshot(control)
    result = Session(control).plan('short', snap, [names[0], names[7]], .25, transition_s=1)
    path = control/result['attempt']/'plan'
    data = np.genfromtxt(path/'position_plan.csv', delimiter=',', names=True)
    q = np.column_stack([data['q'+str(i)] for i in range(14)])
    np.testing.assert_allclose(q[0], q[-1], atol=1e-12)
    for index in set(range(14))-{0, 7}: np.testing.assert_allclose(q[:, index], q[0, index], atol=1e-12)
    assert field.check_seal(path) is None


def test_frontend_supervision_is_required_even_for_direct_backend(control):
    joint = Session(control).names[0]
    relative = fake_attempt(control, 'no_guard', joint)
    with pytest.raises(ValueError, match='存活监督'): Session(control).execute(relative)


@pytest.mark.parametrize('reason,fault,terminal', [('operator_cancel', 'interrupted', False),
    ('frontend_heartbeat_timeout', 'interrupted', True), ('operator_cancel', 'stale_feedback', True)])
def test_only_controlled_interrupt_can_start_new_attempt(control, monkeypatch, reason, fault, terminal):
    joint = Session(control).names[0]
    relative = fake_attempt(control, 'interruption', joint)
    # Fresh, unexecuted input: fake_attempt normally represents a completed trial.
    import shutil
    shutil.rmtree(control/relative/'capture'); (control/relative/'outcome.json').unlink()
    monkeypatch.setattr(field, 'execute_field', lambda *args, **kwargs:
                        dict(passed=False, fault=fault, supervisor_interruption=reason))
    result = Session(control).execute(relative, cancel_check=lambda: reason)
    assert bool(Session(control).blocked_reason()) == terminal
    assert result['controlled_interruption'] == (not terminal)


def test_unsealed_execution_never_silently_resumes(control):
    event(control, 'execution_started', attempt='attempts/killed', snapshot='snapshots/fixture')
    assert Session(control).blocked_reason()
    with pytest.raises(ValueError, match='未封存'): Session(control).assert_motion()


def test_reused_session_object_cannot_ignore_a_later_terminal_event(control):
    session = Session(control)
    event(control, 'terminal_fault', reason='unit late event')
    with pytest.raises(ValueError, match='终止'): session.assert_motion()


def test_implementation_change_requires_a_new_motion_session(control, monkeypatch):
    import casbot_calib.fieldwork_control as module
    monkeypatch.setattr(module, 'implementation', lambda: dict(changed=True))
    with pytest.raises(ValueError, match='实现/SDK'): Session(control).assert_motion()


def test_heartbeat_missing_clock_mismatch_and_stale_are_terminal_reasons(tmp_path):
    from casbot_calib.fieldwork import clock_id
    path = tmp_path/'heartbeat.json'
    check = heartbeat_check(path)
    assert check() == 'frontend_heartbeat_unavailable'
    path.write_text(json.dumps(dict(clock_id='wrong', monotonic_ns=time.monotonic_ns())))
    assert check() == 'frontend_clock_mismatch'
    path.write_text(json.dumps(dict(clock_id=clock_id(), monotonic_ns=time.monotonic_ns()-3000000000)))
    assert check() == 'frontend_heartbeat_timeout'
    path.write_text(json.dumps(dict(clock_id=clock_id(), monotonic_ns=time.monotonic_ns())))
    assert check() is None


def test_gui_stop_reason_persists_until_backend_consumes_it(control, monkeypatch):
    # No GUI/window/ROS required to test the frontend's cancellation record.
    from casbot_calib.fieldwork_gui import Desk
    from casbot_calib.fieldwork import clock_id
    desk = Desk.__new__(Desk)
    desk.heartbeat = control/'heartbeat.json'; desk.local_clock = clock_id(); desk.owner_id = 'unit_owner'
    desk.pulse('operator_cancel')
    assert heartbeat_check(desk.heartbeat)() == 'operator_cancel'


def test_new_window_cannot_keep_previous_owner_alive(control):
    from casbot_calib.fieldwork_gui import claim_frontend
    from casbot_calib.fieldwork import clock_id
    lease = claim_frontend(control)
    try:
        with pytest.raises(ValueError, match='第二个'): claim_frontend(control)
    finally:
        lease.close()
    reopened = claim_frontend(control); reopened.close()
    path = control/'heartbeat.json'
    path.write_text(json.dumps(dict(clock_id=clock_id(), owner_id='new_window', monotonic_ns=time.monotonic_ns())))
    assert heartbeat_check(path, expected_owner='old_window')() == 'frontend_owner_changed'


def test_scope_must_match_mock_connection(control, tmp_path):
    with pytest.raises(ValueError, match='scope'):
        create_session(control/'protocol/profile.yaml', tmp_path/'wrong', 181, 'hardware_position_stream')
