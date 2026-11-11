"""Manual desk value provenance, latest-value mailbox and lifecycle contracts."""
from pathlib import Path
import time

import numpy as np
import pytest
import yaml

from casbot_calib.fieldwork_control import (Session, create_session, heartbeat_check,
    initial_commands, validate_commands, write_command, tuning_config, DEFAULT_TUNING)
from casbot_calib.hardware_batch import atomic_json, event

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def control(tmp_path):
    from casbot_calib.sdk_connection import mock_connection
    config = yaml.safe_load((ROOT/'configs/hardware_field.yaml').read_text())
    config.update(collision_scene=str(ROOT/'configs/collision_scene.yaml'),
                  connection=mock_connection('/casbot_calib_s2_manual_fixture',181,'a'*64))
    path=tmp_path/'profile.yaml'; path.write_text(yaml.safe_dump(config))
    root=tmp_path/'session'; create_session(path,root,181,'localhost_contract_mock')
    return root


def test_session_no_review_or_site_forms(control):
    session=Session(control); status=session.status()
    assert status['blocked_reason'] is None
    assert status['hidden_compensation'] is False
    assert status['mechanical_zero_known'] is False
    assert not hasattr(session,'review') and not hasattr(session,'require_all_directions')
    assert not (control/'reviews').exists()


def test_first_feedback_initializes_without_mutating_input():
    q=np.linspace(-.4,.4,14); kp=np.arange(14)+20; kd=np.arange(14)+1
    values,sources=initial_commands(q,kp,kd,DEFAULT_TUNING)
    np.testing.assert_array_equal(values['position'],q)
    np.testing.assert_array_equal(values['kp'],kp)
    np.testing.assert_array_equal(values['kd'],kd)
    assert np.all(values['velocity']==0) and np.all(values['effort']==0)
    values['position'][0]=1
    assert q[0]==-.4 and '反馈' in sources['kp']


@pytest.mark.parametrize('value',[None,[],[float('nan')]*14,[-1]*14])
def test_missing_pd_is_explicit_default(value):
    values,sources=initial_commands(np.zeros(14),value,value,DEFAULT_TUNING)
    assert np.all(values['kp']==20) and np.all(values['kd']==2)
    assert sources['kp'].startswith('配置默认')


def test_zero_pd_feedback_is_not_replaced():
    values,_=initial_commands(np.zeros(14),[0]*14,[0]*14,DEFAULT_TUNING)
    assert np.all(values['kp']==0) and np.all(values['kd']==0)


def test_mailbox_replaces_complete_frames(tmp_path):
    path=tmp_path/'latest.csv'
    values,_=initial_commands(np.zeros(14),None,None,DEFAULT_TUNING)
    write_command(path,1,values); values['effort'][3]=.2; write_command(path,2,values)
    row=np.loadtxt(path,delimiter=',')
    assert row.shape==(71,) and row[0]==2 and row[32]==.2
    assert not path.with_suffix('.pending').exists()


@pytest.mark.parametrize('field,value',[('position',2),('velocity',1),('effort',11),('kp',1001),('kd',101),('effort',float('nan'))])
def test_command_ranges(field,value):
    values,_=initial_commands(np.zeros(14),None,None,DEFAULT_TUNING)
    values[field][0]=value
    with pytest.raises(ValueError): validate_commands(values,-np.ones(14),np.ones(14),DEFAULT_TUNING,.75)


def test_zero_pd_and_signed_effort_are_valid():
    values,_=initial_commands(np.zeros(14),[0]*14,[0]*14,DEFAULT_TUNING)
    values['effort'][0]=-2
    validate_commands(values,-np.ones(14),np.ones(14),DEFAULT_TUNING,.75)


@pytest.mark.parametrize('change',[dict(kp=-1),dict(effort_limit_nm=101),dict(kd_max=101),dict(extra=1)])
def test_tuning_config_rejects_invalid_values(change):
    with pytest.raises(ValueError): tuning_config(dict(manual_tuning=change))


def test_unsealed_and_faulted_sessions_cannot_resume(control):
    event(control,'stream_started',run='runs/unit')
    assert Session(control).status()['blocked_reason']
    event(control,'stream_finished',run='runs/unit',controlled_stop=True)
    assert Session(control).status()['blocked_reason'] is None
    event(control,'terminal_fault',run='runs/unit',reason='stale_feedback')
    assert Session(control).status()['blocked_reason']


def test_heartbeat_owner_and_lifecycle(tmp_path):
    from casbot_calib.fieldwork import clock_id
    path=tmp_path/'heartbeat.json'
    value=dict(clock_id=clock_id(),owner_id='a',monotonic_ns=time.monotonic_ns(),cancel=None)
    atomic_json(path,value)
    assert heartbeat_check(path,expected_owner='a')() is None
    assert heartbeat_check(path,expected_owner='b')()=='frontend_owner_changed'
    value['monotonic_ns']-=3_000_000_000; atomic_json(path,value,replace=True)
    assert heartbeat_check(path)()=='frontend_heartbeat_timeout'
    value['cancel']='operator_cancel'; atomic_json(path,value,replace=True)
    assert heartbeat_check(path)()=='operator_cancel'


def test_no_preloaded_commands(control):
    directory=control/'runs/unit'; directory.mkdir(parents=True)
    (directory/'latest.csv').write_text('old command')
    with pytest.raises(ValueError,match='邮箱必须为空'):
        Session(control).stream('runs/unit',directory/'latest.csv')


def test_short_trajectory_has_exact_endpoints():
    from casbot_calib.fieldwork_control import short_position
    assert short_position(.2,.01,6.,0.) == pytest.approx(.2)
    assert short_position(.2,.01,6.,2.) == pytest.approx(.21)
    assert short_position(.2,.01,6.,4.) == pytest.approx(.19)
    assert short_position(.2,.01,6.,6.) == pytest.approx(.2)
    assert short_position(.2,.01,6.,10.) == pytest.approx(.2)


def test_video_cancel_closes_encoder_without_publishing_final_file(tmp_path, monkeypatch):
    from contextlib import contextmanager
    from types import SimpleNamespace
    imageio_ffmpeg=pytest.importorskip('imageio_ffmpeg')
    from casbot_calib import fieldwork_worker as worker, presentation
    directory=tmp_path/'runs/unit/recording'; directory.mkdir(parents=True)
    (directory/'feedback.csv').write_text('receive_ns,q0\n100,0\n1000000100,0.01\n')
    monkeypatch.setattr(worker,'Session',lambda root:SimpleNamespace(metadata={}))
    monkeypatch.setattr(presentation,'_font',lambda size:None)
    @contextmanager
    def view(*args): yield object()
    monkeypatch.setattr(worker,'open_view',view)
    closed=[]
    def encoder(*args,**kwargs):
        try:
            while True: yield
        finally: closed.append(True)
    monkeypatch.setattr(imageio_ffmpeg,'write_frames',encoder)
    with pytest.raises(ValueError,match='frontend_closed'):
        worker.render_recording(tmp_path,'runs/unit',cancel_check=lambda:'frontend_closed')
    assert closed==[True] and not (directory/'motion.mp4').exists()
