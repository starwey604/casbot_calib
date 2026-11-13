"""Pose bookmark and shared CLI/GUI service, without ROS/physical hardware."""
import json
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import pytest
import yaml

from casbot_calib import fieldwork_zero as zero
from casbot_calib import hardware_field as field
from casbot_calib.fieldwork_cli import main, parser
from test_hardware_field import make_inputs


@pytest.fixture
def config():
    return field.load_profile('configs/hardware_field.yaml')


def test_bookmark_replaces_same_file_and_preserves_named_q(tmp_path, config):
    path=tmp_path/'zero.yaml';q=np.linspace(-.1,.1,14)
    zero.save_zero(config,181,'hardware_position_stream',q,receive_ns=123,path=path)
    data=zero.load_zero(config,181,'hardware_position_stream',path=path)
    assert data['units']=='rad' and data['source']['receive_ns']==123
    np.testing.assert_array_equal(list(data['position_rad'].values()),q)
    zero.save_zero(config,181,'hardware_position_stream',q+.01,receive_ns=456,path=path)
    data=zero.load_zero(config,181,'hardware_position_stream',path=path)
    np.testing.assert_array_equal(list(data['position_rad'].values()),q+.01)
    assert len(list(tmp_path.iterdir()))==1
    before=path.read_text()
    with pytest.raises(ValueError):zero.save_zero(config,181,'hardware_position_stream',[0]*13,receive_ns=0,path=path)
    assert path.read_text()==before
    with pytest.raises(ValueError,match='域号'):zero.load_zero(config,182,'hardware_position_stream',path=path)
    with pytest.raises(ValueError,match='scope'):zero.load_zero(config,181,'localhost_contract_mock',path=path)
    data['position_rad'].pop(data['joint_names'][0]);path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError,match='完整14轴'):zero.load_zero(config,181,'hardware_position_stream',path=path)


def test_capture_uses_actual_named_q_without_velocity_or_effort(tmp_path,config,monkeypatch):
    import casbot_calib.vendor_capture as capture
    names=[j.name for j in field.model_for(config)[0].model.joints];q=np.arange(14)*.01
    def fake(path,output,**kw):
        assert kw['scope']=='hardware_read_only'
        assert not kw['cancel_check']()
        packet=dict(channel='joint',receive_sequence=0,receive_ns=123,type='sensor_msgs/msg/JointState',
            message=dict(name=names[::-1],position=q[::-1].tolist(),velocity=[],effort=[]))
        kw['live_callback']('FIELDWORK_PACKET '+json.dumps(packet))
        assert kw['cancel_check']()=='zero_position_received'
    monkeypatch.setattr(capture,'capture',fake)
    path=tmp_path/'zero.yaml'
    result=zero.capture_zero(config,181,'hardware_position_stream',path=path,output=tmp_path/'capture')
    assert result['passed']
    np.testing.assert_array_equal(list(result['zero']['position_rad'].values()),q)


def test_no_feedback_and_cancel_preserve_bookmark(tmp_path,config,monkeypatch):
    import casbot_calib.vendor_capture as capture
    path=tmp_path/'zero.yaml';path.write_text('old bookmark')
    monkeypatch.setattr(capture,'capture',lambda *args,**kw:None)
    with pytest.raises(ValueError,match='未收到'):zero.capture_zero(config,181,'hardware_position_stream',path=path,output=tmp_path/'missing')
    with pytest.raises(ValueError,match='取消'):zero.capture_zero(config,181,'hardware_position_stream',path=path,output=tmp_path/'cancel',cancel_check=lambda:'operator_cancel')
    assert path.read_text()=='old bookmark'
    monkeypatch.setattr(capture,'capture',lambda *args,**kw:dict(user_interrupted=True))
    with pytest.raises(KeyboardInterrupt):zero.capture_zero(config,181,'hardware_position_stream',path=path,output=tmp_path/'interrupt')
    assert path.read_text()=='old bookmark'


def test_recording_consumer_resolves_start_replanned_zero(tmp_path,monkeypatch):
    from casbot_calib.fieldwork_control import recording_capture
    directory=tmp_path/'run';(directory/'capture').mkdir(parents=True)
    assert recording_capture(directory)==directory/'capture'
    (directory/'capture/result.json').write_text(json.dumps(dict(schema_version='casbot_field_start_replans_v1',passed=True)))
    effective=directory/'capture/start_replans/attempt_001/capture'
    def resolve(plan,capture):
        assert plan==directory/'plan' and capture==directory/'capture'
        return directory/'new_plan',effective
    monkeypatch.setattr(field,'resolve_field_capture',resolve)
    assert recording_capture(directory)==effective


def test_zero_reference_is_one_way_and_replans_to_same_frozen_target(tmp_path,config,monkeypatch):
    binary=tmp_path/'binary';binary.write_text('test executor')
    monkeypatch.setattr(field,'executor_binary',lambda:(binary,{'test':True}))
    _,snapshot=make_inputs(tmp_path)
    start=field._read_snapshot(snapshot);target=start.copy();target[0]+=.015
    plan=tmp_path/'plan';zero.prepare_zero(config,snapshot,plan,target,1.)
    data=np.loadtxt(plan/'position_plan.csv',delimiter=',',skiprows=1)
    np.testing.assert_allclose(data[0,1:15],start,atol=1e-12)
    np.testing.assert_allclose(data[-1,1:15],target,atol=1e-12)
    assert abs(data[-1,1]-data[0,1])>.01
    field.replan_field(plan,snapshot,tmp_path/'replan')
    np.testing.assert_allclose(np.loadtxt(tmp_path/'replan/position_plan.csv',delimiter=',',skiprows=1)[-1,1:15],target,atol=1e-12)
    assert not json.loads((plan/'motion.json').read_text())['initial_final_home']


def test_return_is_shared_explicit_pd_no_hidden_compensation(tmp_path,config,monkeypatch):
    path=tmp_path/'zero.yaml';zero.save_zero(config,181,'hardware_position_stream',np.zeros(14),receive_ns=123,path=path)
    seen=[]
    def snapshot(c,*args,**kwargs):seen.append(c);return dict(passed=False,reason='snapshot_failed')
    monkeypatch.setattr(field,'run_executor',snapshot)
    result=zero.return_zero(config,181,'hardware_position_stream',path=path,output=tmp_path/'return',kp=[21.]*14,kd=3.,notify=lambda _:None)
    assert not result['passed']
    assert seen[0]['gravity_feedforward']=='none'
    assert seen[0]['breakaway_parameters']==dict(command_kp=[21.]*14,command_kd=[3.]*14)


def test_cli_has_fixed_file_automatic_output_and_shared_service(config,monkeypatch):
    calls=[]
    monkeypatch.setattr(zero,'capture_zero',lambda *a,**kw:calls.append((a,kw)) or dict(passed=True,recording='auto'))
    assert main(['zero-capture','--domain-id','181','--scope','hardware_position_stream'])==0
    assert calls[0][0][1:]==(181,'hardware_position_stream') and calls[0][1]=={}
    monkeypatch.setattr(zero,'return_zero',lambda *a,**kw:calls.append((a,kw)) or dict(passed=True,recording='auto',result=dict(reason='complete')))
    assert main(['zero-return','--domain-id','181','--scope','hardware_position_stream','--kp','12','--kd','1'])==0
    assert calls[-1][1]==dict(duration_s=5.,kp=12.,kd=1.)
    with pytest.raises(SystemExit):parser().parse_args(['zero-capture'])


def test_gui_capture_and_return_use_shared_bookmark_no_parallel_sender(tmp_path,config,monkeypatch):
    from casbot_calib.fieldwork_gui import Desk
    from casbot_calib.fieldwork_status import OperatorStatus
    desk=Desk.__new__(Desk);desk.config=config;desk.status=dict(domain_id=181,scope='hardware_position_stream')
    desk.preview=desk.read_only=False;desk.observer=SimpleNamespace(done=lambda:False)
    desk.commands=dict(kp=np.full(14,20.),kd=np.full(14,2.))
    desk.q=np.zeros(14);desk.receive_ns=time.monotonic_ns();desk.root=tmp_path;desk.task=None;desk.start_requested=False
    calls=[]
    monkeypatch.setattr(zero,'save_zero',lambda *a,**kw:calls.append(('capture',a,kw)) or {})
    desk.capture_zero();assert calls[0][0]=='capture';np.testing.assert_array_equal(calls[0][1][3],desk.q)
    desk.receive_ns=time.monotonic_ns()-config['max_feedback_age_ns']-1
    desk.capture_zero();assert len(calls)==1 and '过期' in desk.message
    desk.task=object();desk.pending=dict(action='stream')
    monkeypatch.setattr(zero,'load_zero',lambda *a,**kw:pytest.fail('must not start alongside sender'))
    desk.return_zero()
    desk.control_cancel=None;desk.short=None;desk.pulse=lambda:None
    desk.operator=OperatorStatus();desk.pending=dict(action='zero_return')
    desk.stop();assert desk.control_cancel=='operator_cancel'


def test_gui_idle_return_freezes_pose_and_launches_shared_worker(tmp_path,config,monkeypatch):
    import casbot_calib.fieldwork_gui as gui
    desk=gui.Desk.__new__(gui.Desk);desk.config=config;desk.status=dict(domain_id=181,scope='hardware_position_stream')
    desk.preview=desk.read_only=False;desk.observer=SimpleNamespace(done=lambda:False);desk.q=np.zeros(14)
    desk.commands=dict(kp=np.arange(14,dtype=float),kd=np.full(14,2.));desk.task=None;desk.start_requested=False
    desk.root=tmp_path;desk.observation_relative="observations/test";desk.control_roots=[];desk.zero_duration=4.;desk.short=None
    desk.names=[j.name for j in field.model_for(config)[0].model.joints]
    saved=dict(position_rad=dict(zip(desk.names,[.1]*14)))
    monkeypatch.setattr(zero,'load_zero',lambda *a,**kw:saved)
    monkeypatch.setattr(gui,'Workspace',lambda root:SimpleNamespace(new_control=lambda:'controls/test'))
    calls=[]
    desk.launch=lambda *args:calls.append(args) or (object(),dict(action='zero_return'))
    desk.return_zero()
    action,values,root=calls[0]
    assert action=='zero_return' and root==tmp_path/'controls/test'
    assert values['duration_s']==4. and values['zero']==saved
    np.testing.assert_array_equal(desk.commands['position'],np.full(14,.1))
    assert values['kp']==list(range(14)) and Path(values['output']).parent==root/'runs'
    desk.commands['kp'][0]=999
    assert values['kp'][0]==0
