"""SDK transport parity and declared Mock policies, without ROS/device I/O."""
from pathlib import Path
import pytest
import yaml
from casbot_calib.sdk_connection import mock_connection, connection_remaps, SDK_ENDPOINTS
from casbot_calib.sdk_mock import mock_config
from casbot_calib import hardware_field as field
from casbot_calib import hardware_batch as batch

ROOT=Path(__file__).resolve().parents[1]


def test_connection_maps_only_sdk_endpoints_without_stop_or_truth():
    value=mock_connection('/casbot_calib_s2_unified',222,'a'*64)
    result=connection_remaps({'connection':value},222)
    assert set(result)==set(SDK_ENDPOINTS)
    assert all(target=='/casbot_calib_s2_unified'+name for name,target in result.items())
    assert not any('stop' in name or 'plant' in name for name in result)
    assert connection_remaps({},222) is None


@pytest.mark.parametrize('root',['/joint_states','/casbot_calib_s2_test/../plant','/casbot_calib_s2_test/','/casbot_calib_s2_test/foo'])
def test_mock_namespace_cannot_escape_private_scope(root):
    with pytest.raises(ValueError):mock_connection(root,222,'a'*64)


def test_wrong_domain_and_conflicting_remaps_fail_before_io(tmp_path):
    profile=field.load_profile(ROOT/'configs/hardware_field.yaml')
    profile['connection']=mock_connection('/casbot_calib_s2_unified',222,'a'*64)
    with pytest.raises(ValueError,match='domain'):
        field.run_executor(profile,tmp_path/'no_output',223,snapshot=True)
    assert not (tmp_path/'no_output').exists()
    with pytest.raises(ValueError,match='differ'):
        connection_remaps(profile,222,{'/joint_states':'/other'})


def test_mock_defaults_hide_device_time_and_do_not_enable_torque():
    config=mock_config(ROOT/'configs/sdk_mock.yaml')
    assert config['control_hz']==config['publish_hz']==1000
    assert config['interpolation_ns']==2000000 and config['cache_period_ns']>1000000
    assert not config['allow_effort'] and not config['allow_custom_pd']
    assert not config['plant']['zero_gravity']


@pytest.mark.parametrize('key,value',[('allow_effort',1),('cache_period_ns',0),('control_hz',0),('initial_q',[0]*13)])
def test_invalid_mock_configuration_is_rejected(tmp_path,key,value):
    config=yaml.safe_load((ROOT/'configs/sdk_mock.yaml').read_text());config[key]=value
    path=tmp_path/'invalid.yaml';path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):mock_config(path)


def test_h2_automatically_freezes_mock_connection_instead_of_hardware_scope(tmp_path,monkeypatch):
    profile=yaml.safe_load((ROOT/'configs/hardware_field.yaml').read_text())
    profile['collision_scene']=str(ROOT/'configs/collision_scene.yaml')
    profile['connection']=mock_connection('/casbot_calib_s2_unified',222,'a'*64)
    profile_path=tmp_path/'field.yaml';profile_path.write_text(yaml.safe_dump(profile))
    config=yaml.safe_load((ROOT/'configs/hardware_batch.yaml').read_text())
    config['frequency']['enabled']=config['static']['enabled']=False
    config['direction']['joints']=['left_shoulder_pitch_joint']
    path=tmp_path/'batch.yaml';path.write_text(yaml.safe_dump(config))
    monkeypatch.setattr(batch,'implementation',lambda:dict(unit_test=True))
    root=tmp_path/'batch'
    batch.plan_batch(path,profile_path,root,domain_id=222)
    _,p=batch.load_batch(root)
    assert p['connection_scope']=='localhost_contract_mock'
    assert p['profile']['connection']==profile['connection']
    with pytest.raises(ValueError,match='scope'):
        batch.plan_batch(path,profile_path,tmp_path/'wrong',domain_id=222,connection_scope='hardware')


def test_snapshot_cannot_be_relabelled_as_another_connection(tmp_path):
    from casbot_calib.data import write_json
    source=mock_connection('/casbot_calib_s2_first',222,'a'*64)
    write_json(tmp_path/'request.json',dict(config=dict(connection=source),domain_id=222))
    field.seal(tmp_path)
    assert field.snapshot_binding(tmp_path,dict(connection=source))['source_domain_id']==222
    with pytest.raises(ValueError,match='connections'):field.snapshot_binding(tmp_path,{})
    other=mock_connection('/casbot_calib_s2_second',222,'b'*64)
    with pytest.raises(ValueError,match='connections'):field.snapshot_binding(tmp_path,dict(connection=other))


def test_mock_initial_pose_checks_the_selected_scene_before_starting(tmp_path):
    from casbot_calib.sdk_mock import SDKMockSession
    scene=yaml.safe_load((ROOT/'configs/collision_scene.yaml').read_text())
    scene['obstacles'].append(dict(name='blocking_fixture',center=[0,0,0],half_extents=[10,10,10],attached_to_base=True))
    scene_path=tmp_path/'scene.yaml';scene_path.write_text(yaml.safe_dump(scene))
    config=yaml.safe_load((ROOT/'configs/hardware_field.yaml').read_text())
    config['collision_scene']=str(scene_path)
    path=tmp_path/'field.yaml';path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError,match='collision scene'):
        SDKMockSession(ROOT/'configs/sdk_mock.yaml',tmp_path/'not_started',222,profile=path)
    assert not (tmp_path/'not_started').exists()
