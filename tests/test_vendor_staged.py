import copy
import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from test_dynamics_s3 import fixture,imported
from casbot_calib.vendor_staged import freeze_staged,reference,StagedObservations,analyse_staged,require_wrist_probes
from casbot_calib.torque_observations import output_torque


def write(path,value):Path(path).write_text(json.dumps(value))


def spec(tmp_path,*,records=(),preflight=None):
    value=dict(schema_version='casbot_vendor_s4_spec_v1',scope='synthetic_observations',
        settings=dict(mode='qualified_observations_only',drive={}),records=list(records),preflight=preflight or {})
    path=tmp_path/'spec.yaml';path.write_text(yaml.safe_dump(value));return path


def row(tmp_path,*,name='friction_train',split='train'):
    motion=tmp_path/(name+'.json')
    write(motion,dict(name=name,stage='friction',conditions=dict(batch=split,joint=0,speed=.04),knots_s=[0,14],labels=['constant_velocity']))
    gate=tmp_path/'gate.json';write(gate,dict(passed=True,scope='test fixture, no physical path claim'))
    return dict(name=name,stage='friction',split=split,time_origin_ns=0,motion=reference(motion),
        qualification=reference(tmp_path/'qualification/manifest.json'),reference_collision=reference(gate),actual_collision=reference(gate))


def test_torque_channels_are_exclusive_and_gain_is_applied_once():
    current=np.ones((5,14));torque=output_torque(dict(current=current),2.)
    np.testing.assert_array_equal(torque,output_torque(dict(torque_nm=torque),None))
    with pytest.raises(ValueError,match='one torque observation'):output_torque(dict(current=current,torque_nm=torque),2.)
    with pytest.raises(ValueError,match='declared output gain'):output_torque(dict(current=current))


@pytest.mark.parametrize('missing',['one_arm','validation','reverse'])
def test_small_total_residual_cannot_replace_both_wrist_probe_roles(missing):
    complete=[dict(joint=j,direction=d,batch=b) for j in (4,11) for d in (-1,1) for b in ('train','validation')]
    require_wrist_probes(complete)
    if missing=='one_arm':incomplete=[c for c in complete if c['joint']==4]
    if missing=='validation':incomplete=[c for c in complete if c['batch']=='train']
    if missing=='reverse':incomplete=[c for c in complete if c['direction']==1]
    with pytest.raises(ValueError,match='Both wrists'):require_wrist_probes(incomplete)


def test_unknown_protocol_is_inspectable_but_cannot_fit_or_execute(tmp_path):
    report=freeze_staged(spec(tmp_path),tmp_path/'protocol')
    assert not report['passed'] and all(not r['execution_enabled'] for r in report['stages'].values())
    result=analyse_staged(tmp_path/'protocol',tmp_path/'analysis')
    assert not result['passed'] and not result['stages'] and 'preflight' in result['blocked_reason']
    assert not result['hardware_executable'] and not result['independent_final_test_passed']


def test_frozen_protocol_tamper_is_rejected(tmp_path):
    freeze_staged(spec(tmp_path),tmp_path/'protocol')
    with (tmp_path/'protocol/protocol.json').open('a') as f:f.write(' ')
    with pytest.raises(ValueError,match='protocol changed'):StagedObservations(tmp_path/'protocol')


def test_reader_returns_real_torque_without_current_alias(tmp_path,fixture):
    imported(tmp_path,fixture);entry=row(tmp_path)
    freeze_staged(spec(tmp_path,records=[entry]),tmp_path/'protocol')
    reader=StagedObservations(tmp_path/'protocol');records=list(reader('friction'))
    assert len(records)==1 and 'current' not in records[0][2]
    np.testing.assert_allclose(records[0][2]['torque_nm'],fixture[0]['current']*2)
    assert reader.config['drive']['gain_nm_a'] is None


def test_same_capture_cannot_be_both_training_and_validation(tmp_path,fixture):
    imported(tmp_path,fixture)
    entries=[row(tmp_path),row(tmp_path,name='friction_validation',split='validation')]
    with pytest.raises(ValueError,match='Same raw capture'):freeze_staged(spec(tmp_path,records=entries),tmp_path/'protocol')


def test_role_must_match_recorded_motion(tmp_path,fixture):
    imported(tmp_path,fixture);entry=row(tmp_path);entry['split']='validation'
    with pytest.raises(ValueError,match='Role differs'):freeze_staged(spec(tmp_path,records=[entry]),tmp_path/'protocol')


def test_probe_cannot_silently_expand_the_qualified_frequency_band(tmp_path,fixture):
    imported(tmp_path,fixture);entry=row(tmp_path);entry['stage']='inertia_probe'
    path=Path(entry['motion']['path']);motion=json.loads(path.read_text());motion['stage']='inertia_probe'
    write(path,motion);entry['motion']=reference(path)
    freeze_staged(spec(tmp_path,records=[entry]),tmp_path/'protocol')
    with pytest.raises(ValueError,match='30Hz'):StagedObservations(tmp_path/'protocol').load(entry['name'])


@pytest.mark.parametrize('source_batch',['left_validation','right_validation'])
def test_legacy_single_arm_validation_stays_validation_not_training(tmp_path,fixture,source_batch):
    imported(tmp_path,fixture);entry=row(tmp_path,name='independent_inertia',split='validation');entry['stage']='inertia'
    motion=Path(entry['motion']['path']);value=json.loads(motion.read_text())
    value['stage']='inertia';value['conditions']['batch']=source_batch
    write(motion,value);entry['motion']=reference(motion)
    freeze_staged(spec(tmp_path,records=[entry]),tmp_path/'protocol')
    _,loaded,_,_,_=StagedObservations(tmp_path/'protocol').load(entry['name'])
    assert loaded['conditions']['batch']=='validation' and loaded['conditions']['source_batch']==source_batch
    assert json.loads(motion.read_text())['conditions']['batch']==source_batch


@pytest.mark.parametrize('bad',['gap','nonuniform','async','collision'])
def test_s3_eligibility_does_not_bypass_s4_kernel_requirements(tmp_path,fixture,bad):
    def raw_change(raw):
        if bad=='gap':
            for key in list(raw):raw[key]=np.delete(raw[key],28000,axis=0)
        if bad=='nonuniform':raw['sample_ns'][::2]+=1000
        if bad=='async':raw['encoder_ns']=raw['sample_ns']+50000
    def change(c):
        if bad=='async':c['position_time']['field']='encoder_ns'
    imported(tmp_path,fixture,raw_change=raw_change,change=change);entry=row(tmp_path)
    if bad=='collision':
        write(tmp_path/'gate.json',dict(passed=False));entry['actual_collision']=reference(tmp_path/'gate.json');entry['reference_collision']=reference(tmp_path/'gate.json')
    freeze_staged(spec(tmp_path,records=[entry]),tmp_path/'protocol')
    with pytest.raises(ValueError):list(StagedObservations(tmp_path/'protocol')('friction'))


def test_previous_failure_blocks_following_fitter(tmp_path,monkeypatch):
    import casbot_calib.vendor_staged as module
    import casbot_calib.staged_identification as fitting
    protocol=tmp_path/'protocol';protocol.mkdir();write(protocol/'protocol.json',{})
    class Reader:
        config={};protocol=dict(scope='synthetic_observations');preflight=dict(mapping=dict(passed=True),timing=dict(passed=True))
        def __init__(self,*args):pass
        def __call__(self,stage):
            assert stage=='gravity'
            for split in ('train','validation'):
                for pose in range(2):
                    for sign in (-1,1):yield str((split,pose,sign)),dict(conditions=dict(batch=split,pose=pose,approach=sign)),{}
    monkeypatch.setattr(module,'StagedObservations',Reader)
    monkeypatch.setattr(fitting,'identify_gravity',lambda *a,**k:(dict(passed=False),None))
    def forbidden(*args,**kwargs):raise AssertionError('Following stage must not be called')
    monkeypatch.setattr(fitting,'identify_friction',forbidden)
    result=analyse_staged(protocol,tmp_path/'analysis')
    assert not result['passed'] and list(result['stages'])==['gravity']
    assert 'following stage blocked' in result['blocked_reason']
