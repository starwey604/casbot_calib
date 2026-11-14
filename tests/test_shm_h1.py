"""SHM H1 contracts use real native physics/Coal; no hardware endpoints."""
import csv
import json
import os
from pathlib import Path
import shutil
import struct
import sys
import numpy as np
import pytest
import yaml
from casbot_calib.shm_h1 import snapshot,prepare,execute,analyse,load_profile,replan
from casbot_calib.field_artifacts import check_seal,seal,FieldContractError
from casbot_calib.shm_data import ROOT
from casbot_calib.shm_h1_analysis import TimeMean

pytestmark=pytest.mark.skipif(sys.platform!='linux',reason='POSIX SHM only')
NATIVE=Path(os.environ.get('CASBOT_SHM_TEST_BIN',ROOT/'build/humble/cpp/src/shm'))/'shm_executor'


@pytest.fixture
def config(tmp_path):
    c=yaml.safe_load((ROOT/'configs/shm/hardware_field.yaml').read_text())
    c['collision_scene']=str(ROOT/'configs/collision_scene.yaml')
    c['initial_hold_s']=c['final_hold_s']=.05;c['anchor_transition_s']=.1
    p=tmp_path/'profile.yaml';p.write_text(yaml.safe_dump(c));return p


@pytest.fixture
def planned(tmp_path,config):
    if not NATIVE.exists():pytest.skip('Build SHM physics executables')
    snap=tmp_path/'snapshot';snapshot(config,snap,executor=NATIVE,mock=True)
    plan=tmp_path/'plan';prepare(config,snap,plan,executor=NATIVE,amplitude_deg=.1)
    return snap,plan


def test_h1_reference_keeps_continuous_collision_proof_and_explicit_gravity(planned):
    snap,plan=planned;m=json.loads((plan/'plan.json').read_text());reference=json.loads((plan/'reference.json').read_text())
    check_seal(snap);check_seal(plan)
    assert json.loads((plan/'collision_reference.json').read_text())['passed']
    assert m['config']['transport']=='shm_mc' and 'sdk_identity' not in m
    assert m['scope']=='private_mock' and reference['origin']=='absolute'
    assert reference['purpose']=='shm_h1_collision_checked'
    assert any(any(abs(t)>0 for t in r['value']) for r in reference['effort'])
    assert all(isinstance(t,int) for t in reference['knots_ns'])
    assert 'nominal_gravity_at_reference_q' in m['config']['assumptions']['gravity_feedforward']


def test_h1_preview_never_spawns_and_mock_plan_cannot_target_real_shm(planned,tmp_path):
    _,plan=planned;out=tmp_path/'never'
    assert execute(plan,out,executor=NATIVE,mock=True)==0 and not out.exists()
    with pytest.raises(FieldContractError,match='范围不同'):execute(plan,out,executor=NATIVE,mock=False,do_execute=True)
    assert not out.exists()


def test_complete_h1_physics_capture_reanalysis_and_tamper_rejection(planned,tmp_path):
    _,plan=planned;capture=tmp_path/'capture'
    assert execute(plan,capture,executor=NATIVE,mock=True,do_execute=True)==0
    report=analyse(plan,capture,tmp_path/'analysis')
    assert report['execution_completed'] and report['analysis_available'] and not report['hardware_calibration_accepted']
    assert report['time_basis']=='mapped master conversion time, not acquisition'
    assert report['observation_time_origin_ns']>report['first_shm_write_mono_ns']
    assert json.loads((tmp_path/'analysis/collision_received_interpolant.json').read_text())['passed']
    child=capture/json.loads((capture/'result.json').read_text())['effective_capture']
    with (child/'native/feedback.csv').open('a') as f:f.write('changed\n')
    with pytest.raises(FieldContractError):analyse(plan,capture,tmp_path/'tampered')


def test_replan_reuses_recipe_and_new_actual_start(planned,tmp_path,config):
    _,plan=planned;snap=tmp_path/'new';snapshot(config,snap,executor=NATIVE,mock=True)
    new=tmp_path/'replanned';report=replan(plan,snap,new,executor=NATIVE)
    assert report['recipe']==json.loads((plan/'plan.json').read_text())['recipe']
    assert json.loads((new/'collision_reference.json').read_text())['passed']


def test_collision_refusal_is_not_the_fieldwork_jog_exception(planned,tmp_path,config):
    snap,_=planned;unsafe=tmp_path/'unsafe';shutil.copytree(snap,unsafe)
    raw=bytearray((unsafe/'native/status.bin').read_bytes())
    # All-zero model pose has the known shoulder/body collision.
    from casbot_calib.shm_data import DIRECTION,OFFSET
    for j in range(14):
        struct.pack_into('<i',raw,12+(j+12)*16+2,int(DIRECTION[j]*(0-OFFSET[j])*1e6))
        struct.pack_into('<i',raw,12+(j+12)*16+6,0)
    (unsafe/'native/status.bin').write_bytes(raw);seal(unsafe)
    with pytest.raises(FieldContractError,match='碰撞'):prepare(config,unsafe,tmp_path/'refused',executor=NATIVE,amplitude_deg=.1)


def test_time_mean_equalizes_duplicate_groups_and_refuses_jumps():
    def compute(repeats):
        mean=TimeMean()
        for t,count in zip((0,10,20),repeats):
            for _ in range(count):mean.add(dict(observation_time_ns=t,clock_segment=0,time_ambiguous=False),np.array([t,t*t],float))
        return mean.result(),mean
    a,_=compute((1,1,1));b,mean=compute((1,4,2));np.testing.assert_array_equal(a,b)
    assert mean.groups==3 and mean.rows==7
    mean=TimeMean();mean.add(dict(observation_time_ns=0,clock_segment=0,time_ambiguous=False),np.array([1.]))
    mean.add(dict(observation_time_ns=10,clock_segment=1,time_ambiguous=True),np.array([2.]))
    assert mean.result() is None


def test_only_zero_write_no_fault_start_mismatch_can_replan(planned,tmp_path,monkeypatch):
    import casbot_calib.shm_h1 as h1
    snap,plan=planned;original=h1._execute_once;calls=[]
    def once(plan_root,out,**kwargs):
        calls.append(kwargs.get('do_execute',False))
        if len(calls)==1:
            out=Path(out);(out/'native').mkdir(parents=True)
            shutil.copyfile(snap/'native/status.bin',out/'native/status.bin')
            (out/'result.json').write_text(json.dumps(dict(native=dict(commands_sent='0',first_shm_write_mono_ns='0',reason='Reference start differs from fresh measured position; no commands sent'))))
            seal(out);return 1
        return original(plan_root,out,**kwargs)
    monkeypatch.setattr(h1,'_execute_once',once)
    out=tmp_path/'refresh'
    assert execute(plan,out,executor=NATIVE,mock=True,do_execute=True)==0
    result=json.loads((out/'result.json').read_text())
    assert len(result['attempts'])==2 and result['attempts'][0]['reason']=='start_replan'
    assert result['effective_plan'] and not result['resume_permitted']
    assert analyse(plan,out,tmp_path/'after_refresh')['execution_completed']


def test_fault_during_execution_ends_collection_without_resume(tmp_path,config):
    cfg=json.loads((ROOT/'configs/shm/h1_mock.json').read_text());cfg['urdf']=str((ROOT/'configs/shm'/cfg['urdf']).resolve());cfg['fault_after_commands']=80
    mock=tmp_path/'fault.json';mock.write_text(json.dumps(cfg))
    snap=tmp_path/'snapshot';snapshot(config,snap,executor=NATIVE,mock=True,mock_config=mock)
    plan=tmp_path/'plan';prepare(config,snap,plan,executor=NATIVE,amplitude_deg=.1)
    out=tmp_path/'fault';assert execute(plan,out,executor=NATIVE,mock=True,do_execute=True)!=0
    result=json.loads((out/'result.json').read_text())
    assert len(result['attempts'])==1 and not result['execution_completed'] and not result['resume_permitted']
    assert 'drive_error' in result['attempts'][0]['reason']
    with pytest.raises(FieldContractError,match='未完成'):analyse(plan,out,tmp_path/'not_a_success')
