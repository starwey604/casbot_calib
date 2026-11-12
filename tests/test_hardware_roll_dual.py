"""New dual-tone protocol, original callers, and analytical delivered feedback."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from casbot_calib import hardware_roll_dual as dual
from casbot_calib import hardware_probe_capture as capture
from casbot_calib.hardware_probe import settings
from test_hardware_probe import rotor


def policy():
    return dict(primary_model='free_b_complex',comparison_models=['zero_phase_local','frozen_b_complex','free_b_complex'],
        inertia_scale_kg_m2=5e-5,viscous_scale_nm_s_rad=.035,leave_blocks=4,leave_block_duration_s=.4)


def fixture(advance=0.,direction=1):
    t=np.arange(0,4,.001);s=np.floor((t-.0013)/.005)*.005+.0013;f=np.array([2.5,10.]);a=np.array([.0075,.00068]);w=2*np.pi*f
    def channels(x):
        angle=x[:,None]*w
        return .18*x+np.sum(a*np.sin(angle),axis=1),.18+np.sum(a*w*np.cos(angle),axis=1),np.sum(-a*w*w*np.sin(angle),axis=1)
    q,v,_=channels(s);_,tv,ta=channels(s+advance);z=np.zeros((len(t),14));qs=z.copy();vs=z.copy();tau=z.copy()
    qs[:,13]=direction*q;vs[:,13]=direction*v;tau[:,13]=direction*(5e-5*ta+.035*tv+.04)
    obs=dict(receive_ns=10**15+np.rint(t*1e9).astype(np.int64),q=qs,dq=vs,effort_raw=tau)
    meta=dict(conditions=dict(joint=13,direction=direction),labels=['initial','probe_measure','final'],knots_s=[0.,1.,3.,4.])
    prior=dict(viscous=[.035]*14,coulomb=[.04]*14,static_friction=[.04]*14)
    return obs,t,meta,prior


@pytest.mark.parametrize('advance',[-.005,0.,.005])
def test_adapter_recovers_free_and_frozen_models_from_cached_tones(advance):
    obs,t,meta,prior=fixture(advance)
    r,p,_=dual.inspect(obs,t,meta,rotor(),settings(),dict(effort_gain=1.,effort_zero_nm=0.),prior,policy(),[[2.5,.0075],[10.,.00068]])
    assert r['probe_preprocessing_passed'] and [x['frequency_hz'] for x in r['frequency_signals']]==[2.5,10.]
    assert not any('5hz' in k for k in r)
    full=r['frequency_analysis']['full'];assert full['available'] and full['observed_rank']==3
    for model in ['free_b_complex','frozen_b_complex']:
        assert abs(full[model]['inertia_kg_m2']/5e-5-1)<.01
        assert abs(full[model]['effective_advance_s']-advance)<3e-5
    assert len(r['frequency_analysis']['leave_block_out'])==4
    assert not r['frequency_analysis']['phase_correction_written_to_observations']
    assert not r['sample_time_known']


def test_failed_quality_still_reports_both_tones_but_does_not_fit():
    obs,t,meta,prior=fixture();obs['dq'][:,3]=.002
    r,p,_=dual.inspect(obs,t,meta,rotor(),settings(),dict(effort_gain=1.,effort_zero_nm=0.),prior,policy(),[[2.5,.0075],[10.,.00068]])
    assert p is None and not r['raw_checks']['other_axes_held']
    assert len(r['frequency_signals'])==2 and 'frequency_analysis' not in r


def test_missing_delivered_second_tone_is_unidentifiable_not_an_exact_fit():
    t=np.arange(0,1.6,.002);w=5*np.pi;v=.18+.12*np.cos(w*t);a=-.12*w*np.sin(w*t);z=np.zeros((len(t),28));z[:,13]=v
    p=dict(joint=13,time_s=t,dq=np.zeros((len(t),14)),ddq=np.zeros((len(t),14)),extra=z,static_feature=np.zeros(len(t)),
        observed_torque_nm=5e-5*a+.035*v+.04,projected_gravity_nm=np.zeros(len(t)),rotor=np.zeros(len(t)))
    p['dq'][:,13]=v;p['ddq'][:,13]=a
    prior=dict(viscous=[.035]*14,coulomb=[.04]*14,static_friction=[.04]*14)
    result=dual.frequency_analysis(p,prior,policy(),[2.5,10.])
    assert not result['full']['available'] and result['full']['observed_rank']==2
    assert result['leave_block_out']==[]


def freeze_fixture(tmp_path):
    source=tmp_path/'old';(source/'protocol').mkdir(parents=True);c=settings()
    old=dict(schema_version='casbot_sdk_wrist_roll_screen_v1',joint=13,probe_config=c,holding=dict(hold_minimum_kp=30.),transition_s=4.,settle_s=3.,
        fixture_friction_prior=dict(viscous=[.01]*14,coulomb=[.005]*14,static_friction=[.01]*14),prior_status='fixture',command_hz=500,pose_rad=[0.]*14)
    (source/'protocol/experiment.json').write_text(json.dumps(old))
    for name in ['mock.yaml','scene.yaml']:(source/'protocol'/name).write_text('fixture: true\n')
    (source/'protocol/base_profile.yaml').write_text(Path('configs/hardware_field.yaml').read_text())
    dual.field.seal(source/'protocol');design=tmp_path/'design';(design/'references').mkdir(parents=True)
    report=dict(selected_id='dual',designs=[dict(id='dual',tones=[[2.5,.0075],[10.,.00068]])],protocol=dict(source=str(source)),
        selected_reference_checks=[dict(full_reference_passed=True,command_preview_passed=True)],planning_inertia_kg_m2=5e-5,planning_viscous_nm_s_rad=.01)
    (design/'report.json').write_text(json.dumps(report));dual.field.seal(design)
    root=tmp_path/'new';p=dual.freeze(design,root);return root,p


def test_freeze_is_self_contained_and_does_not_open_sdk(tmp_path,monkeypatch):
    def no_io(*a,**kw):raise AssertionError('No SDK for freeze')
    monkeypatch.setattr(capture.SDKMockSession,'__enter__',no_io)
    root,p=freeze_fixture(tmp_path)
    assert len(p['jobs'])==p['maximum_trials']==2 and p['tones']==[[2.5,.0075],[10.,.00068]]
    assert 'frequency_hz' not in p['probe_config']['probe'] and 'amplitude_rad' not in p['probe_config']['probe']
    assert not p['automatic_expansion'] and p['original_inertia_relative_threshold']==.1
    assert p['probe_config']['schema_version']=='casbot_sdk_wrist_probe_v2'
    assert 'maximum_callback_gap_s' not in p['probe_config']['processing']
    assert not (root/'mock').exists() and not (root/'collection.json').exists()
    # Runtime uses only the copied input bundle, not the former design/source.
    (tmp_path/'old').rename(tmp_path/'old_moved');(tmp_path/'design').rename(tmp_path/'design_moved')
    assert dual.load_protocol(root)==p


def mock_collection(monkeypatch,starts,execute):
    calls=[]
    class Mock:
        def __init__(self,*a,**kw):calls.append('mock_init')
        def __enter__(self):return SimpleNamespace(client_profile='private_mock')
        def __exit__(self,*a):calls.append('closed')
    monkeypatch.setattr(capture,'SDKMockSession',Mock)
    monkeypatch.setattr(capture.field,'snapshot_field',lambda *a:dict(passed=True))
    monkeypatch.setattr(capture.field,'_read_snapshot',lambda *a:starts.pop(0))
    monkeypatch.setattr(capture.field,'load_profile',lambda *a:{})
    monkeypatch.setattr(capture,'require_local',lambda *a:None)
    monkeypatch.setattr(capture,'holding_gains',lambda *a:(np.ones(14),np.ones(14)))
    monkeypatch.setattr(capture.field,'snapshot_binding',lambda *a:{})
    monkeypatch.setattr(capture.field,'execute_field',execute)
    return calls


def test_shared_collector_builds_dual_motion_from_each_actual_snapshot(tmp_path,monkeypatch):
    root,p=freeze_fixture(tmp_path);starts=[np.full(14,.001),np.full(14,.002)];seen=[]
    calls=mock_collection(monkeypatch,starts,lambda *a:dict(passed=True))
    monkeypatch.setattr(capture.field,'finish_plan',lambda folder,profile,motion,*a:seen.append(motion))
    dual.collect(root,227)
    assert calls==['mock_init','closed'] and len(seen)==2
    np.testing.assert_allclose(seen[0].states[0][0],.001);np.testing.assert_allclose(seen[1].states[0][0],.002)
    assert [m.conditions['direction'] for m in seen]==[-1,1]
    assert all(m.conditions['tones']==[dict(frequency_hz=f,amplitude_rad=a) for f,a in p['tones']] for m in seen)
    assert json.loads((root/'collection.json').read_text())['completed']


def test_sdk_fault_ends_pair_and_blocks_reentry_before_opening_mock(tmp_path,monkeypatch):
    root,p=freeze_fixture(tmp_path);calls=mock_collection(monkeypatch,[np.zeros(14)],lambda *a:dict(passed=False,fault='mode_lost'))
    monkeypatch.setattr(capture.field,'finish_plan',lambda *a:None)
    with pytest.raises(RuntimeError,match='terminate whole session'):dual.collect(root,227)
    result=json.loads((root/'collection.json').read_text());assert result['terminal_fault'] and not result['resume_permitted']
    assert result['clear_fault']['status']=='not_implemented' and not result['clear_fault']['performed']
    with pytest.raises(RuntimeError,match='cannot resume'):dual.collect(root,227)
    assert calls==['mock_init','closed'] and not (root/p['jobs'][1]['id']).exists()


def test_analysis_requires_observations_and_does_not_start_collection(tmp_path,monkeypatch):
    root,p=freeze_fixture(tmp_path)
    def no_io(*a,**kw):raise AssertionError('Analysis cannot collect')
    monkeypatch.setattr(capture.SDKMockSession,'__enter__',no_io)
    with pytest.raises(ValueError,match='No completed dual-tone collection'):dual.analyse(root,tmp_path/'analysis')
    assert not (root/'mock').exists() and not (tmp_path/'analysis').exists()


def test_readonly_analysis_entry_consumes_delivered_arrays_and_keeps_protocol_binding(tmp_path,monkeypatch):
    # A temporary test transport, not a fabricated SDK capture in artifacts.
    root,p=freeze_fixture(tmp_path);profile=dict(model_context='test_rotor',gravity_world=[0.,0.,0.],effort_gain=1.,effort_zero_nm=0.,gravity_feedforward='nominal',command_hz=500)
    (root/'collection.json').write_text(json.dumps(dict(completed=True)))
    motions={};delivered={}
    for job in p['jobs']:
        obs,t,meta,prior=fixture(direction=job['direction']);meta['conditions']['tones']=[dict(frequency_hz=f,amplitude_rad=a) for f,a in p['tones']]
        motions[str(root/job['id']/'plan/motion.json')]=meta;motions[str(root/job['id']/'capture/request.json')]=dict(domain_id=227)
        delivered[str(root/job['id']/'plan')]=(dict(config=profile),None,None,obs,t,dict(passed=True))
    # Use the analytical fixture's frozen friction for this temporary contract.
    p['fixture_friction_prior']=prior;p['observation_policy'].update(policy())
    (root/'protocol/experiment.json').write_text(json.dumps(p));dual.field.seal(root/'protocol')
    original_read=dual.read
    monkeypatch.setattr(dual,'read',lambda path:deepcopy(motions[str(path)]) if str(path) in motions else original_read(path))
    monkeypatch.setattr(dual.field,'resolve_field_capture',lambda plan,capture:(plan,capture))
    monkeypatch.setattr(dual.field,'field_observations',lambda plan,capture:delivered[str(plan)])
    monkeypatch.setattr(dual.field,'load_profile',lambda *a:profile)
    monkeypatch.setattr(dual,'require_local',lambda *a:None)
    monkeypatch.setattr(dual,'model_for',lambda *a:(rotor(),None))
    monkeypatch.setattr(dual,'plot',lambda *a:None)
    out=tmp_path/'analysis';result=dual.analyse(root,out)
    assert result['all_frequency_models_available'] and len(result['model_comparisons'])==3
    assert result['source_protocol_manifest_sha256'] and result['joint']==13
    assert not result['formal_hardware_calibration_accepted'] and not result['full_p0_r1_r2_acceptance']
    dual.field.check_seal(out)
    changed=deepcopy(profile);changed['effort_gain']=1.1
    with pytest.raises(ValueError,match='effort_gain'):dual.check_record(p,p['jobs'][0],motions[str(root/p['jobs'][0]['id']/'plan/motion.json')],changed,profile)


@pytest.mark.parametrize('action,blocked',[('collect',False),('analyse',True)])
def test_cli_truth_guard_separates_mock_readiness_from_observation_analysis(tmp_path,action,blocked):
    import subprocess
    import sys
    target=tmp_path/'evaluation/server/server_events.csv';target.parent.mkdir(parents=True);target.write_text('settled_initial_hold')
    # The audit hook is process-lifetime; keep this test isolated, with no SDK.
    code="""import runpy,sys
from pathlib import Path
api=runpy.run_path('scripts/humble/sdk_roll_dual.py')
api['install_analysis_guard'](sys.argv[1])
try:
    Path(sys.argv[2]).read_text()
except RuntimeError:
    print('blocked')
else:
    print('allowed')
"""
    result=subprocess.run([sys.executable,'-c',code,action,str(target)],check=True,capture_output=True,text=True)
    assert result.stdout.strip()==('blocked' if blocked else 'allowed')


def test_cli_collect_dispatches_fresh_readonly_process_then_marks_analysis_complete(tmp_path,monkeypatch):
    import runpy
    import subprocess
    import sys
    api=runpy.run_path('scripts/humble/sdk_roll_dual.py');root=tmp_path/'frozen';root.mkdir();calls=[]
    monkeypatch.setattr(dual,'collect',lambda source,domain:calls.append(('collect',source,domain)))
    monkeypatch.setattr(subprocess,'run',lambda command,**kw:calls.append(('analysis',command,kw)))
    monkeypatch.setattr(sys,'argv',['sdk_roll_dual.py','collect','--source',str(root),'--domain-id','227'])
    api['main']()
    assert calls[0]==('collect',root,227) and calls[1][1][2]=='analyse'
    assert calls[1][1][0]==sys.executable and calls[1][2]['check']
    status=json.loads((root/'status.json').read_text());assert status['stage']=='collected_and_analysed' and status['records_collected']==2
    assert not status['physical_device_tested']
