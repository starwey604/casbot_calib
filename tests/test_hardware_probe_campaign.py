"""Campaign inventory, legacy pair preservation and fixed subset fit budgets."""
from pathlib import Path
import numpy as np
from casbot_calib import hardware_field as field
from casbot_calib.hardware_probe_capture import design as pair_design
from casbot_calib.hardware_probe_campaign import design,variant_jobs,fit_variant
from casbot_calib.hardware_wrist_exploration import read
from casbot_calib.data import write_json
from casbot_calib.sdk_connection import mock_connection


def source_fixture(root):
    p=pair_design(root);write_json(root/'collection.json',dict(completed=True))
    a=root/'analysis';a.mkdir();write_json(a/'report.json',dict(all_motion_and_preprocessing_checks_passed=True));field.seal(a)
    profile=field.load_profile('configs/hardware_field.yaml')
    profile['connection']=mock_connection('/casbot_calib_s2_campaign_unit',227,'0'*64)
    for j in p['jobs']:
        plan=root/j['id']/'plan';plan.mkdir(parents=True)
        write_json(plan/'plan.json',dict(config=profile));field.seal(plan)
        capture=root/j['id']/'capture';capture.mkdir()
        write_json(capture/'result.json',dict(passed=True,local_contract_test=True));field.seal(capture)


def test_supplement_has_six_new_and_two_bound_records_with_honest_sessions(tmp_path):
    source=tmp_path/'source';source_fixture(source)
    probes=tmp_path/'points.npz';np.savez(probes,q=np.zeros((128,14)),dq=np.ones((128,14)),ddq=np.ones((128,14)))
    p=design(source,tmp_path/'supplement',probes)
    assert len(p['jobs'])==6 and len(p['reused_jobs'])==2
    assert {(j['center'],j['batch'],j['direction']) for j in p['jobs']+p['reused_jobs']}=={(c,b,d) for c in ['c00','c01'] for b in ['b00','b01'] for d in [-1,1]}
    assert p['poses']['c01'][12:14]==[-.2,.15]
    assert len({j['session'] for j in p['jobs']+p['reused_jobs']})==3
    assert p['fitting']['row_squared_weight_budget']==256 and p['operator_policy']['maximum_mass_diagonal_relative']==.1
    assert p['probe_config']['cache_period_ns']==5000000
    assert read(source/'protocol/experiment.json')['maximum_trials']==2
    assert not (tmp_path/'supplement/sessions').exists()


def test_logical_and_actual_session_deletions_are_distinct_and_duplicate_fit_can_reuse():
    jobs=[dict(id=str(k),center=c,batch=b,session='source_pair' if (c,b)==('c00','b00') else 'b00_extension' if b=='b00' else 'b01')
          for k,(b,c) in enumerate([('b00','c00'),('b00','c01'),('b01','c00'),('b01','c01')])]
    variants=dict(variant_jobs(jobs))
    assert len(variants)==8
    assert variants['delete_batch_b01']==variants['delete_session_b01']
    assert variants['delete_batch_b00']!=variants['delete_session_source_pair']


def test_delete_group_does_not_renormalize_surviving_record_rows(monkeypatch):
    from casbot_calib import hardware_probe_campaign as campaign
    calls=[]
    def fake(y,target,prior,**kw):calls.append((y.copy(),kw));return dict(parameters=prior.tolist())
    monkeypatch.setattr(campaign,'fit_rigid',fake)
    p={name:dict(regression=np.ones((n,140)),rigid_target=np.zeros(n)) for name,n in [('a',4),('b',8)]}
    policy=dict(row_squared_weight_budget=256,ridge=1e-4,torque_scale_nm=.2,robust_scale_nm=.02,rank_rtol=1e-7,physical_length_scale_m=.1)
    prior=np.ones(140);scales=np.arange(1,141);metric=np.eye(140);equalities=(np.zeros((1,140)),np.zeros(1))
    full,w=fit_variant(p,[dict(id='a'),dict(id='b')],None,prior,scales,metric,equalities,policy)
    subset,sw=fit_variant(p,[dict(id='a')],None,prior,scales,metric,equalities,policy)
    np.testing.assert_array_equal(calls[0][0][:4],calls[1][0])
    assert w[0]==sw[0] and w[0]['squared_budget']==256
    assert calls[0][1]['fixed_scales'] is calls[1][1]['fixed_scales']
    assert calls[0][1]['regularization_metric'] is calls[1][1]['regularization_metric']


def test_completed_analysis_exports_only_a_development_comparison_candidate(tmp_path):
    from casbot_calib.hardware_probe_campaign import export_candidate
    root=tmp_path/'campaign';protocol=root/'protocol';protocol.mkdir(parents=True)
    write_json(protocol/'experiment.json',dict(scope='unit'));field.seal(protocol)
    profile=field.load_profile('configs/hardware_field.yaml')
    p=dict(fixture_friction_prior=dict(viscous=[.01]*14,coulomb=[.005]*14,static_friction=[.01]*14),model_profile=profile)
    out=root/'analysis';out.mkdir()
    export_candidate(root,p,out,[1.]*140,True)
    candidate=read(out/'development_candidate/candidate.json')
    field.check_seal(out/'development_candidate')
    assert candidate['identified_scope']==['right_wrist_pitch_joint']
    assert candidate['source_operator_checks_passed']
    assert not candidate['auto_deploy'] and not candidate['formal_hardware_calibration_accepted']
