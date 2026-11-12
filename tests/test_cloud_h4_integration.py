"""Cloud H4 semantics meet local wrist protocols without changing old evidence."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from casbot_calib import hardware_field as field
from casbot_calib.hardware_probe import settings,field_probe_config,probe_problem,make_probe
from casbot_calib.hardware_probe_capture import inspect_observations
from casbot_calib.hardware_roll_dual import protocol_motion
from test_hardware_probe import fixture,rotor
from test_hardware_field import planned,profile
from test_hardware_start_replan import moved_snapshot


def test_clean_projection_numerics_are_identical_across_quality_policy_versions():
    obs,t,meta=fixture();old=settings();new=field_probe_config(old);engine=rotor();p=dict(effort_gain=1.,effort_zero_nm=0.)
    a=probe_problem(obs,t,meta,engine,old,p);b=probe_problem(obs,t,meta,engine,new,p)
    assert new['schema_version']=='casbot_sdk_wrist_probe_v2' and 'maximum_callback_gap_s' not in new['processing']
    assert 'maximum_callback_gap_s' in old['processing']
    for key in a:
        if isinstance(a[key],np.ndarray):np.testing.assert_array_equal(a[key],b[key])
    assert not b['preprocessing']['empirical_quality_gate_applied'] and a['preprocessing']['empirical_quality_gate_applied']


@pytest.mark.parametrize('change',['held','gap','velocity_mismatch','stop'])
def test_new_field_quality_is_diagnostic_while_frozen_v1_keeps_its_contract(change):
    obs,t,meta=fixture();old=settings();new=field_probe_config(old)
    if change=='held':obs['dq'][:,0]=.002
    elif change=='gap':
        keep=~((t>1.7)&(t<1.78));t=t[keep];obs={k:v[keep] for k,v in obs.items()}
    elif change=='velocity_mismatch':obs['q'][:,4]*=2.
    else:obs['dq'][(t>1.7)&(t<1.8),4]=0.
    p=dict(effort_gain=1.,effort_zero_nm=0.)
    with pytest.raises(ValueError):probe_problem(obs,t,meta,rotor(),old,p)
    prior=dict(viscous=[.035]*14,coulomb=[.04]*14,static_friction=[.04]*14)
    report,problem,_=inspect_observations(obs,t,meta,rotor(),new,p,prior)
    assert problem is not None and report['probe_preprocessing_passed']
    assert not report['empirical_quality_gate_applied'] and report['raw_checks']=={}
    assert np.isfinite(problem['regression']).all()


@pytest.mark.parametrize('tones',[None,[[2.5,.0075],[10.,.00068]]])
def test_custom_probe_replan_preserves_recipe_and_pd_but_uses_fresh_start(planned,tmp_path,tones):
    _,metadata=planned;config=deepcopy(metadata['config']);home=np.asarray(metadata['observed_start_rad']);pose=home.copy();pose[[5,12]]=.2;pose[[6,13]]=.3
    c=field_probe_config(settings());job=dict(id='probe',direction=1)
    recipe=dict(kind='sdk_wrist_probe',name='probe',pose_rad=pose.tolist(),joint=13,direction=1,config=c,transition_s=4.,settle_s=3.,tones=tones,job=job)
    config['breakaway_parameters']=dict(command_kp=[30.]*14,command_kd=[1.]*14)
    if tones is None:motion=make_probe('probe',home,pose,13,1,config=c)
    else:motion=protocol_motion('probe',home,pose,13,1,config=c,tones=tones,transition_s=4.,settle_s=3.)
    original=tmp_path/'probe';original.mkdir();before=field.finish_plan(original,config,motion,[],dict(**field.snapshot_binding(tmp_path/'snapshot',config),replan_recipe=recipe,job=job))
    fresh=tmp_path/'probe_snapshot';q=moved_snapshot(tmp_path/'snapshot',fresh,config)
    rebuilt=tmp_path/'probe_rebuilt';after=field.replan_field(original,fresh,rebuilt)
    assert after['replan_recipe']==recipe and after['job']==job and after['config']['breakaway_parameters']==config['breakaway_parameters']
    np.testing.assert_array_equal(after['observed_start_rad'],q)
    assert json.loads((rebuilt/'collision_reference.json').read_text())['passed']
    assert json.loads((rebuilt/'collision_command_interpolant.json').read_text())['passed']
    field.check_seal(original);field.check_seal(rebuilt)
