"""Independent analytical rotor plus cached public feedback; no collector."""
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.hardware_probe import settings,probe_problem,inertia_balance,make_probe


def rotor():
    m=_core.ArmModel();m.model_id='sdk_probe_rotors';m.model_version='tree_rigid_v1';m.gravity_world=np.zeros(3)
    joints=[]
    for k in range(14):
        j=_core.ArmJoint();j.name=f'axis_{k}';j.link_name=f'body_{k}';body=j.body;body.inertia_com=np.eye(3)*5e-5;j.body=body;joints.append(j)
    m.joints=joints;m.parents=[-1]*14
    return _core.ArmDynamics(m)


def fixture(*,phase=.0013,effort_offset=0.,callback_jitter=False,noise=0.):
    t=np.arange(0.,4.,.001)
    if callback_jitter:t=t+np.random.default_rng(12).uniform(0,.0003,len(t))
    sample=np.floor((t-phase)/.005)*.005+phase
    def channel(s):
        w=10*np.pi
        q=.18*s+.004*np.sin(w*s);v=.18+.004*w*np.cos(w*s);a=-.004*w*w*np.sin(w*s)
        return q,v,a
    q,v,a=channel(sample);_,tv,ta=channel(sample+effort_offset)
    effort=5e-5*ta+.035*tv+.04
    if noise:
        _,index=np.unique(sample,return_inverse=True)
        effort+=np.random.default_rng(8).normal(0,noise,index.max()+1)[index]
    z=np.zeros((len(t),14));qs=z.copy();vs=z.copy();tau=z.copy();qs[:,4]=q;vs[:,4]=v;tau[:,4]=effort
    obs=dict(receive_ns=10**15+np.rint(t*1e9).astype(np.int64),q=qs,dq=vs,effort_raw=tau,
             publish_stamp_ns=np.arange(len(t),dtype=np.int64))
    meta=dict(conditions=dict(joint=4,direction=1,frequency_hz=5.),labels=['initial','probe_measure','final'],knots_s=[0.,1.,3.,4.])
    return obs,t,meta


@pytest.mark.parametrize('phase',[0.,.0013,.0027,.0049])
def test_common_5ms_cache_preserves_torque_regressor_and_small_inertia(phase):
    e=rotor();obs,t,meta=fixture(phase=phase)
    p=probe_problem(obs,t,meta,e,settings(),dict(effort_gain=1.,effort_zero_nm=0.))
    r=inertia_balance(p)
    assert abs(r['inertia_kg_m2']/5e-5-1)<.02
    assert abs(r['viscous_nm_s_rad']/.035-1)<.02
    predicted=p['regression']@e.rigid_parameters()+p['extra']@np.r_[np.full(14,.035),np.full(14,.04)]
    assert np.sqrt(np.mean((predicted-p['target'])**2))<2e-6
    prep=p['preprocessing'];assert not prep['sample_time_known'] and prep['independent_acquisition_count'] is None
    assert not prep['evaluation_grid_is_acquisition'] and prep['cache_standard_ns']==5000000


def test_jittered_callback_bursts_keep_all_publications_without_sample_clock():
    obs,t,meta=fixture(callback_jitter=True,noise=.0001)
    p=probe_problem(obs,t,meta,rotor(),settings(),dict(effort_gain=1.,effort_zero_nm=0.))
    assert abs(inertia_balance(p)['inertia_kg_m2']/5e-5-1)<.10
    assert p['preprocessing']['callback_publications']==np.count_nonzero((t>=1)&(t<3))


@pytest.mark.parametrize('failure',['reversal','held','gap','inconsistent_velocity'])
def test_rejects_real_probe_failures(failure):
    obs,t,meta=fixture()
    if failure=='reversal':obs['dq'][(t>1.7)&(t<1.8),4]=0
    elif failure=='held':obs['dq'][:,0]=.002
    elif failure=='gap':
        keep=~((t>1.7)&(t<1.8));t=t[keep];obs={k:v[keep] for k,v in obs.items()}
    else:obs['q'][:,4]*=2
    with pytest.raises(ValueError):probe_problem(obs,t,meta,rotor(),settings(),dict(effort_gain=1.,effort_zero_nm=0.))


def test_5ms_shared_cache_does_not_imply_relative_channel_alignment():
    obs,t,meta=fixture(effort_offset=.005)
    p=probe_problem(obs,t,meta,rotor(),settings(),dict(effort_gain=1.,effort_zero_nm=0.))
    assert abs(inertia_balance(p)['inertia_kg_m2']/5e-5-1)>.10
    assert not p['preprocessing']['relative_channel_alignment_verified']


def test_sdk_reference_drops_native_4khz_acquisition_hint():
    from casbot_calib.model_runtime import model_for
    from casbot_calib.hardware_inertia import settings as h4_settings
    e,_=model_for({});c=h4_settings('configs/hardware_inertia.yaml',[j.name for j in e.model.joints])
    m=make_probe('no_fake_sample_clock',c['home_rad'],c['home_rad'],4,1)
    assert 'sample_period_ns' not in m.conditions
    assert m.conditions['observation_protocol']=='sdk_callback_nonreversing_v1'
