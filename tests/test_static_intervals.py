"""Physical stationary bounds survive Gibbs ringing but reject real torque bias."""
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.static_intervals import observation_problem,metrics,POLICY
from casbot_calib.staged_inertia import _project,_metrics


@pytest.fixture(scope='module')
def engine():
    e,_=nominal_dual_arm();m=e.model;m.gravity_world=np.zeros(3);return _core.ArmDynamics(m)


def observations(q,v,torque,step=250000):
    ns=np.arange(len(q),dtype=np.int64)*step
    return dict(q=q,dq=v,torque_nm=torque,sample_ns=ns,status=np.zeros_like(q,dtype=int))


def test_fourier_overshoot_is_diagnostic_not_a_physical_static_failure(engine):
    n=4096;q=np.zeros((n,14));effort=np.zeros_like(q);effort[:,0]=-.07;effort[n//2:,0]=.07
    raw=observation_problem(engine,observations(q,q.copy(),effort))
    physical=metrics(raw,engine.rigid_parameters(),np.full(14,.07))
    assert physical['stationary_qualified'] and physical['maximum_stationary_interval_excess_nm']==0.
    filtered=_project(effort,40);assert filtered[:,0].max()-.07>.01
    problem=dict(regression=np.zeros((n,14,140)),extra=np.zeros((n,14,28)),fixed_friction=np.zeros_like(q),rotor=np.zeros_like(q),
        current_torque=filtered,moving=np.zeros_like(q,dtype=bool),static_observations=raw)
    report=_metrics(problem,engine.rigid_parameters(),np.zeros(28),np.full(14,.07),.2)
    assert report['maximum_stationary_interval_excess_nm']==0
    assert report['filtered_stationary_diagnostic']['maximum_excess_nm']>.01
    assert not report['filtered_stationary_diagnostic']['used_for_acceptance']


def test_real_interval_violation_and_no_stationary_evidence_are_not_passes(engine):
    q=np.zeros((1200,14));effort=q.copy();effort[:,5]=.10
    p=observation_problem(engine,observations(q,q.copy(),effort));r=metrics(p,engine.rigid_parameters(),np.full(14,.07))
    assert r['stationary_interval_excess_nm'][5]==pytest.approx(.03)
    velocity=np.full_like(q,.001);position=np.arange(len(q))[:,None]*.00025*velocity
    p=observation_problem(engine,observations(position,velocity,np.zeros_like(q)))
    r=metrics(p,engine.rigid_parameters(),np.full(14,.07))
    assert not r['stationary_qualified'] and r['stationary_observations']['counts']==[0]*14


def test_position_motion_disqualifies_frozen_velocity_channel(engine):
    q=np.zeros((1000,14));q[:,2]=np.arange(1000)*1e-6
    p=observation_problem(engine,observations(q,np.zeros_like(q),np.zeros_like(q)))
    assert not p['eligible'] and p['counts'][2]==0


@pytest.mark.parametrize('fault',['gap','duplicate','invalid'])
def test_no_differentiation_across_bad_raw_time_or_status(engine,fault):
    q=np.zeros((1000,14));obs=observations(q,q.copy(),q.copy())
    if fault=='gap':obs['sample_ns'][500:]+=250000
    elif fault=='duplicate':obs['sample_ns'][500]=obs['sample_ns'][499]
    else:obs['status'][500,1]=1
    p=observation_problem(engine,obs);assert not p['eligible'] and len(p['q'])==0


def test_moving_chain_is_not_static_but_other_fixed_base_branch_is(engine):
    t=np.arange(4000)*.00025;q=np.zeros((len(t),14));v=q.copy();a=q.copy()
    q[:,0]=.2*np.sin(3*t);v[:,0]=.6*np.cos(3*t);a[:,0]=-1.8*np.sin(3*t)
    parts=engine.evaluate_batch(q,v,a);torque=parts['inertia']+parts['velocity']+parts['gravity']
    p=observation_problem(engine,observations(q,v,torque));r=metrics(p,engine.rigid_parameters(),np.full(14,1e-6))
    assert p['branches']==[list(range(7)),list(range(7,14))]
    assert p['counts'][:7]==[0]*7 and min(p['counts'][7:])>100
    assert r['stationary_interval_excess_nm'][:7]==[None]*7
    assert max(r['stationary_interval_excess_nm'][7:])<1e-7


def test_direct_nm_and_current_observations_have_the_same_single_channel_meaning(engine):
    q=np.zeros((1000,14));torque=np.ones_like(q)*.04
    direct=observations(q,q.copy(),torque);current={k:v for k,v in direct.items() if k!='torque_nm'};current['current']=torque/2
    a=observation_problem(engine,direct);b=observation_problem(engine,current,2.)
    np.testing.assert_array_equal(a['observed'],b['observed'])
    with pytest.raises(ValueError,match='one torque observation'):observation_problem(engine,dict(direct,current=torque),2.)


def test_actual_gravity_is_used_and_wrong_mass_is_not_hidden_by_static_bound():
    e,_=nominal_dual_arm();q=np.tile([0,.45,0,-.9,0,0,0,0,-.45,0,-.9,0,0,0],(1000,1));v=np.zeros_like(q)
    torque=e.gravity_batch(q)+.03
    p=observation_problem(e,observations(q,v,torque));theta=e.rigid_parameters();fs=np.full(14,.07)
    assert metrics(p,theta,fs)['maximum_stationary_interval_excess_nm']==0
    assert metrics(p,theta*1.2,fs)['maximum_stationary_interval_excess_nm']>.01


def test_missing_static_observations_block_prediction_acceptance(engine):
    from casbot_calib.staged_multifit import score_problems
    q=np.zeros((1000,14));v=np.ones_like(q)*.001
    raw=observation_problem(engine,observations(q,v,q.copy()))
    n=64;p=dict(regression=np.zeros((n,14,140)),extra=np.zeros((n,14,28)),fixed_friction=np.zeros((n,14)),
        rotor=np.zeros((n,14)),current_torque=np.zeros((n,14)),moving=np.ones((n,14),bool),static_observations=raw)
    conditions=dict(split='train',center_id='c0',reference_id='r0',acquisition_batch_id='b0',repeat_id=0,active_arm_group='both')
    row=dict(problem=p,meta=dict(stage='inertia',conditions=conditions),coverage=dict(range_rad=[1]*14,speed_rms_rad_s=[1]*14,acceleration_rms_rad_s2=[1]*14))
    config=dict(p0=dict(acceptance=dict(torque_floor_nm=.2,dynamic_axis_nrmse=.05,stationary_excess_nm=.01)))
    scores,_=score_problems({'unit':row},None,engine.rigid_parameters(),np.zeros(28),np.full(14,.07),config)
    assert scores['unit']['maximum_normalized_rmse']==0 and not scores['unit']['passed']
