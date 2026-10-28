"""Feedback-delivery causality and actual-command replay for guarded ramps."""
import numpy as np
import pytest
from casbot_calib import _core


def experiment(maximum=.2,feedback_delay=700000):
    m=_core.ArmModel();m.model_id='guard_oracle';m.gravity_world=np.zeros(3)
    j=_core.ArmJoint();j.name='axis';j.link_name='body';b=j.body;b.inertia_com=np.eye(3)*.1;j.body=b
    f=j.friction;f.coulomb=.04;f.static_friction=.07;f.stribeck_velocity=.002;j.friction=f;m.joints=[j]
    plant=_core.ArmDynamics(m);c=_core.ArmSimulationConfig();c.gain=np.ones(1);c.stiction_enabled=True;c.position_noise=0.;c.current_noise=0.;c.feedback_delay_ns=feedback_delay
    times=np.arange(0,2000000000,2000000,dtype=np.int64);values=np.zeros((len(times),1,5));values[:,0,3:]=[20.,2.]
    probe=(times>=200000000)&(times<1200000000);values[probe,0,3:]=0.;values[probe,0,2]=(times[probe]-200000000)*1e-9*maximum
    guard=_core.ArmGuardConfig();guard.begin_ns=200000000;guard.end_ns=1200000000
    return plant,c,times,values,guard


def test_delivered_feedback_guard_stops_ramp_and_saved_commands_replay_exactly():
    plant,c,times,values,guard=experiment()
    obs,_,executed,events=_core.run_guarded_arm(plant,c,times,values,2000000000,np.zeros(1),np.zeros(1),guard)
    assert len(events)==1 and events[0]['reason']=='breakaway_confirmed' and not events[0]['fault']
    event=events[0]
    assert event['decision_ns']>=event['confirmed_sample_ns']+c.feedback_delay_ns
    assert event['confirmed_sample_ns']-event['first_sample_ns']>=guard.sustain_ns
    active=(times>=event['decision_ns'])&(times<guard.end_ns)
    assert np.all(executed[active,0,3]==guard.restore_kp) and np.all(executed[active,0,2]==0.)
    assert np.max(abs(obs['q']))<.005
    replay,_=_core.ArmSimulationRunner(plant,c).run(times,executed,2000000000,np.zeros(1),np.zeros(1))
    np.testing.assert_allclose(obs['q'],replay['q'],atol=1e-12,rtol=0)
    np.testing.assert_allclose(obs['current'],replay['current'],atol=1e-12,rtol=0)


def test_subthreshold_ramp_and_stale_feedback_are_explicit_failures():
    plant,c,times,values,guard=experiment(maximum=.03)
    obs,_,_,events=_core.run_guarded_arm(plant,c,times,values,2000000000,np.zeros(1),np.zeros(1),guard)
    assert events[0]['reason']=='not_detected' and events[0]['fault'] and np.max(abs(obs['q']))==0.
    c.feedback_delay_ns=100000000
    _,_,executed,events=_core.run_guarded_arm(plant,c,times,values,2000000000,np.zeros(1),np.zeros(1),guard)
    assert events[0]['reason']=='feedback_timeout' and events[0]['decision_ns']==guard.begin_ns
    assert executed[np.searchsorted(times,guard.begin_ns),0,3]>0


def test_motion_limit_precedes_confirmation_and_bad_policy_is_rejected():
    plant,c,times,values,guard=experiment();guard.sustain_ns=100000000;guard.maximum_displacement=.00021
    _,_,_,events=_core.run_guarded_arm(plant,c,times,values,2000000000,np.zeros(1),np.zeros(1),guard)
    assert events[0]['fault'] and events[0]['reason']=='position_limit'
    guard.maximum_displacement=guard.displacement_threshold
    with pytest.raises(ValueError):_core.run_guarded_arm(plant,c,times,values,2000000000,np.zeros(1),np.zeros(1),guard)
