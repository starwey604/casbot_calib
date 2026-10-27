"""Analytic event-boundary and coupled-state checks for the shared Odeint backend."""
import numpy as np
from scipy.linalg import expm
from casbot_calib import _core
from casbot_calib.device import simulate,default_plant


def test_fractional_steps_and_delivery_match_piecewise_analytic_motion():
    plant=dict(default_plant(),spring=0.,coulomb=0.,damping=0.,current_tau=0.,initial_q=.2,initial_v=-.1,
        integration_ns=73_001,sample_period_ns=997_003,command_delay_ns=1_353_007,feedback_delay_ns=703_009)
    times=np.array([0,307_003,1_909_017,4_000_003,10_000_019,37_000_021,79_000_003],dtype=np.int64)
    commands=np.zeros((len(times),6));commands[:,3]=[.7,-.4,.6,-.5,.8,-.2,0.]
    observations,truth,metadata=simulate(plant,times,commands,200_000_019)
    t=truth['time_ns']*1e-9;arrivals=(times+plant['command_delay_ns'])*1e-9
    elapsed=np.maximum(t[:,None]-arrivals,0.)
    increments=np.diff(np.r_[0.,commands[:,3]])
    expected_q=plant['initial_q']+plant['initial_v']*t+(elapsed**2)@increments/(2*plant['inertia'])
    expected_v=plant['initial_v']+elapsed@increments/plant['inertia']
    np.testing.assert_allclose(truth['q'],expected_q,atol=1e-13)
    np.testing.assert_allclose(truth['dq'],expected_v,atol=1e-13)
    np.testing.assert_array_equal(observations['sample_ns'],np.arange(len(t))*plant['sample_period_ns'])
    np.testing.assert_array_equal(observations['receive_ns']-observations['sample_ns'],plant['feedback_delay_ns'])
    assert metadata['integration_backend']==_core.SimulationRunner.integration_backend()
    assert metadata['integration_backend'].startswith('boost.odeint.rk4/')
    assert metadata['integration_step_ns']==73_001
    assert metadata['integration_schedule']=='int64_ns_event_truncated'


def test_command_at_sample_boundary_uses_old_command_for_preceding_step():
    plant=dict(default_plant(),spring=0.,coulomb=0.,damping=0.,current_tau=0.,integration_ns=73_001)
    commands=np.zeros((2,6));commands[:,3]=[.3,-.4]
    _,truth,_=simulate(plant,np.array([0,1_000_000],dtype=np.int64),commands,2_000_000)
    np.testing.assert_allclose(truth['q'][1],.3*.001**2/(2*plant['inertia']),atol=1e-16)
    np.testing.assert_allclose(truth['dq'][1],.3*.001/plant['inertia'],atol=1e-15)
    # State reached the boundary with old input; the new command is visible at the boundary.
    np.testing.assert_allclose(truth['torque_request'],[.3,-.4,-.4],atol=0.)
    np.testing.assert_allclose(truth['ddq'],np.array([.3,-.4,-.4])/plant['inertia'],atol=0.)


def test_fixed_double_inertia_matches_independent_linear_exponential():
    plant=dict(default_plant(),double_inertia=True,coulomb=0.,current_tau=0.,initial_q=.2,initial_v=-.1,
        motor_inertia=.02,coupling_spring=90.,coupling_damping=.25)
    j,jm=plant['inertia'],plant['motor_inertia'];k,b=plant['coupling_spring'],plant['coupling_damping']
    system=np.array([[0.,1.,0.,0.],[-(plant['spring']+k)/j,-(plant['damping']+b)/j,k/j,b/j],
        [0.,0.,0.,1.],[k/jm,b/jm,-k/jm,-b/jm]])
    _,truth,_=simulate(plant,np.empty(0,dtype=np.int64),np.empty((0,6)),300_000_000)
    initial=np.array([.2,-.1,.2,-.1])
    for index in range(0,len(truth['time_ns']),19):
        expected=expm(system*(truth['time_ns'][index]*1e-9))@initial
        np.testing.assert_allclose([truth['q'][index],truth['dq'][index],truth['motor_q'][index]],expected[:3],atol=1e-10)
