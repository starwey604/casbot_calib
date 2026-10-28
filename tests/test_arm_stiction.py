"""Static threshold, coupled sticking and dissipation, without inverse-fit truth."""
import numpy as np
from casbot_calib import _core
from casbot_calib.arm_model import load_arm_model,planar_two_link_config


def pendulum():
    m=_core.ArmModel();m.model_id='stiction_oracle';m.gravity_world=np.zeros(3)
    j=_core.ArmJoint();j.name='axis';j.link_name='body';b=j.body;b.mass=1.;b.inertia_com=np.eye(3)*.1;j.body=b
    f=j.friction;f.coulomb=.06;f.static_friction=.1;f.stribeck_velocity=.002;f.smoothing_velocity=.001;j.friction=f;m.joints=[j]
    return _core.ArmDynamics(m)


def run(plant,times,torque,end=1.,initial_v=None,step=50000):
    n=len(plant.model.joints);t=np.rint(np.asarray(times)*1e9).astype(np.int64);values=np.asarray(torque).reshape(len(t),n)
    c=_core.ArmSimulationConfig();c.gain=np.ones(n);c.command_delay_ns=0;c.current_tau=0.;c.position_noise=0.;c.current_noise=0.
    c.stiction_enabled=True;c.integration_ns=step
    commands=np.zeros((len(t),n,5));commands[:,:,2]=values
    return _core.ArmSimulationRunner(plant,c).run(t,commands,round(end*1e9),np.zeros(n),np.zeros(n) if initial_v is None else initial_v)


def test_subthreshold_stays_still_and_ramp_breaks_away_at_static_limit():
    model=pendulum();obs,_=run(model,[0.],[[.099]])
    assert np.max(abs(obs['q']))==0 and np.max(abs(obs['dq']))==0
    t=np.arange(0,1.,.002);ramp=t*.2;obs,_=run(model,t,ramp)
    moving=np.flatnonzero(abs(obs['dq'][:,0])>1e-5)
    assert len(moving)>0
    first=moving[0];assert .099<=obs['current'][first,0]<=.102
    assert obs['q'][-1,0]>.03


def test_dry_friction_dissipates_energy_and_stop_converges_with_step_halving():
    model=pendulum();a,_=run(model,[0.],[0.],initial_v=[.2]);b,_=run(model,[0.],[0.],initial_v=[.2],step=25000)
    energy=.05*a['dq'][:,0]**2
    assert np.max(np.diff(energy))<1e-12 and abs(a['dq'][-1,0])==0
    assert abs(a['q'][-1,0]-b['q'][-1,0])<1e-5
    # With almost all motion outside the very narrow Stribeck region, constant
    # Coulomb stopping distance m*v²/(2*Fc) is an independent close benchmark.
    assert abs(a['q'][-1,0]-.1*.2**2/(2*.06))<1e-4


def test_constraint_reaction_keeps_second_coupled_joint_stationary():
    m=load_arm_model(planar_two_link_config());m.gravity_world=np.zeros(3);joints=m.joints
    for j in joints:
        f=j.friction;f.coulomb=.2;f.static_friction=.4;f.stribeck_velocity=.002;j.friction=f
    m.joints=joints;model=_core.ArmDynamics(m)
    obs,_=run(model,[0.],[[.5,0.]],end=.02)
    assert obs['q'][-1,0]>0 and np.max(abs(obs['q'][:,1]))==0
    assert np.max(abs(obs['dq'][:,1]))==0


def test_fully_stuck_current_response_matches_independent_exponential():
    m=pendulum();c=_core.ArmSimulationConfig();c.gain=np.ones(1);c.stiction_enabled=True
    c.position_noise=0.;c.current_noise=0.;c.current_tau=.003;c.command_delay_ns=250000
    commands=np.zeros((1,1,5));commands[0,0,2]=.05
    obs,_=_core.ArmSimulationRunner(m,c).run(np.array([0],dtype=np.int64),commands,20000000,np.zeros(1),np.zeros(1))
    elapsed=np.maximum(obs['sample_ns']-250000,0)*1e-9
    np.testing.assert_allclose(obs['current'][:,0],.05*(1-np.exp(-elapsed/.003)),atol=1e-14)
    assert np.max(abs(obs['q']))==0


def test_coupled_zero_speed_releases_match_independent_box_qp():
    from itertools import product
    from scipy.optimize import minimize
    m=load_arm_model(planar_two_link_config());m.gravity_world=np.zeros(3);joints=m.joints
    for j in joints:
        f=j.friction;f.coulomb=.2;f.static_friction=.4;f.stribeck_velocity=.002;j.friction=f
    m.joints=joints;model=_core.ArmDynamics(m);mass=model.mass_matrix(np.zeros(2));inverse=np.linalg.inv(mass)
    c=_core.ArmSimulationConfig();c.gain=np.ones(2);c.stiction_enabled=True;c.current_tau=0.;c.command_delay_ns=0
    c.position_noise=0.;c.current_noise=0.;c.sample_period_ns=100;c.integration_ns=100
    c.stick_velocity_tolerance=0.  # Resolve the infinitesimal acceleration oracle.
    # Maximum dissipation at zero speed has an independent convex dual box QP:
    # min_f .5*(tau-f)' M^-1 (tau-f), |f| <= Fs. No native active-set logic used.
    for values in product([-.6,-.3,0.,.3,.6],repeat=2):
        torque=np.array(values);answer=minimize(lambda f:.5*(torque-f)@inverse@(torque-f),np.clip(torque,-.4,.4),
            jac=lambda f:inverse@(f-torque),bounds=[(-.4,.4)]*2,method='L-BFGS-B',options=dict(ftol=1e-15,gtol=1e-12))
        expected=inverse@(torque-answer.x);commands=np.zeros((1,2,5));commands[0,:,2]=torque
        obs,_=_core.ArmSimulationRunner(model,c).run(np.array([0],dtype=np.int64),commands,100,np.zeros(2),np.zeros(2))
        np.testing.assert_allclose(obs['dq'][-1]/1e-7,expected,atol=1e-5,rtol=1e-5)


def test_full_tree_stopping_does_not_inject_energy_through_mass_coupling():
    import yaml
    from casbot_calib.staged_collection import synthetic_plant
    with open('configs/staged_dual_arm.yaml') as stream:config=yaml.safe_load(stream)
    plant=synthetic_plant(config);m=plant.model;m.gravity_world=np.zeros(3);plant=_core.ArmDynamics(m)
    c=_core.ArmSimulationConfig();c.gain=np.ones(14);c.stiction_enabled=True;c.current_tau=0.;c.command_delay_ns=0
    c.position_noise=0.;c.current_noise=0.;c.sample_period_ns=c.integration_ns
    commands=np.zeros((1,14,5))
    obs,_=_core.ArmSimulationRunner(plant,c).run(np.array([0],dtype=np.int64),commands,200000000,np.array(config['home_rad']),np.linspace(-.2,.2,14))
    workspace=plant.workspace();energy=np.array([workspace.energy(q,v).kinetic for q,v in zip(obs['q'],obs['dq'])])
    assert energy[-1]<energy[0] and np.max(np.diff(energy))<1e-10
