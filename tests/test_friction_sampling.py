"""Independent 1-DOF constant-speed torque balance exposes sample-phase aliasing."""
import numpy as np
from casbot_calib import _core


def test_instantaneous_current_sampling_converges_before_viscous_acceptance():
    model=_core.ArmModel();model.model_id='sampling_oracle';model.gravity_world=np.zeros(3)
    joint=_core.ArmJoint();joint.name='axis';joint.link_name='body';body=joint.body;body.inertia_com=np.eye(3)*.1;joint.body=body
    friction=joint.friction;friction.viscous=.035;friction.coulomb=.04;friction.static_friction=.07;friction.stribeck_velocity=.002;joint.friction=friction
    model.joints=[joint];plant=_core.ArmDynamics(model);coefficients=[]
    for sample_ns in (1000000,250000,125000):
        targets=[];velocities=[]
        for speed in (.04,.08,.14):
            times=np.arange(0,2000000000,2000000,dtype=np.int64);commands=np.zeros((len(times),1,5))
            commands[:,0,0]=times*1e-9*speed;commands[:,0,1]=speed;commands[:,0,3]=150.;commands[:,0,4]=10.
            config=_core.ArmSimulationConfig();config.gain=np.ones(1);config.sample_period_ns=sample_ns;config.position_noise=0.;config.current_noise=0.;config.stiction_enabled=True
            obs,_=_core.ArmSimulationRunner(plant,config).run(times,commands,2000000000,np.zeros(1),np.zeros(1))
            valid=(obs['sample_ns']>=1000000000)&(obs['sample_ns']<2000000000)
            # The independent stationary torque balance is fc+b*v. No measured
            # acceleration, native regressor or plant evaluation truth is used.
            targets.append(obs['current'][valid,0].mean());velocities.append(obs['dq'][valid,0].mean())
        value=np.linalg.lstsq(np.column_stack([velocities,np.ones(3)]),targets,rcond=None)[0];coefficients.append(value)
    true=np.array([.035,.04]);error=abs(np.array(coefficients)-true)/true
    assert error[0,0]>.05  # A tiny same-phase prediction residual is insufficient.
    assert np.max(error[1])<.05 and np.max(error[2])<.015
    assert np.max(abs(coefficients[1]-coefficients[2])/true)<.05
