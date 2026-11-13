"""Explicit power events cut active torque, not physical gravity or state evolution."""
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.shm_data import URDF


def session():
    engine=_core.ArmDynamics.from_urdf(str(URDF))
    c=_core.ArmSimulationConfig();c.gain=np.ones(14)*2
    c.position_noise=0;c.current_noise=0;c.current_tau=0
    c.command_delay_ns=0;c.feedback_delay_ns=0;c.integration_ns=25000
    q=np.zeros(14);q[3]=-.5;q[10]=-.5
    return _core.ArmSimulationSession(engine,c,q,np.zeros(14)),q


def test_power_event_is_per_axis_and_pending_commands_do_not_reenable():
    s,q=session();fields=np.zeros((14,5));fields[:,0]=q;fields[0,2]=2
    s.submit(0,fields);obs,_=s.advance_to(1_000_000)
    assert obs['current'][-1,0]==pytest.approx(1)
    mask=np.ones(14,dtype=np.int32);mask[0]=0
    s.set_drive_enabled(s.time_ns,mask);s.submit(s.time_ns,fields)
    obs,_=s.advance_to(3_000_000)
    assert np.all(obs['current'][:,0]==0)
    assert np.linalg.norm(obs['q'][-1]-q)>0
    mask[0]=1;s.set_drive_enabled(s.time_ns,mask)
    obs,_=s.advance_to(4_000_000)
    assert obs['current'][-1,0]==pytest.approx(1)


def test_power_event_checks_time_shape_and_empty_observation_shape():
    s,_=session()
    with pytest.raises(ValueError):s.set_drive_enabled(1,np.ones(14,dtype=np.int32))
    with pytest.raises(ValueError):s.set_drive_enabled(0,np.ones(13,dtype=np.int32))
    with pytest.raises(ValueError):s.set_drive_enabled(0,np.ones(14,dtype=np.int32)*2)
    s.advance_to(0)
    obs,_=s.advance_to(10)
    assert obs['q'].shape==(0,14)
