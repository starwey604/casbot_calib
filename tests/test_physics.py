import numpy as np
import pytest
from scipy.linalg import expm
from casbot_calib.device import simulate, default_plant
from casbot_calib.excitation import torque_commands
from casbot_calib import _core

def test_damped_response_matches_independent_matrix_exponential():
    plant={**default_plant(),"coulomb":0.,"initial_q":.2,"initial_v":-.1}
    obs,truth,_=simulate(plant,np.empty(0,dtype=np.int64),np.empty((0,6)),2_000_000_000)
    system=np.array([[0.,1.],[-plant["spring"]/plant["inertia"],-plant["damping"]/plant["inertia"]]])
    for n in range(0,len(truth["time_ns"]),73):
        expected=expm(system*truth["time_ns"][n]*1e-9)@np.array([.2,-.1])
        np.testing.assert_allclose([truth["q"][n],truth["dq"][n]],expected,atol=1e-11)
    assert obs["sample_ns"].dtype==np.int64
    assert np.all(np.diff(truth["energy"])<=1e-12)

def test_fractional_command_delivery_and_current_response():
    p={**default_plant(),"command_delay_ns":1_350_000,"current_tau":.002,"feedback_delay_ns":700_000}
    values=np.zeros((1,6));values[0,3]=1.
    o,t,_=simulate(p,np.array([0],dtype=np.int64),values,10_000_000)
    expected=.5*(1-np.exp(-np.maximum(t["time_ns"]*1e-9-.00135,0)/.002))
    np.testing.assert_allclose(t["current"],expected,atol=1e-9)
    np.testing.assert_array_equal(o["receive_ns"]-o["sample_ns"],700_000)
    assert not _core.DeviceCapabilities().independent_torque_sensor

def test_fixed_seed_packets_and_derived_torque():
    p={**default_plant(),"seed":19,"jitter_ns":200_000,"feedback_delay_ns":1_000_000,
       "drop_probability":.05,"repeat_probability":.05,"current_noise":.001}
    times,values=torque_commands(duration_s=1.)
    a,_,_=simulate(p,times,values,1_000_000_000)
    b,_,_=simulate(p,times,values,1_000_000_000)
    for name in a: np.testing.assert_array_equal(a[name],b[name])
    np.testing.assert_array_equal(a["estimated_torque"],2*a["current"])
    assert len(a["q"])<1001 and np.any(a["status"]&2)

def test_reject_invalid_interfaces():
    with pytest.raises(ValueError):
        simulate({**default_plant(),"inertia":-1},np.array([0]),np.zeros((1,6)),10_000_000)
    with pytest.raises(ValueError):
        simulate(default_plant(),np.array([1,0]),np.zeros((2,6)),10_000_000)
    with pytest.raises(ValueError):
        simulate(default_plant(),np.array([0]),np.zeros((1,7)),10_000_000)
    with pytest.raises(ValueError):
        simulate({"gear_teeth":17},np.array([0]),np.zeros((1,6)),10_000_000)

def test_mechanical_step_halving_converges():
    ts,values=torque_commands(duration_s=3.)
    _,coarse,_=simulate(default_plant(),ts,values,3_000_000_000)
    _,fine,_=simulate({**default_plant(),"integration_ns":25_000},ts,values,3_000_000_000)
    assert np.max(np.abs(coarse["q"]-fine["q"]))/np.max(np.abs(fine["q"]))<.01

def test_contact_reaction_conserves_or_dissipates_total_energy():
    p={**default_plant(),"contact_enabled":True,"initial_q":-.03,"initial_v":2.,
       "spring":0.,"coulomb":0.,"damping":0.,"string_damping":0.,
       "contact_damping":0.,"integration_ns":10_000}
    _,conservative,_=simulate(p,np.empty(0,dtype=np.int64),np.empty((0,6)),200_000_000)
    assert np.any(conservative["contact"])
    assert np.max(np.abs(conservative["energy"]/conservative["energy"][0]-1))<1e-5
    _,dissipative,_=simulate({**p,"contact_damping":.2},np.empty(0,dtype=np.int64),np.empty((0,6)),200_000_000)
    assert dissipative["energy"][-1]<dissipative["energy"][0]
    assert np.max(np.diff(dissipative["energy"]))<1e-8
