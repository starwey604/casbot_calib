"""Fixed-waist tree, native multi-axis physics and observation-only workflow."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import sys
import numpy as np
import pytest
import yaml
from casbot_calib import _core
from casbot_calib.dual_model import nominal_dual_arm,physical_model
from casbot_calib.dual_excitation import periodic_reference


def test_native_tree_without_python_pinocchio_and_cross_branch_independence(monkeypatch):
    monkeypatch.setitem(sys.modules,'pinocchio',None)
    engine,source=nominal_dual_arm();model=engine.model
    assert len(model.joints)==14 and model.model_version=='tree_rigid_v1'
    assert model.parents==[-1,0,1,2,3,4,5,-1,7,8,9,10,11,12]
    assert source['hardware_validated'] is False
    q=np.linspace(-.2,.3,14);v=np.linspace(.2,-.4,14);a=np.linspace(-.5,.5,14)
    mass=engine.mass_matrix(q);assert np.linalg.eigvalsh(mass).min()>0
    np.testing.assert_allclose(mass[:7,7:],0.,atol=1e-14)
    changed=q.copy();changed[:7]+=.3
    np.testing.assert_allclose(engine.gravity(changed)[7:],engine.gravity(q)[7:],atol=1e-12)
    np.testing.assert_allclose(engine.forward(q,v,engine.inverse(q,v,a)),a,atol=1e-10)
    np.testing.assert_allclose(engine.torque_regressor(q,v,a)@engine.rigid_parameters(),engine.inverse(q,v,a),atol=1e-12)
    assert len(engine.fk(q))==14
    with pytest.raises(ValueError,match='named frame'):engine.tool_jacobian(q)
    invalid=engine.model;invalid.parents=[-1]*13+[14]
    with pytest.raises(ValueError):_core.ArmDynamics(invalid)


def test_tree_matches_independent_complete_pinocchio_parser():
    pin=pytest.importorskip('pinocchio')
    from importlib.resources import files,as_file
    engine,_=nominal_dual_arm()
    with as_file(__import__('casbot_calib.model_context',fromlist=['load_context']).load_context().urdf) as path:
        model=pin.buildModelFromUrdf(str(path))
    data=model.createData();rng=np.random.default_rng(32)
    assert [j.name for j in engine.model.joints]==list(model.names)[1:]
    for _ in range(8):
        q,v,a=rng.uniform(-.4,.4,(3,14));mass=pin.crba(model,data,q).copy();mass=np.triu(mass)+np.triu(mass,1).T
        np.testing.assert_allclose(engine.mass_matrix(q),mass,atol=1e-12)
        np.testing.assert_allclose(engine.inverse(q,v,a),pin.rnea(model,data,q,v,a),atol=1e-12)


def test_multi_axis_current_response_and_causal_timestamps():
    engine,_=nominal_dual_arm();config=_core.ArmSimulationConfig();config.gain=np.linspace(1.5,2.5,14)
    config.current_tau=.002;config.position_noise=0.;config.current_noise=0.
    commands=np.zeros((1,14,5));commands[0,:,2]=np.linspace(.1,.5,14)
    obs,truth=_core.ArmSimulationRunner(engine,config).run(np.array([0],dtype=np.int64),commands,10_000_000,np.zeros(14),np.zeros(14))
    elapsed=np.maximum(truth['sample_ns']*1e-9-config.command_delay_ns*1e-9,0.)
    expected=(1-np.exp(-elapsed[:,None]/config.current_tau))*commands[0,:,2]/config.gain
    np.testing.assert_allclose(truth['current'],expected,atol=1e-9)
    np.testing.assert_array_equal(obs['receive_ns']-obs['sample_ns'],config.feedback_delay_ns)


def test_multi_axis_integrator_energy_convergence_and_repeatability():
    from casbot_calib.arm_model import load_arm_model,planar_two_link_config
    model=load_arm_model(planar_two_link_config());model.gravity_world=np.zeros(3);engine=_core.ArmDynamics(model)
    config=_core.ArmSimulationConfig();config.gain=np.full(2,2.);config.current_tau=0.;config.position_noise=0.;config.current_noise=0.
    empty=np.empty((0,2,5));times=np.empty(0,dtype=np.int64);q=np.array([.3,-.4]);v=np.array([.2,-.5])
    runner=_core.ArmSimulationRunner(engine,config);obs,truth=runner.run(times,empty,100_000_000,q,v)
    energies=np.array([engine.energy(qq,vv).kinetic for qq,vv in zip(truth['q'],truth['dq'],strict=True)])
    np.testing.assert_allclose(energies,energies[0],atol=1e-12)
    config.integration_ns=25_000
    _,fine=_core.ArmSimulationRunner(engine,config).run(times,empty,100_000_000,q,v)
    np.testing.assert_allclose(truth['q'],fine['q'],atol=1e-11)
    config.drop_probability=.1;config.repeat_probability=.2;config.position_noise=1e-6
    runner=_core.ArmSimulationRunner(engine,config);left,_=runner.run(times,empty,100_000_000,q,v);right,_=runner.run(times,empty,100_000_000,q,v)
    for key in left:np.testing.assert_array_equal(left[key],right[key])
    assert np.any(left['status']&2)
    with pytest.raises(ValueError):runner.run(times,empty,100_000_000,np.zeros(3),v)












