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
from casbot_calib.dual_identification import periodic_observations


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
    with as_file(files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')) as path:
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


def test_excitation_derivatives_and_fourier_acquisition_fit_with_gaps():
    t=np.arange(0.,16.,.002);center=np.zeros(14)
    q,v,a,jerk=periodic_reference(t,center);h=1e-5
    plus=periodic_reference(t+h,center);minus=periodic_reference(t-h,center)
    for actual,index in ((v,0),(a,1),(jerk,2)):
        np.testing.assert_allclose(actual,(plus[index]-minus[index])/(2*h),atol=1e-7)
    obs=dict(sample_ns=np.rint(t*1e9).astype(np.int64),q=q,dq=v,current=.1*q,status=np.zeros_like(q,dtype=int))
    obs['status'][::17]=2
    fitted=periodic_observations(obs,4.,2,24,512)
    expected=periodic_reference(fitted['phase_s'],center)
    for key,array in zip(('q','dq','ddq'),expected[:3],strict=True):np.testing.assert_allclose(fitted[key],array,atol=1e-10)
    with pytest.raises(ValueError):physical_model(nominal_dual_arm()[0],-1.,.1,.1)


@pytest.fixture(scope='module')
def dual_workflow(tmp_path_factory):
    from casbot_calib.dual_workflow import run_dual_workflow
    config=yaml.safe_load((Path(__file__).parents[1]/'configs/dual_arm_dynamics.yaml').read_text(encoding='utf-8'))
    config['motion'].update(periods=4,gravity_poses=4,candidate_seeds=[27,28])
    config['drive_assumptions'].update(integration_ns=100_000,position_noise=0.,current_noise=0.)
    folder=tmp_path_factory.mktemp('dual')/'session'
    result=run_dual_workflow(folder,config)
    return folder,result


def test_complete_dual_workflow_and_truth_independent_reidentification(dual_workflow,tmp_path,monkeypatch):
    from casbot_calib.dual_workflow import identify_dual_session
    folder,report=dual_workflow;assert report['passed']
    assert report['full_rigid_regressor']['columns']==140
    assert report['full_rigid_regressor']['rank']<140
    assert 'fixed nominal COM/shape' in report['stages']['dynamic']['parameterization']
    # Delete every truth artifact, then forbid accidental reads of evaluation paths.
    shutil.rmtree(folder/'evaluation')
    for path in folder.glob('*/evaluation_truth.npz'):path.unlink()
    original=Path.open
    def guarded(self,*args,**kwargs):
        assert 'evaluation' not in str(self),'Estimator tried to read simulator truth'
        return original(self,*args,**kwargs)
    monkeypatch.setattr(Path,'open',guarded)
    repeated,_=identify_dual_session(folder,tmp_path/'repeated')
    np.testing.assert_allclose(repeated['stages']['dynamic']['parameters'],report['stages']['dynamic']['parameters'],atol=0.)
    session=json.loads((folder/'session.json').read_text(encoding='utf-8'));assert 'mock' not in session
    session['drive_assumptions']['gain_status']='unanchored';(folder/'session.json').write_text(json.dumps(session),encoding='utf-8')
    with pytest.raises(ValueError,match='scale anchor'):identify_dual_session(folder,tmp_path/'unanchored')
    session['drive_assumptions']['gain_status']='synthetic_declared';(folder/'session.json').write_text(json.dumps(session),encoding='utf-8')


def test_dual_geometry_scene_and_read_only_render(dual_workflow,tmp_path,monkeypatch):
    import os,xml.etree.ElementTree as ET
    from casbot_calib.dual_presentation import write_dual_scene,render_dual_workflow
    engine,_=nominal_dual_arm();scene=tmp_path/'scene.xml';adapter=write_dual_scene(engine,scene)
    root=ET.parse(scene).getroot()
    assert len(root.findall('.//joint'))==14 and not root.findall('.//actuator') and not root.findall('.//mesh')
    assert len(adapter.mappings)==14
    if os.environ.get('CASBOT_TEST_RENDER')!='1':pytest.skip('Optional EGL render check')
    mj=pytest.importorskip('mujoco')
    def forbidden(*args,**kwargs):raise AssertionError('Presentation called a physics API')
    for name in ('mj_step','mj_forward','mj_inverse','mj_rne'):monkeypatch.setattr(mj,name,forbidden)
    monkeypatch.setattr(_core,'ArmSimulationRunner',forbidden)
    folder,_=dual_workflow
    output=render_dual_workflow(folder,tmp_path/'render',fps=5,clip_seconds=.2,video=False)
    result=json.loads((output/'presentation.json').read_text(encoding='utf-8'))
    assert result['passed'] and not result['truth_read'] and result['max_fk_error']<1e-12
    assert len(list(output.glob('*_stage.png')))==8


def test_one_arm_training_cannot_certify_all_fourteen_axes(dual_workflow,tmp_path):
    from casbot_calib.dual_workflow import identify_dual_session
    original,_=dual_workflow;session=tmp_path/'one_arm';session.mkdir()
    shutil.copyfile(original/'session.json',session/'session.json')
    if (original/'preflight.json').exists():
        proof=json.loads((original/'preflight.json').read_text(encoding='utf-8'))
        shutil.copyfile(original/'preflight.json',session/'preflight.json')
        for relative in proof['source_sha256']:
            target=session/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(original/relative,target)
    for name in ('gravity_train','friction_train','dynamic_train','dynamic_validation','left_only_validation','right_only_validation'):
        target=session/name;target.mkdir();source=original/('left_only_validation' if name=='dynamic_train' else name)
        for filename in ('observations.npz','metadata.json','observation_schema.json'):shutil.copyfile(source/filename,target/filename)
    report,_=identify_dual_session(session,tmp_path/'one_arm_fit')
    assert not report['passed'] and not report['checks']['all_axes_dynamically_excited']


def test_observation_joint_order_digest_and_integer_time_contract(tmp_path):
    from casbot_calib.dual_workflow import _save_run
    from casbot_calib.dual_identification import load_observations
    engine,_=nominal_dual_arm();names=[j.name for j in engine.model.joints]
    times=np.arange(3,dtype=np.int64)*1_000_000;zero=np.zeros((3,14))
    observations=dict(sample_ns=times,receive_ns=times+700_000,sequence=np.arange(3,dtype=np.int64),
        q=zero,dq=zero,current=zero,status=np.zeros((3,14),dtype=np.int32))
    folder=tmp_path/'record'
    _save_run(folder,observations,{},times,np.zeros((3,14,5)),(zero,zero,zero,zero),names,{'test_only':True})
    assert load_observations(folder,names)['q'].shape==(3,14)
    schema=json.loads((folder/'observation_schema.json').read_text(encoding='utf-8'))
    schema['joint_names'].reverse();(folder/'observation_schema.json').write_text(json.dumps(schema),encoding='utf-8')
    with pytest.raises(ValueError,match='joint order'):load_observations(folder,names)
    schema['joint_names'].reverse();(folder/'observation_schema.json').write_text(json.dumps(schema),encoding='utf-8')
    path=folder/'observations.npz';path.write_bytes(path.read_bytes()+b'tampered')
    with pytest.raises(ValueError,match='digest'):load_observations(folder,names)
