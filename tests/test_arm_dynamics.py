"""R1 independent analytic, physical, boundary, batch and geometry acceptance."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from importlib.resources import files
import json
import sys
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.arm_model import load_arm_model,default_arm_config,planar_two_link_config
from casbot_calib.arm_reference import planar_terms


@pytest.fixture
def planar():return _core.ArmDynamics(load_arm_model(planar_two_link_config()))


def test_pinocchio_backend_matches_independent_planar_oracle(planar):
    assert planar.backend_version().startswith('pinocchio-3.9.')
    rng=np.random.default_rng(20261020)
    for _ in range(40):
        q,v,a=rng.uniform(-1.,1.,size=(3,2));expected=planar_terms(q,v,a)
        actual=planar.components(q,v,a)
        for name in ('mass_matrix','inertia','velocity','gravity','total'):
            np.testing.assert_allclose(getattr(actual,name),expected[name],rtol=1e-10,atol=1e-12)
        energy=planar.energy(q,v)
        np.testing.assert_allclose([energy.kinetic,energy.potential],
                                   [expected['kinetic'],expected['potential']],rtol=1e-10,atol=1e-12)
        # Torque is computed by the independent analytic oracle, not by inverse().
        np.testing.assert_allclose(planar.forward(q,v,expected['total']),a,atol=1e-11)
    q,v,a=np.array([.4,-.7]),np.array([.8,-.5]),np.array([-.2,.6])
    assert np.linalg.norm(planar.velocity_bias(q,v))>.01
    m=planar.mass_matrix(q)
    assert np.linalg.norm((m-np.diag(np.diag(m)))@a)>.01
    assert np.linalg.norm(planar.inverse(q,v,a)-(np.diag(m)*a+planar.gravity(q)))>.01


def test_rotor_and_tool_inertia_are_applied_once():
    config=planar_two_link_config()
    for joint,rotor in zip(config['joints'],(.07,.03),strict=True):joint['rotor_inertia']=rotor
    config['payload']=dict(enabled=True,body=dict(mass=.35,com=[.08,0.,0.],inertia_com=np.diag([.004,.005,.006]).tolist()))
    engine=_core.ArmDynamics(load_arm_model(config));q=np.array([.4,-.7]);v=np.array([.8,-.5]);a=np.array([-.2,.6])
    oracle=planar_terms(q,v,a,rotor=(.07,.03),payload_mass=.35,payload_com_x=.08,payload_izz=.006)
    np.testing.assert_allclose(engine.mass_matrix(q),oracle['mass_matrix'],atol=1e-12)
    np.testing.assert_allclose(engine.inverse(q,v,a),oracle['total'],atol=1e-12)
    np.testing.assert_allclose(engine.forward(q,v,oracle['total']),a,atol=1e-11)
    e=engine.energy(q,v)
    np.testing.assert_allclose([e.kinetic,e.potential],[oracle['kinetic'],oracle['potential']],atol=1e-12)


def test_seven_joint_model_is_physical_and_has_velocity_coupling():
    engine=_core.ArmDynamics(load_arm_model());config=default_arm_config()
    q=np.array(config['reference_pose']);v=np.array([.3,-.5,.4,.6,-.2,.5,-.4]);a=np.linspace(-.4,.5,7)
    mass=engine.mass_matrix(q)
    np.testing.assert_allclose(mass,mass.T,atol=1e-13)
    assert np.linalg.eigvalsh(mass).min()>1e-6
    np.testing.assert_allclose(engine.velocity_bias(q,np.zeros(7)),0.,atol=1e-13)
    np.testing.assert_allclose(engine.inverse(q,np.zeros(7),np.zeros(7)),engine.gravity(q),atol=1e-13)
    assert np.linalg.norm(mass-np.diag(np.diag(mass)))>.01
    assert np.linalg.norm(engine.velocity_bias(q,v))>.001
    terms=engine.components(q,v,a)
    np.testing.assert_allclose(engine.inverse(q,v,a),terms.total,atol=1e-12)
    np.testing.assert_allclose(engine.forward(q,v,terms.total),a,atol=1e-10)
    # Independent physical identities, including arbitrary fixed-base orientation.
    h=1e-6
    derivative=np.array([(engine.mass_matrix(q+np.eye(7)[k]*h)-engine.mass_matrix(q-np.eye(7)[k]*h))/(2*h) for k in range(7)])
    christoffel=np.einsum('kij,j,k->i',derivative,v,v)-.5*np.einsum('ijk,j,k->i',derivative,v,v)
    np.testing.assert_allclose(engine.velocity_bias(q,v),christoffel,atol=1e-8)
    potential_gradient=np.array([(engine.energy(q+np.eye(7)[k]*h,v).potential-engine.energy(q-np.eye(7)[k]*h,v).potential)/(2*h) for k in range(7)])
    np.testing.assert_allclose(engine.gravity(q),potential_gradient,atol=1e-8)
    np.testing.assert_allclose(engine.energy(q,v).kinetic,.5*v@mass@v,atol=1e-13)


def test_base_rotation_and_payload_coordinates_obey_world_gravity():
    from scipy.spatial.transform import Rotation
    config=default_arm_config();q=np.array(config['reference_pose']);v=np.arange(7)*.05
    original=_core.ArmDynamics(load_arm_model(config));r=Rotation.from_rotvec([.2,-.3,.4]).as_matrix()
    config['base']['rotation']=(r@np.array(config['base']['rotation'])).tolist()
    config['base']['translation']=(r@np.array(config['base']['translation'])).tolist()
    config['gravity_world']=(r@np.array(config['gravity_world'])).tolist()
    rotated=_core.ArmDynamics(load_arm_model(config))
    np.testing.assert_allclose(rotated.mass_matrix(q),original.mass_matrix(q),atol=1e-12)
    np.testing.assert_allclose(rotated.gravity(q),original.gravity(q),atol=1e-12)
    np.testing.assert_allclose(rotated.velocity_bias(q,v),original.velocity_bias(q,v),atol=1e-12)


def test_friction_is_dissipative_and_stribeck_is_separate_from_smoothing():
    config=planar_two_link_config()
    for joint in config['joints']:
        joint['friction']=dict(viscous=.02,coulomb=.1,static_friction=.2,stribeck_velocity=.05,smoothing_velocity=.001)
    engine=_core.ArmDynamics(load_arm_model(config))
    for speed in np.linspace(-.5,.5,101):
        velocity=np.array([speed,-speed]);f=engine.friction(velocity)
        assert np.all(f*velocity>=-1e-14)
        np.testing.assert_allclose(engine.friction(-velocity),-f,atol=1e-14)
    assert engine.friction(np.array([.01,.01]))[0]>engine.friction(np.array([.2,.2]))[0]
    np.testing.assert_array_equal(engine.friction(np.zeros(2)),np.zeros(2))


def test_batch_matches_scalars_and_model_snapshots_are_immutable(planar):
    rng=np.random.default_rng(19);q,v,a=rng.normal(size=(3,9,2));original=q.copy()
    output=planar.evaluate_batch(q,v,a)
    for i in range(9):
        single=planar.components(q[i],v[i],a[i])
        for key in output:np.testing.assert_allclose(output[key][i],getattr(single,key),atol=1e-12)
    np.testing.assert_array_equal(q,original)
    assert planar.evaluate_batch(np.empty((0,2)),np.empty((0,2)),np.empty((0,2)))['total'].shape==(0,2)
    model=load_arm_model(planar_two_link_config());engine=_core.ArmDynamics(model)
    model.gravity_world=np.zeros(3);snapshot=engine.model;snapshot.gravity_world=np.zeros(3)
    assert np.linalg.norm(engine.gravity(np.array([.4,-.7])))>1.
    with ThreadPoolExecutor(max_workers=4) as pool:
        values=list(pool.map(lambda _:engine.inverse(q[0],v[0],a[0]),range(16)))
    for x in values:np.testing.assert_allclose(x,values[0],atol=1e-14)


def test_default_fk_matches_complete_urdf_parser_reference_without_source_repo():
    reference=json.loads(files('casbot_calib').joinpath('assets/casbot_right_arm_fk_reference.json').read_text(encoding='utf-8'))
    config=default_arm_config();engine=_core.ArmDynamics(load_arm_model(config))
    assert reference['source_sha256']==config['provenance']['source_sha256']
    assert reference['joint_names']==config['joint_names']
    for sample in reference['samples']:
        np.testing.assert_allclose(np.array(engine.fk(np.array(sample['q']))),sample['world_poses'],atol=1e-12)
    assert len(config['provenance']['source_fixed_hand_links_replaced'])>1
    assert config['provenance']['hardware_validated'] is False


@pytest.mark.parametrize('mutate',[
    lambda c:c.update(model_version='unknown'),
    lambda c:c['units'].update(angle='deg'),
    lambda c:c['joint_names'].reverse(),
    lambda c:c['joints'][0].update(axis=[0,0,2]),
    lambda c:c['joints'][0]['origin'].update(rotation=np.diag([1,1,-1]).tolist()),
    lambda c:c['joints'][0]['body'].update(mass=-1.),
    lambda c:c['joints'][0]['body'].update(inertia_com=np.diag([.1,.1,1.]).tolist()),
    lambda c:c['joints'][0]['body'].update(inertia_com=np.diag([-1.,1.,1.]).tolist()),
    lambda c:c['joints'][0]['friction'].update(static_friction=-1.),
    lambda c:c['joints'][0].update(rotor_inertia=-.1),
    lambda c:c['joints'][1].update(name='joint_1'),
])
def test_reject_invalid_contracts(mutate):
    config=planar_two_link_config();mutate(config)
    with pytest.raises(ValueError):load_arm_model(config)


def test_reject_invalid_states_and_batch_shapes(planar,monkeypatch):
    monkeypatch.setitem(sys.modules,'mujoco',None)
    with pytest.raises(ValueError):planar.mass_matrix(np.zeros(3))
    with pytest.raises(ValueError):planar.gravity(np.array([np.nan,0.]))
    with pytest.raises(ValueError):planar.evaluate_batch(np.zeros((3,2)),np.zeros((4,2)),np.zeros((3,2)))
    with pytest.raises(ValueError):planar.evaluate_batch(np.zeros((3,2)),np.zeros((3,2)),np.full((3,2),np.inf))
    assert planar.gravity(np.zeros(2)).shape==(2,)


def test_report_runs_without_mujoco_and_without_runtime_python_pinocchio(tmp_path,monkeypatch):
    from casbot_calib.arm_report import run_arm_checks
    monkeypatch.setitem(sys.modules,'mujoco',None)
    monkeypatch.setitem(sys.modules,'pinocchio',None)
    folder=tmp_path/'r1'
    report=run_arm_checks(folder)
    assert report['passed'] and report['backend'].startswith('pinocchio-')
    assert report['scope'].endswith('no calibration') and report['synthetic_only']
    assert report['metrics']['maximum_velocity_bias_nm']>.001
    assert (folder/'mass_matrix.png').exists() and (folder/'torque_components.png').exists()
    assert len(list(folder.glob('*.npz')))==1


def test_geometry_scene_and_mapping_require_no_mesh(tmp_path):
    import xml.etree.ElementTree as ET
    from casbot_calib.arm_viewer import write_arm_scene
    config=default_arm_config();scene=tmp_path/'arm.xml';adapter=write_arm_scene(config,scene)
    root=ET.parse(scene).getroot()
    assert len(root.findall('.//joint'))==7 and not root.findall('.//mesh')
    assert tuple(m.name for m in adapter.mappings)==tuple(config['joint_names'])
    assert not root.findall('.//actuator')


def test_arm_binding_source_delegates_rigid_dynamics_to_pinocchio():
    # Source check is supplementary; independent physical oracles above remain authoritative.
    from pathlib import Path
    source=Path(__file__).parents[1]/'cpp/src/arm_dynamics.cpp'
    if not source.exists():pytest.skip('Source tree unavailable')
    text=source.read_text(encoding='utf-8')
    for api in ('rnea','aba','crba','computeGeneralizedGravity','nonLinearEffects','forwardKinematics'):
        assert f'pinocchio::{api}(' in text
    assert 'struct Motion' not in text and 'ArmDynamics::rnea' not in text


def test_failed_optional_render_keeps_core_report_and_model_probes(tmp_path,monkeypatch):
    from casbot_calib.arm_report import run_arm_checks
    monkeypatch.setitem(sys.modules,'mujoco',None)
    report=run_arm_checks(tmp_path/'no_viewer',render=True)
    assert not report['passed'] and report['checks']['analytic_two_link']
    assert not report['checks']['geometry_only_viewer_fk'] and 'error' in report['geometry']
    assert (tmp_path/'no_viewer/report.json').exists()
    assert (tmp_path/'no_viewer/model_probes.npz').exists()
