"""Fixed-waist tree, native multi-axis physics and observation-only workflow."""
from concurrent.futures import ThreadPoolExecutor
from importlib.resources import files
from scipy.spatial.transform import Rotation
import gc
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


# Test-only serial models for independent analytic and binding contracts.
UNITS=dict(length="m",mass="kg",inertia="kg m^2",angle="rad",torque="Nm",gravity="m/s^2")


def default_arm_config():
    return json.loads(files("casbot_calib").joinpath("assets/casbot_right_arm.json").read_text(encoding="utf-8"))


def _pose(data):
    pose=_core.ArmPose()
    pose.rotation=np.asarray(data["rotation"],dtype=float)
    pose.translation=np.asarray(data["translation"],dtype=float)
    return pose


def _body(data):
    body=_core.ArmBody();body.mass=data["mass"]
    body.com=np.asarray(data["com"],dtype=float)
    body.inertia_com=np.asarray(data["inertia_com"],dtype=float)
    return body


def load_arm_model(config=None):
    """Build only declared SI/versioned contracts. Engine copies and validates it."""
    if config is None:config=default_arm_config()
    elif isinstance(config,(str,Path)):config=json.loads(Path(config).read_text(encoding="utf-8"))
    if config.get("schema_version")!=1 or config.get("units")!=UNITS:
        raise ValueError("Expected arm schema 1 with explicit SI units")
    if config.get("model_version")!="serial_rigid_v1":
        raise ValueError("Unsupported arm dynamics model version")
    model=_core.ArmModel();model.model_id=config["robot_model_id"];model.model_version=config["model_version"]
    model.base=_pose(config["base"]);model.tool=_pose(config["tool"])
    model.gravity_world=np.asarray(config["gravity_world"],dtype=float)
    joints=[]
    for entry in config["joints"]:
        if entry.get("type")!="revolute":raise ValueError("Arm contract supports serial revolute joints only")
        joint=_core.ArmJoint();joint.name=entry["name"];joint.link_name=entry["link_name"]
        joint.origin=_pose(entry["origin"]);joint.axis=np.asarray(entry["axis"],dtype=float)
        joint.body=_body(entry["body"]);joint.rotor_inertia=entry["rotor_inertia"]
        for key in ("lower","upper","velocity_limit","effort_limit"):setattr(joint,key,entry["limits"][key])
        friction=_core.ArmFriction()
        for key in ("viscous","coulomb","static_friction","stribeck_velocity","smoothing_velocity"):
            setattr(friction,key,entry["friction"][key])
        joint.friction=friction;joints.append(joint)
    if config["joint_names"]!=[j.name for j in joints]:raise ValueError("Arm joint order does not match contract")
    model.joints=joints
    enabled=config["payload"]["enabled"]
    if not isinstance(enabled,bool):raise ValueError("Payload enable must be boolean")
    model.payload_enabled=enabled
    if model.payload_enabled:model.payload=_body(config["payload"]["body"])
    # Validate now, before callers can use FK/dynamics or accidentally repair inputs.
    _core.ArmDynamics(model)
    return model


def planar_two_link_config():
    """Independent analytic fixture convention: XY plane, gravity along -Y."""
    pose=lambda xyz:dict(translation=xyz,rotation=np.eye(3).tolist())
    config=dict(schema_version=1,model_version="serial_rigid_v1",robot_model_id="analytic_planar_2r_v1",
        units=UNITS.copy(),base=pose([0.,0.,0.]),tool=pose([.45,0.,0.]),
        gravity_world=[0.,-9.81,0.],joint_names=["joint_1","joint_2"],joints=[],payload=dict(enabled=False))
    for j,(length,mass,izz) in enumerate(((.7,1.3,.054),(.45,.8,.0135))):
        config["joints"].append(dict(name=f"joint_{j+1}",link_name=f"link_{j+1}",type="revolute",
            origin=pose([0. if j==0 else .7,0.,0.]),axis=[0.,0.,1.],rotor_inertia=0.,
            body=dict(mass=mass,com=[length/2,0.,0.],inertia_com=(np.eye(3)*izz).tolist()),
            limits=dict(lower=-np.pi,upper=np.pi,velocity_limit=10.,effort_limit=20.),
            friction=dict(viscous=0.,coulomb=0.,static_friction=0.,stribeck_velocity=.05,smoothing_velocity=.01)))
    return deepcopy(config)


@pytest.fixture
def planar():return _core.ArmDynamics(load_arm_model(planar_two_link_config()))


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


def test_arm_binding_source_delegates_rigid_dynamics_to_pinocchio():
    # Source check is supplementary; independent physical oracles above remain authoritative.
    from pathlib import Path
    source=Path(__file__).parents[1]/'cpp/src/arm_dynamics.cpp'
    if not source.exists():pytest.skip('Source tree unavailable')
    text=source.read_text(encoding='utf-8')
    for api in ('rnea','aba','crba','computeGeneralizedGravity','nonLinearEffects','forwardKinematics'):
        assert f'pinocchio::{api}(' in text
    assert 'struct Motion' not in text and 'ArmDynamics::rnea' not in text


def configured_engine(seven=False):
    config=default_arm_config() if seven else planar_two_link_config()
    for j,joint in enumerate(config['joints']):
        joint['rotor_inertia']=.01*(j+1)
        joint['friction']=dict(viscous=.02,coulomb=.08,static_friction=.15,
            stribeck_velocity=.05,smoothing_velocity=.01)
    config['payload']=dict(enabled=True,body=dict(mass=.35,com=[.08,0.,0.],
        inertia_com=np.diag([.004,.005,.006]).tolist()))
    return _core.ArmDynamics(load_arm_model(config))


def test_parameter_order_origin_inertia_and_defensive_copy():
    config=planar_two_link_config();body=config['joints'][0]['body']
    body.update(com=[.2,-.1,.15],inertia_com=[[.06,.002,-.003],[.002,.07,.004],[-.003,.004,.08]])
    engine=_core.ArmDynamics(load_arm_model(config))
    c=np.array(body['com']);m=body['mass']
    origin=np.array(body['inertia_com'])+m*(c@c*np.eye(3)-np.outer(c,c))
    expected=np.r_[m,m*c,origin[0,0],origin[0,1],origin[1,1],origin[0,2],origin[1,2],origin[2,2]]
    parameters=engine.rigid_parameters()
    np.testing.assert_allclose(parameters[:10],expected,atol=1e-14)
    assert engine.rigid_parameter_names()[:10]==[
        'joint_1.'+x for x in ('mass','hx','hy','hz','Ixx','Ixy','Iyy','Ixz','Iyz','Izz')]
    parameters[:]=0.
    np.testing.assert_allclose(engine.rigid_parameters()[:10],expected,atol=1e-14)


def test_regressor_depends_on_geometry_not_nominal_body_values():
    config=default_arm_config();changed=deepcopy(config)
    for j,joint in enumerate(changed['joints']):
        joint['body']['mass']*=1.2+.1*j
        joint['body']['com']=(np.array(joint['body']['com'])+[.01,-.02,.03]).tolist()
        joint['body']['inertia_com']=(np.array(joint['body']['inertia_com'])*1.3).tolist()
        joint['rotor_inertia']=.02
    changed['tool']['rotation']=Rotation.from_rotvec([.2,-.1,.3]).as_matrix().tolist()
    changed['payload']['body']['mass']*=1.4
    engine=_core.ArmDynamics(load_arm_model(config));other=_core.ArmDynamics(load_arm_model(changed))
    rng=np.random.default_rng(28);q,v,a=rng.uniform(-.4,.4,(3,7))
    y=engine.torque_regressor(q,v,a)
    np.testing.assert_allclose(y,other.torque_regressor(q,v,a),atol=1e-14)
    np.testing.assert_allclose(y@other.rigid_parameters()+other.rotor_torque(a)+other.friction(v),
        other.inverse(q,v,a),atol=1e-12)


def central_jacobian(function,x,h=1e-6):
    return np.column_stack([(function(x+axis*h)-function(x-axis*h))/(2*h) for axis in np.eye(len(x))])


@pytest.mark.parametrize('seven',[False,True])
def test_analytic_derivatives_match_full_frictional_functions(seven):
    engine=configured_engine(seven);n=len(engine.model.joints);runtime=engine.workspace()
    rng=np.random.default_rng(29)
    for speed in (0.,.01,.1):
        q=rng.uniform(-.4,.4,n);v=np.linspace(-speed,speed,n);a=rng.uniform(-.3,.3,n)
        torque=engine.inverse(q,v,a)
        inverse=runtime.inverse_derivatives(q,v,a)
        forward=runtime.forward_derivatives(q,v,torque)
        for actual,expected in (
            (inverse.dtau_dq,central_jacobian(lambda x:engine.inverse(x,v,a),q)),
            (inverse.dtau_dv,central_jacobian(lambda x:engine.inverse(q,x,a),v)),
            (inverse.dtau_da,engine.mass_matrix(q)),
            (forward.ddq_dq,central_jacobian(lambda x:engine.forward(x,v,torque),q)),
            (forward.ddq_dv,central_jacobian(lambda x:engine.forward(q,x,torque),v)),
            (forward.ddq_dtau,central_jacobian(lambda x:engine.forward(q,v,x),torque))):
            np.testing.assert_allclose(actual,expected,rtol=2e-7,atol=2e-6)
        np.testing.assert_allclose(forward.ddq_dtau@inverse.dtau_da,np.eye(n),atol=1e-12)
        np.testing.assert_allclose(forward.ddq_dq+forward.ddq_dtau@inverse.dtau_dq,0.,atol=1e-11)
        np.testing.assert_allclose(forward.ddq_dv+forward.ddq_dtau@inverse.dtau_dv,0.,atol=1e-11)


def test_planar_tcp_jacobian_has_world_linear_then_angular_rows():
    engine=_core.ArmDynamics(load_arm_model(planar_two_link_config()));q=np.array([.4,-.7])
    l1,l2=.7,.45;q1,q2=q
    expected=np.array([[-l1*np.sin(q1)-l2*np.sin(q1+q2),-l2*np.sin(q1+q2)],
        [l1*np.cos(q1)+l2*np.cos(q1+q2),l2*np.cos(q1+q2)],[0.,0.],[0.,0.],[0.,0.],[1.,1.]])
    np.testing.assert_allclose(engine.tool_jacobian(q),expected,atol=1e-13)


def test_tcp_jacobian_includes_tool_offset_base_rotation_and_angular_convention():
    config=default_arm_config();r=Rotation.from_rotvec([.2,-.3,.4]).as_matrix()
    config['base']['rotation']=r.tolist();config['base']['translation']=[.4,-.2,.1]
    config['tool']['translation']=[.1,-.03,.07]
    config['tool']['rotation']=Rotation.from_rotvec([-.1,.4,.2]).as_matrix().tolist()
    engine=_core.ArmDynamics(load_arm_model(config));q=np.linspace(-.3,.4,7)
    jacobian=engine.tool_jacobian(q);pose=engine.fk(q)[-1]
    linear=central_jacobian(lambda x:engine.fk(x)[-1][:3,3],q)
    angular=[];h=1e-6
    for axis in np.eye(7):
        dr=(engine.fk(q+h*axis)[-1][:3,:3]-engine.fk(q-h*axis)[-1][:3,:3])/(2*h)
        skew=dr@pose[:3,:3].T
        angular.append([skew[2,1],skew[0,2],skew[1,0]])
    np.testing.assert_allclose(jacobian,np.vstack([linear,np.array(angular).T]),atol=2e-9)


def test_workspace_survives_owner_and_interleaved_algorithms_do_not_leave_stale_data():
    engine=configured_engine(True);q=np.linspace(-.2,.3,7);v=np.linspace(-.1,.1,7);a=np.linspace(.2,-.3,7)
    torque=engine.inverse(q,v,a);expected=engine.components(q,v,a);parameters=engine.rigid_parameters()
    expected_j=engine.tool_jacobian(q);expected_fk=engine.fk(q);expected_e=engine.energy(q,v)
    runtime=engine.workspace();del engine;gc.collect()
    saved=runtime.mass_matrix(q).copy()
    for _ in range(4):
        runtime.forward_derivatives(-q,-v,torque)
        runtime.inverse_derivatives(q,v,a)
        np.testing.assert_allclose(runtime.forward(q,v,torque),a,atol=1e-11)
        actual=runtime.components(q,v,a)
        for key in ('mass_matrix','inertia','velocity','gravity','friction','total'):
            np.testing.assert_allclose(getattr(actual,key),getattr(expected,key),atol=1e-12)
        np.testing.assert_allclose(runtime.torque_regressor(q,v,a)@parameters+runtime.rotor_torque(a)+runtime.friction(v),torque,atol=1e-12)
        np.testing.assert_allclose(runtime.tool_jacobian(q),expected_j,atol=1e-13)
        np.testing.assert_allclose(runtime.fk(q),expected_fk,atol=1e-13)
        e=runtime.energy(q,v)
        np.testing.assert_allclose([e.kinetic,e.potential],[expected_e.kinetic,expected_e.potential],atol=1e-12)
        np.testing.assert_allclose(runtime.mass_matrix(q),saved,atol=1e-13)


def test_concurrent_batches_have_independent_workspaces():
    engine=configured_engine(True);rng=np.random.default_rng(31)
    inputs=[rng.uniform(-.3,.3,(3,32,7)) for _ in range(12)]
    def run(arrays):
        q,v,a=arrays
        return engine.evaluate_batch(q,v,a)['total'],engine.regressor_batch(q,v,a)['regressor']
    expected=[run(x) for x in inputs]
    with ThreadPoolExecutor(max_workers=4) as pool:actual=list(pool.map(run,inputs))
    for left,right in zip(expected,actual,strict=True):
        for x,y in zip(left,right,strict=True):np.testing.assert_allclose(x,y,atol=1e-13)


def test_new_boundaries_empty_noncontiguous_batches_and_invalid_states():
    engine=configured_engine();runtime=engine.workspace();zero=np.zeros(2)
    for obj in (engine,runtime):
        for fn,args in ((obj.torque_regressor,(zero,zero,np.full(2,np.nan))),
            (obj.inverse_derivatives,(zero,np.zeros(3),zero)),
            (obj.forward_derivatives,(zero,zero,np.full(2,np.inf))),
            (obj.tool_jacobian,(np.zeros(3),)),(obj.rotor_torque,(np.full(2,np.nan),))):
            with pytest.raises(ValueError):fn(*args)
    empty=np.empty((0,2));assert engine.regressor_batch(empty,empty,empty)['regressor'].shape==(0,2,20)
    data=np.arange(20.).reshape(5,4)[:,::2]
    np.testing.assert_allclose(engine.regressor_batch(data,data,data)['regressor'][0],engine.torque_regressor(data[0],data[0],data[0]))
    with pytest.raises(ValueError):engine.regressor_batch(data,np.zeros((4,2)),data)
    with pytest.raises(ValueError):engine.regressor_batch(data,data,np.full((5,2),np.nan))


def test_gravity_batch_matches_independent_planar_formula_with_strided_input():
    model=_core.ArmModel();model.model_id='batch_gravity_2r';model.gravity_world=np.array([0.,-9.81,0.]);joints=[]
    for k,(mass,com) in enumerate([(1.3,.35),(.8,.225)]):
        j=_core.ArmJoint();j.name=f'joint_{k}';j.link_name=f'link_{k}';pose=j.origin;pose.translation=np.array([0. if k==0 else .7,0.,0.]);j.origin=pose
        b=j.body;b.mass=mass;b.com=np.array([com,0.,0.]);j.body=b;joints.append(j)
    model.joints=joints;engine=_core.ArmDynamics(model)
    q=np.array([[.2,.4],[-.7,.9],[.8,-.3]])[::-1]
    expected=np.column_stack([9.81*((1.3*.35+.8*.7)*np.cos(q[:,0])+.8*.225*np.cos(q.sum(axis=1))),9.81*.8*.225*np.cos(q.sum(axis=1))])
    np.testing.assert_allclose(engine.gravity_batch(q),expected,atol=1e-13,rtol=0)
    with pytest.raises(ValueError):engine.gravity_batch(np.zeros((2,3)))
    q=q.copy();q[0,0]=np.nan
    with pytest.raises(ValueError):engine.gravity_batch(q)
