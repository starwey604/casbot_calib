"""Regression, analytic derivatives, TCP conventions and reusable Data contracts."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import gc
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from casbot_calib import _core
from casbot_calib.arm_model import load_arm_model, default_arm_config, planar_two_link_config
from casbot_calib.arm_reference import planar_terms


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


@pytest.mark.parametrize('seven',[False,True])
def test_regressor_body_parameters_plus_rotor_and_friction(seven):
    engine=configured_engine(seven);n=len(engine.model.joints)
    rng=np.random.default_rng(27);q,v,a=rng.uniform(-.4,.4,(3,16,n))
    result=engine.regressor_batch(q,v,a)
    assert result['regressor'].shape==(16,n,10*n)
    assert result['parameter_names']==engine.rigid_parameter_names()
    assert 'parameters' not in result  # no model-value side channel into an estimator
    predicted=result['regressor']@engine.rigid_parameters()+result['rotor_torque']+result['friction']
    np.testing.assert_allclose(predicted,engine.evaluate_batch(q,v,a)['total'],atol=1e-12)
    for i in range(16):
        np.testing.assert_allclose(result['regressor'][i],engine.torque_regressor(q[i],v[i],a[i]),atol=1e-14)
    if not seven:
        for i in range(16):
            expected=planar_terms(q[i],v[i],a[i],rotor=(.01,.02),payload_mass=.35,payload_com_x=.08,payload_izz=.006)
            np.testing.assert_allclose(predicted[i]-result['friction'][i],expected['total'],atol=1e-12)


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
