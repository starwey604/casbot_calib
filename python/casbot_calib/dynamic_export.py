"""Identified rigid model -> aggregated inertial URDF -> independent reload checks.

Pinocchio owns spatial inertia conversion and fixed-body aggregation. URDF holds
rigid properties only; nonlinear friction and reflected armature stay explicit
in a runtime sidecar. Nothing here fits data or grants hardware permissions.
"""
import copy
import csv
from importlib.resources import files
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
from . import _core
from .data import write_json
from .dual_model import nominal_dual_arm
from .rigid_identification import model_from_parameters
from .artifact_io import sha256
from .urdf_resources import rebase_resources,canonical_resources

SCHEMA='casbot_dynamic_urdf_export_v1'
ATOL=1e-10
RTOL=1e-10


def _number(values):return ' '.join(format(float(x),'.17g') for x in np.asarray(values).ravel())


def write_inertial_urdf(source,engine,path):
    """Write one aggregate inertia per moving joint, preserving all geometry.

    Fixed descendants are already included in fitted joint-body parameters.
    Keeping their old inertias would count them twice. Fixed-root properties
    remain unchanged because fixed-base joint data cannot identify them.
    """
    source=Path(source);path=Path(path)
    if path.exists() or source.resolve()==path.resolve():raise FileExistsError('A new URDF output is required')
    root=ET.fromstring(source.read_bytes());parsed=pin.buildModelFromUrdf(str(source))
    names=[j.name for j in engine.model.joints]
    if list(parsed.names)[1:]!=names:raise ValueError('Source/candidate joint order differs')
    model=engine.model
    if model.payload_enabled:raise ValueError('Merge a separate payload into an explicit fitted aggregate before export')
    # Geometry is unchanged: do not silently combine a kinematic correction
    # with inertias expressed in another joint/frame gauge.
    native=_core.ArmDynamics.from_urdf(str(source));q=np.zeros(len(names))
    if native.model.parents!=model.parents or not np.allclose(native.model.base.matrix(),model.base.matrix(),rtol=0,atol=1e-12):
        raise ValueError('Candidate geometry differs from source base/tree')
    for original,candidate in zip(native.model.joints,model.joints):
        if not np.allclose(original.axis,candidate.axis,rtol=0,atol=1e-12) or any(
            getattr(original,key)!=getattr(candidate,key) for key in ('lower','upper','velocity_limit','effort_limit')):
            raise ValueError('Candidate geometry/constraints differ from source: '+candidate.name)
    if native.frame_names()!=engine.frame_names():raise ValueError('Source/candidate frame set differs')
    for frame in native.frame_names():
        if not np.allclose(native.frame_pose(q,frame),engine.frame_pose(q,frame),rtol=0,atol=1e-12):
            raise ValueError('Candidate geometry differs from source: '+frame)
    links={link.get('name'):link for link in root.findall('link')}
    children={joint.get('name'):joint.find('child').get('link') for joint in root.findall('joint') if joint.get('type')!='fixed'}
    removed=[];root_links=[]
    for name,link in links.items():
        frame=parsed.frames[parsed.getFrameId(name)]
        if frame.parentJoint==0:
            root_links.append(name);continue
        if link.find('inertial') is not None:
            link.remove(link.find('inertial'));removed.append(name)
    blocks=np.asarray(engine.rigid_parameters()).reshape(-1,10);mapping=[]
    for index,(name,theta) in enumerate(zip(names,blocks)):
        child=children[name];frame=parsed.frames[parsed.getFrameId(child)]
        # actInv rotates COM inertia and shifts its COM through the actual
        # joint->link transform; do not hand-code another parallel-axis model.
        body=frame.placement.actInv(pin.Inertia.FromDynamicParameters(theta))
        element=ET.SubElement(links[child],'inertial')
        ET.SubElement(element,'origin',xyz=_number(body.lever),rpy='0 0 0')
        ET.SubElement(element,'mass',value=_number([body.mass]))
        inertia=np.asarray(body.inertia)
        ET.SubElement(element,'inertia',**{key:format(float(inertia[i,j]),'.17g') for key,i,j in
            [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
        members=[link for link in links if parsed.frames[parsed.getFrameId(link)].parentJoint==index+1]
        mapping.append(dict(joint=name,inertial_link=child,aggregate_links=members,mass_kg=float(body.mass),
            com_link_m=np.asarray(body.lever).tolist(),inertia_com_link_kg_m2=inertia.tolist()))
    rebase_resources(root,source,path)
    ET.indent(root,space='  ');path.write_text(ET.tostring(root,encoding='unicode')+'\n',encoding='utf-8')
    return dict(bodies=mapping,fixed_root_links_unchanged=root_links,
        fixed_descendant_inertials_collapsed=[name for name in removed if name not in children.values()],
        geometry_policy='All links, joints, visual/collision and fixed frames retained; no mesh assets copied')


def _restore_runtime(engine,runtime):
    model=engine.model;model.gravity_world=np.asarray(runtime['gravity_world'],dtype=float);joints=model.joints
    for i,joint in enumerate(joints):
        joint.rotor_inertia=runtime['rotor_inertia'][i];friction=joint.friction
        for key,values in runtime['friction'].items():setattr(friction,key,values[i])
        joint.friction=friction
    model.joints=joints
    return _core.ArmDynamics(model)


def _without_inertials(root,source=None):
    result=copy.deepcopy(root)
    if source is not None:canonical_resources(result,source)
    for link in result.findall('link'):
        for item in link.findall('inertial'):link.remove(item)
    # Ignore indentation introduced by serialization, not model attributes.
    for item in result.iter():
        if item.text is not None and not item.text.strip():item.text=None
        if item.tail is not None and not item.tail.strip():item.tail=None
    return ET.tostring(result)


def check_reload(path,expected,source,*,samples=32,seed=20261029):
    """Direct Pinocchio URDF and native adapter vs frozen candidate, no truth."""
    path=Path(path);source=Path(source);plain=_core.ArmDynamics.from_urdf(str(path))
    runtime=dict(gravity_world=np.asarray(expected.model.gravity_world).tolist(),
        rotor_inertia=[j.rotor_inertia for j in expected.model.joints],
        friction={key:[getattr(j.friction,key) for j in expected.model.joints] for key in
            ('viscous','coulomb','static_friction','smoothing_velocity','stribeck_velocity')})
    loaded=_restore_runtime(plain,runtime);parsed=pin.buildModelFromUrdf(str(path));data=parsed.createData()
    parsed.gravity.linear=np.asarray(expected.model.gravity_world)
    names=[j.name for j in expected.model.joints];checks={};errors={}
    def compare(name,a,b):
        a=np.asarray(a);b=np.asarray(b)
        errors[name]=max(errors.get(name,0.),float(np.max(abs(a-b),initial=0)))
        checks[name]=checks.get(name,True) and bool(np.allclose(a,b,rtol=RTOL,atol=ATOL))
    checks['joint_order']=list(parsed.names)[1:]==names and [j.name for j in plain.model.joints]==names
    checks['geometry_xml_unchanged']=_without_inertials(ET.parse(path).getroot(),path)==_without_inertials(ET.parse(source).getroot(),source)
    compare('rigid_parameters',np.concatenate([x.toDynamicParameters() for x in parsed.inertias[1:]]),expected.rigid_parameters())
    for new,old in zip(loaded.model.joints,expected.model.joints):
        compare('mass_kg',[new.body.mass],[old.body.mass]);compare('com_m',new.body.com,old.body.com)
        compare('inertia_com_kg_m2',new.body.inertia_com,old.body.inertia_com)
    rng=np.random.default_rng(seed);bounds=np.array([[j.lower,j.upper] for j in expected.model.joints])
    qvalues=rng.uniform(bounds[:,0]+.1*(bounds[:,1]-bounds[:,0]),bounds[:,1]-.1*(bounds[:,1]-bounds[:,0]),(samples,len(names)))
    for q in qvalues:
        v=rng.uniform(-.6,.6,len(names));a=rng.uniform(-1.,1.,len(names))
        compare('gravity_nm',pin.computeGeneralizedGravity(parsed,data,q),expected.gravity(q))
        # Pinocchio's rigid M is compared before separately restored armature.
        compare('rigid_mass_matrix_kg_m2',pin.crba(parsed,data,q),expected.mass_matrix(q)-np.diag(runtime['rotor_inertia']))
        compare('velocity_bias_nm',pin.nonLinearEffects(parsed,data,q,v)-pin.computeGeneralizedGravity(parsed,data,q),expected.velocity_bias(q,v))
        compare('rigid_inverse_nm',pin.rnea(parsed,data,q,v,a),expected.inverse(q,v,a)-expected.friction(v)-expected.rotor_torque(a))
        compare('runtime_mass_matrix_kg_m2',loaded.mass_matrix(q),expected.mass_matrix(q))
        compare('runtime_inverse_nm',loaded.inverse(q,v,a),expected.inverse(q,v,a))
        for frame in expected.frame_names():compare('frame_pose',loaded.frame_pose(q,frame),expected.frame_pose(q,frame))
    original=pin.buildModelFromUrdf(str(source))
    compare('fixed_root_inertia',parsed.inertias[0].toDynamicParameters(),original.inertias[0].toDynamicParameters())
    return dict(passed=all(checks.values()),checks=checks,maximum_absolute_errors=errors,atol=ATOL,rtol=RTOL,
        samples=samples,seed=seed,backend='Pinocchio direct URDF plus native adapter',hardware_executable=False,
        scope='serialization/coordinate/runtime consistency only; not new parameter identification or physical validation')


