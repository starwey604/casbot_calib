"""Accepted S4 model -> aggregated inertial URDF -> independent reload checks.

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
from .vendor_identity import sha256

SCHEMA='casbot_dynamic_urdf_export_v1'
ATOL=1e-10
RTOL=1e-10


def _read(path):return json.loads(Path(path).read_text())


def _manifest(root,required=()):
    root=Path(root)
    manifest=_read(root/'manifest.json')
    if not set(required)<=manifest.keys():raise ValueError('Required files missing from manifest')
    for name,digest in manifest.items():
        path=(root/name).resolve()
        if not path.is_relative_to(root.resolve()) or sha256(path)!=digest:
            raise ValueError('Sealed export/analysis input changed: '+name)


def _snapshot(analysis,stage):
    root=Path(analysis).resolve();_manifest(root,('analysis.json',stage+'/report.json'));summary=_read(root/'analysis.json')
    if summary.get('schema_version')!='casbot_vendor_s4_analysis_v1' or not summary['passed'] or not summary['stages'].get(stage,{}).get('passed'):
        raise ValueError('Accepted S4 analysis and requested stage required')
    required={'analysis.json',stage+'/report.json'}
    manifest=_read(root/'manifest.json')
    if not required<=manifest.keys():raise ValueError('S4 manifest does not seal requested stage')
    report=_read(root/stage/'report.json')
    if not report['passed']:raise ValueError('Requested stage was rejected')
    nominal,source=nominal_dual_arm();inputs={str(root/'manifest.json'):sha256(root/'manifest.json')}
    source_digest=summary.get('source_model_sha256')
    if source_digest is None and 'inertia/candidate.json' in manifest and summary['stages'].get('inertia',{}).get('passed'):
        source_digest=_read(root/'inertia/candidate.json')['source_sha256']
    if source_digest!=source['sha256']:raise ValueError('Accepted analysis does not bind the current nominal source geometry')
    if stage=='inertia':
        if 'inertia/candidate.json' not in manifest:raise ValueError('Candidate is not sealed by S4 analysis')
        from .staged_control import load_candidate
        bundle=_read(root/'inertia/candidate.json');engine=load_candidate(root/'inertia/candidate.json')
        fit=report['joint_refinement']
        if not np.array_equal(bundle['parameters'],fit['parameters']) or not np.array_equal(bundle['viscous']+bundle['coulomb'],fit['auxiliary_parameters']):
            raise ValueError('Candidate differs from accepted fit')
        if not np.array_equal(bundle['static_friction'],report['prior_stage_acceptance']['static_friction_frozen']):
            raise ValueError('Candidate static-friction freeze differs')
        inputs[str(root/'inertia/candidate.json')]=sha256(root/'inertia/candidate.json')
        friction_status='accepted staged model; runtime sidecar only'
    elif stage=='gravity':
        fit=report['fit'];engine=model_from_parameters(nominal,fit['parameters'])
        friction_status='not identified by gravity; disabled in this software model, does not replace SDK compensation'
    else:raise ValueError('Export stage must be gravity or inertia')
    model=engine.model
    runtime=dict(gravity_world=np.asarray(model.gravity_world).tolist(),rotor_inertia=[j.rotor_inertia for j in model.joints],
        friction={key:[getattr(j.friction,key) for j in model.joints] for key in
                  ('viscous','coulomb','static_friction','smoothing_velocity','stribeck_velocity')})
    metadata=dict(schema_version=SCHEMA,stage=stage,scope=summary['scope'],source_urdf_sha256=source['sha256'],
        model_version='tree_full_physical_inertia_v1',joint_names=[j.name for j in model.joints],
        parameters=engine.rigid_parameters().tolist(),runtime=runtime,friction_status=friction_status,
        identifiability=fit['identifiability'],fit_rank=fit['rank'],fit_columns=fit['columns'],
        individual_body_parameters_unique=bool(fit['unique_body_parameters']),source_inputs=inputs,
        inertia_estimation_stage_included=stage=='inertia',
        source_stage_report_sha256=sha256(root/stage/'report.json'),hardware_executable=False,
        semantics='Moving-joint aggregate body representative, not separately measured hand/link component masses',
        units=dict(mass='kg',com='m',inertia='kg m^2',torque='Nm'))
    return engine,metadata


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


def _expected(metadata):
    nominal,source=nominal_dual_arm()
    if metadata['schema_version']!=SCHEMA or metadata['source_urdf_sha256']!=source['sha256'] or metadata['joint_names']!=[j.name for j in nominal.model.joints]:
        raise ValueError('Export source/version/joint identity differs')
    rigid=model_from_parameters(nominal,metadata['parameters'])
    return _restore_runtime(rigid,metadata['runtime'])


def _without_inertials(root):
    result=copy.deepcopy(root)
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
    checks['geometry_xml_unchanged']=_without_inertials(ET.parse(path).getroot())==_without_inertials(ET.parse(source).getroot())
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


def export_dynamics_urdf(analysis,output,stage='inertia'):
    engine,metadata=_snapshot(analysis,stage);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    source=files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')
    metadata['aggregation']=write_inertial_urdf(source,engine,output/'candidate.urdf')
    metadata['urdf_sha256']=sha256(output/'candidate.urdf')
    write_json(output/'calibration.json',metadata)
    report=check_reload(output/'candidate.urdf',engine,source)
    report.update(stage=stage,source_urdf_sha256=metadata['source_urdf_sha256'],urdf_sha256=metadata['urdf_sha256'],
        mesh_assets_bundled=False,geometry_content_preserved=True,collision_revalidation_required=True,
        friction_and_armature_in_urdf=False,individual_body_parameters_unique=metadata['individual_body_parameters_unique'])
    with (output/'bodies.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.writer(f);writer.writerow(['joint','aggregate_inertial_link','mass_kg','com_x_m','com_y_m','com_z_m','status'])
        for row in metadata['aggregation']['bodies']:
            writer.writerow([row['joint'],row['inertial_link'],row['mass_kg'],*row['com_link_m'],'aggregate physical representative; individual components not uniquely identified'])
    write_json(output/'report.json',report)
    (output/'manifest.json').write_text(json.dumps({p.name:sha256(p) for p in sorted(output.iterdir()) if p.is_file()},indent=2)+'\n')
    return report


def load_exported_dynamics(folder):
    root=Path(folder);_manifest(root,('candidate.urdf','calibration.json','report.json','bodies.csv'));metadata=_read(root/'calibration.json')
    if not _read(root/'report.json')['passed']:raise ValueError('URDF reload verification rejected')
    expected=_expected(metadata)
    if sha256(root/'candidate.urdf')!=metadata['urdf_sha256']:raise ValueError('URDF differs from calibration sidecar')
    source=files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')
    if _without_inertials(ET.parse(root/'candidate.urdf').getroot())!=_without_inertials(ET.parse(source).getroot()):
        raise ValueError('Exported geometry differs from bound source')
    loaded=_restore_runtime(_core.ArmDynamics.from_urdf(str(root/'candidate.urdf')),metadata['runtime'])
    if not np.allclose(loaded.rigid_parameters(),expected.rigid_parameters(),rtol=RTOL,atol=ATOL):
        raise ValueError('Reloaded rigid properties differ from frozen candidate')
    return loaded,metadata


def verify_dynamics_export(folder,output):
    loaded,metadata=load_exported_dynamics(folder)
    source=files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')
    result=check_reload(Path(folder)/'candidate.urdf',_expected(metadata),source)
    result['export_manifest_sha256']=sha256(Path(folder)/'manifest.json')
    output=Path(output);output.mkdir(parents=True,exist_ok=False);write_json(output/'report.json',result)
    return result
