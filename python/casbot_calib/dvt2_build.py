"""Build-time Pinocchio reduction and source-local collision/viewer assets for N2."""
from hashlib import sha256
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import yaml
from . import _core
from .data import write_json
from .model_sources import dvt2_source
from .kinematics import geometry_manifest
from .trical_fixture import sphere_envelope


def transform(entry):
    t=np.asarray(entry['translation_m'],dtype=float);r=np.asarray(entry['rotation_vector_rad'],dtype=float)
    if t.shape!=(3,) or r.shape!=(3,) or not np.isfinite(np.r_[t,r]).all():raise ValueError('Finite metric translation and radian rotation vector required')
    return pin.SE3(Rotation.from_rotvec(r).as_matrix(),t)

def values(a):return ' '.join(format(float(x),'.17g') for x in np.asarray(a).ravel())


def assembly_xml(source, assembly):
    """Keep source files immutable; remove only the explicit ordinary-hand subtrees."""
    root = ET.parse(source.urdf).getroot()
    if assembly == 'source_unchanged':
        return root
    if assembly != 'bare_flange':
        raise ValueError('Unsupported source assembly: ' + assembly)
    removed = {'left_palm_base_link', 'right_palm_base_link'}
    joints = root.findall('joint')
    while True:
        descendants = {j.find('child').get('link') for j in joints
                       if j.find('parent').get('link') in removed}
        if descendants <= removed:
            break
        removed |= descendants
    for item in list(root):
        if (item.tag == 'link' and item.get('name') in removed) or (
                item.tag == 'joint' and item.find('child').get('link') in removed):
            root.remove(item)
    return root


def source_assembly_model(source, assembly):
    return pin.buildModelFromXML(ET.tostring(assembly_xml(source, assembly), encoding='unicode'))


def configuration(path):
    c=yaml.safe_load(Path(path).read_text());source=dvt2_source(c['source_variant'])
    if c['schema_version']!='casbot_dvt2_reduction_v1' or c['assembly'] not in {'source_unchanged','bare_flange'}:raise ValueError('Explicit supported source assembly required')
    if type(c['assembly_confirmed']) is not bool:raise ValueError('Explicit assembly confirmation status required')
    full=source_assembly_model(source,c['assembly']);active=list(source.calibration_joint_names)
    expected=set(full.names[1:])-set(active)
    if set(c['lock_positions_rad'])!=expected:raise ValueError('Every non-arm active source joint needs an explicit lock position')
    q=pin.neutral(full)
    for name,value in c['lock_positions_rad'].items():
        j=full.getJointId(name);idx=full.joints[j].idx_q
        if not np.isfinite(value) or not full.lowerPositionLimit[idx]<=value<=full.upperPositionLimit[idx]:raise ValueError('Lock position outside source limits: '+name)
        q[idx]=value
    source_xml=assembly_xml(source,c['assembly'])
    for joint in source_xml.findall('joint'):
        mimic=joint.find('mimic')
        if mimic is not None:
            a=full.joints[full.getJointId(joint.get('name'))].idx_q;b=full.joints[full.getJointId(mimic.get('joint'))].idx_q
            if not np.isclose(q[a],float(mimic.get('multiplier','1'))*q[b]+float(mimic.get('offset','0')),atol=1e-12,rtol=0):raise ValueError('Inconsistent locked mimic relationship')
    home=np.asarray(c['home_rad'],dtype=float)
    indices=[full.joints[full.getJointId(n)].idx_q for n in active]
    if home.shape!=(14,) or not np.isfinite(home).all():raise ValueError('Fourteen finite home values required')
    if np.any(home<full.lowerPositionLimit[indices]) or np.any(home>full.upperPositionLimit[indices]):raise ValueError('Home outside source limits')
    world=transform(c['world_from_source'])
    gravity=np.asarray(c['gravity_world_m_s2']);
    if gravity.shape!=(3,) or not np.isfinite(gravity).all() or np.linalg.norm(gravity)<=0:raise ValueError('Explicit gravity vector required')
    return c,source,full,q,world


def write_reduced(c,source,full,q,world,path):
    data=full.createData();pin.framesForwardKinematics(full,data,q)
    root=assembly_xml(source,c['assembly']);root.set('name',c['model_id']);active=set(source.calibration_joint_names)
    for item in list(root):
        if item.tag not in ('link','joint','material'):root.remove(item)
    for joint in root.findall('joint'):
        if joint.get('type')!='fixed' and joint.get('name') not in active:
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            placement=data.oMf[full.getFrameId(parent)].inverse()*data.oMf[full.getFrameId(child)]
            joint.set('type','fixed')
            for item in list(joint):
                if item.tag not in ('parent','child'):joint.remove(item)
            ET.SubElement(joint,'origin',xyz=values(placement.translation),rpy=values(Rotation.from_matrix(placement.rotation).as_euler('xyz')))
    for mesh in root.findall('.//mesh'):
        mesh.set('filename',Path(os.path.relpath(source.resolve_asset(mesh.get('filename')),Path(path).parent)).as_posix())
    ET.SubElement(root,'link',name='calib_world');mount=ET.SubElement(root,'joint',name='calib_world_mount',type='fixed')
    ET.SubElement(mount,'parent',link='calib_world');ET.SubElement(mount,'child',link='base_footprint')
    ET.SubElement(mount,'origin',xyz=values(world.translation),rpy=values(Rotation.from_matrix(world.rotation).as_euler('xyz')))
    for tool in c['tools']:
        if tool['name'] in {x.get('name') for x in root.findall('link')}:raise ValueError('Duplicate tool frame')
        placement=transform(tool);ET.SubElement(root,'link',name=tool['name']);joint=ET.SubElement(root,'joint',name=tool['name']+'_mount',type='fixed')
        ET.SubElement(joint,'parent',link=tool['parent_frame']);ET.SubElement(joint,'child',link=tool['name'])
        ET.SubElement(joint,'origin',xyz=values(placement.translation),rpy=values(Rotation.from_matrix(placement.rotation).as_euler('xyz')))
    ET.indent(root,space='  ');ET.ElementTree(root).write(path,encoding='unicode',xml_declaration=True)


def convex(points):
    v=np.unique(np.asarray(points),axis=0);h=ConvexHull(v);lookup=np.full(len(v),-1,dtype=np.int64);lookup[h.vertices]=np.arange(len(h.vertices))
    p=v[h.vertices];f=lookup[h.simplices].astype(np.int32)
    n=np.cross(p[f[:,1]]-p[f[:,0]],p[f[:,2]]-p[f[:,0]]);flip=np.sum(n*h.equations[:,:3],axis=1)<0;f[flip]=f[flip][:,[0,2,1]]
    violation=max(float(np.max(v[k:k+256]@h.equations[:,:3].T+h.equations[:,3])) for k in range(0,len(v),256))
    if violation>1e-9:raise ValueError('Convex envelope does not contain source vertices')
    return p,f,violation


def write_collision(c,source,urdf,out,urdf_digest):
    m=pin.buildModelFromUrdf(str(urdf));gm=pin.buildGeomFromUrdf(m,str(urdf),pin.GeometryType.COLLISION,package_dirs=[str(urdf.parent)])
    arrays={};rows=[]
    for i,g in enumerate(gm.geometryObjects):
        if hasattr(g.geometry,'vertices'):vertices=np.asarray(g.geometry.vertices())
        elif type(g.geometry).__name__=='Sphere':vertices=sphere_envelope('source_sphere',[0,0,0],g.geometry.radius).vertices
        else:raise ValueError('Unsupported source collision shape: '+type(g.geometry).__name__)
        p,f,error=convex(vertices@g.placement.rotation.T+g.placement.translation);key='g'+str(i)
        arrays[key+'_vertices']=p;arrays[key+'_triangles']=f
        rows.append(dict(name='robot_'+g.name,parent_joint_index=int(g.parentJoint)-1,array_key=key,source_link=m.frames[g.parentFrame].name,
            source_shape=type(g.geometry).__name__,source_vertices=len(vertices),hull_vertices=len(p),maximum_vertex_plane_violation_m=error))
    xml=ET.parse(source.urdf).getroot();parents={j.find('child').get('link'):j.find('parent').get('link') for j in xml.findall('joint')}
    links={r['source_link'] for r in rows};adjacent=set()
    for link in links:
        parent=parents.get(link)
        while parent is not None and parent not in links:parent=parents.get(parent)
        if parent is not None:adjacent.add(frozenset((parent,link)))
    source_exclusions={frozenset((x.get('body1'),x.get('body2'))) for x in ET.parse(source.reference_mjcf).getroot().findall('.//exclude')}
    audited={frozenset(pair) for pair in c['audited_assembly_overlaps']}
    if not audited<=source_exclusions:raise ValueError('Audited overlap not present in this source MJCF')
    exclusions=[]
    for i,a in enumerate(rows):
        for b in rows[i+1:]:
            pair=frozenset((a['source_link'],b['source_link']))
            if a['parent_joint_index']==b['parent_joint_index']:continue
            if pair in adjacent or pair in audited:
                exclusions.append(dict(first=a['name'],second=b['name'],source_links=sorted(pair),
                    reason='nearest collision-bearing URDF parent assembly' if pair in adjacent else 'explicitly audited source wrist housing overlap; exact links only'))
    np.savez_compressed(out/'collision.npz',**arrays)
    meta=dict(schema_version='casbot_dvt2_collision_v1',urdf_sha256=urdf_digest,joint_names=list(m.names)[1:],geometries=rows,excluded_pairs=exclusions,
        array_sha256=sha256((out/'collision.npz').read_bytes()).hexdigest(),source_geometry_count=len(gm.geometryObjects),
        builder='Pinocchio geometry placement + SciPy/Qhull outer envelopes; source spheres circumscribed',
        exclusions_policy='Exact source-link adjacency and two checked source wrist housing pairs only; no blanket MJCF/SRDF suppression')
    write_json(out/'collision.json',meta)
    return m


def scene_config(c,engine):
    settings=c['scene']['motion_limits']
    scene=c['scene'];home=np.array(c['home_rad']);frame=engine.frame_pose(home,scene['fixture']['parent_frame'])
    f=scene['fixture'];center=frame[:3,3]+frame[:3,:3]@np.asarray(f['center_local_m'])
    obstacles=[dict(name='dvt2_waist_clamp_proxy',center=center.tolist(),rotation=frame[:3,:3].tolist(),half_extents=f['half_extents_m'],attached_to_base=False)]
    floor=float(scene['floor_surface_z_m']);obstacles.append(dict(name='dvt2_floor_proxy',center=[0,0,floor-.03],half_extents=[2,2,.03],attached_to_base=False))
    balls=[];visits=[]
    for side,index in [('left',0),('right',7)]:
        for k,pose in enumerate(scene['arm_target_poses_rad']):
            q=home.copy();q[index:index+7]=pose
            if side=='right':q[index+1]*=-1
            tcp=engine.frame_pose(q,side+'_trical_tcp');bid=f'{side}_ball_{k+1}'
            balls.append(dict(id=bid,center_m=tcp[:3,3].tolist(),stem_direction_world=tcp[:3,2].tolist()))
            visits.append(dict(id=bid,frame=side+'_trical_tcp',ball_id=bid,nominal_q_rad=q.tolist()))
    trical=dict(schema_version='casbot_dvt2_trical_scene_v1',scene_only=True,tools=c['tools'],fixture=dict(id=c['model_id']+'_six_ball_scene',frame='dvt2_artifact',world_translation_m=[0,0,0],world_rotation_vector_rad=[0,0,0],
        ball_radius_m=scene['ball_radius_m'],collision_envelope_samples=scene['ball_envelope_samples'],balls=balls),sensor=dict(face_radius=scene['tools_face_radius_m']),
        workflow=dict(**settings,home_rad=home.tolist(),collision_scene=dict(obstacles=obstacles),clearance_m=scene['clearance_m'],
            allowed_tip_stroke_m=scene['allowed_tip_stroke_m'],geometry_allowance_rad=scene['geometry_allowance_rad'],geometry_error_budget_m=0.,transition_s=5.,visits=visits))
    return dict(schema_version='casbot_dvt2_scene_v1',synthetic_only=True,fixture_geometry_measured=False,installation_measured=c['installation_measured'],
        coordinate_frame='calib_world; source root mounted by explicit world_from_source',obstacles=obstacles,floor_surface_z_m=floor),trical


def build_robot_assets(config_path,output):
    c,source,full,q,world=configuration(config_path)
    out=Path(output).resolve();out.mkdir(parents=True,exist_ok=False)
    locks=[i for i in range(1,full.njoints) if full.names[i] not in source.calibration_joint_names]
    authoritative=pin.buildReducedModel(full,locks,q)
    if list(authoritative.names)[1:]!=list(source.calibration_joint_names):raise ValueError('Reduced logical arm order mismatch')
    write_reduced(c,source,full,q,world,out/'model.urdf');digest=sha256((out/'model.urdf').read_bytes()).hexdigest()
    engine=_core.ArmDynamics.from_urdf(str(out/'model.urdf'));model=engine.model;model.gravity_world=np.asarray(c['gravity_world_m_s2']);engine=_core.ArmDynamics(model)
    write_collision(c,source,out/'model.urdf',out,digest)
    return engine, dict(schema_version='casbot_dvt2_bundle_v1',model_id=c['model_id'],source_variant=c['source_variant'],
        source_urdf_sha256=source.urdf_sha256,urdf_sha256=digest,joint_names=list(source.calibration_joint_names),profile=c,
        geometry=geometry_manifest(engine,'dvt2_locked_v1'),default_model_changed=False,hardware_executable=False,
        tool_load_added=False,assembly_confirmed=c['assembly_confirmed'],
        friction_policy='Explicit zero friction in native rigid model; source URDF attributes are not calibrated values')


def build_bundle(config_path,output):
    engine,metadata=build_robot_assets(config_path,output)
    out=Path(output).resolve();c=metadata['profile'];scene,trical=scene_config(c,engine)
    metadata['scene']=scene
    write_json(out/'trical_scene.json',trical)
    from .dvt2_bundle import bundle_identity
    metadata['bundle_sha256']=bundle_identity(out,metadata)
    write_json(out/'model.json',metadata)
    from .dvt2_view import write_view
    write_view(out)
    metadata['viewer_sha256']=sha256((out/'scene.xml').read_bytes()).hexdigest()
    metadata['bundle_sha256']=bundle_identity(out,metadata)
    write_json(out/'model.json',metadata)
    return metadata


def main():
    import argparse
    parser=argparse.ArgumentParser(description='生成独立 DVT2 14轴候选包，不切换默认模型')
    parser.add_argument('--config',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    result=build_bundle(args.config,args.output)
    print(f"{result['model_id']}: {args.output}")


if __name__=='__main__':main()
