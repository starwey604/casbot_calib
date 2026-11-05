"""1105_03 physical assembly: one tool, shared artifact, native Coal profiles.

Only the assembly/geometry differs between cases. All planning, motion bounds,
telemetry and (later) collection use the existing calib interfaces.
"""
from pathlib import Path
from copy import copy
from hashlib import sha256
import numpy as np
from . import _core
from .dvt2_bundle import read,load_bundle,robot_profile
from .model_sources import model_directory,_resource
from .kinematics import digest
from .collision import CollisionProfile


def asset_directory(bundle=None):
    name='trical_1105_03' if bundle is None else read(Path(bundle)/'model.json').get('trical_asset_directory','trical_1105_03')
    if not isinstance(name,str) or name in {'.','..'} or Path(name).name!=name:raise ValueError('A packaged TriCal asset directory name is required')
    return model_directory()/name


def asset_manifest(root=None):
    root=asset_directory() if root is None else Path(root)
    m=read(root/'manifest.json')
    if m['schema_version']!='casbot_trical_cad_v1' or m['unit']!='m':raise ValueError('Unsupported metric TriCal CAD package')
    return m


def collision_assets(root=None):
    root=asset_directory() if root is None else Path(root);m=asset_manifest(root)
    cache=read(root/'collision.json');path=_resource(root/'collision.npz')
    if cache['manifest_sha256']!=digest(m) or sha256(path.read_bytes()).hexdigest()!=cache['array_sha256']:
        raise ValueError('TriCal CAD/collision identity mismatch')
    result={}
    with np.load(path,allow_pickle=False) as a:
        for row in cache['parts']:
            g=_core.CollisionGeometry();g.name=row['name'];g.vertices=a[row['key']+'_v'];g.triangles=a[row['key']+'_f']
            result[g.name]=g
    return m,cache,result


def profile(folder,active_arm,ball_id=None,contact=False,*,assets=None,layout=None):
    if active_arm not in {'left','right'}:raise ValueError('One active tool side required')
    if contact and ball_id not in {'A','B','C','D'}:raise ValueError('Explicit selected contact ball required')
    engine,metadata=load_bundle(folder);base=robot_profile(folder,environment=True)
    m,cache,shapes=collision_assets(asset_directory(folder) if assets is None else assets)
    shift=layout_shift(layout)
    if metadata['trical_assets_sha256']!=digest(m) or metadata['trical_collision_sha256']!=digest(cache):raise ValueError('Bundle/tool assembly mismatch')
    frames={f.name:f for f in engine.model.frames};tool_frame=frames[active_arm+'_trical_tcp']
    geometry=list(base.geometry);source_names={g.name:g.name for g in geometry}
    descriptors={r['name']:r for r in m['parts']}
    for item in cache['parts']:
        row=descriptors[item['source_name']]
        if row.get('side',active_arm)!=active_arm:continue
        g=shapes[item['name']]
        g.parent=tool_frame.parent if 'side' in row else -1
        if 'side' in row:g.placement=tool_frame.placement
        elif is_platform_part(row):
            pose=_core.ArmPose();pose.translation=part_shift(row,layout);g.placement=pose
        g.attached_to_base=False;geometry.append(g);source_names[g.name]=row['name']
    names=[g.name for g in geometry];indices={n:i for i,n in enumerate(names)}
    excluded=[(indices[x['first']],indices[x['second']]) for x in base.metadata['excluded_pairs']]
    data={**base.metadata,'model_id':engine.model.model_id,'active_tool':active_arm,'inactive_tool':None,
        'fixture_layout':layout,'assembly_manifest_sha256':digest(m),'geometry_sha256':digest(dict(robot=metadata['geometry']['geometry_sha256'],assets=cache,active_arm=active_arm,layout=layout)),
        'exact_contact_pairs':[],'near_contact_pairs':[],'tool_load_known':False,'probe_geometry_scope':'reviewed working-contact state, full free-plunger envelope not measured','assembly_policy':'both hands removed; one physical TriCal mounted at a time'}
    result=CollisionProfile(engine,geometry,excluded,data)
    policy=m['near_contact_policy']
    thresholds=np.full(len(result.pairs),policy['ordinary_clearance_m'])
    if contact:
        tips={row['name'] for row in m['parts'] if row.get('side')==active_arm and row['role']=='tip'}
        near_parts={active_arm+':'+p for p in policy['near_ball_parts'][active_arm]}
        target='ball:'+ball_id;found=[]
        for i,(a,b) in enumerate(result.pairs):
            first,second=names[a],names[b];source_first,source_second=source_names[first],source_names[second]
            if (first in tips and second==target) or (second in tips and first==target):
                thresholds[i]=-policy['maximum_tip_stroke_m'];found.append(dict(first=first,second=second,minimum_distance_m=-policy['maximum_tip_stroke_m']))
            elif ((source_first in near_parts and second==target) or (source_second in near_parts and first==target)
                  or (first in tips and second=='stem:'+ball_id) or (second in tips and first=='stem:'+ball_id)):
                thresholds[i]=policy['near_clearance_m']
                result.metadata['near_contact_pairs'].append(dict(first=first,second=second,minimum_distance_m=policy['near_clearance_m'],
                    reason='1105_03 near_policy_v1: coupling/gauge bodies to selected ball; three flat tips to selected neck; positive 2mm gap, no contact permitted'))
        if len(found)!=3:raise ValueError('Three exact tip/selected-ball pairs required')
        result.metadata['exact_contact_pairs']=found
    result.pair_clearance_m=thresholds
    result.metadata['pair_clearances_sha256']=digest(dict(pairs=result.pairs,thresholds=thresholds.tolist()))
    return result


def indicator_calibration(side,centered_readings_m):
    """Reuse the existing native model with CAD axes and caller-supplied zero."""
    from .kinematics import calibration
    m=asset_manifest()
    if side not in m['axes_sensor']:raise ValueError('Unknown tool mount')
    return calibration(dict(indicators=dict(axes_sensor=m['axes_sensor'][side],centered_readings_m=centered_readings_m)))


def released_profile(contact,tip_clearance_m=.0005):
    """Same native geometry; after release the three tips may not recontact."""
    if not np.isfinite(tip_clearance_m) or tip_clearance_m<=0:raise ValueError('Positive released-tip clearance required')
    indices=np.flatnonzero(contact.pair_clearance_m<0)
    if len(indices)!=3:raise ValueError('A selected three-tip contact profile is required')
    result=copy(contact);result.pair_clearance_m=contact.pair_clearance_m.copy()
    result.pair_clearance_m[indices]=tip_clearance_m
    result.metadata={**contact.metadata,'phase':'released_no_recontact','released_tip_clearance_m':tip_clearance_m,
        'exact_contact_pairs':[dict(r,minimum_distance_m=tip_clearance_m) for r in contact.metadata['exact_contact_pairs']]}
    result.metadata['pair_clearances_sha256']=digest(dict(pairs=result.pairs,thresholds=result.pair_clearance_m.tolist()))
    return result


def layout_shift(layout):
    if layout is None:return np.zeros(3)
    if layout.get('schema_version')!='casbot_trical_column_layout_v1':
        raise ValueError('Explicit column layout schema required')
    shift=np.asarray(layout['translation_m'],dtype=float)
    if shift.shape!=(3,) or not np.isfinite(shift).all() or abs(shift[2])>1e-12:
        raise ValueError('Table-hole changes require a finite XY translation, unchanged Z')
    return shift


def is_column_part(row):
    return row['role'] in {'ball','stem'} or 'Part__Feature5376' in row.get('source_parts',[])


def ball_center(ball_id,layout=None):
    m=asset_manifest();b=next(b for b in m['setup']['balls'] if b['id']==ball_id)
    return (np.array(b['center_relative_waist_mm'])/1000
            +np.array(m['setup']['cad_to_robot']['world_waist_m'])+layout_shift(layout)+ball_offset(ball_id,layout)+np.array([0.,0.,platform_lift(layout)]))


def ball_offset(ball_id,layout):
    if layout is None:return np.zeros(3)
    entries=layout.get('ball_offsets_m',{})
    if set(entries)-set('ACD'):raise ValueError('Only existing side-ball hole rows can change height')
    offset=np.asarray(entries.get(ball_id,[0.,0.,0.]),dtype=float)
    if offset.shape!=(3,) or not np.isfinite(offset).all() or np.any(abs(offset[:2])>1e-12):
        raise ValueError('Side-hole relocation must be a finite vertical translation')
    if ball_id in entries:
        m=asset_manifest();ball=next(b for b in m['setup']['balls'] if b['id']==ball_id)
        z=ball['center_relative_waist_mm'][2]/1000+m['setup']['cad_to_robot']['world_waist_m'][2]+offset[2]
        levels=np.array(read(asset_directory()/'pillar_holes.json')['world_z_m'])
        if np.min(abs(levels-z))>1e-7:raise ValueError('Side ball must use an existing CAD pillar hole')
    return offset


def part_shift(row,layout):
    shift=layout_shift(layout) if is_column_part(row) else np.zeros(3)
    if is_platform_part(row):shift=shift+np.array([0.,0.,platform_lift(layout)])
    if row['role'] in {'ball','stem'}:shift=shift+ball_offset(row['ball'],layout)
    return shift


def is_platform_part(row):
    return is_column_part(row) or row['name'] in {
        'fixed:300X500X130001','fixed:3000001_ASM','fixed:3000001_ASM001'}


def platform_lift(layout):
    lift=0. if layout is None else float(layout.get('platform_lift_m',0.))
    if not np.isfinite(lift):raise ValueError('Finite platform height offset required')
    return lift


def static_platform_check(collision,clearance_m=.01):
    """Fixed robot vs manually repositioned fixture: conservative AABB proof.

    Native same-parent suppression is correct for rigid assemblies but omits
    these world-fixed pairs. A failed bound is unresolved, not proof of impact.
    """
    robot=[];environment=[]
    for g in collision.geometry:
        if g.parent>=0:continue
        points=np.asarray(g.vertices)@g.placement.rotation.T+g.placement.translation
        if g.attached_to_base:
            base=collision.engine.model.base
            points=points@base.rotation.T+base.translation
        row=(g.name,points.min(0),points.max(0))
        if g.name.startswith('robot_'):robot.append(row)
        elif g.name.startswith(('ball:','stem:')) or g.name in {'fixed:11050002','fixed:300X500X130001','fixed:3000001_ASM','fixed:3000001_ASM001'}:
            environment.append(row)
    nearest=None
    for a in robot:
        for b in environment:
            bound=float(np.linalg.norm(np.maximum(0,np.maximum(a[1]-b[2],b[1]-a[2]))))
            if nearest is None or bound<nearest['lower_bound_m']:
                nearest=dict(pair=[a[0],b[0]],lower_bound_m=bound)
    return dict(passed=nearest is not None and nearest['lower_bound_m']>clearance_m,
        method='world AABB separation lower bound; only locked-height fixture pose',nearest=nearest,
        clearance_m=clearance_m,unresolved_is_not_collision=True)
