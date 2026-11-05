#!/usr/bin/env python3
"""Export reviewed 1105_03 parts as metric STL, not a second robot/physics model.

Fast path consumes the existing review's extracted BREP mesh cache. Without it,
FreeCAD imports the archived STEP and extracts exactly the persisted selection.
This is an authoring command; installed calibration never imports FreeCAD.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys
import numpy as np

REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/'python'))
from casbot_calib.mesh_io import write_stl


def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def save(p,value):Path(p).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def working_gauge_shape(sh,tool):
    """The reviewed working stroke cut; preserve every housing face."""
    import FreeCAD as A,Part
    C=np.array(tool['cad_sphere_center_mm'])
    cyl=next(face.Surface for face in sh.Faces if type(face.Surface).__name__=='Cylinder' and abs(face.Surface.Radius-4.25)<.01)
    axis=np.array([cyl.Axis.x,cyl.Axis.y,cyl.Axis.z]);center=np.array([cyl.Center.x,cyl.Center.y,cyl.Center.z]);axis*=np.sign((center-C)@axis)
    rot=A.Rotation(A.Vector(0,0,1),A.Vector(*axis));box=Part.makeBox(1000,1000,1000)
    box.Placement=A.Placement(A.Vector(*C)+rot.multVec(A.Vector(-500,-500,17.7)),rot)
    removed=sh.cut(box);rv,_=removed.tessellate(.1)
    rel=np.array([[x.x,x.y,x.z] for x in rv])-C
    radial=np.linalg.norm(rel-np.outer(rel@axis,axis),axis=1)
    if radial.max()>=4.251:raise ValueError('Contact cut removes housing material')
    return sh.common(box),A.Placement(A.Vector(*C),rot)


def export_holes(parts,registration,output):
    R=np.array(registration['registration']['rotation']);t=np.array(registration['registration']['translation_m'])
    plate=next(p for p in parts if p['name']=='Part__Feature5164');pillar=next(p for p in parts if p['name']=='Part__Feature5376')
    points=[];levels=[]
    for surface in plate['surfaces']:
        if surface['kind']=='Cylinder' and abs(surface['axis'][2])>.999 and abs(surface['radius']-2.5)<1e-5:
            points.append(tuple(np.round((R@np.array(surface['center'])/1000+t)[:2],9)))
    for surface in pillar['surfaces']:
        if surface['kind']=='Cylinder' and abs(surface['axis'][2])<.1 and abs(surface['radius']-2.5)<1e-5:
            levels.append(round(float((R@np.array(surface['center'])/1000+t)[2]),9))
    points=sorted(set(points));box=np.array(plate['bbox']).reshape(2,3)/1000@R.T+t;pb=np.array(pillar['bbox']).reshape(2,3)/1000@R.T+t
    save(Path(output)/'table_holes.json',dict(schema_version='casbot_trical_table_holes_v1',source='1105_03.stp',source_plate=plate['name'],unit='m',
        world_xy_m=[list(p) for p in points],plate_bbox_world_m=[box.min(0).tolist(),box.max(0).tolist()],plate_surface_z_m=float(box.max(0)[2]),
        baseline_column_center_xy_m=((pb.min(0)+pb.max(0))/2)[:2].tolist(),column_half_extents_xy_m=((pb.max(0)-pb.min(0))/2)[:2].tolist(),
        x_pitch_m=float(np.median(np.diff(sorted(set(p[0] for p in points))))),
        mounting_assumption='Column centre reference aligned to one table hole; fasteners/adapter compatibility is not reviewed'))
    save(Path(output)/'pillar_holes.json',dict(schema_version='casbot_trical_pillar_holes_v1',source_part=pillar['name'],world_z_m=sorted(set(levels)),pitch_m=.01,unit='m'))


def native_parts(selection, step, registration, checkpoint=None):
    """Recreate the reviewed contact-state cuts directly from the CAD BREP."""
    import zipfile
    sys.path.append('/usr/lib/freecad/lib')
    import FreeCAD as A,Import,Part
    doc=None;archive=None
    if checkpoint is None:
        doc=A.newDocument('Trical110503');Import.insert(str(step),doc.Name);doc.recompute()
    else:archive=zipfile.ZipFile(checkpoint)
    tools=registration['tools'];body={p:row for row in tools for p in row['body_parts']}
    result=[];contact={};stems={}
    try:
        for row in selection:
            if archive is None:
                obj=doc.getObject(row['name'])
                if obj is None:raise ValueError('Missing selected CAD part: '+row['name'])
                sh=obj.Shape.copy()
            else:
                sh=Part.Shape();sh.importBrepFromString(archive.read(row['shape']).decode(),False)
            sh.Placement=A.Placement(A.Matrix(*row['global']))
            clone=dict(row);v,f=sh.tessellate(.4)
            clone.update(vertices=[[x.x,x.y,x.z] for x in v],triangles=f)
            if row['name'] in {'Part__Feature5164','Part__Feature5376'}:
                clone['surfaces']=[dict(kind='Cylinder',radius=face.Surface.Radius,
                    center=[face.Surface.Center.x,face.Surface.Center.y,face.Surface.Center.z],
                    axis=[face.Surface.Axis.x,face.Surface.Axis.y,face.Surface.Axis.z])
                    for face in sh.Faces if type(face.Surface).__name__=='Cylinder']
            result.append(clone)
            if row['name'] in body:
                working,_=working_gauge_shape(sh,body[row['name']]);v,f=working.tessellate(.4)
                contact[row['name']]=dict(vertices=[[x.x,x.y,x.z] for x in v],triangles=f)
            spheres=[face.Surface for face in sh.Faces if type(face.Surface).__name__=='Sphere' and abs(face.Surface.Radius-12.7)<1e-5]
            if spheres:
                stem=sh.cut(Part.makeSphere(12.7,spheres[0].Center))
                stem=Part.makeCompound([face for face in stem.Faces if type(face.Surface).__name__!='Sphere'])
                v,f=stem.tessellate(.35);stems[row['name']]=dict(vertices=[[x.x,x.y,x.z] for x in v],triangles=f)
    finally:
        if doc is not None:A.closeDocument(doc.Name)
        if archive is not None:archive.close()
    return result,contact,stems


def export(reference, output, cache=None, checkpoint=None):
    reference=Path(reference);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    (output/'meshes').mkdir();setup=read(reference/'setup.json');reg=read(reference/'registration.json')
    R=np.array(reg['registration']['rotation']);t=np.array(reg['registration']['translation_m'])
    if cache is None:
        parts,contact,stems=native_parts(read(reference/'selection.json'),reference/'1105_03.stp',reg,checkpoint)
    else:
        parts=read(Path(cache)/'active_parts.json')
        contact=read(Path(cache)/'contact_state_meshes.json');stems=read(Path(cache)/'stems.json')
    selection=[{k:v for k,v in row.items() if k not in {'vertices','triangles','surfaces'}} for row in parts]
    save(reference/'selection.json',selection)
    rows=[];groups=defaultdict(list);ball_parts={b['source_part']:b for b in setup['balls']}
    tools={x['arm']:x for x in reg['tools']}
    for p in parts:
        side=next((s for s,row in tools.items() if p['parent']==row['parent']),None)
        if side is not None:
            tool=tools[side]
            if p['name']==tool['reference_ball_part']:continue
            data=contact.get(p['name'],p);v=np.array(data['vertices'])/1000@R.T+t
            tip=p['name'] in tool['tip_parts']
            if tip:
                axis=np.array(tool['sensor_axes_cad'][tool['tip_parts'].index(p['name'])])
                v-=(R@axis)*tool['tip_extension_m']
            W=tool['world_tcp'];v=(v-np.array(W['translation_m']))@np.array(W['rotation'])
            role='tip' if tip else 'weight' if '200G' in p['label'] else 'mount' if ('6CD55170' in p['label'] or p['name'] in tool['bridge_parts']) else 'tool'
            groups[side+':'+p['name']].append((p,v,np.array(data['triangles']),dict(role=role,side=side)))
        elif p['name'] in ball_parts:
            data=stems[p['name']];v=np.array(data['vertices'])/1000@R.T+t
            groups['stem:'+ball_parts[p['name']]['id']].append((p,v,np.array(data['triangles']),dict(role='stem',ball=ball_parts[p['name']]['id'])))
        else:
            # One convex outer envelope per fixed physical assembly, never one
            # robot-wide/tool-wide hull. STL visuals retain the source holes.
            key='fixed:'+p['path'].split('/')[1]
            v=np.array(p['vertices'])/1000@R.T+t
            groups[key].append((p,v,np.array(p['triangles']),dict(role='fixed')))
    for i,(key,pieces) in enumerate(sorted(groups.items())):
        vs=[];fs=[];offset=0
        for p,v,f,role in pieces:vs.append(v);fs.append(f+offset);offset+=len(v)
        v=np.concatenate(vs);f=np.concatenate(fs);name=f'meshes/part_{i:03}.stl';write_stl(output/name,v,f)
        rows.append(dict(name=key,mesh=name,**pieces[0][3],source_parts=[p[0]['name'] for p in pieces],
            coordinate_frame='tool TCP' if 'side' in pieces[0][3] else 'calib_world',source_triangles=len(f)))
    sensor={}
    for side,row in tools.items():
        cad_tcp=R.T@np.array(row['world_tcp']['rotation'])
        sensor[side]=(np.array(row['sensor_axes_cad'])@cad_tcp).tolist()
    manifest=dict(schema_version='casbot_trical_cad_v1',source='references/cad/trical_1105_03/1105_03.stp',
        source_size_bytes=setup['source_size_bytes'],unit='m',mesh_format='binary STL / metric float32',
        cad_tessellation_deflection_m=.0004,parts=rows,setup=setup,axes_sensor=sensor,
        load_status='unknown total tool mass/COM; six 200G CAD labels per tool, not weighed',
        near_contact_policy=read(reference/'collision_policy.json'),reference_ball_removed=True,excluded_fixed_references=read(reference/'excluded_references.json'))
    # Strip local author paths; retain physical and numeric provenance.
    manifest['setup']={k:v for k,v in setup.items() if k not in {'source_step','robot_urdf','reference_photo'}}
    save(output/'manifest.json',manifest)
    export_holes(parts,reg,output)
    print('STL assets:',len(rows),'triangles:',sum(r['source_triangles'] for r in rows),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference',default=str(REPO/'references/cad/trical_1105_03'))
    p.add_argument('--review-cache');p.add_argument('--native-checkpoint');p.add_argument('--output',required=True)
    a=p.parse_args();export(a.reference,a.output,a.review_cache,a.native_checkpoint)
