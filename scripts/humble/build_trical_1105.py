#!/usr/bin/env python3
"""Build the N2 real-assembly candidate; do not change any default consumer."""
import argparse
from hashlib import sha256
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from casbot_calib.dvt2_bundle import read,bundle_identity
from casbot_calib.dvt2_build import convex,build_robot_assets
from casbot_calib.trical_assembly import asset_directory,asset_manifest
from casbot_calib.trical_fixture import sphere_envelope
from casbot_calib.mesh_io import read_stl
from casbot_calib.kinematics import digest
from casbot_calib.data import write_json

REPO=Path(__file__).resolve().parents[2]


def build_collision_assets(root):
    m=asset_manifest(root);arrays={};rows=[]
    inputs=[row for row in m['parts'] if row['role']!='ball']
    for b in m['setup']['balls']:
        inputs.append(dict(name='ball:'+b['id'],role='ball',ball=b['id']))
    for row in inputs:
        mesh_paths=row.get('collision_meshes',[row.get('mesh')])
        for cell,path in enumerate(mesh_paths):
            if row['role']=='ball':
                ball=next(b for b in m['setup']['balls'] if b['id']==row['ball'])
                center=np.array(ball['center_relative_waist_mm'])/1000+np.array(m['setup']['cad_to_robot']['world_waist_m'])
                sphere=sphere_envelope(row['name'],center,.0127,samples=512)
                v=np.asarray(sphere.vertices)+center;f=np.asarray(sphere.triangles);error=0.
            else:
                vertices,triangles=read_stl(root/path)
                if 'collision_cover' in row:
                    from scipy.spatial import ConvexHull,HalfspaceIntersection
                    hull=ConvexHull(vertices);planes=hull.equations.copy()
                    planes[:,3]-=row['collision_cover']['padding_m']
                    vertices=HalfspaceIntersection(planes,vertices[hull.vertices].mean(0)).intersections
                v,f,error=convex(vertices)
            key='p'+str(len(rows));arrays[key+'_v']=v;arrays[key+'_f']=f
            name=row['name'] if len(mesh_paths)==1 else row['name']+'__cell'+str(cell)
            rows.append(dict(name=name,source_name=row['name'],key=key,maximum_vertex_plane_violation_m=error,convex=True))
    # Runtime part descriptors include balls, but their analytic center/radius
    # remain in setup; no STEP parser runs while collecting or displaying.
    m['parts']=inputs
    write_json(root/'manifest.json',m)
    np.savez_compressed(root/'collision.npz',**arrays)
    write_json(root/'collision.json',dict(schema_version='casbot_trical_cad_collision_v1',
        manifest_sha256=digest(m),array_sha256=sha256((root/'collision.npz').read_bytes()).hexdigest(),parts=rows,
        policy='conservative outer convex envelopes; all source vertices retained in the envelope; selected ball circumscribed'))


def make_config(reference):
    c=yaml.safe_load((REPO/'configs/dvt2_model_candidate.yaml').read_text())
    setup=read(reference/'setup.json')
    c.update(model_id='casbot_hl12p1_dvt2_bare_flange_1105_03_v1',assembly='bare_flange',assembly_confirmed=True)
    c['home_rad']=np.deg2rad([0,70,0,-60,0,0,0,0,-70,0,-60,0,0,0]).tolist()
    c['tools']=[dict(name=side+'_trical_tcp',parent_frame=side+'_flange_frame',
        translation_m=setup['tool_flange_to_tcp'][side]['translation_m'],
        rotation_vector_rad=Rotation.from_matrix(setup['tool_flange_to_tcp'][side]['rotation']).as_rotvec().tolist()) for side in ['left','right']]
    # No synthetic six-ball/clamp configuration can leak into this case.
    del c['scene']
    c.update(tool_geometry_measured=False,tool_load_known=False,assembly_source='1105_03 STEP + reviewed left90/right0 tool clocking')
    return c


def build(config,output,assets=None):
    engine,m=build_robot_assets(config,output);out=Path(output)
    asset_root=asset_directory() if assets is None else Path(assets)
    if asset_root.resolve().parent!=asset_directory().resolve().parent:raise ValueError('Use a versioned packaged TriCal asset root')
    assets=asset_manifest(asset_root);m.update(scene=dict(schema_version='casbot_dvt2_scene_v1',obstacles=[dict(name='dvt2_floor_proxy',center=[0,0,-.03],half_extents=[2,2,.03],attached_to_base=False)],floor_surface_z_m=0.),
        trical_asset_directory=asset_root.name,trical_assets_sha256=digest(assets),trical_collision_sha256=digest(read(asset_root/'collision.json')),dynamics_scope='bare robot rigid dynamics only; physical TriCal total load unknown',
        tool_load_known=False,tool_load_added=False)
    write_json(out/'trical_scene.json',dict(schema_version='casbot_trical_1105_scene_v1',scene_only=True,
        assets=asset_root.name,active_tools=['left','right'],one_tool_at_a_time=True,
        axes_sensor=assets['axes_sensor'],balls=assets['setup']['balls'],
        installation_clock_delta_deg=dict(left=90,right=0),installation_status='reviewed CAD mounting, not field metrology'))
    m['bundle_sha256']=bundle_identity(out,m);write_json(out/'model.json',m)
    from casbot_calib.trical_assembly_view import write_views
    write_views(out)
    viewer_files=[out/'scene_left.xml',out/'scene_right.xml',*sorted((out/'visual_meshes').glob('*.stl'))]
    m['viewer_files_sha256']={p.relative_to(out).as_posix():sha256(p.read_bytes()).hexdigest() for p in viewer_files}
    m['bundle_sha256']=bundle_identity(out,m);write_json(out/'model.json',m)
    return m


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);p.add_argument('--prepare-assets',action='store_true');p.add_argument('--config',type=Path,default=REPO/'configs/dvt2_trical_1105_03.yaml');p.add_argument('--write-config',action='store_true')
    p.add_argument('--assets',type=Path);a=p.parse_args()
    if a.prepare_assets:build_collision_assets(a.assets or asset_directory())
    if a.write_config:
        c=make_config(REPO/'references/cad/trical_1105_03');a.config.write_text(yaml.safe_dump(c,sort_keys=False),encoding='utf-8')
    result=build(a.config,a.output,a.assets);print(result['model_id'])
