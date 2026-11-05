#!/usr/bin/env python3
"""Source-STL surface audit of the nearest tool/fixture pairs at contact poses."""
import argparse
from pathlib import Path
import numpy as np
import coal
from casbot_calib.dvt2_bundle import read
from casbot_calib.trical_assembly import asset_directory,asset_manifest,profile,ball_center
from casbot_calib.mesh_io import read_stl
from casbot_calib.data import write_json


def audit(bundle,output,poses=None):
    out=Path(output);out.mkdir(parents=True,exist_ok=False);assets=asset_directory(bundle);manifest=asset_manifest(assets);descriptors={r['name']:r for r in manifest['parts']};shapes={};reports=[]
    def shape(name):
        if name not in shapes:
            if name.startswith('ball:'):
                shapes[name]=coal.Sphere(.0127);return shapes[name]
            v,f=read_stl(assets/descriptors[name]['mesh']);points=coal.StdVec_Vec3s();faces=coal.StdVec_Triangle()
            for p in v:points.append(p)
            for face in f:faces.append(coal.Triangle(*map(int,face)))
            bvh=coal.BVHModelOBBRSS();bvh.beginModel(len(f),len(v));bvh.addSubModel(points,faces);bvh.endModel();shapes[name]=bvh
        return shapes[name]
    rows=read(asset_directory()/'seeds.json')['results'] if poses is None else poses
    for row in rows:
        side=row['arm'];p=profile(bundle,side,row['ball'],True);q=np.array(row['q_rad'] if 'q_rad' in row else row['contact'][0]['q']);fk=p.engine.fk(q)
        d=p.check(q)['distances'][0];order=np.argsort(d-p.pair_clearance_m);seen=set();results=[]
        for k in order:
            indices=p.pairs[k];names=[p.names[j] for j in indices];source_names=[n.split('__cell')[0] for n in names]
            if p.pair_clearance_m[k]!=.01:continue
            if not any(n.startswith(('fixed:','ball:')) for n in names) or not any(n.startswith(side+':') for n in names):continue
            if tuple(source_names) in seen:continue
            seen.add(tuple(source_names));poses_world=[]
            for index in indices:
                g=p.geometry[index];base=np.eye(4) if g.parent<0 else fk[g.parent]
                T=base@g.placement.matrix()
                if g.name.startswith('ball:'):T[:3,3]+=ball_center(g.name.split(':')[1])
                poses_world.append(coal.Transform3s(T[:3,:3],T[:3,3]))
            result=coal.DistanceResult();gap=coal.distance(shape(source_names[0]),poses_world[0],shape(source_names[1]),poses_world[1],coal.DistanceRequest(True,True),result)
            results.append(dict(pair=names,hull_distance_m=float(d[k]),required_clearance_m=float(p.pair_clearance_m[k]),source_surface_distance_m=float(gap),
                source_nearest_points_world_m=[result.getNearestPoint1().tolist(),result.getNearestPoint2().tolist()]))
            if len(results)==3:break
        reports.append(dict(arm=side,ball=row['ball'],q_rad=q.tolist(),pairs=results))
        print(side,row['ball'],[(r['pair'],r['hull_distance_m']*1000,r['source_surface_distance_m']*1000) for r in results],flush=True)
    result=dict(scope='source STL surface audit only; solid collision and policy unchanged',results=reports);write_json(out/'audit.json',result);return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',required=True);p.add_argument('--output',required=True);p.add_argument('--poses',type=Path);a=p.parse_args();audit(a.bundle,a.output,read(a.poses) if a.poses else None)
