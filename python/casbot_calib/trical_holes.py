"""Offline hole-position experiments for the complete 1105_03 column assembly."""
import argparse
from copy import deepcopy
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.spatial.transform import Rotation
from .trical_assembly import profile,asset_directory,ball_center
from .dvt2_bundle import read,load_bundle
from .trical_motion import solve_position
from .trical_paths import transfer_candidates
from .kinematics import digest
from .data import write_json


def column_layout(hole_xy):
    grid=read(asset_directory()/'table_holes.json');holes=np.array(grid['world_xy_m']);xy=np.asarray(hole_xy)
    if xy.shape!=(2,) or not np.isfinite(xy).all() or np.min(np.linalg.norm(holes-xy,axis=1))>1e-7:
        raise ValueError('Select an actual extracted CAD table hole')
    low,high=np.array(grid['plate_bbox_world_m']);half=np.array(grid['column_half_extents_xy_m'])
    if np.any(xy-half<low[:2]) or np.any(xy+half>high[:2]):raise ValueError('Column footprint crosses table edge')
    baseline=np.array(grid['baseline_column_center_xy_m']);xs=sorted(set(holes[:,0]));ys=sorted(set(holes[:,1]))
    row=xs.index(float(xy[0]))+1;col=ys.index(float(xy[1]))+1
    return dict(schema_version='casbot_trical_column_layout_v1',id=f'hole_R{row:02}_C{col:02}',translation_m=[*(xy-baseline),0.],
        hole_center_world_xy_m=xy.tolist(),row_from_robot= row,column_from_robot_right=col,
        unit='m',source_plate='Part__Feature5164',mounting_assumption=grid['mounting_assumption'])


def candidate_layouts():
    holes=np.array(read(asset_directory()/'table_holes.json')['world_xy_m'])
    xs=sorted(set(holes[:,0]));ys=sorted(set(holes[:,1]));middle=int(np.argmin(abs(np.array(ys))))
    # Near/middle/far positions on three neighbouring lateral hole columns.
    for x in xs[1:7]:
        for y in ys[middle-1:middle+2]:yield column_layout([x,y])


def side_height_layouts():
    """Keep the whole column still; move C/D and their stems to existing side holes."""
    for c in [-.04,-.03,-.02,-.01,0.,.01,.02,.03,.04]:
        for d in [-.04,-.03,-.02,-.01,0.,.01,.02,.03,.04]:
            yield dict(schema_version='casbot_trical_column_layout_v1',id=f'side_C{c*1000:+04.0f}_D{d*1000:+04.0f}',
                translation_m=[0.,0.,0.],ball_offsets_m=dict(C=[0.,0.,c],D=[0.,0.,d]),
                unit='m',source_part='Part__Feature5376',mounting_assumption='Original side ball/stem at a different existing10mm-pitch side hole')


def platform_height_layouts(heights_mm=range(-150,151,25)):
    for value in heights_mm:
        yield dict(schema_version='casbot_trical_column_layout_v1',id=f'platform_Z{value:+04d}',
            translation_m=[0.,0.,0.],platform_lift_m=value/1000,unit='m',
            moving_assemblies=['table','table-attached mounts','column','all four balls and stems'],
            adjustment_status='Manual height candidate; real lift travel/locking not verified')


def search(folder,bundle,seeds,*,layouts=None,orientation_candidates=None,spin_angles_deg=(0.,),alternatives_per_case=1):
    """Screen nominal IK and complete local/transfer chords; never issue a pass certificate here."""
    if type(alternatives_per_case) is not int or alternatives_per_case<1:raise ValueError('Positive alternatives_per_case required')
    if not len(spin_angles_deg) or not np.isfinite(spin_angles_deg).all():raise ValueError('Finite spin candidates required')
    original=read(seeds)
    if any(row.get('withdrawal') for row in original['results']):raise ValueError('Hole screening requires straight seeds; replan release-then-turn references with trical_withdrawal')
    root=Path(folder);root.mkdir(parents=True,exist_ok=False);engine,metadata=load_bundle(bundle)
    home=np.array(metadata['profile']['home_rad']);all_layouts=list(candidate_layouts()) if layouts is None else layouts
    reports=[];started=perf_counter();extra=read(asset_directory(bundle)/'orientation_seeds.json') if orientation_candidates is None else orientation_candidates;pose_cache={}
    write_json(root/'protocol.json',dict(bundle_sha256=metadata['bundle_sha256'],source_seed_sha256=digest(original),extra_seed_sha256=digest(extra),spin_angles_deg=list(spin_angles_deg),alternatives_per_case=alternatives_per_case,screening_only=True))
    for layout in all_layouts:
        name='baseline' if layout is None else layout['id'];case_root=root/name;case_root.mkdir();results=[];accepted=[]
        for side in ['left','right']:
            free=profile(bundle,side,layout=layout)
            for row in original['results']:
                if row['arm']!=side:continue
                bid=row['ball'];frame=side+'_trical_tcp';target=ball_center(bid,layout)
                contact=profile(bundle,side,bid,True,layout=layout)
                options=[dict(row,rotation=(np.array(row['rotation'])@Rotation.from_rotvec([np.deg2rad(angle),0,0]).as_matrix()).tolist(),spin_deg=angle) for angle in spin_angles_deg]
                options += [r for r in extra if r['arm']==side and r['ball']==bid]
                best=None;diagnostics=[];feasible=[]
                for option_i,option in enumerate(options):
                    rotation=np.array(option['rotation']);seed=np.array(option['contact'][0]['q'])
                    key=(side,bid,tuple(target),tuple(rotation.ravel()),tuple(seed))
                    if key not in pose_cache:
                        try:q=solve_position(engine,frame,target,seed,rotation)
                        except ValueError as exc:
                            diagnostics.append(dict(option=option_i,reason='IK_rejected',detail=str(exc)));continue
                        length=float(row['contact'][-1]['retract_m']);local=[];qseed=q
                        offsets=np.linspace(0,length,max(6,round(length/.005)+1))
                        for offset in offsets:
                            try:qseed=solve_position(engine,frame,target+rotation[:,0]*offset,qseed,rotation)
                            except ValueError:break
                            local.append(dict(q=qseed.tolist(),retract_m=float(offset)))
                        if len(local)!=len(offsets):
                            diagnostics.append(dict(option=option_i,reason='local_IK_rejected'));continue
                        pose_cache[key]=(local,length)
                    local,length=pose_cache[key]
                    waypoints=np.array([r['q'] for r in local]);interior=(waypoints[:-1]+waypoints[1:])/2
                    ds=contact.check(np.vstack([waypoints,interior]))['distances']-contact.pair_clearance_m
                    i,j=np.unravel_index(np.argmin(ds),ds.shape)
                    if ds[i,j]<=0:
                        diagnostics.append(dict(option=option_i,reason='local_nominal_clearance_rejected',surplus_m=float(ds[i,j]),pair=[contact.names[k] for k in contact.pairs[j]]));continue
                    transfer=None
                    for method,path in transfer_candidates(home,waypoints[-1],side):
                        samples=[]
                        for a,b in zip(path[:-1],path[1:]):samples.extend(a+(b-a)*u for u in np.linspace(0,1,25))
                        distances=free.check(samples)['distances']
                        if distances.min()>.01:
                            transfer=path;break
                    if transfer is None:
                        diagnostics.append(dict(option=option_i,reason='nominal_transfer_search_failed'));continue
                    candidate=dict(arm=side,ball=bid,contact=local,rotation=rotation.tolist(),transfer=transfer.tolist(),
                        local_retract_m=length,orientation_option=option_i,method=method,
                        goal_position_error_m=float(np.linalg.norm(engine.frame_pose(waypoints[0],frame)[:3,3]-target)),
                        minimum_nominal_surplus_m=float(ds.min()),spin_deg=option.get('spin_deg'),
                        robust_ordinary_slack_m=float(np.min(ds[:,contact.pair_clearance_m==.01]-(contact.radii@np.full(14,.00105))[contact.pair_clearance_m==.01])))
                    feasible.append(candidate)
                    if len(feasible)>=alternatives_per_case:break
                if feasible:
                    best=max(feasible,key=lambda r:r['robust_ordinary_slack_m']);best['alternatives']=[x for x in feasible if x is not best]
                    accepted.append(best)
                results.append(dict(arm=side,ball=bid,nominal_candidate_found=best is not None,attempts=diagnostics))
        seed_rows=accepted+[dict(arm=r['arm'],ball=r['ball'],candidate_available=False,reason='nominal_search_failed') for r in results if not r['nominal_candidate_found']]
        write_json(case_root/'planning_seeds.json',dict(schema_version='casbot_trical_hole_seeds_v1',fixture_layout=layout,
            source_seed_sha256=digest(original),results=seed_rows))
        write_json(case_root/'seeds.json',dict(schema_version='casbot_trical_hole_seeds_v1',fixture_layout=layout,
            source_seed_sha256=digest(original),results=accepted))
        report=dict(layout=layout,name=name,nominal_combinations=len(accepted),required_combinations=8,cases=results,
            minimum_nominal_surplus_m=min((r['minimum_nominal_surplus_m'] for r in accepted),default=None),
            screening_only=True,continuous_gate_passed=False,hardware_executable=False,spin_angles_deg=list(spin_angles_deg))
        reports.append(report);write_json(case_root/'screen.json',report);write_json(root/'screening.json',reports)
        print(name,len(accepted),'/8 nominal; not certified',flush=True)
    write_json(root/'summary.json',dict(layouts=len(reports),wall_s=perf_counter()-started,
        full_nominal_layouts=[r['name'] for r in reports if r['nominal_combinations']==8],screening_only=True))
    return reports


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',required=True);p.add_argument('--seeds',type=Path)
    p.add_argument('--output',required=True);p.add_argument('--extra-orientations',type=Path);p.add_argument('--kind',choices=['column','side','height'],default='column')
    p.add_argument('--layouts',type=Path,help='JSON list of explicit layouts, overriding --kind')
    p.add_argument('--spin-deg',type=float,nargs='+',default=[0.]);p.add_argument('--alternatives-per-case',type=int,default=1);a=p.parse_args()
    search(a.output,a.bundle,a.seeds or asset_directory(a.bundle)/'seeds.json',
           layouts=read(a.layouts) if a.layouts else [None,*candidate_layouts()] if a.kind=='column' else list(side_height_layouts()) if a.kind=='side' else list(platform_height_layouts()),
           orientation_candidates=read(a.extra_orientations) if a.extra_orientations else None,
           spin_angles_deg=a.spin_deg,alternatives_per_case=a.alternatives_per_case)
