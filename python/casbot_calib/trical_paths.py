"""Reference-only 1105_03 workflow, using shared Motion and Coal certificates.

No SDK, simulated plant, measurement fitting or MuJoCo mechanics are started.
Every command stream starts/ends at rest and belongs to one installed tool case.
"""
import argparse
from itertools import permutations
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .dvt2_bundle import read,load_bundle
from .trical_assembly import profile,released_profile,asset_directory,static_platform_check,ball_center
from .staged_motion import Motion,derivative_bounds
from .collision import certify_polyline,certify_reference_box,certify_chord_box_pruned
from .data import write_json
from .kinematics import digest

SETTINGS=dict(control_period_ns=2_000_000,maximum_velocity_rad_s=.35,
              maximum_acceleration_rad_s2=1.,maximum_jerk_rad_s3=8.,
              position_allowance_rad=.00105,joint_box_query_budget=30_000,joint_box_max_depth=20,certified_pair_pruning=True,clearance_m=.01,hold_s=.5,contact_hold_s=2.)


def timed_motion(name,points,labels,settings=None):
    settings=SETTINGS if settings is None else settings
    points=np.asarray(points);motion=Motion(name,'trical_1105_reference',points[0])
    period=settings['control_period_ns']/1e9
    limits=np.array([settings['maximum_velocity_rad_s'],settings['maximum_acceleration_rad_s2'],settings['maximum_jerk_rad_s3']])
    for q,label in zip(points[1:],labels,strict=True):
        previous=motion.states[-1][0]
        if np.array_equal(previous,q):duration=settings['contact_hold_s'] if label=='contact_hold' else settings['hold_s']
        else:
            trial=Motion('duration','trical',previous).add(1.,q).polynomial()
            peaks=np.array([np.max(derivative_bounds(trial,k)) for k in (1,2,3)])
            duration=max(.25,peaks[0]/limits[0],np.sqrt(peaks[1]/limits[1]),np.cbrt(peaks[2]/limits[2]))*1.001
        duration=np.ceil(duration/period)*period
        motion.add(duration,q,label=label)
    return motion


def gate_chord(collision,start,end,settings=None,cache=None):
    """A rest-to-rest septic traverses exactly this chord, monotonically.

    Time scaling does not change its geometric image. A continuous polyline
    certificate therefore covers the shared Motion curve, including rest holds.
    """
    settings=SETTINGS if settings is None else settings
    start=np.asarray(start);end=np.asarray(end);allowance=np.full(14,settings['position_allowance_rad'])
    if cache is not None:
        endpoints=sorted([start.tobytes(),end.tobytes()])
        key=(collision.metadata['geometry_sha256'],collision.pair_clearance_m.tobytes(),*endpoints,tuple(sorted(settings.items())))
        if key in cache:return {**cache[key],'reused_in_same_planning_run':True}
        result=gate_chord(collision,start,end,settings);cache[key]=result;return result
    probes=collision.check(np.array([start,(start+end)/2,end]))['distances']
    thresholds=collision.pair_clearance_m
    if np.any(probes<=thresholds):
        i,j=np.unravel_index(np.argmin(probes-thresholds),probes.shape)
        return dict(passed=False,reason='nominal_chord_probe_rejected',minimum_sampled_distance_m=float(probes.min()),
                    minimum_surplus_m=float((probes-thresholds)[i,j]),pair=[collision.names[k] for k in collision.pairs[j]])
    if np.any(probes-collision.radii@allowance<=thresholds):
        if settings.get('certified_pair_pruning',False):
            gate=certify_chord_box_pruned(collision,start,end,allowance,settings['clearance_m'],settings['joint_box_query_budget'],max_joint_depth=settings.get('joint_box_max_depth',20),local_motion_bounds=settings.get('local_motion_bounds',False))
        else:
            gate=certify_reference_box(collision,lambda t:start+np.asarray(t)[:,None]*(end-start),
                1.,abs(end-start),allowance,settings['clearance_m'],max_queries=settings['joint_box_query_budget'],max_joint_depth=settings.get('joint_box_max_depth',20))
    else:
        gate=certify_polyline(collision,np.vstack([start,end]),clearance_m=settings['clearance_m'],
            block_samples=1,position_allowance=allowance,max_depth=14)
    gate['covers']='continuous chord and declared joint box; identical path image for C3 rest-to-rest Motion'
    return gate


def transfer_candidates(home,goal,side,review_path=None):
    if review_path is not None:
        path=np.asarray(review_path)
        if np.max(abs(path[-1]-goal))<1e-6:yield 'review_seed',path
    yield 'direct',np.array([home,goal])
    ix=np.arange(7) if side=='left' else np.arange(7,14)
    groups=[ix[:2],ix[2:4],ix[4:]]
    for order in permutations(range(3)):
        q=home.copy();points=[q.copy()]
        for group in order:q=q.copy();q[groups[group]]=goal[groups[group]];points.append(q.copy())
        yield 'staged_'+''.join(str(i) for i in order),np.array(points)


def withdrawal_rules(row,count):
    spec=row.get('withdrawal')
    if spec is None:return None
    if spec.get('schema_version')!='casbot_trical_release_then_turn_v1':raise ValueError('Unknown withdrawal phase rules')
    index=spec.get('release_index')
    if type(index) is not int or not 1<=index<count:raise ValueError('Release must follow a contact chord and precede the transfer')
    try:clearance=float(spec['minimum_released_tip_clearance_m']);angle=float(spec['contact_orientation_tolerance_rad'])
    except (KeyError,TypeError,ValueError) as exc:raise ValueError('Release clearance and angular tolerance are required') from exc
    if not np.isfinite([clearance,angle]).all() or clearance<=0 or not 0<angle<np.pi:raise ValueError('Positive release clearance and angular tolerance required')
    return spec


def gate_orientation(engine,frame,start,end,reference,tolerance,allowance,max_depth=12):
    """SO(3) triangle bound over a nominal joint chord and its joint box."""
    start=np.asarray(start);end=np.asarray(end);joint=next(f.parent for f in engine.model.frames if f.name==frame);ancestors=[]
    while joint>=0:ancestors.append(joint);joint=engine.model.parents[joint]
    box=float(np.sum(np.broadcast_to(allowance,start.shape)[ancestors]));travel=float(np.sum(abs(end-start)[ancestors]))
    pending=[(0.,1.,0)];maximum=sampled=0.;intervals=0
    while pending:
        a,b,depth=pending.pop();mid=(a+b)/2
        angle=float(Rotation.from_matrix(reference.T@engine.frame_pose(start+(end-start)*mid,frame)[:3,:3]).magnitude())
        sampled=max(sampled,angle);bound=angle+travel*(b-a)/2+box
        if bound<=tolerance:maximum=max(maximum,bound);intervals+=1;continue
        if depth>=max_depth:return dict(passed=False,reason='contact_orientation_envelope_unresolved',bound_rad=bound,tolerance_rad=tolerance)
        pending.extend([(a,mid,depth+1),(mid,b,depth+1)])
    return dict(passed=True,maximum_angle_bound_rad=maximum,maximum_sampled_angle_rad=sampled,tolerance_rad=tolerance,certified_intervals=intervals,
        method='SO(3) triangle inequality: midpoint angle + sum of ancestor joint travel and uncertainty')


def plan(folder,bundle,seeds,settings=None,*,layout=None,gate_cache=None):
    settings=dict(SETTINGS if settings is None else settings)
    root=Path(folder);root.mkdir(parents=True,exist_ok=False)
    engine,metadata=load_bundle(bundle);home=np.array(metadata['profile']['home_rad'])
    gate=lambda p,a,b:gate_chord(p,a,b,settings,gate_cache)
    reference=read(seeds);rows=reference['results'];reports=[];motions=[]
    seed_layout=reference.get('fixture_layout')
    if 'bundle_sha256' in reference:
        if reference['bundle_sha256']!=metadata['bundle_sha256']:raise ValueError('Reference seeds belong to another model bundle')
        if layout is not None and layout!=seed_layout:raise ValueError('Frozen reference/layout mismatch')
    if layout is None:layout=seed_layout
    elif seed_layout is not None and digest(layout)!=digest(seed_layout):raise ValueError('Seed/layout identity mismatch')
    if len(rows)!=8 or {(r['arm'],r['ball']) for r in rows}!={(s,b) for s in ('left','right') for b in 'ABCD'}:
        raise ValueError('Exactly left/right × A/B/C/D candidates required')
    write_json(root/'protocol.json',dict(schema_version='casbot_trical_1105_reference_v1',settings=settings,
        model_id=metadata['model_id'],bundle_sha256=metadata['bundle_sha256'],seed_sha256=digest(reference),fixture_layout=layout,
        tool_sequence=['left','manual_remount_at_rest','right'],hardware_executable=False,reference_only=True))
    for side in ['left','right']:
        free=profile(bundle,side,layout=layout)
        static_check=static_platform_check(free,settings['clearance_m'])
        for row in rows:
            if row['arm']!=side:continue
            bid=row['ball']
            if row.get('candidate_available',True) is False:
                reports.append(dict(arm=side,ball=bid,passed=False,reason='no_nominal_candidate',transfer_attempts=[],local_gates=[],measurement_hold_gate=dict(passed=False,reason='no_nominal_candidate')))
                write_json(root/'cases.json',reports);print(side,bid,'REJECT no nominal seed',flush=True);continue
            contact=profile(bundle,side,bid,True,layout=layout)
            local=np.array([x['q'] for x in row['contact']]);stage=local[-1];chosen=None;attempts=[]
            spec=withdrawal_rules(row,len(local));released=released_profile(contact,spec['minimum_released_tip_clearance_m']) if spec else None
            position_error=float(np.linalg.norm(engine.frame_pose(local[0],side+'_trical_tcp')[:3,3]-ball_center(bid,layout))) if spec else None
            contact_position_ok=spec is None or position_error<=5e-5
            static=gate(contact,local[0],local[0])
            release_gate=gate(released,local[spec['release_index']],local[spec['release_index']]) if spec else dict(passed=True,reason='legacy_reference_without_release_phase')
            local_gates=[];orientation_gates=[]
            if static['passed'] and release_gate['passed'] and contact_position_ok:
                R0=engine.frame_pose(local[0],side+'_trical_tcp')[:3,:3]
                for i,(a,b) in enumerate(zip(local[:-1],local[1:])):
                    proof=gate(released if spec and i>=spec['release_index'] else contact,a,b);local_gates.append(proof)
                    if not proof['passed']:break
                    if spec and i<spec['release_index']:
                        orientation=gate_orientation(engine,side+'_trical_tcp',a,b,R0,spec['contact_orientation_tolerance_rad'],settings['position_allowance_rad']);orientation_gates.append(orientation)
                        if not orientation['passed']:break
            local_ok=static['passed'] and release_gate['passed'] and contact_position_ok and len(local_gates)==len(local)-1 and all(g['passed'] for g in local_gates) and all(g['passed'] for g in orientation_gates)
            for method,path in transfer_candidates(home,stage,side,row.get('transfer')):
                if not local_ok:break
                # Cheap screening chooses routes; only continuous certificates admit them.
                screen=[]
                for a,b in zip(path[:-1],path[1:]):screen.extend(a+(b-a)*u for u in np.linspace(0,1,17))
                ds=free.check(screen)['distances']
                if ds.min()<=settings['clearance_m']:
                    attempts.append(dict(method=method,passed=False,reason='route_screen_rejected'));continue
                gates=[gate(free,a,b) for a,b in zip(path[:-1],path[1:])]
                attempts.append(dict(method=method,passed=all(g['passed'] for g in gates),gates=gates))
                if all(g['passed'] for g in gates):chosen=(method,path);break
            passed=static_check['passed'] and chosen is not None and local_ok
            report=dict(arm=side,ball=bid,passed=passed,static_platform_check=static_check,transfer_attempts=attempts,local_gates=local_gates,
                measurement_hold_gate=static,exact_contact_pairs=contact.metadata['exact_contact_pairs'],
                withdrawal_rules=spec,release_hold_gate=release_gate,contact_orientation_gates=orientation_gates,
                new_contact_position_error_m=position_error,
                near_contact_pairs=contact.metadata['near_contact_pairs'],collision_geometry_sha256=contact.metadata['geometry_sha256'],
                local_retract_m=float(row['contact'][-1]['retract_m']))
            if passed:
                method,path=chosen
                points=[home,home];labels=['initial_hold']
                for q in path[1:]:points.append(q);labels.append('approach')
                for j in range(len(local)-2,-1,-1):
                    points.append(local[j]);labels.append(('insert_released' if j>=spec['release_index'] else 'insert_contact') if spec else 'insert')
                points.append(local[0]);labels.append('contact_hold')
                for j,q in enumerate(local[1:]):
                    points.append(q);labels.append(('withdraw_released' if j>=spec['release_index'] else 'withdraw_contact') if spec else 'withdraw')
                for q in path[-2::-1]:points.append(q);labels.append('return')
                points.append(home);labels.append('final_hold')
                motion=timed_motion(side+'_'+bid,points,labels,settings);curve=motion.polynomial()
                if spec:motion.conditions['withdrawal']=spec
                # Bernstein convex hulls bound q and all three derivatives continuously.
                limits=np.array([[j.lower,j.upper] for j in engine.model.joints])
                allowance=settings['position_allowance_rad']
                position_ok=bool(np.all(curve.c.min(axis=(0,1))-allowance>=limits[:,0]) and np.all(curve.c.max(axis=(0,1))+allowance<=limits[:,1]))
                peaks=[float(np.max(derivative_bounds(curve,k))) for k in (1,2,3)]
                report.update(method=method,position_limits_passed=position_ok,derivative_peaks=peaks,duration_s=motion.duration)
                report['passed']=position_ok and all(a<=b for a,b in zip(peaks,[settings[k] for k in ['maximum_velocity_rad_s','maximum_acceleration_rad_s2','maximum_jerk_rad_s3']]))
                if report['passed']:
                    case=root/motion.name;case.mkdir()
                    write_json(case/'motion.json',motion.metadata())
                    n=round(motion.duration*1e9/settings['control_period_ns'])
                    ns=np.arange(n+1,dtype=np.int64)*settings['control_period_ns'];t=ns/1e9
                    np.savez_compressed(case/'reference.npz',time_ns=ns,q=curve(t),dq=curve(t,nu=1))
                    motions.append(motion.name)
            reports.append(report);write_json(root/'cases.json',reports)
            print(side,bid,'PASS' if report['passed'] else 'REJECT',flush=True)
    summary=dict(passed=len(reports)==8 and all(r['passed'] for r in reports),cases=len(reports),admitted=motions,
        control_hz=1e9/settings['control_period_ns'],continuous_joint_box=True,
        hardware_executable=False,tool_load_known=False,full_kinematic_identification=False)
    write_json(root/'report.json',summary);return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',required=True);p.add_argument('--seeds',type=Path)
    p.add_argument('--output',required=True);p.add_argument('--layout',type=Path);p.add_argument('--settings',type=Path,help='Explicit frozen JSON reference settings');a=p.parse_args()
    report=plan(a.output,a.bundle,a.seeds or asset_directory(a.bundle)/'seeds.json',settings=read(a.settings) if a.settings else None,layout=read(a.layout) if a.layout else None)
    print(report);return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
