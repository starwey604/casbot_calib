"""Contact hold refinement and release-then-turn reference planning.

All distances and kinematics use the existing native Pinocchio/Coal kernel.
"""
import argparse
from copy import copy
from itertools import product
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from .collision import CollisionProfile
from .dvt2_bundle import read,load_bundle
from .trical_assembly import profile,released_profile,ball_center,asset_directory,asset_manifest
from .trical_paths import gate_chord,gate_orientation,transfer_candidates,plan,SETTINGS
from .trical_motion import solve_position
from .data import write_json


def require_baseline_inputs(source,metadata):
    if source.get('schema_version')!='casbot_trical_withdrawal_seeds_v1':raise ValueError('Bound withdrawal hold/seed inputs required')
    if source.get('bundle_sha256')!=metadata['bundle_sha256']:raise ValueError('Withdrawal inputs belong to another bundle')
    if source.get('fixture_layout') is not None:raise ValueError('This withdrawal search supports the baseline fixture only; do not silently reuse another layout')


def bounded_profile(full,center,half_width):
    """Omit only pairs with a distance lower bound over the entire query box."""
    center=np.asarray(center);half_width=np.asarray(half_width)
    if center.shape!=(full.joints,) or half_width.shape!=center.shape or not np.isfinite(np.r_[center,half_width]).all() or np.any(half_width<0):raise ValueError('Finite center and nonnegative query box required')
    d=full.check(center)['distances'][0];lower=d-full.radii@half_width
    wanted=np.flatnonzero(lower<full.pair_clearance_m)
    if len(wanted)==0:return None,dict(all_pairs_preproved=True,minimum_surplus_m=float(np.min(lower-full.pair_clearance_m)))
    active={full.pairs[i] for i in wanted};n=len(full.geometry)
    exclusions=[(i,j) for i in range(n) for j in range(i+1,n) if (i,j) not in active]
    subset=CollisionProfile(full.engine,full.geometry,exclusions,dict(local_box_pruning=True))
    if subset.pairs!=[full.pairs[i] for i in wanted]:raise RuntimeError('Local pair order changed')
    subset.pair_clearance_m=full.pair_clearance_m[wanted]
    return subset,dict(pair_indices=wanted.tolist(),total_pairs=len(full.pairs),preproved_pairs=len(full.pairs)-len(wanted),half_width_rad=half_width.tolist())


def refine_hold(contact,side,bid,qseed,target,settings=None):
    settings=SETTINGS if settings is None else settings;engine=contact.engine;frame=side+'_trical_tcp'
    ix=np.arange(7) if side=='left' else np.arange(7,14);allow=settings['position_allowance_rad'];qseed=np.asarray(qseed)
    initial_gate=gate_chord(contact,qseed,qseed,settings)
    if initial_gate['passed'] and np.linalg.norm(engine.frame_pose(qseed,frame)[:3,3]-target)<5e-5:return qseed,dict(passed=True,method='existing_hold',gate=initial_gate)
    trust=.12;limits=np.array([[j.lower+.02,j.upper-.02] for j in engine.model.joints])[ix]
    lower=np.maximum(limits[:,0],qseed[ix]-trust);upper=np.minimum(limits[:,1],qseed[ix]+trust)
    center=qseed.copy();center[ix]=np.clip(center[ix],lower,upper)
    width=np.full(14,allow);width[ix]+=np.maximum(abs(lower-center[ix]),abs(upper-center[ix]))
    local,proof=bounded_profile(contact,center,width)
    if local is None:return center,dict(passed=np.linalg.norm(engine.frame_pose(center,frame)[:3,3]-target)<5e-5,method='whole_box_bound',pruning=proof)
    signs=np.array(list(product([-1.,1.],repeat=7)))*allow;selected=[0,127];history=[]
    moving=np.any(local.radii[:,ix]>0,axis=1).astype(float);x=center[ix].copy()
    for iteration in range(5):
        probes=np.tile(center,(128,1));probes[:,ix]=x+signs
        d=local.check(probes)['distances']-local.pair_clearance_m
        worst=np.argsort(d.min(axis=1))[:4]
        selected=list(dict.fromkeys([*selected,*worst.tolist()]));offsets=np.vstack([np.zeros(7),signs[selected]])
        cache={}
        def evaluate(z):
            key=z[:7].tobytes()
            if key not in cache:
                q=center.copy();q[ix]=z[:7];batch=np.tile(q,(len(offsets),1));batch[:,ix]+=offsets
                slack=local.check(batch)['distances']-local.pair_clearance_m
                cache.clear();cache[key]=(q,slack)
            return cache[key]
        start=np.r_[x,min(1.,float(d.min()*1000)-.05)]
        fit=minimize(lambda z:-z[7]+.001*np.sum((z[:7]-center[ix])**2),start,method='SLSQP',bounds=[*zip(lower,upper),(-100.,2.)],
            constraints=[dict(type='eq',fun=lambda z:1000*(engine.frame_pose(evaluate(z)[0],frame)[:3,3]-target)),
                         dict(type='ineq',fun=lambda z:(evaluate(z)[1]*1000-z[7]*moving).ravel())],
            options=dict(maxiter=100,eps=1e-5,ftol=1e-8))
        q,slack=evaluate(fit.x);x=fit.x[:7];err=float(np.linalg.norm(engine.frame_pose(q,frame)[:3,3]-target))
        probes=np.tile(q,(128,1));probes[:,ix]+=signs;corners=local.check(probes)['distances']-local.pair_clearance_m
        row=dict(iteration=iteration,optimizer_success=bool(fit.success),position_error_m=err,corner_minimum_surplus_m=float(corners.min()),selected_corners=len(selected))
        if err<5e-5 and corners.min()>0:
            gate=gate_chord(contact,q,q,settings);row['gate']=gate
            if gate['passed']:
                history.append(row);return q,dict(passed=True,method='corner_constraint_generation',history=history,pruning=proof)
        history.append(row)
    return q,dict(passed=False,method='corner_constraint_generation',history=history,pruning=proof)



# Offline proof budget only; physical margins, tracking box and frequency stay fixed.
ROUTE_SETTINGS={**SETTINGS,'joint_box_max_depth':32,'joint_box_query_budget':100_000,'local_motion_bounds':True}


def route_gate(p,a,b,cache=None):return gate_chord(p,a,b,ROUTE_SETTINGS,cache)


def escape_step(full,side,q,target,home):
    """Small joint step with generated tracking-corner constraints."""
    ix=np.arange(7) if side=='left' else np.arange(7,14);engine=full.engine;frame=side+'_trical_tcp';trust=.025
    allow=SETTINGS['position_allowance_rad'];moving=np.any(full.radii[:,ix]>0,axis=1)
    guarded=copy(full);guarded.pair_clearance_m=full.pair_clearance_m+moving*.0002
    width=np.full(14,allow);width[ix]+=trust;local,proof=bounded_profile(guarded,q,width)
    limits=np.array([[j.lower+.02,j.upper-.02] for j in engine.model.joints])[ix]
    lower=np.maximum(limits[:,0],q[ix]-trust);upper=np.minimum(limits[:,1],q[ix]+trust)
    p0=engine.frame_pose(q,frame)[:3,3];radius=np.linalg.norm(p0-target);normal=(p0-target)/max(radius,1e-8)
    J=engine.frame_jacobian(q,frame)[:3,ix]
    cart=np.linalg.lstsq(J,.003*normal,rcond=None)[0];cart*=min(1.,.018/max(abs(cart).max(),1e-12))
    home_step=home[ix]-q[ix];home_step*=min(1.,.008/max(abs(home_step).max(),1e-12))
    preferred=np.clip(q[ix]+cart+home_step,lower,upper)
    signs=np.array(list(product([-1.,1.],repeat=7)))*allow;selected=[0,127];x=np.clip(q[ix],lower,upper);history=[]
    for iteration in range(4):
        probes=np.tile(q,(128,1));probes[:,ix]=x+signs
        corners=np.empty((128,0)) if local is None else local.check(probes)['distances']-local.pair_clearance_m
        worst=np.array([],dtype=int) if local is None else np.argsort(corners.min(axis=1))[:4]
        selected=list(dict.fromkeys([*selected,*worst.tolist()]));offsets=np.vstack([np.zeros(7),signs[selected]]);cache={}
        def evaluate(values):
            key=values.tobytes()
            if key not in cache:
                value=q.copy();value[ix]=values;position=engine.frame_pose(value,frame)[:3,3]
                batch=np.tile(value,(len(offsets),1));batch[:,ix]+=offsets
                slack=np.empty(0) if local is None else (local.check(batch)['distances']-local.pair_clearance_m).ravel()
                cache.clear();cache[key]=(value,position,slack)
            return cache[key]
        fit=minimize(lambda values:np.sum(((values-preferred)/trust)**2),x,method='SLSQP',bounds=list(zip(lower,upper)),
            constraints=[dict(type='ineq',fun=lambda values:np.r_[1000*evaluate(values)[2],1000*(np.linalg.norm(evaluate(values)[1]-target)-radius-.001)])],
            options=dict(maxiter=60,eps=1e-5,ftol=1e-8))
        x=fit.x;value,position,_=evaluate(x);probes=np.tile(value,(128,1));probes[:,ix]+=signs
        corners=np.empty((128,0)) if local is None else local.check(probes)['distances']-local.pair_clearance_m
        margin=float(corners.min()) if local is not None else float('inf')
        gap=full.check(value)['distances'][0]-full.pair_clearance_m
        passed=bool(gap.min()>0 and margin>=-1e-8 and np.linalg.norm(position-target)>radius+.0005)
        history.append(dict(optimizer_success=bool(fit.success),sampled_corner_minimum_surplus_m=margin,selected_corners=len(selected)))
        if passed:break
    return value,dict(passed=passed,minimum_surplus_m=float(gap.min()),radius_m=float(np.linalg.norm(position-target)),pruning=proof,corner_iterations=history)


def bridge(free,home,stage,side,cache):
    for method,path in transfer_candidates(home,stage,side):
        probes=np.vstack([a+(b-a)*np.linspace(0,1,17)[:,None] for a,b in zip(path[:-1],path[1:])])
        if free.check(probes)['distances'].min()<=.01:continue
        proofs=[route_gate(free,a,b,cache=cache) for a,b in zip(path[:-1],path[1:])]
        if all(p['passed'] for p in proofs):return path,dict(method=method,gates=proofs)
    return None,dict(reason='no_direct_or_staged_bridge')


def finish_reference(contact,released,free,points,release_index,side,bid,target,home,cache):
    """Certify the local phases, then connect through fully free-space motion."""
    engine=contact.engine;frame=side+'_trical_tcp';R0=engine.frame_pose(points[0],frame)[:3,:3];gates=[]
    for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
        p=released if i>=release_index else contact;proof=route_gate(p,a,b,cache=cache);gates.append(proof)
        if not proof['passed']:return None,dict(reason='local_joint_box_gate',segment=i,gate=proof)
        if i<release_index:
            orientation=gate_orientation(engine,frame,a,b,R0,np.deg2rad(1),SETTINGS['position_allowance_rad'])
            if not orientation['passed']:return None,dict(reason='contact_orientation_gate',segment=i,gate=orientation)
    q=points[-1];free_chain=[q.copy()]
    for step in range(41):
        if step%5==0:
            path,proof=bridge(free,home,q,side,cache)
            if path is not None:
                transfer=np.vstack([path,np.array(free_chain[-2::-1])]) if len(free_chain)>1 else path
                local=[dict(q=x.tolist(),retract_m=float(np.linalg.norm(engine.frame_pose(x,frame)[:3,3]-target))) for x in points]
                return dict(arm=side,ball=bid,contact=local,rotation=R0.tolist(),transfer=transfer.tolist(),
                    withdrawal=dict(schema_version='casbot_trical_release_then_turn_v1',release_index=release_index,
                        minimum_released_tip_clearance_m=.0005,contact_orientation_tolerance_rad=float(np.deg2rad(1)))),dict(passed=True,local_gates=gates,bridge=proof,free_extension_steps=step)
        if step==40:break
        value,diagnostic=escape_step(free,side,q,target,home)
        if not diagnostic['passed']:return None,dict(reason='free_escape_stalled',step=step,diagnostic=diagnostic)
        accepted=None
        for factor in [1.,.5,.25,.125]:
            trial=q+(value-q)*factor;proof=route_gate(free,q,trial,cache=cache)
            if proof['passed']:accepted=trial;break
        if accepted is None:return None,dict(reason='free_escape_joint_box_gate',step=step)
        q=accepted;free_chain.append(q.copy())
    return None,dict(reason='free_escape_step_budget')


def cone_directions(R,axes,target,pillar_xy):
    weights=[np.ones(3)]
    for high in [2.,6.]:
        for j in range(3):
            w=np.ones(3);w[j]=high;weights.extend([w,high+1-w])
    # Also try the outward normal of the physical pillar, projected into the
    # three-probe release cone. No ball or robot is moved.
    outward=target-np.r_[pillar_xy,target[2]]
    if np.linalg.norm(outward)>1e-8:weights.insert(0,np.maximum(axes@(R.T@outward),.05*np.linalg.norm(outward)))
    return [R@axes.T@w/np.linalg.norm(w) for w in weights]


def route_case(bundle,side,bid,q0,cache):
    contact=profile(bundle,side,bid,True);released=released_profile(contact);free=profile(bundle,side)
    engine=contact.engine;_,meta=load_bundle(bundle);home=np.array(meta['profile']['home_rad']);target=ball_center(bid);frame=side+'_trical_tcp'
    position_error=float(np.linalg.norm(engine.frame_pose(q0,frame)[:3,3]-target))
    if position_error>5e-5:return None,dict(reason='contact_position_mismatch',position_error_m=position_error)
    initial=route_gate(contact,q0,q0,cache=cache)
    if not initial['passed']:return None,dict(reason='hold_not_certified',gate=initial)
    R=engine.frame_pose(q0,frame)[:3,:3];axes=np.array(asset_manifest(asset_directory(bundle))['axes_sensor'][side]);tip_indices=np.flatnonzero(contact.pair_clearance_m<0)
    attempts=[]
    for direction_i,direction in enumerate(cone_directions(R,axes,target,read(asset_directory(bundle)/'table_holes.json')['baseline_column_center_xy_m'])):
        points=[q0.copy()];release_index=None;release_prefix=None;last_tip=contact.check(q0)['distances'][0,tip_indices];note=dict(direction_index=direction_i,direction_world=direction.tolist())
        stage_found=False
        for distance in np.arange(.002,.060001,.002):
            try:q=solve_position(engine,frame,target+direction*distance,points[-1],R)
            except ValueError:note['reason']='push_IK_failed';break
            if Rotation.from_matrix(R.T@engine.frame_pose(q,frame)[:3,:3]).magnitude()>.001:note['reason']='push_orientation_failed';break
            current=released if release_index is not None else contact;d=current.check(q)['distances'][0];slack=d-current.pair_clearance_m
            if slack.min()<=0:note['reason']='push_clearance_failed';break
            if release_index is None and np.min(d[tip_indices]-last_tip)<-1e-5:note['reason']='probe_recompression';break
            last_tip=d[tip_indices];points.append(q.copy())
            if release_index is None and min(d[tip_indices])>.0015:
                release=route_gate(released,q,q,cache=cache)
                if release['passed']:release_index=len(points)-1;release_prefix=list(points)
            if release_index is not None and free.check(q)['distances'].min()>.0105:
                free_hold=route_gate(free,q,q,cache=cache)
                if free_hold['passed']:stage_found=True;break
        if stage_found:
            row,certificate=finish_reference(contact,released,free,points,release_index,side,bid,target,home,cache)
            note['straight_certificate']=certificate
            if row is not None:return row,dict(passed=True,mode='release_cone',attempts=[*attempts,note])
        if release_prefix is not None:
            # Once all three probes have positive certified clearance, allow
            # orientation to change. Subsequent chords use the stricter profile.
            points=release_prefix;q=points[-1]
            for step in range(40):
                value,diagnostic=escape_step(released,side,q,target,home)
                if not diagnostic['passed']:note['turn_reason']='released_escape_stalled';break
                accepted=None
                for factor in [1.,.5,.25,.125]:
                    trial=q+(value-q)*factor;proof=route_gate(released,q,trial,cache=cache)
                    if proof['passed']:accepted=trial;break
                if accepted is None:note['turn_reason']='released_escape_joint_box_gate';note['failed_gate']=proof;break
                q=accepted;points.append(q.copy())
                if free.check(q)['distances'].min()>.0105 and route_gate(free,q,q,cache=cache)['passed']:
                    row,certificate=finish_reference(contact,released,free,points,release_index,side,bid,target,home,cache);note['turn_certificate']=certificate
                    if row is not None:return row,dict(passed=True,mode='release_then_turn',attempts=[*attempts,note])
                    break
            else:note['turn_reason']='released_escape_step_budget'
        attempts.append(note);print(side,bid,'direction',direction_i,note.get('turn_reason',note.get('reason')),flush=True)
    return None,dict(passed=False,reason='withdrawal_candidates_exhausted',attempts=attempts)


def run_routes(bundle,source_path,output):
    root=Path(output);root.mkdir(parents=True,exist_ok=False);source=read(source_path);_,meta=load_bundle(bundle)
    require_baseline_inputs(source,meta)
    write_json(root/'protocol.json',dict(bundle_sha256=meta['bundle_sha256'],source=str(source_path),settings=dict(ROUTE_SETTINGS),released_tip_clearance_m=.0005,maximum_push_m=.06,turn_step_budget=40,hardware_executable=False))
    chosen={};reports=[];cache={}
    for seed in source['results']:
        key=(seed['arm'],seed['ball'])
        if key in chosen:continue
        row,report=route_case(bundle,*key,np.array(seed['q_rad']),cache);reports.append(dict(arm=key[0],ball=key[1],report=report))
        if row is not None:chosen[key]=row
        write_json(root/'search.json',reports);print(*key,'REFERENCE',row is not None,flush=True)
    original=read(asset_directory(bundle)/'seeds.json');rows=[]
    for row in original['results']:
        key=(row['arm'],row['ball'])
        rows.append(chosen[key] if key in chosen else dict(arm=key[0],ball=key[1],candidate_available=False) if key in {('left','D'),('right','C'),('right','A')} else row)
    write_json(root/'planning_seeds.json',dict(schema_version='casbot_trical_withdrawal_plan_v1',fixture_layout=None,bundle_sha256=meta['bundle_sha256'],results=rows))
    result=plan(root/'plan',bundle,root/'planning_seeds.json',settings=ROUTE_SETTINGS,gate_cache=cache)
    write_json(root/'report.json',dict(reference_plan=result,searched_cases=reports,reference_only=True,hardware_executable=False));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',required=True);g=p.add_mutually_exclusive_group(required=True);g.add_argument('--seeds');g.add_argument('--holds');p.add_argument('--output',required=True);a=p.parse_args()
    if a.holds:return 0 if run_routes(a.bundle,a.holds,a.output)['passed'] else 1
    root=Path(a.output);root.mkdir(parents=True,exist_ok=False);source=read(a.seeds);_,meta=load_bundle(a.bundle)
    require_baseline_inputs(source,meta)
    write_json(root/'protocol.json',dict(bundle_sha256=meta['bundle_sha256'],source=str(a.seeds),settings=dict(SETTINGS)))
    results=[]
    for row in source['results']:
        side=row['arm'];bid=row['ball'];contact=profile(a.bundle,side,bid,True)
        q,report=refine_hold(contact,side,bid,row['q_rad'],ball_center(bid));entry=dict(arm=side,ball=bid,q_rad=q.tolist(),hold=report)
        results.append(entry);write_json(root/'holds.json',results);print(side,bid,'HOLD',report['passed'],flush=True)
    requested={(r['arm'],r['ball']) for r in results};accepted={(r['arm'],r['ball']) for r in results if r['hold']['passed']}
    write_json(root/'report.json',dict(bundle_sha256=meta['bundle_sha256'],holds=results,hold_coverage_passed=requested==accepted,reference_only=True,hardware_executable=False))
    return 0 if requested==accepted else 1


if __name__=='__main__':raise SystemExit(main())
