"""Collision-aware cross-side access search using native FK/Coal and SciPy.

Position is constrained; tool orientation and the seven active joints are free.
This produces geometric candidates, not SDK commands or calibration data.
"""
import argparse
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from .dvt2_bundle import read,load_bundle
from .trical_assembly import profile,ball_center,asset_directory
from .trical_motion import solve_position
from .trical_paths import transfer_candidates,gate_chord,SETTINGS
from .data import write_json


def optimize_endpoints(engine,contact,free,frame,ix,q0,target,length,lower,upper):
    """Jointly place contact/retract poses, keeping one free tool orientation."""
    T=engine.frame_pose(q0,frame)
    try:q1=solve_position(engine,frame,target+T[:3,0]*length,q0,T[:3,:3])
    except ValueError:q1=q0.copy()
    cache={}
    def evaluate(z):
        key=z[:14].tobytes()
        if key not in cache:
            a=q0.copy();b=q0.copy();a[ix]=z[:7];b[ix]=z[7:14]
            A=engine.frame_pose(a,frame);B=engine.frame_pose(b,frame)
            d0=contact.check(a)['distances'][0]-contact.pair_clearance_m
            d1=free.check(b)['distances'][0]-free.pair_clearance_m
            cache.clear();cache[key]=(A,B,np.r_[d0,d1])
        return cache[key]
    influence=np.r_[np.any(contact.radii[:,ix]>0,axis=1),np.any(free.radii[:,ix]>0,axis=1)].astype(float)
    def equality(z):
        A,B,_=evaluate(z)
        return np.r_[1000*(A[:3,3]-target),1000*(B[:3,3]-A[:3,3]-A[:3,0]*length),Rotation.from_matrix(A[:3,:3].T@B[:3,:3]).as_rotvec()]
    # Keep the coupled correction local to its already restored endpoints.
    # Wide joint jumps lead to deep penetration and expensive, poor gradients.
    a_lo=np.maximum(lower,q0[ix]-.35);a_hi=np.minimum(upper,q0[ix]+.35)
    b_lo=np.maximum(lower,q1[ix]-.35);b_hi=np.minimum(upper,q1[ix]+.35)
    initial=np.r_[q0[ix],q1[ix],-20.]
    return minimize(lambda z:-z[-1]+1e-3*np.sum((z[:7]-q0[ix])**2),initial,method='SLSQP',
        bounds=[*zip(a_lo,a_hi),*zip(b_lo,b_hi),(-200.,1.5)],
        constraints=[dict(type='eq',fun=equality),dict(type='ineq',fun=lambda z:evaluate(z)[2]*1000-z[-1]*influence)],
        options=dict(maxiter=120,ftol=1e-8,eps=2e-5))


def search(bundle,output,*,attempts=24,seed=20261105,layout=None,coupled=False):
    if type(attempts) is not int or attempts<1:raise ValueError('Positive attempt budget required')
    root=Path(output);root.mkdir(parents=True,exist_ok=False);engine,meta=load_bundle(bundle)
    rng=np.random.default_rng(seed);home=np.array(meta['profile']['home_rad']);source=read(asset_directory(bundle)/'seeds.json')['results']
    limits=np.array([[j.lower+.02,j.upper-.02] for j in engine.model.joints]);reports=[];accepted=[]
    write_json(root/'protocol.json',dict(bundle_sha256=meta['bundle_sha256'],fixture_layout=layout,attempts_per_case=attempts,seed=seed,
        optimization='SLSQP max-min clearance with exact TCP position; free orientation; native Coal signed convex distances',coupled_contact_and_retract=coupled,ordinary_clearance_m=.01,
        minimum_requested_surplus_m=.00035,reference_gate_settings=dict(SETTINGS),coupled_trust_half_width_rad=.35 if coupled else None,hardware_executable=False))
    for side,bid in [('left','D'),('right','C')]:
        begin=perf_counter();contact=profile(bundle,side,bid,True,layout=layout);free=profile(bundle,side,layout=layout)
        frame=side+'_trical_tcp';ix=np.arange(7) if side=='left' else np.arange(7,14);target=ball_center(bid,layout)
        reference=next(r for r in source if (r['arm'],r['ball'])==(side,bid));q0=np.array(reference['contact'][0]['q'])
        lower=limits[ix,0];upper=limits[ix,1];case=dict(arm=side,ball=bid,attempts=[],best=None);cache={}
        def evaluate(x):
            key=x[:7].tobytes()
            if key not in cache:
                q=q0.copy();q[ix]=x[:7];T=engine.frame_pose(q,frame);d=contact.check(q)['distances'][0]-contact.pair_clearance_m
                cache.clear();cache[key]=(q,T,d)
            return cache[key]
        for attempt in range(attempts):
            x=q0[ix] if attempt==0 else q0[ix]+rng.normal(0,.3 if attempt%3==1 else .8,7) if attempt%3 else rng.uniform(lower,upper)
            x=np.clip(x,lower+1e-6,upper-1e-6)
            # Restore position before solving inequalities; orientation stays free.
            try:qseed=solve_position(engine,frame,target,np.r_[x,q0[7:]] if side=='left' else np.r_[q0[:7],x]);x=qseed[ix]
            except ValueError:pass
            _,_,d=evaluate(np.r_[x,0]);initial=np.r_[x,max(-150.,min(float(d.min()*1000)-.1,1.))]
            fit=minimize(lambda z:-z[7]+1e-3*np.sum((z[:7]-q0[ix])**2),initial,method='SLSQP',
                bounds=[*zip(lower,upper),(-200.,1.)],
                constraints=[dict(type='eq',fun=lambda z:(evaluate(z)[1][:3,3]-target)*1000),
                             dict(type='ineq',fun=lambda z:evaluate(z)[2]*1000-z[7])],
                options=dict(maxiter=180,ftol=1e-8,eps=2e-5))
            if coupled:
                candidate=q0.copy();candidate[ix]=fit.x[:7]
                fit=optimize_endpoints(engine,contact,free,frame,ix,candidate,target,.025 if side=='right' else .04,lower,upper)
            q,T,d=evaluate(fit.x);k=int(np.argmin(d));error=float(np.linalg.norm(T[:3,3]-target));surplus=float(d[k])
            row=dict(attempt=attempt,optimizer_success=bool(fit.success),message=str(fit.message),position_error_m=error,
                minimum_surplus_m=surplus,worst_pair=[contact.names[j] for j in contact.pairs[k]],q_rad=q.tolist(),rotation=T[:3,:3].tolist())
            case['attempts'].append(row)
            if error<5e-5 and (case['best'] is None or surplus>case['best']['minimum_surplus_m']):case['best']=row
            print(side,bid,attempt,'pos_um',round(error*1e6,2),'slack_mm',round(surplus*1000,3),row['worst_pair'],flush=True)
            write_json(root/f'{side}_{bid}.json',case)
            if error>5e-5 or surplus<.00035:continue
            rotation=T[:3,:3];length=.025 if side=='right' else .04;local=[];qseed=q.copy()
            for off in np.linspace(0,length,round(length/.005)+1):
                try:qseed=solve_position(engine,frame,target+rotation[:,0]*off,qseed,rotation)
                except ValueError:break
                if Rotation.from_matrix(rotation.T@engine.frame_pose(qseed,frame)[:3,:3]).magnitude()>.001:break
                local.append(dict(q=qseed.tolist(),retract_m=float(off)))
            if len(local)!=round(length/.005)+1:row['local_reason']='local_IK_or_orientation_failed';continue
            points=np.array([r['q'] for r in local]);samples=np.vstack([points,(points[:-1]+points[1:])/2])
            ds=contact.check(samples)['distances']-contact.pair_clearance_m
            if ds.min()<=0:
                i,j=np.unravel_index(np.argmin(ds),ds.shape);row['local_reason']='local_clearance_failed';row['local_worst_pair']=[contact.names[k] for k in contact.pairs[j]];continue
            hold=gate_chord(contact,points[0],points[0]);row['hold_gate']=hold
            if not hold['passed']:continue
            gates=[gate_chord(contact,a,b) for a,b in zip(points[:-1],points[1:])];row['local_gates']=gates
            if not all(g['passed'] for g in gates):continue
            route=None
            for method,path in transfer_candidates(home,points[-1],side):
                samples=np.vstack([a+(b-a)*np.linspace(0,1,25)[:,None] for a,b in zip(path[:-1],path[1:])])
                if free.check(samples)['distances'].min()<=.01:continue
                proofs=[gate_chord(free,a,b) for a,b in zip(path[:-1],path[1:])]
                if all(p['passed'] for p in proofs):route=(method,path,proofs);break
            if route is None:row['local_reason']='no_certified_transfer';continue
            method,path,proofs=route
            candidate=dict(arm=side,ball=bid,contact=local,rotation=rotation.tolist(),transfer=path.tolist(),minimum_nominal_surplus_m=surplus,
                proofs=dict(hold=hold,local=gates,transfer=proofs),method=method)
            accepted.append(candidate);write_json(root/'candidates.json',dict(fixture_layout=layout,results=accepted))
            row['complete_reference_certified']=True;break
        case['wall_s']=perf_counter()-begin;reports.append(case);write_json(root/f'{side}_{bid}.json',case)
    result=dict(cases=reports,admitted=[r['arm']+'_'+r['ball'] for r in accepted],all_cross_cases_certified=len(accepted)==2,hardware_executable=False)
    write_json(root/'report.json',result);return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',required=True);p.add_argument('--output',required=True);p.add_argument('--attempts',type=int,default=24);p.add_argument('--layout',type=Path);p.add_argument('--coupled',action='store_true');a=p.parse_args()
    search(a.bundle,a.output,attempts=a.attempts,layout=read(a.layout) if a.layout else None,coupled=a.coupled)
