"""Read-only verification of exported TriCal references; no pose/path selection."""
import argparse,csv
from itertools import product
from pathlib import Path
import numpy as np
from scipy.interpolate import BPoly
from .dvt2_bundle import read,load_bundle
from .trical_assembly import profile,released_profile,static_platform_check,ball_center
from .trical_paths import gate_chord,gate_orientation,withdrawal_rules
from .staged_motion import derivative_bounds
from .kinematics import digest
from .data import write_json


def verify(plan_folder,bundle,seeds,output):
    root=Path(output);root.mkdir(parents=True,exist_ok=False);plan_folder=Path(plan_folder)
    protocol=read(plan_folder/'protocol.json');source=read(seeds);engine,meta=load_bundle(bundle);settings=protocol['settings'];layout=protocol['fixture_layout']
    if protocol['bundle_sha256']!=meta['bundle_sha256'] or protocol['seed_sha256']!=digest(source):raise ValueError('Model or seed identity mismatch')
    if source.get('fixture_layout')!=layout:raise ValueError('Fixture layout mismatch')
    rows={(r['arm'],r['ball']):r for r in source['results']};results=[];series=[];cache={}
    home=np.array(meta['profile']['home_rad']);allow=settings['position_allowance_rad'];period=settings['control_period_ns']
    for side in ['left','right']:
        free=profile(bundle,side,layout=layout);static=static_platform_check(free)
        for bid in 'ABCD':
            row=rows[(side,bid)];folder=plan_folder/(side+'_'+bid);motion=read(folder/'motion.json')
            states=np.array(motion['states']);Q=states[:,0];knots=np.array(motion['knots_s']);labels=motion['labels'];curve=BPoly.from_derivatives(knots,states)
            if len(knots)!=len(states) or len(labels)+1!=len(states) or np.any(np.diff(knots)<=0):raise ValueError('Malformed motion knots')
            if not np.allclose(states[:,1:],0.,atol=1e-14,rtol=0):raise ValueError('Chord proof requires rest-to-rest C3 states')
            if not np.array_equal(Q[0],home) or not np.array_equal(Q[-1],home):raise ValueError('Motion does not start and finish at home')
            if not np.allclose(knots*1e9/period,np.round(knots*1e9/period),atol=1e-7,rtol=0):raise ValueError('Knots do not align with control period')
            contact=profile(bundle,side,bid,True,layout=layout);local=np.array([r['q'] for r in row['contact']]);spec=withdrawal_rules(row,len(local))
            released=released_profile(contact,spec['minimum_released_tip_clearance_m']) if spec else None
            if motion['conditions'].get('withdrawal')!=spec:raise ValueError('Motion/seed withdrawal rules differ')
            contact_error=float(np.linalg.norm(engine.frame_pose(local[0],side+'_trical_tcp')[:3,3]-ball_center(bid,layout))) if spec else None
            h=labels.index('contact_hold');n=len(local)
            if labels.count('contact_hold')!=1 or not np.array_equal(Q[h-n+1:h+1],local[::-1]) or not np.array_equal(Q[h+1:h+n+1],local):raise ValueError('Exported local path differs from seed')
            for i in range(n-1):
                suffix=('_released' if i>=spec['release_index'] else '_contact') if spec else ''
                if labels[h-1-i]!='insert'+suffix or labels[h+1+i]!='withdraw'+suffix:raise ValueError('Exported release phases differ from seed')
            if not np.array_equal(Q[h],Q[h+1]):raise ValueError('Measurement hold moves')
            limits=np.array([[j.lower,j.upper] for j in engine.model.joints]);position_ok=bool(np.all(curve.c.min(axis=(0,1))-allow>=limits[:,0]) and np.all(curve.c.max(axis=(0,1))+allow<=limits[:,1]))
            peaks=[float(np.max(derivative_bounds(curve,k))) for k in [1,2,3]]
            dynamics_ok=all(a<=settings[k] for a,k in zip(peaks,['maximum_velocity_rad_s','maximum_acceleration_rad_s2','maximum_jerk_rad_s3']))
            with np.load(folder/'reference.npz',allow_pickle=False) as data:
                ns=data['time_ns'];t=ns/1e9
                stream_ok=bool(ns[0]==0 and abs(ns[-1]/1e9-knots[-1])<1e-8 and np.all(np.diff(ns)==period) and np.isfinite(data['q']).all())
                q_error=float(np.max(abs(data['q']-curve(t))));dq_error=float(np.max(abs(data['dq']-curve(t,nu=1))))
            frame=side+'_trical_tcp';R0=engine.frame_pose(local[0],frame)[:3,:3];gates=[];orientations=[]
            for i,label in enumerate(labels):
                if label in {'initial_hold','approach','return','final_hold'}:p=free
                elif label in {'insert','withdraw','contact_hold','insert_contact','withdraw_contact'}:p=contact
                elif label in {'insert_released','withdraw_released'} and spec:p=released
                else:raise ValueError('Unknown or unbound motion phase: '+label)
                g=gate_chord(p,Q[i],Q[i+1],settings,cache);gates.append(g)
                if spec and label in {'insert_contact','withdraw_contact','contact_hold'}:
                    orientations.append(gate_orientation(engine,frame,Q[i],Q[i+1],R0,spec['contact_orientation_tolerance_rad'],allow))
                distances=p.check(curve(np.array([knots[i],(knots[i]+knots[i+1])/2,knots[i+1]])))['distances']
                j,k=np.unravel_index(np.argmin(distances-p.pair_clearance_m),distances.shape)
                series.append(dict(case=side+'_'+bid,segment=i,phase=label,start_s=knots[i],end_s=knots[i+1],
                    sampled_minimum_surplus_m=float((distances-p.pair_clearance_m)[j,k]),passed=g['passed']))
            passed=bool(static['passed'] and position_ok and dynamics_ok and stream_ok and q_error<1e-12 and dq_error<1e-12 and (contact_error is None or contact_error<=5e-5) and all(g['passed'] for g in gates) and all(g['passed'] for g in orientations))
            result=dict(case=side+'_'+bid,passed=passed,samples=len(ns),duration_s=float(knots[-1]),position_limits_passed=position_ok,derivative_peaks=peaks,
                stream_period_passed=stream_ok,q_reload_error_rad=q_error,dq_reload_error_rad_s=dq_error,
                continuous_gates=gates,contact_orientation_gates=orientations,withdrawal_rules=spec)
            result['new_contact_position_error_m']=contact_error
            results.append(result);write_json(root/'cases.json',results);print(side,bid,'VERIFIED',passed,flush=True)
    with (root/'segments.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,list(series[0]));w.writeheader();w.writerows(series)
    result=dict(passed=len(results)==8 and all(r['passed'] for r in results),cases=len(results),segments=len(series),bundle_sha256=meta['bundle_sha256'],
        control_hz=1e9/period,hardware_executable=False,probe_geometry_scope='reviewed working-contact geometry; free-plunger envelope unmeasured',
        source=str(plan_folder),settings=settings)
    write_json(root/'verification.json',result);return result



def probe_local_corners(plan_folder,bundle,seeds,output):
    """Independent native-distance probes; no local motion radius is consulted."""
    root=Path(output);root.mkdir(parents=True,exist_ok=False);protocol=read(Path(plan_folder)/'protocol.json');source=read(seeds)
    if protocol['seed_sha256']!=digest(source):raise ValueError('Corner-probe seed identity mismatch')
    _,meta=load_bundle(bundle)
    if protocol['bundle_sha256']!=meta['bundle_sha256']:raise ValueError('Corner-probe model identity mismatch')
    allow=protocol['settings']['position_allowance_rad'];offsets=np.array(list(product([-1.,1.],repeat=7)))*allow;reports=[]
    for row in source['results']:
        if not row.get('withdrawal'):continue
        side=row['arm'];bid=row['ball'];q=np.array([r['q'] for r in row['contact']]);spec=withdrawal_rules(row,len(q))
        p=profile(bundle,side,bid,True,layout=protocol['fixture_layout']);released=released_profile(p,spec['minimum_released_tip_clearance_m']);ix=np.arange(7) if side=='left' else np.arange(7,14)
        for i in range(len(q)):
            center=q[i] if i==len(q)-1 else (q[i]+q[i+1])/2
            batch=np.tile(center,(128,1));batch[:,ix]+=offsets
            geometry=released if i>=spec['release_index'] else p
            d=geometry.check(batch)['distances']-geometry.pair_clearance_m;j,k=np.unravel_index(np.argmin(d),d.shape)
            reports.append(dict(case=side+'_'+bid,segment=i,phase='released' if geometry is released else 'contact',samples=128,
                minimum_surplus_m=float(d[j,k]),pair=[geometry.names[a] for a in geometry.pairs[k]],passed=bool(d.min()>0)))
    with (root/'corners.csv').open('w',encoding='utf-8-sig',newline='') as f:
        fields=['case','segment','phase','samples','minimum_surplus_m','pair','passed']
        w=csv.DictWriter(f,fields);w.writeheader();w.writerows(reports)
    result=dict(passed=all(r['passed'] for r in reports),batches=len(reports),samples=sum(r['samples'] for r in reports),
        minimum_sampled_surplus_m=min((r['minimum_surplus_m'] for r in reports),default=None),rows=reports,
        scope='Native Coal probes at local chord midpoints/last endpoint plus seven-axis box corners; supplements, does not replace continuous proofs')
    write_json(root/'corners.json',result);return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['plan','bundle','seeds','output']:p.add_argument('--'+key,required=True)
    a=p.parse_args()
    # The verifier may only consume frozen references and native geometry.
    from . import _core
    import scipy.optimize
    def forbidden(*args,**kwargs):raise RuntimeError('Optimization and plant execution are forbidden during reference verification')
    scipy.optimize.minimize=forbidden;scipy.optimize.least_squares=forbidden
    _core.ArmSimulationRunner=forbidden;_core.TricalMockSession=forbidden
    result=verify(a.plan,a.bundle,a.seeds,a.output)
    extra=probe_local_corners(a.plan,a.bundle,a.seeds,Path(a.output)/'corner_probe')
    result['optimization_and_plant_blocked']=True;result['corner_probe_passed']=extra['passed'];result['passed']=result['passed'] and extra['passed'];write_json(Path(a.output)/'verification.json',result)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
