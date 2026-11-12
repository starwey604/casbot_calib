#!/usr/bin/env python3
"""Independent per-kernel native force verification of a sustained-wrist R2 fit."""
import argparse
from pathlib import Path
import numpy as np

from design_wrist_supplement import protect_observations
from diagnose_wrist_periodic_counterfactual import model,sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.staged_attribution import arrays,formal_test_untouched
from casbot_calib.staged_collection import load_trial
from casbot_calib.staged_identification import read_record
from casbot_calib.staged_multifit import assemble_training,load_problems
from casbot_calib.staged_protocol import load_protocol,_read,_unseal
from casbot_calib.staged_pitch_sustained import WRIST_POLICY
from casbot_calib.staged_right_pitch import right_companion
from casbot_calib.staged_wrist_periodic import companion

def verify(root,r2,output):
    root,r2,output=(p.resolve() for p in (root,r2,output))
    if output.exists():raise ValueError('Use a fresh verification file')
    _,config,manifest=load_protocol(root);formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    for reader in (companion,right_companion):
        cap,names=reader(root);protect_observations(cap,{(n,) for n in names})
    sealed_r2(root,r2);coord=arrays(r2/'coordinates.npz')
    state=arrays(r2/'variants/ridge_0.0001_full/solution.npz');v=state['vector']
    nominal,_=model_for(config);engine=model(nominal,v,coord['static'])
    train=load_problems(root,'train',coord['static'])
    y,x,target,w=assemble_training(train,config)
    if w!=_unseal(r2,'training_setup.json')['weights']:raise ValueError('Frozen weights changed')
    residual=y@v[:140]+x@v[140:]-target;del y,x,target
    expected={};start=0
    for a in w:
        end=start+a['rows']
        if a['stage']=='inertia' and a['joint'] in WRIST_POLICY['joints']:
            expected[a['trial_id'],a['joint']]=residual[start:end]/a['sqrt_weight']
        start=end
    del residual,train
    names=_read(root/'session.json')['joint_names'];frozen=arrays(root/WRIST_POLICY['support_file'])
    step=config['inertia_sample_period_ns'];dt=step*1e-9
    margin=round(WRIST_POLICY['same_sign_margin_s']/dt)
    sigma=WRIST_POLICY['sigma_ms']*1e-3/dt
    maxradius=int(np.ceil(3*sigma));reports=[]
    primary=sorted({a[0] for a in expected})
    for name in primary:
        meta,_=load_trial(root,name);_,obs=read_record(root,name,meta,names)
        period=round(meta['conditions']['period_s']*1e9)
        begin=round(meta['knots_s'][meta['labels'].index('periodic')]*1e9)+2*period
        lo,hi=np.searchsorted(obs['sample_ns'],[begin,begin+2*period])
        ix=np.arange(lo+2,hi-2);speed=obs['dq'];q=obs['q']
        acc=(speed[ix-2]-8*speed[ix-1]+8*speed[ix+1]-speed[ix+2])/(12*dt)
        native=engine.evaluate_batch(q[ix],speed[ix],acc)['total']
        tau=obs['current'][ix]*config['drive']['gain_nm_a']
        error=native-tau
        for j in WRIST_POLICY['joints']:
            valid=np.arange(lo+margin,hi-margin)
            good=np.ones(len(valid),bool);s=speed[:,j]
            for shift in range(-margin,margin+1):
                good&=(abs(s[valid+shift])>WRIST_POLICY['minimum_speed_rad_s'])&(s[valid+shift]*s[valid]>0)
            valid=valid[good];centers=valid[(valid-lo)%WRIST_POLICY['stride']==0]
            np.testing.assert_array_equal(centers,frozen[f'{name}__j{j:02}'])
            ends=np.r_[0,np.flatnonzero(np.diff(valid)!=1)+1,len(valid)]
            filtered=[]
            for where in np.searchsorted(valid,centers):
                seg=np.searchsorted(ends[1:],where,side='right')
                r=min(maxradius,where-ends[seg],ends[seg+1]-where-1)
                off=np.arange(-r,r+1);coeff=np.exp(-.5*(off/sigma)**2);coeff/=coeff.sum()
                filtered.append(float(coeff@error[valid[where+off]-lo-2,j]))
            diff=float(np.max(abs(np.asarray(filtered)-expected[name,j])))
            if diff>1e-10:raise ValueError('Independent native force balance disagrees')
            reports.append(dict(trial=name,joint=j,rows=len(centers),maximum_direct_force_difference_nm=diff))
    formal_test_untouched(root)
    write_json(output,dict(passed=True,checks=len(reports),maximum_force_difference_nm=max(
        r['maximum_direct_force_difference_nm'] for r in reports),rows=reports,ab_observations_used=False,
        formal_test_consumed=False,truth_read=False))
    print('Independent native force verification passed',len(reports),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('root','r2','output'):p.add_argument(arg,type=Path)
    verify(**vars(p.parse_args()))
