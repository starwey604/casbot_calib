#!/usr/bin/env python3
"""Offline verification of raw sustained right-pitch force balance; no collection."""
import argparse
from pathlib import Path

import numpy as np
from scipy.ndimage import minimum_filter1d

from design_wrist_supplement import protect_observations
from diagnose_right_pitch_balance import raw_balance
from diagnose_wrist_periodic_counterfactual import model, sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.rigid_identification import fit_rigid
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_collection import load_trial
from casbot_calib.staged_identification import read_record
from casbot_calib.staged_multifit import assemble_training, load_problems, score_problems
from casbot_calib.staged_protocol import _read, _unseal, load_protocol
from casbot_calib.staged_right_pitch import right_companion
from casbot_calib.staged_stability import frozen_group_mask
from casbot_calib.staged_stability_math import operator_difference
from casbot_calib.staged_wrist_local import local_kernel
from casbot_calib.staged_wrist_periodic import companion

PITCH=12
SIGMA_MS=1.5
STRIDE=15
SPEED=.02
MARGIN_S=.005


def support(obs,meta,step):
    period=round(meta['conditions']['period_s']*1e9)
    start=round(meta['knots_s'][meta['labels'].index('periodic')]*1e9)+2*period
    lo,hi=np.searchsorted(obs['sample_ns'],[start,start+2*period])
    np.testing.assert_array_equal(obs['sample_ns'][lo:hi],np.arange(start,start+2*period,step))
    radius=round(MARGIN_S*1e9/step)
    if radius<2:raise ValueError('Sustained margin must enclose the differentiation stencil')
    v=obs['dq'][:,PITCH]
    good=((minimum_filter1d(v,size=2*radius+1,mode='nearest')>SPEED)
          |(minimum_filter1d(-v,size=2*radius+1,mode='nearest')>SPEED))
    valid=np.flatnonzero(good & (np.arange(len(v))>=lo+radius) & (np.arange(len(v))<hi-radius))
    centers=valid[(valid-lo)%STRIDE==0]
    if len(centers)<32:raise ValueError('Insufficient sustained pitch observations')
    # Recheck finite difference and full raw support locally, independently of
    # the run segmentation used by the smoother.
    for offset in range(-radius,radius+1):
        if not np.all((abs(v[valid+offset])>SPEED)&(v[valid+offset]*v[valid]>0)):
            raise ValueError('An eligible window crosses a pitch stop/sign change')
    return lo,hi,valid,centers


def fit_case(X,target,mask,ridge,coord,nominal,queries):
    fit=fit_rigid(X[mask,:140],target[mask],coord['nominal_theta'],
        auxiliary=(X[mask,140:],coord['friction'],coord['lower'],coord['upper']),
        equalities=(coord['equality'],coord['equality_target']),ridge=ridge,
        physical_length_scale=.1,lift_residuals=True,fixed_scales=coord['scales'],regularization_metric=coord['metric'])
    v=np.r_[fit['parameters'],fit['auxiliary_parameters']]
    pred=model(nominal,v,coord['static']).evaluate_batch(queries['q'],queries['dq'],queries['ddq'])
    return dict(vector=v,mass=pred['mass_matrix'],torque=pred['total']),fit


def verify(root,r2,output):
    root,r2,output=(p.resolve() for p in (root,r2,output))
    if output.exists() or any(output.is_relative_to(p) for p in (root,r2)):
        raise ValueError('Use new diagnostic output outside original inputs')
    _,config,manifest=load_protocol(root)
    formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    for reader in (companion,right_companion):
        cap,names=reader(root);protect_observations(cap,{(n,) for n in names})
    protect_observations(root.parent/'motion_pair_capture_v1',set())
    protocol=sealed_r2(root,r2)
    coord=arrays(r2/'coordinates.npz');queries=arrays(r2/'operator_probe_set.npz')
    nominal,_=model_for(config)
    problems=load_problems(root,'train',coord['static'])
    y,x,target,weights=assemble_training(problems,config)
    if weights!=_unseal(r2,'training_setup.json')['weights']:raise ValueError('Frozen input weights differ')
    original=np.column_stack((y,x));del y,x
    names=_read(root/'session.json')['joint_names']
    step=config['inertia_sample_period_ns']
    replacement={};infos=[];frozen_support={}
    for name,trial in problems.items():
        if trial['meta']['stage']!='inertia':continue
        meta,_=load_trial(root,name)
        record,obs=read_record(root,name,meta,names)
        if record['sample_period_ns']!=step:raise ValueError('Wrong sample clock')
        lo,hi,valid,centers=support(obs,meta,step)
        rows,_=raw_balance(nominal,obs,config['drive']['gain_nm_a'],coord['static'],step*1e-9)
        K,radii,nominal_radius=local_kernel(valid,centers,SIGMA_MS*1e6/step)
        block=K@rows[valid-2]
        replacement[name]=block;frozen_support[name]=centers
        q=obs['q'][centers];old=trial['problem'];oldmask=old['moving'][:,PITCH]
        half=(hi-lo)//2
        infos.append(dict(trial=name,center=meta['conditions']['center_id'],
            total_raw_samples=hi-lo,eligible_raw_samples=len(valid),rows=len(centers),
            rows_per_cycle=[int(np.count_nonzero(centers<lo+half)),int(np.count_nonzero(centers>=lo+half))],
            roll_range_rad=[float(q[:,13].min()),float(q[:,13].max())],
            pitch_range_rad=[float(q[:,12].min()),float(q[:,12].max())],
            old_roll_range_rad=[float(old['q'][oldmask,13].min()),float(old['q'][oldmask,13].max())],
            old_pitch_range_rad=[float(old['q'][oldmask,12].min()),float(old['q'][oldmask,12].max())],
            minimum_raw_speed_rad_s=float(np.min(abs(obs['dq'][valid,PITCH]))),
            nominal_filter_radius_samples=nominal_radius,
            shortened_filter_rows=int(np.count_nonzero(radii<nominal_radius)),point_only_filter_rows=int(np.count_nonzero(radii==0))))
    output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'raw_sample_indices.npz',**frozen_support)
    write_json(output/'policy.json',dict(sigma_ms=SIGMA_MS,sustained_minimum_speed_rad_s=SPEED,
        raw_same_sign_margin_s=MARGIN_S,stride=STRIDE,
        weighting='preserve each record-axis squared-weight budget and torque normalizer; unchanged blocks are byte-equal numeric arrays',
        filtering='evaluate raw native force balance first; one centered kernel for all target/regression columns, never across an invalid span',
        preprocessing_changed_only='original periodic right-pitch channel; left and single-axis probes unchanged',
        input_roles='existing train only; no A/B, selection, test or evaluation'))
    def assemble(mode):
        matrices=[];targets=[];new_weights=[];offset=0
        for w in weights:
            end=offset+w['rows'];s=slice(offset,end);offset=end
            change=w['stage']=='inertia' and w['joint']==PITCH and (mode=='all_centers' or w['center_id']=='train_c02')
            if change:
                block=replacement[w['trial_id']]
                factor=w['sqrt_weight']*np.sqrt(w['rows']/len(block))
                matrices.append(block[:,:168]*factor);targets.append(block[:,-1]*factor)
                new={**w,'rows':len(block),'sqrt_weight':factor}
                np.testing.assert_allclose(new['rows']*factor**2,w['rows']*w['sqrt_weight']**2,rtol=1e-14,atol=0)
            else:
                matrices.append(original[s]);targets.append(target[s]);new=w
                np.testing.assert_array_equal(matrices[-1],original[s])
                np.testing.assert_array_equal(targets[-1],target[s])
            new_weights.append(new)
        return np.vstack(matrices),np.concatenate(targets),new_weights
    results={}
    for mode in ('c02_only','all_centers'):
        X,t,w=assemble(mode)
        d=output/mode;d.mkdir()
        write_json(d/'weights.json',w)
        schedule=[(1e-4,'full')]
        if mode=='all_centers':
            schedule+=[(1e-4,'omit_'+kind) for key in ('center_id','acquisition_batch_id') for kind in sorted({a[key] for a in w})]
            schedule+=[(ridge,subset) for ridge in (1e-5,1e-3) for subset in ('full','omit_train_c02')]
        for ridge,label in schedule:
            keep=np.ones(len(t),dtype=bool)
            if label.startswith('omit_'):
                value=label.removeprefix('omit_');key='acquisition_batch_id' if 'batch' in value else 'center_id'
                keep=frozen_group_mask(w,key,value)
            state,fit=fit_case(X,t,keep,ridge,coord,nominal,queries)
            path=d/f'ridge_{ridge:g}_{label}';path.mkdir()
            np.savez_compressed(path/'solution.npz',**state)
            write_json(path/'fit.json',fit)
            v=state['vector'];candidate=model(nominal,v,coord['static'])
            # Retain original per-trial scores for transparency. For the changed
            # channel, the new sustained observations below are authoritative
            # only for this diagnostic, not an R1 gate redefinition.
            ordinary,probes=score_problems(problems,candidate,v[:140],v[140:],coord['static'],config)
            sustained={}
            for name,block in replacement.items():
                error=block[:,:168]@v-block[:,-1]
                sustained[name]=dict(rmse_nm=float(np.sqrt(np.mean(error**2))),normalizer_nm=.2,
                                    normalized_rmse=float(np.sqrt(np.mean(error**2)))/.2)
            row=dict(ridge=ridge,label=label,mode=mode,
                mass_query49_kg_m2=float(state['mass'][49,PITCH,PITCH]),
                sustained_predictions=sustained,original_probes=probes,
                original_periodic_scores=ordinary,official_qualification=False)
            if label!='full':
                full=arrays(d/f'ridge_{ridge:g}_full/solution.npz')
                row['operators']=operator_difference(full['mass'],state['mass'],full['torque'],state['torque'],protocol['settings'])
            write_json(path/'diagnostic.json',row)
            results[f'{mode}/ridge_{ridge:g}_{label}']=row
            print(mode,ridge,label,row['mass_query49_kg_m2'],flush=True)
    formal_test_untouched(root)
    write_json(output/'report.json',dict(scope='offline train-only matched sustained observation verification, not R1/R2 qualification',
        fixed_operator_points=len(queries['q']),coverage=infos,results=results,
        ab_observations_used=False,formal_test_consumed=False,truth_read=False,official_qualification=False))


def verify_saved(root,r2,output):
    """Independent native torque/explicit-kernel readback, without refitting."""
    _,config,_=load_protocol(root)
    formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in _read(root/'split_manifest.json')['trials'] if r['split']=='train'})
    protect_observations(root.parent/'motion_pair_capture_v1',set())
    report=_read(output/'report.json');coord=arrays(r2/'coordinates.npz')
    nominal,_=model_for(config);queries=arrays(r2/'operator_probe_set.npz')
    opmax=0.
    for case in report['results'].values():
        folder=output/case['mode']/f"ridge_{case['ridge']:g}_{case['label']}"
        state=arrays(folder/'solution.npz');v=state['vector']
        engine=model(nominal,v,coord['static'])
        parts=engine.evaluate_batch(queries['q'],queries['dq'],queries['ddq'])
        opmax=max(opmax,float(np.max(abs(parts['mass_matrix']-state['mass']))),
                  float(np.max(abs(parts['total']-state['torque']))))
    if opmax>1e-10:raise ValueError('Native saved operator mismatch')
    base=output/'all_centers/ridge_0.0001_full'
    state=arrays(base/'solution.npz');v=state['vector'];engine=model(nominal,v,coord['static'])
    support_set=arrays(output/'raw_sample_indices.npz')
    names=_read(root/'session.json')['joint_names']
    radius=round(MARGIN_S*1e9/config['inertia_sample_period_ns'])
    sigma=SIGMA_MS*1e6/config['inertia_sample_period_ns'];nominal_radius=int(np.ceil(3*sigma))
    stats=[]
    for info in report['coverage']:
        name=info['trial'];meta,_=load_trial(root,name);_,obs=read_record(root,name,meta,names)
        centers=support_set[name];dq=obs['dq'][:,PITCH]
        period=round(meta['conditions']['period_s']*1e9)
        begin=round(meta['knots_s'][meta['labels'].index('periodic')]*1e9)+2*period
        lo,hi=np.searchsorted(obs['sample_ns'],[begin,begin+2*period])
        # Explicit per-offset speed test, not the minimum-filter construction.
        valid=np.arange(lo+radius,hi-radius)
        good=np.ones(len(valid),dtype=bool)
        for offset in range(-radius,radius+1):
            good&=(abs(dq[valid+offset])>SPEED)&(dq[valid+offset]*dq[valid]>0)
        valid=valid[good]
        np.testing.assert_array_equal(centers,valid[(valid-lo)%STRIDE==0])
        w=obs['dq'];dt=config['inertia_sample_period_ns']*1e-9
        acceleration=(w[valid-2]-8*w[valid-1]+8*w[valid+1]-w[valid+2])/(12*dt)
        # The candidate's full native torque is independent of the assembly's
        # 169-column matrix; evaluate at raw states before smoothing.
        torque=engine.evaluate_batch(obs['q'][valid],w[valid],acceleration)['total'][:,PITCH]
        error=torque-obs['current'][valid,PITCH]*config['drive']['gain_nm_a']
        groups=np.r_[0,np.flatnonzero(np.diff(valid)!=1)+1,len(valid)]
        where=np.searchsorted(valid,centers); filtered=[]
        for index in where:
            group=np.searchsorted(groups[1:],index,side='right')
            r=min(nominal_radius,index-groups[group],groups[group+1]-1-index)
            offsets=np.arange(-r,r+1);weights=np.exp(-.5*(offsets/sigma)**2)
            filtered.append(float(weights@error[index+offsets]/weights.sum()))
        actual=float(np.sqrt(np.mean(np.square(filtered))))
        expected=report['results']['all_centers/ridge_0.0001_full']['sustained_predictions'][name]['rmse_nm']
        if abs(actual-expected)>1e-10:raise ValueError('Independent direct-kernel force balance differs')
        stats.append(dict(trial=name,rows=len(centers),native_torque_rmse_nm=actual,
                          difference_from_assembled_rmse_nm=abs(actual-expected)))
    formal_test_untouched(root)
    write_json(output/'verification.json',dict(native_operator_maximum_difference=opmax,
        independently_checked_cases=len(report['results']),explicit_kernel_train_force_balance=stats,
        ab_observations_used=False,formal_test_consumed=False,truth_read=False))
    print('Verified saved operators, sample support and explicit-kernel native force balance',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('root','r2','output'):p.add_argument(arg,type=Path)
    p.add_argument('--verify-only',action='store_true')
    a=p.parse_args()
    (verify_saved if a.verify_only else verify)(a.root,a.r2,a.output)
