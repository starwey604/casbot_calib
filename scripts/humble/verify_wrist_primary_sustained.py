#!/usr/bin/env python3
"""Discriminate a uniform raw sustained rule on original wrist torque channels."""
import argparse
from pathlib import Path
import numpy as np

from design_wrist_supplement import protect_observations
from diagnose_wrist_periodic_counterfactual import sealed_r2
from verify_right_pitch_sustained import fit_case
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.staged_attribution import arrays,formal_test_untouched
from casbot_calib.staged_collection import load_trial
from casbot_calib.staged_identification import read_record
from casbot_calib.staged_multifit import assemble_training,load_problems,score_problems
from casbot_calib.staged_pitch_sustained import support,problem as sustained
from casbot_calib.staged_protocol import _read,_unseal,load_protocol
from casbot_calib.staged_right_pitch import right_companion
from casbot_calib.staged_stability import frozen_group_mask
from casbot_calib.staged_stability_math import operator_difference
from casbot_calib.staged_wrist_periodic import companion

AXES=(4,5,6,11,12,13)

def run(root,r2,output):
    root,r2,output=(p.resolve() for p in (root,r2,output))
    if output.exists() or any(output.is_relative_to(p) for p in (root,r2)):raise ValueError('Use a fresh independent output')
    _,config,manifest=load_protocol(root);formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    for reader in (companion,right_companion):
        cap,names=reader(root);protect_observations(cap,{(n,) for n in names})
    protect_observations(root.parent.parent/'dvt2_004_right_pitch_integration_v1/motion_pair_capture_v1',set())
    policy=sealed_r2(root,r2)
    coord=arrays(r2/'coordinates.npz');queries=arrays(r2/'operator_probe_set.npz')
    nominal,_=model_for(config)
    train=load_problems(root,'train',coord['static'])
    y,x,t,weights=assemble_training(train,config);X=np.column_stack((y,x));del y,x
    if weights!=_unseal(r2,'training_setup.json')['weights']:raise ValueError('Changed training weights')
    replacements={};indices={};coverage=[]
    names=_read(root/'session.json')['joint_names']
    for name,row in train.items():
        if row['meta']['stage']!='inertia':continue
        meta,_=load_trial(root,name);_,obs=read_record(root,name,meta,names)
        for J in AXES:
            valid,centers=support(obs,meta,config['inertia_sample_period_ns'],joint=J)
            p=sustained(nominal,obs,meta,config['drive']['gain_nm_a'],coord['static'],
                        config['inertia_sample_period_ns'],centers,joint=J)
            replacements[name,J]=p;indices[f'{name}__j{J:02}']=centers
            coverage.append(dict(trial=name,joint=J,rows=len(centers),eligible_raw_samples=len(valid),
                minimum_raw_speed_rad_s=float(np.min(abs(obs['dq'][valid,J]))),
                wrist_range_rad=[[float(obs['q'][centers,k].min()),float(obs['q'][centers,k].max())]
                                 for k in ((4,5,6) if J<7 else (11,12,13))]))
    output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'raw_sample_indices.npz',**indices)
    results={}
    for mode in ('all_centers',):
        matrices=[];targets=[];newweights=[];offset=0
        for w in weights:
            s=slice(offset,offset+w['rows']);offset+=w['rows']
            change=w['stage']=='inertia' and w['joint'] in AXES and (mode=='all_centers' or w['center_id']=='train_c00')
            if change:
                p=replacements[w['trial_id'],w['joint']];count=len(p['target']);factor=w['sqrt_weight']*np.sqrt(w['rows']/count)
                matrices.append(np.column_stack((p['regression'],p['extra']))*factor);targets.append(p['target']*factor)
                new={**w,'rows':count,'sqrt_weight':factor}
                np.testing.assert_allclose(count*factor**2,w['rows']*w['sqrt_weight']**2,rtol=1e-14)
            else:matrices.append(X[s]);targets.append(t[s]);new=w
            newweights.append(new)
        A=np.vstack(matrices);target=np.concatenate(targets)
        d=output/mode;d.mkdir();write_json(d/'weights.json',newweights)
        cases=[(1e-4,'full')]
        if mode=='all_centers':
            cases+=[(1e-4,'omit_'+g) for k in ('center_id','acquisition_batch_id') for g in sorted({w[k] for w in weights})]
            cases+=[(r,s) for r in (1e-5,1e-3) for s in ('full','omit_train_c00','omit_train_c02')]
        for ridge,label in cases:
            keep=np.ones(len(target),bool)
            if label!='full':
                val=label.removeprefix('omit_');k='acquisition_batch_id' if 'batch' in val else 'center_id'
                keep=frozen_group_mask(newweights,k,val)
            state,fit=fit_case(A,target,keep,ridge,coord,nominal,queries)
            path=d/f'ridge_{ridge:g}_{label}';path.mkdir()
            np.savez_compressed(path/'solution.npz',**state);write_json(path/'fit.json',fit)
            row=dict(ridge=ridge,label=label,mode=mode,mass_query19_kg_m2=float(state['mass'][19,4,4]))
            if label!='full':
                ref=arrays(d/f'ridge_{ridge:g}_full/solution.npz')
                row['operators']=operator_difference(ref['mass'],state['mass'],ref['torque'],state['torque'],policy['settings'])
            # Keep original scores intact; this is a diagnostic, not a new
            # R1 contract. Separately record the new sustained-channel score.
            from diagnose_wrist_periodic_counterfactual import model
            v=state['vector'];engine=model(nominal,v,coord['static'])
            primary,probes=score_problems(train,engine,v[:140],v[140:],coord['static'],config)
            row.update(original_primary_passed=all(x['passed'] for x in primary.values()),
                original_probes_passed=all(x['passed'] for x in probes.values()),
                maximum_probe_inertia_disagreement=max(x['relative_inertia_disagreement'] for x in probes.values()),
                sustained_rmse_nm={str(n):float(np.sqrt(np.mean((p['regression']@v[:140]+p['extra']@v[140:]-p['target'])**2))) for n,p in replacements.items()})
            results[mode+'/'+path.name]=row
            print(mode,ridge,label,row['mass_query19_kg_m2'],row.get('operators',{}).get('maximum_mass_diagonal_relative'),flush=True)
    formal_test_untouched(root)
    write_json(output/'report.json',dict(scope='uniform per-axis raw sustained processing of six original wrist torque channels; train only',
        coverage=coverage,results=results,ab_observations_used=False,formal_test_consumed=False,truth_read=False))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for a in ('root','r2','output'):p.add_argument(a,type=Path)
    run(**vars(p.parse_args()))
