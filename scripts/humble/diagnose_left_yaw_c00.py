#!/usr/bin/env python3
"""Locate train_c00 information behind the remaining left-yaw leave-center sensitivity."""
import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.linalg import solve

from design_wrist_supplement import mass_rows, protect_observations
from diagnose_right_pitch_balance import diagnostic_fit
from diagnose_wrist_periodic_counterfactual import RIDGE, sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_multifit import assemble_training, load_problems
from casbot_calib.staged_protocol import _read, _unseal, load_protocol
from casbot_calib.staged_right_pitch import right_companion
from casbot_calib.staged_wrist_periodic import companion

JOINT=4
CENTER='train_c00'


def run(root,r2,output):
    root,r2,output=(p.resolve() for p in (root,r2,output))
    if output.exists() or any(output.is_relative_to(p) for p in (root,r2)):
        raise ValueError('Use a fresh diagnostic directory outside inputs')
    _,config,manifest=load_protocol(root);formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    for reader in (companion,right_companion):
        cap,names=reader(root);protect_observations(cap,{(n,) for n in names})
    protect_observations(root.parent.parent/'dvt2_004_right_pitch_integration_v1/motion_pair_capture_v1',set())
    settings=sealed_r2(root,r2)['settings']
    coord=arrays(r2/'coordinates.npz');queries=arrays(r2/'operator_probe_set.npz')
    full=arrays(r2/'variants/ridge_0.0001_full/solution.npz')
    cut=arrays(r2/'variants/ridge_0.0001_omit_train_c00/solution.npz')
    diff=(cut['mass'][:,JOINT,JOINT]-full['mass'][:,JOINT,JOINT])/np.maximum(full['mass'][:,JOINT,JOINT],settings['mass_diagonal_floor_kg_m2'])
    index=int(np.argmax(abs(diff)))
    nominal,_=model_for(config)
    train=load_problems(root,'train',coord['static'])
    y,x,target,weights=assemble_training(train,config)
    setup=_unseal(r2,'training_setup.json')
    if weights!=setup['weights']:raise ValueError('Original frozen row weights changed')
    X=np.column_stack((y,x));del y,x
    basis=arrays(r2/'comparison_basis.npz')['basis']
    transform=coord['scales'][:,None]*basis
    Z=X@transform
    residual=target-X@cut['vector'];full_residual=target-X@full['vector']
    scale=.02;curvature=np.hypot(1,residual/scale)**-3
    H=Z.T@(curvature[:,None]*Z)+.2**2*RIDGE*basis.T@coord['metric']@basis
    g=mass_rows(nominal,queries['q'][[index]],JOINT)[0]@transform[:140]
    leverage=Z@solve(H,g,assume_a='pos'); del Z
    scores=leverage*residual/np.hypot(1,residual/scale)
    groups=defaultdict(lambda:dict(score=0.,pressure=0.,rows=0,indices=[],trials=[]))
    start=0
    for w in weights:
        stop=start+w['rows'];ix=np.arange(start,stop);start=stop
        if w['center_id']!=CENTER:continue
        key=(w['stage'],w['joint'])
        group=groups[key];group['score']+=float(scores[ix].sum())
        group['pressure']+=float((residual[ix]/np.hypot(1,residual[ix]/scale))@(residual[ix]-full_residual[ix]))
        group['rows']+=len(ix);group['indices'].append(ix);group['trials'].append(w['trial_id'])
    ranked=sorted(groups,key=lambda key:abs(groups[key]['score']),reverse=True)
    output.mkdir(parents=True,exist_ok=False)
    restored=np.zeros(len(target),dtype=bool)
    all_center=np.zeros(len(target),dtype=bool)
    for key,row in groups.items():
        ix=np.concatenate(row['indices']);all_center[ix]=True
        if key==ranked[0]:restored[ix]=True
    summary=dict(query_index=index,query_q_rad=queries['q'][index].tolist(),
        full_mass_kg_m2=float(full['mass'][index,JOINT,JOINT]),omit_mass_kg_m2=float(cut['mass'][index,JOINT,JOINT]),
        signed_relative=float(diff[index]),joint=JOINT,groups=[
            dict(stage=k[0],joint=k[1],score_kg_m2=groups[k]['score'],
                 directional_pressure=groups[k]['pressure'],rows=groups[k]['rows'],trials=groups[k]['trials']) for k in ranked],
        restorations={},ab_observations_used=False,formal_test_consumed=False,truth_read=False)
    for label,mask in [('restore_top_stage_axis',~all_center|restored),('restore_complement',~restored)]:
        state,fit=diagnostic_fit(X[mask],target[mask],coord,nominal,queries)
        np.savez_compressed(output/f'{label}.npz',**state);write_json(output/f'{label}_fit.json',fit)
        mass=float(state['mass'][index,JOINT,JOINT])
        summary['restorations'][label]=dict(mass_kg_m2=mass,
            fraction_restored_toward_full=(mass-summary['omit_mass_kg_m2'])/(summary['full_mass_kg_m2']-summary['omit_mass_kg_m2']))
        print(label,mass,flush=True)
    # Report the actual train feature coverage behind the leading block.
    coverage=[]
    leading=set(groups[ranked[0]]['trials'])
    for name in sorted(leading):
        p=train[name]['problem'];meta=train[name]['meta'];stage=meta['stage']
        if stage=='wrist_periodic':
            # Use actual acquired reference-independent coverage already built
            # by the shared consumer; its q is intentionally not replaced.
            c=train[name]['coverage']
            coverage.append(dict(trial=name,stage=stage,
                wrist_min_rad=c['minimum_rad'][4:7],wrist_max_rad=c['maximum_rad'][4:7],
                wrist_mean_rad=c['mean_rad'][4:7]))
        else:
            q=p['q'];v=p['dq']
            use=p['moving'][:,JOINT] if stage=='inertia' else np.ones(len(q),dtype=bool)
            coverage.append(dict(trial=name,stage=stage,wrist_min_rad=q[use,4:7].min(axis=0),
                                wrist_max_rad=q[use,4:7].max(axis=0),wrist_mean_rad=q[use,4:7].mean(axis=0)))
    summary['leading_block_coverage']=coverage
    formal_test_untouched(root);write_json(output/'report.json',summary)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('root','r2','output'):p.add_argument(name,type=Path)
    run(**vars(p.parse_args()))
