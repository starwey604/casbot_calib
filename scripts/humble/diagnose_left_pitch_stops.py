#!/usr/bin/env python3
"""Train-only raw-state target interventions on c00's original left-pitch row."""
from pathlib import Path
import argparse
import numpy as np

from diagnose_c02_pitch_phase import transition_targets
from diagnose_right_pitch_balance import diagnostic_fit
from diagnose_wrist_periodic_counterfactual import model,sealed_r2
from design_wrist_supplement import protect_observations
from casbot_calib.staged_protocol import load_protocol,_read,_unseal
from casbot_calib.staged_multifit import load_problems,assemble_training
from casbot_calib.staged_attribution import arrays,formal_test_untouched
from casbot_calib.staged_right_pitch import right_companion
from casbot_calib.staged_wrist_periodic import companion
from casbot_calib.model_runtime import model_for
from casbot_calib.data import write_json

def run(root,r2,output):
    root,r2,output=(p.resolve() for p in (root,r2,output))
    if output.exists() or any(output.is_relative_to(p) for p in (root,r2)):raise ValueError('Fresh output required')
    _,config,manifest=load_protocol(root);formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    for reader in (companion,right_companion):
        cap,names=reader(root);protect_observations(cap,{(n,) for n in names})
    sealed_r2(root,r2)
    coord=arrays(r2/'coordinates.npz');q=arrays(r2/'operator_probe_set.npz')
    full=arrays(r2/'variants/ridge_0.0001_full/solution.npz')
    cut=arrays(r2/'variants/ridge_0.0001_omit_train_c00/solution.npz')
    nominal,_=model_for(config);base=model(nominal,cut['vector'],coord['static'])
    train=load_problems(root,'train',coord['static'])
    y,x,t,weights=assemble_training(train,config);X=np.column_stack((y,x));del y,x
    if weights!=_unseal(r2,'training_setup.json')['weights']:raise ValueError('Frozen weights changed')
    pieces={s:np.zeros(len(t)) for s in ('stopped','near_transition','sustained')};reports={};offset=0
    names=_read(root/'session.json')['joint_names']
    for w in weights:
        s=slice(offset,offset+w['rows']);offset+=w['rows']
        if (w['center_id'],w['stage'],w['joint'])!=('train_c00','inertia',5):continue
        name=w['trial_id']
        blocks,row=transition_targets(root,name,config,names,base,cut['vector'],coord['static'],train[name]['problem'],joint=5)
        reports[name]=row
        for key,b in blocks.items():pieces[key][s]=b*w['sqrt_weight']
    output.mkdir(parents=True,exist_ok=False)
    result=dict(base_mass_kg_m2=float(cut['mass'][19,4,4]),full_mass_kg_m2=float(full['mass'][19,4,4]),
        records=reports,cases={},ab_observations_used=False,formal_test_consumed=False)
    for label,change in [('neutralize_stopped',pieces['stopped']),('neutralize_sustained',pieces['sustained'])]:
        state,fit=diagnostic_fit(X,t-change,coord,nominal,q)
        np.savez_compressed(output/f'{label}.npz',**state);write_json(output/f'{label}_fit.json',fit)
        result['cases'][label]=dict(mass_kg_m2=float(state['mass'][19,4,4]))
        print(label,result['cases'][label],flush=True)
    formal_test_untouched(root);write_json(output/'report.json',result)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('root','r2','output'):p.add_argument(arg,type=Path)
    run(**vars(p.parse_args()))
