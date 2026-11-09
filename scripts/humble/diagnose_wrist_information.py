#!/usr/bin/env python3
"""Train-only R2 weak-direction diagnostics; no refit, selection or plant truth."""
import argparse,json,os,sys
from pathlib import Path
import numpy as np
from casbot_calib.data import write_json
from casbot_calib.staged_protocol import load_protocol
from casbot_calib.staged_multifit import load_problems,assemble_training
from casbot_calib.staged_attribution import check_manifest,formal_test_untouched


def diagnose(root,analysis,output):
    def deny_truth(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
            raise RuntimeError('Plant truth is not a train-information input')
    sys.addaudithook(deny_truth)
    root=Path(root).resolve();analysis=Path(analysis).resolve();output=Path(output).resolve()
    if output.exists():raise FileExistsError(output)
    formal_test_untouched(root);check_manifest(analysis,'delivery_manifest.json')
    protocol,config,_=load_protocol(root)
    with np.load(analysis/'coordinates.npz') as a:static=a['static'].copy()
    train=load_problems(root,'train',static);y,x,target,weights=assemble_training(train,config)
    rows=[];centers={}
    for center in sorted({w['center_id'] for w in weights}):
        q=np.array([p['coverage']['mean_rad'] for p in train.values() if p['meta']['conditions']['center_id']==center])
        centers[center]=dict(mean_rad=q.mean(axis=0).tolist(),right_child_wrist_pitch_roll_rad=q.mean(axis=0)[12:14].tolist())
    summary=json.loads((analysis/'report.json').read_text())
    for ridge in config['p0']['selection']['ridge_candidates']:
        folder=analysis/'variants'/f'ridge_{ridge:g}_full'
        with np.load(folder/'solution.npz') as a:full={k:a[k].copy() for k in a.files}
        for variant in (r for r in summary['variants'] if r['ridge']==ridge and r['kind']=='center'):
            with np.load(analysis/'variants'/variant['label']/'solution.npz') as a:other={k:a[k].copy() for k in a.files}
            delta=other['vector']-full['vector'];rigid=y@delta[:140];friction=x@delta[140:];combined=rigid+friction
            diag=np.diagonal(full['mass'],axis1=1,axis2=2);change=np.diagonal(other['mass']-full['mass'],axis1=1,axis2=2)
            relative=abs(change)/np.maximum(diag,config['p0']['stability']['mass_diagonal_floor_kg_m2']);probe,j=np.unravel_index(relative.argmax(),relative.shape)
            row=dict(variant=variant['label'],joint_index=int(j),maximum_diagonal_relative=float(relative[probe,j]),
                reference_diagonal_kg_m2=float(diag[probe,j]),change_diagonal_kg_m2=float(change[probe,j]),
                weighted_rigid_change_norm=float(np.linalg.norm(rigid)),weighted_friction_change_norm=float(np.linalg.norm(friction)),
                weighted_combined_change_norm=float(np.linalg.norm(combined)),
                rigid_friction_cosine=float(rigid@friction/max(np.linalg.norm(rigid)*np.linalg.norm(friction),1e-30)),axis_blocks=[])
            begin=0
            for w in weights:
                end=begin+w['rows']
                if w['joint'] in (4,11):
                    row['axis_blocks'].append(dict(trial=w['trial_id'],joint=w['joint'],stage=w['stage'],center=w['center_id'],
                        rigid_change_rms_nm=float(np.sqrt(np.mean(rigid[begin:end]**2)))/w['sqrt_weight'],
                        combined_change_rms_nm=float(np.sqrt(np.mean(combined[begin:end]**2)))/w['sqrt_weight']))
                begin=end
            rows.append(row)
    # The declared signed probe: expected acceleration versus independent 30Hz
    # matched-band residual scale; source is observations/known trajectory only.
    probes=[]
    for name,r in train.items():
        if r['meta']['stage']!='inertia_probe':continue
        p=r['problem'];j=p['joint'];probes.append(dict(trial=name,joint=j,
            measured_acceleration_rms_rad_s2=float(np.sqrt(np.mean(p['ddq'][:,j]**2))),
            minimum_signed_speed_rad_s=float(np.min(r['meta']['conditions']['direction']*p['dq'][:,j])),
            held_wrist_pitch_roll_rad=np.mean(p['q'],axis=0)[[5,6,12,13]].tolist()))
    report=dict(scope='Same sealed train observations and completed R2 candidate differences; no truth or selection values, no fit',
        truth_read=False,selection_values_read=False,formal_test_consumed=False,parameter_refit=False,
        original_r2_passed=summary['passed'],centers=centers,variants=rows,observed_wrist_probes=probes)
    output.mkdir();write_json(output/'report.json',report)
    print(json.dumps(dict(centers=centers,worst=[{k:r[k] for k in ('variant','joint_index','maximum_diagonal_relative','reference_diagonal_kg_m2','change_diagonal_kg_m2','rigid_friction_cosine')} for r in rows]),ensure_ascii=False))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--analysis',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();diagnose(a.root,a.analysis,a.output)
