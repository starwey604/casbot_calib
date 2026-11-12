#!/usr/bin/env python3
"""Refit two fixed R2 variants using only raw sustained-motion companion rows."""
import argparse
from pathlib import Path

import numpy as np

from design_wrist_supplement import protect_observations
from diagnose_wrist_periodic_counterfactual import RIDGE, OMIT, SUBSETS, model, sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.rigid_identification import fit_rigid
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_multifit import assemble_training, load_problems
from casbot_calib.staged_protocol import _read, _unseal, load_protocol
from casbot_calib.staged_stability import frozen_group_mask
from casbot_calib.staged_stability_math import operator_difference
from casbot_calib.staged_wrist_periodic import AXES, wrist_records

from casbot_calib.staged_wrist_local import MIN_SPEED, STRIDE, raw_sustained_support, sustained_samples


def make_problem(nominal, raw, meta, gain, static, step):
    keep,a,info=sustained_samples(raw,meta,step)
    q,v=raw['q'][keep],raw['dq'][keep]
    tau=raw['current'][keep]*gain
    terms=nominal.regressor_batch(q,v,a)
    s=np.sign(v);decay=np.exp(-(v/.002)**2)
    aux=np.zeros((len(keep),14,28))
    for j in AXES:
        aux[:,j,j]=v[:,j];aux[:,j,14+j]=(s*(1-decay))[:,j]
    fixed=s*decay*static
    # All terms are on the very same raw sample indices. Do not project a
    # complete period, shift torque timestamps, or impute reactions at zero.
    return dict(regression=terms['regressor'],extra=aux,
        target=tau-terms['rotor_torque']-fixed,moving=np.ones_like(v,dtype=bool)),keep,info


def run(base_root, base_r2, integrated_root, integrated_r2, output):
    base_root,base_r2,integrated_root,integrated_r2,output=(Path(p).resolve() for p in
        (base_root,base_r2,integrated_root,integrated_r2,output))
    if output.exists() or any(output.is_relative_to(p) for p in (base_root,base_r2,integrated_root,integrated_r2)):
        raise ValueError('Use a new output outside all input sessions')
    _,_,manifest=load_protocol(base_root)
    _,config,_=load_protocol(integrated_root)
    formal_test_untouched(base_root);formal_test_untouched(integrated_root)
    protect_observations(base_root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    protect_observations(integrated_root,set())
    base_protocol=sealed_r2(base_root,base_r2);protocol=sealed_r2(integrated_root,integrated_r2)
    coord=arrays(integrated_r2/'coordinates.npz');probes=arrays(integrated_r2/'operator_probe_set.npz')
    for path,expected in [('coordinates.npz',coord),('operator_probe_set.npz',probes)]:
        before=arrays(base_r2/path)
        if before.keys()!=expected.keys() or any(not np.array_equal(v,before[k]) for k,v in expected.items()):
            raise ValueError('Frozen comparison arrays differ: '+path)
    if base_protocol['settings']!=protocol['settings'] or base_protocol['policy']!=protocol['policy']:
        raise ValueError('Original R2 comparison policy changed')
    parent_train=_unseal(base_root,'role_inputs/train.json')['records']
    integrated_train=_unseal(integrated_root,'role_inputs/train.json')['records']
    if parent_train!=integrated_train:
        raise ValueError('Baseline train inputs differ from the integrated session')
    static=coord['static'];nominal,_=model_for(config)
    if not np.array_equal(nominal.rigid_parameters(),coord['nominal_theta']):
        raise ValueError('Loaded model differs from frozen nominal')
    problems=load_problems(base_root,'train',static)
    selections={};support={}
    for name,meta,raw in wrist_records(integrated_root):
        p,keep,info=make_problem(nominal,raw,meta,config['drive']['gain_nm_a'],static,config['inertia_sample_period_ns'])
        problems[name]=dict(meta=meta,problem=p)
        selections[name]=keep;support[name]=info
    if len(support)!=4:
        raise ValueError('Expected exactly four sealed companion trials')
    y,x,target,weights=assemble_training(problems,config)
    del problems
    w_base=[w for w in weights if w['stage']!='wrist_periodic']
    before=_unseal(integrated_r2,'training_setup.json')['weights']
    if w_base!=[w for w in before if w['stage']!='wrist_periodic'] or w_base!=_unseal(base_r2,'training_setup.json')['weights']:
        raise ValueError('Original training row weights changed')
    old={ (w['trial_id'],w['joint']):w for w in before if w['stage']=='wrist_periodic' }
    new=[w for w in weights if w['stage']=='wrist_periodic']
    if len(new)!=12 or any((w['trial_id'],w['joint']) not in old for w in new):
        raise ValueError('Wrist trial/axis allocation changed')
    for w in new:
        z=old[w['trial_id'],w['joint']]
        if (any(w[k]!=z[k] for k in ('center_id','acquisition_batch_id','torque_normalizer_nm'))
                or not np.isclose(w['rows']*w['sqrt_weight']**2,z['rows']*z['sqrt_weight']**2,rtol=1e-14)):
            raise ValueError('Wrist objective budget changed')
    omit=frozen_group_mask(weights,'center_id',OMIT)
    wrist_mask=np.repeat([w['stage']=='wrist_periodic' for w in weights],[w['rows'] for w in weights])
    output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'raw_sample_indices.npz',**selections)
    write_json(output/'support.json',support)
    states={};metrics={}
    for label, keep in zip(SUBSETS,(np.ones(len(target),dtype=bool),omit)):
        fit=fit_rigid(y[keep],target[keep],coord['nominal_theta'],
            auxiliary=(x[keep],coord['friction'],coord['lower'],coord['upper']),
            equalities=(coord['equality'],coord['equality_target']),ridge=RIDGE,
            physical_length_scale=.1,lift_residuals=True,fixed_scales=coord['scales'],
            regularization_metric=coord['metric'])
        vector=np.r_[fit['parameters'],fit['auxiliary_parameters']]
        values=model(nominal,vector,static).evaluate_batch(probes['q'],probes['dq'],probes['ddq'])
        state=dict(vector=vector,mass=values['mass_matrix'],torque=values['total']);states[label]=state
        np.savez_compressed(output/f'{label}.npz',**state)
        write_json(output/f'{label}_fit.json',fit)
        residual=y[wrist_mask]@vector[:140]+x[wrist_mask]@vector[140:]-target[wrist_mask]
        begin=0; by_block=[]
        for w in new:
            end=begin+w['rows']
            by_block.append(dict(trial_id=w['trial_id'],joint=w['joint'],
                rmse_nm=float(np.sqrt(np.mean(residual[begin:end]**2))/w['sqrt_weight'])))
            begin=end
        metrics[label]=dict(weighted_fit_rows=int(keep.sum()),sustained_training_error=by_block,
            gravity_equality_error_nm=float(np.max(abs(coord['equality']@vector[:140]-coord['equality_target']))))
        print('Finished sustained-motion fit:',label,flush=True)

    comparisons={};deltas={}
    settings=protocol['settings']
    for label,r2 in [('yaw_only',base_r2),('measured_periodic',integrated_r2),('sustained_raw',None)]:
        full,cut=(states[k] if r2 is None else arrays(r2/'variants'/f'ridge_{RIDGE:g}_{k}'/'solution.npz') for k in SUBSETS)
        ref=np.diagonal(full['mass'],axis1=1,axis2=2);delta=np.diagonal(cut['mass']-full['mass'],axis1=1,axis2=2)
        relative=delta/np.maximum(ref,settings['mass_diagonal_floor_kg_m2'])
        i,j=np.unravel_index(np.argmax(abs(relative)),relative.shape)
        comparisons[label]=dict(operators=operator_difference(full['mass'],cut['mass'],full['torque'],cut['torque'],settings),
            worst_probe_index=int(i),worst_joint_index=int(j),signed_relative_by_probe=relative.tolist())
        deltas[label]=relative
    i,j=(comparisons['measured_periodic'][k] for k in ('worst_probe_index','worst_joint_index'))
    formal_test_untouched(base_root);formal_test_untouched(integrated_root)
    report=dict(scope='two train-only diagnostic refits, not complete R2 qualification',ridge=RIDGE,omitted_center=OMIT,
        fixed_operator_probes=len(probes['q']),unchanged_old_rows=int((~wrist_mask).sum()),
        added_raw_rows=int(wrist_mask.sum()),sample_period_ns=config['inertia_sample_period_ns'],
        stride=STRIDE,minimum_speed_rad_s=MIN_SPEED,raw_support=support,comparisons=comparisons,
        same_measured_worst_point=dict(probe_index=i,joint_index=j,
            signed_relative_by_case={k:float(v[i,j]) for k,v in deltas.items()}),
        fit_metrics=metrics,formal_test_consumed=False,selection_observations_used=False,truth_read=False,
        convention='Only companion records changed; raw torque, q, dq and locally differentiated dq at common raw sample centers; no full-cycle projection')
    write_json(output/'report.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('base_root','base_r2','integrated_root','integrated_r2','output'):
        p.add_argument(name,type=Path)
    result=run(**vars(p.parse_args()))
    print({k:v['operators']['maximum_mass_diagonal_relative'] for k,v in result['comparisons'].items()})
