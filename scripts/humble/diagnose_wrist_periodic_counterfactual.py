#!/usr/bin/env python3
"""Train-only diagnosis of the wrist R2 regression with two target-swap fits."""
import argparse
import csv
from pathlib import Path

import numpy as np

from design_wrist_supplement import protect_observations
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.rigid_identification import fit_rigid, model_from_parameters
from casbot_calib.staged_attribution import arrays, check_manifest, formal_test_untouched
from casbot_calib.staged_multifit import assemble_training, load_problems
from casbot_calib.staged_protocol import _unseal, file_hash, load_protocol
from casbot_calib.staged_residuals import matched_components
from casbot_calib.staged_stability import frozen_group_mask
from casbot_calib.staged_stability_math import operator_difference, physical_margins
from casbot_calib.staged_wrist_periodic import AXES, BINS

RIDGE = 1e-4
OMIT = 'train_c02'
SUBSETS = ('full', f'omit_{OMIT}')


def sealed_r2(root, analysis):
    protocol = _unseal(analysis, 'analysis_protocol.json')
    if (protocol['parent_protocol_sha256'] != file_hash(root/'protocol.json')
            or protocol['train_manifest_sha256'] != file_hash(root/'role_inputs/train.json')
            or any(file_hash(analysis/p) != d for p,d in protocol['inputs'].items())):
        raise ValueError('R2 is not bound to these training inputs')
    check_manifest(analysis, 'delivery_manifest.json')
    return protocol


def model(nominal, vector, static):
    return model_from_parameters(nominal, vector[:140], vector[140:154], vector[154:], static)


def swap_target(y, x, target, weights, anchor):
    # Solver convention: weighted targets already exclude fixed static torque
    # and rotor torque. The anchor must use the same convention.
    added = np.repeat([w['stage']=='wrist_periodic' for w in weights], [w['rows'] for w in weights])
    if added.shape != target.shape or not added.any():
        raise ValueError('Missing wrist-periodic weighted rows')
    changed = target.copy()
    changed[added] = y[added]@anchor[:140] + x[added]@anchor[140:]
    return changed, added


def spectrum(series):
    """Complex one-sided RMS; used only at the specified nonzero harmonics."""
    return np.fft.rfft(series, axis=0)*np.sqrt(2)/len(series)


def spectral_rows(training, nominal, states, static, harmonics):
    models = {label:model(nominal,state['vector'],static) for label,state in states.items()}
    rows=[]
    for name,row in training.items():
        if row['meta']['stage']!='wrist_periodic':
            continue
        p=row['problem']; c=row['meta']['conditions']
        observed=spectrum(p['current_torque']); acceleration=spectrum(p['ddq'])
        for label,state in states.items():
            v=state['vector']
            parts,prediction,closure=matched_components(models[label],v[:140],v[140:],p,harmonics)
            error=p['current_torque']-prediction
            residual=spectrum(error)
            components={key:spectrum(val) for key,val in parts.items()}
            for joint,own_bin in zip(AXES,BINS):
                for k in BINS:
                    e=residual[k,joint]
                    relative=e*np.exp(-1j*np.angle(acceleration[k,joint])) if k==own_bin else None
                    r=dict(trial_id=name,center=c['center_id'],batch=c['acquisition_batch_id'],model=label,
                        joint=joint,bin=k,own_excitation=k==own_bin,frequency_hz=k/c['period_s'],
                        observed_rms_nm=float(abs(observed[k,joint])),residual_rms_nm=float(abs(e)),
                        residual_real_nm=float(e.real),residual_imag_nm=float(e.imag),
                        residual_phase_vs_acceleration_deg=float(np.angle(relative,deg=True)) if relative is not None else None,
                        residual_in_phase_nm=float(relative.real) if relative is not None else None,
                        residual_quadrature_nm=float(relative.imag) if relative is not None else None,
                        full_period_rmse_nm=float(np.sqrt(np.mean(error[:,joint]**2))),
                        acceleration_bin_rms_rad_s2=float(abs(acceleration[k,joint])),
                        maximum_component_closure_nm=closure['maximum_matched_closure_nm'])
                    for key,a in components.items():
                        z=a[k,joint]
                        r.update({f'{key}_rms_nm':float(abs(z)),f'{key}_real_nm':float(z.real),f'{key}_imag_nm':float(z.imag)})
                    rows.append(r)
    return rows


def diagnose(base_root, base_r2, augmented_root, augmented_r2, output):
    base_root,base_r2,augmented_root,augmented_r2,output = (Path(p).resolve() for p in
        (base_root,base_r2,augmented_root,augmented_r2,output))
    if output.exists() or any(output.is_relative_to(p) for p in (base_root,base_r2,augmented_root,augmented_r2)):
        raise ValueError('Use a fresh output outside the sealed inputs')
    _,base_config,_=load_protocol(base_root)
    _,config,manifest=load_protocol(augmented_root)
    if 'wrist_periodic_train' in base_config['p0'] or 'wrist_periodic_train' not in config['p0']:
        raise ValueError('Expected yaw-only and integrated sessions')
    formal_test_untouched(base_root);formal_test_untouched(augmented_root)
    protect_observations(augmented_root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    protect_observations(base_root,set())
    base=sealed_r2(base_root,base_r2); protocol=sealed_r2(augmented_root,augmented_r2)
    coord=arrays(augmented_r2/'coordinates.npz');probes=arrays(augmented_r2/'operator_probe_set.npz')
    for path,expected in [('coordinates.npz',coord),('operator_probe_set.npz',probes)]:
        before=arrays(base_r2/path)
        if before.keys()!=expected.keys() or any(not np.array_equal(v,before[k]) for k,v in expected.items()):
            raise ValueError('Fixed R2 coordinates or queries differ: '+path)
    if base['settings']!=protocol['settings'] or base['policy']!=protocol['policy']:
        raise ValueError('R2 comparison policies differ')
    states={f'{kind}_{subset}':arrays(r2/'variants'/f'ridge_{RIDGE:g}_{subset}'/'solution.npz')
            for kind,r2 in [('yaw_only',base_r2),('measured',augmented_r2)] for subset in SUBSETS}
    static=coord['static'];nominal,_=model_for(config)
    if not np.array_equal(nominal.rigid_parameters(),coord['nominal_theta']):
        raise ValueError('Loaded model differs from frozen nominal')
    training=load_problems(augmented_root,'train',static)
    y,x,target,weights=assemble_training(training,config)
    if (weights!=_unseal(augmented_r2,'training_setup.json')['weights'] or
            [w for w in weights if w['stage']!='wrist_periodic']!=_unseal(base_r2,'training_setup.json')['weights']):
        raise ValueError('Training weights differ from frozen R2')
    changed,added=swap_target(y,x,target,weights,states['yaw_only_full']['vector'])
    omit=frozen_group_mask(weights,'center_id',OMIT)
    if not omit[added].all():
        raise ValueError('New data assigned to omitted center')
    output.mkdir(parents=True,exist_ok=False)
    for subset,keep in zip(SUBSETS,(np.ones(len(target),dtype=bool),omit)):
        fit=fit_rigid(y[keep],changed[keep],coord['nominal_theta'],
            auxiliary=(x[keep],coord['friction'],coord['lower'],coord['upper']),
            equalities=(coord['equality'],coord['equality_target']),ridge=RIDGE,
            physical_length_scale=.1,lift_residuals=True,fixed_scales=coord['scales'],regularization_metric=coord['metric'])
        vector=np.r_[fit['parameters'],fit['auxiliary_parameters']]
        out=model(nominal,vector,static).evaluate_batch(probes['q'],probes['dq'],probes['ddq'])
        label='target_swap_'+subset
        states[label]=dict(vector=vector,mass=out['mass_matrix'],torque=out['total'])
        np.savez_compressed(output/f'{label}.npz',**states[label])
        write_json(output/f'{label}_fit.json',fit)
        print('Finished target swap:',subset,flush=True)

    comparisons={};signed={}
    settings=protocol['settings']
    for kind in ('yaw_only','measured','target_swap'):
        full,cut=(states[f'{kind}_{s}'] for s in SUBSETS)
        ref=np.diagonal(full['mass'],axis1=1,axis2=2)
        delta=np.diagonal(cut['mass']-full['mass'],axis1=1,axis2=2)
        relative=delta/np.maximum(ref,settings['mass_diagonal_floor_kg_m2'])
        i,j=np.unravel_index(np.argmax(abs(relative)),relative.shape)
        comparisons[kind]=dict(operators=operator_difference(full['mass'],cut['mass'],full['torque'],cut['torque'],settings),
            worst_probe_index=int(i),worst_joint=int(j),signed_relative_by_probe=relative.tolist(),
            margins={s:physical_margins(state['vector'][:140],state['vector'][140:],coord['lower'],coord['upper'])
                     for s,state in [('full',full),('omit',cut)]})
        signed[kind]=(ref,delta)
    i,j=(comparisons['measured'][k] for k in ('worst_probe_index','worst_joint'))
    fixed_point={k:dict(full_kg_m2=float(a[i,j]),omit_kg_m2=float(a[i,j]+b[i,j]),signed_delta_kg_m2=float(b[i,j]),
        signed_relative=float(b[i,j]/max(a[i,j],settings['mass_diagonal_floor_kg_m2']))) for k,(a,b) in signed.items()}
    spectral=spectral_rows(training,nominal,states,static,config['p0']['preprocessing']['harmonics'])
    with (output/'frequency_residuals.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(spectral[0]));writer.writeheader();writer.writerows(spectral)
    own={}
    for label in states:
        own[label]={}
        for axis in AXES:
            rows=[r for r in spectral if r['model']==label and r['joint']==axis and r['own_excitation']]
            own[label][str(axis)]=dict(
                rms_over_trials_nm=float(np.sqrt(np.mean([r['residual_rms_nm']**2 for r in rows]))),
                mean_in_phase_nm=float(np.mean([r['residual_in_phase_nm'] for r in rows])),
                mean_quadrature_nm=float(np.mean([r['residual_quadrature_nm'] for r in rows])),
                mean_rigid_inertia_rms_nm=float(np.mean([r['rigid_inertia_rms_nm'] for r in rows])))
    formal_test_untouched(base_root);formal_test_untouched(augmented_root)
    report=dict(scope='train-only counterfactual diagnosis, no qualification',base_r2=str(base_r2),augmented_r2=str(augmented_r2),
        ridge=RIDGE,omitted_center=OMIT,formal_test_consumed=False,selection_observations_used=False,truth_read=False,
        changed_rows=int(added.sum()),unchanged_rows=int((~added).sum()),counterfactual_fits=2,
        fixed_operator_probes=len(probes['q']),comparisons=comparisons,
        measured_worst_point=dict(probe_index=i,joint_index=j,positions_rad=probes['q'][i].tolist(),fixed_point_comparison=fixed_point),
        weighted_target_swap_rms=float(np.sqrt(np.mean((changed[added]-target[added])**2))),
        residual_convention='observed minus predicted; complex RMS, own tone relative to position-derived acceleration',
        own_excitation=own)
    write_json(output/'report.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('base_root','base_r2','augmented_root','augmented_r2','output'):
        p.add_argument(name,type=Path)
    report=diagnose(**vars(p.parse_args()))
    print({k:v['operators']['maximum_mass_diagonal_relative'] for k,v in report['comparisons'].items()})
