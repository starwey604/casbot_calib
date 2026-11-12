#!/usr/bin/env python3
"""Same-support, matched local bandwidth sensitivity of the two wrist fits."""
import argparse
from pathlib import Path

import numpy as np

from design_wrist_supplement import protect_observations
from diagnose_wrist_periodic_counterfactual import RIDGE, OMIT, SUBSETS, model, sealed_r2
from refit_wrist_sustained import make_problem
from casbot_calib.staged_wrist_local import local_kernel, raw_rows, raw_sustained_support
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.rigid_identification import fit_rigid
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_multifit import assemble_training, load_problems
from casbot_calib.staged_protocol import _read, _unseal, load_protocol
from casbot_calib.staged_stability import frozen_group_mask
from casbot_calib.staged_stability_math import operator_difference
from casbot_calib.staged_wrist_periodic import AXES, wrist_records

SIGMAS_MS = (.5, 1.5, 4.)


def evaluate(full,omitted,settings):
    full_diag=np.diagonal(full['mass'],axis1=1,axis2=2)
    signed=(np.diagonal(omitted['mass'],axis1=1,axis2=2)-full_diag)/np.maximum(full_diag,settings['mass_diagonal_floor_kg_m2'])
    i,j=np.unravel_index(np.argmax(abs(signed)),signed.shape)
    return dict(operators=operator_difference(full['mass'],omitted['mass'],full['torque'],omitted['torque'],settings),
                worst_probe_index=int(i),worst_joint_index=int(j),signed_relative_by_probe=signed.tolist())


def compare(base_root, base_r2, integrated, integrated_r2, baseline, output):
    base_root,base_r2,integrated,integrated_r2,baseline,output=(Path(p).resolve() for p in
        (base_root,base_r2,integrated,integrated_r2,baseline,output))
    if output.exists() or any(output.is_relative_to(p) for p in (base_root,base_r2,integrated,integrated_r2,baseline)):
        raise ValueError('Use a fresh output outside sealed inputs')
    _,_,manifest=load_protocol(base_root)
    _,config,_=load_protocol(integrated)
    formal_test_untouched(base_root);formal_test_untouched(integrated)
    protect_observations(base_root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    protect_observations(integrated,set())
    base=sealed_r2(base_root,base_r2);protocol=sealed_r2(integrated,integrated_r2)
    coord=arrays(integrated_r2/'coordinates.npz');probes=arrays(integrated_r2/'operator_probe_set.npz')
    for path,expected in [('coordinates.npz',coord),('operator_probe_set.npz',probes)]:
        original=arrays(base_r2/path)
        if original.keys()!=expected.keys() or any(not np.array_equal(x,original[k]) for k,x in expected.items()):
            raise ValueError('Frozen comparison arrays differ: '+path)
    if (base['settings']!=protocol['settings'] or base['policy']!=protocol['policy']
            or _unseal(base_root/'role_inputs','train.json')['records']!=_unseal(integrated/'role_inputs','train.json')['records']):
        raise ValueError('R2 settings or original train records differ')
    baseline_report=_read(baseline/'report.json')
    frozen_indices=arrays(baseline/'raw_sample_indices.npz')
    settings=protocol['settings']
    no_filter=evaluate(*(arrays(baseline/f'{label}.npz') for label in SUBSETS),settings)
    if not np.isclose(no_filter['operators']['maximum_mass_diagonal_relative'],
                      baseline_report['comparisons']['sustained_raw']['operators']['maximum_mass_diagonal_relative'],rtol=1e-12):
        raise ValueError('Saved unsmoothed baseline does not reproduce its reported metric')

    static=coord['static'];nominal,_=model_for(config);step=config['inertia_sample_period_ns']
    if not np.array_equal(nominal.rigid_parameters(),coord['nominal_theta']):
        raise ValueError('Nominal model changed')
    original=load_problems(base_root,'train',static)
    y0,x0,t0,old_weights=assemble_training(original,config)
    del original
    declared=_unseal(integrated_r2,'training_setup.json')['weights']
    if old_weights!=[w for w in declared if w['stage']!='wrist_periodic'] or old_weights!=_unseal(base_r2,'training_setup.json')['weights']:
        raise ValueError('Old training row weights changed')
    policy=config['p0']['wrist_periodic_train']
    filter_stats={str(s):[] for s in SIGMAS_MS}
    new_columns={str(s):[] for s in SIGMAS_MS}
    new_weights=[];seen=set()
    for name,meta,raw in wrist_records(integrated):
        seen.add(name)
        centers=frozen_indices[name]
        check,keep,_=make_problem(nominal,raw,meta,config['drive']['gain_nm_a'],static,step)
        if not np.array_equal(keep,centers):
            raise ValueError('Archived sample support differs from original rule')
        _,_,valid=raw_sustained_support(raw,meta,step)
        rows,a=raw_rows(nominal,raw,valid,config['drive']['gain_nm_a'],static,step)
        exact=np.concatenate([check['regression'][:,AXES],check['extra'][:,AXES],check['target'][:,AXES,None]],axis=2)
        np.testing.assert_allclose(rows[np.searchsorted(valid,centers)],exact,rtol=1e-12,atol=1e-12)
        del check,exact
        c=meta['conditions']; count=len(centers)
        factor=np.sqrt(1024*policy['squared_weight_per_axis']/(count*policy['baseline_probe_count']))/policy['normalization_nm']
        for j in AXES:
            new_weights.append(dict(trial_id=name,joint=j,rows=count,sqrt_weight=factor,
                center_id=c['center_id'],acquisition_batch_id=c['acquisition_batch_id'],stage='wrist_periodic'))
        for sigma in SIGMAS_MS:
            key=str(sigma)
            matrix,radius,nominal_radius=local_kernel(valid,centers,sigma*1e6/step)
            # One matrix acts on measured torque minus rotor/static terms and
            # every rigid/friction regressor column, without pre-smoothing q.
            filtered=(matrix@rows.reshape(len(rows),-1)).reshape(count,len(AXES),169)
            new_columns[key].append((filtered*factor).transpose(1,0,2).reshape(-1,169))
            a_filtered=matrix@a
            filter_stats[key].append(dict(trial_id=name,rows_per_axis=count,
                nominal_radius_samples=nominal_radius,edge_shortened_rows=int(np.count_nonzero(radius<nominal_radius)),
                point_only_rows=int(np.count_nonzero(radius==0)),
                maximum_effective_roll_acceleration_rad_s2=float(np.max(abs(a_filtered[:,2]))),
                p99_effective_roll_acceleration_rad_s2=float(np.quantile(abs(a_filtered[:,2]),.99))))
    if seen!=set(frozen_indices) or len(seen)!=4 or sum(w['rows'] for w in new_weights)!=baseline_report['added_raw_rows']:
        raise ValueError('Wrist record or retained-row count changed')
    old_blocks={(w['trial_id'],w['joint']):w for w in declared if w['stage']=='wrist_periodic'}
    for w in new_weights:
        z=old_blocks[w['trial_id'],w['joint']]
        if (any(w[k]!=z[k] for k in ('center_id','acquisition_batch_id'))
                or not np.isclose(w['rows']*w['sqrt_weight']**2,z['rows']*z['sqrt_weight']**2,rtol=1e-14)):
            raise ValueError('Wrist weight budget or deletion group changed')
    omit=frozen_group_mask(old_weights+new_weights,'center_id',OMIT)
    output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'raw_sample_indices.npz',**frozen_indices)
    results={}
    for sigma in SIGMAS_MS:
        key=str(sigma);block=np.vstack(new_columns.pop(key))
        y=np.concatenate((y0,block[:,:140]));x=np.concatenate((x0,block[:,140:168]))
        target=np.concatenate((t0,block[:,168]))
        directory=output/f'sigma_{key}_ms';directory.mkdir()
        states={}
        for label,keep in [('full',slice(None)),(SUBSETS[1],omit)]:
            fit=fit_rigid(y[keep],target[keep],coord['nominal_theta'],
                auxiliary=(x[keep],coord['friction'],coord['lower'],coord['upper']),
                equalities=(coord['equality'],coord['equality_target']),ridge=RIDGE,
                physical_length_scale=.1,lift_residuals=True,fixed_scales=coord['scales'],
                regularization_metric=coord['metric'])
            vector=np.r_[fit['parameters'],fit['auxiliary_parameters']]
            values=model(nominal,vector,static).evaluate_batch(probes['q'],probes['dq'],probes['ddq'])
            states[label]=dict(vector=vector,mass=values['mass_matrix'],torque=values['total'])
            np.savez_compressed(directory/f'{label}.npz',**states[label])
            write_json(directory/f'{label}_fit.json',fit)
            print(f'sigma={key} ms {label} complete',flush=True)
        results[key]=dict(sigma_ms=sigma,nominal_gaussian_minus_3db_hz=float(np.sqrt(np.log(2))/(2*np.pi*sigma*1e-3)),
                          filter_by_trial=filter_stats[key],**evaluate(*(states[k] for k in SUBSETS),settings))
    formal_test_untouched(base_root);formal_test_untouched(integrated)
    report=dict(scope='train-only diagnostic bandwidth comparison, not R2 qualification',ridge=RIDGE,omitted_center=OMIT,
        fixed_operator_probes=len(probes['q']),unchanged_old_rows=len(t0),fixed_added_rows=sum(w['rows'] for w in new_weights),
        same_sample_centers_as=str(baseline/'raw_sample_indices.npz'),unsmoothed=no_filter,matched_bandwidths=results,
        formal_test_consumed=False,selection_observations_used=False,truth_read=False,
        filter_convention='centered normalized 3-sigma Gaussian shortened symmetrically at sustained-span edges; same filter on observation and every design column; the acceleration component is five-point derivative followed by the same filter')
    write_json(output/'report.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('base_root','base_r2','integrated','integrated_r2','baseline','output'):
        p.add_argument(name,type=Path)
    report=compare(**vars(p.parse_args()))
    print({k:v['operators']['maximum_mass_diagonal_relative'] for k,v in report['matched_bandwidths'].items()})
