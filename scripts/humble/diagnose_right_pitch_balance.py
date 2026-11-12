#!/usr/bin/env python3
"""Compare existing right-pitch train observations on matched temporal support."""
import argparse
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d, minimum_filter1d
from scipy.signal import butter, sosfiltfilt

from design_wrist_supplement import mass_rows, protect_observations
from diagnose_right_wrist_pitch import PITCH, ROLL, harmonic_design
from diagnose_wrist_periodic_counterfactual import RIDGE, SUBSETS, model, sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.rigid_identification import fit_rigid
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_collection import load_trial
from casbot_calib.staged_identification import _windows, read_record
from casbot_calib.staged_multifit import assemble_training, load_problems
from casbot_calib.staged_protocol import _read, _unseal, load_protocol
from casbot_calib.staged_right_pitch import POSES, right_companion, right_records
from casbot_calib.staged_stability import frozen_group_mask
from casbot_calib.staged_stability_math import operator_difference
from casbot_calib.torque_observations import output_torque

BANDS = (8., 30., 88.)
ROLL_HALF_WIDTH = .025
# Common pitch interval inside the observed c00 probe and c02 roll-bin
# ranges. There is no such interval for the c01 probe.
C00_PITCH_OVERLAP = (-.317, -.171)


def raw_balance(nominal, obs, gain, static, dt):
    """Differentiate measured velocity; evaluate model before filtering."""
    q = obs['q'][2:-2]; v = obs['dq']; w = v[2:-2]
    a = (v[:-4]-8*v[1:-3]+8*v[3:-1]-v[4:])/(12*dt)
    out = np.zeros((len(q), 169))
    out[:, 140+PITCH] = w[:, PITCH]
    sign = np.sign(w[:, PITCH])
    transition = sign*np.exp(-(w[:, PITCH]/.002)**2)
    out[:, 154+PITCH] = sign-transition
    target = output_torque(obs, gain)[2:-2, PITCH]-static[PITCH]*transition
    for start in range(0, len(q), 4096):
        s = slice(start, start+4096)
        r = nominal.regressor_batch(q[s], w[s], a[s])
        out[s, :140] = r['regressor'][:, PITCH]
        out[s, -1] = target[s]-r['rotor_torque'][:, PITCH]
    return out, a


def periodic_rows(raw, t, start, period, harmonics, phase_points):
    """Apply the same 40-harmonic phase projection to the native raw columns."""
    n = int(round(period / np.median(np.diff(t))))
    i = np.searchsorted(t, start-1e-9)
    ix = slice(i, i+2*n)
    if len(t[ix]) != 2*n or not np.allclose(t[ix]-start, np.arange(2*n)*period/n, rtol=0, atol=1e-8):
        raise ValueError('Expected two full original c02 cycles')
    phase = raw[ix].reshape(2,n,-1).mean(axis=0)
    spec = np.fft.rfft(phase, axis=0)[:harmonics+1]
    return np.fft.irfft(spec, n=phase_points, axis=0)*(phase_points/n)


def projection(t, a, velocity, own, error):
    # Remove slow drift, viscous velocity and motion sign; use the same local
    # estimator for both sources. Other dynamic terms come from the anchor.
    t = t-t.mean()
    nuisance = np.column_stack((np.ones(len(t)), t, t*t, velocity, np.sign(velocity)))
    rhs = np.column_stack((a, own, error))
    rhs -= nuisance @ np.linalg.lstsq(nuisance, rhs, rcond=1e-9)[0]
    a, own, error = rhs.T
    norm = a@a
    if norm <= 0:
        raise ValueError('No pitch acceleration left after removing nuisance terms')
    return dict(rows=len(t), projected_acceleration_rms_rad_s2=float(np.sqrt(norm/len(t))),
                anchor_mass_kg_m2=float((a@own)/norm), observed_effective_mass_kg_m2=float((a@(own+error))/norm),
                correction_kg_m2=float((a@error)/norm), acceleration_energy_rad2_s4=float(norm),
                residual_rms_nm=float(np.sqrt(np.mean(error**2))))


def shared_bands(name, t, raw, q, v, a, window, dt, harmonics, vectors, source, conditions):
    # Same raw indices at all three bandwidths; the widest kernel may not
    # cross a pitch stop, a sign change or the designated time window.
    radius = round(3/(2*np.pi*8*dt))
    speed = v[:, PITCH]
    smooth_ok = ((minimum_filter1d(speed, size=2*radius+1, mode='nearest')>.02)
                 | (minimum_filter1d(-speed, size=2*radius+1, mode='nearest')>.02))
    valid = smooth_ok & (t>=window[0]+radius*dt) & (t<window[1]-radius*dt)
    valid &= np.arange(len(t)) % 4 == 0
    predicted = raw[:, :168]@vectors.T
    error = raw[:, -1:] - predicted
    inertia = harmonic_design(q[:, ROLL]) @ (harmonics @ vectors[:, :140].T)
    own = inertia*a[:, PITCH:PITCH+1]
    base = np.column_stack((a[:,PITCH],speed,own,error))
    for band in BANDS:
        filtered = gaussian_filter1d(base, 1/(2*np.pi*band*dt), axis=0, truncate=3)
        for center,roll in POSES.items():
            if source=='probe' and center!=conditions['center_id']:
                continue
            roll_mask=valid & (abs(q[:,ROLL]-roll)<=ROLL_HALF_WIDTH)
            scopes={'roll_only':roll_mask}
            if center=='train_c00':
                lo,hi=C00_PITCH_OVERLAP
                scopes['roll_and_pitch']=roll_mask & (q[:,PITCH]>=lo) & (q[:,PITCH]<=hi)
            for scope,keep in scopes.items():
                if keep.sum()<32:
                    continue
                m=len(vectors)
                for k in range(m):
                    row=projection(t[keep],filtered[keep,0],filtered[keep,1],
                        filtered[keep,2+k],filtered[keep,2+m+k])
                    row.update(trial=name, source=source, batch=conditions['acquisition_batch_id'],
                        roll_group=center, band_hz=band, anchor=k, pose_scope=scope,
                        roll_rad=[float(q[keep,ROLL].min()),float(q[keep,ROLL].max())],
                        pitch_rad=[float(q[keep,PITCH].min()),float(q[keep,PITCH].max())])
                    yield row


def diagnostic_fit(matrix, torque, coord, nominal, queries):
    fit=fit_rigid(matrix[:, :140],torque,coord['nominal_theta'],
        auxiliary=(matrix[:, 140:],coord['friction'],coord['lower'],coord['upper']),
        equalities=(coord['equality'],coord['equality_target']),ridge=RIDGE,
        physical_length_scale=.1,lift_residuals=True,fixed_scales=coord['scales'],regularization_metric=coord['metric'])
    vector=np.r_[fit['parameters'],fit['auxiliary_parameters']]
    pred=model(nominal,vector,coord['static']).evaluate_batch(queries['q'],queries['dq'],queries['ddq'])
    return dict(vector=vector,mass=pred['mass_matrix'],torque=pred['total']),fit


def restore_blocks(output, matrix, target, weights, coord, nominal, queries, after, settings):
    """Restore selected portions of c02 to the measured leave-c02-out fit."""
    full,cut=(after[s] for s in SUBSETS)
    c02=lambda w: w['center_id']=='train_c02'
    pitch=lambda w: c02(w) and w['stage']=='inertia' and w['joint']==PITCH
    yaw_probe=lambda w: c02(w) and w['stage']=='inertia_probe' and w['joint']==11
    restored={
        'c02_pitch_periodic':pitch,
        'c02_yaw_probes':yaw_probe,
        'c02_other_rows':lambda w:c02(w) and not pitch(w) and not yaw_probe(w),
        'c02_except_pitch_periodic':lambda w:c02(w) and not pitch(w),
    }
    expand=lambda test:np.repeat([test(w) for w in weights],[w['rows'] for w in weights])
    omitted=expand(c02)
    def compare(state,ref):
        return operator_difference(ref['mass'],state['mass'],ref['torque'],state['torque'],settings)
    signed_at=(cut['mass'][:,PITCH,PITCH]-full['mass'][:,PITCH,PITCH])/np.maximum(
        full['mass'][:,PITCH,PITCH],settings['mass_diagonal_floor_kg_m2'])
    index=int(np.argmax(abs(signed_at)))
    initial=dict(restore_none=compare(cut,full), restore_all=compare(full,full))
    results={}
    for label,rule in restored.items():
        add=expand(rule);keep=~omitted|add
        state,fit=diagnostic_fit(matrix[keep],target[keep],coord,nominal,queries)
        np.savez_compressed(output/f'{label}.npz',**state)
        write_json(output/f'{label}_fit.json',fit)
        results[label]=dict(restored_rows=int(add.sum()), relative_to_full=compare(state,full),
                            relative_to_omit=compare(state,cut),
                            mass_at_original_worst_kg_m2=float(state['mass'][index,PITCH,PITCH]))
        print('Restored',label,flush=True)
    write_json(output/'restorations.json',dict(ridge=RIDGE,probes=len(queries['q']),
        worst_measured_probe=index,roll_at_worst_rad=float(queries['q'][index,ROLL]),
        full_mass_at_worst_kg_m2=float(full['mass'][index,PITCH,PITCH]),
        omit_mass_at_worst_kg_m2=float(cut['mass'][index,PITCH,PITCH]),
        initial=initial,restorations=results,formal_test_consumed=False,truth_read=False))


def run(root, r2, baseline, output, *, restorations_only=False, balance_only=False):
    root,r2,baseline,output=(Path(p).resolve() for p in (root,r2,baseline,output))
    if output.exists() or any(output.is_relative_to(p) for p in (root,r2,baseline)):
        raise ValueError('Use fresh diagnostic output outside inputs')
    _,config,manifest=load_protocol(root)
    formal_test_untouched(root)
    protect_observations(root,{(r['trial_id'],) for r in manifest['trials'] if r['split']=='train'})
    cap,names=right_companion(root)
    protect_observations(cap,{(n,) for n in names})
    r2_protocol=sealed_r2(root,r2)
    coord=arrays(r2/'coordinates.npz')
    queries=arrays(r2/'operator_probe_set.npz')
    before={s:arrays(baseline/f'{s}.npz') for s in SUBSETS}
    after={s:arrays(r2/'variants'/f'ridge_{RIDGE:g}_{s}'/'solution.npz') for s in SUBSETS}
    vector_labels=['before_full','before_omit_c02','with_probe_full','with_probe_omit_c02']
    vectors=np.stack([d[s]['vector'] for d in (before,after) for s in SUBSETS])
    gain=config['drive']['gain_nm_a']; step=config['inertia_sample_period_ns']; dt=step*1e-9
    nominal,_=model_for(config); static=coord['static']
    angles=np.linspace(-np.pi,np.pi,12,endpoint=False)
    poses=np.repeat(queries['q'][:1],len(angles),axis=0);poses[:,ROLL]=angles
    harmonics=np.linalg.lstsq(harmonic_design(angles),mass_rows(nominal,poses,PITCH),rcond=None)[0]
    for index,states in enumerate((before,after)):
        for k,subset in enumerate(SUBSETS):
            replay=harmonic_design(queries['q'][:,ROLL])@harmonics@vectors[2*index+k,:140]
            np.testing.assert_allclose(replay,states[subset]['mass'][:,PITCH,PITCH],atol=1e-12,rtol=0)
    training=load_problems(root,'train',static)
    y,x,target,weights=assemble_training(training,config)
    if weights!=_unseal(r2,'training_setup.json')['weights']:
        raise ValueError('Expected the original frozen training row weights')
    if restorations_only:
        output.mkdir(parents=True,exist_ok=False)
        restore_blocks(output,np.column_stack((y,x)),target,weights,coord,nominal,queries,after,r2_protocol['settings'])
        formal_test_untouched(root)
        return
    matched={}; rows=[]; production=[]
    plan=[r['trial_id'] for r in manifest['trials'] if r['split']=='train' and r['center_id']=='train_c02' and r['stage']=='inertia']
    joint_names=_read(root/'session.json')['joint_names']
    c02=((n,m,read_record(root,n,m,joint_names)[1]) for n in plan for m,_ in [load_trial(root,n)])
    for source,records in [('probe',right_records(root)),('c02_periodic',c02)]:
        for name,meta,obs in records:
            raw,a=raw_balance(nominal,obs,gain,static,dt)
            q=obs['q'][2:-2]; v=obs['dq'][2:-2]; t=obs['sample_ns'][2:-2]*1e-9
            p=training[name]['problem']
            if source=='probe':
                windows_for_probe=_windows(meta,'probe_measure')
                begin,end=windows_for_probe[0][0],windows_for_probe[-1][1]
                ix=np.searchsorted(t,p['time_s'])
                np.testing.assert_array_equal(t[ix],p['time_s'])
                matched[name]=sosfiltfilt(butter(6,30.,fs=1/dt,output='sos'),raw,axis=0)[ix]
                windows=(begin+.2,end-.2)
            else:
                period=meta['conditions']['period_s']
                begin=_windows(meta,'periodic')[0][0]+2*period
                phase=periodic_rows(raw,t,begin,period,config['p0']['preprocessing']['harmonics'],
                                    config['p0']['preprocessing']['phase_points'])
                mask=p['moving'][:,PITCH]
                np.testing.assert_allclose(phase[mask,-1],p['target'][mask,PITCH],atol=1e-11,rtol=0)
                matched[name]=phase[mask]
                windows=(begin,begin+2*period)
            rows.extend(shared_bands(name,t,raw,q,v,a,windows,dt,harmonics,vectors,source,meta['conditions']))
    frozen=np.column_stack((y,x))
    replace=np.zeros(len(target),dtype=bool)
    c02_pitch=np.zeros(len(target),dtype=bool)
    new_rows=frozen.copy(); new_targets=target.copy()
    pos=0
    for w in weights:
        n=w['rows'];s=slice(pos,pos+n);pos+=n
        if w['trial_id'] not in matched or w['joint']!=PITCH:
            continue
        m=matched[w['trial_id']]*w['sqrt_weight'];new_rows[s]=m[:,:168];new_targets[s]=m[:,-1]
        group=replace if w['stage']=='right_pitch_probe' else c02_pitch
        group[s]=True
        residual=target[s,None]-frozen[s]@vectors.T
        residual_matched=new_targets[s,None]-new_rows[s]@vectors.T
        production.append(dict(trial=w['trial_id'],source=w['stage'],rows=n,
            residual_rms_nm=(np.sqrt(np.mean(residual**2,axis=0))/w['sqrt_weight']).tolist(),
            matched_residual_rms_nm=(np.sqrt(np.mean(residual_matched**2,axis=0))/w['sqrt_weight']).tolist(),
            matched_prediction_change_rms_nm=(np.sqrt(np.mean((residual_matched-residual)**2,axis=0))/w['sqrt_weight']).tolist(),
            weighted_sse=(residual**2).sum(axis=0).tolist()))
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'balance.json',dict(anchor_labels=vector_labels,shared_band_rows=rows,production_rows=production,
        roll_half_width_rad=ROLL_HALF_WIDTH,gaussian_amplitude_sigma_s={str(b):1/(2*np.pi*b) for b in BANDS},
        common_band='evaluate raw force balance first; same zero-phase kernel applied to all terms and torque',
        phase_projection='native raw regressor to original phase grid and 40 harmonics, fixed original row mask',
        train_only=True, formal_test_consumed=False, truth_read=False))
    print('Saved matched-band balance',flush=True)
    if balance_only:
        formal_test_untouched(root)
        return
    # Replace only the stated rows/targets; all original scales, group masks,
    # bounds, priors and equality constraints stay frozen.
    neutral=target.copy();neutral[replace]=frozen[replace]@vectors[0]
    omit=frozen_group_mask(weights,'center_id','train_c02')
    states={'before':before,'measured':after}
    for label in ('matched_probe','matched_both','neutral_probe_target'):
        states[label]={}
        for subset,keep in zip(SUBSETS,(np.ones(len(target),dtype=bool),omit)):
            if label=='matched_both' and subset!=SUBSETS[0]:
                states[label][subset]=states['matched_probe'][subset]
                continue
            row_mask=(replace|c02_pitch) if label=='matched_both' else replace
            matrix=frozen[keep].copy()
            if label=='neutral_probe_target':
                torque=neutral[keep]
            else:
                part=row_mask[keep];matrix[part]=new_rows[keep & row_mask]
                torque=target[keep].copy();torque[part]=new_targets[keep & row_mask]
            state,fit=diagnostic_fit(matrix,torque,coord,nominal,queries)
            states[label][subset]=state
            np.savez_compressed(output/f'{label}_{subset}.npz',**state)
            write_json(output/f'{label}_{subset}_fit.json',fit)
            print(label,subset,flush=True)
    compared={}
    for label,values in states.items():
        ref,other=(values[s] for s in SUBSETS)
        delta=operator_difference(ref['mass'],other['mass'],ref['torque'],other['torque'],r2_protocol['settings'])
        compared[label]=dict(operators=delta,right_pitch_maximum_relative=delta['axis_mass_diagonal_maximum_relative'][PITCH])
    formal_test_untouched(root)
    write_json(output/'report.json',dict(ridge=RIDGE,probes=len(queries['q']),cases=compared,
        replaced_probe_rows=int(replace.sum()),replaced_c02_pitch_rows=int(c02_pitch.sum()),
        formal_test_consumed=False,truth_read=False,scope='train-only counterfactuals; no R1/R2 qualification'))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('root','r2','baseline','output'):
        p.add_argument(arg,type=Path)
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--restorations-only',action='store_true')
    mode.add_argument('--balance-only',action='store_true')
    run(**vars(p.parse_args()))
