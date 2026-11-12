#!/usr/bin/env python3
"""Frozen-candidate force balance on observed wrist cycles; no candidate refit."""
import argparse
import csv
from pathlib import Path

import numpy as np

from design_wrist_supplement import protect_observations
from diagnose_wrist_periodic_counterfactual import model, sealed_r2
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_inertia import periodic_problem
from casbot_calib.staged_protocol import load_protocol
from casbot_calib.staged_residuals import matched_components
from casbot_calib.staged_wrist_periodic import AXES, BINS, wrist_records

BANDS = (40, 80, 120, 240)


def derivative(values, period, order, band):
    coeff = np.fft.rfft(values,axis=0)
    coeff[band+1:] = 0
    omega = 2j*np.pi*np.arange(len(coeff))/period
    return np.fft.irfft(coeff*omega[:,None]**order,n=len(values),axis=0)


def tone(values, k):
    n=len(values)
    return np.sqrt(2)*np.sum(values*np.exp(-2j*np.pi*k*np.arange(n)/n))/n


def moving_contrast(velocity, acceleration, residual):
    """Equal signed-speed bins remove an arbitrary memoryless speed offset."""
    slope_terms=[]
    for direction in (-1,1):
        for lo,hi in zip(np.arange(.02,.26,.01),np.arange(.03,.27,.01)):
            # Compare + and - acceleration in the same speed interval, away from turnaround.
            use=(direction*velocity>=lo)&(direction*velocity<hi)
            pos=use&(acceleration>=.5);neg=use&(acceleration<=-.5)
            if min(pos.sum(),neg.sum())<8:
                continue
            delta_a=acceleration[pos].mean()-acceleration[neg].mean()
            delta_tau=residual[pos].mean()-residual[neg].mean()
            count=min(pos.sum(),neg.sum())
            slope_terms.append((float(delta_a),float(delta_tau),int(count)))
    if not slope_terms:
        raise ValueError('No sustained-motion matched speed bins')
    a,e,w=np.asarray(slope_terms).T
    return dict(bins=len(w),paired_phase_samples=int(w.sum()),
        effective_extra_inertia_kg_m2=float(np.sum(w*a*e)/np.sum(w*a*a)),
        scope='descriptive contrast; equal signed speed does not distinguish inertia from history-dependent friction')


def inspect(base, base_r2, integrated, output):
    base,base_r2,integrated,output=(Path(p).resolve() for p in (base,base_r2,integrated,output))
    if output.exists() or any(output.is_relative_to(p) for p in (base,base_r2,integrated)):
        raise ValueError('Use a fresh output outside the sealed sessions')
    _,config,_=load_protocol(integrated)
    formal_test_untouched(base);formal_test_untouched(integrated)
    protect_observations(base,set());protect_observations(integrated,set())
    sealed_r2(base,base_r2)
    vector=arrays(base_r2/'variants/ridge_0.0001_full/solution.npz')['vector']
    static=arrays(base_r2/'coordinates.npz')['static']
    nominal,_=model_for(config);candidate=model(nominal,vector,static)
    gain=config['drive']['gain_nm_a']; step=config['inertia_sample_period_ns']
    rows=[]; windows=[]; phase_series={}
    for name,meta,raw in wrist_records(integrated):
        c=meta['conditions']; period=c['period_s']
        period_ns=round(period*1e9); count=period_ns//step
        first=meta['labels'].index('periodic'); start=round(meta['knots_s'][first]*1e9)+2*period_ns
        ix=(raw['sample_ns']>=start)&(raw['sample_ns']<start+2*period_ns)
        if not np.array_equal(raw['sample_ns'][ix],start+np.arange(2*count,dtype=np.int64)*step):
            raise ValueError('Expected two contiguous complete cycles on the acquisition grid')
        def read(key):
            return raw[key][ix].reshape(2,count,14)
        q,v,tau=read('q'),read('dq'),read('current')*gain
        # Apply nonlinear friction at the raw sample level before averaging cycles.
        dry=np.sign(v); exp=np.exp(-(v/.002)**2)
        viscous=v*vector[140:154]
        coulomb=dry*(1-exp)*vector[154:]
        transition=dry*exp*static
        # Mass and nonlinear terms are evaluated at each acquired sample, not at
        # a truncated position or a cycle-averaged velocity.
        terms=candidate.evaluate_batch(q.reshape(-1,14),v.reshape(-1,14),np.zeros((2*count,14)))
        mass=terms['mass_matrix'].reshape(2,count,14,14)
        bias=(terms['gravity']+terms['velocity']).reshape(2,count,14)
        friction=viscous+coulomb+transition
        original=periodic_problem(nominal,raw,meta,gain,static,40,1024)
        _,old_prediction,_=matched_components(candidate,vector[:140],vector[140:],original,40)
        old_residual=original['current_torque']-old_prediction
        for band in BANDS:
            for source, signals, order in [('position',q,2),('velocity',v,1)]:
                a=np.array([derivative(cycle,period,order,band) for cycle in signals])
                inertial=np.einsum('ctij,ctj->cti',mass,a)
                residual=tau-bias-inertial-friction
                for axis,k in zip(AXES,BINS):
                    # Integrate each region on the original grid before projecting
                    # the residual. This avoids moving filtered tails between regions.
                    accel_ref=tone(a[:,:,axis].mean(axis=0),k)
                    align=np.exp(-1j*np.angle(accel_ref))
                    mean=residual[:,:,axis].mean(axis=0)
                    z=tone(mean,k)
                    old_z=tone(old_residual[:,axis],k)
                    masks={'near_zero':abs(v[:,:,axis])<=.002,
                           'transition':(abs(v[:,:,axis])>.002)&(abs(v[:,:,axis])<.02),
                           'sustained':abs(v[:,:,axis])>=.02}
                    bins={label:tone((residual[:,:,axis]*mask).mean(axis=0),k) for label,mask in masks.items()}
                    if abs(sum(bins.values())-z)>1e-12:
                        raise ValueError('Raw phase regions do not reconstruct the residual')
                    inertia=tone(inertial[:,:,axis].mean(axis=0),k)
                    rows.append(dict(trial_id=name,center=c['center_id'],batch=c['acquisition_batch_id'],axis=axis,
                        harmonic=k,frequency_hz=k/period,retained_harmonics=band,derivative=source,
                        rms_residual_nm=float(abs(z)),in_phase_residual_nm=float((z*align).real),
                        quadrature_residual_nm=float((z*align).imag),
                        original_40_residual_rms_nm=float(abs(old_z)),inertial_rms_nm=float(abs(inertia)),
                        per_cycle_tone_difference_nm=float(abs(tone(residual[0,:,axis]-residual[1,:,axis],k)))))
                    for label,mask in masks.items():
                        windows.append(dict(trial_id=name,axis=axis,retained_harmonics=band,derivative=source,
                            region=label,phase_sample_fraction=float(mask.mean()),
                            in_phase_contribution_nm=float((bins[label]*align).real),
                            quadrature_contribution_nm=float((bins[label]*align).imag),
                            rms_residual_within_region_nm=float(np.sqrt(np.mean(residual[:,:,axis][mask]**2))) if mask.any() else None))
                    if axis==6 and band==240:
                        contrast=moving_contrast(v[:,:,axis],a[:,:,axis],residual[:,:,axis])
                        rows[-1]['sustained_equal_speed_contrast']=contrast
                        phase_series[f'{name}_{source}']=np.stack([
                            np.arange(count)*step*1e-9,v[:,:,axis].mean(axis=0),a[:,:,axis].mean(axis=0),mean],axis=1)
    formal_test_untouched(integrated)
    output.mkdir(parents=True,exist_ok=False)
    with (output/'speed_regions.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(windows[0]));writer.writeheader();writer.writerows(windows)
    np.savez_compressed(output/'roll_phase_series.npz',**phase_series)
    report=dict(scope='frozen yaw-only rigid/friction candidate, no fitting or qualification',
        raw_cycles_per_trial=2, sample_period_ns=step, records=len({r['trial_id'] for r in rows}),rows=rows,
        phase_series_columns=['phase_s','observed_velocity_rad_s','bandlimited_acceleration_rad_s2','torque_residual_nm'],
        segmentation_rad_s=dict(near_zero_max=.002,sustained_min=.02),formal_test_consumed=False,
        selection_observations_used=False,truth_read=False)
    write_json(output/'report.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('base','base_r2','integrated','output'):
        p.add_argument(name,type=Path)
    report=inspect(**vars(p.parse_args()))
    print(f"Compared {len(report['rows'])} fixed-candidate balances across {report['records']} records.")
