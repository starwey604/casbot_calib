#!/usr/bin/env python3
"""Read-only follow-up: distinguish sampled timing, cross-axis terms and reversal residual."""
import argparse
from pathlib import Path
import numpy as np

from diagnose_wrist_periodic_counterfactual import model, spectrum, sealed_r2
from design_wrist_supplement import protect_observations
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_inertia import _project, periodic_problem
from casbot_calib.staged_protocol import load_protocol
from casbot_calib.staged_residuals import matched_components
from casbot_calib.staged_wrist_periodic import AXES, BINS, wrist_records


def inspect(base, base_r2, integrated, output):
    base,base_r2,integrated,output=(Path(p).resolve() for p in (base,base_r2,integrated,output))
    if any(output.is_relative_to(p) for p in (base,base_r2,integrated)):
        raise ValueError('Write outside sealed inputs')
    _,config,_=load_protocol(integrated);formal_test_untouched(integrated)
    protect_observations(integrated,set())
    protect_observations(base,set())
    sealed_r2(base,base_r2)
    vector=arrays(base_r2/'variants/ridge_0.0001_full/solution.npz')['vector']
    static=arrays(base_r2/'coordinates.npz')['static']
    nominal,_=model_for(config);candidate=model(nominal,vector,static)
    settings=config['p0']['preprocessing']; h=settings['harmonics']; n=settings['phase_points']
    rows=[]
    for name,meta,raw in wrist_records(integrated):
        p=periodic_problem(nominal,raw,meta,config['drive']['gain_nm_a'],static,h,n)
        components,prediction,_=matched_components(candidate,vector[:140],vector[140:],p,h)
        error=p['current_torque']-prediction
        terms=candidate.evaluate_batch(p['q'],p['dq'],p['ddq'])
        diag=np.diagonal(terms['mass_matrix'],axis1=1,axis2=2)
        off_axis=_project(terms['inertia']-diag*p['ddq'],h)
        period=meta['conditions']['period_s']; step=config['inertia_sample_period_ns']
        start=meta['knots_s'][meta['labels'].index('periodic')]
        expected_ns=round((start+2*period)*1e9)
        samples_per_period=round(period*1e9)//step
        mask=(raw['sample_ns']>=expected_ns)&(raw['sample_ns']<expected_ns+2*round(period*1e9))
        if mask.sum()!=2*samples_per_period or raw['sample_ns'][mask][0]!=expected_ns:
            raise ValueError('Matched raw acquisition cycles are not on the declared phase origin')
        v=raw['dq'][mask].reshape(2,samples_per_period,14).mean(axis=0)
        latency=np.unique(raw['receive_ns'][mask]-raw['sample_ns'][mask])
        for j,k in zip(AXES,BINS):
            e=spectrum(error[:,j])[k]
            f=spectrum(components['friction'][:,j])[k]
            vq=spectrum(p['dq'][:,j])[k]
            vv=spectrum(v[:,j])[k]
            ac=spectrum(p['ddq'][:,j])[k]
            omega=2*np.pi*k/period
            phase=np.arange(n)*2*np.pi*k/n
            carrier=np.sqrt(2)*np.exp(-1j*(phase+np.angle(ac)))
            aligned=(error[:,j]*carrier).real
            v_fund=np.sqrt(2)*abs(vq)*np.cos(phase+np.angle(vq))
            near=np.abs(v_fund)<=.2*np.max(abs(v_fund))
            rows.append(dict(trial=name,joint=j,frequency_hz=k/period,
                residual_rms_nm=float(abs(e)), friction_rms_nm=float(abs(f)),
                equivalent_friction_lead_ms=float(np.angle(1+e/f)*1e3/omega),
                raw_velocity_vs_position_derivative_phase_ms=float(np.angle(vv/vq)*1e3/omega),
                receive_minus_acquire_ns=latency.tolist(),
                cross_axis_inertia_rms_nm=float(abs(spectrum(off_axis[:,j])[k])),
                velocity_coupling_rms_nm=float(abs(spectrum(components['velocity'][:,j])[k])),
                near_reversal_phase_share=float(near.mean()),
                near_reversal_in_phase_contribution_nm=float(np.sum(aligned[near])/n),
                remainder_in_phase_contribution_nm=float(np.sum(aligned[~near])/n)))
    write_json(output,dict(scope='observed four train trajectories; yaw-only full candidate; no fit or plant truth',
        current_response_tau_ms=config['drive']['current_tau']*1e3,rows=rows,
        equivalent_lead='angle((friction+residual)/friction)/omega; diagnostic, not a measured clock offset',
        near_reversal='matched-band residual; |fundamental velocity| <=20% of its peak; filtering spreads phase-local effects'))
    return rows


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('base','base_r2','integrated','output'):
        p.add_argument(name,type=Path)
    inspect(**vars(p.parse_args()))
