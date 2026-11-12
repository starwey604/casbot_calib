#!/usr/bin/env python3
"""Check the required reaction on observed roll dwells against frozen static bounds."""
import argparse
from pathlib import Path
import numpy as np

from design_wrist_supplement import protect_observations
from diagnose_wrist_periodic_counterfactual import model, sealed_r2
from inspect_wrist_wideband import derivative, tone
from casbot_calib.data import write_json
from casbot_calib.model_runtime import model_for
from casbot_calib.staged_attribution import arrays, formal_test_untouched
from casbot_calib.staged_protocol import load_protocol
from casbot_calib.staged_wrist_periodic import wrist_records


def inspect(base, analysis, integrated, output):
    base,analysis,integrated,output=(Path(p).resolve() for p in (base,analysis,integrated,output))
    if output.exists() or any(output.is_relative_to(p) for p in (base,analysis,integrated)):
        raise ValueError('Use a fresh output outside the sealed inputs')
    _,config,_=load_protocol(integrated)
    formal_test_untouched(base);formal_test_untouched(integrated)
    protect_observations(base,set());protect_observations(integrated,set())
    sealed_r2(base,analysis)
    vector=arrays(analysis/'variants/ridge_0.0001_full/solution.npz')['vector']
    static=arrays(analysis/'coordinates.npz')['static'];nominal,_=model_for(config)
    candidate=model(nominal,vector,static)
    axis=6;step=config['inertia_sample_period_ns'];rows=[]
    for name,meta,raw in wrist_records(integrated):
        period=meta['conditions']['period_s'];count=round(period*1e9)//step
        first=meta['labels'].index('periodic');start=round((meta['knots_s'][first]+2*period)*1e9)
        begin=int(np.searchsorted(raw['sample_ns'],start))
        selection=slice(begin,begin+2*count)
        if not np.array_equal(raw['sample_ns'][selection],start+np.arange(2*count,dtype=np.int64)*step):
            raise ValueError('Expected two contiguous acquisition cycles')
        q=raw['q'][selection];v=raw['dq'][selection];tau=raw['current'][selection]*config['drive']['gain_nm_a']
        a=np.vstack([derivative(c,period,1,240) for c in v.reshape(2,count,14)])
        terms=candidate.evaluate_batch(q,v,np.zeros_like(v))
        exact=v[:,axis]==0.
        edges=np.flatnonzero(np.diff(np.r_[False,exact,False].astype(int)))
        dwells=[(lo,hi) for lo,hi in zip(edges[::2],edges[1::2]) if hi-lo>=4 and np.ptp(q[lo:hi,axis])<=2e-6]
        core=np.zeros(len(v),bool)
        for lo,hi in dwells:
            core[lo+1:hi-1]=True  # omit the sample at each dwell edge
        # For a held roll, solve the balance with a_roll=0; other accelerations
        # still enter through the frozen mass matrix. This is an interval check,
        # not a replacement prediction using observed torque.
        inertia=np.einsum('ti,ti->t',terms['mass_matrix'][:,axis,:],a)
        other_inertia=inertia-terms['mass_matrix'][:,axis,axis]*a[:,axis]
        reaction=tau[:,axis]-terms['gravity'][:,axis]-terms['velocity'][:,axis]-other_inertia
        z=tone(np.where(core,reaction,0).reshape(2,count).mean(axis=0),17)
        reference=tone(a[:,axis].reshape(2,count).mean(axis=0),17)
        rows.append(dict(trial_id=name,qualified_dwells=len(dwells),
            dwell_duration_ms=[(hi-lo)*step*1e-6 for lo,hi in dwells],
            exact_zero_sample_fraction=float(exact.mean()),dwell_core_fraction=float(core.mean()),
            frozen_static_limit_nm=float(static[axis]),
            maximum_required_reaction_nm=float(np.max(abs(reaction[core]))),
            fraction_within_static_interval=float(np.mean(abs(reaction[core])<=static[axis])),
            p99_excess_nm=float(np.quantile(np.maximum(abs(reaction[core])-static[axis],0),.99)),
            roll_fundamental_dwell_reaction_in_phase_nm=float((z*np.exp(-1j*np.angle(reference))).real)))
    write_json(output,dict(scope='frozen-model feasibility diagnostic on raw observed roll dwells; not a fitted friction model',
        dwell_rule='>=1 ms at observed dq=0 and q span <=2 microrad; trim both edges',
        rows=rows,truth_read=False,formal_test_consumed=False))
    return rows


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('base','analysis','integrated','output'):
        p.add_argument(name,type=Path)
    print(inspect(**vars(p.parse_args())))
