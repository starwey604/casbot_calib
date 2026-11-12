#!/usr/bin/env python3
"""Verify the frozen dual-tone adapter with analytical arrays; no SDK session."""
import argparse
import os
from pathlib import Path
import sys
import numpy as np
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('protocol',type=Path);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    def guard(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
            raise RuntimeError('Private evaluation forbidden')
    sys.addaudithook(guard)
    from casbot_calib import hardware_roll_dual as dual
    from casbot_calib import hardware_probe_capture as capture
    from casbot_calib.hardware_wrist_exploration import FLAGS,read
    from casbot_calib.model_runtime import model_for
    from casbot_calib.data import write_json
    def no_io(*args,**kwargs):raise RuntimeError('Offline verification cannot open SDK or collect')
    capture.SDKMockSession.__enter__=no_io;capture.field.snapshot_field=no_io;capture.field.execute_field=no_io
    root=a.protocol.resolve();p=dual.load_protocol(root);out=dual.field.output_outside(a.output,root);out.mkdir(parents=True,exist_ok=False)
    profile=yaml.safe_load((root/'protocol/base_profile.yaml').read_text());engine,_=model_for(profile)
    # Numerical evaluation arrays, not observations from a new robot/Mock run.
    home=np.asarray(p['pose_rad']);J=p['observation_policy']['inertia_scale_kg_m2'];cases=[];example_rows=[];example_traces=[];scalar_diff=0.
    for job in p['jobs']:
        motion=dual.protocol_motion(job['id'],home,p['pose_rad'],p['joint'],job['direction'],config=p['probe_config'],tones=p['tones'],transition_s=p['transition_s'],settle_s=p['settle_s'])
        begin=motion.windows('probe_measure')[0][0];end=motion.windows('probe_measure')[-1][1]
        t=np.arange(begin-.02,end+.02,.001);cached=np.floor((t-.0013)/.005)*.005+.0013;q,v,_,_=motion.at(cached)
        for advance,bfactor in [(-.005,.8),(0.,1.),(.005,1.2)]:
            tq,tv,ta,_=motion.at(cached+advance);effort=engine.gravity_batch(tq);speed=tv[:,13];sign=np.sign(speed);decay=np.exp(-(speed/.005)**2)
            b=p['fixture_friction_prior']['viscous'][13]*bfactor
            effort[:,13]+=J*ta[:,13]+b*speed+.005*sign*(1-decay)+.01*sign*decay
            obs=dict(receive_ns=10**15+np.rint(t*1e9).astype(np.int64),q=q,dq=v,effort_raw=effort)
            report,problem,trace=dual.inspect(obs,t,motion.metadata(),engine,p['probe_config'],profile,p['fixture_friction_prior'],p['observation_policy'],p['tones'])
            full=report['frequency_analysis']['full'];fit=full['free_b_complex']
            # Independently solve the zero-phase full temporal scalar balance.
            tt=problem['time_s'];x=tt-tt.mean();X=np.column_stack([problem['ddq'][:,13],problem['dq'][:,13],np.ones(len(tt)),x,x*x])
            coefficients=np.linalg.lstsq(X,problem['target']-problem['projected_gravity_nm'],rcond=None)[0]
            scalar_diff=max(scalar_diff,float(abs(coefficients[0]-report['inertia_diagnostic']['inertia_kg_m2'])))
            cases.append(dict(job=job,imposed_advance_s=advance,imposed_viscous_factor=bfactor,report=report,
                inertia_error=abs(fit['inertia_kg_m2']/J-1),effective_advance_error_s=abs(fit['effective_advance_s']-advance)))
            if advance==0.:
                report.update(job=job,received_collision_passed=None);example_rows.append(report);example_traces.append(trace)
                np.savez_compressed(out/(job['id']+'_analytical_problem.npz'),**{k:v for k,v in problem.items() if isinstance(v,np.ndarray)})
    pair=dual.pair_summary(example_rows,p)
    checks=dict(frozen_jobs_exactly_negative_positive=[j['direction'] for j in p['jobs']]==[-1,1],
        cache_5ms=p['probe_config']['cache_period_ns']==5000000,
        all_raw_and_preprocessing_passed=all(all(c['report']['raw_checks'].values()) and c['report']['probe_preprocessing_passed'] for c in cases),
        both_frequencies_reported=all([s['frequency_hz'] for s in c['report']['frequency_signals']]==[2.5,10.] for c in cases),
        no_stale_5hz_signal_keys=all(not any('5hz' in k for k in c['report']) for c in cases),
        all_frequency_models_available=all(c['report']['frequency_analysis']['full']['available'] for c in cases),
        free_model_ideal_inertia_under_original10_percent=max(c['inertia_error'] for c in cases)<.1,
        free_model_effective_phase_error_under_0p1ms=max(c['effective_advance_error_s'] for c in cases)<.0001,
        original_threshold_preserved=p['original_inertia_relative_threshold']==.1,
        independent_zero_phase_inertia_difference_small=scalar_diff<1e-12,
        no_mock_or_collection_created=not (root/'mock').exists() and not (root/'collection.json').exists())
    write_json(out/'verification.json',dict(passed=all(checks.values()),checks=checks,analytical_cases=cases,
        maximum_analytic_inertia_error=max(c['inertia_error'] for c in cases),maximum_effective_phase_error_s=max(c['effective_advance_error_s'] for c in cases),
        independent_zero_phase_maximum_difference=scalar_diff,example_pair=pair,
        analytical_readout_only=True,records_collected=0,formal_test_consumed=False,**FLAGS))
    dual.plot(example_rows,example_traces,out,fixture_only=True)
    print(checks);print('maximum_analytic_inertia_error',max(c['inertia_error'] for c in cases))


if __name__=='__main__':main()
