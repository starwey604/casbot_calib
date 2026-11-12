#!/usr/bin/env python3
"""Independent public-CSV checks and feature replay; no evaluation data or motion."""
import argparse
import csv
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();root=a.source.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    def guard(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
            raise RuntimeError('Private evaluation forbidden')
    sys.addaudithook(guard)
    try:open('/__guard_probe__/evaluation/private','rb')
    except RuntimeError:pass
    else:raise RuntimeError('Private-data guard inactive')
    from casbot_calib import hardware_field as field
    from casbot_calib.hardware_probe import probe_problem,inertia_balance
    from casbot_calib.hardware_wrist_exploration import FLAGS
    from casbot_calib.model_runtime import model_for
    from casbot_calib.data import write_json
    field.check_seal(root/'protocol');field.check_seal(root/'analysis')
    protocol=json.loads((root/'protocol/experiment.json').read_text());original=json.loads((root/'analysis/report.json').read_text())
    differences=[];rows=[];all_local=True;largest_feature=0.;largest_inertia=0.;largest_diagnostic=0.;phase_mixing=[]
    for job,saved in zip(protocol['jobs'],original['records']):
        trial=root/job['id'];plan,captured=field.resolve_field_capture(trial/'plan',trial/'capture');field.check_seal(plan);field.check_seal(captured)
        meta=json.loads((plan/'plan.json').read_text());motion=json.loads((plan/'motion.json').read_text())
        result=json.loads((captured/'result.json').read_text());request=json.loads((captured/'request.json').read_text())
        all_local &= result['local_contract_test'] and request['config']['connection']['scope']=='localhost_contract_mock'
        with (captured/'execution/commands.csv').open() as stream:command=next(csv.DictReader(stream))
        origin=int(command['planned_ns'])-int(command['planned_index'])*meta['command_period_ns']
        with (captured/'execution/feedback.csv').open() as stream:csvrows=list(csv.DictReader(stream))
        receive=np.array([int(r['receive_ns']) for r in csvrows],dtype=np.int64)
        obs=dict(receive_ns=receive,**{key:np.array([[float(r[prefix+str(j)]) for j in range(14)] for r in csvrows])
            for key,prefix in [('q','q'),('dq','v'),('effort_raw','effort_raw')]})
        elapsed=(receive-np.int64(origin))*1e-9;j=int(protocol['joint']);sign=job['direction'];held=[k for k in range(14) if k!=j]
        start,end=saved['measurement_window_s'];full=(elapsed>=start)&(elapsed<end)
        checks=dict(signed_speed_minimum_rad_s=float(np.min(sign*obs['dq'][full,j])),
                    maximum_held_speed_rad_s=float(np.max(abs(obs['dq'][full][:,held]))))
        start,end=saved['retained_window_s'];use=(elapsed>=start)&(elapsed<end);t=elapsed[use];t=t-t.mean()
        dt=np.diff(t);weights=np.r_[dt[0]/2,(dt[:-1]+dt[1:])/2,dt[-1]/2]
        design=np.column_stack([np.ones(len(t)),t,t*t,np.sin(10*np.pi*t),np.cos(10*np.pi*t)])
        y=(obs['effort_raw'][use]-np.asarray(meta['config']['effort_zero_nm']))*np.asarray(meta['config']['effort_gain'])
        engine,_=model_for(meta['config']);model=engine.model;model.gravity_world=np.asarray(meta['config']['gravity_world']);engine=type(engine)(model);v=obs['dq'][use,j];sgn=np.sign(v);decay=np.exp(-(v/.005)**2)
        prior=protocol['fixture_friction_prior'];friction=prior['viscous'][j]*v+prior['coulomb'][j]*sgn*(1-decay)+prior['static_friction'][j]*sgn*decay
        dynamic=y[:,j]-engine.gravity_batch(obs['q'][use])[:,j]-friction
        for key,values in [('estimated_effort_5hz_rms_nm',y[:,j]),('observed_minus_frozen_gravity_friction_5hz_rms_nm',dynamic)]:
            coef=np.linalg.lstsq(design*np.sqrt(weights)[:,None],values*np.sqrt(weights),rcond=1e-10)[0]
            checks[key]=float(np.linalg.norm(coef[-2:])/np.sqrt(2))
        differences.extend(abs(value-saved[key]) for key,value in checks.items())
        problem=probe_problem(obs,elapsed,motion,engine,protocol['probe_config'],meta['config'])
        with np.load(root/'analysis'/(job['id']+'_problem.npz'),allow_pickle=False) as d:
            for key in d.files:largest_feature=max(largest_feature,float(np.max(abs(d[key]-problem[key]))))
        inertia=inertia_balance(problem)
        largest_inertia=max(largest_inertia,abs(inertia['inertia_kg_m2']-saved['inertia_diagnostic']['inertia_kg_m2']))
        for diagnostic in saved.get('diagnostics',[]):
            if not diagnostic['available']:continue
            variant=diagnostic['variant'];cfg=deepcopy(protocol['probe_config'])
            cfg['processing']['fit_band_hz']=variant['fit_band_hz']
            alt=probe_problem(obs,elapsed,motion,engine,cfg,meta['config'],effort_relative_offset_s=variant['effort_offset_s'])
            estimate=inertia_balance(alt)
            largest_diagnostic=max(largest_diagnostic,abs(estimate['inertia_kg_m2']-diagnostic['inertia_kg_m2']))
            delta=variant['effort_offset_s']
            if delta:
                omega=2*np.pi*protocol['probe_config']['probe']['frequency_hz']
                joint_inertia=inertia['inertia_kg_m2'];viscous=inertia['viscous_nm_s_rad']
                expected=joint_inertia*np.cos(omega*delta)+viscous/omega*np.sin(omega*delta)
                phase_mixing.append(dict(job=job,imposed_offset_s=delta,
                    formula_inertia_kg_m2=float(expected),reprocessed_inertia_kg_m2=estimate['inertia_kg_m2'],
                    formula_mismatch_relative_to_primary=float(abs(expected-estimate['inertia_kg_m2'])/abs(joint_inertia)),
                    first_order_viscous_timing_fraction=float(abs(viscous*delta/joint_inertia))))
        rows.append(dict(job=job,independent_csv_checks=checks))
    checks=dict(exactly_two_records=len(rows)==len(original['records'])==2,
        local_sdk_only=bool(all_local),private_evaluation_access_denied=True,
        independent_window_speed_force_checks_identical=max(differences)<1e-12,
        public_probe_feature_replay_identical=largest_feature<1e-10,
        public_inertia_diagnostic_replay_identical=largest_inertia<1e-12,
        public_diagnostic_variants_replay_identical=largest_diagnostic<1e-12,
        cache_configuration_5ms=protocol['probe_config']['cache_period_ns']==5000000,
        original_10_percent_unchanged=protocol['original_inertia_relative_threshold']==.1)
    write_json(out/'verification.json',dict(passed=all(checks.values()),checks=checks,records=rows,
        maximum_scalar_check_difference=max(differences),maximum_feature_difference=largest_feature,
        maximum_inertia_diagnostic_difference=largest_inertia,device_writes=0,**FLAGS))
    if phase_mixing:
        write_json(out/'phase_mixing.json',dict(records=phase_mixing,
            formula='J_apparent = J*cos(omega*delta) + (b/omega)*sin(omega*delta)',
            scope='single-frequency inertia-viscous phase mixing approximation; imposed timing diagnostic, not measured SDK latency',
            physical_delay_estimated=False,used_for_selection=False))
    print(checks)


if __name__=='__main__':main()
