#!/usr/bin/env python3
"""Verify public dual-tone captures, then separately score the declared Mock model."""
import argparse
import csv
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
from scipy.optimize import minimize_scalar,nnls


def profiled_fit(frequencies,acceleration,velocity,known,observed,*,fixed_b=None):
    """Independent one-dimensional phase profile with a linear nonnegative fit."""
    w=2*np.pi*np.asarray(frequencies);A=np.asarray(acceleration);V=np.asarray(velocity);C=np.asarray(known);Y=np.asarray(observed)
    columns=np.column_stack([A,V]) if fixed_b is None else A[:,None]
    X=np.vstack([columns.real,columns.imag]);scales=np.linalg.norm(X,axis=0);X=X/scales
    def at(delta):
        target=np.exp(-1j*w*delta)*Y-C
        if fixed_b is not None:target-=fixed_b*V
        coefficients,norm=nnls(X,np.r_[target.real,target.imag])
        return norm,coefficients/scales
    bound=1/(4*max(frequencies));grid=np.linspace(-bound,bound,201);costs=[at(d)[0] for d in grid];solutions=[]
    for index in np.argsort(costs)[:3]:
        lo=grid[max(index-1,0)];hi=grid[min(index+1,len(grid)-1)]
        fit=minimize_scalar(lambda delta:at(delta)[0]**2,bounds=(lo,hi),method='bounded',options=dict(xatol=1e-14))
        norm,coef=at(fit.x);solutions.append((norm,coef,fit.x))
    norm,coef,delta=min(solutions,key=lambda x:x[0])
    return dict(inertia_kg_m2=float(coef[0]),viscous_nm_s_rad=float(coef[1] if fixed_b is None else fixed_b),
        effective_advance_s=float(delta),residual_complex_norm_nm=float(norm))


def public_checks(root,out):
    def guard(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
            raise RuntimeError('No private evaluation data in public capture verification')
    sys.addaudithook(guard)
    try:open('/__guard_probe__/evaluation/private','rb')
    except RuntimeError:pass
    else:raise RuntimeError('Evaluation guard inactive')
    from casbot_calib import hardware_roll_dual as dual
    from casbot_calib.hardware_probe import probe_problem
    from casbot_calib.hardware_wrist_exploration import read,FLAGS
    from casbot_calib.model_runtime import model_for
    from casbot_calib.data import write_json
    p=dual.load_protocol(root);dual.field.check_seal(root/'analysis');original=read(root/'analysis/report.json')
    out.mkdir(parents=True,exist_ok=False);scalar=0.;feature=0.;replay=0.;independent_J=0.;independent_delta=0.;all_local=True;records=[]
    for job,saved in zip(p['jobs'],original['records']):
        trial=root/job['id'];plan,captured=dual.field.resolve_field_capture(trial/'plan',trial/'capture')
        for folder in (plan,captured):dual.field.check_seal(folder)
        meta=read(plan/'plan.json');motion=read(plan/'motion.json');request=read(captured/'request.json');result=read(captured/'result.json')
        all_local &= result['passed'] and result['local_contract_test'] and request['config']['connection']['scope']=='localhost_contract_mock'
        with (captured/'execution/commands.csv').open() as stream:command=next(csv.DictReader(stream))
        origin=int(command['planned_ns'])-int(command['planned_index'])*meta['command_period_ns']
        with (captured/'execution/feedback.csv').open() as stream:raw=list(csv.DictReader(stream))
        receive=np.array([int(r['receive_ns']) for r in raw],dtype=np.int64)
        obs=dict(receive_ns=receive,**{key:np.array([[float(r[prefix+str(j)]) for j in range(14)] for r in raw])
            for key,prefix in [('q','q'),('dq','v'),('effort_raw','effort_raw')]})
        elapsed=(receive-np.int64(origin))*1e-9;j=13;held=list(range(13));begin,end=saved['measurement_window_s'];full=(elapsed>=begin)&(elapsed<end)
        checks=dict(signed_speed_minimum_rad_s=float(np.min(job['direction']*obs['dq'][full,j])),maximum_held_speed_rad_s=float(np.max(abs(obs['dq'][full][:,held]))))
        scalar=max(scalar,max(abs(checks[k]-saved[k]) for k in checks))
        begin,end=saved['retained_window_s'];use=(elapsed>=begin)&(elapsed<end);t=elapsed[use];u=t-t.mean();dt=np.diff(t);weights=np.r_[dt[0]/2,(dt[:-1]+dt[1:])/2,dt[-1]/2]
        frequencies=[f for f,a in p['tones']]
        # Independent column order (sin,cos), common full-window trend and weights.
        X=np.column_stack([np.ones(len(t)),u,u*u,*[c for f in frequencies for c in (np.sin(2*np.pi*f*u),np.cos(2*np.pi*f*u))]])
        profile=meta['config'];engine,_=model_for(profile);v=obs['dq'][use,j];sign=np.sign(v);decay=np.exp(-(v/.005)**2);prior=p['fixture_friction_prior']
        friction=prior['viscous'][j]*v+prior['coulomb'][j]*sign*(1-decay)+prior['static_friction'][j]*sign*decay
        tau=((obs['effort_raw']-np.asarray(profile['effort_zero_nm']))*np.asarray(profile['effort_gain']))[use,j]
        dynamic=tau-engine.gravity_batch(obs['q'][use])[:,j]-friction
        coef=np.linalg.lstsq(X*np.sqrt(weights)[:,None],np.column_stack([tau,dynamic,v])*np.sqrt(weights)[:,None],rcond=1e-10)[0]
        rms=np.hypot(coef[3::2],coef[4::2])/np.sqrt(2)
        for k,s in enumerate(saved['frequency_signals']):
            for col,key in enumerate(['estimated_effort_rms_nm','observed_minus_frozen_gravity_friction_rms_nm','reported_velocity_rms_rad_s']):scalar=max(scalar,abs(float(rms[k,col])-s[key]))
        problem=probe_problem(obs,elapsed,motion,engine,p['probe_config'],profile)
        with np.load(root/'analysis'/(job['id']+'_problem.npz'),allow_pickle=False) as data:
            for key in data.files:feature=max(feature,float(np.max(abs(problem[key]-data[key]))))
        fresh=dual.frequency_analysis(problem,prior,p['observation_policy'],frequencies);old=saved['frequency_analysis']
        for oldfit,newfit in zip([old['full']]+old['leave_block_out'],[fresh['full']]+fresh['leave_block_out']):
            for model in ['free_b_complex','frozen_b_complex']:
                for key in ['inertia_kg_m2','viscous_nm_s_rad','effective_advance_s','residual_complex_norm_nm']:
                    replay=max(replay,abs(oldfit[model][key]-newfit[model][key]))
        values=np.asarray(fresh['full']['components_real_imag']);z=values[:,:,0]+1j*values[:,:,1];Y,A,V,C=z.T;fits={}
        for model,b in [('free_b_complex',None),('frozen_b_complex',prior['viscous'][j])]:
            independent=profiled_fit(frequencies,A,V,C,Y,fixed_b=b);recorded=old['full'][model]
            independent_J=max(independent_J,abs(independent['inertia_kg_m2']-recorded['inertia_kg_m2']))
            independent_delta=max(independent_delta,abs(independent['effective_advance_s']-recorded['effective_advance_s']))
            fits[model]=independent
        records.append(dict(job=job,independent_csv_checks=checks,independent_profiled_fits=fits))
    checks=dict(exactly_two=len(p['jobs'])==len(original['records'])==len(records)==2,local_sdk_only=bool(all_local),evaluation_guard_active=True,
        independent_speed_and_tones_identical=scalar<1e-12,feature_rebuild_identical=feature<1e-10,all_window_model_replay_identical=replay<1e-12,
        independent_profiled_inertia_agrees=independent_J<1e-9,independent_profiled_phase_agrees=independent_delta<1e-6,
        cache_5ms=p['probe_config']['cache_period_ns']==5000000,original_threshold_preserved=p['original_inertia_relative_threshold']==.1)
    write_json(out/'verification.json',dict(passed=all(checks.values()),checks=checks,records=records,maximum_scalar_difference=scalar,maximum_feature_difference=feature,
        maximum_window_model_replay_difference=replay,maximum_profiled_inertia_difference=independent_J,maximum_profiled_phase_difference_s=independent_delta,
        device_writes=0,formal_test_consumed=False,**FLAGS))
    print(checks)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--public-only',action='store_true',help=argparse.SUPPRESS);a=p.parse_args()
    root=a.source.resolve();out=a.output.resolve()
    if a.public_only:public_checks(root,out);return
    out.mkdir(parents=True,exist_ok=False)
    with (out/'public_verification.log').open('x') as log:
        subprocess.run([sys.executable,__file__,str(root),'--output',str(out/'public'),'--public-only'],stdout=log,stderr=subprocess.STDOUT,check=True)
    from casbot_calib.hardware_wrist_exploration import read,FLAGS
    from casbot_calib.data import write_json
    from casbot_calib.model_runtime import model_for
    from casbot_calib import hardware_field as field
    report=read(root/'analysis/report.json');protocol=read(root/'protocol/experiment.json');field.check_seal(root/'mock/protocol')
    behavior=read(root/'mock/protocol/behavior.json');nominal,_=model_for(behavior['config']);model=nominal.model;joints=model.joints
    for j,joint in enumerate(joints):
        body=joint.body;body.inertia_com=body.inertia_com*behavior['config']['plant']['inertia_scales'][j];joint.body=body
    model.joints=joints;synthetic=type(nominal)(model);scores=[]
    for row in report['records']:
        with np.load(root/'analysis'/(row['job']['id']+'_problem.npz'),allow_pickle=False) as data:q=data['q']
        true=synthetic.evaluate_batch(q,np.zeros_like(q),np.zeros_like(q))['mass_matrix'][:,13,13]-model.joints[13].rotor_inertia
        values={name:row['inertia_diagnostic']['inertia_kg_m2'] if name=='zero_phase_local' else row['frequency_analysis']['full'][name]['inertia_kg_m2'] for name in protocol['observation_policy']['comparison_models']}
        scores.append(dict(job=row['job'],mean_synthetic_rigid_inertia_kg_m2=float(np.mean(true)),models={k:dict(inertia_kg_m2=J,maximum_relative_error=float(np.max(abs(J-true)/true))) for k,J in values.items()}))
    primary=protocol['observation_policy']['primary_model'];worst=max(s['models'][primary]['maximum_relative_error'] for s in scores)
    write_json(out/'selected_axis_synthetic_evaluation.json',dict(joint=13,scope='Only right-wrist-roll local rigid inertia on the two delivered paths; not fullP0R2',records=scores,
        primary_model=primary,maximum_primary_relative_error=worst,within_original10_percent=worst<=.1,
        truth_used_for_fitting=False,truth_used_for_selection=False,physical_measurement=False))
    public=read(out/'public/verification.json');write_json(out/'verification.json',dict(passed=public['passed'],public_verification_passed=public['passed'],selected_axis_synthetic_error_within10_percent=worst<=.1,
        primary_direction_difference_relative=report['model_comparisons'][primary]['direction_inertia_difference_relative'],full_p0_r1_r2_acceptance=False,device_writes=0,formal_test_consumed=False,**FLAGS))
    print(dict(public_verification=public['passed'],primary_synthetic_error=worst))


if __name__=='__main__':main()
