#!/usr/bin/env python3
"""Read-only independent phasors plus native-gravity analytical readout fixtures."""
import argparse
import json
import os
from pathlib import Path
import sys
import numpy as np
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('design',type=Path);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    def guard(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
            raise RuntimeError('No evaluation data in offline frequency verification')
    sys.addaudithook(guard)
    from casbot_calib import hardware_field as field
    from casbot_calib.hardware_roll_frequency import phasors,fit_components,multitone_motion,response
    from casbot_calib.hardware_probe import probe_problem
    from casbot_calib.model_runtime import model_for,collision_for
    from casbot_calib.hardware_wrist_exploration import read,FLAGS
    from casbot_calib.data import write_json
    design=a.design.resolve();report=read(design/'report.json');root=Path(report['protocol']['source']);out=field.output_outside(a.output,root);out.mkdir(parents=True,exist_ok=False)
    field.check_seal(design);field.check_seal(root/'protocol');field.check_seal(root/'analysis')
    p=read(root/'protocol/experiment.json');scalar_diff=0.;complex_diff=0.
    for row in report['records']:
        with np.load(root/'analysis'/(row['job']['id']+'_problem.npz'),allow_pickle=False) as d:data={k:d[k] for k in d.files}
        t=data['time_s'];u=t-(t[0]+t[-1])/2;dt=np.diff(t);w=np.r_[dt[0]/2,(dt[:-1]+dt[1:])/2,dt[-1]/2]
        j=13;fr=data['extra']@np.r_[p['fixture_friction_prior']['viscous'],p['fixture_friction_prior']['coulomb']]+p['fixture_friction_prior']['static_friction'][j]*data['static_feature']
        y=np.column_stack([data['observed_torque_nm'],data['ddq'][:,j],data['dq'][:,j],data['projected_gravity_nm']+data['rotor'],fr])
        X=np.column_stack([np.ones(len(t)),u,u*u,np.cos(10*np.pi*u),np.sin(10*np.pi*u)])
        coef=np.linalg.lstsq(X*np.sqrt(w)[:,None],y*np.sqrt(w)[:,None],rcond=1e-10)[0];z=coef[-2]-1j*coef[-1]
        saved=np.asarray(row['complex_channels_real_imag']);complex_diff=max(complex_diff,float(np.max(abs(z-(saved[:,0]+1j*saved[:,1])))))
        A=z[1];C=z[3]+z[4];Y=z[0]
        # Scalar quadratic formula is independent of frozen_single's roots call.
        aa=abs(A)**2;bb=2*np.real(A.conjugate()*C);cc=abs(C)**2-abs(Y)**2
        J=(-bb+np.sqrt(bb*bb-4*aa*cc))/(2*aa)
        scalar_diff=max(scalar_diff,float(abs(J-row['frozen_friction']['solutions'][0]['inertia_kg_m2'])))
    selected=next(d for d in report['designs'] if d['id']==report['selected_id']);freq=np.array([x[0] for x in selected['tones']]);amps=np.array([x[1] for x in selected['tones']])
    profile=yaml.safe_load((root/'protocol/base_profile.yaml').read_text());engine,_=model_for(profile)
    scene=collision_for(profile);frozen_scene=yaml.safe_load((root/'protocol/scene.yaml').read_text())
    assert scene.metadata['scene']==frozen_scene
    meta=read(root/p['jobs'][0]['id']/'plan/motion.json');home=np.asarray(meta['states'][0][0]);J0=report['planning_inertia_kg_m2'];b0=report['planning_viscous_nm_s_rad'];cases=[]
    for direction in [-1,1]:
        motion=multitone_motion('analytical_readout',home,p['pose_rad'],13,direction,p['probe_config'],selected['tones'],transition_s=p['transition_s'],settle_s=p['settle_s'])
        window=motion.windows('probe_measure');begin=window[0][0];end=window[-1][1];t=np.arange(begin-.02,end+.02,.001)
        cached=np.floor((t-.0013)/.005)*.005+.0013;q,v,_,_=motion.at(cached)
        for advance in [-.005,0.,.005]:
            tq,tv,ta,_=motion.at(cached+advance);effort=engine.gravity_batch(tq)
            speed=tv[:,13];sgn=np.sign(speed);decay=np.exp(-(speed/.005)**2)
            effort[:,13]+=J0*ta[:,13]+b0*speed+.005*sgn*(1-decay)+.01*sgn*decay
            obs=dict(receive_ns=10**15+np.rint(t*1e9).astype(np.int64),q=q,dq=v,effort_raw=effort)
            d=probe_problem(obs,t,motion.metadata(),engine,p['probe_config'],profile);tt=d['time_s'];origin=(tt[0]+tt[-1])/2
            known=d['projected_gravity_nm']+d['rotor']+.005*d['extra'][:,27]+.01*d['static_feature']
            zs=phasors(tt,np.column_stack([d['observed_torque_nm'],d['ddq'][:,13],d['dq'][:,13],known]),freq,origin_s=origin)
            fit=fit_components(freq,zs[:,1],zs[:,2],zs[:,3],zs[:,0],J0,b0)
            cases.append(dict(direction=direction,imposed_advance_s=advance,fit=fit,
                relative_inertia_error=abs(fit['inertia_kg_m2']/J0-1),effective_advance_error_s=abs(fit['effective_advance_s']-advance)))
    # Gain counterexample: low complex residual does not certify absolute scale.
    gain_counter=[];V=2*np.pi*freq*amps;K=report['nominal_gravity_stiffness_nm_rad']
    for gain in [.85,1.15]:
        y=response(freq,V,J0,b0,0.,K,gain=gain)
        fit=fit_components(freq,1j*2*np.pi*freq*V,V,K/(1j*2*np.pi*freq)*V,y,J0,b0)
        gain_counter.append(dict(imposed_gain=gain,fit=fit,relative_inertia_error=abs(fit['inertia_kg_m2']/J0-1)))
    checks=dict(independent_public_phasor_difference_zero=complex_diff<1e-12,
        independent_frozen_single_inertia_difference_zero=scalar_diff<1e-12,
        frozen_scene_matches_native_reference_scene=True,
        native_gravity_shared_cache_fixture_under_original10_percent=max(x['relative_inertia_error'] for x in cases)<.1,
        native_gravity_effective_phase_fixture_error_under_0p1ms=max(x['effective_advance_error_s'] for x in cases)<.0001,
        selected_full_and_500hz_reference_checks_passed=all(x['full_reference_passed'] and x['command_preview_passed'] for x in report['selected_reference_checks']))
    write_json(out/'verification.json',dict(passed=all(checks.values()),checks=checks,maximum_complex_difference=complex_diff,maximum_inertia_difference=scalar_diff,
        analytical_native_gravity_readout_fixtures=cases,scale_counterexamples=gain_counter,
        analytical_fixture_only=True,collection_performed=False,formal_test_consumed=False,**FLAGS))
    recommendation=dict(status='offline_design_not_collected',source_design=str(design),source_records=str(root),
        joint=13,center='c00',pose_rad=p['pose_rad'],tones=selected['tones'],bias_speed_rad_s=.18,measurement_duration_s=2.,
        proposed_jobs=[dict(id='c00_roll_dual_'+name,direction=direction) for name,direction in [('neg',-1),('pos',1)]],
        proposed_records=2,command_hz=500,cache_standard_ns=5000000,
        main_processing='Same callback rows and30Hz matched operator; simultaneous2.5/10Hz+quadratic trend, compare frozen-b and free-b complex models; per-record effective phase, no fabricated acquisition time',
        gravity='frozen nominal Pinocchio gravity evaluated on delivered q; no linear-stiffness approximation in observation analysis',
        friction='frozen fc/fs; fitted b is diagnostic nuisance only, no H3/friction candidate update',
        next_analysis='Check actual signed speed, held axes, both tone signals, effective phase consistency, leave-window sensitivity and positive/negative local inertia. No automatic expansion or R2 qualification.',
        adapter_work_before_collection='Existing capture analysis has a single5Hz tone; adapt multi-tone observation analysis and freeze the new exploratory protocol. At execution rebuild full paths from measured starting snapshots.',
        formal_10_percent_pass_predicted=False,original_threshold=.1,formal_test_consumed=False,collection_authorized=False,**FLAGS)
    write_json(out/'recommendation.json',recommendation)
    print(checks);print('maximum_native_fixture_error',max(x['relative_inertia_error'] for x in cases))


if __name__=='__main__':main()
