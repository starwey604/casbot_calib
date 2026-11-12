#!/usr/bin/env python3
"""Compare existing roll phasors and plan (never collect) a minimal multitone pair."""
import argparse
import json
import os
from pathlib import Path
import sys
import numpy as np
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    def guard(event,args):
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
            raise RuntimeError('Private evaluation data forbidden in offline roll design')
    sys.addaudithook(guard)
    from casbot_calib.hardware_probe import exposure
    from casbot_calib.hardware_roll_frequency import phasors,frozen_single,response,information,fit_response,multitone_motion,magnitude_pair
    from casbot_calib import hardware_field as field
    from casbot_calib.hardware_wrist_exploration import FLAGS,read
    from casbot_calib.data import write_json
    from casbot_calib.model_runtime import model_for,collision_for
    from casbot_calib.staged_motion import derivative_bounds,gate_motion
    from casbot_calib.collision import certify_polyline
    from casbot_calib.run_marker import run_marker
    root=a.source.resolve();out=field.output_outside(a.output,root);out.mkdir(parents=True,exist_ok=False)
    field.check_seal(root/'protocol');field.check_seal(root/'analysis');p=read(root/'protocol/experiment.json')
    if p['schema_version']!='casbot_sdk_wrist_roll_screen_v1' or p['joint']!=13:raise ValueError('Existing c00 roll pair required')
    # Iterative offline design grid, recorded here for replay; not heldout validation.
    candidates=[dict(id='repeat_5hz',tones=[[5.,.004]]),
        dict(id='add_2p5hz_separately',tones=[[2.5,.008]]),
        dict(id='dual_2p5_7p5_same_budget',tones=[[2.5,.005],[7.5,.001]]),
        dict(id='dual_2p5_7p5',tones=[[2.5,.005],[7.5,.0015]]),
        dict(id='dual_2p5_10',tones=[[2.5,.0075],[10.,.00068]]),
        dict(id='triple_2p5_5_7p5',tones=[[2.5,.003],[5.,.001],[7.5,.001]])]
    protocol=dict(source=str(root),input_role='two existing public roll records; no new observations',
        candidates=candidates,original_threshold=.1,cache_standard_ns=5000000,
        phase_convention='positive delta advances effort, effective model phase not physical acquisition latency',
        fixed_model='unit gain, frozen nominal gravity, known rotor, no unknown effort filter',
        friction_multipliers=[.8,1.,1.2],imposed_advances_s=[-.005,0.,.005],
        temporal_omission='four fixed0.4s blocks from the same1.6s retained window; not independent acquisitions or confidence intervals',
        design_noise='conditional constant complex-phasor error budget equal to largest observed leave-block-out change at5Hz; not measured noise at proposed frequencies',
        selection='two frequencies in each signed record; compare certified signed-speed margin and conditional sensitivity within the existing1/8/200 envelope',
        collection_performed=False,formal_test_consumed=False,**FLAGS,**run_marker())
    write_json(out/'protocol.json',protocol)
    rows=[];noise_budget=0.;j0s=[]
    for job in p['jobs']:
        with np.load(root/'analysis'/(job['id']+'_problem.npz'),allow_pickle=False) as d:problem={k:d[k] for k in d.files}
        j=13;t=problem['time_s'];origin=float((t[0]+t[-1])/2);freq=5.;weights=exposure(t)
        friction=problem['extra']@np.r_[p['fixture_friction_prior']['viscous'],p['fixture_friction_prior']['coulomb']]+p['fixture_friction_prior']['static_friction'][j]*problem['static_feature']
        channels=np.column_stack([problem['observed_torque_nm'],problem['ddq'][:,j],problem['dq'][:,j],
            problem['projected_gravity_nm']+problem['rotor'],friction])
        def estimate(mask):
            z=phasors(t[mask],channels[mask],[freq],origin_s=origin,weights=weights[mask])[0]
            frozen=frozen_single(z[0],z[1],z[3]+z[4],freq)
            return z,frozen
        z,full=estimate(np.ones(len(t),bool));omissions=[]
        start=t[0]
        for k in range(4):
            mask=~((t>=start+.4*k-1e-8)&(t<start+.4*(k+1)-1e-8));zz,ff=estimate(mask)
            change=float(abs(zz[0]-z[0]));noise_budget=max(noise_budget,change)
            omissions.append(dict(block=k,effort_phasor_change_nm=change,frozen_fit=ff))
        sensitivity=[]
        for factor in protocol['friction_multipliers']:
            sensitivity.append(dict(friction_multiplier=factor,fit=frozen_single(z[0],z[1],z[3]+factor*z[4],freq)))
        shifts=[]
        for delta in protocol['imposed_advances_s']:
            shifts.append(dict(imposed_advance_s=delta,fit=frozen_single(z[0]*np.exp(2j*np.pi*freq*delta),z[1],z[3]+z[4],freq)))
        local=read(root/'analysis/report.json')['records'][len(rows)]['inertia_diagnostic'];j0s.append(local['inertia_kg_m2'])
        # Exact family: freely changing b and compensating J/phase fits one tone.
        rows.append(dict(job=job,complex_channels_real_imag=np.column_stack([z.real,z.imag]).tolist(),
            frozen_friction=full,friction_sensitivity=sensitivity,imposed_phasor_advances=shifts,
            omitted_blocks=omissions,local_zero_phase=local))
    # Hypothetical design inertia is centered on public diagnostics, not plant truth.
    j0=float(np.mean(j0s));b0=p['fixture_friction_prior']['viscous'][13]
    profile=yaml.safe_load((root/'protocol/base_profile.yaml').read_text());profile['collision_scene']=str(root/'protocol/scene.yaml')
    engine,_=model_for(profile);model=engine.model;model.gravity_world=np.asarray(profile['gravity_world']);engine=type(engine)(model)
    pose=np.asarray(p['pose_rad']);eps=1e-6;up=pose.copy();down=pose.copy();up[13]+=eps;down[13]-=eps
    stiffness=float((engine.gravity(up)[13]-engine.gravity(down)[13])/(2*eps))
    # Source home is a sealed reference, not a newly measured robot pose.
    first=p['jobs'][0]['id'];motionmeta=read(root/first/'plan/motion.json');home=np.asarray(motionmeta['states'][0][0])
    designs=[]
    for candidate in candidates:
        f=np.array([x[0] for x in candidate['tones']]);amps=np.array([x[1] for x in candidate['tones']]);v=2*np.pi*f*amps
        info=information(f,v,j0,b0,stiffness)
        motion=multitone_motion(candidate['id'],home,pose,13,1,p['probe_config'],candidate['tones'],transition_s=p['transition_s'],settle_s=p['settle_s'])
        curve=motion.polynomial();peaks=[float(np.max(derivative_bounds(curve,k))) for k in (1,2,3)]
        measure=[k for k,l in enumerate(motion.labels) if l=='probe_measure']
        minv=float(np.min(curve.derivative().c[:,measure,13]))
        envelope=all(x<=y for x,y in zip(peaks,[1.,8.,200.])) and minv>.02
        row=dict(**candidate,information=info,continuous_peak_bounds=peaks,minimum_signed_velocity_bound_rad_s=minv,
            dedicated_envelope_passed=bool(envelope),measurement_duration_s=p['probe_config']['probe']['duration_s'],
            same_record_free_J_b_phase=info['rank']==3,
            hypothetical_inertia_signal_rms_nm=(j0*(2*np.pi*f)**2*amps/np.sqrt(2)).tolist())
        if info['rank']==3:
            n=len(f);s=np.array(info['inertia_sensitivity_nm_inverse'])
            row['conditional_linear_worst_inertia_fraction']=float(noise_budget*np.hypot(s[:n],s[n:]).sum())
            noiseless=[]
            for bfactor in protocol['friction_multipliers']:
                for delta in protocol['imposed_advances_s']:
                    observed=response(f,v,j0,b0*bfactor,delta,stiffness)
                    fit=fit_response(f,v,observed,j0,b0,stiffness)
                    noiseless.append(dict(viscous_factor=bfactor,imposed_advance_s=delta,fit=fit,
                        relative_inertia_error=abs(fit['inertia_kg_m2']/j0-1)))
            row['analytic_model_closure']=noiseless
            mismatches=[]
            for gain,tau in [(1.,.002),(1.,.005),(.9,0.),(1.1,0.)]:
                observed=response(f,v,j0,b0,0.,stiffness,gain=gain,tau=tau)
                fit=fit_response(f,v,observed,j0,b0,stiffness)
                mismatches.append(dict(imposed_effort_gain=gain,imposed_effort_lowpass_s=tau,fit=fit,
                    relative_inertia_error=abs(fit['inertia_kg_m2']/j0-1)))
            row['unmodelled_filter_scale_diagnostics']=mismatches
        if candidate['id']=='add_2p5hz_separately':
            combined_f=np.array([2.5,5.]);combined_v=2*np.pi*combined_f*np.array([.008,.004])
            combined_y=response(combined_f,combined_v,j0,b0,0.,stiffness)*np.exp(1j*np.array([.17,-.24]))
            recovery=magnitude_pair(combined_f,combined_v,combined_y,stiffness)
            # Different per-record phases drop out of amplitudes, but there is
            # no within-record cross-frequency model residual in this option.
            w=2*np.pi*combined_f;h=b0+1j*(w*j0-stiffness/w)
            d=np.column_stack([np.abs(combined_v)*(w*j0-stiffness/w)*w*j0/np.abs(h),np.abs(combined_v)*b0*b0/np.abs(h)])
            sensitivity=np.linalg.inv(d)[0]
            row['existing5hz_plus_new2p5_amplitudes']=dict(recovery=recovery,amplitude_rank=int(np.linalg.matrix_rank(d)),
                shared_phase_between_records_required=False,extra_complex_equations_after_J_b_two_phases=0,
                conditional_linear_worst_inertia_fraction=float(noise_budget*np.sum(abs(sensitivity))))
        designs.append(row)
    # Freeze the selected comparison after inspecting information, before gates.
    selected=next(x for x in designs if x['id']=='dual_2p5_10')
    f=np.array([x[0] for x in selected['tones']]);v=2*np.pi*f*np.array([x[1] for x in selected['tones']])
    ideal=response(f,v,j0,b0,0.,stiffness);noise_cases=[]
    for first_phase in np.arange(8)*np.pi/4:
        for second_phase in np.arange(8)*np.pi/4:
            perturbation=noise_budget*np.exp(1j*np.array([first_phase,second_phase]))
            fit=fit_response(f,v,ideal+perturbation,j0,b0,stiffness)
            noise_cases.append(dict(noise_phases_rad=[float(first_phase),float(second_phase)],fit=fit,relative_inertia_error=abs(fit['inertia_kg_m2']/j0-1)))
    selected['conditional_nonlinear_noise_cases']=noise_cases
    selected['maximum_conditional_nonlinear_noise_inertia_error']=max(x['relative_inertia_error'] for x in noise_cases)
    if not selected['dedicated_envelope_passed']:raise ValueError('Selected reference envelope rejected')
    reference=[]
    for direction in (-1,1):
        name='c00_roll_multifrequency_'+('neg' if direction<0 else 'pos')
        motion=multitone_motion(name,home,pose,13,direction,p['probe_config'],selected['tones'],transition_s=p['transition_s'],settle_s=p['settle_s'])
        scene=collision_for(profile);gate=gate_motion(scene,motion,scene.metadata['scene']['clearance_m'])
        folder=out/'references'/name;folder.mkdir(parents=True);write_json(folder/'motion.json',motion.metadata());write_json(folder/'collision_reference.json',gate)
        curve=motion.polynomial();np.savez_compressed(folder/'reference.npz',coefficients=curve.c,breakpoints=curve.x)
        t=np.arange(0,int(round(motion.duration*1e9))+1,2000000,dtype=np.int64)*1e-9
        preview=certify_polyline(scene,curve(t),t,clearance_m=scene.metadata['scene']['clearance_m'],block_samples=1000)
        write_json(folder/'collision_command_preview.json',preview)
        reference.append(dict(name=name,full_reference_passed=gate['passed'],command_preview_passed=preview['passed'],
            clearance_m=scene.metadata['scene']['clearance_m']))
    frozen=[x['frozen_friction']['solutions'][0] for x in rows if x['frozen_friction']['available']]
    report=dict(protocol=protocol,records=rows,planning_inertia_kg_m2=j0,planning_viscous_nm_s_rad=b0,nominal_gravity_stiffness_nm_rad=stiffness,
        empirical_5hz_leave_block_phasor_budget_nm=noise_budget,designs=designs,
        frozen_friction_direction_inertia_difference=(abs(frozen[1]['inertia_kg_m2']/frozen[0]['inertia_kg_m2']-1) if len(frozen)==2 else None),
        selected_id=selected['id'],selected_new_records=2,selected_reference_checks=reference,
        selection_reason='Two within-record tones estimate J/b/effective phase and retain one real model-check equation; separate frequencies can also identify J/b from amplitudes without shared phase but have no such residual.The2.5/10Hz candidate has lower conditional noise sensitivity and a larger certified signed-speed margin than2.5/7.5Hz, within the original1/8/200 envelope.',
        observation_processing_adapter_still_required=True,formal_10_percent_pass_predicted=False,**FLAGS)
    write_json(out/'report.json',report)
    plot(report,out)
    write_json(out/'delivery.json',dict(completed=True,records_read=2,new_records_collected=0,original_threshold=.1,formal_test_consumed=False,
        selected_id=selected['id'],proposed_records=2,references_passed=all(x['full_reference_passed'] and x['command_preview_passed'] for x in reference),
        phase_is_effective_model_parameter=True,formal_10_percent_pass_predicted=False,**FLAGS))
    field.seal(out)
    print(dict(selected=selected['id'],phasor_budget_nm=noise_budget,references=reference),flush=True)


def plot(report,out):
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from casbot_calib.presentation import _font
    font_manager.fontManager.addfont(_font(12).path);plt.rcParams['font.family']=font_manager.FontProperties(fname=_font(12).path).get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
    for row,label in zip(report['records'],['负向','正向']):
        xs=[];ys=[]
        for variant in row['friction_sensitivity']:
            if variant['fit']['available']:
                xs.append(variant['friction_multiplier']);ys.append(variant['fit']['solutions'][0]['inertia_kg_m2']*1e4)
        axes[0].plot(xs,ys,'o-',label=label)
    axes[0].set(xlabel='冻结摩擦系数倍数',ylabel='局部惯量（×1e-4 kg·m²）',title='单频：摩擦假设改变惯量');axes[0].legend()
    ds=[d for d in report['designs'] if d['same_record_free_J_b_phase']]
    labels=['双频\n同预算','双频\n7.5Hz','双频\n10Hz','三频']
    axes[1].bar(np.arange(len(ds)),[100*d['conditional_linear_worst_inertia_fraction'] for d in ds],color=['#1c897f' if d['id']==report['selected_id'] else '#aaa' for d in ds])
    axes[1].set_xticks(np.arange(len(ds)),labels);axes[1].axhline(10,color='r',ls='--');axes[1].set(ylabel='条件线性最坏惯量变化（%）',title='假设各新频点沿用现有误差预算')
    row=next(d for d in report['designs'] if d['id']==report['selected_id'])
    t=np.linspace(0,2,2000);v=np.full(len(t),.18)
    for f,a in row['tones']:v+=a*2*np.pi*f*np.cos(2*np.pi*f*t)
    axes[2].plot(t,v);axes[2].axhline(.02,color='r',ls='--');axes[2].set(xlabel='测量窗时间（s）',ylabel='参考速度（rad/s）',title='推荐：2.5Hz + 10Hz 非换向')
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(out/'frequency_design.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
