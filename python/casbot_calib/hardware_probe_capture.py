"""Local SDK wrist probe pairs and public-feedback functional inspection."""
from pathlib import Path
from copy import deepcopy
from shutil import copyfile
import numpy as np
import yaml
from .data import write_json
from .run_marker import run_marker
from . import hardware_field as field
from .hardware_probe import settings,make_probe,probe_problem,inertia_balance,exposure,legacy_quality
from .hardware_wrist_exploration import FLAGS,require_local,read
from .hardware_inertia import settings as h4_settings
from .hardware_friction import holding_gains
from .model_runtime import model_for
from .sdk_mock import SDKMockSession
from .vendor_identity import sha256

JOINT=12
JOBS=[dict(id='c00_right_pitch_neg',direction=-1),dict(id='c00_right_pitch_pos',direction=1)]


def design(output):
    root=Path(output).resolve();root.mkdir(parents=True,exist_ok=False)
    protocol=root/'protocol';protocol.mkdir();c=settings()
    mock=yaml.safe_load(Path('configs/sdk_mock.yaml').read_text())
    engine,_=model_for(mock);h4=h4_settings('configs/hardware_inertia.yaml',[j.name for j in engine.model.joints])
    pose=np.asarray(h4['home_rad']).copy();pose[[5,12]]=.2;pose[[6,13]]=.3
    # Keep the preceding local fixture, changing only cache/probe recipe.
    prior=dict(viscous=[.01]*14,coulomb=[.005]*14,static_friction=[.01]*14)
    mock['plant'].update(stiction_enabled=True,sample_period_ns=1000000,
        friction_viscous=prior['viscous'],friction_coulomb=prior['coulomb'],
        friction_static=prior['static_friction'],inertia_scales=[1.8]*14)
    mock['cache_period_ns']=c['cache_period_ns']
    mock['assumptions']['cache']='user_selected_synthetic_5ms_shared_cache_not_measured_device_sampling'
    (protocol/'mock.yaml').write_text(yaml.safe_dump(mock))
    holding=dict(h4['holding'],hold_minimum_kp=30.)
    p=dict(schema_version='casbot_sdk_wrist_probe_pair_v1',probe_config=c,jobs=JOBS,joint=JOINT,
        pose_rad=pose.tolist(),holding=holding,transition_s=h4['transition_s'],settle_s=h4['settle_s'],
        fixture_friction_prior=prior,prior_status='declared synthetic fixture, not H3 acceptance',
        command_hz=c['command_hz'],original_inertia_relative_threshold=.10,
        signal_diagnostics='5Hz and adjacent4.375/5.625Hz tones with quadratic drift, same retained callback window',
        automatic_expansion=False,maximum_trials=2,training_dataset_refitted=False,formal_test_consumed=False,
        recovery_policy='log_noop_and_terminate_whole_session_no_fault_resume',**FLAGS,**run_marker())
    write_json(protocol/'experiment.json',p);field.seal(protocol)
    return p


def design_roll_screen(source,output):
    """Clone only the execution/observation recipe, not the pitch candidate."""
    source=Path(source).resolve();field.check_seal(source/'protocol')
    old=read(source/'protocol/experiment.json')
    if old['schema_version']!='casbot_sdk_wrist_probe_campaign_v1':
        raise ValueError('SDK campaign execution recipe required')
    root=field.output_outside(output,source);root.mkdir(parents=True,exist_ok=False)
    protocol=root/'protocol';protocol.mkdir()
    for name in ('mock.yaml','base_profile.yaml','scene.yaml'):copyfile(source/'protocol'/name,protocol/name)
    p={k:deepcopy(old[k]) for k in ('probe_config','holding','transition_s','settle_s','fixture_friction_prior','prior_status','command_hz')}
    p.update(schema_version='casbot_sdk_wrist_roll_screen_v1',joint=13,joint_label='右腕 roll',
        pose_rad=old['poses']['c00'],base_profile='base_profile.yaml',
        jobs=[dict(id='c00_right_roll_'+tag,direction=d) for d,tag in [(-1,'neg'),(1,'pos')]],
        source_recipe=str(source),source_protocol_manifest_sha256=sha256(source/'protocol/manifest.json'),
        role='local_signal_screen_only',pitch_candidate_used_for_roll=False,full_rigid_fit_planned=False,
        maximum_trials=2,automatic_expansion=False,original_inertia_relative_threshold=.1,
        diagnostic_variants=[dict(id='band10',fit_band_hz=10.,effort_offset_s=0.),dict(id='band20',fit_band_hz=20.,effort_offset_s=0.),
            dict(id='effort_minus5ms',fit_band_hz=30.,effort_offset_s=-.005),dict(id='effort_plus5ms',fit_band_hz=30.,effort_offset_s=.005)],
        diagnostics_used_for_selection=False,formal_test_consumed=False,
        recovery_policy='log_noop_and_terminate_whole_session_no_fault_resume',**FLAGS,**run_marker())
    write_json(protocol/'experiment.json',p);field.seal(protocol);return p


def terminate_fault(root,job,phase,result):
    from .hardware_batch import SDKBatchBackend
    reason=result.get('fault') or result.get('reason') or phase+'_failed'
    recovery=SDKBatchBackend().clear_fault(reason)
    write_json(Path(root)/'clear_fault.json',recovery)
    write_json(Path(root)/'collection.json',dict(completed=False,terminal_fault=True,
        failed_job=job,phase=phase,resume_permitted=False,clear_fault=recovery,**FLAGS))
    raise RuntimeError('SDK fault; terminate whole session without recovery or resume: '+reason)


def collect_jobs(root,p,session,domain,jobs,*,motion_builder=None):
    """Shared measured-start executor; callers own their frozen job inventory."""
    root=Path(root)
    for job in jobs:
        trial=root/job['id'];trial.mkdir();print('COLLECT',job['id'],flush=True)
        result=field.snapshot_field(session.client_profile,trial/'snapshot',domain)
        if not result['passed']:terminate_fault(root,job,'snapshot',result)
        start=field._read_snapshot(trial/'snapshot');profile=field.load_profile(session.client_profile)
        require_local(profile,domain)
        pose=np.asarray(job['pose_rad'] if 'pose_rad' in job else p['pose_rad'])
        build=make_probe if motion_builder is None else motion_builder
        motion=build(job['id'],start,pose,p['joint'],job['direction'],config=p['probe_config'],
            transition_s=p['transition_s'],settle_s=p['settle_s'])
        kp,kd=holding_gains(pose,p['holding'],profile)
        profile=dict(profile,command_hz=p['command_hz'],breakaway_parameters=dict(command_kp=kp.tolist(),command_kd=kd.tolist()))
        plan=trial/'plan';plan.mkdir()
        field.finish_plan(plan,profile,motion,[],dict(**field.snapshot_binding(trial/'snapshot',profile),
            local_probe_pair=p['schema_version']=='casbot_sdk_wrist_probe_pair_v1',
            local_probe_collection=True,job=job,fixture_prior_not_h3_accepted=True,
            replan_recipe=dict(kind='sdk_wrist_probe',name=job['id'],pose_rad=pose.tolist(),joint=p['joint'],direction=job['direction'],
                config=p['probe_config'],transition_s=p['transition_s'],settle_s=p['settle_s'],tones=p.get('tones'),job=job)))
        result=field.execute_field(plan,trial/'capture',domain)
        if not result['passed']:terminate_fault(root,job,'execution',result)



def replan_probe(profile,snapshot,output,recipe,binding):
    start=field._read_snapshot(snapshot);pose=np.asarray(recipe['pose_rad'])
    if recipe['tones'] is None:
        motion=make_probe(recipe['name'],start,pose,recipe['joint'],recipe['direction'],config=recipe['config'],transition_s=recipe['transition_s'],settle_s=recipe['settle_s'])
    else:
        from .hardware_roll_dual import protocol_motion
        motion=protocol_motion(recipe['name'],start,pose,recipe['joint'],recipe['direction'],config=recipe['config'],tones=recipe['tones'],transition_s=recipe['transition_s'],settle_s=recipe['settle_s'])
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    return field.finish_plan(out,profile,motion,[],dict(**binding,replan_recipe=recipe,local_probe_collection=True,job=recipe['job']))

def collect(root,domain):
    root=Path(root).resolve();field.check_seal(root/'protocol');p=read(root/'protocol/experiment.json')
    options=dict(profile=root/'protocol'/p['base_profile']) if 'base_profile' in p else {}
    with SDKMockSession(root/'protocol/mock.yaml',root/'mock',domain,**options) as session:
        collect_jobs(root,p,session,domain,p['jobs'])
    write_json(root/'collection.json',dict(completed=True,records=len(p['jobs']),**FLAGS))


def tone(time_s,values,frequency):
    """Exposure-weighted tone plus quadratic drift; coefficients remain diagnostic."""
    t=np.asarray(time_s);t=t-t.mean();v=np.asarray(values)
    if v.ndim==1:v=v[:,None]
    b=np.column_stack([np.ones(len(t)),t,t*t,np.sin(2*np.pi*frequency*t),np.cos(2*np.pi*frequency*t)])
    w=exposure(t);s=np.sqrt(w);coef=np.linalg.lstsq(b*s[:,None],v*s[:,None],rcond=1e-10)[0]
    residual=v-b@coef
    return dict(rms=np.linalg.norm(coef[-2:],axis=0)/np.sqrt(2),sine=coef[-2],cosine=coef[-1],
                non_tone_residual_rms=np.sqrt(np.average(residual**2,axis=0,weights=w)))


def inspect_observations(obs,elapsed,motion,engine,c,profile,prior,*,signal_frequencies=None):
    """Always report requested raw checks, even when probe preprocessing rejects."""
    j=motion['conditions']['joint'];sign=motion['conditions']['direction'];pol=c['processing']
    windows=[(motion['knots_s'][i],motion['knots_s'][i+1]) for i,l in enumerate(motion['labels']) if l=='probe_measure']
    begin,end=windows[0][0],windows[-1][1]
    full=(elapsed>=begin)&(elapsed<end);inner=(elapsed>=begin+pol['edge_trim_s'])&(elapsed<end-pol['edge_trim_s'])
    if full.sum()<2 or inner.sum()<2:raise ValueError('Missing received non-reversing measurement window')
    held=[k for k in range(14) if k!=j];v=obs['dq'][full];q=obs['q'][full]
    raw=(obs['effort_raw']-np.asarray(profile['effort_zero_nm']))*np.asarray(profile['effort_gain'])
    velocity=obs['dq'][inner,j];direction=np.sign(velocity);decay=np.exp(-(velocity/.005)**2)
    friction=prior['viscous'][j]*velocity+prior['coulomb'][j]*direction*(1-decay)+prior['static_friction'][j]*direction*decay
    gravity=engine.gravity_batch(obs['q'][inner])[:,j]
    dynamic=raw[inner,j]-gravity-friction;time=elapsed[inner]
    if signal_frequencies is None:
        total_tone=tone(time,raw[inner,j],5.);dynamic_tone=tone(time,dynamic,5.)
        neighbors={str(f):float(tone(time,dynamic,f)['rms'][0]) for f in (4.375,5.625)}
        signals=dict(estimated_effort_5hz_rms_nm=float(total_tone['rms'][0]),
            observed_minus_frozen_gravity_friction_5hz_rms_nm=float(dynamic_tone['rms'][0]),
            dynamic_adjacent_tone_rms_nm=neighbors,
            dynamic_5hz_to_largest_adjacent_amplitude_ratio=float(dynamic_tone['rms'][0]/max(max(neighbors.values()),1e-15)),
            dynamic_non5hz_raw_residual_rms_nm=float(dynamic_tone['non_tone_residual_rms'][0]))
    else:
        from .hardware_roll_frequency import phasors
        z=phasors(time,np.column_stack([raw[inner,j],dynamic,velocity]),signal_frequencies,origin_s=float(time.mean()))
        signals=dict(frequency_signals=[dict(frequency_hz=float(f),estimated_effort_rms_nm=float(abs(row[0])/np.sqrt(2)),
            observed_minus_frozen_gravity_friction_rms_nm=float(abs(row[1])/np.sqrt(2)),
            reported_velocity_rms_rad_s=float(abs(row[2])/np.sqrt(2))) for f,row in zip(signal_frequencies,z)],
            signal_projection='simultaneous declared tones plus quadratic drift, exposure weighted')
    values=np.column_stack([obs[k][full] for k in ('q','dq','effort_raw')])
    held_max=np.max(abs(v[:,held]),axis=0);held_worst=held[int(np.argmax(held_max))]
    report=dict(measurement_window_s=[begin,end],retained_window_s=[begin+pol['edge_trim_s'],end-pol['edge_trim_s']],
        raw_publications=int(full.sum()),independent_acquisition_count=None,
        signed_speed_minimum_rad_s=float(np.min(sign*v[:,j])),signed_speed_maximum_rad_s=float(np.max(sign*v[:,j])),
        signed_speed_mean_rad_s=float(np.mean(sign*v[:,j])),
        maximum_held_speed_rad_s=float(np.max(held_max)),worst_held_joint=held_worst,
        held_joint_maximum_speed_rad_s={str(k):float(x) for k,x in zip(held,held_max)},
        maximum_held_position_range_rad=float(np.max(np.ptp(q[:,held],axis=0))),
        maximum_callback_gap_s=float(np.max(np.diff(elapsed[full]))),
        identical_adjacent_public_value_fraction=float(np.mean(np.all(values[1:]==values[:-1],axis=1))),
        **signals,
        dynamic_torque_note='observed Nm minus declared nominal gravity and friction; not pure independently measured inertia torque',
        empirical_quality_gate_applied=legacy_quality(c),
        raw_checks=(dict(signed_motion=bool(np.min(sign*v[:,j])>pol['minimum_signed_velocity_rad_s']),
            other_axes_held=bool(np.max(held_max)<=pol['maximum_held_velocity_rad_s']),
            callback_gap=bool(np.max(np.diff(elapsed[full]))<=pol['maximum_callback_gap_s'])) if legacy_quality(c) else {}),**FLAGS)
    problem=None
    try:
        problem=probe_problem(obs,elapsed,motion,engine,c,profile)
        report.update(probe_preprocessing_passed=True,preprocessing=problem['preprocessing'],
            actual_acceleration_rms_rad_s2=float(np.sqrt(np.mean(problem['ddq'][:,j]**2))))
        try:report['inertia_diagnostic']=inertia_balance(problem)
        except (ValueError,np.linalg.LinAlgError) as error:report['inertia_diagnostic']=dict(available=False,reason=str(error))
        dynamic_projected=problem['target']-problem['projected_gravity_nm']-problem['extra']@np.r_[prior['viscous'],prior['coulomb']]-prior['static_friction'][j]*problem['static_feature']
        if signal_frequencies is None:
            report['projected_dynamic_5hz_rms_nm']=float(tone(problem['time_s'],dynamic_projected,5.)['rms'][0])
        else:
            z=phasors(problem['time_s'],dynamic_projected,signal_frequencies,origin_s=float(problem['time_s'].mean()))
            report['projected_dynamic_frequency_rms_nm']=(abs(z)/np.sqrt(2)).tolist()
    except ValueError as error:
        report.update(probe_preprocessing_passed=False,preprocessing_rejection=str(error))
    traces=dict(time_s=time,reported_speed=obs['dq'][inner,j],effort_nm=raw[inner,j],
        frozen_gravity_nm=gravity,frozen_friction_nm=friction,dynamic_nm=dynamic,
        held_max_abs_velocity=np.max(abs(obs['dq'][inner][:,held]),axis=1))
    return report,problem,traces


def analyse(root,output):
    root=Path(root).resolve();out=Path(output).resolve();out.mkdir(parents=True,exist_ok=False)
    field.check_seal(root/'protocol');p=read(root/'protocol/experiment.json');rows=[];traces=[]
    for job in p['jobs']:
        trial=root/job['id'];plan,captured=field.resolve_field_capture(trial/'plan',trial/'capture')
        meta,commands,origin,obs,elapsed,collision=field.field_observations(plan,captured)
        require_local(meta['config'],read(captured/'request.json')['domain_id'])
        engine,_=model_for(meta['config'])
        model=engine.model;model.gravity_world=np.asarray(meta['config']['gravity_world']);engine=type(engine)(model)
        motion=read(plan/'motion.json')
        report,problem,trace=inspect_observations(obs,elapsed,motion,engine,p['probe_config'],meta['config'],p['fixture_friction_prior'])
        report.update(job=job,received_collision_passed=collision['passed'])
        if problem is not None and 'diagnostic_variants' in p:
            report['diagnostics']=[];baseline=report['inertia_diagnostic']['inertia_kg_m2']
            for variant in p['diagnostic_variants']:
                cfg=deepcopy(p['probe_config']);cfg['processing']['fit_band_hz']=variant['fit_band_hz']
                try:
                    alt=probe_problem(obs,elapsed,motion,engine,cfg,meta['config'],effort_relative_offset_s=variant['effort_offset_s'])
                    estimate=inertia_balance(alt)
                    report['diagnostics'].append(dict(variant=variant,available=True,**estimate,
                        relative_change_from_primary=abs(estimate['inertia_kg_m2']-baseline)/max(abs(baseline),1e-8)))
                except ValueError as error:report['diagnostics'].append(dict(variant=variant,available=False,reason=str(error)))
        rows.append(report);traces.append(trace)
        if problem is not None:
            np.savez_compressed(out/(job['id']+'_problem.npz'),**{k:v for k,v in problem.items() if isinstance(v,np.ndarray)})
        np.savez_compressed(out/(job['id']+'_public_traces.npz'),**trace)
    summary=dict(schema_version='casbot_sdk_wrist_probe_pair_report_v1',records=rows,joint=p['joint'],
        role=p.get('role','local_pair_exploration'),
        original_inertia_relative_threshold=p['original_inertia_relative_threshold'],
        all_motion_and_preprocessing_checks_passed=all(all(r['raw_checks'].values()) and r['probe_preprocessing_passed'] for r in rows),
        pair_is_not_full_r1_r2_acceptance=True,automatic_expansion=False,formal_test_consumed=False,**FLAGS)
    if all(r['probe_preprocessing_passed'] for r in rows):
        a,b=[r['inertia_diagnostic']['inertia_kg_m2'] for r in rows]
        summary['signed_direction_inertia_difference_relative']=abs(a-b)/max(abs(a),1e-6)
        summary['direction_difference_is_diagnostic_not_r2']=True
    write_json(out/'report.json',summary);plot(rows,traces,out,joint_label=p.get('joint_label','右腕 pitch'));field.seal(out)
    return summary


def plot(rows,traces,out,*,joint_label='右腕 pitch'):
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from .presentation import _font
    path=_font(12).path;font_manager.fontManager.addfont(path)
    plt.rcParams['font.family']=font_manager.FontProperties(fname=path).get_name();plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(3,2,figsize=(13,9),layout='constrained')
    for col,(row,t) in enumerate(zip(rows,traces)):
        elapsed=t['time_s']-t['time_s'][0]
        axes[0,col].plot(elapsed,t['reported_speed'],lw=1);axes[0,col].set_ylabel(joint_label+' 速度（rad/s）')
        axes[0,col].set_title('负向' if row['job']['direction']<0 else '正向')
        axes[1,col].plot(elapsed,(t['effort_nm']-t['frozen_gravity_nm'])*1000,lw=.8,label='观测 − nominal 重力')
        axes[1,col].plot(elapsed,t['dynamic_nm']*1000,lw=.8,label='再扣声明摩擦')
        axes[1,col].set_ylabel('估算力矩（mNm）');axes[1,col].legend(fontsize=9)
        axes[2,col].plot(elapsed,t['held_max_abs_velocity'],lw=.8);axes[2,col].axhline(.001,color='r',ls='--')
        axes[2,col].set_ylabel('其它轴最大速度（rad/s）');axes[2,col].set_xlabel('保留窗口内时间（s）')
        for ax in axes[:,col]:ax.grid(alpha=.25)
    fig.suptitle('c00 '+joint_label+' 非换向探针｜本机 SDK Mock，5ms 缓存')
    fig.savefig(out/'pair_public_feedback.png',dpi=160);plt.close(fig)
