"""H4 frozen H3 -> SDK excitation -> physically consistent inertial candidate."""
import csv
from importlib.resources import files
from pathlib import Path
import shutil
import numpy as np
import yaml
from . import hardware_field as field
from .data import write_json
from .dual_model import nominal_dual_arm
from .hardware_friction import read,center,holding_gains,gravity_engine,plateau_pairs
from .staged_motion import periodic_motion,derivative_bounds
from .rigid_identification import fit_rigid,model_from_parameters
from .hardware_periodic import periodic_problem,prediction_metrics
from .vendor_identity import sha256


def settings(path,names):
    c=yaml.safe_load(Path(path).read_text())
    if c.get('schema_version')!='casbot_hardware_inertia_v1':raise ValueError('Explicit H4 schema required')
    if c['joints']=='all':c['joints']=list(names)
    if not c['joints'] or len(set(c['joints']))!=len(c['joints']) or not set(c['joints'])<=set(names):raise ValueError('Unique H4 joint names required')
    for key in ('home_rad','amplitude_rad'):
        value=np.asarray(c[key])
        if value.shape!=(14,) or not np.isfinite(value).all() or (key=='amplitude_rad' and np.any(value<=0)):raise ValueError('Finite 14-axis H4 reference required')
    for key in ('period_s','transition_s','excitation_ramp_s','settle_s','maximum_acceleration_rad_s2','maximum_jerk_rad_s3','maximum_gap_s',
                'moving_speed_rad_s','minimum_velocity_rms_rad_s','minimum_acceleration_rms_rad_s2','maximum_velocity_consistency_rad_s',
                'validation_absolute_rmse_nm','validation_normalized_rmse','torque_floor_nm','gravity_preservation_nm','ridge'):
        if not np.isfinite(c[key]) or c[key]<=0:raise ValueError('Positive H4 setting required: '+key)
    for key in ('periods','discard_periods','harmonics','phase_points'):
        if type(c[key]) is not int or c[key]<1:raise ValueError('Positive integer H4 setting required')
    if c['periods']-c['discard_periods']<2 or c['phase_points']<=2*c['harmonics']+1 or c['harmonics']<6:raise ValueError('Two steady periods and adequate matched bandwidth required')
    roles=[]
    for role in ('train','validation'):
        poses=c['pose_offsets_rad'][role];seeds=c[role+'_seeds']
        if not poses or len(set(poses))!=len(poses) or not np.isfinite(poses).all():raise ValueError('Distinct finite H4 poses required')
        if not seeds or len(set(seeds))!=len(seeds) or any(type(s) is not int or s<0 for s in seeds):raise ValueError('Distinct H4 seeds required')
        roles+=seeds
    control=c['control']
    for key in ('hold_natural_frequency_rad_s','hold_damping_ratio','hold_minimum_kp','hold_minimum_kd'):
        if not np.isfinite(c['holding'][key]) or c['holding'][key]<=0:raise ValueError('Positive explicit H4 holding gain design required')
    roles += [control['tuning_seed'],control['validation_seed']]
    if len(set(roles))!=len(roles) or set(c['pose_offsets_rad']['train'])&set(c['pose_offsets_rad']['validation']):raise ValueError('Whole H4 trajectory roles must be disjoint')
    if type(control['enabled']) is not bool or control['kp_factors']!=[.5,.75,1.,1.25,1.5]:raise ValueError('Five predeclared common PD factors required')
    if not np.isfinite(control['maximum_effort_nm']) or not 0<control['maximum_effort_nm']<=100 or not np.isfinite(control['maximum_speed_rad_s']) or control['maximum_speed_rad_s']<=0:raise ValueError('Bounded H4 control envelope required')
    offsets=np.asarray(c['offset_diagnostic_grid_s'])
    if offsets.ndim!=1 or len(offsets)<2 or not np.isfinite(offsets).all() or np.max(abs(offsets))>.05 or not np.isfinite(c['effort_relative_offset_s']) or abs(c['effort_relative_offset_s'])>.05:raise ValueError('Bounded explicit relative-time diagnostic required')
    if not np.isfinite(c['inter_job_pause_s']) or c['inter_job_pause_s']<0:raise ValueError('Nonnegative H4 pause required')
    return c


def snapshot_prior(source,profile,c,output):
    from . import hardware_batch as batch
    root,p=batch.load_batch(source);history,pending=batch.outcomes(root,p)
    if p.get('workflow')!='h3_friction' or pending:raise ValueError('Complete H3 source required')
    pointer=read(root/'latest.json');report_root=root/pointer['report'];field.check_seal(report_root)
    if sha256(report_root/'manifest.json')!=pointer['manifest_sha256']:raise ValueError('H3 report identity changed')
    report=read(report_root/'report.json')
    if (not report['passed'] or not report['moving_friction']['passed'] or not p['h3']['breakaway']['enabled'] or
        not report['breakaway'].get('available') or not report['breakaway'].get('passed')):
        raise ValueError('Accepted H3 gravity/moving/onset commissioning required')
    names=p['joint_names'];calibration=read(root/'protocol/gravity/calibration.json')
    _,model_source=nominal_dual_arm()
    if p['source_model_sha256']!=model_source['sha256']:raise ValueError('H3 model changed')
    for key in ('effort_gain','effort_zero_nm','gravity_world'):
        if not np.allclose(calibration[key],profile[key],rtol=0,atol=1e-12):raise ValueError('H4 gravity/effort scale changed: '+key)
    if not set(c['joints'])<=set(p['h3']['joints']):raise ValueError('H4 axes lack identified H3 friction')
    b=np.zeros(14);fc=b.copy();fs=b.copy()
    for axis in report['moving_friction']['axes']:
        j=names.index(axis['joint']);b[j]=axis['viscous_nm_s_rad'];fc[j]=axis['coulomb_nm']
    for axis in report['breakaway']['axes']:
        j=names.index(axis['joint']);fs[j]=max(v['estimate_nm'] for v in axis['directions'].values())
    if np.any(fs<fc):raise ValueError('H3 onset is below moving Coulomb friction; inspect earlier stages')
    output=Path(output);output.mkdir();shutil.copytree(root/'protocol/gravity',output/'gravity')
    frozen=dict(parameters=calibration['parameters'],joint_names=names,source_model_sha256=p['source_model_sha256'],
        gravity_world=calibration['gravity_world'],viscous=b.tolist(),coulomb=fc.tolist(),static_friction=fs.tolist(),
        identified_joints=p['h3']['joints'],holding=p['h3']['breakaway'],h3=p['h3'],profile=p['profile'],
        moving_report=report['moving_friction'],onset_report=report['breakaway'],
        source_protocol_manifest_sha256=sha256(root/'protocol/manifest.json'),source_report_manifest_sha256=sha256(report_root/'manifest.json'))
    write_json(output/'prior.json',frozen)
    # Keep observation-only derivatives of accepted H3 captures for a genuine
    # post-inertia recheck. Original CDR/capture bundles are never modified.
    records=[];directory=output/'records';directory.mkdir()
    for job in p['jobs']:
        row=batch.accepted(history[job['id']])
        if row is None:raise ValueError('Incomplete H3 source')
        if job['joint'] not in c['joints']:continue
        attempt=root/row['attempt'];meta=read(attempt/'plan/plan.json');motion=read(attempt/'plan/motion.json')
        obs=field.read_feedback(attempt/'capture/execution/feedback.csv')
        with (attempt/'capture/execution/commands.csv').open() as stream:first=next(csv.DictReader(stream))
        origin=int(first['planned_ns'])-int(first['planned_index'])*meta['command_period_ns']
        elapsed=(obs['receive_ns']-np.int64(origin))*1e-9;name=job['id']
        np.savez_compressed(directory/(name+'.npz'),elapsed=elapsed,**obs)
        records.append(dict(id=name,job=job,motion=motion,profile=meta['config'],source_attempt_manifest_sha256=row['manifest_sha256']))
    write_json(output/'records.json',records);field.seal(output)
    return frozen


def plan_inertia(config_path,profile_path,prior,output,*,domain_id):
    from . import hardware_batch as batch
    from .sdk_connection import connection_remaps
    if type(domain_id) is not int or not 0<=domain_id<=232:raise ValueError('Explicit H4 ROS domain required')
    profile=field.load_profile(profile_path);connection_remaps(profile,domain_id)
    engine,source=nominal_dual_arm();names=[j.name for j in engine.model.joints];c=settings(config_path,names)
    root=field.output_outside(output,prior);root.mkdir(parents=True,exist_ok=False);protocol=root/'protocol';protocol.mkdir()
    snapshot_prior(prior,profile,c,protocol/'prior')
    jobs=[]
    for role in ('train','validation'):
        for pose,offset in enumerate(c['pose_offsets_rad'][role]):
            for seed in c[role+'_seeds']:
                jobs.append(dict(id=f'inertia_{len(jobs):03}',kind='inertia',joint='both_arms',hz=profile['command_hz'],
                    split=role,pose=pose,pose_offset_rad=offset,seed=seed))
    if c['control']['enabled']:
        for factor in c['control']['kp_factors']:
            jobs.append(dict(id=f'tuning_{len(jobs):03}',kind='control_tuning',joint='both_arms',hz=profile['command_hz'],
                split='selection',pose_offset_rad=0.,seed=c['control']['tuning_seed'],kp_factor=factor,feedforward='prior'))
        for model in ('prior','identified'):
            for gains in ('original','tuned'):
                jobs.append(dict(id='control_'+model+'_'+gains,kind='control_validation',joint='both_arms',hz=profile['command_hz'],
                    split='control_validation',pose_offset_rad=0.,seed=c['control']['validation_seed'],feedforward=model,gain_strategy=gains))
    generic=dict(direction=dict(enabled=False,require_review=False,joints=[]),frequency=dict(enabled=False,joints=[]),
        static=dict(enabled=False,quality_retries=0,retry_settle_extra_s=0.),inter_job_pause_s=c['inter_job_pause_s'])
    record=dict(schema_version='casbot_hardware_batch_protocol_v1',workflow='h4_inertia',jobs=jobs,domain_id=domain_id,
        connection_scope=profile.get('connection',{}).get('scope','hardware'),joint_names=names,source_model_sha256=source['sha256'],
        implementation=batch.implementation(),config=generic,h4=c,profile=profile,
        recovery_policy='log_noop_and_terminate_session_no_fault_resume',hardware_calibration_accepted=False,device_writes_during_planning=0)
    write_json(protocol/'protocol.json',record);write_json(protocol/'profile.json',profile)
    (protocol/'scene.yaml').write_text(yaml.safe_dump(profile['scene']));field.seal(protocol)
    batch.event(root,'planned',protocol_sha256=sha256(protocol/'manifest.json'))
    return batch.summarize(root,'planned')


def candidate_engine(root,bundle=None):
    prior=read(Path(root)/'protocol/prior/prior.json');engine=gravity_engine(Path(root)/'protocol/prior/gravity')
    return model_from_parameters(engine,prior['parameters'] if bundle is None else bundle['parameters'],
        prior['viscous'],prior['coulomb'],prior['static_friction'],stribeck_velocity=.005)


def prepare_trial(root,protocol,job,profile_path,snapshot,output):
    from .hardware_control import control_factor
    root=Path(root);c=protocol['h4'];prior=read(root/'protocol/prior/prior.json');profile=field.load_profile(profile_path)
    start=field._read_snapshot(snapshot);pose=center(c,job['pose_offset_rad'])
    amplitude=np.array(c['amplitude_rad']);active=[protocol['joint_names'].index(j) for j in c['joints']]
    amplitude[[j for j in range(14) if j not in active]]=0.
    recipe=dict(dynamic_period_s=c['period_s'],dynamic_periods=c['periods'],pose_amplitude_rad=amplitude,
        settle_s=c['settle_s'],dynamic_transition_s=c['transition_s'],smooth_periodic_envelope=True,dynamic_ramp_s=c['excitation_ramp_s'])
    m=periodic_motion(job['id'],job['kind'],start,pose,recipe,job['seed'],job['split']!='train','both',dict(job,refine_position_bounds=True))
    curve=m.polynomial()
    if np.max(derivative_bounds(curve,2))>c['maximum_acceleration_rad_s2'] or np.max(derivative_bounds(curve,3))>c['maximum_jerk_rad_s3']:raise ValueError('H4 continuous acceleration/jerk envelope exceeded')
    out=field.output_outside(output,root/'protocol',snapshot);out.mkdir(parents=True,exist_ok=False)
    write_json(out/'h4.json',c);write_json(out/'friction.json',prior)
    kp,kd=holding_gains(pose,c['holding']);factor=control_factor(root,job)
    kp*=factor;kd*=np.sqrt(factor)
    extra=dict(command_kp=kp.tolist(),command_kd=kd.tolist())
    if job['kind']=='inertia':
        shutil.copytree(root/'protocol/prior/gravity',out/'gravity')
        extra.update(gravity_feedforward_enabled=True,gravity_model_path=str(out/'gravity/candidate_exploratory.urdf'),gravity_world=profile['gravity_world'])
    else:
        bundle=read(root/'inertia_checkpoint/candidate.json') if job['feedforward']=='identified' else None
        model=candidate_engine(root,bundle)
        period=1000000000//profile['command_hz'];times=np.arange(0,int(np.ceil(m.duration*1e9/period))*period+1,period,dtype=np.int64)
        t=np.minimum(times*1e-9,m.duration)
        ff=model.evaluate_batch(curve(t),curve(t,nu=1),curve(t,nu=2))['total']
        if np.max(abs(ff))>c['control']['maximum_effort_nm']:raise ValueError('H4 feedforward exceeds frozen effort envelope')
        ff_path=out/'feedforward_plan.csv'
        np.savetxt(ff_path,np.column_stack([times,ff]),delimiter=',',header='relative_ns'+''.join(',effort'+str(j) for j in range(14)),comments='',fmt=['%d']+['%.17g']*14)
        extra.update(gravity_feedforward_enabled=False,feedforward_plan_file=str(ff_path),feedforward_limit_nm=c['control']['maximum_effort_nm'])
    profile=dict(profile,gravity_feedforward='none',breakaway_parameters=extra)
    return field.finish_plan(out,profile,m,[],dict(**field.snapshot_binding(snapshot,profile),h4=True,job=job,
        source_prior_manifest_sha256=sha256(root/'protocol/prior/manifest.json'),kp_factor=factor,
        control_reference_model='frozen H3 physical representative' if job.get('feedforward')!='identified' else 'frozen H4 inertial candidate'))


def inspect_trial(plan,capture,output):
    from .hardware_control import control_metrics
    meta,commands,origin,obs,elapsed,collision=field.field_observations(plan,capture)
    plan=Path(plan);c=read(plan/'h4.json');out=Path(output);out.mkdir(parents=True,exist_ok=False)
    if meta['stage']=='inertia':
        prior=read(plan/'friction.json');engine=gravity_engine(plan/'gravity')
        p=periodic_problem(obs,elapsed,read(plan/'motion.json'),engine,c,meta['config'],prior)
        np.savez_compressed(out/'periodic_problem.npz',**{k:v for k,v in p.items() if isinstance(v,np.ndarray)})
        # Store alternative observed torque phase evaluations, not newly fitted
        # mechanics. They expose sensitivity without selecting on validation.
        alternatives=[]
        for shift in c['offset_diagnostic_grid_s']:
            alternative=periodic_problem(obs,elapsed,read(plan/'motion.json'),engine,c,meta['config'],prior,offset=shift)
            alternatives.append(alternative['observed_torque_nm'])
        np.savez_compressed(out/'timing_diagnostics.npz',offsets_s=c['offset_diagnostic_grid_s'],observed_torque_nm=alternatives)
        report=dict(passed=True,stage='inertia',job=meta['job'],preprocessing=p['preprocessing'],sample_time_known=False,
            formal_hardware_calibration_accepted=False)
    else:report=control_metrics(plan,meta,commands,origin,obs,elapsed,c)
    write_json(out/'collision_received_interpolant.json',collision);write_json(out/'report.json',report);field.seal(out);return report


def gravity_constraints(engine,home,seed=9401):
    rng=np.random.default_rng(seed);q=np.asarray(home)+rng.uniform(-.4,.4,(128,14));z=np.zeros_like(q)
    matrix=engine.regressor_batch(q,z,z)['regressor'].reshape(-1,140)
    return matrix,matrix@engine.rigid_parameters()


def prior_recheck(root,candidate,c):
    root=Path(root);prior=read(root/'protocol/prior/prior.json');old=gravity_engine(root/'protocol/prior/gravity')
    matrix,target=gravity_constraints(old,c['home_rad'],9402)
    change=float(np.max(abs(matrix@candidate.rigid_parameters()-target)))
    rows=read(root/'protocol/prior/records.json');friction=[];onset=[]
    for row in rows:
        with np.load(root/'protocol/prior/records'/(row['id']+'.npz'),allow_pickle=False) as d:data={k:d[k] for k in d.files}
        elapsed=data.pop('elapsed');job=row['job']
        if job['kind']=='moving_friction' and job['split']=='validation':
            x,y,_=plateau_pairs(data,elapsed,row['motion'],candidate,prior['h3'],row['profile']);j=prior['joint_names'].index(job['joint'])
            error=x@np.array([prior['viscous'][j],prior['coulomb'][j]])-y
            friction.append(dict(job=row['id'],rmse_nm=float(np.sqrt(np.mean(error**2)))))
        elif job['kind']=='breakaway':
            q=data['q'][::50];onset.append(float(np.max(abs(candidate.gravity_batch(q)-old.gravity_batch(q)))))
    return dict(gravity_prediction_change_nm=change,gravity_check='new deterministic full-configuration Pinocchio queries, not training rows',
        frozen_gravity_validation_rmse_nm=read(root/'protocol/prior/gravity/report.json')['validation_rmse_nm'],
        moving_friction_validation=friction,maximum_onset_gravity_change_nm=max(onset,default=float('inf')),
        moving_and_onset_parameters_frozen=True,passed=bool(change<=c['gravity_preservation_nm'] and friction and onset and
            max(r['rmse_nm'] for r in friction)<=prior['h3']['validation_rmse_nm'] and max(onset)<=c['gravity_preservation_nm']))


def inertia_result(root,protocol,history):
    from . import hardware_batch as batch
    root=Path(root);selected=[j for j in protocol['jobs'] if j['kind']=='inertia']
    if any(not batch.accepted(history[j['id']]) for j in selected):return dict(available=False,passed=False),None
    c=protocol['h4'];axes=[protocol['joint_names'].index(j) for j in c['joints']];prior=read(root/'protocol/prior/prior.json')
    engine=gravity_engine(root/'protocol/prior/gravity');problems={};sources=[];train=[];target=[]
    for job in selected:
        source=batch.accepted(history[job['id']]);folder=root/source['attempt']/'inspection'
        with np.load(folder/'periodic_problem.npz',allow_pickle=False) as d:p={k:d[k] for k in d.files}
        problems[job['id']]=(job,p,folder);sources.append(dict(job=job,attempt=source['attempt'],manifest_sha256=source['manifest_sha256']))
        if job['split']=='train':
            mask=np.zeros_like(p['moving']);mask[:,axes]=p['moving'][:,axes]
            train.append(p['regression'][mask]);target.append(p['target'][mask])
    equality=gravity_constraints(engine,c['home_rad'])
    try:
        fit=fit_rigid(np.vstack(train),np.concatenate(target),np.asarray(prior['parameters']),equalities=equality,
            ridge=c['ridge'],physical_length_scale=.1,lift_residuals=True)
    except (ValueError,RuntimeError) as exc:
        return dict(available=False,passed=False,observations_complete=True,reason='inertia_fit_rejected:'+str(exc),sources=sources,
            formal_hardware_calibration_accepted=False),None
    theta=np.asarray(fit['parameters']);candidate=candidate_engine(root,dict(parameters=theta))
    validation={};phase_sensitivity={}
    for name,(job,p,folder) in problems.items():
        if job['split']!='validation':continue
        validation[name]=prediction_metrics(p,theta,axes,c)
        with np.load(folder/'timing_diagnostics.npz',allow_pickle=False) as d:
            phase_sensitivity[name]=[dict(offset_s=float(offset),**prediction_metrics(dict(p,observed_torque_nm=value),theta,axes,c)) for offset,value in zip(d['offsets_s'],d['observed_torque_nm'])]
    velocity=np.min([np.sqrt(np.mean(p['dq'][:,axes]**2,axis=0)) for job,p,_ in problems.values() if job['split']=='train'],axis=0)
    acceleration=np.min([np.sqrt(np.mean(p['ddq'][:,axes]**2,axis=0)) for job,p,_ in problems.values() if job['split']=='train'],axis=0)
    preserved=prior_recheck(root,candidate,c)
    checks=dict(physical_candidate=True,independent_prediction=bool(validation and all(v['passed'] for v in validation.values())),
        prior_stages_preserved=preserved['passed'],selected_axes_excited=bool(np.all(velocity>=c['minimum_velocity_rms_rad_s']) and np.all(acceleration>=c['minimum_acceleration_rms_rad_s2'])))
    report=dict(available=True,passed=all(checks.values()),checks=checks,fit=fit,validation=validation,prior_stage_acceptance=preserved,
        velocity_rms_rad_s=velocity.tolist(),acceleration_rms_rad_s2=acceleration.tolist(),joint_names=c['joints'],sources=sources,
        timing_sensitivity=phase_sensitivity,timing_sensitivity_used_for_selection=False,configured_effort_offset_s=c['effort_relative_offset_s'],
        sample_time_known=False,sampling_qualification_passed=False,formal_hardware_calibration_accepted=False,
        dedicated_high_bandwidth_wrist_validation_passed=False,individual_body_parameters_unique=False,
        limitations=['Low-frequency operational candidate; weak/null inertial modes retain physical prior dependence',
            'Unknown relative timing, caches and filtering can bias inertia; phase sensitivity is not a calibrated physical delay',
            'Not the original S3/S4/P0 acceptance or a replacement for dedicated wrist probes'])
    bundle=dict(schema_version='casbot_h4_inertia_candidate_v1',parameters=theta.tolist(),source_model_sha256=protocol['source_model_sha256'],
        joint_names=protocol['joint_names'],identified_joint_scope=c['joints'],viscous=prior['viscous'],coulomb=prior['coulomb'],
        static_friction=prior['static_friction'],gravity_world=prior['gravity_world'],source_prior_manifest_sha256=sha256(root/'protocol/prior/manifest.json'),
        individual_body_parameters_unique=False,formal_hardware_calibration_accepted=False,auto_deploy=False,fit_band_hz=[0,c['harmonics']/c['period_s']])
    return report,bundle


def freeze_inertia(root,protocol,history):
    from . import hardware_batch as batch
    root=Path(root);folder=root/'inertia_checkpoint'
    if folder.exists():
        field.check_seal(folder);events=[e for e in batch.events(root) if e['event']=='inertia_frozen']
        if len(events)!=1 or events[0]['manifest_sha256']!=sha256(folder/'manifest.json'):raise ValueError('H4 frozen inertia changed')
        return read(folder/'report.json')['passed']
    report,bundle=inertia_result(root,protocol,history)
    if not report['passed']:return False
    folder.mkdir();write_inertia_output(root,folder,report,bundle);field.seal(folder)
    batch.event(root,'inertia_frozen',manifest_sha256=sha256(folder/'manifest.json'));return True


def write_inertia_output(root,folder,report,bundle):
    from .dynamic_export import write_inertial_urdf,check_reload
    folder=Path(folder);write_json(folder/'report.json',report)
    if bundle is None:return
    write_json(folder/'candidate.json',bundle);engine=candidate_engine(root,bundle)
    source=files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')
    write_inertial_urdf(source,engine,folder/'candidate_exploratory.urdf')
    reload_report=check_reload(folder/'candidate_exploratory.urdf',engine,source)
    if not reload_report['passed']:raise ValueError('H4 URDF reload failed')
    write_json(folder/'urdf_reload.json',reload_report)
    with (folder/'parameters.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        writer=csv.writer(stream);writer.writerow(['parameter','representative_SI','status'])
        writer.writerows([name,value,'observable combinations; nullspace is prior-dependent'] for name,value in zip(engine.rigid_parameter_names(),bundle['parameters']))


def summarize_h4(root,protocol,history,destination,report):
    from .hardware_control import control_result,plot_results
    root=Path(root);checkpoint=root/'inertia_checkpoint';folder=Path(destination)/'inertia'
    if checkpoint.exists():
        freeze_inertia(root,protocol,history);shutil.copytree(checkpoint,folder)
        inertial=read(folder/'report.json')
    else:
        inertial,bundle=inertia_result(root,protocol,history);folder.mkdir();write_inertia_output(root,folder,inertial,bundle)
    control=control_result(root,protocol,history);write_json(Path(destination)/'control_comparison.json',control)
    report.update(workflow='h4_inertia',inertia=inertial,control=control,gravity=dict(available=True,frozen_input=True),
        formal_hardware_calibration_accepted=False,observation_protocol='SDK callback-periodic exploratory candidate')
    report['passed']=bool(report['passed'] and inertial['passed'] and (not protocol['h4']['control']['enabled'] or control['passed']))
    plot_results(destination,inertial,control)
