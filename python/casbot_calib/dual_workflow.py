"""Fixed-waist 14-axis mock workflow. No SDK or hardware actuation path."""
from copy import deepcopy
import csv,hashlib,json
from pathlib import Path
from time import perf_counter
import numpy as np
from . import _core
from .data import write_json
from .dual_model import nominal_dual_arm,physical_model
from .dual_excitation import periodic_reference,gravity_reference,direction_reference
from .dual_preflight import mock_encoder_channels,mapping_adapter,check_direction,pulse_commands,identify_driver_timing,build_storyboard,preflight_evidence,verify_preflight
from .dual_identification import load_observations,valid_observations,periodic_observations,feature_matrix,fit_parameters,residual_metrics


def _array(value,n=14):return np.broadcast_to(np.asarray(value,dtype=float),(n,)).copy()


def select_excitation(nominal,config):
    motion=config['motion'];results=[];phase=np.arange(256)*motion['period_s']/256
    for seed in motion['candidate_seeds']:
        q,v,a,jerk=periodic_reference(phase,motion['center'],motion['period_s'],motion['amplitude'],seed)
        features,_=feature_matrix(nominal,q,v,a,config['identification']['friction_velocity'])
        matrix=features.reshape(-1,42);norm=np.linalg.norm(matrix,axis=0)
        singular=np.linalg.svd(matrix/np.maximum(norm,1e-12),compute_uv=False)
        rank=int(np.count_nonzero(singular>singular[0]*1e-7));envelope=motion['synthetic_envelope']
        peaks=dict(speed_rad_s=float(np.max(np.abs(v))),acceleration_rad_s2=float(np.max(np.abs(a))),jerk_rad_s3=float(np.max(np.abs(jerk))))
        admissible=all(peaks[k]<=envelope[k] for k in peaks)
        results.append(dict(seed=seed,rank=rank,log_information=float(np.sum(np.log(np.maximum(singular[:rank],1e-12)))),
            minimum_singular_value=float(singular[-1]),admissible=admissible,peaks=peaks,
            planned_range_deg=np.degrees(np.ptp(q,axis=0)).tolist()))
    valid=[r for r in results if r['admissible']]
    if not valid:raise ValueError('No excitation candidate meets the declared synthetic envelope')
    best=max(valid,key=lambda row:(row['rank'],row['log_information']))
    return dict(selected_seed=best['seed'],candidates=results,method='finite multisine phase candidate search in restricted scaled regressor space',
        collision_checked=False,hardware_executable=False,limits_source='synthetic experiment envelope; device limits intentionally deferred')


def _save_run(folder,obs,truth,times,commands,references,names,conditions):
    folder.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(folder/'observations.npz',**obs)
    np.savez_compressed(folder/'evaluation_truth.npz',**truth)
    np.savez_compressed(folder/'commands.npz',send_ns=times,commands=commands,q_ref=references[0],dq_ref=references[1],ddq_ref=references[2],jerk_ref=references[3])
    write_json(folder/'reference_summary.json',dict(q_peak_rad=np.max(np.abs(references[0]),axis=0),
        speed_peak_rad_s=np.max(np.abs(references[1]),axis=0),acceleration_peak_rad_s2=np.max(np.abs(references[2]),axis=0),
        jerk_peak_rad_s3=np.max(np.abs(references[3]),axis=0),torque_feedforward_peak_nm=np.max(np.abs(commands[:,:,2]),axis=0),
        source='analytic reference and nominal feedforward; not measured acceleration or force'))
    write_json(folder/'observation_schema.json',dict(schema_version=2,joint_names=names,
        units=dict(q='rad',dq='rad/s',current='A',time='int64 ns'),
        clock='shared synthetic acquisition clock; current and joint samples synchronous; receive time separate',
        torque_source='current times declared output-side gain, not independent torque sensor'))
    write_json(folder/'metadata.json',dict(schema_version=2,conditions=conditions,
        raw_sha256=hashlib.sha256((folder/'observations.npz').read_bytes()).hexdigest(),
        commands_sha256=hashlib.sha256((folder/'commands.npz').read_bytes()).hexdigest(),
        integration_backend=_core.SimulationRunner.integration_backend(),closed_loop=True))


def _collect(folder,name,plant,feedforward,config,references,duration,seed):
    motion=config['motion']
    times=np.arange(0,round(duration*1e9),motion['control_period_ns'],dtype=np.int64)
    q,v,a,jerk=references(times*1e-9)
    ff=feedforward.evaluate_batch(q,v,a)['total']
    commands=np.stack([q,v,ff,np.broadcast_to(_array(motion['kp']),q.shape),np.broadcast_to(_array(motion['kd']),q.shape)],axis=2)
    return _collect_commands(folder,name,plant,config,times,commands,(q,v,a,jerk),duration,seed)


def _collect_commands(folder,name,plant,config,times,commands,references,duration,seed):
    drive=config['drive_assumptions'];motion=config['motion'];n=14
    native=_core.ArmSimulationConfig();native.gain=_array(drive['gain_nm_a'])
    for key in ('integration_ns','sample_period_ns','command_delay_ns','feedback_delay_ns','current_tau','torque_limit','position_noise','current_noise'):
        setattr(native,key,drive[key])
    native.seed=seed
    q=references[0]
    start=perf_counter();obs,truth=_core.ArmSimulationRunner(plant,native).run(times,commands,round(duration*1e9),q[0],np.zeros(n))
    elapsed=perf_counter()-start
    names=[joint.name for joint in plant.model.joints]
    _save_run(folder/name,obs,truth,times,commands,references,names,
        dict(control_period_ns=motion['control_period_ns'],sample_period_ns=native.sample_period_ns,
             command_delay_ns=native.command_delay_ns,feedback_delay_ns=native.feedback_delay_ns,
             simulation_wall_s=elapsed,duration_s=duration,synthetic_only=True))
    return obs


def _periodic_problem(nominal,obs,session,period=None):
    ident=session['identification'];motion=session['motion'];period=period or motion['period_s']
    data=periodic_observations(obs,period,motion['discard_periods'],ident['harmonics'],ident['phase_points'])
    features,raw=feature_matrix(nominal,data['q'],data['dq'],data['ddq'],ident['friction_velocity'],ident['harmonics'])
    return features,data['current']*_array(session['drive_assumptions']['gain_nm_a']),raw,data


def identify_dual_session(folder,output):
    """Reads only session, nominal model and observations. Evaluation truth is optional."""
    folder=Path(folder);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    session=json.loads((folder/'session.json').read_text(encoding='utf-8'));nominal,source=nominal_dual_arm()
    if session['source_sha256']!=source['sha256'] or session['schema_version']!=1:raise ValueError('Session/model mismatch')
    if session['drive_assumptions']['gain_status'] not in ('synthetic_declared','independently_validated'):
        raise ValueError('An output torque-scale anchor is required for absolute inertial identification')
    verify_preflight(folder,session)
    names=[j.name for j in nominal.model.joints]
    if names!=session['joint_names']:raise ValueError('Session joint order mismatch')
    n=14;initial=np.r_[np.ones(n),np.full(n,.01),np.full(n,.02)]
    load=lambda name:load_observations(folder/name,names)
    gravity=valid_observations(load('gravity_train'))
    t=gravity['sample_ns']*1e-9
    hold=session['motion']['gravity_hold_s']
    mask=(t%hold>hold-1.)&(np.max(np.abs(gravity['dq']),axis=1)<.002)
    if np.count_nonzero(mask)<40:raise ValueError('Insufficient settled gravity holds')
    q=gravity['q'][mask][::10];zero=np.zeros_like(q)
    features,_=feature_matrix(nominal,q,zero,zero,session['identification']['friction_velocity'])
    target=gravity['current'][mask][::10]*_array(session['drive_assumptions']['gain_nm_a'])
    gravity_fit=fit_parameters(features,target,initial,active=np.arange(n))
    friction_features,friction_torque,_,_=_periodic_problem(nominal,load('friction_train'),session,period=8.)
    friction_fit=fit_parameters(friction_features,friction_torque,gravity_fit['parameters'],active=np.arange(n,3*n))
    features,torque,raw,actual_motion=_periodic_problem(nominal,load('dynamic_train'),session)
    speed_rms=np.sqrt(np.mean(actual_motion['dq']**2,axis=0));acceleration_rms=np.sqrt(np.mean(actual_motion['ddq']**2,axis=0))
    minimum_speed=session['identification'].get('minimum_dynamic_velocity_rms_rad_s',.01)
    minimum_acceleration=session['identification'].get('minimum_dynamic_acceleration_rms_rad_s2',.02)
    dynamic_fit=fit_parameters(features,torque,friction_fit['parameters'])
    parameters=np.array(dynamic_fit['parameters']);candidate=physical_model(nominal,parameters[:n],parameters[n:2*n],parameters[2*n:],session['identification']['friction_velocity'])
    raw_matrix=raw.reshape(-1,140);scales=np.linalg.norm(raw_matrix,axis=0)
    singular=np.linalg.svd(raw_matrix/np.maximum(scales,1e-12),compute_uv=False)
    raw_rank=int(np.count_nonzero(singular>singular[0]*1e-7))
    validation={};floor=session['identification']['torque_floor_nm']
    for name in ('dynamic_validation','left_only_validation','right_only_validation'):
        x,y,_,_=_periodic_problem(nominal,load(name),session)
        validation[name]=residual_metrics(x@parameters,y,floor)
        validation[name]['nominal']=residual_metrics(x@np.r_[np.ones(n),np.zeros(2*n)],y,floor)
    checks=dict(physical_candidate_constructed=True,restricted_subspace_identifiable=dynamic_fit['rank']==dynamic_fit['columns'],
        all_axes_dynamically_excited=bool(np.all(speed_rms>=minimum_speed) and np.all(acceleration_rms>=minimum_acceleration)),
        solvers_converged=all(r['success'] for r in (gravity_fit,friction_fit,dynamic_fit)),
        independent_prediction=all(v['maximum_normalized_rmse']<=session['identification']['maximum_validation_nrmse'] for v in validation.values()))
    bundle=dict(schema_version=1,model_version='tree_rigid_v1',robot_model_id='casbot_20251015_nominal_fixed_waist_dual_mock',
        joint_names=names,source_sha256=source['sha256'],fixture=session['fixture'],geometry_status='frozen nominal, not kinematically calibrated',
        gain_source=session['drive_assumptions']['gain_status'],gain_nm_a=session['drive_assumptions']['gain_nm_a'],
        parameterization=dynamic_fit['parameterization'],body_scales=parameters[:n].tolist(),viscous=parameters[n:2*n].tolist(),coulomb=parameters[2*n:].tolist(),
        smoothing_velocity=session['identification']['friction_velocity'],validation=validation,synthetic_only=True,hardware_executable=False)
    report=dict(schema_version=1,passed=all(checks.values()),checks=checks,stages=dict(gravity=gravity_fit,friction=friction_fit,dynamic=dynamic_fit),
        full_rigid_regressor=dict(columns=140,rank=raw_rank,singular_values=singular.tolist(),column_scales=scales.tolist(),
            status='diagnostic only; candidate uses restricted positive body scales, not full 140-parameter recovery'),
        validation=validation,source=source,fixture=session['fixture'],synthetic_only=True,hardware_executable=False,
        excitation_quality=dict(velocity_rms_rad_s=speed_rms.tolist(),acceleration_rms_rad_s2=acceleration_rms.tolist(),
            minimum_velocity_rms_rad_s=minimum_speed,minimum_acceleration_rms_rad_s2=minimum_acceleration),
        limitations=['No SDK or hardware limits used','Fixture transform unmeasured, nominal fixed base retained',
                    'Fixed COM/inertia shape; only body scales and viscous/Coulomb friction fitted','No kinematic calibration','No collision clearance or clamp reaction certification'])
    write_json(output/'candidate.json',bundle);write_json(output/'report.json',report)
    with (output/'parameters.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['joint','body_scale','viscous_Nm_s_rad','coulomb_Nm'])
        writer.writerows(zip(names,parameters[:n],parameters[n:2*n],parameters[2*n:],strict=True))
    _plot_identification(output,names,parameters,validation)
    (output/'report.md').write_text('# 双臂动力学模拟标定\n\n'+
        'nominal固定腰部、左右各七轴；夹具按图示腰部抱紧，精确变换未测量。全部为Mock结果。\n\n'+
        f"结果：{'通过' if report['passed'] else '失败'}；完整刚体回归探针秩 {raw_rank}/140；本轮候选限于正质量/惯量比例与摩擦。\n\n"+
        '\n'.join(f"- {name}：逐轴最大归一化力矩RMSE {value['maximum_normalized_rmse']:.3%}" for name,value in validation.items())+
        '\n\n[完整报告](report.json) · [候选参数](candidate.json) · [参数表](parameters.csv)\n\n不包含运动学标定、完整惯性张量恢复或实机下发。\n',encoding='utf-8')
    return report,candidate


def _plot_identification(folder,names,parameters,validation):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure=Figure(figsize=(12,6));FigureCanvasAgg(figure);axes=figure.subplots(2,1)
    axes[0].plot(parameters[:14],'o-',label='physical body scale');axes[0].legend();axes[0].grid(alpha=.2)
    for name,value in validation.items():axes[1].plot(np.array(value['normalized_rmse'])*100,'o-',label=name)
    axes[1].set(ylabel='Torque NRMSE / %',xlabel='L1..L7, R1..R7');axes[1].legend();axes[1].grid(alpha=.2)
    figure.tight_layout();figure.savefig(folder/'identification.png',dpi=140)


def run_dual_workflow(folder,config):
    config=deepcopy(config)
    if config.get('schema_version')!=1 or config.get('mode')!='mock_only':raise ValueError('Only versioned mock-only workflow is supported')
    if 'preflight' not in config:raise ValueError('Use the current dual-arm configuration with a preflight section')
    if config['motion']['periods']-config['motion']['discard_periods']<2:raise ValueError('Need at least two retained periods')
    if config['identification']['parameterization']!='body_scales_fixed_com_shape_plus_viscous_coulomb':raise ValueError('Unsupported parameterization')
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False);nominal,source=nominal_dual_arm()
    mock=config.pop('mock');session={**config,'joint_names':[j.name for j in nominal.model.joints], 'source_sha256':source['sha256']}
    write_json(folder/'session.json',session);write_json(folder/'source.json',source)
    evaluation=folder/'evaluation';evaluation.mkdir();write_json(evaluation/'mock_parameters.json',mock)
    plant=physical_model(nominal,mock['body_scales'],mock['viscous'],mock['coulomb'],config['identification']['friction_velocity'])
    pre=config['preflight'];motion=config['motion'];center=motion['center']
    print('双臂：逐轴正反向 / 编码器零位映射检查',flush=True)
    jog_duration=pre['initial_hold_s']+14*pre['direction_slot_s']
    direction_ref=lambda t:direction_reference(t,center,pre['direction_amplitude_rad'],pre['direction_slot_s'],pre['initial_hold_s'])
    jog=_collect(folder,'direction_jog',plant,nominal,config,direction_ref,jog_duration,mock['seed'])
    raw=mock_encoder_channels(jog);np.savez_compressed(folder/'direction_jog/raw_telemetry.npz',**raw)
    adapter=mapping_adapter(session['joint_names'],config)
    write_json(folder/'encoder_mapping.json',dict(robot_model_id=adapter.robot_model_id,joints=[m.__dict__ for m in adapter.mappings],synthetic_only=True))
    direction=check_direction(jog,raw,direction_ref(jog['sample_ns']*1e-9)[0],session['joint_names'],config)
    write_json(folder/'direction_check.json',direction)
    if not direction['passed']:
        failure=dict(passed=False,checks=dict(direction_mapping=False),failed_stage='direction_and_zero',synthetic_only=True)
        write_json(folder/'workflow.json',failure);return failure
    print('双臂：驱动电流响应 / 指令延迟 / 1、2、4 ms周期验证',flush=True)
    for index,(name,period_ns) in enumerate([('driver_timing_train',1_000_000),('driver_timing_validation_2ms',2_000_000),('driver_timing_validation_4ms',4_000_000)]):
        timing_config=deepcopy(config);timing_config['motion']['control_period_ns']=period_ns
        times=np.arange(0,round(pre['timing_duration_s']*1e9),period_ns,dtype=np.int64)
        commands,reference=pulse_commands(times,nominal,center,validation=index>0)
        _collect_commands(folder,name,plant,timing_config,times,commands,reference,pre['timing_duration_s'],mock['seed']+200+index)
    timing=identify_driver_timing(folder,session)
    write_json(folder/'preflight.json',preflight_evidence(folder,session,direction,timing))
    if not timing['passed']:
        failure=dict(passed=False,checks=dict(driver_timing=False),failed_stage='driver_timing',synthetic_only=True)
        write_json(folder/'workflow.json',failure);return failure
    excitation=select_excitation(nominal,config);write_json(folder/'excitation_design.json',excitation)
    motion=config['motion'];center=motion['center'];seed=excitation['selected_seed'];period=motion['period_s'];duration=period*motion['periods']
    print('双臂：重力保持采集',flush=True)
    hold=motion['gravity_hold_s'];poses=motion['gravity_poses']
    _collect(folder,'gravity_train',plant,nominal,config,lambda t:gravity_reference(t,center,hold,poses),hold*poses,mock['seed'])
    cases=[('friction_train',8.,False,'both',seed+10),('dynamic_train',period,False,'both',seed),
        ('dynamic_validation',period,True,'both',seed+100),('left_only_validation',period,True,'left',seed+101),
        ('right_only_validation',period,True,'right',seed+102)]
    references={}
    for index,(name,p,validation,active,s) in enumerate(cases):
        print('双臂：'+name,flush=True)
        reference=lambda t,p=p,validation=validation,active=active,s=s:periodic_reference(t,center,p,motion['amplitude'],s,validation,active)
        _collect(folder,name,plant,nominal,config,reference,p*motion['periods'],mock['seed']+index+1)
        references[name]=reference
    print('双臂：观测辨识与独立预测',flush=True)
    report,candidate=identify_dual_session(folder,folder/'identification')
    # Same validation trajectory, plant, initial state, PD and seed; only feedforward changes.
    obs=_collect(folder,'identified_control_validation',plant,candidate,config,references['dynamic_validation'],duration,mock['seed']+3)
    old=load_observations(folder/'dynamic_validation');desired=references['dynamic_validation'](obs['sample_ns']*1e-9)[0]
    old_rmse=np.sqrt(np.mean((old['q']-desired)**2,axis=0));new_rmse=np.sqrt(np.mean((obs['q']-desired)**2,axis=0))
    control=dict(nominal_rmse_rad=old_rmse.tolist(),identified_rmse_rad=new_rmse.tolist(),
        aggregate_nominal_rmse_rad=float(np.sqrt(np.mean(old_rmse**2))),aggregate_identified_rmse_rad=float(np.sqrt(np.mean(new_rmse**2))),
        convention='reference at common acquisition time, fixed PD; command delay included equally; no gain tuning',
        independent=True)
    write_json(folder/'control_comparison.json',control)
    build_storyboard(folder,session)
    dynamic_obs=load_observations(folder/'dynamic_train');steady=dynamic_obs['sample_ns']>=round(period*motion['discard_periods']*1e9)
    write_json(folder/'motion_coverage.json',dict(observed_range_deg=np.degrees(np.ptp(dynamic_obs['q'][steady],axis=0)),
        minimum_range_deg=float(np.degrees(np.ptp(dynamic_obs['q'][steady],axis=0)).min()),angles_amplified=False,
        source='recorded actual steady-state joint positions, not target amplitudes'))
    write_json(folder/'workflow.json',dict(passed=report['passed'] and direction['passed'] and timing['passed'],synthetic_only=True,hardware_executable=False,
        stages=['direction_jog','driver_timing_train','gravity_train']+[n for n,*_ in cases],preflight_passed=True,
        control=control,source_sha256=source['sha256'],integration_backend=_core.SimulationRunner.integration_backend(),limitations=report['limitations']))
    (folder/'README.md').write_text('# 双臂动力学流程\n\n[方向与软件零位](direction_check.json) · [驱动时序](driver_timing.json) · [辨识与独立验证](identification/report.md) · [激励设计](excitation_design.json) · [实际运动跨度](motion_coverage.json) · [固定PD前馈对照](control_comparison.json)\n\n'+
        '双臂保持整机名义装配，腰部按图示位置刚性固定；精确夹具变换未测量。原始观测与evaluation真值分开；可删除真值重新辨识。\n\n'+
        '14轴依次正转、反转、回位，检查独立合成编码器的软件方向/零位映射；机械零位与实机方向仍待独立确认。驱动采用纯力矩脉冲辨识电流响应/指令延迟，再冻结参数验证2/4ms命令周期。\n\n'+
        '本轮使用合成设备参数和实验包络，未连接实机；只辨识固定COM/形状下的惯性比例及摩擦，未做运动学。\n',encoding='utf-8')
    return report
