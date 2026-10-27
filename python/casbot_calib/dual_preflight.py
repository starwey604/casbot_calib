"""Observation-based 14-axis mapping and driver timing checks, before mechanics."""
import json
import hashlib
from pathlib import Path
import numpy as np
from .data import write_json
from .telemetry import JointMapping,TelemetryAdapter
from .identification import identify_current_response,current_response_residual,_current_response_problem
from .dual_identification import load_observations


def session_signature(session):
    return hashlib.sha256(json.dumps(dict(preflight=session['preflight'],drive=session['drive_assumptions']),sort_keys=True).encode()).hexdigest()


def preflight_evidence(folder,session,direction,timing):
    folder=Path(folder);paths=['direction_check.json','driver_timing.json','encoder_mapping.json',
        'direction_jog/observations.npz','direction_jog/commands.npz','direction_jog/raw_telemetry.npz']
    for name in ('driver_timing_train','driver_timing_validation_2ms','driver_timing_validation_4ms'):
        paths.extend([name+'/observations.npz',name+'/commands.npz',name+'/observation_schema.json',name+'/metadata.json',name+'/current_prediction.npz'])
    return dict(passed=direction['passed'] and timing['passed'],direction=direction,timing=timing,
        session_signature=session_signature(session),source_sha256={name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in paths})


def verify_preflight(folder,session):
    if 'preflight' not in session:return  # compatibility with earlier observation recordings
    folder=Path(folder);report=json.loads((folder/'preflight.json').read_text(encoding='utf-8'))
    if not report['passed'] or report['session_signature']!=session_signature(session):raise ValueError('Preflight failed or drive/mapping declarations changed')
    for name,digest in report['source_sha256'].items():
        if hashlib.sha256((folder/name).read_bytes()).hexdigest()!=digest:raise ValueError('Preflight evidence changed: '+name)


def mock_encoder_channels(obs):
    """Producer conventions are independent of the mapping being checked."""
    n=obs['q'].shape[1];direction=np.where(np.arange(n)%2==0,1.,-1.)
    zero=65536.+512.*np.arange(n);counts_per_rad=131072./(2*np.pi)
    return dict(raw_position=zero+direction*counts_per_rad*obs['q'],
        raw_velocity=direction*counts_per_rad*obs['dq'],raw_current=obs['current'].copy())


def mapping_adapter(names,config):
    pre=config['preflight'];counts=pre['encoder_counts_per_revolution']
    return TelemetryAdapter('casbot_20251015_dual_mock',[
        JointMapping(name,name,2*np.pi/counts,int(pre['encoder_directions'][j]),float(pre['encoder_zero_ticks'][j]))
        for j,name in enumerate(names)])


def check_direction(obs,raw,reference,names,config):
    adapter=mapping_adapter(names,config);pre=config['preflight'];t=obs['sample_ns']*1e-9;rows=[]
    initial=pre['initial_hold_s'];slot=pre['direction_slot_s']
    for j,mapping in enumerate(adapter.mappings):
        mask=(t>=initial+j*slot)&(t<initial+(j+1)*slot)
        if np.count_nonzero(mask)<20:raise ValueError('Missing direction jog samples')
        decoded=mapping.position(raw['raw_position'][:,j]);error=float(np.max(np.abs(decoded-obs['q'][:,j])))
        sent=reference[mask,j];encoder=raw['raw_position'][mask,j]
        covariance=float((sent-sent.mean())@(encoder-encoder.mean()))
        inferred=1 if covariance>0 else -1
        span=float(np.ptp(obs['q'][mask,j]));end=np.flatnonzero(mask)[-max(1,int(.3/(config['drive_assumptions']['sample_period_ns']*1e-9))):]
        rows.append(dict(joint=mapping.name,declared_direction=mapping.direction,inferred_direction=inferred,
            zero_raw=mapping.zero_raw,software_zero_rad=float(mapping.position(mapping.zero_raw)),
            maximum_mapping_error_rad=error,observed_range_rad=span,observed_min_rad=float(obs['q'][mask,j].min()),
            observed_max_rad=float(obs['q'][mask,j].max()),return_reference_error_rad=float(np.mean(decoded[end])-config['motion']['center'][j]),
            passed=bool(error<1e-10 and inferred==mapping.direction and span>=pre['minimum_jog_range_rad'])))
    return dict(passed=all(row['passed'] for row in rows),axes=rows,synthetic_only=True,
        physical_direction='unconfirmed_without_independent_physical_observation',
        physical_zero='unconfirmed; encoder declarations and return motion are checked, no geometric zero calibration',
        producer='independent synthetic floating encoder ticks; not ENCOS encoder specifications')


def pulse_commands(times_ns,nominal,center,validation=False):
    """Torque-only known requests isolate current response from PD/mechanical feedback."""
    n=len(center);q=np.broadcast_to(center,(len(times_ns),n)).copy();zero=np.zeros_like(q)
    baseline=nominal.gravity(np.array(center));amplitude=np.tile([.3,.3,.2,.2,.06,.04,.03],2)
    if validation:
        pattern=np.array([1.,-1.,1.,1.,-1.,-1.]);width_ns=17_300_000;amplitude*=.8
    else:pattern=np.array([1.,-1.,-1.,1.]);width_ns=10_000_000
    signal=pattern[(times_ns//width_ns)%len(pattern)]
    torque=baseline+signal[:,None]*amplitude
    commands=np.stack([q,zero,torque,zero,zero],axis=2)
    return commands,(q,zero,zero,zero)


def _single_axis(obs,commands,j):
    n=len(obs['sample_ns']);single={key:obs[key] for key in ('sample_ns','receive_ns','sequence')}
    single.update(current_sample_ns=obs['sample_ns'],current=obs['current'][:,j],status=obs['status'][:,j])
    values=np.zeros((len(commands),6));values[:,3]=commands[:,j,2]
    if np.any(commands[:,j,3:]!=0):raise ValueError('Timing identification must use torque-only requests')
    if np.any(single['status']!=0):raise ValueError('Driver timing requires valid unsaturated acquisition samples')
    return single,values


def identify_driver_timing(folder,session):
    folder=Path(folder);names=session['joint_names'];pre=session['preflight']
    gain=np.broadcast_to(np.asarray(session['drive_assumptions']['gain_nm_a']),(14,))
    cases=['driver_timing_train','driver_timing_validation_2ms','driver_timing_validation_4ms'];records={}
    for name in cases:
        obs=load_observations(folder/name,names)
        metadata=json.loads((folder/name/'metadata.json').read_text(encoding='utf-8'))
        if hashlib.sha256((folder/name/'commands.npz').read_bytes()).hexdigest()!=metadata['commands_sha256']:
            raise ValueError('Driver command digest mismatch')
        with np.load(folder/name/'commands.npz',allow_pickle=False) as data:times,commands=data['send_ns'].copy(),data['commands'].copy()
        expected={'driver_timing_train':1_000_000,'driver_timing_validation_2ms':2_000_000,'driver_timing_validation_4ms':4_000_000}[name]
        if not np.all(np.diff(times)==expected):raise ValueError('Unexpected driver experiment control period')
        records[name]=(obs,times,commands)
    training=records[cases[0]];rows=[];prediction={name:np.empty_like(data[0]['current']) for name,data in records.items()}
    targets={}
    for name,(obs,times,commands) in records.items():
        index=np.searchsorted(times,obs['sample_ns'],side='right')-1
        targets[name]=commands[np.maximum(index,0),:,2]/gain
        targets[name][index<0]=0.
    for j,joint in enumerate(names):
        single,values=_single_axis(training[0],training[2],j)
        fit=identify_current_response(single,training[1],values,float(gain[j]),
            tuple(pre['delay_bounds_s']),tuple(pre['current_tau_bounds_s']))
        fit['gain_status']=session['drive_assumptions']['gain_status']
        validation={}
        for name,(obs,times,commands) in records.items():
            channel,values=_single_axis(obs,commands,j)
            _,evaluate=_current_response_problem(channel,times,values,float(gain[j]))
            prediction[name][:,j]=evaluate(fit['current_tau_s'],fit['command_delay_s'])
            if name!=cases[0]:validation[name]=current_response_residual(channel,times,values,float(gain[j]),fit['current_tau_s'],fit['command_delay_s'])
        rows.append(dict(joint=joint,fit=fit,validation=validation,
            passed=bool(fit['success'] and fit['rmse_a']<=pre['current_validation_rmse_a'] and
                all(value['rmse_a']<=pre['current_validation_rmse_a'] for value in validation.values()))))
    for name,(obs,times,commands) in records.items():
        np.savez_compressed(folder/name/'current_prediction.npz',sample_ns=obs['sample_ns'],predicted_current=prediction[name],sent_target_current=targets[name])
    feedback=training[0]['receive_ns']-training[0]['sample_ns']
    report=dict(passed=all(row['passed'] for row in rows),axes=rows,
        feedback_transport_delay_ms=float(np.median(feedback)*1e-6),acquisition_time_known=True,
        training_control_period_ms=1.,validation_control_periods_ms=[2.,4.],feedback_period_ms=session['drive_assumptions']['sample_period_ns']*1e-6,
        parameters_frozen_for_validation=True,method='shared exact ZOH plus first-order current response, known acquisition timestamps',
        current_response_is_not_mechanical_delay=True,synthetic_only=True,
        scale_source=session['drive_assumptions']['gain_status'])
    write_json(folder/'driver_timing.json',report)
    return report


def build_storyboard(folder,session):
    """Actual recording intervals and playback rates, never enlarge rendered angles."""
    folder=Path(folder);pre=session['preflight'];motion=session['motion'];presentation=session.get('presentation',{})
    seconds=presentation.get('stage_seconds',6.);jog_seconds=presentation.get('direction_seconds_per_joint',2.)
    stages=[]
    def stage(key,title,segments):stages.append(dict(key=key,title=title,segments=segments))
    def segment(run,start,end,duration,joint=3):return dict(run=run,start_s=float(start),end_s=float(end),seconds=float(duration),joint=int(joint))
    stage('direction','方向 / 编码器零位 / 逐轴正反转',[
        segment('direction_jog',pre['initial_hold_s']+j*pre['direction_slot_s'],pre['initial_hold_s']+(j+1)*pre['direction_slot_s'],jog_seconds,j) for j in range(14)])
    stage('timing','电流响应 / 指令延迟 / 周期对照',[
        segment(name,.035,min(.095,pre['timing_duration_s']),seconds/3,0)
        for name in ('driver_timing_train','driver_timing_validation_2ms','driver_timing_validation_4ms')])
    stage('gravity','多构型重力保持采集',[segment('gravity_train',0,min(2*motion['gravity_hold_s'],motion['gravity_hold_s']*motion['gravity_poses']),seconds)])
    friction_start=8.*motion['discard_periods']
    stage('friction','正反向低速摩擦',[segment('friction_train',friction_start,friction_start+8.,seconds)])
    start=motion['period_s']*motion['discard_periods'];end=start+motion['period_s']
    stage('dynamic','双臂大幅多频 / 加速度激励',[segment('dynamic_train',start,end,seconds)])
    stage('validation','冻结参数 / 独立双臂轨迹',[segment('dynamic_validation',start,end,seconds)])
    stage('single_arm','左右臂分别运动 / 交叉验证',[
        segment('left_only_validation',start,end,seconds/2,3),segment('right_only_validation',start,end,seconds/2,10)])
    stage('control','固定 PD / 前馈前后对照',[segment('identified_control_validation',start,end,seconds)])
    for row in stages:
        for clip in row['segments']:
            clip['playback_rate']=(clip['end_s']-clip['start_s'])/clip['seconds']
    result=dict(schema_version=1,stages=stages,angles_amplified=False,source='recorded observations and command logs; no evaluation truth')
    write_json(folder/'storyboard.json',result);return result
