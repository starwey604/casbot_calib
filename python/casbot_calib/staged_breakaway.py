"""Gravity-compensated guarded ramps and operational static-onset intervals."""
import csv,json
from pathlib import Path
import numpy as np
from . import _core
from .data import write_json
from .staged_collection import load_trial,collect_trial
from .staged_identification import recordings,session_config,_windows
from .torque_observations import output_torque


def ramp_policy(meta,curve,gravity,config):
    j=meta['conditions']['joint'];direction=meta['conditions']['direction'];slope=meta['conditions']['slope_nm_s']
    begin,end=_windows(meta,'torque_ramp')[0];dwell_begin,_=_windows(meta,'static_dwell')[0]
    baseline=float(gravity.gravity(curve(begin))[j]);guard=_core.ArmGuardConfig()
    guard.joint=j;guard.begin_ns=round(begin*1e9);guard.end_ns=round(end*1e9)
    guard.baseline_torque=baseline;guard.restore_kp=config['drive']['kp'][j];guard.restore_kd=config['drive']['kd'][j]
    guard.maximum_displacement=.002;guard.maximum_speed=.4
    def transform(t,commands,metadata):
        off=(t>=dwell_begin)&(t<end);ramp=np.maximum(t-begin,0)
        commands[off,j,2]=baseline+direction*slope*ramp[off];commands[off,j,3:]=0.
        return commands
    return transform,guard


def collect_breakaway(root,name,plant,gravity_friction,config,profile):
    meta,curve=load_trial(root,name);transform,guard=ramp_policy(meta,curve,gravity_friction,config)
    return collect_trial(root,name,plant,gravity_friction,config,profile,command_transform=transform,guard=guard)


def identify_breakaway(root,output,gravity,*,records=None,config=None,guard_reader=None):
    root=Path(root);output=Path(output);output.mkdir(parents=True,exist_ok=False);config=session_config(root) if config is None else config;rows=[]
    source=(lambda stage:recordings(root,stage)) if records is None else records
    for name,meta,obs in source('breakaway'):
        policy=json.loads((root/'runs'/name/'guard_events.json').read_text()) if guard_reader is None else guard_reader(name)
        rows.append(onset_record(name,meta,obs,policy,gravity,config))
    axes=[]
    for j in range(14):
        directions={}
        for sign in (-1,1):
            selected=[r for r in rows if r['joint']==j and r['direction']==sign]
            training=[r['estimate_nm'] for r in selected if r['repeat']==0];validation=[r['estimate_nm'] for r in selected if r['repeat']==1]
            if not training or not validation:raise ValueError('Independent repeated ramp batch missing')
            estimate=float(np.median(training));error=np.abs(np.asarray(validation)-estimate)
            groups={key:{str(value):float(np.median([r['estimate_nm'] for r in selected if r[key]==value])) for value in sorted({r[key] for r in selected})}
                for key in ('pose','dwell_s','slope_nm_s')}
            directions[str(sign)]=dict(estimate_nm=estimate,validation_rmse_nm=float(np.sqrt(np.mean(error**2))),
                minimum_nm=float(min(training+validation)),maximum_nm=float(max(training+validation)),
                group_medians_nm=groups,training_repeats=[0],validation_repeats=[1],
                passed=bool(np.max(error)<=.01 and all(r['passed'] for r in selected)))
        axes.append(dict(joint=j,directions=directions,equivalent_symmetric_static_nm=float(np.mean([d['estimate_nm'] for d in directions.values()])),
            passed=all(d['passed'] for d in directions.values())))
    report=dict(stage='breakaway',passed=all(r['passed'] for r in axes),axes=axes,ramps=rows,
        static_friction=[r['equivalent_symmetric_static_nm'] for r in axes],
        onset_velocity_resolution_rad_s=1e-7,controller_confirmation=dict(displacement_rad=.0002,speed_rad_s=.005,sustain_ns=3000000),
        torque_source='total output torque observation minus frozen gravity at actual sampled posture; legacy current uses declared output gain',
        gravity_uncertainty_source='earlier independent paired gravity residual and approach spread; not eliminated by a fast ramp',
        limitations=['Operational threshold at this velocity/current resolution, not a material maximum static-friction constant',
            'No dwell- or posture-dependent friction law fitted; group effects are reported',
            'The equivalent symmetric value is for Mock/prediction; direction-specific results are retained'])
    write_json(output/'report.json',report)
    with (output/'thresholds.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['trial','joint','direction','pose','dwell_s','slope_Nm_s','repeat','onset_Nm','lower_Nm','upper_Nm','confirmation_Nm'])
        writer.writerows([r['trial'],r['joint'],r['direction'],r['pose'],r['dwell_s'],r['slope_nm_s'],r['repeat'],r['estimate_nm'],*r['onset_interval_nm'],r['operational_confirmation_nm']] for r in rows)
    _plot(output,report)
    return report


def onset_record(name,meta,obs,policy,gravity,config):
    """Observation-only operational bracket, shared by fitting and frozen tests."""
    conditions=meta['conditions'];j=conditions['joint'];direction=conditions['direction'];begin,end=_windows(meta,'torque_ramp')[0]
    if len(policy['events'])!=1:raise ValueError('Exactly one feedback confirmation required')
    event=policy['events'][0]
    if event['fault'] or event['reason']!='breakaway_confirmed':raise ValueError('Feedback guard failed: '+name)
    t=obs['sample_ns']*1e-9;pre=(t>begin-.15)&(t<begin-.02)
    if not np.any(pre) or np.max(abs(obs['dq'][pre,j]))>1e-5:raise ValueError('Joint was not stationary before ramp: '+name)
    initial_q=float(np.median(obs['q'][pre,j]));mask=(t>=begin)&(obs['sample_ns']<=event['confirmed_sample_ns'])
    indices=np.flatnonzero(mask);velocity=direction*obs['dq'][indices,j]
    moving=np.flatnonzero(velocity>1e-7)
    if not len(moving) or moving[0]==0:raise ValueError('No resolvable still-to-moving bracket: '+name)
    first=indices[moving[0]];last=first-1
    confirmation=np.searchsorted(obs['sample_ns'],event['confirmed_sample_ns'])
    if direction*(obs['q'][confirmation,j]-initial_q)<policy['displacement_threshold_rad']*.8:
        raise ValueError('Displacement evidence inconsistent with guard confirmation')
    q=obs['q'][[last,first,confirmation]];g=gravity.gravity_batch(q)[:,j]
    efforts=direction*(output_torque(obs,config['drive'].get('gain_nm_a'))[[last,first,confirmation],j]-g)
    uncertainty=3*(config['drive']['torque_noise_nm'] if 'torque_noise_nm' in config['drive'] else config['drive']['current_noise_a']*config['drive']['gain_nm_a'])
    low=float(min(efforts[:2])-uncertainty);high=float(max(efforts[:2])+uncertainty)
    return dict(trial=name,**conditions,last_still_ns=int(obs['sample_ns'][last]),first_moving_ns=int(obs['sample_ns'][first]),
        confirmed_sample_ns=event['confirmed_sample_ns'],decision_ns=event['decision_ns'],
        onset_interval_nm=[low,high],estimate_nm=float(np.mean(efforts[:2])),operational_confirmation_nm=float(efforts[2]),
        motion_confirmation_delay_s=float((event['confirmed_sample_ns']-obs['sample_ns'][first])*1e-9),
        passed=bool(low>0 and high-low<.01),
        interpretation='operational onset at stated velocity resolution, with current-noise interval; gravity uncertainty reported separately')


def _plot(output,report):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure=Figure(figsize=(11,5));FigureCanvasAgg(figure);axis=figure.subplots()
    for sign in (-1,1):axis.plot([r['directions'][str(sign)]['estimate_nm'] for r in report['axes']],'o-',label=f'direction {sign:+}')
    axis.set(xlabel='L1..L7, R1..R7',ylabel='Operational static onset / Nm');axis.legend();axis.grid(alpha=.2)
    figure.tight_layout();figure.savefig(output/'thresholds.png',dpi=140)
    (output/'report.md').write_text('# 起动摩擦阶段\n\n'+f"{'通过' if report['passed'] else '失败'}；多姿态、正反向、两种静置时间、两种斜率、独立重复批次。\n\n"+
        '电机确认位移及持续速度后停止斜坡并恢复保持；随后由采集数据回溯起动区间，单独保留确认时较晚的力矩。\n\n[JSON](report.json) · [CSV](thresholds.csv) · [分布图](thresholds.png)\n\n仅为合成Mock的操作性阈值，不替代实机仪器分辨率和重力误差验收。\n',encoding='utf-8')
