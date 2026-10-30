"""H4 control selection and independent model x gain comparisons over the SDK."""
import csv
import hashlib
from pathlib import Path
import numpy as np
from scipy.interpolate import BPoly
from . import hardware_field as field
from .data import write_json
from .hardware_friction import read
from .vendor_identity import sha256


def control_factor(root,job):
    if job['kind']=='control_tuning':return job['kp_factor']
    if job.get('gain_strategy')!='tuned':return 1.
    folder=Path(root)/'controller_selection';field.check_seal(folder)
    return read(folder/'report.json')['selected_kp_factor']


def control_metrics(plan,meta,commands,origin,obs,elapsed,c):
    with np.load(Path(plan)/'reference.npz',allow_pickle=False) as data:curve=BPoly(data['coefficients'],data['breakpoints'])
    motion=read(Path(plan)/'motion.json');windows=[(motion['knots_s'][i],motion['knots_s'][i+1]) for i,label in enumerate(motion['labels']) if label=='periodic']
    begin=windows[0][0]+c['discard_periods']*c['period_s'];end=windows[-1][1]
    use=(elapsed>=begin)&(elapsed<end);times=elapsed[use]
    if len(times)<20 or np.ptp(times)<1.8*c['period_s'] or np.max(np.diff(times))>c['maximum_gap_s']:raise ValueError('Insufficient contiguous SDK control observations')
    axes=[meta['joint_names'].index(name) for name in c['joints']]
    dt=np.diff(times);weights=np.r_[dt[0]/2,(dt[:-1]+dt[1:])/2,dt[-1]/2];weights/=weights.sum()
    errors=obs['q'][use]-curve(times);per_axis=np.sqrt(np.sum(weights[:,None]*errors**2,axis=0))
    config=meta['config'];effort=(obs['effort_raw']-np.asarray(config['effort_zero_nm']))*np.asarray(config['effort_gain'])
    maximum_speed=float(np.max(abs(obs['dq'])));maximum_effort=float(np.max(abs(effort)))
    # Compare the executed effort/PD arrays to the exact frozen schedule. This
    # is publisher evidence, not a claimed device adoption acknowledgment.
    schedule=np.loadtxt(Path(plan)/'feedforward_plan.csv',delimiter=',',skiprows=1)
    extra=config['breakaway_parameters'];kp=np.asarray(extra['command_kp']);kd=np.asarray(extra['command_kd'])
    for row in commands:
        index=int(row['planned_index'])
        for key,expected in [('effort',schedule[index,1:]),('kp',kp),('kd',kd)]:
            actual=np.array([float(row[key+str(j)]) for j in range(14)])
            if not np.allclose(actual,expected,rtol=0,atol=1e-10):raise ValueError('Executed H4 effort/PD differs from frozen plan')
    phase=np.arange(c['phase_points'])*c['period_s']/c['phase_points']
    definition=np.ascontiguousarray(curve(windows[0][0]+phase),dtype='<f8')
    admissible=bool(maximum_speed<=c['control']['maximum_speed_rad_s'] and maximum_effort<=c['control']['maximum_effort_nm'])
    return dict(passed=admissible,admissible=admissible,stage=meta['stage'],job=meta['job'],
        aggregate_rmse_rad=float(np.sqrt(np.mean(per_axis[axes]**2))),rmse_rad=per_axis.tolist(),
        periodic_reference_sha256=hashlib.sha256(definition.tobytes()).hexdigest(),
        scoring_interval_s=[begin,end],maximum_observed_speed_rad_s=maximum_speed,maximum_observed_effort_nm=maximum_effort,
        command_schedule_checked=True,device_adoption_known=False,saturation_status_known=False,
        sample_time_known=False,formal_hardware_calibration_accepted=False,
        time_basis='callback q versus common planned reference at callback availability; no per-group lag optimization',
        initial_condition='fresh measured snapshot and complete entry; physical state not reset or asserted identical')


def freeze_selection(root,protocol,history):
    from . import hardware_batch as batch
    root=Path(root);folder=root/'controller_selection'
    if folder.exists():
        field.check_seal(folder);logged=[e for e in batch.events(root) if e['event']=='controller_selected']
        if len(logged)!=1 or logged[0]['manifest_sha256']!=sha256(folder/'manifest.json'):raise ValueError('Frozen H4 controller selection changed')
        return read(folder/'report.json')['passed']
    jobs=[j for j in protocol['jobs'] if j['kind']=='control_tuning'];grid=[]
    if len(jobs)!=5:return False
    for job in jobs:
        source=batch.accepted(history[job['id']])
        if source is None:return False
        row=read(root/source['attempt']/'inspection/report.json')
        grid.append(dict(**row,source_attempt=source['attempt'],source_manifest_sha256=source['manifest_sha256']))
    if len({r['periodic_reference_sha256'] for r in grid})!=1:raise ValueError('PD candidates used different periodic references')
    allowed=[r for r in grid if r['admissible']]
    if not allowed:return False
    best=min(allowed,key=lambda r:r['aggregate_rmse_rad'])
    folder.mkdir();write_json(folder/'report.json',dict(passed=True,selected_kp_factor=best['job']['kp_factor'],
        selected_kd_factor=float(np.sqrt(best['job']['kp_factor'])),grid=grid,
        validation_used_for_selection=False,selection_objective='five-point tracking RMSE, same frozen prior feedforward',
        inertia_checkpoint_manifest_sha256=sha256(root/'inertia_checkpoint/manifest.json')))
    field.seal(folder);batch.event(root,'controller_selected',manifest_sha256=sha256(folder/'manifest.json'));return True


def control_result(root,protocol,history):
    from . import hardware_batch as batch
    root=Path(root)
    if not protocol['h4']['control']['enabled']:return dict(available=False,passed=True,enabled=False)
    jobs=[j for j in protocol['jobs'] if j['kind']=='control_validation']
    if any(not batch.accepted(history[j['id']]) for j in jobs):return dict(available=False,passed=False)
    if not (root/'controller_selection').exists() or not freeze_selection(root,protocol,history):return dict(available=False,passed=False)
    groups={}
    for job in jobs:
        source=batch.accepted(history[job['id']]);groups[job['id']]=dict(read(root/source['attempt']/'inspection/report.json'),source_attempt=source['attempt'])
    if len(groups)!=4 or len({r['periodic_reference_sha256'] for r in groups.values()})!=1:raise ValueError('Four common-reference H4 control groups required')
    value=lambda name:groups['control_'+name]['aggregate_rmse_rad']
    baseline=value('prior_original');identified=value('identified_original');tuned=value('prior_tuned')
    selection=read(root/'controller_selection/report.json');gain_changed=selection['selected_kp_factor']!=1.
    repeat_changes={}
    if not gain_changed:
        repeat_changes={model:abs(value(model+'_original')-value(model+'_tuned')) for model in ('prior','identified')}
    model_change=baseline-identified
    interpretation=benefit_interpretation(model_change,repeat_changes)
    return dict(available=True,passed=all(r['admissible'] for r in groups.values()),groups=groups,
        fixed_pd_feedforward_rmse_reduction=1-identified/max(baseline,1e-15),prior_model_pd_rmse_reduction=1-tuned/max(baseline,1e-15),
        identified_model_pd_rmse_reduction=1-value('identified_tuned')/max(identified,1e-15),
        selection=selection,independent_control_trajectory=True,selected_gain_changed=gain_changed,
        pd_difference_interpretation='different configured gains; exploratory paired observations' if gain_changed else 'identical gains; measured differences are repeat variability, not PD tuning benefit',
        **interpretation,
        unconditional_improvement_claim=False,formal_hardware_calibration_accepted=False,
        interpretation='same periodic reference, limits and frozen gain selection; measured initial states retained, no physical state teleport; negative benefits retained')


def benefit_interpretation(model_change,repeat_changes):
    """Descriptive repeat differences are not a confidence interval or p-value."""
    smaller=bool(repeat_changes and abs(model_change)<=max(repeat_changes.values()))
    return dict(model_rmse_difference_rad=model_change,observed_same_gain_repeat_changes_rad=repeat_changes,
        model_difference_smaller_than_observed_repeat_change=smaller,stable_control_benefit_demonstrated=False,
        control_benefit_status='inconclusive: model difference is smaller than observed identical-gain repeat change' if smaller else
            'exploratory single batch; repeated randomized validation required for a stable benefit claim')


def plot_results(destination,inertia,control):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    destination=Path(destination)
    if inertia['available']:
        fig=Figure(figsize=(10,5));FigureCanvasAgg(fig);ax=fig.subplots()
        for name,values in inertia['validation'].items():ax.plot(values['rmse_nm'],'o-',label=name)
        ax.set(xlabel='Identified joint index',ylabel='Independent prediction RMSE / Nm');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(destination/'inertia_validation.png')
    if control['available']:
        rows=list(control['groups'].values())
        with (destination/'control_comparison.csv').open('w',newline='',encoding='utf-8-sig') as stream:
            writer=csv.writer(stream);writer.writerow(['group','RMSE_rad','maximum_speed_rad_s','maximum_effort_Nm','admissible'])
            writer.writerows([r['job']['id'],r['aggregate_rmse_rad'],r['maximum_observed_speed_rad_s'],r['maximum_observed_effort_nm'],r['admissible']] for r in rows)
        fig=Figure(figsize=(10,4));FigureCanvasAgg(fig);ax=fig.subplots();ax.bar(range(4),[r['aggregate_rmse_rad'] for r in rows])
        ax.set_xticks(range(4),[r['job']['id'].replace('control_','') for r in rows]);ax.set(ylabel='Independent tracking RMSE / rad');ax.grid(alpha=.2);fig.tight_layout();fig.savefig(destination/'control_comparison.png')
