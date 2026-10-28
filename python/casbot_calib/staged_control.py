"""Five-point PD selection and independent four-way feedforward/PD validation."""
import csv,hashlib,json
from pathlib import Path
import numpy as np
from .data import write_json
from .dual_model import nominal_dual_arm
from .rigid_identification import model_from_parameters
from .staged_collection import collect_trial,load_trial


def load_candidate(path):
    bundle=json.loads(Path(path).read_text());nominal,source=nominal_dual_arm()
    if bundle['model_version']!='tree_full_physical_inertia_v1' or bundle['source_sha256']!=source['sha256'] or bundle['joint_names']!=[j.name for j in nominal.model.joints]:
        raise ValueError('Calibrated model version/source/order mismatch')
    if bundle['gain_status'] not in ('synthetic_declared','independently_validated'):raise ValueError('Candidate torque scale is not anchored')
    return model_from_parameters(nominal,bundle['parameters'],bundle['viscous'],bundle['coulomb'],bundle['static_friction'])


def _metrics(root,name,config):
    root=Path(root);meta,curve=load_trial(root,name)
    metadata=json.loads((root/'runs'/name/'metadata.json').read_text())
    for filename,key in [('observations.npz','raw_sha256'),('commands.npz','command_sha256'),('feedforward.json','feedforward_sha256')]:
        if hashlib.sha256((root/'runs'/name/filename).read_bytes()).hexdigest()!=metadata[key]:raise ValueError('Control recording changed: '+name)
    with np.load(root/'runs'/name/'observations.npz',allow_pickle=False) as data:obs={k:data[k] for k in data.files}
    window=[(meta['knots_s'][i],meta['knots_s'][i+1]) for i,label in enumerate(meta['labels']) if label=='periodic']
    begin=window[0][0]+2*meta['conditions']['period_s'];end=window[-1][1];t=obs['sample_ns']*1e-9;mask=(t>=begin)&(t<end)
    error=obs['q'][mask]-curve(t[mask]);per_axis=np.sqrt(np.mean(error**2,axis=0))
    speed=float(np.max(abs(obs['dq'])));torque=float(np.max(abs(obs['current']*config['drive']['gain_nm_a'])))
    return dict(trial=name,aggregate_rmse_rad=float(np.sqrt(np.mean(error**2))),rmse_rad=per_axis.tolist(),
        maximum_observed_speed_rad_s=speed,maximum_observed_current_torque_nm=torque,saturated=metadata['saturated'],
        collision_passed=metadata['actual_interpolant_gate_passed'],
        admissible=bool(metadata['passed'] and speed<=2 and torque<=config['drive']['torque_limit_nm']),
        timing='target at acquisition time, including identical command/drive delay for all groups',
        scoring_interval_s=[begin,end],limits_scope='synthetic speed envelope and observed drive limits; no manufacturer capabilities asserted')


def _scale(factor):
    def transform(t,commands,meta):
        commands[:,:,3]*=factor;commands[:,:,4]*=np.sqrt(factor)
        return commands
    return transform


def _verify_reused_controller(root,name,model,config,factor):
    saved=json.loads((root/'runs'/name/'feedforward.json').read_text())
    for key,values in [('rigid_parameters',model.rigid_parameters()),('viscous',[j.friction.viscous for j in model.model.joints]),('coulomb',[j.friction.coulomb for j in model.model.joints])]:
        if not np.allclose(saved[key],values,rtol=1e-9,atol=1e-11):raise ValueError('Frozen controller model changed; do not reuse control validation')
    _,curve=load_trial(root,name)
    with np.load(root/'runs'/name/'commands.npz',allow_pickle=False) as data:
        if not np.allclose(data['commands'][:,:,3],np.asarray(config['drive']['kp'])*factor,rtol=0,atol=1e-12) or not np.allclose(data['commands'][:,:,4],np.asarray(config['drive']['kd'])*np.sqrt(factor),rtol=0,atol=1e-12):
            raise ValueError('Controller gains changed; control validation cannot be reused')
        t=data['send_ns']*1e-9
        expected=model.evaluate_batch(curve(t),curve(t,nu=1),curve(t,nu=2))['total']
        if not np.allclose(data['commands'][:,:,2],expected,rtol=1e-9,atol=1e-10):
            raise ValueError('Actual feedforward changed, including static/Stribeck friction; do not reuse control validation')


def run_control(root,plant,config,profile,inertia_folder,resume=False):
    root=Path(root);inertia_folder=Path(inertia_folder);inertia=json.loads((inertia_folder/'report.json').read_text())
    if not inertia['passed']:raise ValueError('Control validation requires accepted inertia and previous stages')
    candidate=load_candidate(inertia_folder/'candidate.json');nominal,_=nominal_dual_arm()
    selected=json.loads((root/'plan_report.json').read_text())['selected_trials'];grid=[]
    for name in selected:
        meta,_=load_trial(root,name)
        if meta['stage']!='control_tuning':continue
        factor=meta['conditions']['kp_factor']
        if resume and (root/'runs'/name/'metadata.json').exists():_verify_reused_controller(root,name,nominal,config,factor)
        if not(resume and (root/'runs'/name/'metadata.json').exists()):
            try:collect_trial(root,name,plant,nominal,config,profile,command_transform=_scale(factor))
            except RuntimeError:
                path=root/'runs'/name/'metadata.json'
                if not path.exists() or json.loads(path.read_text())['passed']:raise
        metrics=_metrics(root,name,config);grid.append(dict(kp_factor=factor,kd_factor=float(np.sqrt(factor)),**metrics))
    if len(grid)!=5:raise ValueError('Exactly five predeclared PD candidates required')
    allowed=[r for r in grid if r['admissible']]
    if not allowed:raise RuntimeError('No PD candidate satisfies the declared constraints')
    best=min(allowed,key=lambda x:x['aggregate_rmse_rad']);factor=best['kp_factor']
    selection=dict(grid=grid,selected_kp_factor=factor,selected_kd_factor=float(np.sqrt(factor)),
        selection_objective='tracking RMSE on nominal feedforward tuning trajectory only',
        validation_used_for_selection=False,training_seed=227,validation_seed=327,
        common_gain_for_both_models=True,baseline_kp=config['drive']['kp'],baseline_kd=config['drive']['kd'])
    write_json(root/'controller_selection.json',selection)
    results={}
    for name in selected:
        meta,_=load_trial(root,name)
        if meta['stage']!='control_validation':continue
        identified=meta['conditions']['feedforward']=='identified';tuned=meta['conditions']['gain_strategy']=='tuned'
        if resume and (root/'runs'/name/'metadata.json').exists():_verify_reused_controller(root,name,candidate if identified else nominal,config,factor if tuned else 1.)
        if not(resume and (root/'runs'/name/'metadata.json').exists()):
            collect_trial(root,name,plant,candidate if identified else nominal,config,profile,command_transform=_scale(factor if tuned else 1.))
        results[name]=_metrics(root,name,config)
    if len(results)!=4:raise ValueError('Four independent control groups required')
    n0=results['control_nominal_original']['aggregate_rmse_rad'];i0=results['control_identified_original']['aggregate_rmse_rad']
    n1=results['control_nominal_tuned']['aggregate_rmse_rad'];i1=results['control_identified_tuned']['aggregate_rmse_rad']
    report=dict(passed=all(r['admissible'] for r in results.values()) and 1-i0/n0>=.2,groups=results,
        fixed_pd_feedforward_rmse_reduction=1-i0/n0,nominal_model_pd_rmse_reduction=1-n1/n0,
        identified_model_pd_rmse_reduction=1-i1/i0,selected_kp_factor=factor,
        independent_validation=True,minimum_synthetic_fixed_pd_feedforward_reduction=.2,
        interpretation='same plant/reference/initial state/noise/limits; model and PD factors crossed; negative PD benefit retained if observed')
    write_json(root/'control_comparison.json',report)
    with (root/'control_comparison.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['group','aggregate_RMSE_rad','max_observed_speed_rad_s','max_observed_current_torque_Nm','admissible'])
        writer.writerows((name,row['aggregate_rmse_rad'],row['maximum_observed_speed_rad_s'],row['maximum_observed_current_torque_nm'],row['admissible']) for name,row in results.items())
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure=Figure(figsize=(10,5));FigureCanvasAgg(figure);axis=figure.subplots()
    for name,row in results.items():axis.plot(np.asarray(row['rmse_rad'])*1000,'o-',label=name.removeprefix('control_'))
    axis.set(xlabel='L1..L7, R1..R7',ylabel='Independent tracking RMSE / mrad');axis.legend();axis.grid(alpha=.2)
    figure.tight_layout();figure.savefig(root/'control_comparison.png',dpi=140)
    return report
