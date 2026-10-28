"""Train-only multi-trajectory physical fitting and R1 development diagnostics."""
import csv
from pathlib import Path
from time import perf_counter
import numpy as np
from .data import write_json
from .dual_model import nominal_dual_arm
from .rigid_identification import fit_rigid, model_from_parameters
from .staged_identification import recordings
from .staged_inertia import periodic_problem, probe_problem, probe_metrics, _metrics, _preserve_prior_stages
from .staged_protocol import file_hash, load_protocol, _read


def load_problems(root, split, static):
    protocol, config, _ = load_protocol(root); nominal, source = nominal_dual_arm()
    if source['sha256'] != protocol['model']['sha256']:
        raise ValueError('R1 nominal model changed since acquisition; revalidate geometry/model before analysis')
    purpose = {'train': 'fit', 'selection': 'select', 'test': 'final_test'}[split]
    result = {}
    for stage in ('inertia', 'inertia_probe'):
        for name, meta, obs in recordings(root, stage, split=split, purpose=purpose):
            problem = periodic_problem(nominal, obs, meta, config['drive']['gain_nm_a'], static,
                config['p0']['preprocessing']['harmonics'], config['p0']['preprocessing']['phase_points']) if stage == 'inertia' else probe_problem(nominal, obs, meta, config['drive']['gain_nm_a'])
            # Coverage comes from actual observations in the active window.
            begin, end = (meta['knots_s'][0], meta['duration_s'])
            label = 'periodic' if stage == 'inertia' else 'probe_measure'
            windows = [(meta['knots_s'][i], meta['knots_s'][i+1]) for i, l in enumerate(meta['labels']) if l == label]
            begin = windows[0][0] + (2*config['dynamic_period_s'] if stage == 'inertia' else .2)
            end = windows[-1][1] - (0. if stage == 'inertia' else .2)
            t = obs['sample_ns']*1e-9; active = (t >= begin) & (t < end)
            coverage = dict(minimum_rad=obs['q'][active].min(axis=0).tolist(), maximum_rad=obs['q'][active].max(axis=0).tolist(),
                mean_rad=obs['q'][active].mean(axis=0).tolist(),
                range_rad=np.ptp(obs['q'][active], axis=0).tolist(), speed_rms_rad_s=np.sqrt(np.mean(obs['dq'][active]**2,axis=0)).tolist(),
                acceleration_rms_rad_s2=np.sqrt(np.mean(problem['ddq']**2,axis=0)).tolist(), source='actual acquired q/dq, Fourier/probe derivative; active measurement window')
            result[name] = dict(meta=meta, problem=problem, coverage=coverage)
    return result


def assemble_training(problems, config):
    """Equal trajectory/axis cost, fixed train-only torque normalizers.

    Keep 1024 effective rows per block as a declared global objective scale, so
    the ridge path is independent of raw acquisition rate and repeated frames.
    """
    if not problems or any(r['meta']['conditions']['split'] != 'train' for r in problems.values()):
        raise ValueError('Only complete train trajectories may enter a fit')
    periodic = [r for r in problems.values() if r['meta']['stage'] == 'inertia']
    probes = [r for r in problems.values() if r['meta']['stage'] == 'inertia_probe']
    if not periodic or not probes:
        raise ValueError('Both primary trajectories and wrist probes are required')
    floor = config['p0']['weighting']['torque_floor_nm']; rows = []; extras = []; targets = []; weights = []
    for name, row in problems.items():
        p = row['problem']; conditions = row['meta']['conditions']
        if row['meta']['stage'] == 'inertia':
            for j in range(14):
                mask = p['moving'][:, j]; count = int(mask.sum())
                if count < 16:
                    raise ValueError('Insufficient train excitation for axis ' + str(j))
                normalizer = max(float(np.sqrt(np.mean(p['current_torque'][mask,j]**2))), floor)
                factor = np.sqrt(1024./(count*len(periodic)))/normalizer
                if j in (4, 11): factor *= config['p0']['weighting']['primary_wrist_sqrt_weight']
                rows.append(p['regression'][mask,j,:]*factor); extras.append(p['extra'][mask,j,:]*factor); targets.append(p['target'][mask,j]*factor)
                weights.append(dict(trial_id=name, joint=j, rows=count, torque_normalizer_nm=normalizer, sqrt_weight=factor,
                    center_id=conditions['center_id'], acquisition_batch_id=conditions['acquisition_batch_id'], stage='inertia'))
        else:
            count = len(p['target']); normalizer = max(float(np.sqrt(np.mean(p['target']**2))), floor)
            factor = np.sqrt(1024./(count*len(probes)))/normalizer
            rows.append(p['regression']*factor); extras.append(p['extra']*factor); targets.append(p['target']*factor)
            weights.append(dict(trial_id=name, joint=p['joint'], rows=count, torque_normalizer_nm=normalizer, sqrt_weight=factor,
                center_id=conditions['center_id'], acquisition_batch_id=conditions['acquisition_batch_id'], stage='inertia_probe'))
    return np.vstack(rows), np.vstack(extras), np.concatenate(targets), weights


def score_problems(problems, candidate, theta, friction, static, config):
    threshold = config['p0']['acceptance']; rows = {}; probes = {}
    for name, row in problems.items():
        p = row['problem']; meta = row['meta']; conditions = meta['conditions']
        identity = {k: conditions[k] for k in ('split', 'center_id', 'reference_id', 'acquisition_batch_id', 'repeat_id', 'active_arm_group')}
        if meta['stage'] == 'inertia':
            metric = _metrics(p, theta, friction, static, threshold['torque_floor_nm'])
            group = conditions['active_arm_group']; active = list(range(14)) if group == 'both' else list(range(7) if group == 'left' else range(7,14))
            coverage = row['coverage']
            excited = all(coverage['range_rad'][j] > .15 and coverage['speed_rms_rad_s'][j] > .01 and coverage['acceleration_rms_rad_s2'][j] > .02 for j in active)
            predicted = p['regression']@theta+p['extra']@friction+p['fixed_friction']+p['rotor']
            metric.update(identity, actual_coverage=coverage, active_joints=active,
                mean_full_period_residual_nm=np.mean(predicted-p['current_torque'],axis=0).tolist(),
                passed=bool(excited and metric['maximum_normalized_rmse'] <= threshold['dynamic_axis_nrmse'] and
                    metric['maximum_stationary_interval_excess_nm'] <= threshold['stationary_excess_nm']))
            rows[name] = metric
        else:
            probes[name] = dict(**probe_metrics(p,candidate,theta,friction), **identity, actual_coverage=row['coverage'])
    return rows, probes


def batch_repeatability(rows, limit=.01):
    grouped = {}; result = []
    for name, row in rows.items():
        grouped.setdefault(row['reference_id'], []).append((name,row))
    for reference, records in grouped.items():
        if len(records) != 2 or {r['repeat_id'] for _,r in records} != {0,1}:
            raise ValueError('Missing/duplicate batch in repeatability: ' + reference)
        a,b = [r for _,r in sorted(records, key=lambda pair:pair[1]['repeat_id'])]
        difference = abs(np.asarray(a['mean_full_period_residual_nm'])-b['mean_full_period_residual_nm'])
        result.append(dict(reference_id=reference, split=a['split'], center_id=a['center_id'],
            trials=[name for name,_ in records], mean_residual_difference_nm=difference.tolist(),
            rmse_nm=[a['rmse_nm'],b['rmse_nm']],
            passed=bool(np.max(difference)<=limit and a['passed'] and b['passed'])))
    return dict(schema_version=1, passed=bool(result and all(r['passed'] for r in result)), references=result,
        maximum_mean_difference_nm=max((max(r['mean_residual_difference_nm']) for r in result),default=None),
        scope='same reference / independent Mock initialization and sensor noise; train rows remain in-sample, not thermal/cross-day evidence')


def grouped_prediction_summary(metrics, probes):
    result={}
    for key in ('split','center_id','acquisition_batch_id'):
        groups={}
        for value in sorted({r[key] for r in [*metrics.values(),*probes.values()]}):
            primary=[r for r in metrics.values() if r[key]==value];wrist=[r for r in probes.values() if r[key]==value]
            groups[value]=dict(primary_trials=len(primary),wrist_trials=len(wrist),
                maximum_axis_nrmse=max((r['maximum_normalized_rmse'] for r in primary),default=None),
                maximum_stationary_excess_nm=max((r['maximum_stationary_interval_excess_nm'] for r in primary),default=None),
                maximum_wrist_inertia_disagreement=max((r['relative_inertia_disagreement'] for r in wrist),default=None),
                maximum_wrist_prediction_rmse_nm=max((r['rmse_nm'] for r in wrist),default=None),
                passed=all(r['passed'] for r in primary+wrist))
        result[key]=groups
    return result


def wrist_repeatability(probes,limit):
    groups={};rows=[]
    for name,row in probes.items():groups.setdefault(row['reference_id'],[]).append((name,row))
    for reference,items in groups.items():
        if len(items)!=2 or {r['repeat_id'] for _,r in items}!={0,1}:raise ValueError('Missing independent wrist repeat')
        a,b=[r for _,r in items];difference=abs(a['mean_residual_nm']-b['mean_residual_nm'])
        rows.append(dict(reference_id=reference,trials=[n for n,_ in items],mean_residual_difference_nm=difference,
            rmse_nm=[a['rmse_nm'],b['rmse_nm']],passed=bool(difference<=limit and a['passed'] and b['passed'])))
    return dict(passed=bool(rows and all(r['passed'] for r in rows)),references=rows,
        scope='matched nonperiodic acceleration-probe windows; no periodic FFT interpretation')


def measured_center_coverage(metrics,config):
    from .staged_multiconfiguration import center_separation
    names=list(dict.fromkeys(r['center_id'] for r in metrics.values()))
    centers={name:np.mean([r['actual_coverage']['mean_rad'] for r in metrics.values() if r['center_id']==name],axis=0) for name in names}
    pairs=[dict(first=a,second=b,**center_separation(centers[a],centers[b],np.asarray(config['pose_amplitude_rad']),config['p0']['center_design']))
           for i,a in enumerate(names) for b in names[i+1:]]
    return dict(passed=bool(pairs and all(p['passed'] for p in pairs)),centers={n:q.tolist() for n,q in centers.items()},pairs=pairs,
        source='actual acquired mean q over complete steady periods; test centers not observed')


def candidate_bundle(theta, friction, static, nominal, source, config, fit, training):
    return dict(model_version='tree_full_physical_inertia_v1', source_sha256=source['sha256'], parameters=np.asarray(theta).tolist(),
        viscous=np.asarray(friction[:14]).tolist(), coulomb=np.asarray(friction[14:]).tolist(), static_friction=np.asarray(static).tolist(),
        gain_status=config['drive']['gain_status'], gain_nm_a=config['drive']['gain_nm_a'], joint_names=[j.name for j in nominal.model.joints],
        hardware_executable=False, parameter_status=fit['identifiability'], training=list(training), validation=[],
        development_only=True, p0_acceptance_passed=False, publishable=False)


def identify_development(root, output, prior_analysis):
    started=perf_counter();root=Path(root);output=Path(output)
    _,config,_=load_protocol(root);nominal,source=nominal_dual_arm();prior_analysis=Path(prior_analysis)
    if output.exists():raise FileExistsError(output)
    g,f,s=(_read(prior_analysis/stage/'report.json') for stage in ('gravity','friction','breakaway'))
    if not all(r['passed'] for r in (g,f,s)):raise ValueError('Predecessor development gates failed')
    theta_g=np.asarray(g['fit']['parameters']);frozen=np.r_[f['viscous'],f['coulomb']];static=np.asarray(s['static_friction'])
    training=load_problems(root,'train',static)
    y,x,target,weights=assemble_training(training,config)
    with np.load(prior_analysis/'gravity/gravity_constraints.npz',allow_pickle=False) as arrays:
        equality=(arrays['regressor'],arrays['prediction'])
    initial=fit_rigid(y,target-x@frozen,theta_g,equalities=equality,ridge=1e-4,physical_length_scale=.1,lift_residuals=True)
    radius=np.maximum(abs(frozen)*.05,np.r_[np.full(14,.0005),np.full(14,.00025)])
    lower=np.maximum(0,frozen-radius);upper=frozen+radius;upper[14:]=np.minimum(upper[14:],static-.001)
    refined=fit_rigid(y,target,np.asarray(initial['parameters']),equalities=equality,ridge=1e-4,physical_length_scale=.1,
        lift_residuals=True,auxiliary=(x,frozen,lower,upper))
    theta=np.asarray(refined['parameters']);friction=np.asarray(refined['auxiliary_parameters'])
    candidate=model_from_parameters(nominal,theta,friction[:14],friction[14:],static)
    # Selection observations are loaded only after this one predeclared R1 fit.
    selection=load_problems(root,'selection',static)
    metrics,probes=score_problems({**training,**selection},candidate,theta,friction,static,config)
    from .staged_multiconfiguration import load_development_reference
    prior_root=load_development_reference(root)
    preserved=_preserve_prior_stages(prior_root,candidate,friction,static,model_from_parameters(nominal,theta_g),
        {**config['acceptance'],'gain_nm_a':config['drive']['gain_nm_a'],'control_period_ns':config['control_period_ns']})
    repeat=batch_repeatability(metrics,config['p0']['acceptance']['repeat_residual_mean_difference_nm'])
    repeat['wrist']=wrist_repeatability(probes,config['p0']['acceptance']['repeat_residual_mean_difference_nm'])
    repeat['passed']=repeat['passed'] and repeat['wrist']['passed']
    observed_centers=measured_center_coverage(metrics,config)
    checks=dict(prior_development=preserved['passed'],all_primary_predictions=all(r['passed'] for r in metrics.values()),
        all_wrist_predictions=bool(probes and all(r['passed'] for r in probes.values())),independent_batches=repeat['passed'],measured_center_separation=observed_centers['passed'])
    output.mkdir(parents=True,exist_ok=False)
    report=dict(schema_version=1,round='R1',development_passed=all(checks.values()),passed=all(checks.values()),checks=checks,
        p0_acceptance_passed=False,formal_test_consumed=False,publishable=False,truth_read=False,
        protocol_sha256=file_hash(root/'protocol.json'),input_manifests={s:file_hash(root/f'role_inputs/{s}.json') for s in ('train','selection')},
        initial_fit=initial,joint_refinement=refined,weights=weights,validation=metrics,probe_validation=probes,
        grouped_prediction=grouped_prediction_summary(metrics,probes),
        prior_stage_acceptance=preserved,refinement_band=dict(lower=lower.tolist(),upper=upper.tolist()),
        training_trials=list(training),selection_trials=list(selection),wall_s=perf_counter()-started,
        scope='predeclared 1e-4 development fit; selection diagnostics, no R2/R3 candidate selection or final-test acceptance')
    write_json(output/'candidate_initial.json',candidate_bundle(initial['parameters'],frozen,static,nominal,source,config,initial,training))
    write_json(output/'candidate.json',candidate_bundle(theta,friction,static,nominal,source,config,refined,training))
    write_json(output/'batch_repeatability.json',repeat);write_json(output/'report.json',report)
    write_json(output/'configuration_coverage.json',dict(schema_version=1,trials={name:row['coverage'] for name,row in {**training,**selection}.items()},measured_centers=observed_centers,source='measured, not reference coverage'))
    with (output/'axis_metrics.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['trial','split','center','batch','joint','rmse_Nm','nrmse','stationary_excess_Nm','passed'])
        for name,row in metrics.items():
            for j in range(14):writer.writerow([name,row['split'],row['center_id'],row['acquisition_batch_id'],j,row['rmse_nm'][j],row['normalized_rmse'][j],row['stationary_interval_excess_nm'][j],row['passed']])
    return report
