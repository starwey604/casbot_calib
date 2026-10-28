"""R1: resolve P0 identities into complete, continuously gated references."""
from copy import deepcopy
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from .data import write_json
from .staged_motion import build_recipes, gate_motion, derivative_bounds, periodic_motion, wrist_probe_motion
from .staged_protocol import IDENTITY, file_hash, _read, _local, load_protocol


def center_separation(first, second, amplitude, policy):
    difference = abs(np.asarray(first) - second)
    normalized = float(np.sqrt(np.mean((difference / amplitude)**2)))
    count = int(np.count_nonzero(difference >= policy['minimum_joint_difference_rad']))
    return dict(passed=normalized >= policy['minimum_normalized_rms_distance'] and count >= policy['minimum_different_joints'],
                normalized_rms_distance=normalized, different_joints=count, maximum_difference_rad=float(difference.max()))


def motion_for_slot(slot, center, config):
    conditions = {k: slot[k] for k in IDENTITY}
    conditions.update(center_rad=np.asarray(center).tolist(), phase_id=slot.get('phase_id'),
                      excitation_harmonics=slot.get('excitation_harmonics', []))
    if slot['stage'] == 'inertia_probe':
        return wrist_probe_motion(slot['trial_id'], config['home_rad'], center, config,
                                  slot['joint'], slot['direction'], conditions)
    return periodic_motion(slot['trial_id'], 'inertia', config['home_rad'], center, config,
                           slot['reference_seed'], slot['split'] != 'train', slot['active_arm_group'], conditions)


def motion_coverage(motion, config):
    curve = motion.polynomial()
    peaks = [float(np.max(derivative_bounds(curve, k))) for k in (1, 2, 3)]
    if motion.stage == 'inertia_probe':
        windows = motion.windows('probe_measure'); active = [motion.conditions['joint']]
        t = np.linspace(windows[0][0], windows[-1][1], 512, endpoint=False)
        limits = [1., 8., 200.]
    else:
        begin = motion.windows('periodic')[0][0]
        t = begin + np.arange(1024) * config['dynamic_period_s'] / 1024
        group = motion.conditions['active_arm_group']
        active = list(range(14)) if group == 'both' else list(range(7) if group == 'left' else range(7, 14))
        limits = [2., 8., 60.]
    q, v, a = (curve(t, nu=d) for d in range(3))
    travel = np.ptp(q, axis=0); speed = np.sqrt(np.mean(v*v, axis=0)); acceleration = np.sqrt(np.mean(a*a, axis=0))
    enough = motion.stage == 'inertia_probe' or (min(travel[active]) > .15 and min(speed[active]) > .01 and min(acceleration[active]) > .02)
    return dict(passed=bool(enough and all(p <= limit for p, limit in zip(peaks, limits))), active_joints=active,
        range_rad=travel.tolist(), speed_rms_rad_s=speed.tolist(), acceleration_rms_rad_s2=acceleration.tolist(),
        minimum_rad=q.min(axis=0).tolist(), maximum_rad=q.max(axis=0).tolist(),
        peak_derivative_bounds=peaks, derivative_limits=limits, source='continuous derivative bounds / reference active window; not measured coverage')


def _write_motion(root, motion, gate):
    target = root / 'plans' / motion.name; target.mkdir(parents=True, exist_ok=False)
    write_json(target / 'motion.json', motion.metadata()); write_json(target / 'collision_reference.json', gate)
    curve = motion.polynomial()
    np.savez_compressed(target / 'reference.npz', coefficients=curve.c, breakpoints=curve.x)
    write_json(target / 'manifest.json', dict(reference_gate_passed=gate['passed'],
        reference_sha256=file_hash(target / 'reference.npz'), motion_sha256=file_hash(target / 'motion.json'),
        collision_sha256=file_hash(target / 'collision_reference.json')))
    return {str(p.relative_to(root)): file_hash(p) for p in sorted(target.iterdir())}


def final_prior_recipes(config):
    """Independent final-test-only references; never collected during R1."""
    settings = config['p0']['final_prior_holdout']; c = deepcopy(config)
    c.pop('p0'); c['schema_version'] = 1
    c.update(gravity_train_poses=0, gravity_validation_poses=settings['gravity_poses'],
        gravity_candidate_multiplier=1, gravity_validation_seed=settings['gravity_reference_seed'],
        friction_poses=settings['friction_poses'], friction_validation_seed=settings['friction_reference_seed'],
        breakaway_pose_seed=settings['breakaway_reference_seed'])
    result = []
    for motion in build_recipes(c):
        if motion.stage == 'gravity' and motion.conditions['batch'] == 'validation' or motion.stage == 'friction' and motion.conditions['batch'] == 'validation' or motion.stage == 'breakaway':
            index = len(result); old = motion.conditions; motion.name = 'final_' + motion.name
            old.update(split='test', center_id=f"final_{motion.stage}_{old['pose']:02}",
                reference_seed=settings[motion.stage + '_reference_seed'],
                acquisition_batch_id='final_prior_batch_00', repeat_id=old.get('repeat', 0),
                active_arm_group='both', acquisition_seed=settings['acquisition_seed_base'] + index,
                reference_id=motion.name, purpose='final_prior_holdout')
            result.append(motion)
    expected = 2*settings['gravity_poses'] + 14*len(config['friction_speeds_rad_s'])*settings['friction_poses'] + settings['breakaway_poses']*14*2*len(config['breakaway_dwell_s'])*len(config['breakaway_slopes_nm_s'])*config['breakaway_repeats']
    if len(result) != expected:
        raise ValueError('Incomplete final predecessor holdout design')
    return result


def bind_development_reference(root, source, config, profile):
    """Pin external immutable *development* observations without duplicating GBs."""
    from .staged_identification import recordings
    source = Path(source).resolve(); session = _read(source / 'session.json')
    if session['config'].get('schema_version') != 1 or session['collision'] != profile.metadata:
        raise ValueError('Development reference must use the same nominal geometry/scene and legacy protocol')
    keys = ('model', 'control_period_ns', 'sample_period_ns', 'integration_ns', 'timing_integration_ns', 'home_rad',
            'pose_amplitude_rad', 'drive', 'acceptance', 'friction_sample_period_ns', 'friction_convergence_sample_period_ns',
            'friction_speeds_rad_s', 'inertia_sample_period_ns')
    if any(session['config'].get(k) != config.get(k) for k in keys):
        raise ValueError('Development reference acquisition/model conditions changed')
    paths = [source / 'session.json', source / 'plan_report.json']; counts = {}
    for stage in ('direction', 'timing', 'gravity', 'friction', 'breakaway'):
        counts[stage] = 0
        for name, _, _ in recordings(source, stage):
            counts[stage] += 1
            paths.extend(p for directory in ('plans', 'runs') for p in (source / directory / name).iterdir() if p.is_file())
        if not counts[stage]:
            raise ValueError('Missing development predecessor stage: ' + stage)
    required = ('direction_check.json', 'driver_timing.json', 'identification_gravity/report.json',
                'identification_gravity/gravity_constraints.npz', 'identification_friction/report.json', 'identification_breakaway/report.json')
    for name in required:
        if name.endswith('.json') and not _read(source / name)['passed']:
            raise ValueError('Development predecessor was rejected')
        paths.append(source / name)
    record = dict(schema_version=1, source=str(source), scope='reused development observations only; not fresh batches or final holdout',
        counts=counts, input_sha256={str(p.relative_to(source)): file_hash(p) for p in sorted(set(paths))})
    write_json(root / 'development_reference.json', record)
    return record


def load_development_reference(root):
    load_protocol(root); record = _read(Path(root) / 'development_reference.json'); source = Path(record['source'])
    for path, expected in record['input_sha256'].items():
        if file_hash(_local(source, path)) != expected:
            raise ValueError('Development reference changed: ' + path)
    return source


def plan_references(root, config, manifest, profile, development_reference=None):
    """Called before protocol sealing. No observed test data is used here."""
    started = perf_counter(); root = Path(root); policy = config['p0']['center_design']
    home = np.asarray(config['home_rad']); amplitude = np.asarray(config['pose_amplitude_rad'])
    rng = np.random.default_rng(policy['candidate_seed'])
    pool = home + rng.uniform(-.45, .45, (policy['candidate_pool_size'], 14)) * amplitude
    centers = {}; trials = []; rejected = []; candidates_used = set(); files = {}; coverage = []
    center_ids = list(dict.fromkeys(r['center_id'] for r in manifest['trials']))
    for center_id in center_ids:
        slots = [s for s in manifest['trials'] if s['center_id'] == center_id and s['repeat_id'] == 0]
        accepted = None
        for index, center in enumerate(pool):
            if index in candidates_used:
                continue
            separations = {name: center_separation(center, other, amplitude, policy) for name, other in centers.items()}
            if not all(r['passed'] for r in separations.values()):
                continue
            attempts = []
            for slot in slots:
                motion = motion_for_slot(slot, center, config); measured = motion_coverage(motion, config)
                gate = gate_motion(profile, motion, profile.metadata['scene']['clearance_m']) if measured['passed'] else dict(passed=False, reason='excitation_or_derivative_envelope')
                attempts.append((slot, motion, gate, measured))
                if not gate['passed']:
                    break
            if len(attempts) == len(slots) and all(r[2]['passed'] for r in attempts):
                accepted = attempts; centers[center_id] = center; candidates_used.add(index)
                break
            rejected.append(dict(center_id=center_id, pool_index=index, center_rad=center.tolist(),
                failed_trial=attempts[-1][0]['trial_id'], collision=attempts[-1][2], coverage=attempts[-1][3]))
        if accepted is None:
            write_json(root / 'planning_failure.json', dict(passed=False, failed_center=center_id, rejected=rejected))
            raise ValueError('No complete collision-free informative center: ' + center_id)
        for slot, motion, gate, measured in accepted:
            for repeat in (0, 1):
                repeated = next(s for s in manifest['trials'] if s['reference_id'] == slot['reference_id'] and s['repeat_id'] == repeat)
                m = motion_for_slot(repeated, centers[center_id], config)
                repeated_gate = {**gate, 'motion': m.name}
                files.update(_write_motion(root, m, repeated_gate))
                trials.append({**repeated, 'center_rad': centers[center_id].tolist(),
                    'reference_sha256': file_hash(root / 'plans' / m.name / 'reference.npz'), 'reference_status': 'continuous_reference_gate_passed'})
                coverage.append(dict(trial_id=m.name, **measured))
        print(f'P0 构型 {center_id}：PASS，候选 {index}，{len(accepted)} 个完整参考', flush=True)
    prior_slots = []
    for motion in final_prior_recipes(config):
        gate = gate_motion(profile, motion, profile.metadata['scene']['clearance_m'])
        files.update(_write_motion(root, motion, gate))
        if not gate['passed']:
            write_json(root / 'planning_failure.json', dict(passed=False, failed_final_prior=motion.name, collision=gate))
            raise ValueError('Final predecessor reference collision rejected: ' + motion.name)
        prior_slots.append(dict(trial_id=motion.name, stage=motion.stage, **motion.conditions))
    separation = [dict(first=a, second=b, **center_separation(centers[a], centers[b], amplitude, policy))
                  for i, a in enumerate(centers) for b in list(centers)[i+1:]]
    write_json(root / 'configuration_coverage.json', dict(schema_version=1, source='planned references only',
        centers={name: q.tolist() for name, q in centers.items()}, center_separation=separation, trials=coverage))
    report = dict(schema_version=2, protocol=manifest['protocol'], completed_round='R1', passed=True,
        contract_passed=True, p0_acceptance_passed=False, status='references_certified_no_observed_test',
        selected_trials=[r['trial_id'] for r in manifest['trials']] + [r['trial_id'] for r in prior_slots],
        periodic_trials=32, wrist_trials=56, final_prior_trials=len(prior_slots),
        total_virtual_duration_s=sum(_read(root / 'plans' / name / 'motion.json')['duration_s'] for name in
                                     [r['trial_id'] for r in manifest['trials']] + [r['trial_id'] for r in prior_slots]),
        wall_s=perf_counter()-started, rejected_center_candidates=len(rejected), hardware_executable=False)
    report['center_reference_attempts']=len(rejected)+len(centers)
    report['center_reference_rejection_rate']=len(rejected)/(len(rejected)+len(centers))
    write_json(root / 'motion_plan.json', dict(schema_version=1, report=report, trials=trials, final_prior_trials=prior_slots,
        files=files, rejected_center_candidates=rejected, planned_execution_order=manifest['planned_execution_order'],
        final_prior_execution_order=[r['trial_id'] for r in prior_slots]))
    extras = ['motion_plan.json', 'configuration_coverage.json']
    if development_reference is not None:
        bind_development_reference(root, development_reference, config, profile)
        extras.append('development_reference.json')
    return report, extras


def verify_motion_file(root, name, protocol):
    if protocol['schema_version'] < 2:
        return
    plan = _read(Path(root) / 'motion_plan.json')
    prefix = 'plans/' + name + '/'
    required = {prefix + p for p in ('motion.json', 'reference.npz', 'collision_reference.json', 'manifest.json')}
    if not required.issubset(plan['files']):
        raise ValueError('Trial is absent from the frozen R1 reference manifest')
    for path in required:
        if file_hash(_local(root, path)) != plan['files'][path]:
            raise ValueError('Frozen R1 reference changed: ' + path)


def planned_slot(root, name, manifest):
    slot = next((r for r in manifest['trials'] if r['trial_id'] == name), None)
    if slot is None and (Path(root) / 'motion_plan.json').exists():
        slot = next((r for r in _read(Path(root) / 'motion_plan.json')['final_prior_trials'] if r['trial_id'] == name), None)
    return slot


def validate_resolved_plan(plan, manifest, config):
    slots={r['trial_id']:r for r in manifest['trials']};rows=plan['trials'];centers={}
    if len(rows)!=len(slots) or {r['trial_id'] for r in rows}!=set(slots):
        raise ValueError('Incomplete R1 resolved role manifest')
    for row in rows:
        slot=slots[row['trial_id']]
        if any(row[k]!=slot[k] for k in IDENTITY):raise ValueError('Resolved role identity changed')
        q=np.asarray(row['center_rad'],dtype=float)
        if q.shape!=(14,) or not np.isfinite(q).all():raise ValueError('Invalid resolved center')
        previous=centers.setdefault(row['center_id'],q)
        if not np.array_equal(previous,q):raise ValueError('Role center changed across repetitions')
    amplitude=np.asarray(config['pose_amplitude_rad']);names=list(centers)
    if any(not center_separation(centers[a],centers[b],amplitude,config['p0']['center_design'])['passed']
           for i,a in enumerate(names) for b in names[i+1:]):raise ValueError('Resolved centers are not separated')
    final=plan['final_prior_trials'];ids=[r['trial_id'] for r in final]
    settings=config['p0']['final_prior_holdout']
    expected=dict(gravity=2*settings['gravity_poses'],friction=14*settings['friction_poses']*len(config['friction_speeds_rad_s']),
        breakaway=14*settings['breakaway_poses']*2*len(config['breakaway_dwell_s'])*len(config['breakaway_slopes_nm_s'])*config['breakaway_repeats'])
    if len(ids)!=len(set(ids)) or set(ids)&set(slots) or any(sum(r['stage']==stage for r in final)!=count for stage,count in expected.items()):
        raise ValueError('Incomplete final predecessor references')
    if any(r['split']!='test' or r['purpose']!='final_prior_holdout' for r in final):
        raise ValueError('Final predecessor role leakage')
    required={f'plans/{name}/{f}' for name in [*slots,*ids] for f in ('motion.json','reference.npz','collision_reference.json','manifest.json')}
    if set(plan['files'])!=required or plan['report']['selected_trials']!=list(slots)+ids:
        raise ValueError('Incomplete frozen reference file manifest')
