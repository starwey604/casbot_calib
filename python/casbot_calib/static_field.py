"""Offline field gravity preset; Pinocchio information and Coal reference gates.

This designs references and roles, never samples a device or qualifies feedback.
Every selected target has two approaches and independent home-to-home trials.
"""
import csv
from itertools import product
import json
from pathlib import Path
from time import perf_counter
import numpy as np
import yaml
from .collision import nominal_profile
from .data import write_json
from .dual_model import nominal_dual_arm
from .rigid_identification import parameter_scales
from .staged_motion import Motion, derivative_bounds, gate_motion
from .vendor_identity import sha256


def validate_config(config, engine, source):
    if config.get('schema_version') != 'casbot_static_field_v1' or config.get('mode') != 'offline_plan_only':
        raise ValueError('Static field preset is offline_plan_only')
    if config.get('model') != engine.model.model_id or config.get('source_model_sha256') != source['sha256']:
        raise ValueError('Static preset model identity mismatch')
    home = np.asarray(config['home_rad'], dtype=float)
    joints = engine.model.joints
    if home.shape != (14,) or not np.isfinite(home).all() or any(not j.lower <= q <= j.upper for j, q in zip(joints, home)):
        raise ValueError('Finite in-limit 14-axis home required')
    if type(config.get('command_period_ns')) is not int or config['command_period_ns'] <= 0:
        raise ValueError('Integer positive command period required')
    if set(config['arms']) != {'left', 'right'}:
        raise ValueError('Explicit left and right arm mappings required')
    for arm, mapping in config['arms'].items():
        expected = {j.name for j in joints if j.name.startswith(arm + '_')}
        single = mapping['single_axis_joints']; combo = mapping['combination_joints']
        if len(single) != 7 or set(single) != expected or not combo or len(set(combo)) != len(combo) or not set(combo) <= expected:
            raise ValueError('Named seven-axis scan and unique same-arm combination joints required')
    for key in ('single_axis_offsets_deg', 'combination_offsets_deg'):
        values = np.asarray(config[key], dtype=float)
        if values.ndim != 1 or not len(values) or not np.isfinite(values).all() or len(set(values)) != len(values):
            raise ValueError('Finite distinct scan offsets required')
    if any(not np.isfinite(v) or v <= 0 for v in [config['approach_offset_rad'], *config['timing'].values(), *config['planning_envelope'].values()]):
        raise ValueError('Positive finite timing, approach and planning envelope required')
    for key in ('train_poses_per_arm', 'validation_poses_per_arm'):
        if type(config['selection'][key]) is not int or config['selection'][key] <= 0:
            raise ValueError('Positive integer pose budgets required')
    if config['selection']['train_poses_per_arm'] < 14:
        raise ValueError('Training budget must allow fourteen signed single-axis diagnostics')
    for key in ('information_ridge', 'rank_rtol'):
        if not np.isfinite(config['selection'][key]) or config['selection'][key] <= 0:
            raise ValueError('Positive information/rank thresholds required')
    if type(config['selection']['validation_seed']) is not int or config['selection']['validation_seed'] < 0:
        raise ValueError('Nonnegative integer validation seed required')
    raw_count = sum(len(m['single_axis_joints']) * len(config['single_axis_offsets_deg']) +
        len(config['combination_offsets_deg']) ** len(m['combination_joints']) for m in config['arms'].values())
    if raw_count > 20000:
        raise ValueError('Candidate grid too large for this field preset')


def candidate_pool(config, engine):
    """Deduplicate neutral/overlapping targets without changing requested angles."""
    names = [j.name for j in engine.model.joints]; home = np.asarray(config['home_rad'])
    result = []; raw_count = 0
    for arm, mapping in config['arms'].items():
        unique = {}
        scans = []
        for name in mapping['single_axis_joints']:
            scans.extend(('single_axis', [name], [angle]) for angle in config['single_axis_offsets_deg'])
        scans.extend(('combination', mapping['combination_joints'], angles)
            for angles in product(config['combination_offsets_deg'], repeat=len(mapping['combination_joints'])))
        for family, active, angles in scans:
            raw_count += 1; q = home.copy(); indices = [names.index(n) for n in active]
            q[indices] += np.deg2rad(angles)
            key = tuple(q)
            descriptor = dict(family=family, joints=active, offsets_deg=list(angles))
            if key in unique:
                unique[key]['aliases'].append(descriptor)
                continue
            row = dict(id=f'{arm}_{len(unique):03}', arm=arm, q_rad=q.tolist(),
                family=family, active_joints=indices, offsets_deg=list(angles), aliases=[descriptor],
                status='unreviewed', split=None)
            unique[key] = row; result.append(row)
    return result, raw_count


def _move(motion, target, minimum, envelope, label):
    unit = Motion('unit', 'gravity', motion.states[-1][0]).add(1., target)
    curve = unit.polynomial()
    peaks = [float(np.max(derivative_bounds(curve, order))) for order in (1, 2, 3)]
    limits = [envelope[k] for k in ('speed_rad_s', 'acceleration_rad_s2', 'jerk_rad_s3')]
    duration = max(minimum, *(float((p / limit) ** (1 / order)) for order, (p, limit) in enumerate(zip(peaks, limits), 1)))
    motion.add(duration * (1 + 1e-10), target, label=label)


def paired_motions(row, config, *, split):
    home = np.asarray(config['home_rad']); q = np.asarray(row['q_rad']); timing = config['timing']
    result = []
    for sign in (-1, 1):
        near = q.copy(); near[row['active_joints']] += sign * config['approach_offset_rad']
        conditions = dict(batch=split, pose=row['id'], approach=sign, active_arm_group=row['arm'],
            candidate_id=row['id'], target_q_rad=q.tolist(), planned_measure_s=timing['measure_s'])
        motion = Motion(f"static_{split}_{row['id']}_{'minus' if sign < 0 else 'plus'}", 'gravity', home, conditions)
        motion.add(timing['initial_hold_s'], home, label='initial_hold')
        _move(motion, near, timing['minimum_transition_s'], config['planning_envelope'], 'approach')
        _move(motion, q, timing['minimum_paired_approach_s'], config['planning_envelope'], 'paired_approach')
        motion.add(timing['settle_s'], q, label='settle')
        motion.add(timing['measure_s'], q, label='static_measure')
        _move(motion, home, timing['minimum_transition_s'], config['planning_envelope'], 'return_home')
        motion.add(timing['final_hold_s'], home, label='final_hold')
        result.append(motion)
    return result


def screen_candidates(rows, config, profile, margin):
    """Endpoint screening only; selected paths must still pass gate_motion."""
    home = np.asarray(config['home_rad']); joints = profile.engine.model.joints
    limits = np.array([[j.lower, j.upper] for j in joints]); pending = []; points = []
    for row in rows:
        q = np.asarray(row['q_rad']); near = []
        for sign in (-1, 1):
            x = q.copy(); x[row['active_joints']] += sign * config['approach_offset_rad']; near.append(x)
        endpoints = np.array([q, *near])
        if np.array_equal(q, home):
            row.update(status='baseline_only', reason='neutral target not assigned to both data roles')
        elif np.any(endpoints < limits[:, 0]) or np.any(endpoints > limits[:, 1]):
            row.update(status='rejected', reason='target_or_paired_approach_outside_urdf_travel')
        else:
            pending.append(row); points.extend(endpoints)
    if not points:
        return
    distances = profile.check(np.array(points))['distances'].reshape(len(pending), 3, -1)
    for row, values in zip(pending, distances):
        minimum = float(values.min())
        row.update(minimum_endpoint_distance_m=minimum, endpoint_gate_only=True)
        row.update(status='endpoint_admissible' if minimum > margin else 'rejected',
            reason='selected_paths_still_require_continuous_gate' if minimum > margin else 'endpoint_collision_or_clearance')


def information(regressors, scales, rank_rtol):
    matrix = np.asarray(regressors).reshape(-1, len(scales)) * scales
    _, singular, vh = np.linalg.svd(matrix, full_matrices=False)
    rank = int(np.count_nonzero(singular > max(float(singular[0]) * rank_rtol, 1e-12))) if len(singular) else 0
    return dict(rank=rank, columns=len(scales), singular_values=singular.tolist(),
        observable_condition_number=float(singular[0] / singular[rank - 1]) if rank else None,
        unique_link_mass_com_claimed=False), vh[:rank].T


def _select_arm(rows, config, engine, profile, margin):
    settings = config['selection']; arm = rows[0]['arm']
    active = [k for k, j in enumerate(engine.model.joints) if j.name.startswith(arm + '_')]
    pool = [r for r in rows if r['status'] == 'endpoint_admissible']
    if not pool:
        return [], [], dict(passed=False, reason='no_endpoint_admissible_candidates')
    q = np.array([r['q_rad'] for r in pool]); zero = np.zeros_like(q)
    regressors = engine.regressor_batch(q, zero, zero)['regressor'][:, active, :]
    scales = parameter_scales(engine.rigid_parameters())
    pool_info, basis = information(regressors, scales, settings['rank_rtol'])
    blocks = (regressors * scales) @ basis
    grams = np.swapaxes(blocks, 1, 2) @ blocks
    current = np.eye(pool_info['rank']) * settings['information_ridge']
    training = []; validation = []; plans = []; rejected = []; missing = []

    def accept(index, split):
        row = pool[index]; motions = paired_motions(row, config, split=split)
        gates = [gate_motion(profile, m, margin) for m in motions]
        if not all(g['passed'] for g in gates):
            row.update(status='rejected', reason='continuous_reference_rejected', path_gates=gates)
            rejected.append(row['id']); return False
        row.update(status='selected', split=split, endpoint_gate_only=False,
            trial_ids=[m.name for m in motions])
        plans.extend(zip(motions, gates)); (training if split == 'train' else validation).append(index)
        return True

    # Preserve signed single-axis diagnostics before information selection.
    for joint in active:
        for sign in (-1, 1):
            options = [i for i, r in enumerate(pool) if r['family'] == 'single_axis' and
                r['active_joints'] == [joint] and np.sign(r['offsets_deg'][0]) == sign]
            options.sort(key=lambda i: (-abs(pool[i]['offsets_deg'][0]), pool[i]['id']))
            if not any(accept(i, 'train') for i in options):
                missing.append(dict(joint=engine.model.joints[joint].name, direction=sign))
    for index in training:
        current += grams[index]
    # Holdouts use reference geometry only; no measured data or final P0 test.
    # Include remaining single-axis points: five-axis combinations alone omit
    # the wrist-roll observable direction, despite appearing to cover space.
    rng = np.random.default_rng(settings['validation_seed'] + (0 if arm == 'left' else 1))
    validation_order = rng.permutation(len(pool)); validation_information = np.eye(pool_info['rank']) * settings['information_ridge']
    while len(validation) < settings['validation_poses_per_arm']:
        indices = [int(i) for i in validation_order if pool[i]['status'] == 'endpoint_admissible']
        if not indices:
            break
        _, scores = np.linalg.slogdet(validation_information[None, :, :] + grams[indices])
        index = indices[int(np.argmax(scores))]
        if accept(index, 'validation'):
            validation_information += grams[index]
    while len(training) < settings['train_poses_per_arm']:
        indices = [i for i, r in enumerate(pool) if r['status'] == 'endpoint_admissible']
        if not indices:
            break
        _, scores = np.linalg.slogdet(current[None, :, :] + grams[indices])
        index = indices[int(np.argmax(scores))]
        if accept(index, 'train'):
            current += grams[index]
    train_info, _ = information(regressors[training], scales, settings['rank_rtol'])
    validation_info, _ = information(regressors[validation], scales, settings['rank_rtol'])
    passed = bool(not missing and len(training) == settings['train_poses_per_arm'] and
        len(validation) == settings['validation_poses_per_arm'] and
        train_info['rank'] == validation_info['rank'] == pool_info['rank'])
    return plans, [pool[i] for i in [*training, *validation]], dict(passed=passed,
        endpoint_pool_information=pool_info, training_information=train_info, validation_information=validation_info,
        missing_signed_diagnostics=missing, continuous_path_rejections=rejected,
        training_poses=len(training), validation_poses=len(validation),
        selection_method='signed single-axis diagnostics, disjoint reference-only D-optimal holdouts with seeded tie order, scaled D-optimal training')


def plan_static_field(config_path, output):
    started = perf_counter(); config_path = Path(config_path).resolve()
    config = yaml.safe_load(config_path.read_text(encoding='utf-8')); engine, source = nominal_dual_arm()
    validate_config(config, engine, source)
    scene_path = (config_path.parent / config['collision_scene']).resolve()
    scene = yaml.safe_load(scene_path.read_text(encoding='utf-8'))
    profile = nominal_profile(engine, scene); margin = float(scene['clearance_m'])
    home_gate = gate_motion(profile, Motion('home', 'gravity', config['home_rad']).add(1., config['home_rad']), margin)
    if not home_gate['passed']:
        raise ValueError('Configured home fails nominal collision gate')
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    write_json(output / 'config.json', config); write_json(output / 'scene.json', scene)
    rows, raw_count = candidate_pool(config, engine); screen_candidates(rows, config, profile, margin)
    selected = []; plans = []; arms = {}
    for arm in ('left', 'right'):
        motions, poses, report = _select_arm([r for r in rows if r['arm'] == arm], config, engine, profile, margin)
        selected.extend(poses); plans.extend(motions); arms[arm] = report
    for motion, gate in plans:
        folder = output / 'plans' / motion.name; folder.mkdir(parents=True, exist_ok=False)
        write_json(folder / 'motion.json', motion.metadata()); write_json(folder / 'collision_reference.json', gate)
        curve = motion.polynomial()
        np.savez_compressed(folder / 'reference.npz', coefficients=curve.c, breakpoints=curve.x)
    write_json(output / 'candidates.json', rows)
    names = [j.name for j in engine.model.joints]
    with (output / 'poses.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream); writer.writerow(['candidate_id', 'arm', 'split', 'family', *names])
        writer.writerows([r['id'], r['arm'], r['split'], r['family'], *r['q_rad']] for r in selected)
    q = np.array([r['q_rad'] for r in selected]).reshape(-1, 14); zero = np.zeros_like(q)
    regressors = engine.regressor_batch(q, zero, zero)['regressor'] if len(q) else np.empty((0, 14, 140))
    np.savez_compressed(output / 'design.npz', q=q, regressor=regressors,
        nominal_gravity_nm=engine.gravity_batch(q) if len(q) else np.empty((0, 14)),
        parameter_scales=parameter_scales(engine.rigid_parameters()))
    report = dict(schema_version='casbot_static_field_plan_v1', passed=all(a['passed'] for a in arms.values()),
        hardware_executable=False, observations_qualified=False, parameters_identified=False,
        model_id=engine.model.model_id, source_model_sha256=source['sha256'], joint_names=names,
        config_source_sha256=sha256(config_path), scene_source_sha256=sha256(scene_path), collision=profile.metadata,
        command_period_ns=config['command_period_ns'], home_q_rad=config['home_rad'], home_collision=home_gate,
        raw_candidate_requests=raw_count, unique_candidates=len(rows), selected_poses=len(selected),
        selected_trials=len(plans), arms=arms, planned_execution_order=[m.name for m, _ in plans],
        planned_total_s=sum(m.duration for m, _ in plans),
        planned_arm_s={arm: sum(m.duration for m, _ in plans if m.conditions['active_arm_group'] == arm) for arm in arms},
        planned_measurement_s=sum(config['timing']['measure_s'] for _ in plans),
        candidate_status_counts={key: sum(r['status'] == key for r in rows) for key in sorted({r['status'] for r in rows})},
        units=dict(q='rad', offsets='deg relative to configured home', time='s', torque='Nm nominal prediction'),
        wall_s=perf_counter() - started,
        limitations=['nominal geometry and unmeasured fixture proxy only; no hardware execution',
            'selected paths certified; unselected endpoint-admissible poses have no continuous path certificate',
            'static gravity identifies parameter combinations, not every link mass/COM or rotational inertia',
            'opposite approaches bracket stiction, not guaranteed cancellation',
            'publish stamp and repeated cache do not qualify as independent acquisition samples',
            'planned 1 s windows are not acquired measurements; S3/S4 qualification remains required',
            'not a new independent final P0 test or a control performance validation'])
    write_json(output / 'report.json', report)
    write_json(output / 'manifest.json', dict(schema_version='casbot_static_field_manifest_v1',
        files={str(p.relative_to(output)): sha256(p) for p in sorted(output.rglob('*')) if p.is_file()}))
    return report


def verify_static_field(folder, output):
    """Recompute nominal references/regressors from the frozen, portable bundle."""
    root = Path(folder); output = Path(output)
    if root.resolve() == output.resolve() or root.resolve() in output.resolve().parents:
        raise ValueError('Verification output must be outside the frozen static plan')
    manifest = json.loads((root / 'manifest.json').read_text())
    if manifest.get('schema_version') != 'casbot_static_field_manifest_v1':
        raise ValueError('Unknown static plan manifest')
    actual = {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p != root / 'manifest.json'}
    if actual != set(manifest['files']):
        raise ValueError('Static plan manifest coverage mismatch')
    for name, digest in manifest['files'].items():
        if sha256(root / name) != digest:
            raise ValueError('Static plan input changed: ' + name)
    config = json.loads((root / 'config.json').read_text()); report = json.loads((root / 'report.json').read_text())
    scene = json.loads((root / 'scene.json').read_text()); engine, source = nominal_dual_arm()
    validate_config(config, engine, source); profile = nominal_profile(engine, scene)
    if report['hardware_executable'] or report['observations_qualified'] or report['parameters_identified']:
        raise ValueError('A static reference plan cannot qualify hardware or observations')
    rows = json.loads((root / 'candidates.json').read_text()); selected = [r for r in rows if r['status'] == 'selected']
    expected_rows, raw_count = candidate_pool(config, engine)
    if len(rows) != len(expected_rows) or report['raw_candidate_requests'] != raw_count or report['unique_candidates'] != len(rows):
        raise ValueError('Static candidate coverage mismatch')
    for row, expected in zip(rows, expected_rows):
        if any(row[k] != expected[k] for k in ('id', 'q_rad', 'arm', 'family', 'active_joints', 'offsets_deg', 'aliases')):
            raise ValueError('Static candidate differs from configured scan')
    if report['source_model_sha256'] != source['sha256'] or report['joint_names'] != [j.name for j in engine.model.joints]:
        raise ValueError('Static report model identity mismatch')
    if report['selected_poses'] != len(selected) or report['selected_trials'] != len(report['planned_execution_order']):
        raise ValueError('Static selected counts mismatch')
    arm_checks = {}
    for arm in ('left', 'right'):
        active = [k for k, j in enumerate(engine.model.joints) if j.name.startswith(arm + '_')]
        train = [r for r in selected if r['arm'] == arm and r['split'] == 'train']
        validation = [r for r in selected if r['arm'] == arm and r['split'] == 'validation']
        signs = {(r['active_joints'][0], int(np.sign(r['offsets_deg'][0]))) for r in train if r['family'] == 'single_axis'}
        if len(train) != config['selection']['train_poses_per_arm'] or len(validation) != config['selection']['validation_poses_per_arm'] or not {(j, s) for j in active for s in (-1, 1)} <= signs:
            raise ValueError('Static role budget or signed diagnostic coverage mismatch')
        for split, subset in [('training', train), ('validation', validation)]:
            q = np.array([r['q_rad'] for r in subset]); z = np.zeros_like(q)
            block = engine.regressor_batch(q, z, z)['regressor'][:, active, :]
            info, _ = information(block, parameter_scales(engine.rigid_parameters()), config['selection']['rank_rtol'])
            expected = report['arms'][arm][split + '_information']
            if info['rank'] != expected['rank'] or not np.allclose(info['singular_values'], expected['singular_values'], atol=1e-10, rtol=1e-10):
                raise ValueError('Static information report mismatch')
            if info['rank'] != report['arms'][arm]['endpoint_pool_information']['rank']:
                raise ValueError('Static role does not cover the declared observable directions')
        arm_checks[arm] = dict(training_poses=len(train), validation_poses=len(validation), signed_diagnostics=len(signs))
    by_id = {r['id']: r for r in selected}; checks = []; order = []; duration = 0.
    for candidate_id in [json.loads((root / 'plans' / name / 'motion.json').read_text())['conditions']['candidate_id']
            for name in report['planned_execution_order'][::2]]:
        row = by_id[candidate_id]
        for motion in paired_motions(row, config, split=row['split']):
            target = root / 'plans' / motion.name
            if json.loads((target / 'motion.json').read_text()) != motion.metadata():
                raise ValueError('Static motion differs from candidate/config')
            with np.load(target / 'reference.npz', allow_pickle=False) as data:
                curve = motion.polynomial()
                if not np.array_equal(data['coefficients'], curve.c) or not np.array_equal(data['breakpoints'], curve.x):
                    raise ValueError('Static reference polynomial mismatch')
            gate = gate_motion(profile, motion, scene['clearance_m'])
            checks.append(gate['passed']); order.append(motion.name); duration += motion.duration
    if order != report['planned_execution_order'] or len(order) != 2 * len(selected) or len(set(order)) != len(order):
        raise ValueError('Static execution order or approach-pair coverage mismatch')
    if not np.isclose(duration, report['planned_total_s'], rtol=0, atol=1e-8):
        raise ValueError('Static duration changed')
    with np.load(root / 'design.npz', allow_pickle=False) as data:
        zero = np.zeros_like(data['q']); computed = engine.regressor_batch(data['q'], zero, zero)['regressor']
        error = float(np.max(abs(computed - data['regressor']), initial=0))
        gravity_error = float(np.max(abs(engine.gravity_batch(data['q']) - data['nominal_gravity_nm']), initial=0))
        expected_q = {tuple(r['q_rad']) for r in selected}
        if len(expected_q) != len(selected) or {tuple(q) for q in data['q']} != expected_q:
            raise ValueError('Duplicate roles/targets or design target mismatch')
    result = dict(passed=bool(report['passed'] and all(checks) and error < 1e-10 and gravity_error < 1e-10),
        hardware_executable=False, source_manifest_sha256=sha256(root / 'manifest.json'),
        reference_trials_rechecked=len(checks), maximum_regressor_error=error, maximum_gravity_error_nm=gravity_error,
        planned_total_s=duration, arms=arm_checks)
    output.mkdir(parents=True, exist_ok=False); write_json(output / 'report.json', result)
    return result
