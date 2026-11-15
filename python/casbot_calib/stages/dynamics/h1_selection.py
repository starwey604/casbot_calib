"""H1 candidate screening and information-based paired-pose selection."""
from itertools import product
import numpy as np
from ...rigid_identification import parameter_scales
from ...staged_motion import Motion, gate_motion, _move

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

def paired_motions(row, config, *, split):
    home = np.asarray(config['home_rad']); q = np.asarray(row['q_rad']); timing = config['timing']
    result = []
    for sign in (-1, 1):
        near = q.copy(); near[row.get('approach_joints', row['active_joints'])] += sign * config['approach_offset_rad']
        conditions = dict(batch=split, pose=row['id'], approach=sign, active_arm_group=row['arm'],
            candidate_id=row['id'], target_q_rad=q.tolist(), planned_measure_s=timing['measure_s'])
        if 'approach_joints' in row:
            conditions['approach_joints'] = list(row['approach_joints'])
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
            x = q.copy(); x[row.get('approach_joints', row['active_joints'])] += sign * config['approach_offset_rad']; near.append(x)
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
