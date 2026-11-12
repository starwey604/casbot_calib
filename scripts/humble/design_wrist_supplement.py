#!/usr/bin/env python3
"""Train-only wrist information ablation and offline probe design; no fitting or collection."""
import argparse
from collections import defaultdict
import os
from pathlib import Path
import sys

import numpy as np
from scipy.linalg import solve

from casbot_calib.data import write_json
from casbot_calib.model_runtime import collision_for, model_for
from casbot_calib.staged_attribution import arrays, check_manifest, formal_test_untouched
from casbot_calib.staged_inertia import probe_problem
from casbot_calib.staged_motion import gate_motion, wrist_probe_motion
from casbot_calib.staged_multiconfiguration import _write_motion, motion_coverage
from casbot_calib.staged_multifit import assemble_training, load_problems
from casbot_calib.staged_protocol import _read, _unseal, file_hash, load_protocol


def protect_observations(root, train_names):
    def audit(event, args):
        if event != 'open' or not isinstance(args[0], (str, bytes, os.PathLike)):
            return
        path = Path(os.fsdecode(args[0])).resolve()
        if 'evaluation' in path.parts:
            raise RuntimeError('Plant truth is outside the train design')
        if path.is_relative_to(root / 'runs') and path.relative_to(root / 'runs').parts[:1] not in train_names:
            raise RuntimeError('Non-train observation is outside the train design')
    sys.addaudithook(audit)


def mass_rows(model, poses, joint):
    poses = np.asarray(poses); zero = np.zeros_like(poses)
    acceleration = zero.copy(); acceleration[:, joint] = 1.
    return (model.regressor_batch(poses, zero, acceleration)['regressor'][:, joint]
            - model.regressor_batch(poses, zero, zero)['regressor'][:, joint])


def goal_noise_gain(gram, goals):
    """Gain per unit weighted torque error, all other coordinates are nuisance."""
    return np.sqrt(np.sum(goals.T * solve(gram, goals.T, assume_a='pos'), axis=0))


def reference_design(motion, config, model, project, normalizer, count):
    # Use the same 30 Hz processing, trim, and downsample as acquired probes.
    step = config['inertia_sample_period_ns']
    stamps = np.arange(0, round(motion.duration * 1e9), step, dtype=np.int64)
    q, v, _, _ = motion.at(stamps * 1e-9)
    problem = probe_problem(model, dict(sample_ns=stamps, q=q, dq=v, torque_nm=np.zeros_like(q)),
                            motion.metadata(), None)
    design = np.column_stack([problem['regression'], problem['extra']]) @ project
    design *= np.sqrt(1024. / (len(design) * count)) / normalizer
    return design.T @ design


def design_supplement(root, analysis, output):
    root, analysis, output = (Path(p).resolve() for p in (root, analysis, output))
    if output.exists():
        raise FileExistsError(output)
    parent, config, manifest = load_protocol(root)
    if parent['experiment_version'] != 'synthetic_dvt2_p0_004':
        raise ValueError('Expected the sealed 004 train experiment')
    formal_test_untouched(root)
    protect_observations(root, {(r['trial_id'],) for r in manifest['trials'] if r['split'] == 'train'})
    r2 = _unseal(analysis, 'analysis_protocol.json')
    if r2['parent_protocol_sha256'] != file_hash(root / 'protocol.json') or r2['train_manifest_sha256'] != file_hash(root / 'role_inputs/train.json'):
        raise ValueError('R2 does not match the sealed train observations')
    setup = _unseal(analysis, 'training_setup.json')
    if file_hash(analysis / 'coordinates.npz') != r2['inputs']['coordinates.npz']:
        raise ValueError('Changed R2 coordinate input')
    if file_hash(analysis / 'comparison_basis.npz') != setup['files']['comparison_basis.npz']:
        raise ValueError('Changed R2 basis')
    check_manifest(analysis, 'delivery_manifest.json')

    coordinate = arrays(analysis / 'coordinates.npz')
    common = arrays(analysis / 'comparison_basis.npz')
    rank = setup['common_rank']
    # Precondition so full-train Gram is I; preserves the frozen common subspace.
    project = (coordinate['scales'][:, None] * common['basis']) / common['singular_values'][:rank]
    train = load_problems(root, 'train', coordinate['static'])
    y, x, _, weights = assemble_training(train, config)
    if weights != setup['weights']:
        raise ValueError('R2 row weights not reproduced')
    design = np.column_stack([y, x]) @ project
    del y, x
    blocks = defaultdict(lambda: np.zeros((rank, rank)))
    by_batch = defaultdict(lambda: np.zeros((rank, rank)))
    begin = 0
    for w in weights:
        end = begin + w['rows']
        a = design[begin:end]
        info = a.T @ a
        blocks[w['center_id'], w['stage'], w['joint']] += info
        by_batch[w['acquisition_batch_id'], w['stage']] += info
        begin = end
    full = sum(blocks.values())
    if np.max(abs(full - np.eye(rank))) > 1e-8:
        raise ValueError('Full-train normalization does not reproduce the sealed basis')
    centers = sorted({w['center_id'] for w in weights})
    omit = {c: sum(v for k, v in blocks.items() if k[0] != c) for c in centers}

    positions = _read(root / 'configuration_coverage.json')['centers']
    poses = {c: np.asarray(positions[c]) for c in centers}
    model, _ = model_for(config)
    j = 4
    # One functional at each train center and a crossed left pitch/roll grid.
    grid = []
    for pitch in (-.3, 0., .3):
        for roll in (-.3, 0., .3):
            q = poses['train_c02'].copy()
            q[5:7] = (pitch, roll)
            grid.append(q)
    grid = np.array(grid)
    raw = mass_rows(model, grid, j)
    full_candidate = arrays(analysis / 'variants/ridge_0.0001_full/solution.npz')['vector']
    dropped = arrays(analysis / 'variants/ridge_0.0001_omit_train_c02/solution.npz')['vector']
    from casbot_calib.rigid_identification import model_from_parameters
    zero = np.zeros_like(grid)
    def mass_diagonal(candidate):
        fitted = model_from_parameters(model, candidate[:140], candidate[140:154], candidate[154:], coordinate['static'])
        return fitted.evaluate_batch(grid, zero, zero)['mass_matrix'][:, j, j]
    inertia = mass_diagonal(full_candidate)
    predicted_change = raw @ (dropped - full_candidate)[:140]
    native_error = float(np.max(abs(predicted_change - (mass_diagonal(dropped) - inertia))))
    if native_error > 1e-12:
        raise ValueError('Mass functional does not reproduce the native candidate difference')
    goals = raw @ project[:140] / inertia[:, None]
    shift = predicted_change / inertia
    roll_delta = abs(shift.reshape(3, 3)[:, 2] - shift.reshape(3, 3)[:, 0])
    # Use all nine goals for design; the c02 held wrist point is used only for attribution.
    index = 5  # pitch 0, roll +0.3 rad
    baseline = goal_noise_gain(full, goals)
    missing = {c: goal_noise_gain(g, goals) / baseline for c, g in omit.items()}
    weak = solve(omit['train_c02'], goals[index], assume_a='pos')
    energy = {k: float(weak @ v @ weak) for k, v in blocks.items()}
    omitted_energy = sum(v for k, v in energy.items() if k[0] == 'train_c02')
    ablation = {}
    for center in centers:
        restored = {k: v for k, v in blocks.items() if k[0] == center}
        base = omit[center]
        ablation[center] = dict(
            periodic_only=(goal_noise_gain(base + sum(v for k, v in restored.items() if k[1] == 'inertia'), goals) / baseline).tolist(),
            left_probe_only=(goal_noise_gain(base + restored[center, 'inertia_probe', j], goals) / baseline).tolist(),
            all_but_left_probe=(goal_noise_gain(full - restored[center, 'inertia_probe', j], goals) / baseline).tolist(),
        )
    swaps = []
    for pose in poses.values():
        trial = grid.copy()
        trial[:, :5] = pose[:5]
        trial[:, 7:] = pose[7:]
        swaps.append(float(np.max(abs(mass_rows(model, trial, j) - raw))))

    normalizers = [w['torque_normalizer_nm'] for w in weights if w['stage'] == 'inertia_probe' and w['joint'] == j]
    count = sum(w['stage'] == 'inertia_probe' for w in weights)
    normalizer = float(np.median(normalizers))
    batches = sorted({w['acquisition_batch_id'] for w in weights})
    new_records = 2 * 2 * len(batches)  # two centers, both signs, in every batch
    probe_scale = count / (count + new_records)
    # The full-train objective splits the probe budget evenly over all old and
    # new probes. Match that normalization; retain the old comparison basis.
    adjusted_omit = {c: sum(v * (probe_scale if k[1] == 'inertia_probe' else 1.)
                            for k, v in blocks.items() if k[0] != c) for c in centers}
    collision = collision_for(config)
    options = {}
    # Cross the wrist geometry while keeping each proximal configuration and all other axes.
    for c in centers[:2]:
        for p, r in grid[:, 5:7]:
            key = f'{c}_p{p:+.2f}_r{r:+.2f}'
            pose = poses[c].copy(); pose[5:7] = (p, r)
            motions = [wrist_probe_motion(f'{key}_{sign:+d}', config['home_rad'], pose, config, j, sign)
                       for sign in (-1, 1)]
            info = sum(reference_design(m, config, model, project, normalizer, count) for m in motions)
            options[key] = dict(center=c, pose=pose.tolist(), info=info, motions=motions)

    def evaluate(keys):
        # Leave each center out, including its own added pair. Never credit a removed group's rows.
        augmented = {c: goal_noise_gain(g + sum(options[k]['info'] * (probe_scale * len(batches))
                       for k in keys if options[k]['center'] != c), goals) / baseline
                     for c, g in adjusted_omit.items()}
        return max(float(a.max()) for a in augmented.values()), {k: v.tolist() for k, v in augmented.items()}

    comparisons = []
    for a in options:
        for b in options:
            if options[a]['center'] != centers[0] or options[b]['center'] != centers[1]:
                continue
            score, by_center = evaluate((a, b))
            comparisons.append(dict(pairs=[a, b], worst_gain_ratio=score, by_omitted_center=by_center))
    comparisons.sort(key=lambda row: row['worst_gain_ratio'])
    gate_cache = {}
    for candidate in comparisons:
        for key in candidate['pairs']:
            if key not in gate_cache:
                results = [(gate_motion(collision, m, collision.metadata['scene']['clearance_m']), motion_coverage(m, config))
                           for m in options[key]['motions']]
                gate_cache[key] = results
        if all(g['passed'] and c['passed'] for key in candidate['pairs'] for g, c in gate_cache[key]):
            selected = candidate
            break
    else:
        raise ValueError('No crossed probe pairs passed the existing continuous gates')

    augmented_batch_gains = {}
    for omitted in batches:
        retained = sum(g for k, g in by_batch.items() if k[0] != omitted)
        adjusted = sum(g * (probe_scale if k[1] == 'inertia_probe' else 1.)
                       for k, g in by_batch.items() if k[0] != omitted)
        added = sum(options[k]['info'] for k in selected['pairs']) * (probe_scale * (len(batches) - 1))
        augmented_batch_gains[omitted] = dict(
            before_maximum_ratio=float(np.max(goal_noise_gain(retained, goals) / baseline)),
            proposed_maximum_ratio=float(np.max(goal_noise_gain(adjusted + added, goals) / baseline)))

    output.mkdir(parents=True, exist_ok=False)
    for key in selected['pairs']:
        for motion, (gate, coverage) in zip(options[key]['motions'], gate_cache[key]):
            _write_motion(output, motion, gate)
            write_json(output / 'plans' / motion.name / 'coverage.json', coverage)
    report = dict(
        scope='004 sealed train only; data-only linear information in frozen gravity-constrained R2 observable subspace; no fitting, acquisition or real hardware',
        root=str(root), analysis=str(analysis), common_rank=rank, joint=model.model.joints[j].name,
        goal_grid_pitch_roll_rad=grid[:, 5:7].tolist(), same_ridge_c02_omit_relative_mass_change=shift.tolist(),
        roll_flip_changes_relative_mass_shift=roll_delta.tolist(), max_proximal_swap_mass_row_change=max(swaps), max_native_mass_change_error_kg_m2=native_error,
        full_goal_noise_gain=baseline.tolist(), baseline_leave_center_ratios={c: a.tolist() for c, a in missing.items()},
        ablation=ablation, c02_target_index=index,
        c02_target_weak_direction_energy_by_block=[dict(center=k[0], stage=k[1], joint=k[2], energy=v) for k, v in sorted(energy.items())],
        c02_target_energy_share_in_c02=float(omitted_energy / sum(energy.values())),
        c02_left_probe_share_of_c02_target_energy=float(energy['train_c02', 'inertia_probe', j] / omitted_energy),
        c02_left_pitch_periodic_share_of_c02_target_energy=float(energy['train_c02', 'inertia', 5] / omitted_energy),
        frozen_probe_normalizer_nm=normalizer, planned_records_per_pair_per_batch=2, acquisition_batches=batches,
        added_train_records=new_records, old_probe_squared_weight_multiplier=probe_scale,
        leave_batch_check=augmented_batch_gains,
        proposal=selected, selected_poses={k: options[k]['pose'] for k in selected['pairs']},
        unchanged_wrist_pose_control=dict(pairs=[
            f'{c}_p{poses[c][5]:+.2f}_r{poses[c][6]:+.2f}' for c in centers[:2]],
            worst_gain_ratio=evaluate(tuple(f'{c}_p{poses[c][5]:+.2f}_r{poses[c][6]:+.2f}' for c in centers[:2]))[0]),
        screened_combinations=comparisons, checked_geometry={
            k: [dict(passed=g['passed'], minimum_sampled_distance_m=g['minimum_distance_m'], derivative_bounds=c['peak_derivative_bounds'])
                for g, c in rows] for k, rows in gate_cache.items()},
        selection_observations_used=False, formal_test_consumed=False, hardware_executable=False)
    write_json(output / 'report.json', report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--analysis', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = design_supplement(args.root, args.analysis, args.output)
    print(result['proposal'])
