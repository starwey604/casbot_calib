"""H3 Inertia stage: periodic multisine excitation, harmonic Fourier projection, and gravity-locked 140-param PSD fit."""
from copy import deepcopy
import csv
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import BPoly
from scipy.linalg import lstsq
import yaml
from ...data import write_json
from ...dual_excitation import periodic_reference
from ...pipeline.robot import load_robot_config
from ...pipeline.stage import Stage
from ...rigid_identification import fit_rigid, model_from_parameters
from ...shm_data import NAMES, PARTS, ROOT
from ...shm_ready import ready_position
from ...shm_reference import save_bundle
from ...staged_motion import Motion
from ...static_field import _move
from .common import (
    compose_stage_reference,
    export_stage_candidate,
    read_verified_stage_capture,
    resolve_effective_plan_and_capture,
    resolve_stage_prior,
)
from .h2_friction import _posture_center

DEFAULT_H3_CONFIG = ROOT / 'configs/dynamics/h3.yaml'


def load_h3_recipe(source=DEFAULT_H3_CONFIG, **overrides):
    if isinstance(source, dict):
        c = deepcopy(source)
    else:
        path = Path(source or DEFAULT_H3_CONFIG).resolve()
        c = yaml.safe_load(path.read_text(encoding='utf-8')) or {}

    if overrides.get('mode') is not None:
        c['mode'] = overrides['mode']
    c.setdefault('mode', 'small')
    if c['mode'] == 'smoke':
        raise ValueError('--mode smoke 仅适用于 H1 阶段（bash calib/dynamics_h1.sh --mode smoke）')
    if c['mode'] not in ('small', 'full'):
        raise ValueError(f'未知 H3 模式：{c["mode"]}（可选：small, full）')

    c.setdefault('ready_pose', True)
    c.setdefault('gravity_feedforward', True)
    c.setdefault('harmonics', 12)
    c.setdefault('phase_points', 256)
    c.setdefault('torque_floor_nm', 0.2)
    c.setdefault('ridge', 1e-4)

    harmonics = int(c['harmonics'])
    phase_points = int(c['phase_points'])
    torque_floor = float(c['torque_floor_nm'])
    ridge = float(c['ridge'])
    if harmonics < 6 or phase_points <= 2 * harmonics + 1:
        raise ValueError('H3 harmonics 须 ≥6 且 phase_points > 2*harmonics + 1')
    if not np.isfinite([torque_floor, ridge]).all() or torque_floor <= 0 or ridge <= 0:
        raise ValueError('H3 torque_floor_nm 与 ridge 须为正有限值')
    c.update(harmonics=harmonics, phase_points=phase_points, torque_floor_nm=torque_floor, ridge=ridge)

    block = dict(c.get(c['mode']) or {})
    if not block:
        raise ValueError(f'H3 配置缺少 {c["mode"]} 模式参数')

    home_deg = np.asarray(block['home_deg'], dtype=float)
    if home_deg.shape != (14,) or not np.isfinite(home_deg).all():
        raise ValueError('H3 home_deg 须为 14 轴有限角度')
    block['home_deg'] = home_deg.tolist()

    joints = list(NAMES) if block.get('joints') == 'all' else list(block.get('joints') or ())
    if not joints or len(set(joints)) != len(joints) or not set(joints) <= set(NAMES):
        raise ValueError('H3 joints 须为 all 或不重复的双臂关节名列表')
    block['joints'] = joints
    active_set = set(joints)

    raw_amp = np.broadcast_to(np.asarray(block['amplitude_deg'], dtype=float), (14,)).copy()
    if not np.isfinite(raw_amp).all():
        raise ValueError('H3 amplitude_deg 须为有限数值')
    amp_deg = np.array([raw_amp[j] if NAMES[j] in active_set else 0.0 for j in range(14)], dtype=float)
    if any(amp_deg[NAMES.index(name)] <= 0 for name in joints):
        raise ValueError('H3 被激励关节的 amplitude_deg 须为正数')
    block['amplitude_deg'] = amp_deg.tolist()

    offsets = dict(block['pose_offsets_deg'])
    for split in ('train', 'validation'):
        po = [float(x) for x in offsets.get(split, ())]
        if not po or len(set(po)) != len(po) or not np.isfinite(po).all():
            raise ValueError(f'H3 {split} 姿态偏移须为不重复有限值')
        offsets[split] = po
    if set(offsets['train']) & set(offsets['validation']):
        raise ValueError('H3 train 与 validation 姿态偏移须互不重叠')
    block['pose_offsets_deg'] = offsets

    train_seeds = [int(s) for s in block.get('train_seeds') or ()]
    val_seeds = [int(s) for s in block.get('validation_seeds') or ()]
    if not train_seeds or not val_seeds or len(set(train_seeds)) != len(train_seeds) or len(set(val_seeds)) != len(val_seeds):
        raise ValueError('H3 train_seeds 与 validation_seeds 须为非空不重复整数列表')
    if set(train_seeds) & set(val_seeds):
        raise ValueError('H3 train_seeds 与 validation_seeds 须互不重叠')
    block['train_seeds'] = train_seeds
    block['validation_seeds'] = val_seeds

    period_s = float(block['period_s'])
    periods = int(block['periods'])
    discard = int(block.get('discard_periods', 1))
    if not np.isfinite(period_s) or period_s <= 0 or discard < 0 or periods <= discard:
        raise ValueError('H3 须满足 period_s > 0 且 periods > discard_periods >= 0')
    block.update(period_s=period_s, periods=periods, discard_periods=discard)

    timing = dict(block['timing'])
    envelope = dict(block['planning_envelope'])
    max_acc = float(block['maximum_acceleration_rad_s2'])
    max_jerk = float(block['maximum_jerk_rad_s3'])
    nums = [*timing.values(), *envelope.values(), max_acc, max_jerk]
    if not np.isfinite(nums).all() or min(nums) <= 0:
        raise ValueError('H3 timing、planning_envelope 与导数上限须为正有限值')
    block.update(maximum_acceleration_rad_s2=max_acc, maximum_jerk_rad_s3=max_jerk)

    c[c['mode']] = block
    return c


def _append_enveloped_periodic_block(
    formal,
    center,
    *,
    period_ns,
    periods,
    ramp_ns,
    amplitude_rad,
    seed,
    validation,
):
    period_s = period_ns * 1e-9
    ramp_s = ramp_ns * 1e-9
    env_poly = BPoly.from_derivatives([0.0, ramp_s], [[0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]])
    step_target_ns = max(1_000_000, period_ns // 64)

    periodic_begin_ns = None
    periodic_end_ns = None

    for seg_mode, duration_ns in (('entry', ramp_ns), ('periodic', periods * period_ns), ('exit', ramp_ns)):
        if seg_mode == 'periodic':
            periodic_begin_ns = round(formal.duration * 1e9)
            count = periods * 64
        else:
            count = max(8, int(np.ceil(duration_ns / step_target_ns)))

        grid_ns = np.array([round(k * duration_ns / count) for k in range(count + 1)], dtype=np.int64)
        spans_ns = np.diff(grid_ns)
        t = grid_ns[1:] * 1e-9
        phase = t - ramp_s if seg_mode == 'entry' else t if seg_mode == 'periodic' else periods * period_s + t
        q, v, a, jrk = periodic_reference(phase, center, period_s, amplitude_rad, seed, validation, 'both')
        delta = q - center

        if seg_mode == 'periodic':
            w = np.ones((count, 1), dtype=float)
            d = dd = ddd = np.zeros((count, 1), dtype=float)
        else:
            weights = [np.asarray(env_poly(t, nu=order), dtype=float)[:, None] for order in range(4)]
            if seg_mode == 'exit':
                weights = [1.0 - weights[0], *[-arr for arr in weights[1:]]]
            w, d, dd, ddd = weights

        states = np.stack([
            center + w * delta,
            w * v + d * delta,
            w * a + 2.0 * d * v + dd * delta,
            w * jrk + 3.0 * d * a + 3.0 * dd * v + ddd * delta,
        ], axis=1)

        label = 'periodic' if seg_mode == 'periodic' else f'excitation_{seg_mode}'
        for span_ns, state in zip(spans_ns, states):
            formal.add(int(span_ns) * 1e-9, state[0], state[1], label=label)
            formal.states[-1] = state

        if seg_mode == 'periodic':
            periodic_end_ns = round(formal.duration * 1e9)

    return periodic_begin_ns, periodic_end_ns


def plan_h3(q_start, robot_config, recipe, out_dir, *, session=None):
    out_dir = Path(out_dir)
    q_start = np.asarray(q_start, dtype=float)
    if q_start.shape != (14,) or not np.isfinite(q_start).all():
        raise ValueError('起点关节角须为 14 轴有限弧度')

    prior = resolve_stage_prior(session, 'h3', robot_config)
    mode = recipe['mode']
    cfg = recipe[mode]
    timing = cfg['timing']
    envelope = cfg['planning_envelope']

    home_rad = np.deg2rad(np.asarray(cfg['home_deg'], dtype=float))
    amplitude_rad = np.deg2rad(np.asarray(cfg['amplitude_deg'], dtype=float))
    active_indices = [NAMES.index(name) for name in cfg['joints']]

    period_ns = round(float(cfg['period_s']) * 1e9)
    periods = int(cfg['periods'])
    discard_periods = int(cfg['discard_periods'])
    ramp_ns = round(float(timing['excitation_ramp_s']) * 1e9)
    trans_s = float(timing['transition_s'])

    anchor = ready_position() if recipe['ready_pose'] else q_start.copy()
    formal = Motion(f'shm_h3_inertia_{mode}', 'inertia', anchor, dict(refine_position_bounds=True))
    formal.add(float(timing['initial_hold_s']), anchor, label='initial_hold')

    blocks = []
    markers = []
    block_idx = 0

    for split, seeds in (('train', cfg['train_seeds']), ('validation', cfg['validation_seeds'])):
        for pose_idx, offset_deg in enumerate(cfg['pose_offsets_deg'][split]):
            center = _posture_center(home_rad, np.deg2rad(offset_deg), active_indices)
            for seed in seeds:
                _move(formal, center, trans_s, envelope, 'to_excitation_center')
                p_begin_ns, p_end_ns = _append_enveloped_periodic_block(
                    formal,
                    center,
                    period_ns=period_ns,
                    periods=periods,
                    ramp_ns=ramp_ns,
                    amplitude_rad=amplitude_rad,
                    seed=seed,
                    validation=(split != 'train'),
                )
                steady_begin_ns = p_begin_ns + discard_periods * period_ns
                markers.append(dict(t_ns=steady_begin_ns, event=f'h3_block_{block_idx:02d}'))
                blocks.append(dict(
                    id=f'block_{block_idx:02d}_{split}_s{seed}',
                    split=split,
                    pose_index=pose_idx,
                    pose_offset_deg=offset_deg,
                    seed=seed,
                    center_rad=center.tolist(),
                    period_ns=period_ns,
                    periods=periods,
                    discard_periods=discard_periods,
                    periodic_ns=[p_begin_ns, p_end_ns],
                    steady_ns=[steady_begin_ns, p_end_ns],
                ))
                block_idx += 1

    _move(formal, anchor, trans_s, envelope, 'return_ready')
    formal.add(float(timing['final_hold_s']), anchor, label='final_hold')

    bundle, full_motion, proof, prep_cert, offset_ns, kin_summary = compose_stage_reference(
        formal,
        q_start,
        prior['engine'],
        robot_config,
        ready_pose=bool(recipe['ready_pose']),
        gravity_feedforward=bool(recipe['gravity_feedforward']),
        markers=markers,
        max_acceleration_cap=float(cfg['maximum_acceleration_rad_s2']),
        max_jerk_cap=float(cfg['maximum_jerk_rad_s3']),
    )

    for blk in blocks:
        blk['periodic_ns'] = [blk['periodic_ns'][0] + offset_ns, blk['periodic_ns'][1] + offset_ns]
        blk['steady_ns'] = [blk['steady_ns'][0] + offset_ns, blk['steady_ns'][1] + offset_ns]

    reference_path = save_bundle(out_dir, bundle)
    plan_meta = dict(
        schema='casbot_calib_h3_plan_v1',
        stage='h3',
        mode=mode,
        ready_pose=bool(recipe['ready_pose']),
        gravity_feedforward=bool(recipe['gravity_feedforward']),
        reference_file=reference_path.name,
        duration_ns=bundle['knots_ns'][-1],
        calibration_begin_ns=offset_ns,
        start_q_rad=q_start.tolist(),
        robot_config=robot_config,
        recipe=recipe,
        prior=dict(
            source=prior['source'],
            source_path=prior['source_path'],
            parameters=prior['parameters'].tolist(),
            viscous=prior['viscous'].tolist(),
            coulomb=prior['coulomb'].tolist(),
            static_friction=prior['static_friction'].tolist(),
            smoothing_velocity=prior['smoothing_velocity'],
            stribeck_velocity=prior['stribeck_velocity'],
        ),
        active_joints=cfg['joints'],
        blocks=blocks,
        collision_certificate=dict(
            passed=True,
            minimum_distance_m=proof['minimum_distance_m'],
            clearance_m=robot_config['scene']['clearance_m'],
        ),
        kinematics_summary=kin_summary,
        preparation=prep_cert,
    )
    write_json(out_dir / 'plan.json', plan_meta)
    return plan_meta


def _group_unique_timestamps(rel_ns, q, dq, effort_nm):
    unique_ns, inverse = np.unique(rel_ns, return_inverse=True)
    counts = np.bincount(inverse)[:, None]
    q_u = np.zeros((len(unique_ns), q.shape[1]), dtype=float)
    dq_u = np.zeros_like(q_u)
    eff_u = np.zeros_like(q_u)
    np.add.at(q_u, inverse, q)
    np.add.at(dq_u, inverse, dq)
    np.add.at(eff_u, inverse, effort_nm)
    return unique_ns, q_u / counts, dq_u / counts, eff_u / counts


def _fourier_basis(tau_s, period_s, harmonics):
    omega = 2.0 * np.pi / period_s
    cols = [np.ones_like(tau_s, dtype=float)]
    for h in range(1, harmonics + 1):
        angle = h * omega * tau_s
        cols.extend((np.cos(angle), np.sin(angle)))
    return np.column_stack(cols)


def _evaluate_fourier_kinematics(q_coeffs, period_s, harmonics, t_grid):
    omega = 2.0 * np.pi / period_s
    q = np.broadcast_to(q_coeffs[0], (len(t_grid), q_coeffs.shape[1])).copy()
    dq = np.zeros_like(q)
    ddq = np.zeros_like(q)
    for h in range(1, harmonics + 1):
        hw = h * omega
        cos_a = np.cos(hw * t_grid)[:, None]
        sin_a = np.sin(hw * t_grid)[:, None]
        a_h = q_coeffs[2 * h - 1]
        b_h = q_coeffs[2 * h]
        q += cos_a * a_h + sin_a * b_h
        dq += hw * (-sin_a * a_h + cos_a * b_h)
        ddq -= (hw ** 2) * (cos_a * a_h + sin_a * b_h)
    return q, dq, ddq


def _bandlimit_fft(values, harmonics):
    n = values.shape[0]
    spec = np.fft.rfft(values, axis=0)
    spec[harmonics + 1:] = 0.0
    return np.fft.irfft(spec, n=n, axis=0)


def _gravity_constraints(prior_engine, prior_parameters):
    rng = np.random.default_rng(20261115)
    bounds = np.array([[j.lower, j.upper] for j in prior_engine.model.joints], dtype=float)
    span = bounds[:, 1] - bounds[:, 0]
    q = rng.uniform(bounds[:, 0] + 0.05 * span, bounds[:, 1] - 0.05 * span, size=(128, len(bounds)))
    zero = np.zeros_like(q)
    reg = prior_engine.regressor_batch(q, zero, zero)['regressor'].reshape(-1, len(prior_parameters))
    return (reg, reg @ prior_parameters), q


def _extract_block_harmonic_data(obs, prior_engine, blk, harmonics, phase_points):
    s_begin_ns, s_end_ns = blk['steady_ns']
    p_begin_ns = blk['periodic_ns'][0]
    period_s = blk['period_ns'] * 1e-9
    steady_span_s = (s_end_ns - s_begin_ns) * 1e-9

    mask = (obs['phase'] == 3) & (obs['relative_ns'] >= s_begin_ns) & (obs['relative_ns'] < s_end_ns)
    if int(mask.sum()) < max(32, 4 * harmonics):
        raise ValueError(f'{blk["id"]} 稳态周期有效帧数不足')
    if np.any(obs['time_ambiguous'][mask]) or len(set(obs['clock_segment'][mask].tolist())) != 1:
        raise ValueError(f'{blk["id"]} 稳态周期跨越时钟跳变或分段')

    u_ns, q_u, dq_u, eff_u = _group_unique_timestamps(
        obs['relative_ns'][mask],
        obs['q'][mask],
        obs['dq'][mask],
        obs['effort_nm'][mask],
    )
    if len(u_ns) < 2 * harmonics + 4:
        raise ValueError(f'{blk["id"]} 唯一观测时刻数不足以拟合 {harmonics} 阶谐波')
    covered_s = float((u_ns[-1] - u_ns[0]) * 1e-9)
    if covered_s < 0.85 * steady_span_s:
        raise ValueError(f'{blk["id"]} 稳态周期观测覆盖不足（{covered_s:.2f}s / {steady_span_s:.2f}s）')

    tau_s = (u_ns - p_begin_ns) * 1e-9
    lo_tau = (s_begin_ns - p_begin_ns) * 1e-9
    hi_tau = (s_end_ns - p_begin_ns) * 1e-9
    edges = np.empty(len(tau_s) + 1, dtype=float)
    edges[0] = lo_tau
    edges[-1] = hi_tau
    edges[1:-1] = 0.5 * (tau_s[:-1] + tau_s[1:])
    weights = np.maximum(np.diff(edges), 1e-9)
    weights /= weights.sum()

    phi = _fourier_basis(tau_s, period_s, harmonics)
    sqrt_w = np.sqrt(weights)[:, None]
    phi_w = sqrt_w * phi
    svals = np.linalg.svd(phi_w, compute_uv=False)
    if svals[-1] <= svals[0] * 1e-6:
        raise ValueError(f'{blk["id"]} 谐波回归基病态')

    q_coeffs, _, _, _ = lstsq(phi_w, sqrt_w * q_u)
    dq_coeffs, _, _, _ = lstsq(phi_w, sqrt_w * dq_u)
    eff_coeffs, _, _, _ = lstsq(phi_w, sqrt_w * eff_u)

    t_grid = np.arange(phase_points, dtype=float) * (period_s / phase_points)
    phi_grid = _fourier_basis(t_grid, period_s, harmonics)
    q_grid, dq_grid, ddq_grid = _evaluate_fourier_kinematics(q_coeffs, period_s, harmonics, t_grid)
    dq_meas_grid = phi_grid @ dq_coeffs
    eff_grid = phi_grid @ eff_coeffs

    # Project measured physical friction (evaluated at deduplicated samples) and
    # harmonic velocity friction consistently; using the harmonic trajectory kinematics
    # avoids 1ms encoder quantization jitter at zero crossings.
    batch = prior_engine.regressor_batch(q_grid, dq_grid, ddq_grid)
    reg_band = _bandlimit_fft(batch['regressor'], harmonics)
    rotor_band = _bandlimit_fft(batch['rotor_torque'], harmonics)
    fric_band = _bandlimit_fft(batch['friction'], harmonics)
    rigid_target = eff_grid - rotor_band - fric_band

    vel_consistency_rmse = float(np.sqrt(np.mean((dq_grid - dq_meas_grid) ** 2)))
    eff_fit_rmse = float(np.sqrt(np.mean(weights[:, None] * ((phi @ eff_coeffs - eff_u) ** 2))))
    diag = dict(
        id=blk['id'],
        split=blk['split'],
        seed=blk['seed'],
        unique_timestamps=int(len(u_ns)),
        velocity_consistency_rmse_rad_s=vel_consistency_rmse,
        effort_harmonic_fit_rmse_nm=eff_fit_rmse,
    )
    return reg_band, rigid_target, diag


def analyse_h3(plan_dir, capture_dir, out_dir, *, session=None):
    plan_dir, capture_dir = resolve_effective_plan_and_capture(plan_dir, capture_dir)
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    plan_meta = json.loads((plan_dir / 'plan.json').read_text(encoding='utf-8'))
    robot_config = load_robot_config(plan_meta['robot_config'])
    recipe = plan_meta['recipe']
    harmonics = int(recipe['harmonics'])
    phase_points = int(recipe['phase_points'])
    torque_floor = float(recipe['torque_floor_nm'])
    ridge = float(recipe['ridge'])

    obs, native, received_collision = read_verified_stage_capture(plan_meta, capture_dir, robot_config)

    nominal = resolve_stage_prior(None, 'h3', robot_config)['nominal_engine']
    prior_info = plan_meta['prior']
    prior_params = np.asarray(prior_info['parameters'], dtype=float)
    prior_engine = model_from_parameters(
        nominal,
        prior_params,
        viscous=prior_info['viscous'],
        coulomb=prior_info['coulomb'],
        static=prior_info['static_friction'],
        smoothing_velocity=float(prior_info['smoothing_velocity']),
        stribeck_velocity=float(prior_info['stribeck_velocity']),
    )

    active_indices = [NAMES.index(name) for name in plan_meta['active_joints']]
    split_blocks = {'train': [], 'validation': []}
    block_diagnostics = []

    for blk in plan_meta['blocks']:
        reg_band, rigid_target, diag = _extract_block_harmonic_data(obs, prior_engine, blk, harmonics, phase_points)
        split_blocks[blk['split']].append((reg_band[:, active_indices, :], rigid_target[:, active_indices]))
        block_diagnostics.append(diag)

    if not split_blocks['train'] or not split_blocks['validation']:
        raise ValueError('H3 需要完整的训练与留出验证周期块')

    train_reg = np.concatenate([item[0] for item in split_blocks['train']], axis=0)
    train_target = np.concatenate([item[1] for item in split_blocks['train']], axis=0)
    val_reg = np.concatenate([item[0] for item in split_blocks['validation']], axis=0)
    val_target = np.concatenate([item[1] for item in split_blocks['validation']], axis=0)

    joint_scales = np.maximum(np.sqrt(np.mean(train_target ** 2, axis=0)), torque_floor)
    weighted_reg = (train_reg / joint_scales[None, :, None]).reshape(-1, 140)
    weighted_target = (train_target / joint_scales[None, :]).ravel()

    equalities, gravity_probe_q = _gravity_constraints(prior_engine, prior_params)
    fit = fit_rigid(
        weighted_reg,
        weighted_target,
        prior_params,
        equalities=equalities,
        torque_scale=1.0,
        robust_scale=0.5,
        ridge=ridge,
        physical_length_scale=0.1,
        lift_residuals=True,
    )
    fitted_params = np.asarray(fit['parameters'], dtype=float)

    candidate_engine = model_from_parameters(
        nominal,
        fitted_params,
        viscous=prior_info['viscous'],
        coulomb=prior_info['coulomb'],
        static=prior_info['static_friction'],
        smoothing_velocity=float(prior_info['smoothing_velocity']),
        stribeck_velocity=float(prior_info['stribeck_velocity']),
    )
    max_gravity_change = float(np.max(np.abs(
        candidate_engine.gravity_batch(gravity_probe_q) - prior_engine.gravity_batch(gravity_probe_q)
    )))
    if max_gravity_change > 1e-4:
        raise RuntimeError(f'H3 拟合破坏了冻结静态重力子空间（最大偏差 {max_gravity_change:.3e} Nm）')

    prior_val_pred = val_reg @ prior_params
    cand_val_pred = val_reg @ fitted_params
    prior_axis_rmse = np.sqrt(np.mean((prior_val_pred - val_target) ** 2, axis=0))
    cand_axis_rmse = np.sqrt(np.mean((cand_val_pred - val_target) ** 2, axis=0))
    overall_prior_rmse = float(np.sqrt(np.mean((prior_val_pred - val_target) ** 2)))
    overall_val_rmse = float(np.sqrt(np.mean((cand_val_pred - val_target) ** 2)))

    exported = export_stage_candidate(out_dir, candidate_engine, robot_config)

    active_pos = {j: pos for pos, j in enumerate(active_indices)}
    axis_reports = []
    for j, name in enumerate(NAMES):
        joint_obj = candidate_engine.model.joints[j]
        inertia_diag = np.diag(joint_obj.body.inertia_com)
        if j in active_pos:
            k = active_pos[j]
            p_rmse = float(prior_axis_rmse[k])
            c_rmse = float(cand_axis_rmse[k])
            identified = True
        else:
            p_rmse = None
            c_rmse = None
            identified = False
        axis_reports.append(dict(
            joint_index=j,
            shm_id=j + 12,
            joint=name,
            identified=identified,
            mass_kg=float(joint_obj.body.mass),
            ixx_kg_m2=float(inertia_diag[0]),
            iyy_kg_m2=float(inertia_diag[1]),
            izz_kg_m2=float(inertia_diag[2]),
            min_pseudo_inertia_eig=float(fit['minimum_pseudo_inertia_eigenvalues'][j]),
            prior_validation_rmse_nm=p_rmse,
            validation_rmse_nm=c_rmse,
        ))

    calibration = dict(
        schema_version='casbot_shm_h3_calibration_v1',
        stage='h3',
        model_context=robot_config['model_context'],
        joint_names=list(NAMES),
        identified_joints=list(plan_meta['active_joints']),
        gravity_world=list(robot_config['gravity_world']),
        effort_gain=list(robot_config['effort_gain']),
        effort_zero_nm=list(robot_config['effort_zero_nm']),
        prior_source=prior_info['source'],
        prior_source_path=prior_info['source_path'],
        parameters=fitted_params.tolist(),
        viscous=list(prior_info['viscous']),
        coulomb=list(prior_info['coulomb']),
        static_friction=list(prior_info['static_friction']),
        smoothing_velocity=float(prior_info['smoothing_velocity']),
        stribeck_velocity=float(prior_info['stribeck_velocity']),
        max_gravity_change_nm=max_gravity_change,
        prior_validation_rmse_nm=overall_prior_rmse,
        validation_rmse_nm=overall_val_rmse,
        candidate_urdf=exported['candidate_urdf'],
        runtime_dynamics=exported['runtime_dynamics'],
    )
    write_json(out_dir / 'calibration.json', calibration)

    with (out_dir / 'inertia_axes.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'joint_index', 'shm_id', 'joint', 'identified',
            'mass_kg', 'ixx_kg_m2', 'iyy_kg_m2', 'izz_kg_m2',
            'min_pseudo_inertia_eig', 'prior_validation_rmse_nm', 'validation_rmse_nm',
        ])
        for row in axis_reports:
            writer.writerow([
                row['joint_index'], row['shm_id'], row['joint'], row['identified'],
                row['mass_kg'], row['ixx_kg_m2'], row['iyy_kg_m2'], row['izz_kg_m2'],
                row['min_pseudo_inertia_eig'], row['prior_validation_rmse_nm'], row['validation_rmse_nm'],
            ])

    report = dict(
        schema='casbot_calib_h3_report_v1',
        stage='h3',
        mode=plan_meta['mode'],
        prior_source=prior_info['source'],
        feedback_rows=obs['feedback_rows'],
        formal_rows=obs['formal_rows'],
        mean_feedback_hz=obs['mean_feedback_hz'],
        received_collision=dict(
            passed=True,
            minimum_distance_m=received_collision['minimum_sampled_distance_m'],
        ),
        fit_succeeded=True,
        observable_rank=fit['rank'],
        total_columns=fit['columns'],
        max_gravity_change_nm=max_gravity_change,
        prior_validation_rmse_nm=overall_prior_rmse,
        validation_rmse_nm=overall_val_rmse,
        axes=axis_reports,
        blocks=block_diagnostics,
        candidate_urdf=exported['candidate_urdf'],
        runtime_dynamics=exported['runtime_dynamics'],
    )
    write_json(out_dir / 'report.json', report)
    return report


def write_h3_summary_markdown(stage_dir, plan_meta, capture_summary, report):
    lines = [
        '# H3 动力学激励与惯量标定报告',
        '',
        f'- **模式**：`{plan_meta["mode"]}`（准备位前缀：`{"开启" if plan_meta["ready_pose"] else "关闭"}`）',
        f'- **上游先验来源**：`{plan_meta["prior"]["source"]}`',
        f'- **规划总时长**：{plan_meta["duration_ns"] * 1e-9:.2f} s（周期激励块：{len(plan_meta["blocks"])} 个）',
        f'- **参考轨迹最小碰撞间距**：{plan_meta["collision_certificate"]["minimum_distance_m"] * 1000:.2f} mm',
    ]
    if capture_summary is None or report is None:
        lines.extend(['', '> 仅生成离线规划（`--plan-only`），未连接设备。'])
    else:
        hz = f'{report["mean_feedback_hz"]:.1f} Hz' if report.get('mean_feedback_hz') else 'n/a'
        lines.extend([
            f'- **有效反馈帧数**：{report["feedback_rows"]}（均值频率 {hz}，实测最小碰撞间距 {report["received_collision"]["minimum_distance_m"] * 1000:.2f} mm）',
            f'- **PSD 惯量锥拟合**：成功（可观秩 `{report["observable_rank"]}/{report["total_columns"]}`，静态重力冻结偏差 `{report["max_gravity_change_nm"]:.2e} Nm`）',
            f'- **留出集整体动态力矩 RMSE**：上游先验 **{report["prior_validation_rmse_nm"]:.4f} Nm** → 拟合候选 **{report["validation_rmse_nm"]:.4f} Nm**',
            f'- **导出模型**：`analysis/{report["candidate_urdf"]}` + `analysis/{report["runtime_dynamics"]}`',
            '',
            '## 逐轴质心主惯量与留出集动态力矩 RMSE（Nm）',
            '',
            '| 关节 | SHM ID | Ixx (kg·m²) | Iyy (kg·m²) | Izz (kg·m²) | 先验 RMSE | 拟合后 RMSE |',
            '| --- | :---: | ---: | ---: | ---: | ---: | ---: |',
        ])
        for row in report['axes']:
            if not row['identified']:
                continue
            j = row['joint_index']
            label = ('左' if j < 7 else '右') + PARTS[j % 7]
            lines.append(
                f'| {label} (`{row["joint"]}`) | {row["shm_id"]} '
                f'| {row["ixx_kg_m2"]:.5f} | {row["iyy_kg_m2"]:.5f} | {row["izz_kg_m2"]:.5f} '
                f'| {row["prior_validation_rmse_nm"]:.4f} | {row["validation_rmse_nm"]:.4f} |'
            )

    summary_path = Path(stage_dir) / 'report.md'
    summary_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return summary_path


class H3InertiaStage(Stage):
    name = 'h3'
    title = 'H3 惯量标定'
    default_config = DEFAULT_H3_CONFIG
    implemented = True

    def load_recipe(self, config_path=None, **overrides):
        return load_h3_recipe(config_path or self.default_config, **overrides)

    def plan(self, q_start, robot_config, recipe, out_dir, *, session=None):
        return plan_h3(q_start, robot_config, recipe, out_dir, session=session)

    def analyse(self, plan_dir, capture_dir, out_dir, *, session=None):
        return analyse_h3(plan_dir, capture_dir, out_dir, session=session)

    def write_summary(self, stage_dir, plan_meta, capture_summary, report):
        return write_h3_summary_markdown(stage_dir, plan_meta, capture_summary, report)
