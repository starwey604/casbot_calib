"""H2 Friction stage: constant-velocity bidirectional plateaus + PD onset, chained automatically from H1 gravity."""
from copy import deepcopy
import csv
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
import yaml
from ...data import write_json
from ...pipeline.robot import load_robot_config
from ...pipeline.stage import Stage
from ...rigid_identification import model_from_parameters
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

DEFAULT_H2_CONFIG = ROOT / 'configs/dynamics/h2.yaml'
POSTURE_DIRECTION = np.array([0.5, 0.2, 0.2, -0.5, 0.2, 0.2, 0.2, 0.5, -0.2, -0.2, -0.5, -0.2, 0.2, -0.2])
MIRROR_SIGN_RIGHT = {7: 1.0, 8: -1.0, 9: -1.0, 10: 1.0, 11: -1.0, 12: 1.0, 13: -1.0}


def load_h2_recipe(source=DEFAULT_H2_CONFIG, **overrides):
    if isinstance(source, dict):
        c = deepcopy(source)
    else:
        path = Path(source or DEFAULT_H2_CONFIG).resolve()
        c = yaml.safe_load(path.read_text(encoding='utf-8')) or {}

    if overrides.get('mode') is not None:
        c['mode'] = overrides['mode']
    c.setdefault('mode', 'small')
    if c['mode'] == 'smoke':
        raise ValueError('--mode smoke 仅适用于 H1 阶段（bash calib/dynamics_h1.sh --mode smoke）')
    if c['mode'] not in ('small', 'full'):
        raise ValueError(f'未知 H2 模式：{c["mode"]}（可选：small, full）')

    c.setdefault('ready_pose', True)
    c.setdefault('gravity_feedforward', True)
    c.setdefault('pair_points', 24)
    c.setdefault('onset_speed_rad_s', 0.005)
    if int(c['pair_points']) < 4 or float(c['onset_speed_rad_s']) <= 0:
        raise ValueError('pair_points 须 ≥4，onset_speed_rad_s 须 >0')
    c['pair_points'] = int(c['pair_points'])
    c['onset_speed_rad_s'] = float(c['onset_speed_rad_s'])

    block = dict(c.get(c['mode']) or {})
    if not block:
        raise ValueError(f'H2 配置缺少 {c["mode"]} 模式参数')

    home_deg = np.asarray(block['home_deg'], dtype=float)
    if home_deg.shape != (14,) or not np.isfinite(home_deg).all():
        raise ValueError('H2 home_deg 须为 14 轴有限角度')
    block['home_deg'] = home_deg.tolist()

    joints = list(NAMES) if block.get('joints') == 'all' else list(block.get('joints') or ())
    if not joints or len(set(joints)) != len(joints) or not set(joints) <= set(NAMES):
        raise ValueError('H2 joints 须为 all 或不重复的双臂关节名列表')
    block['joints'] = joints

    stroke_deg = float(block['stroke_half_deg'])
    if not np.isfinite(stroke_deg) or stroke_deg <= 0:
        raise ValueError('H2 stroke_half_deg 须为正数')
    block['stroke_half_deg'] = stroke_deg

    offsets = dict(block['pose_offsets_deg'])
    speeds = dict(block['speeds_rad_s'])
    for split in ('train', 'validation'):
        po = [float(x) for x in offsets.get(split, ())]
        sp = [float(x) for x in speeds.get(split, ())]
        if not po or len(set(po)) != len(po) or not np.isfinite(po).all():
            raise ValueError(f'H2 {split} 姿态偏移须为不重复有限值')
        if not sp or len(set(sp)) != len(sp) or not np.isfinite(sp).all() or min(sp) <= 0:
            raise ValueError(f'H2 {split} 扫描速度须为不重复正数')
        offsets[split] = po
        speeds[split] = sp
    if len(speeds['train']) < 2:
        raise ValueError('H2 train 至少需要 2 个不同匀速以分离粘性与库仑摩擦')
    if set(offsets['train']) & set(offsets['validation']):
        raise ValueError('H2 train 与 validation 姿态偏移须互不重叠')
    block['pose_offsets_deg'] = offsets
    block['speeds_rad_s'] = speeds

    timing = dict(block['timing'])
    envelope = dict(block['planning_envelope'])
    nums = [*timing.values(), *envelope.values()]
    if not np.isfinite(nums).all() or min(nums) <= 0:
        raise ValueError('H2 timing 与 planning_envelope 参数须为正有限值')
    max_speed = max(*speeds['train'], *speeds['validation'])
    if 2.0 * np.deg2rad(stroke_deg) - max_speed * float(timing['speed_ramp_s']) <= 0.005:
        raise ValueError('H2 扫描半幅过短，无法容纳最高速度的加减速段与匀速平台')

    c[c['mode']] = block
    return c


def _joint_groups(active_indices):
    active_set = set(active_indices)
    visited = set()
    groups = []
    for j in active_indices:
        if j in visited:
            continue
        if j < 7 and (j + 7) in active_set:
            visited.update((j, j + 7))
            groups.append(((j, 1.0), (j + 7, MIRROR_SIGN_RIGHT[j + 7])))
        else:
            visited.add(j)
            sign = 1.0 if j < 7 else MIRROR_SIGN_RIGHT[j]
            groups.append(((j, sign),))
    return groups


def _posture_center(home_rad, offset_rad, active_indices):
    q = np.asarray(home_rad, dtype=float).copy()
    if len(active_indices) <= 2:
        for j in active_indices:
            q[j] += offset_rad * (1.0 if j < 7 else MIRROR_SIGN_RIGHT[j])
    else:
        for j in active_indices:
            q[j] += offset_rad * POSTURE_DIRECTION[j]
    return q


def _pair_friction_parts(parts, minimum_overlap, points):
    low = max(float(p[0].min()) for p in parts)
    high = min(float(p[0].max()) for p in parts)
    if high - low < minimum_overlap:
        raise ValueError(f'正反向扫描位置重叠不足（{high - low:.5f} rad < {minimum_overlap:.5f} rad）')
    grid = np.linspace(low, high, points)
    vp = np.interp(grid, parts[0][0], parts[0][1])
    tp = np.interp(grid, parts[0][0], parts[0][2])
    vm = np.interp(grid, parts[1][0], parts[1][1])
    tm = np.interp(grid, parts[1][0], parts[1][2])
    features = np.column_stack([(vp - vm) * 0.5, (np.tanh(vp / 0.001) - np.tanh(vm / 0.001)) * 0.5])
    target = (tp - tm) * 0.5
    return features, target, dict(matched_range_rad=[low, high], paired_points=int(points))


def _fit_axis_friction(x, y, vx, vy):
    scale = np.maximum(np.linalg.norm(x, axis=0), 1e-12)
    singular = np.linalg.svd(x / scale, compute_uv=False)
    rank = int(np.count_nonzero(singular > singular[0] * 1e-7))
    if rank < 2:
        raise ValueError('摩擦回归矩阵不满秩，需要多个不同匀速')
    sol = least_squares(
        lambda p: x @ p - y,
        [0.02, 0.03],
        bounds=(0.0, np.inf),
        loss='soft_l1',
        f_scale=0.01,
        jac=lambda _p: x.copy(),
        xtol=1e-12,
        ftol=1e-12,
        gtol=1e-12,
    )
    train_rmse = float(np.sqrt(np.mean((x @ sol.x - y) ** 2)))
    zero_val_rmse = float(np.sqrt(np.mean(vy ** 2)))
    val_rmse = float(np.sqrt(np.mean((vx @ sol.x - vy) ** 2)))
    return sol.x, dict(
        rank=rank,
        singular_values=singular.tolist(),
        train_rmse_nm=train_rmse,
        zero_friction_validation_rmse_nm=zero_val_rmse,
        validation_rmse_nm=val_rmse,
        success=bool(sol.success),
    )


def plan_h2(q_start, robot_config, recipe, out_dir, *, session=None):
    out_dir = Path(out_dir)
    q_start = np.asarray(q_start, dtype=float)
    if q_start.shape != (14,) or not np.isfinite(q_start).all():
        raise ValueError('起点关节角须为 14 轴有限弧度')

    prior = resolve_stage_prior(session, 'h2', robot_config)
    mode = recipe['mode']
    cfg = recipe[mode]
    timing = cfg['timing']
    envelope = cfg['planning_envelope']
    home_rad = np.deg2rad(np.asarray(cfg['home_deg'], dtype=float))
    extent = float(np.deg2rad(cfg['stroke_half_deg']))
    ramp_s = float(timing['speed_ramp_s'])
    rev_hold_s = float(timing['reverse_hold_s'])
    trans_s = float(timing['transition_s'])

    active_indices = [NAMES.index(name) for name in cfg['joints']]
    groups = _joint_groups(active_indices)
    anchor = ready_position() if recipe['ready_pose'] else q_start.copy()

    formal = Motion(f'shm_h2_friction_{mode}', 'friction', anchor)
    formal.add(float(timing['initial_hold_s']), anchor, label='initial_hold')

    sweeps = []
    markers = []
    sweep_counter = 0

    for split in ('train', 'validation'):
        for pose_idx, offset_deg in enumerate(cfg['pose_offsets_deg'][split]):
            pose_center = _posture_center(home_rad, np.deg2rad(offset_deg), active_indices)
            for speed in cfg['speeds_rad_s'][split]:
                distance = 2.0 * extent - speed * ramp_s
                plateau_s = distance / speed
                for group in groups:
                    start_q = pose_center.copy()
                    for j, base_sign in group:
                        start_q[j] -= base_sign * extent
                    _move(formal, start_q, trans_s, envelope, 'approach')

                    pass_windows = []
                    for pass_dir in (1.0, -1.0):
                        ramp_begin_ns = round(formal.duration * 1e9)
                        v_vec = np.zeros(14, dtype=float)
                        q_after_ramp = formal.states[-1][0].copy()
                        for j, base_sign in group:
                            vj = pass_dir * base_sign * speed
                            v_vec[j] = vj
                            q_after_ramp[j] += 0.5 * vj * ramp_s
                        formal.add(ramp_s, q_after_ramp, v_vec, label='speed_ramp')
                        plat_begin_ns = round(formal.duration * 1e9)

                        q_after_plat = q_after_ramp.copy()
                        for j, base_sign in group:
                            q_after_plat[j] += pass_dir * base_sign * distance
                        formal.add(plateau_s, q_after_plat, v_vec, label='constant_velocity')
                        plat_end_ns = round(formal.duration * 1e9)

                        q_after_brake = q_after_plat.copy()
                        for j, base_sign in group:
                            q_after_brake[j] += 0.5 * pass_dir * base_sign * speed * ramp_s
                        formal.add(ramp_s, q_after_brake, label='brake')
                        formal.add(rev_hold_s, q_after_brake, label='reverse_hold')
                        pass_windows.append(dict(
                            pass_dir=pass_dir,
                            ramp_ns=[ramp_begin_ns, plat_begin_ns],
                            plateau_ns=[plat_begin_ns, plat_end_ns],
                        ))

                    markers.append(dict(t_ns=pass_windows[0]['plateau_ns'][0], event=f'h2_sweep_{sweep_counter:03d}'))
                    for j, base_sign in group:
                        sweeps.append(dict(
                            id=f'sweep_{sweep_counter:03d}_j{j:02d}',
                            split=split,
                            pose_index=pose_idx,
                            pose_offset_deg=offset_deg,
                            joint_index=j,
                            joint_name=NAMES[j],
                            shm_id=j + 12,
                            speed_rad_s=speed,
                            stroke_half_rad=extent,
                            base_sign=base_sign,
                            passes=[
                                dict(
                                    signed_speed_rad_s=pw['pass_dir'] * base_sign * speed,
                                    ramp_ns=list(pw['ramp_ns']),
                                    plateau_ns=list(pw['plateau_ns']),
                                )
                                for pw in pass_windows
                            ],
                        ))
                    sweep_counter += 1

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
    )

    for sw in sweeps:
        for pw in sw['passes']:
            pw['ramp_ns'] = [pw['ramp_ns'][0] + offset_ns, pw['ramp_ns'][1] + offset_ns]
            pw['plateau_ns'] = [pw['plateau_ns'][0] + offset_ns, pw['plateau_ns'][1] + offset_ns]

    reference_path = save_bundle(out_dir, bundle)
    plan_meta = dict(
        schema='casbot_calib_h2_plan_v1',
        stage='h2',
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
        sweeps=sweeps,
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


def _extract_sweep_pair(obs, gravity_engine, gravity_nm, sw, trim_s, pair_points, onset_speed):
    j = int(sw['joint_index'])
    rel_ns = obs['relative_ns']
    elapsed = obs['elapsed_s']
    residual_j = obs['effort_nm'][:, j] - gravity_nm[:, j]
    pass_parts = []
    onset_torques = []

    for pw in sw['passes']:
        p_begin_s = pw['plateau_ns'][0] * 1e-9
        p_end_s = pw['plateau_ns'][1] * 1e-9
        span = p_end_s - p_begin_s
        eff_trim = min(trim_s, 0.25 * span)
        lo_s = p_begin_s + eff_trim
        hi_s = p_end_s - eff_trim
        mask = (elapsed >= lo_s) & (elapsed < hi_s)
        if int(mask.sum()) < 8:
            raise ValueError(f'{sw["id"]} 匀速平台有效观测不足')
        if np.any(obs['time_ambiguous'][mask]) or len(set(obs['clock_segment'][mask].tolist())) != 1:
            raise ValueError(f'{sw["id"]} 匀速平台跨越时钟跳变或分段')

        bin_count = max(6, min(40, int((hi_s - lo_s) / 0.04)))
        edges = np.linspace(lo_s, hi_s, bin_count + 1)
        bins = []
        for left, right in zip(edges[:-1], edges[1:]):
            bmask = mask & (elapsed >= left) & (elapsed < right)
            if not np.any(bmask):
                continue
            t_b = elapsed[bmask]
            w = np.diff(np.r_[t_b, right]) if len(t_b) > 1 else np.ones(1)
            w = np.maximum(w, 1e-6)
            w /= w.sum()
            qbar = np.average(obs['q'][bmask], axis=0, weights=w)
            vbar = np.average(obs['dq'][bmask], axis=0, weights=w)
            vbias = float(gravity_engine.velocity_bias(qbar, vbar)[j])
            rbar = float(np.average(residual_j[bmask], weights=w)) - vbias
            bins.append((qbar[j], vbar[j], rbar))
        if len(bins) < 4:
            raise ValueError(f'{sw["id"]} 匀速分箱不足')
        arr = np.asarray(bins, dtype=float)
        order = np.argsort(arr[:, 0])
        pass_parts.append((float(pw['signed_speed_rad_s']), arr[order, 0], arr[order, 1], arr[order, 2]))

        # PD onset estimate across ramp_ns -> early plateau.
        r_begin_ns, r_end_ns = pw['ramp_ns']
        omask = (rel_ns >= r_begin_ns) & (rel_ns <= r_end_ns + 100_000_000)
        if np.any(omask):
            sign = 1.0 if pw['signed_speed_rad_s'] >= 0 else -1.0
            idx_all = np.flatnonzero(omask)
            moving = np.flatnonzero(sign * obs['dq'][idx_all, j] >= onset_speed)
            if len(moving) > 0:
                first_idx = idx_all[int(moving[0])]
                prev_idx = max(idx_all[0], first_idx - 1)
                est = 0.5 * (abs(residual_j[prev_idx]) + abs(residual_j[first_idx]))
                onset_torques.append(float(est))

    # Order [positive_velocity_pass, negative_velocity_pass]
    pass_parts.sort(key=lambda item: item[0], reverse=True)
    parts = [(p[1], p[2], p[3]) for p in pass_parts]
    min_overlap = min(0.01, 0.2 * float(sw['stroke_half_rad']))
    x, y, pair_meta = _pair_friction_parts(parts, min_overlap, pair_points)
    return x, y, onset_torques, pair_meta


def analyse_h2(plan_dir, capture_dir, out_dir, *, session=None):
    plan_dir, capture_dir = resolve_effective_plan_and_capture(plan_dir, capture_dir)
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    plan_meta = json.loads((plan_dir / 'plan.json').read_text(encoding='utf-8'))
    robot_config = load_robot_config(plan_meta['robot_config'])
    recipe = plan_meta['recipe']
    mode_cfg = recipe[plan_meta['mode']]
    trim_s = float(mode_cfg['timing']['plateau_trim_s'])
    pair_points = int(recipe['pair_points'])
    onset_speed = float(recipe['onset_speed_rad_s'])

    obs, native, received_collision = read_verified_stage_capture(plan_meta, capture_dir, robot_config)

    nominal, _ = resolve_stage_prior(None, 'h2', robot_config)['nominal_engine'], None
    prior_info = plan_meta['prior']
    gravity_engine = model_from_parameters(
        nominal,
        prior_info['parameters'],
        viscous=prior_info['viscous'],
        coulomb=prior_info['coulomb'],
        static=prior_info['static_friction'],
    )
    gravity_nm = gravity_engine.gravity_batch(obs['q'])

    features_by_split = {'train': {}, 'validation': {}}
    onsets_by_joint = {}
    sweep_diagnostics = []

    for sw in plan_meta['sweeps']:
        j = int(sw['joint_index'])
        x, y, onsets, pair_meta = _extract_sweep_pair(obs, gravity_engine, gravity_nm, sw, trim_s, pair_points, onset_speed)
        features_by_split[sw['split']].setdefault(j, []).append((x, y))
        onsets_by_joint.setdefault(j, []).extend(onsets)
        sweep_diagnostics.append(dict(
            id=sw['id'],
            split=sw['split'],
            joint=sw['joint_name'],
            speed_rad_s=sw['speed_rad_s'],
            **pair_meta,
        ))

    viscous = np.asarray(prior_info['viscous'], dtype=float).copy()
    coulomb = np.asarray(prior_info['coulomb'], dtype=float).copy()
    static_friction = np.asarray(prior_info['static_friction'], dtype=float).copy()

    axis_reports = []
    zero_sq_errors = []
    fit_sq_errors = []

    for j, name in enumerate(NAMES):
        if j not in features_by_split['train']:
            axis_reports.append(dict(
                joint_index=j,
                joint=name,
                shm_id=j + 12,
                identified=False,
                viscous_nm_s_rad=float(viscous[j]),
                coulomb_nm=float(coulomb[j]),
                static_friction_nm=float(static_friction[j]),
                zero_friction_validation_rmse_nm=None,
                validation_rmse_nm=None,
            ))
            continue

        tx = np.vstack([a for a, _ in features_by_split['train'][j]])
        ty = np.concatenate([b for _, b in features_by_split['train'][j]])
        vx = np.vstack([a for a, _ in features_by_split['validation'][j]])
        vy = np.concatenate([b for _, b in features_by_split['validation'][j]])

        params, fit_info = _fit_axis_friction(tx, ty, vx, vy)
        b_j, fc_j = float(params[0]), float(params[1])
        onset_list = onsets_by_joint.get(j) or []
        fs_j = max(fc_j, float(np.median(onset_list))) if onset_list else fc_j

        viscous[j] = b_j
        coulomb[j] = fc_j
        static_friction[j] = fs_j

        zero_sq_errors.append(vy ** 2)
        fit_sq_errors.append((vx @ params - vy) ** 2)

        axis_reports.append(dict(
            joint_index=j,
            joint=name,
            shm_id=j + 12,
            identified=True,
            viscous_nm_s_rad=b_j,
            coulomb_nm=fc_j,
            static_friction_nm=fs_j,
            onset_samples=len(onset_list),
            **fit_info,
        ))

    overall_zero_rmse = float(np.sqrt(np.mean(np.concatenate(zero_sq_errors))))
    overall_val_rmse = float(np.sqrt(np.mean(np.concatenate(fit_sq_errors))))

    candidate_engine = model_from_parameters(
        nominal,
        prior_info['parameters'],
        viscous=viscous,
        coulomb=coulomb,
        static=static_friction,
        smoothing_velocity=float(prior_info['smoothing_velocity']),
        stribeck_velocity=float(prior_info['stribeck_velocity']),
    )
    exported = export_stage_candidate(out_dir, candidate_engine, robot_config)

    calibration = dict(
        schema_version='casbot_shm_h2_calibration_v1',
        stage='h2',
        model_context=robot_config['model_context'],
        joint_names=list(NAMES),
        identified_joints=list(plan_meta['active_joints']),
        gravity_world=list(robot_config['gravity_world']),
        effort_gain=list(robot_config['effort_gain']),
        effort_zero_nm=list(robot_config['effort_zero_nm']),
        prior_source=prior_info['source'],
        prior_source_path=prior_info['source_path'],
        parameters=list(prior_info['parameters']),
        viscous=viscous.tolist(),
        coulomb=coulomb.tolist(),
        static_friction=static_friction.tolist(),
        smoothing_velocity=float(prior_info['smoothing_velocity']),
        stribeck_velocity=float(prior_info['stribeck_velocity']),
        zero_friction_validation_rmse_nm=overall_zero_rmse,
        validation_rmse_nm=overall_val_rmse,
        candidate_urdf=exported['candidate_urdf'],
        runtime_dynamics=exported['runtime_dynamics'],
    )
    write_json(out_dir / 'calibration.json', calibration)

    with (out_dir / 'friction_axes.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'joint_index', 'shm_id', 'joint', 'identified',
            'viscous_nm_s_rad', 'coulomb_nm', 'static_friction_nm',
            'zero_friction_validation_rmse_nm', 'validation_rmse_nm',
        ])
        for row in axis_reports:
            writer.writerow([
                row['joint_index'], row['shm_id'], row['joint'], row['identified'],
                row['viscous_nm_s_rad'], row['coulomb_nm'], row['static_friction_nm'],
                row['zero_friction_validation_rmse_nm'], row['validation_rmse_nm'],
            ])

    report = dict(
        schema='casbot_calib_h2_report_v1',
        stage='h2',
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
        zero_friction_validation_rmse_nm=overall_zero_rmse,
        validation_rmse_nm=overall_val_rmse,
        axes=axis_reports,
        sweeps=sweep_diagnostics,
        candidate_urdf=exported['candidate_urdf'],
        runtime_dynamics=exported['runtime_dynamics'],
    )
    write_json(out_dir / 'report.json', report)
    return report


def write_h2_summary_markdown(stage_dir, plan_meta, capture_summary, report):
    lines = [
        '# H2 关节摩擦标定报告',
        '',
        f'- **模式**：`{plan_meta["mode"]}`（准备位前缀：`{"开启" if plan_meta["ready_pose"] else "关闭"}`）',
        f'- **重力先验来源**：`{plan_meta["prior"]["source"]}`',
        f'- **规划总时长**：{plan_meta["duration_ns"] * 1e-9:.2f} s（扫描段数：{len(plan_meta["sweeps"])}）',
        f'- **参考轨迹最小碰撞间距**：{plan_meta["collision_certificate"]["minimum_distance_m"] * 1000:.2f} mm',
    ]
    if capture_summary is None or report is None:
        lines.extend(['', '> 仅生成离线规划（`--plan-only`），未连接设备。'])
    else:
        hz = f'{report["mean_feedback_hz"]:.1f} Hz' if report.get('mean_feedback_hz') else 'n/a'
        lines.extend([
            f'- **有效反馈帧数**：{report["feedback_rows"]}（均值频率 {hz}，实测最小碰撞间距 {report["received_collision"]["minimum_distance_m"] * 1000:.2f} mm）',
            f'- **留出集整体摩擦残差 RMSE**：零摩擦基线 **{report["zero_friction_validation_rmse_nm"]:.4f} Nm** → 拟合候选 **{report["validation_rmse_nm"]:.4f} Nm**',
            f'- **导出模型**：`analysis/{report["candidate_urdf"]}` + `analysis/{report["runtime_dynamics"]}`',
            '',
            '## 逐轴摩擦参数与留出集残差（Nm）',
            '',
            '| 关节 | SHM ID | 粘性 b (Nm·s/rad) | 库仑 fc (Nm) | 静摩擦 fs (Nm) | 零摩擦 RMSE | 拟合后 RMSE |',
            '| --- | :---: | ---: | ---: | ---: | ---: | ---: |',
        ])
        for row in report['axes']:
            if not row['identified']:
                continue
            j = row['joint_index']
            label = ('左' if j < 7 else '右') + PARTS[j % 7]
            lines.append(
                f'| {label} (`{row["joint"]}`) | {row["shm_id"]} '
                f'| {row["viscous_nm_s_rad"]:.4f} | {row["coulomb_nm"]:.4f} | {row["static_friction_nm"]:.4f} '
                f'| {row["zero_friction_validation_rmse_nm"]:.4f} | {row["validation_rmse_nm"]:.4f} |'
            )

    summary_path = Path(stage_dir) / 'report.md'
    summary_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return summary_path


class H2FrictionStage(Stage):
    name = 'h2'
    title = 'H2 关节摩擦标定'
    default_config = DEFAULT_H2_CONFIG
    implemented = True

    def load_recipe(self, config_path=None, **overrides):
        return load_h2_recipe(config_path or self.default_config, **overrides)

    def plan(self, q_start, robot_config, recipe, out_dir, *, session=None):
        return plan_h2(q_start, robot_config, recipe, out_dir, session=session)

    def analyse(self, plan_dir, capture_dir, out_dir, *, session=None):
        return analyse_h2(plan_dir, capture_dir, out_dir, session=session)

    def write_summary(self, stage_dir, plan_meta, capture_summary, report):
        return write_h2_summary_markdown(stage_dir, plan_meta, capture_summary, report)
