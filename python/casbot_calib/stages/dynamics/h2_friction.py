"""H2 Friction stage: logarithmic speed sweeps + C++-homomorphic Stribeck fit, chained automatically from H1 gravity."""
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
from ...staged_motion import _move
from .common import (
    compose_stage_reference,
    evaluate_stage_acceptance,
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
    max_plateau_raw = timing.pop('max_plateau_s', None)
    envelope = dict(block['planning_envelope'])
    nums = [*timing.values(), *envelope.values()]
    if not np.isfinite(nums).all() or min(nums) <= 0:
        raise ValueError('H2 timing 与 planning_envelope 参数须为正有限值')
    if max_plateau_raw is not None:
        max_plateau_s = float(max_plateau_raw)
        if not np.isfinite(max_plateau_s) or max_plateau_s <= 2.0 * float(timing['plateau_trim_s']) + 0.2:
            raise ValueError('H2 timing.max_plateau_s 须大于 2*plateau_trim_s + 0.2s')
        timing['max_plateau_s'] = max_plateau_s

    max_speed = max(*speeds['train'], *speeds['validation'])
    min_speed = min(*speeds['train'], *speeds['validation'])
    ramp_s = float(timing['speed_ramp_s'])
    if 2.0 * np.deg2rad(stroke_deg) - max_speed * ramp_s <= 0.005:
        raise ValueError('H2 扫描半幅过短，无法容纳最高速度的加减速段与匀速平台')
    min_raw_plateau = (2.0 * np.deg2rad(stroke_deg) - min_speed * ramp_s) / min_speed
    eff_min_plateau = min(min_raw_plateau, timing['max_plateau_s']) if 'max_plateau_s' in timing else min_raw_plateau
    if min_speed * eff_min_plateau <= 0.005:
        raise ValueError('H2 最低速度匀速平台有效行程过短（须 > 0.005 rad）')

    block['timing'] = timing
    block['planning_envelope'] = envelope
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


def _stribeck_torque_and_jac(v, params, v0=0.001):
    """Exact C++-homomorphic Stribeck friction model and analytic Jacobian wrt [b, fc, dfs, vs]."""
    v = np.asarray(v, dtype=float)
    b = float(params[0])
    fc = float(params[1])
    dfs = float(params[2])
    vs = float(params[3])
    th = np.tanh(v / float(v0))
    u = v / vs
    u2 = u * u
    ex = np.exp(-u2)
    tau = b * v + (fc + dfs * ex) * th
    jac = np.column_stack([
        v,
        th,
        ex * th,
        dfs * ex * (2.0 * u2 / vs) * th,
    ])
    return tau, jac


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
    y_plat = (tp - tm) * 0.5
    offset_vec = (tp + tm) * 0.5
    pair_meta = dict(
        matched_range_rad=[low, high],
        paired_points=int(points),
        sweep_offset_nm=float(np.mean(offset_vec)),
        torque_offset_rms_nm=float(np.sqrt(np.mean(offset_vec ** 2))),
    )
    return (vp, vm, y_plat), offset_vec, pair_meta


def _extract_transition_pairs(obs, gravity_engine, sw, j, onset_speed, sweep_offset_nm):
    """Extract low-velocity ramp/brake bins and pair +/- passes by speed bin to cancel static offset & inertia."""
    elapsed = obs['elapsed_s']
    pass_bins_by_sign = {}

    for pw in sw['passes']:
        signed_speed = float(pw['signed_speed_rad_s'])
        sign = 1.0 if signed_speed >= 0.0 else -1.0
        v_nom = abs(signed_speed)
        s_hi = 0.85 * v_nom
        if s_hi <= onset_speed * 1.15:
            continue

        windows = [pw['ramp_ns']]
        if 'brake_ns' in pw and pw['brake_ns'] is not None:
            windows.append(pw['brake_ns'])

        s_samples = []
        v_samples = []
        r_samples = []

        for w_lo_ns, w_hi_ns in windows:
            w_lo_s = w_lo_ns * 1e-9
            w_hi_s = w_hi_ns * 1e-9
            if w_hi_s - w_lo_s < 0.05:
                continue
            wmask = (elapsed >= w_lo_s) & (elapsed <= w_hi_s)
            if int(wmask.sum()) < 8:
                continue
            if np.any(obs['time_ambiguous'][wmask]) or len(set(obs['clock_segment'][wmask].tolist())) != 1:
                continue

            n_bins = max(4, min(14, int(round((w_hi_s - w_lo_s) / 0.025))))
            edges = np.linspace(w_lo_s, w_hi_s, n_bins + 1)
            t_list = []
            q_list = []
            v_list = []
            tau_list = []
            for k_edge, (left, right) in enumerate(zip(edges[:-1], edges[1:])):
                if k_edge == n_bins - 1:
                    bmask = wmask & (elapsed >= left) & (elapsed <= right)
                else:
                    bmask = wmask & (elapsed >= left) & (elapsed < right)
                if int(bmask.sum()) < 2:
                    continue
                t_list.append(float(np.mean(elapsed[bmask])))
                q_list.append(np.mean(obs['q'][bmask], axis=0))
                v_list.append(np.mean(obs['dq'][bmask], axis=0))
                tau_list.append(float(np.mean(obs['effort_nm'][bmask, j])))

            if len(t_list) < 3:
                continue
            t_arr = np.asarray(t_list, dtype=float)
            if np.any(np.diff(t_arr) <= 1e-6):
                continue
            q_arr = np.asarray(q_list, dtype=float)
            v_arr = np.asarray(v_list, dtype=float)
            tau_arr = np.asarray(tau_list, dtype=float)
            ddq_arr = np.gradient(v_arr, t_arr, axis=0, edge_order=1)
            terms = gravity_engine.evaluate_batch(q_arr, v_arr, ddq_arr)
            inv_no_fric = np.asarray(terms['total'], dtype=float)[:, j] - np.asarray(terms['friction'], dtype=float)[:, j]
            r_arr = tau_arr - inv_no_fric

            s_arr = sign * v_arr[:, j]
            keep = (s_arr >= onset_speed) & (s_arr <= s_hi)
            if np.any(keep):
                s_samples.append(s_arr[keep])
                v_samples.append(v_arr[keep, j])
                r_samples.append(r_arr[keep])

        if s_samples:
            pass_bins_by_sign[1 if sign > 0 else -1] = (
                np.concatenate(s_samples),
                np.concatenate(v_samples),
                np.concatenate(r_samples),
                s_hi,
            )

    if 1 not in pass_bins_by_sign and -1 not in pass_bins_by_sign:
        return np.empty(0, dtype=float), np.empty(0, dtype=float), np.empty(0, dtype=float)

    s_max = max(item[3] for item in pass_bins_by_sign.values())
    n_s_bins = 7
    s_edges = np.linspace(onset_speed, s_max, n_s_bins + 1)

    vp_out = []
    vm_out = []
    y_out = []
    for lo_v, hi_v in zip(s_edges[:-1], s_edges[1:]):
        pos_entry = None
        neg_entry = None
        if 1 in pass_bins_by_sign:
            sp, vp_s, rp_s, _ = pass_bins_by_sign[1]
            m = (sp >= lo_v) & (sp <= hi_v)
            if np.any(m):
                pos_entry = (float(np.mean(vp_s[m])), float(np.mean(rp_s[m])))
        if -1 in pass_bins_by_sign:
            sm, vm_s, rm_s, _ = pass_bins_by_sign[-1]
            m = (sm >= lo_v) & (sm <= hi_v)
            if np.any(m):
                neg_entry = (float(np.mean(vm_s[m])), float(np.mean(rm_s[m])))

        if pos_entry is not None and neg_entry is not None:
            vp_out.append(pos_entry[0])
            vm_out.append(neg_entry[0])
            y_out.append(0.5 * (pos_entry[1] - neg_entry[1]))
        elif pos_entry is not None:
            vp_out.append(pos_entry[0])
            vm_out.append(-pos_entry[0])
            y_out.append(pos_entry[1] - sweep_offset_nm)
        elif neg_entry is not None:
            vp_out.append(-neg_entry[0])
            vm_out.append(neg_entry[0])
            y_out.append(sweep_offset_nm - neg_entry[1])

    return (
        np.asarray(vp_out, dtype=float),
        np.asarray(vm_out, dtype=float),
        np.asarray(y_out, dtype=float),
    )


def _fit_axis_stribeck(
    plat_pairs,
    trans_pairs,
    val_pairs,
    *,
    onset_list=(),
    n_train_speeds=4,
    min_train_speed=0.03,
    v0=0.001,
    prior_vs=0.005,
):
    vp_p, vm_p, y_p = plat_pairs
    vp_t, vm_t, y_t = trans_pairs
    vp_v, vm_v, y_v = val_pairs

    x_lin = np.column_stack([
        0.5 * (vp_p - vm_p),
        0.5 * (np.tanh(vp_p / v0) - np.tanh(vm_p / v0)),
    ])
    scale = np.maximum(np.linalg.norm(x_lin, axis=0), 1e-12)
    singular = np.linalg.svd(x_lin / scale, compute_uv=False)
    rank = int(np.count_nonzero(singular > singular[0] * 1e-7))
    if rank < 2:
        raise ValueError('摩擦回归矩阵不满秩，需要多个不同匀速')

    sol_lin = least_squares(
        lambda p: x_lin @ p - y_p,
        [0.02, 0.03],
        bounds=(0.0, [50.0, 20.0]),
        loss='soft_l1',
        f_scale=0.02,
        jac=lambda _p: x_lin.copy(),
        xtol=1e-12,
        ftol=1e-12,
        gtol=1e-12,
    )
    b_lin, fc_lin = float(sol_lin.x[0]), float(sol_lin.x[1])
    vx_lin = np.column_stack([
        0.5 * (vp_v - vm_v),
        0.5 * (np.tanh(vp_v / v0) - np.tanh(vm_v / v0)),
    ])
    linear_val_rmse = float(np.sqrt(np.mean((vx_lin @ sol_lin.x - y_v) ** 2)))
    zero_val_rmse = float(np.sqrt(np.mean(y_v ** 2)))

    onset_median = float(np.median(onset_list)) if onset_list else 0.0
    vs_lin = float(np.clip(prior_vs, 0.002, max(0.002, min(0.015, 0.25 * float(min_train_speed)))))
    dfs_lin = float(np.clip(onset_median - fc_lin, 0.0, 2.0)) if onset_list else 0.0
    p_lin = np.array([b_lin, fc_lin, dfs_lin, vs_lin], dtype=float)
    tv_p_lin, _ = _stribeck_torque_and_jac(vp_v, p_lin, v0)
    tv_m_lin, _ = _stribeck_torque_and_jac(vm_v, p_lin, v0)
    val_pred_lin = 0.5 * (tv_p_lin - tv_m_lin)
    lin_stribeck_val_rmse = float(np.sqrt(np.mean((val_pred_lin - y_v) ** 2)))

    if int(n_train_speeds) < 3:
        tp_lin, _ = _stribeck_torque_and_jac(vp_p, p_lin, v0)
        tm_lin, _ = _stribeck_torque_and_jac(vm_p, p_lin, v0)
        train_rmse_lin = float(np.sqrt(np.mean((0.5 * (tp_lin - tm_lin) - y_p) ** 2)))
        return p_lin, val_pred_lin, dict(
            rank=rank,
            singular_values=singular.tolist(),
            transition_pairs=int(len(vp_t)),
            train_rmse_nm=train_rmse_lin,
            linear_validation_rmse_nm=linear_val_rmse,
            zero_friction_validation_rmse_nm=zero_val_rmse,
            validation_rmse_nm=lin_stribeck_val_rmse,
            stribeck_nonlinear_used=False,
            success=bool(sol_lin.success),
        )

    if len(vp_t) > 0:
        vp_all = np.concatenate([vp_p, vp_t])
        vm_all = np.concatenate([vm_p, vm_t])
        y_all = np.concatenate([y_p, y_t])
        w_all = np.concatenate([np.ones(len(vp_p), dtype=float), np.full(len(vp_t), 0.35, dtype=float)])
    else:
        vp_all = vp_p
        vm_all = vm_p
        y_all = y_p
        w_all = np.ones(len(vp_p), dtype=float)

    use_onset_anchor = bool(onset_list) and float(min_train_speed) > 0.035

    def _res_and_jac(p):
        tau_p, jac_p = _stribeck_torque_and_jac(vp_all, p, v0)
        tau_m, jac_m = _stribeck_torque_and_jac(vm_all, p, v0)
        r_data = w_all * (0.5 * (tau_p - tau_m) - y_all)
        j_data = w_all[:, None] * (0.5 * (jac_p - jac_m))

        reg_r = [1e-3 * float(p[2]), 1e-2 * (float(p[3]) - 0.015)]
        reg_j = [[0.0, 0.0, 1e-3, 0.0], [0.0, 0.0, 0.0, 1e-2]]
        if use_onset_anchor and onset_median > float(p[1]):
            reg_r.append(0.05 * (float(p[1]) + float(p[2]) - onset_median))
            reg_j.append([0.0, 0.05, 0.05, 0.0])

        return (
            np.r_[r_data, np.asarray(reg_r, dtype=float)],
            np.vstack([j_data, np.asarray(reg_j, dtype=float)]),
        )

    excess_est = float(np.max(y_all - (0.5 * b_lin * (vp_all - vm_all) + fc_lin), initial=0.0))
    if onset_list:
        excess_est = max(excess_est, onset_median - fc_lin)
    dfs_init = float(np.clip(max(0.02, excess_est), 0.01, 10.0))
    b0 = float(np.clip(b_lin, 0.0, 49.0))
    fc0 = float(np.clip(fc_lin, 0.0, 19.0))

    lower = np.array([0.0, 0.0, 0.0, 0.002], dtype=float)
    upper = np.array([50.0, 20.0, 20.0, 0.08], dtype=float)

    best_sol = None
    for vs0 in (0.008, 0.02, 0.04, 0.065):
        p0 = np.array([b0, fc0, dfs_init, vs0], dtype=float)
        sol = least_squares(
            lambda p: _res_and_jac(p)[0],
            p0,
            jac=lambda p: _res_and_jac(p)[1],
            method='trf',
            bounds=(lower, upper),
            loss='soft_l1',
            f_scale=0.02,
            xtol=1e-10,
            ftol=1e-10,
            gtol=1e-10,
        )
        if best_sol is None or sol.cost < best_sol.cost:
            best_sol = sol

    p_opt = np.asarray(best_sol.x, dtype=float)
    tv_p, _ = _stribeck_torque_and_jac(vp_v, p_opt, v0)
    tv_m, _ = _stribeck_torque_and_jac(vm_v, p_opt, v0)
    val_pred = 0.5 * (tv_p - tv_m)
    val_rmse = float(np.sqrt(np.mean((val_pred - y_v) ** 2)))
    nonlinear_used = True
    if val_rmse > lin_stribeck_val_rmse * 1.002 + 1e-4:
        p_opt = p_lin
        val_pred = val_pred_lin
        val_rmse = lin_stribeck_val_rmse
        nonlinear_used = False

    tp_fit, _ = _stribeck_torque_and_jac(vp_p, p_opt, v0)
    tm_fit, _ = _stribeck_torque_and_jac(vm_p, p_opt, v0)
    train_rmse = float(np.sqrt(np.mean((0.5 * (tp_fit - tm_fit) - y_p) ** 2)))

    return p_opt, val_pred, dict(
        rank=rank,
        singular_values=singular.tolist(),
        transition_pairs=int(len(vp_t)),
        train_rmse_nm=train_rmse,
        linear_validation_rmse_nm=linear_val_rmse,
        zero_friction_validation_rmse_nm=zero_val_rmse,
        validation_rmse_nm=val_rmse,
        stribeck_nonlinear_used=nonlinear_used,
        success=bool(best_sol.success),
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
    max_plateau_s = timing.get('max_plateau_s')

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
            for group in groups:
                start_q = pose_center.copy()
                for j, base_sign in group:
                    start_q[j] -= base_sign * extent
                if not np.allclose(formal.states[-1][0], start_q, rtol=0.0, atol=1e-9):
                    _move(formal, start_q, trans_s, envelope, 'approach')

                for speed in cfg['speeds_rad_s'][split]:
                    raw_distance = 2.0 * extent - speed * ramp_s
                    raw_plateau_s = raw_distance / speed
                    if max_plateau_s is not None and raw_plateau_s > float(max_plateau_s):
                        plateau_s = float(max_plateau_s)
                        distance = speed * plateau_s
                        sweep_extent = 0.5 * (distance + speed * ramp_s)
                    else:
                        plateau_s = raw_plateau_s
                        distance = raw_distance
                        sweep_extent = extent

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

                        if pass_dir < 0.0:
                            q_after_brake = start_q.copy()
                        else:
                            q_after_brake = q_after_plat.copy()
                            for j, base_sign in group:
                                q_after_brake[j] += 0.5 * pass_dir * base_sign * speed * ramp_s
                        formal.add(ramp_s, q_after_brake, label='brake')
                        brake_end_ns = round(formal.duration * 1e9)
                        formal.add(rev_hold_s, q_after_brake, label='reverse_hold')
                        pass_windows.append(dict(
                            pass_dir=pass_dir,
                            ramp_ns=[ramp_begin_ns, plat_begin_ns],
                            plateau_ns=[plat_begin_ns, plat_end_ns],
                            brake_ns=[plat_end_ns, brake_end_ns],
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
                            stroke_half_rad=float(sweep_extent),
                            base_sign=base_sign,
                            passes=[
                                dict(
                                    signed_speed_rad_s=pw['pass_dir'] * base_sign * speed,
                                    ramp_ns=list(pw['ramp_ns']),
                                    plateau_ns=list(pw['plateau_ns']),
                                    brake_ns=list(pw['brake_ns']),
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
            pw['brake_ns'] = [pw['brake_ns'][0] + offset_ns, pw['brake_ns'][1] + offset_ns]

    reference_path = save_bundle(out_dir, bundle)
    stribeck_prior = (
        prior['stribeck_velocity'].tolist()
        if isinstance(prior['stribeck_velocity'], np.ndarray)
        else prior['stribeck_velocity']
    )
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
            smoothing_velocity= (
                prior['smoothing_velocity'].tolist()
                if isinstance(prior['smoothing_velocity'], np.ndarray)
                else prior['smoothing_velocity']
            ),
            stribeck_velocity=stribeck_prior,
            rotor_inertia=prior['rotor_inertia'].tolist(),
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
    plat_tuple, offset_vec, pair_meta = _pair_friction_parts(parts, min_overlap, pair_points)
    trans_tuple = _extract_transition_pairs(
        obs,
        gravity_engine,
        sw,
        j,
        onset_speed,
        pair_meta['sweep_offset_nm'],
    )
    pair_meta['transition_paired_points'] = int(len(trans_tuple[0]))
    return plat_tuple, trans_tuple, offset_vec, onset_torques, pair_meta


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
    train_speeds = sorted({round(float(s), 6) for s in mode_cfg['speeds_rad_s']['train']})
    min_train_speed = float(train_speeds[0])
    n_train_speeds = len(train_speeds)

    obs, native, received_collision = read_verified_stage_capture(plan_meta, capture_dir, robot_config)

    if session is not None:
        live = resolve_stage_prior(session, 'h2', robot_config)
        nominal = live['nominal_engine']
        prior_info = dict(
            source=live['source'],
            source_path=live['source_path'],
            parameters=live['parameters'].tolist(),
            viscous=live['viscous'].tolist(),
            coulomb=live['coulomb'].tolist(),
            static_friction=live['static_friction'].tolist(),
            smoothing_velocity=live['smoothing_velocity'],
            stribeck_velocity=live['stribeck_velocity'],
            rotor_inertia=live['rotor_inertia'].tolist(),
        )
    else:
        nominal = resolve_stage_prior(None, 'h2', robot_config)['nominal_engine']
        prior_info = plan_meta['prior']
    smoothing_vel_arr = np.broadcast_to(np.asarray(prior_info.get('smoothing_velocity', 0.001), dtype=float), (14,))
    stribeck_vel = np.broadcast_to(np.asarray(prior_info.get('stribeck_velocity', 0.005), dtype=float), (14,)).copy()
    prior_rotor = prior_info.get('rotor_inertia')

    # Use zero friction in gravity_engine so evaluate_batch['total'] - ['friction'] is pure rigid + rotor inverse dynamics.
    gravity_engine = model_from_parameters(
        nominal,
        prior_info['parameters'],
        viscous=prior_info['viscous'],
        coulomb=prior_info['coulomb'],
        static=prior_info['static_friction'],
        smoothing_velocity=smoothing_vel_arr,
        stribeck_velocity=stribeck_vel,
        rotor_inertia=prior_rotor,
    )
    gravity_nm = np.asarray(gravity_engine.gravity_batch(obs['q']), dtype=float)

    plat_by_split = {'train': {}, 'validation': {}}
    trans_by_joint = {}
    offsets_by_joint = {}
    onsets_by_joint = {}
    sweep_diagnostics = []

    for sw in plan_meta['sweeps']:
        j = int(sw['joint_index'])
        plat_tuple, trans_tuple, offset_vec, onsets, pair_meta = _extract_sweep_pair(
            obs,
            gravity_engine,
            gravity_nm,
            sw,
            trim_s,
            pair_points,
            onset_speed,
        )
        plat_by_split[sw['split']].setdefault(j, []).append(plat_tuple)
        if sw['split'] == 'train' and len(trans_tuple[0]) > 0:
            trans_by_joint.setdefault(j, []).append(trans_tuple)
        offsets_by_joint.setdefault(j, []).append(offset_vec)
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
        if j not in plat_by_split['train']:
            axis_reports.append(dict(
                joint_index=j,
                joint=name,
                shm_id=j + 12,
                identified=False,
                viscous_nm_s_rad=float(viscous[j]),
                coulomb_nm=float(coulomb[j]),
                static_friction_nm=float(static_friction[j]),
                stribeck_velocity_rad_s=float(stribeck_vel[j]),
                torque_offset_diagnostic_nm=None,
                zero_friction_validation_rmse_nm=None,
                validation_rmse_nm=None,
            ))
            continue

        vp_p = np.concatenate([t[0] for t in plat_by_split['train'][j]])
        vm_p = np.concatenate([t[1] for t in plat_by_split['train'][j]])
        y_p = np.concatenate([t[2] for t in plat_by_split['train'][j]])

        if j in trans_by_joint and trans_by_joint[j]:
            vp_t = np.concatenate([t[0] for t in trans_by_joint[j]])
            vm_t = np.concatenate([t[1] for t in trans_by_joint[j]])
            y_t = np.concatenate([t[2] for t in trans_by_joint[j]])
        else:
            vp_t = vm_t = y_t = np.empty(0, dtype=float)

        vp_v = np.concatenate([t[0] for t in plat_by_split['validation'][j]])
        vm_v = np.concatenate([t[1] for t in plat_by_split['validation'][j]])
        y_v = np.concatenate([t[2] for t in plat_by_split['validation'][j]])

        onset_list = onsets_by_joint.get(j) or []
        v0_j = float(smoothing_vel_arr[j])
        params, val_pred, fit_info = _fit_axis_stribeck(
            (vp_p, vm_p, y_p),
            (vp_t, vm_t, y_t),
            (vp_v, vm_v, y_v),
            onset_list=onset_list,
            n_train_speeds=n_train_speeds,
            min_train_speed=min_train_speed,
            v0=v0_j,
            prior_vs=float(stribeck_vel[j]),
        )
        b_j = float(params[0])
        fc_j = float(params[1])
        fs_j = float(params[1] + params[2])
        vs_j = float(params[3])

        viscous[j] = b_j
        coulomb[j] = fc_j
        static_friction[j] = fs_j
        stribeck_vel[j] = vs_j

        all_offsets_j = np.concatenate(offsets_by_joint[j])
        offset_diag_nm = float(np.sqrt(np.mean(all_offsets_j ** 2)))
        mean_offset_nm = float(np.mean(all_offsets_j))

        zero_sq_errors.append(y_v ** 2)
        fit_sq_errors.append((val_pred - y_v) ** 2)

        axis_reports.append(dict(
            joint_index=j,
            joint=name,
            shm_id=j + 12,
            identified=True,
            viscous_nm_s_rad=b_j,
            coulomb_nm=fc_j,
            static_friction_nm=fs_j,
            stribeck_velocity_rad_s=vs_j,
            torque_offset_diagnostic_nm=offset_diag_nm,
            mean_torque_offset_nm=mean_offset_nm,
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
        smoothing_velocity=smoothing_vel_arr,
        stribeck_velocity=stribeck_vel,
        rotor_inertia=prior_rotor,
    )
    exported = export_stage_candidate(out_dir, candidate_engine, robot_config)
    acceptance = evaluate_stage_acceptance(
        overall_zero_rmse,
        overall_val_rmse,
        axis_rows=[
            (row['joint'], row['zero_friction_validation_rmse_nm'], row['validation_rmse_nm'])
            for row in axis_reports
            if row['identified']
        ],
        reload_passed=exported['reload_passed'],
    )

    smoothing_out = (
        float(smoothing_vel_arr[0])
        if np.allclose(smoothing_vel_arr, smoothing_vel_arr[0])
        else smoothing_vel_arr.tolist()
    )
    calibration = dict(
        schema_version='casbot_shm_h2_calibration_v1',
        stage='h2',
        accepted=acceptance['accepted'],
        acceptance=acceptance,
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
        smoothing_velocity=smoothing_out,
        stribeck_velocity=stribeck_vel.tolist(),
        rotor_inertia=[float(jt.rotor_inertia) for jt in candidate_engine.model.joints],
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
            'stribeck_velocity_rad_s', 'torque_offset_diagnostic_nm',
            'zero_friction_validation_rmse_nm', 'validation_rmse_nm',
        ])
        for row in axis_reports:
            writer.writerow([
                row['joint_index'], row['shm_id'], row['joint'], row['identified'],
                row['viscous_nm_s_rad'], row['coulomb_nm'], row['static_friction_nm'],
                row['stribeck_velocity_rad_s'], row['torque_offset_diagnostic_nm'],
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
        accepted=acceptance['accepted'],
        acceptance=acceptance,
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
    prior_src = report['prior_source'] if report and report.get('prior_source') else plan_meta['prior']['source']
    lines = [
        '# H2 关节摩擦标定报告',
        '',
        f'- **模式**：`{plan_meta["mode"]}`（准备位前缀：`{"开启" if plan_meta["ready_pose"] else "关闭"}`）',
        f'- **重力先验来源**：`{prior_src}`',
        f'- **规划总时长**：{plan_meta["duration_ns"] * 1e-9:.2f} s（扫描段数：{len(plan_meta["sweeps"])}）',
        f'- **参考轨迹最小碰撞间距**：{plan_meta["collision_certificate"]["minimum_distance_m"] * 1000:.2f} mm',
    ]
    if capture_summary is None or report is None:
        lines.extend(['', '> 仅生成离线规划（`--plan-only`），未连接设备。'])
    else:
        hz = f'{report["mean_feedback_hz"]:.1f} Hz' if report.get('mean_feedback_hz') else 'n/a'
        acc = report.get('acceptance') or {}
        gate_str = '通过（自动采纳）' if acc.get('accepted', True) else f'未通过（{acc.get("reason", "degraded")}，自动回退上游先验）'
        lines.extend([
            f'- **有效反馈帧数**：{report["feedback_rows"]}（均值频率 {hz}，实测最小碰撞间距 {report["received_collision"]["minimum_distance_m"] * 1000:.2f} mm）',
            f'- **留出集整体摩擦残差 RMSE**：零摩擦基线 **{report["zero_friction_validation_rmse_nm"]:.4f} Nm** → Stribeck 拟合候选 **{report["validation_rmse_nm"]:.4f} Nm**',
            f'- **防退化验收门禁**：{gate_str}',
            f'- **导出模型**：`analysis/{report["candidate_urdf"]}` + `analysis/{report["runtime_dynamics"]}`',
            '',
            '## 逐轴 Stribeck 摩擦参数与留出集残差（Nm）',
            '',
            '| 关节 | SHM ID | 粘性 b (Nm·s/rad) | 库仑 fc (Nm) | 静摩擦 fs (Nm) | Stribeck vs (rad/s) | 零偏诊断 (Nm) | 零摩擦 RMSE | 拟合后 RMSE |',
            '| --- | :---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |',
        ])
        for row in report['axes']:
            if not row['identified']:
                continue
            j = row['joint_index']
            label = ('左' if j < 7 else '右') + PARTS[j % 7]
            lines.append(
                f'| {label} (`{row["joint"]}`) | {row["shm_id"]} '
                f'| {row["viscous_nm_s_rad"]:.4f} | {row["coulomb_nm"]:.4f} | {row["static_friction_nm"]:.4f} '
                f'| {row["stribeck_velocity_rad_s"]:.4f} | {row["torque_offset_diagnostic_nm"]:.4f} '
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
