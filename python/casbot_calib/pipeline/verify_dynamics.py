"""Bare-flange closed-loop verification of a delivered H1–H3 model."""

import argparse
from dataclasses import dataclass
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.interpolate import BPoly, interp1d
from scipy.signal import butter, sosfiltfilt
import yaml

from .. import _core
from ..data import write_json
from ..dynamic_export import _restore_runtime
from ..model_runtime import context_for, model_for
from ..shm_data import DIRECTION, NAMES, ROOT
from ..shm_observations import native_directory
from ..shm_ready import ready_position
from ..shm_reference import save_bundle
from ..shm_takeover import HandoffError
from ..staged_motion import Motion, _move, derivative_bounds
from ..trical.assembly import profile as trical_profile
from ..stages.dynamics.common import (
    compose_stage_reference, read_verified_stage_capture, received_clearance_line, resolve_effective_plan_and_capture,
)
from .native import execute_with_start_replan, maybe_mock_context, native_snapshot, soft_stop_status
from .preflight import check_hardware_environment, check_model_bundle, check_native_binaries
from .robot import DEFAULT_ROBOT_CONFIG, load_robot_config
from .session import DEFAULT_OUTPUT_ROOT, SAFE_ID, _update_latest_link, resolve_robot_id
from .takeover import ensure_ready_channel

CLIP_ASSET = Path(__file__).resolve().parents[1] / 'assets/band_acceptance_clips.json'
CLIP_STATE_RESOLUTION = np.array([1e-5, 1e-4, 1e-3, 1e-2])[None, :, None]
GROUPS = ('G0_stiff', 'G0_soft', 'G1_soft', 'G2_soft', 'G3_soft')
INSTRUMENTS = ('keyboard', 'drum')
HOLD_S = 1.0
TRANSITION = dict(speed_rad_s=1.25, acceleration_rad_s2=2.5, jerk_rad_s3=10.0)
READY_TO_ANCHOR_SPEED_RAD_S = 0.8
MAX_OBSERVED_VELOCITY_RAD_S = 2.0
OBSERVED_VELOCITY_MARGIN = 1.5
SMOOTHING_FLOOR = 0.10
EFFORT_STEP_NS = 1_000_000


def _read_json(path):
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'预期 JSON 对象：{path}')
    return data


@dataclass(frozen=True)
class Delivery:
    root: Path
    h2_urdf: Path | None
    h2_runtime: Path | None
    h3_urdf: Path
    h3_runtime: Path


def _delivery_at(session):
    final = session / 'delivery'
    urdf = final / 'calibrated.urdf'
    runtime = final / 'runtime_dynamics.json'
    for path in (urdf, runtime):
        if not path.is_file():
            raise ValueError(f'缺少交付文件 {path}')

    h2 = session / 'h2/analysis'
    h2_urdf = h2 / 'candidate.urdf'
    h2_runtime = h2 / 'runtime_dynamics.json'
    if not (h2_urdf.is_file() and h2_runtime.is_file()):
        h2_urdf = h2_runtime = None
    return Delivery(session, h2_urdf, h2_runtime, urdf, runtime)


def resolve_verified_delivery(target_session, output_root, robot_id):
    if target_session is not None:
        return _delivery_at(Path(target_session).resolve())

    robot_dir = Path(output_root).resolve() / robot_id
    for path in sorted(robot_dir.glob('*/delivery/calibrated.urdf'), reverse=True):
        if (path.parent / 'runtime_dynamics.json').is_file():
            return _delivery_at(path.parent.parent)
    return None


def _check_session_robot(session, robot_id):
    metadata = session / 'session.json'
    if robot_id and metadata.is_file() and _read_json(metadata)['robot_id'] != robot_id:
        raise ValueError(f'目标会话与 --robot={robot_id} 不一致')


def _load_feedforward(urdf, sidecar, robot_config):
    runtime = _read_json(sidecar)
    model = _core.ArmDynamics.from_urdf(str(urdf))
    if (runtime['joint_names'] != list(NAMES) or runtime['gravity_world'] != robot_config['gravity_world']
            or tuple(joint.name for joint in model.model.joints) != tuple(NAMES)):
        raise ValueError(f'交付模型与当前轴序或重力不匹配：{urdf}')
    friction = runtime['friction']
    friction['static_friction'] = list(friction['coulomb'])
    friction['smoothing_velocity'] = [
        max(value, SMOOTHING_FLOOR) for value in friction['smoothing_velocity']
    ]
    return _restore_runtime(model, runtime), runtime


def _feedforward_models(robot_config, delivery):
    nominal, _ = model_for(robot_config)
    if delivery is None:
        return (None, None, nominal, nominal, nominal), None, '离线预览使用名义模型'

    h3, h3_runtime = _load_feedforward(delivery.h3_urdf, delivery.h3_runtime, robot_config)
    if delivery.h2_urdf is None:
        h2 = nominal
        comparison = '缺少 H2 对照模型，G2 使用名义前馈'
    else:
        h2, h2_runtime = _load_feedforward(delivery.h2_urdf, delivery.h2_runtime, robot_config)
        comparison = ('H2 与最终模型采用相同的前馈摩擦参数'
                      if h2_runtime['friction'] == h3_runtime['friction']
                      else 'H2 与最终模型的前馈摩擦不同，G2–G3 同时比较摩擦与刚体模型')

    recommended = dict(
        calibrated_urdf=str(delivery.h3_urdf),
        calibrated_urdf_sha256=sha256(delivery.h3_urdf.read_bytes()).hexdigest(),
        feedforward_smoothing_velocity_floor_rad_s=SMOOTHING_FLOOR,
        **{key: h3_runtime[key] for key in ('joint_names', 'gravity_world', 'rotor_inertia', 'friction')},
    )
    return (None, None, nominal, h2, h3), recommended, comparison


def _ns(seconds):
    return round(seconds * 1e9)


def load_clips(path=CLIP_ASSET):
    clips = _read_json(path)
    for clip in clips.values():
        clip['states'] = np.asarray(clip['states'], dtype=float) * CLIP_STATE_RESOLUTION
    return clips


def _append_clip(motion, clip, label):
    begin = _ns(motion.duration)
    states = np.asarray(clip['states'], dtype=float)
    if not np.array_equal(motion.states[-1], states[0]):
        raise ValueError(f'{label} 与转场的端点状态不连续')
    motion.knots.extend((begin + t) * 1e-9 for t in clip['knots_ns'][1:])
    motion.states.extend(states[1:])
    motion.labels.extend([label] * (len(states) - 1))
    return [begin, _ns(motion.duration)]


def _instrument_transition(motion, target, label):
    lift = np.zeros(14)
    lift[[0, 7]] = -.16  # Shoulder pitch
    lift[[3, 10]] = -.12  # Elbow pitch
    for waypoint in (motion.states[-1][0] + lift, target + lift, target):
        _move(motion, waypoint, 1.0, TRANSITION, label)


def _formal_motion(clips):
    keyboard, drum = (clips[f'{name}_clip'] for name in INSTRUMENTS)
    q_ready = ready_position()
    q_anchor = keyboard['states'][0][0]
    motion = Motion('band_dynamics_verification', 'verification', q_ready)
    _move(motion, q_anchor, 1.5, dict(TRANSITION, speed_rad_s=READY_TO_ANCHOR_SPEED_RAD_S), 'ready_to_anchor')
    groups = {}
    for name in GROUPS:
        switch_ns = _ns(motion.duration)
        motion.add(HOLD_S, q_anchor, label='group_switch')
        keyboard_ns = _append_clip(motion, keyboard, 'keyboard')
        _instrument_transition(motion, drum['states'][0][0], 'keyboard_to_drum')
        drum_ns = _append_clip(motion, drum, 'drum')
        _instrument_transition(motion, q_anchor, 'drum_to_anchor')
        groups[name] = dict(switch_ns=switch_ns, run_ns=[keyboard_ns[0], _ns(motion.duration)],
                            keyboard=keyboard_ns, drum=drum_ns)
    _move(motion, q_ready, 1.5, dict(TRANSITION, speed_rad_s=READY_TO_ANCHOR_SPEED_RAD_S), 'anchor_to_ready')
    return motion, groups


def _effort_table(motion, groups, models, limit):
    curve = motion.polynomial()
    last_ns = _ns(motion.duration)
    anchors = [0, last_ns]
    for row in groups.values():
        anchors.extend([row['switch_ns'], *row['run_ns']])
    times = np.unique(np.r_[np.arange(0, last_ns, EFFORT_STEP_NS, dtype=np.int64), anchors])
    torque = np.zeros((len(times), 14))
    for i, name in enumerate(GROUPS):
        model = models[i]
        row = groups[name]
        start, stop = row['run_ns']
        if i == len(GROUPS) - 1:
            stop = last_ns  # Hold the last model through the return to ready.
        motion_mask = (times >= start) & (times <= stop)
        t = times[motion_mask] * 1e-9
        if model is not None:
            torque[motion_mask] = model.evaluate_batch(curve(t), curve(t, nu=1), curve(t, nu=2))['total']

        switch = row['switch_ns']
        blend = (times >= switch) & (times < start)
        initial = torque[np.searchsorted(times, switch)].copy()
        target = torque[np.searchsorted(times, start)]
        u = (times[blend] - switch) / (start - switch)
        s = u ** 3 * (10 + u * (-15 + 6 * u))
        torque[blend] = initial + s[:, None] * (target - initial)

    if not np.isfinite(torque).all() or np.max(np.abs(torque)) > limit:
        raise ValueError('验收参考前馈超出配置力矩上限')
    codes = np.trunc(torque * np.asarray(DIRECTION) * 100.0).astype(np.int64)
    keep = np.r_[True, np.any(codes[1:] != codes[:-1], axis=1)]
    return [dict(t_ns=int(t), value=v.tolist()) for t, v in zip(times[keep], torque[keep])]


def plan_verification(q_start, robot_config, output, *, delivery=None, clips=None):
    if clips is None:
        clips = load_clips()
    formal, groups = _formal_motion(clips)
    models, recommended, comparison = _feedforward_models(robot_config, delivery)
    markers = [
        dict(t_ns=stamp, event=f'hv_{group}_{abbr}_{end}')
        for group, row in groups.items()
        for instrument, abbr in (('keyboard', 'kbd'), ('drum', 'drum'))
        for stamp, end in zip(row[instrument], ('begin', 'end'))
    ]
    fixture = trical_profile(context_for(robot_config).bundle, None)
    bundle, motion, proof, _, offset, kinematics = compose_stage_reference(
        formal, q_start, models[2], robot_config, feedforward_mode='none', markers=markers,
        collision_profile=fixture,
    )
    speed_bounds = derivative_bounds(motion.polynomial()).max(axis=0)
    # Add feedback headroom above the worst planned joint, within the field limit.
    observed_limit = min(MAX_OBSERVED_VELOCITY_RAD_S,
                         math.ceil(10 * OBSERVED_VELOCITY_MARGIN * speed_bounds.max()) / 10)
    bundle['observed_velocity_limit_rad_s'] = observed_limit
    bundle['schema'] = 'casbot_shm_retreat_reference_v1'
    for i, name in enumerate(GROUPS):
        row = groups[name]
        row['switch_ns'] += offset
        for key in ('run_ns', *INSTRUMENTS):
            row[key] = [offset + t for t in row[key]]
        factor = 1.0 if i == 0 else 0.5
        kp = np.asarray(robot_config['kp'], dtype=float) * factor
        kd = np.asarray(robot_config['kd'], dtype=float)
        bundle['pd_schedule'].extend(
            dict(t_ns=t, kp=kp.tolist(), kd=kd.tolist()) for t in row['run_ns']
        )
        # Match the executor's 0.1-unit fixed-point gain fields.
        row['kp'] = (np.trunc(kp * 10) / 10).tolist()
        row['kd'] = (np.trunc(kd * 10) / 10).tolist()
    bundle['soft_stop_checkpoints_ns'] = [
        offset, *(row['switch_ns'] for row in groups.values()), groups[GROUPS[-1]]['run_ns'][1],
    ]
    bundle['effort'] = _effort_table(motion, groups, models, robot_config['feedforward_limit_nm'])
    if recommended is not None:
        recommended.update({key: groups['G3_soft'][key] for key in ('kp', 'kd')})
    plan = dict(
        duration_ns=bundle['knots_ns'][-1], calibration_begin_ns=offset,
        robot_config=robot_config, groups=groups, collision_certificate=proof, kinematics=kinematics,
        fixture_assembly_sha256=fixture.metadata['assembly_manifest_sha256'],
        reference_speed_bounds_rad_s=dict(zip(NAMES, speed_bounds.tolist())),
        observed_velocity_limit_rad_s=observed_limit,
        clips={name: clip['source'] for name, clip in clips.items()},
        delivery_session=str(delivery.root) if delivery else None,
        recommended=recommended, h2_comparison=comparison,
    )
    save_bundle(output, bundle)
    write_json(Path(output) / 'plan.json', plan)
    return plan


def _window_samples(data, bounds_ns):
    relative = data['relative_ns']
    start, stop = bounds_ns
    inside = (relative >= start) & (relative < stop)
    ns, inverse, count = np.unique(relative[inside], return_inverse=True, return_counts=True)
    if not len(ns):
        raise ValueError('窗口没有反馈样本')
    rows = np.column_stack([data[key][inside] for key in ('q', 'dq', 'effort_raw')])
    merged = np.zeros((len(ns), rows.shape[1]))
    np.add.at(merged, inverse, rows)
    q, dq, effort = np.split(merged / count[:, None], 3, axis=1)
    return ns * 1e-9, q, dq, effort


def _time_weights(t):
    if not len(t):
        return np.empty(0)
    dt = np.diff(t) / 2
    return np.r_[dt, 0.] + np.r_[0., dt]


def _rmse(values, weights):
    return np.sqrt(np.average(values ** 2, axis=0, weights=weights))


def _p95(values, weights):
    order = np.argsort(values)
    cumulative = np.cumsum(weights[order])
    return float(values[order[np.searchsorted(cumulative, 0.95 * cumulative[-1])]])


def _scalar_errors(values, weights, *, percentile=False):
    columns = values.reshape(len(weights), -1)
    result = dict(rmse=float(np.sqrt(np.mean(_rmse(columns, weights) ** 2))), max=float(np.max(np.abs(columns))))
    if percentile:
        result['p95'] = _p95(columns.ravel(), np.repeat(weights, columns.shape[1]))
    return result


LAG_GRID_MS = np.arange(-80.0, 80.5, 0.5)


def _window_metrics(observations, bounds_ns, curve, gains, nominal):
    elapsed, q, dq, _ = _window_samples(observations, bounds_ns)
    q_ref, dq_ref = curve(elapsed), curve(elapsed, nu=1)
    joint_error = q_ref - q
    velocity_error = dq - dq_ref
    flange_pos, flange_rot = [], []
    for frame in ('left_flange_frame', 'right_flange_frame'):
        poses = nominal.frame_batch(np.concatenate((q, q_ref)), frame)['poses']
        measured, desired = np.split(poses, 2)
        flange_pos.append(1000 * np.linalg.norm(measured[:, :3, 3] - desired[:, :3, 3], axis=1))
        # Trace of R_measured.T R_ref, without materializing each product.
        cosine = (np.sum(measured[:, :3, :3] * desired[:, :3, :3], axis=(1, 2)) - 1) / 2
        flange_rot.append(np.rad2deg(np.arccos(np.clip(cosine, -1, 1))))

    grid_ns = np.arange(bounds_ns[0], bounds_ns[1], 1_000_000, dtype=np.int64)
    central = grid_ns[(grid_ns >= bounds_ns[0] + 300_000_000) & (grid_ns < bounds_ns[1] - 300_000_000)]
    t = central * 1e-9
    lags_s = LAG_GRID_MS * 1e-3
    t = t[(t + lags_s[0] >= elapsed[0]) & (t + lags_s[-1] <= elapsed[-1])]
    lag_cost = None
    if len(t):
        q_interp = interp1d(elapsed, q, axis=0, assume_sorted=True)
        reference = curve(t)
        lag_cost = np.array([np.sum((q_interp(t + lag) - reference) ** 2) for lag in lags_s])
    grid_s = grid_ns * 1e-9
    grid_s = grid_s[(grid_s >= elapsed[0]) & (grid_s <= elapsed[-1])]
    jitter = np.empty((0, q.shape[1]))
    if len(grid_s) > 1:
        uniform = interp1d(elapsed, velocity_error, axis=0, assume_sorted=True)(grid_s)
        sos = butter(4, 15.0, btype='high', fs=1000.0, output='sos')
        pad = min(3 * (2 * len(sos) + 1), len(grid_s) - 1)
        jitter = sosfiltfilt(sos, uniform, axis=0, padlen=pad)
    else:
        grid_s = grid_s[:0]

    return dict(weights=_time_weights(elapsed), jitter_weights=_time_weights(grid_s),
                flange_pos=np.column_stack(flange_pos), flange_rot=np.column_stack(flange_rot),
                joint=np.rad2deg(joint_error), jitter=jitter, lag_cost=lag_cost,
                pd=np.asarray(gains['kp']) * joint_error - np.asarray(gains['kd']) * velocity_error)


def _summarize(windows):
    data = {key: np.concatenate([window[key] for window in windows]) for key in windows[0] if key != 'lag_cost'}
    weights = data['weights']
    duration = float(weights.sum())
    if not duration:
        weights = np.ones_like(weights)
    flange = _scalar_errors(data['flange_pos'], weights, percentile=True)
    joint = _scalar_errors(data['joint'], weights)
    near = [0, 1, 2, 3, 7, 8, 9, 10]
    far = [4, 5, 6, 11, 12, 13]
    costs = [window['lag_cost'] for window in windows if window['lag_cost'] is not None]
    lag = None
    if costs:
        lag_cost = np.sum(costs, axis=0)
        lags = LAG_GRID_MS[lag_cost == lag_cost.min()]
        lag = float(lags[np.argmin(np.abs(lags))])
    jitter = _rmse(data['jitter'], data['jitter_weights']) if len(data['jitter']) else None
    return dict(
        samples=len(weights), duration_s=duration,
        flange_pos_rmse_mm=flange['rmse'], flange_pos_p95_mm=flange['p95'], flange_pos_max_mm=flange['max'],
        flange_rot_rmse_deg=_scalar_errors(data['flange_rot'], weights)['rmse'],
        flange_by_arm={
            side: {**_scalar_errors(data['flange_pos'][:, j], weights, percentile=True),
                   'rotation_rmse_deg': float(_rmse(data['flange_rot'][:, j], weights))}
            for j, side in enumerate(('left', 'right'))
        },
        joint_rmse_deg=joint['rmse'], joint_max_deg=joint['max'],
        joint_proximal_deg=_scalar_errors(data['joint'][:, near], weights),
        joint_distal_deg=_scalar_errors(data['joint'][:, far], weights),
        phase_lag_ms=lag, phase_lag_at_grid_limit=lag is not None and abs(lag) == float(LAG_GRID_MS[-1]),
        jitter_rms_rad_s=float(np.sqrt(np.mean(jitter ** 2))) if jitter is not None else None,
        pd_effort_rmse_nm=_scalar_errors(data['pd'], weights)['rmse'],
        per_axis=[
            dict(id=i + 12, joint=name, joint_rmse_deg=float(rms), joint_max_deg=float(peak),
                 jitter_rms_rad_s=float(jitter[i]) if jitter is not None else None)
            for i, (name, rms, peak) in enumerate(zip(
                NAMES, _rmse(data['joint'], weights), np.max(np.abs(data['joint']), axis=0),
            ))
        ],
    )


def _acceptance(groups):
    compared = ('G0_stiff', 'G0_soft', 'G1_soft', 'G3_soft')
    reasons = [f'{name}：没有可计算的反馈' for name in compared if not groups[name]['overall']['valid']]
    if reasons:
        return dict(accepted=False, verdict='UNAVAILABLE', reasons=reasons)

    target = groups['G3_soft']['overall']
    for name in ('G0_soft', 'G1_soft'):
        for key in ('flange_pos_rmse_mm', 'joint_rmse_deg'):
            base = groups[name]['overall'][key]
            if target[key] >= base:
                reasons.append(f'G3_soft {key}={target[key]:.5g} 须低于 {name} 的 {base:.5g}')
    stiff = groups['G0_stiff']['overall']
    if target['flange_pos_rmse_mm'] > stiff['flange_pos_rmse_mm']:
        reasons.append('G3_soft 的法兰位置 RMSE 高于 G0_stiff')
    return dict(accepted=not reasons, verdict='FAIL' if reasons else 'PASS', reasons=reasons)


def _write_report(verify_dir, plan, report):
    analysis = verify_dir / 'analysis'
    _archive(analysis)
    analysis.mkdir()
    write_json(analysis / 'report.json', report)
    if report['accepted']:
        (analysis / 'recommended_band_gains.yaml').write_text(
            yaml.safe_dump(plan['recommended'], sort_keys=False, allow_unicode=True), encoding='utf-8',
        )

    lines = ['# 双臂动力学闭环验收', '',
             f'- 结论 **{report["verdict"]}**',
             f'- 交付来源 `{plan["delivery_session"]}`',
             f'- 采集方式 {"私有物理 Mock" if plan.get("mock") else "裸法兰实机"}',
             f'- 参考时长 {plan["duration_ns"] * 1e-9:.2f} s']
    if report.get('h2_comparison'):
        lines.append(f'- G2–G3 对照：{report["h2_comparison"]}')
    if report.get('received_collision'):
        lines.append(received_clearance_line(report['received_collision']))
    lines.extend(f'- {reason}' for reason in report['reasons'])
    for scope, title in (('overall', '总表'), ('keyboard', '电子琴'), ('drum', '架子鼓')):
        lines.extend(['', f'## {title}', '',
                      '| 组 | 法兰 RMSE / P95 / 最大 (mm) | 关节 RMSE / 最大 (°) | 相位滞后 (ms) | 高频速度 RMS (rad/s) | PD 纠偏 RMS (Nm) |',
                      '|---|---:|---:|---:|---:|---:|'])
        for name, entries in report['groups'].items():
            row = entries[scope]
            if not row['valid']:
                lines.append(f'| {name} | — | — | — | — | — |')
                continue
            lag = row['phase_lag_ms']
            lag_text = f'{lag:+.1f}' if lag is not None else '—'
            if row['phase_lag_at_grid_limit']:
                lag_text += '*'
            jitter = row['jitter_rms_rad_s']
            jitter_text = f'{jitter:.6f}' if jitter is not None else '—'
            lines.append(
                f'| {name} | {row["flange_pos_rmse_mm"]:.3f} / {row["flange_pos_p95_mm"]:.3f} / {row["flange_pos_max_mm"]:.3f}'
                f' | {row["joint_rmse_deg"]:.3f} / {row["joint_max_deg"]:.3f} | {lag_text}'
                f' | {jitter_text} | {row["pd_effort_rmse_nm"]:.4f} |'
            )
    if any(row.get('phase_lag_at_grid_limit') for entries in report['groups'].values() for row in entries.values()):
        lines.extend(['', '* 相位搜索达到 ±80 ms 网格边界。'])
    if all(group['overall']['valid'] for group in report['groups'].values()):
        lines.extend(['', '## 分轴对比', '',
                      '| SHM ID | ' + ' | '.join(GROUPS) + ' |',
                      '|---|' + '---:|' * len(GROUPS)])
        for i in range(14):
            axes = [report['groups'][group]['overall']['per_axis'][i] for group in GROUPS]
            cells = []
            for row in axes:
                jitter = row['jitter_rms_rad_s']
                jitter_text = f'{jitter:.5f} rad/s' if jitter is not None else '—'
                cells.append(f'{row["joint_rmse_deg"]:.3f}° / {jitter_text}')
            lines.append(f'| {i + 12} | ' + ' | '.join(cells) + ' |')
    if report['accepted']:
        lines.extend(['', '推荐运行配置见 `analysis/recommended_band_gains.yaml`。'])
    (verify_dir / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def analyse_verification(verify_dir):
    result = _read_json(verify_dir / 'capture/result.json')
    if not result.get('execution_completed'):
        raise ValueError('验收采集未完成')
    plan_dir, capture_dir = resolve_effective_plan_and_capture(verify_dir / 'plan', verify_dir / 'capture')
    plan = _read_json(plan_dir / 'plan.json')
    robot = load_robot_config(plan['robot_config'])
    fixture = trical_profile(context_for(robot).bundle, None) if 'fixture_assembly_sha256' in plan else None
    observations, _, feedback_proof = read_verified_stage_capture(
        plan, capture_dir, robot, collision_profile=fixture, minimum_feedback_rows=0,
    )
    reference = _read_json(native_directory(capture_dir) / 'reference.json')
    curve = BPoly.from_derivatives(np.asarray(reference['knots_ns']) * 1e-9, reference['states'])
    nominal, _ = model_for(robot)

    groups = {}
    for name in GROUPS:
        row = plan['groups'][name]
        windows, entries = [], {}
        for instrument in INSTRUMENTS:
            try:
                window = _window_metrics(observations, row[instrument], curve, row, nominal)
            except ValueError as error:
                entries[instrument] = dict(valid=False, invalid_reason=str(error))
                continue
            windows.append(window)
            entries[instrument] = dict(valid=True, **_summarize([window]))
        entries['overall'] = (dict(valid=True, **_summarize(windows)) if windows
                              else dict(valid=False, invalid_reason='没有可计算的反馈'))
        groups[name] = entries
    report = dict(
        **_acceptance(groups), groups=groups,
        delivery_session=plan['delivery_session'],
        h2_comparison=plan.get('h2_comparison'),
        received_collision=feedback_proof,
        feedback_minimum_sampled_distance_m=feedback_proof['minimum_sampled_distance_m'],
    )
    _write_report(verify_dir, plan, report)
    return report


def _archive(directory):
    if directory.exists():
        directory.rename(directory.with_name(f'{directory.name}.previous_{time.monotonic_ns()}'))


def _complete_capture(verify_dir):
    try:
        return bool(_read_json(verify_dir / 'capture/result.json').get('execution_completed'))
    except (OSError, ValueError):
        return False


def _analysis_target(target, output_root, robot_id):
    if target != 'latest':
        target = Path(target).resolve()
        return target if (target / 'capture').is_dir() else target / 'verify'
    latest = output_root / 'latest_verify'
    if latest.is_dir() and _complete_capture(latest):
        verify = latest.resolve()
        if robot_id is None or verify.parent.parent == output_root / robot_id:
            return verify
    robot_id = robot_id or resolve_robot_id(output_root / 'state')
    for session in sorted((output_root / robot_id).glob('*/session.json'), reverse=True):
        current = session.parent / 'verify'
        candidates = [current, *sorted(
            session.parent.glob('verify.previous_*'), key=lambda path: int(path.name.rsplit('_', 1)[-1]),
            reverse=True,
        )]
        for candidate in candidates:
            if _complete_capture(candidate):
                return candidate
    raise ValueError(f'{robot_id} 没有已完成采集的动力学验收记录')


def _delivered_mock_config(verify_dir, delivery, robot):
    models, _, _ = _feedforward_models(robot, delivery)
    raw_friction = _read_json(delivery.h3_runtime)['friction']
    fixture = _read_json(ROOT / 'configs/shm/h1_mock.json')
    fixture.update(urdf=str(delivery.h3_urdf), gravity_world=robot['gravity_world'], stiction_enabled=True)
    fixture['initial_fields']['effort'] = models[-1].gravity(ready_position()).tolist()
    for name, key in (('viscous', 'viscous'), ('coulomb', 'coulomb'), ('static', 'static_friction')):
        fixture[f'friction_{name}'] = raw_friction[key]
    path = verify_dir / 'mock_input.json'
    write_json(path, fixture)
    return path


def execute_verification(delivery, robot, output_root, *, executor=None, mock=False, mock_config=None):
    endpoint = '/shm_mc'
    binary = check_native_binaries(executor=executor, mock=mock, endpoint=endpoint)
    if not mock:
        check_hardware_environment(endpoint=endpoint)
    verify_dir = delivery.root / 'verify'
    _archive(verify_dir)
    verify_dir.mkdir()
    if mock and mock_config is None:
        mock_config = _delivered_mock_config(verify_dir, delivery, robot)
    if not mock:
        ensure_ready_channel(output_root / 'state', binary, endpoint=endpoint, require_hand_background=False)

    def plan(q_start, plan_dir):
        meta = plan_verification(q_start, robot, plan_dir, delivery=delivery)
        meta['mock'] = mock
        write_json(plan_dir / 'plan.json', meta)
        return meta

    with maybe_mock_context(mock, verify_dir, binary, robot, mock_config=mock_config) as mock_ctx:
        active_endpoint = mock_ctx.endpoint if mock_ctx else endpoint
        peer = mock_ctx.process if mock_ctx else None
        q_start = native_snapshot(binary, active_endpoint, robot, verify_dir / 'snapshot', peer=peer)
        plan_dir = verify_dir / 'plan'
        initial = plan(q_start, plan_dir)
        print(f'五组参考通过预检，执行时长 {initial["duration_ns"] * 1e-9:.2f} s，采集目录 {verify_dir}', flush=True)
        code, summary, _, _, capture_dir = execute_with_start_replan(
            initial_plan_dir=plan_dir, initial_plan_meta=initial, replan_fn=plan,
            capture_root=verify_dir / 'capture', executor=binary, endpoint=active_endpoint,
            robot_config=robot, hand_background=None, peer=peer, soft_stop=True,
        )
    if code != 0 or not summary['execution_completed']:
        reason = summary.get('replan_error') or summary['attempts'][-1]['reason'] or f'returncode={code}'
        if reason == 'operator_soft_stop':
            raise RuntimeError(f'{soft_stop_status(capture_dir)}，采集记录：{verify_dir}。本次未生成验收报告。')
        raise RuntimeError(f'验收执行中止（{reason}），记录见 {verify_dir}')
    _update_latest_link(output_root, verify_dir, 'latest_verify')
    return verify_dir


def main(argv=None):
    parser = argparse.ArgumentParser(description='双臂动力学标定裸法兰闭环验收')
    parser.add_argument('session', nargs='?', type=Path, help='指定标定会话目录')
    parser.add_argument('--hold-body', action='store_true', help='验收期间原地保持腿、头与腰')
    parser.add_argument('--robot', metavar='ID', help='机器编号')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--plan-only', action='store_true', help='离线规划并检查完整参考')
    mode.add_argument('--reanalyse', nargs='?', const='latest', metavar='TARGET', help='离线重算验收报告')
    mode.add_argument('--mock', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--output', type=Path, help='仅规划时的输出目录')
    parser.add_argument('--output-root', type=Path, default=DEFAULT_OUTPUT_ROOT, help=argparse.SUPPRESS)
    parser.add_argument('--robot-config', type=Path, default=DEFAULT_ROBOT_CONFIG, help=argparse.SUPPRESS)
    parser.add_argument('--executor', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--mock-config', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.output and not args.plan_only:
        parser.error('--output 仅用于 --plan-only')
    if args.reanalyse and args.session and args.reanalyse != 'latest':
        parser.error('只需提供一个重算目标目录')
    if args.robot is not None and not SAFE_ID.fullmatch(args.robot):
        parser.error('--robot 仅限字母、数字、点、下划线、连字符')

    try:
        output_root = args.output_root.resolve()
        check_model_bundle()
        if args.reanalyse:
            verify_dir = _analysis_target(args.session or args.reanalyse, output_root, args.robot)
            _check_session_robot(verify_dir.parent, args.robot)
        else:
            robot = load_robot_config(args.robot_config, hold_body=args.hold_body)
            if args.session:
                _check_session_robot(args.session, args.robot)
            robot_id = args.robot or resolve_robot_id(output_root / 'state')
            delivery = resolve_verified_delivery(args.session, output_root, robot_id)
            if args.plan_only:
                preview = args.output or output_root / 'verify_plan_preview' / str(time.monotonic_ns())
                preview.parent.mkdir(parents=True, exist_ok=True)
                plan = plan_verification(ready_position(), robot, preview, delivery=delivery)
                print(f'五组参考通过离线检查（{plan["duration_ns"] * 1e-9:.2f} s）{preview.resolve()}')
                return 0
            if delivery is None:
                raise ValueError(f'{robot_id} 没有动力学交付模型，请先运行 bash calib/dynamics.sh')
            verify_dir = execute_verification(
                delivery, robot, output_root, executor=args.executor, mock=args.mock, mock_config=args.mock_config,
            )
        report = analyse_verification(verify_dir)
        print(f'验收 {report["verdict"]}，报告 {verify_dir / "report.md"}')
        return 0 if report['accepted'] else 1
    except KeyboardInterrupt:
        print('验收已中止。', file=sys.stderr, flush=True)
        return 130
    except (HandoffError, KeyError, OSError, RuntimeError, ValueError) as error:
        print(f'验收中止：{error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
