"""Bare-flange closed-loop verification of a delivered H1–H3 model."""

import argparse
from dataclasses import dataclass
from hashlib import sha256
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.interpolate import BPoly, interp1d
from scipy.signal import butter, sosfiltfilt
import yaml

from .. import _core
from ..control_policy import scale_pd
from ..data import write_json
from ..dynamic_export import _restore_runtime
from ..model_runtime import model_for
from ..shm_data import DIRECTION, NAMES, ROOT
from ..shm_observations import ObservationReader
from ..shm_ready import ready_position
from ..shm_reference import save_bundle
from ..shm_takeover import HandoffError
from ..staged_motion import Motion, _move
from ..stages.dynamics.common import (
    compose_stage_reference, read_verified_stage_capture, resolve_effective_plan_and_capture,
)
from .native import execute_with_start_replan, maybe_mock_context, native_snapshot
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
SMOOTHING_FLOOR = 0.015
EFFORT_STEP_NS = 5_000_000


def _read_json(path):
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'预期 JSON 对象：{path}')
    return data


@dataclass(frozen=True)
class Delivery:
    root: Path
    h2_urdf: Path
    h2_runtime: Path
    h3_urdf: Path
    h3_runtime: Path


def _stage_path(session, metadata, stage):
    record = metadata.get('stages', {}).get(stage, {})
    if record.get('status') != 'completed' or not record.get('accepted') or not record.get('fit_succeeded'):
        raise ValueError(f'{session.name} 的 {stage.upper()} 未通过')
    return session / record.get('calibration_json', f'{stage}/analysis/calibration.json')


def _upstream_path(session, metadata, calibration, stage, output_root, robot_config):
    source = calibration.get('prior_source')
    if source == f'session_{stage}':
        return session, metadata, _stage_path(session, metadata, stage)
    if source == f'latest_{stage}' and calibration.get('prior_source_path'):
        path = Path(calibration['prior_source_path']).resolve()
        robot_dir = output_root / metadata['robot_id']
        if path.is_relative_to(robot_dir):
            upstream_session = robot_dir / path.relative_to(robot_dir).parts[0]
            upstream = _read_json(upstream_session / 'session.json')
            if (upstream.get('robot_id') == metadata['robot_id']
                    and upstream.get('model_context') == robot_config['model_context']
                    and upstream.get('family', 'dynamics') == 'dynamics'
                    and not upstream.get('plan_only')
                    and _stage_path(upstream_session, upstream, stage).resolve() == path):
                return upstream_session, upstream, path
    raise ValueError(f'找不到实际使用的 {stage.upper()} 上游产物')


def _delivery_at(session, output_root, robot_id, robot_config):
    meta = _read_json(session / 'session.json')
    bundle = _read_json(session / 'delivery/calibration_bundle.json')
    if (meta.get('family', 'dynamics') != 'dynamics' or meta.get('plan_only')
            or meta.get('robot_id') != robot_id or bundle.get('robot_id') != robot_id
            or meta.get('model_context') != robot_config['model_context']
            or bundle.get('model_context') != robot_config['model_context']
            or bundle.get('gravity_world') != robot_config['gravity_world']
            or bundle.get('urdf_reload_passed') is not True
            or bundle.get('selected_calibration_source') != 'session_h3'):
        raise ValueError('交付包的机器、模型、重力或采纳阶段不符合要求')

    provenance = bundle.get('provenance', {})
    if provenance.get('inertia') != 'session_h3' or provenance.get('rotor_inertia') != 'session_h3':
        raise ValueError('交付包缺少 H3 惯量来源')

    h3 = _read_json(_stage_path(session, meta, 'h3'))
    h2_session, h2_meta, h2_path = _upstream_path(session, meta, h3, 'h2', output_root, robot_config)
    h2 = _read_json(h2_path)
    if provenance.get('friction') != h3.get('prior_source'):
        raise ValueError('交付摩擦来源与 H3 实际采用的 H2 不一致')
    # The bundle's gravity label describes the local session. For cross-session
    # calibration, the H2 input record names the H1 actually used.
    h1_session, _, h1_path = _upstream_path(h2_session, h2_meta, h2, 'h1', output_root, robot_config)
    h1 = _read_json(h1_path)
    gravity_source = provenance.get('gravity')
    if gravity_source not in ('session_h1', 'latest_h1') and not (
        h1_session != session and gravity_source == 'session_h3'
    ):
        raise ValueError('交付包缺少 H1 重力来源')
    for stage, cal in (('h1', h1), ('h2', h2), ('h3', h3)):
        if (cal.get('accepted') is not True or cal.get('model_context') != robot_config['model_context']
                or cal.get('gravity_world') != robot_config['gravity_world']):
            raise ValueError(f'{stage.upper()} 上游标定与当前模型或重力不一致')

    delivery = Delivery(
        session, h2_path.parent / 'candidate.urdf', h2_path.parent / 'runtime_dynamics.json',
        session / 'delivery/calibrated.urdf', session / 'delivery/runtime_dynamics.json',
    )
    for path in (delivery.h2_urdf, delivery.h2_runtime, delivery.h3_urdf, delivery.h3_runtime):
        if not path.is_file():
            raise ValueError(f'缺少交付文件 {path}')
    return delivery


def resolve_verified_delivery(target_session, output_root, robot_id, robot_config):
    output_root = Path(output_root).resolve()
    if target_session is not None:
        session = Path(target_session).resolve()
        try:
            return _delivery_at(session, output_root, robot_id, robot_config)
        except (ValueError, OSError) as error:
            raise ValueError(f'{session} 缺少完整通过的 H1–H3 交付链（{error}）') from error

    folder = output_root / robot_id
    for bundle in sorted(folder.glob('*/delivery/calibration_bundle.json'), reverse=True):
        try:
            return _delivery_at(bundle.parent.parent, output_root, robot_id, robot_config)
        except (ValueError, OSError):
            continue
    return None


def _feedforward_models(robot_config, delivery):
    nominal, _ = model_for(robot_config)
    if delivery is None:
        return (None, None, nominal, nominal, nominal), None

    h2_runtime, h3_runtime = (_read_json(path) for path in (delivery.h2_runtime, delivery.h3_runtime))
    for runtime in (h2_runtime, h3_runtime):
        if runtime['joint_names'] != list(NAMES) or runtime['gravity_world'] != robot_config['gravity_world']:
            raise ValueError('交付 runtime_dynamics.json 的轴序或重力不匹配')
        friction = runtime['friction']
        friction['smoothing_velocity'] = [
            max(value, SMOOTHING_FLOOR) for value in friction['smoothing_velocity']
        ]
    if h2_runtime['friction'] != h3_runtime['friction']:
        raise ValueError('H2 与 H3 的前馈摩擦不同，无法对照惯量辨识')
    h2 = _restore_runtime(_core.ArmDynamics.from_urdf(str(delivery.h2_urdf)), h2_runtime)
    h3 = _restore_runtime(_core.ArmDynamics.from_urdf(str(delivery.h3_urdf)), h3_runtime)
    recommended = dict(
        calibrated_urdf=str(delivery.h3_urdf),
        calibrated_urdf_sha256=sha256(delivery.h3_urdf.read_bytes()).hexdigest(),
        feedforward_smoothing_velocity_floor_rad_s=SMOOTHING_FLOOR,
        **{key: h3_runtime[key] for key in ('joint_names', 'gravity_world', 'rotor_inertia', 'friction')},
    )
    return (None, None, nominal, h2, h3), recommended


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


def _formal_motion(clips):
    keyboard, drum = (clips[f'{name}_clip'] for name in INSTRUMENTS)
    q_ready = ready_position()
    q_anchor = keyboard['states'][0][0]
    motion = Motion('band_dynamics_verification', 'verification', q_ready)
    _move(motion, q_anchor, 1.5, TRANSITION, 'ready_to_anchor')
    groups = {}
    for name in GROUPS:
        switch_ns = _ns(motion.duration)
        motion.add(HOLD_S, q_anchor, label='group_switch')
        keyboard_ns = _append_clip(motion, keyboard, 'keyboard')
        _move(motion, drum['states'][0][0], 1.5, TRANSITION, 'keyboard_to_drum')
        drum_ns = _append_clip(motion, drum, 'drum')
        _move(motion, q_anchor, 1.5, TRANSITION, 'drum_to_anchor')
        groups[name] = dict(switch_ns=switch_ns, run_ns=[keyboard_ns[0], _ns(motion.duration)],
                            keyboard=keyboard_ns, drum=drum_ns)
    _move(motion, q_ready, 1.5, TRANSITION, 'anchor_to_ready')
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
    models, recommended = _feedforward_models(robot_config, delivery)
    markers = [
        dict(t_ns=stamp, event=f'hv_{group}_{abbr}_{end}')
        for group, row in groups.items()
        for instrument, abbr in (('keyboard', 'kbd'), ('drum', 'drum'))
        for stamp, end in zip(row[instrument], ('begin', 'end'))
    ]
    bundle, motion, proof, _, offset, kinematics = compose_stage_reference(
        formal, q_start, models[2], robot_config, feedforward_mode='none', markers=markers,
    )
    for i, name in enumerate(GROUPS):
        row = groups[name]
        row['switch_ns'] += offset
        for key in ('run_ns', *INSTRUMENTS):
            row[key] = [offset + t for t in row[key]]
        kp, kd = scale_pd(robot_config['kp'], robot_config['kd'], 1.0 if i == 0 else 0.5)
        bundle['pd_schedule'].extend(
            dict(t_ns=t, kp=kp.tolist(), kd=kd.tolist()) for t in row['run_ns']
        )
        # Match the executor's 0.1-unit fixed-point gain fields.
        row['kp'] = (np.trunc(kp * 10) / 10).tolist()
        row['kd'] = (np.trunc(kd * 10) / 10).tolist()
    bundle['effort'] = _effort_table(motion, groups, models, robot_config['feedforward_limit_nm'])
    if recommended is not None:
        recommended.update({key: groups['G3_soft'][key] for key in ('kp', 'kd')})
    plan = dict(
        duration_ns=bundle['knots_ns'][-1], calibration_begin_ns=offset,
        robot_config=robot_config, groups=groups, collision_certificate=proof, kinematics=kinematics,
        clips={name: clip['source'] for name, clip in clips.items()},
        delivery_session=str(delivery.root) if delivery else None,
        recommended=recommended,
    )
    save_bundle(output, bundle)
    write_json(Path(output) / 'plan.json', plan)
    return plan


def _window_samples(reader, bounds_ns):
    start, stop = np.asarray(bounds_ns) * 1e-9
    data, elapsed, _ = reader.window((start, stop))
    inside = (elapsed >= start) & (elapsed < stop)
    ns, inverse, count = np.unique(data['observation_time_ns'][inside], return_inverse=True, return_counts=True)
    elapsed = (ns - reader.origin) * 1e-9
    if len(ns) < max(50, (bounds_ns[1] - bounds_ns[0]) // 2_000_000):
        raise ValueError(f'窗口唯一时间戳不足（{len(ns)} 个）')
    gap = np.diff(ns).max()
    if gap > 20_000_000:
        raise ValueError(f'窗口样本间隔 {gap * 1e-6:.1f} ms 超过 20 ms')
    if ns[0] - reader.origin - bounds_ns[0] > 10_000_000 or bounds_ns[1] - (ns[-1] - reader.origin) > 10_000_000:
        raise ValueError('窗口首末覆盖缺口超过 10 ms')

    rows = np.column_stack([data[key][inside] for key in ('q', 'dq', 'effort_raw')])
    merged = np.zeros((len(ns), rows.shape[1]))
    np.add.at(merged, inverse, rows)
    q, dq, effort = np.split(merged / count[:, None], 3, axis=1)
    return elapsed, q, dq, effort


def _time_weights(t):
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


def _window_metrics(reader, bounds_ns, curve, gains, nominal):
    elapsed, q, dq, _ = _window_samples(reader, bounds_ns)
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
    q_interp = interp1d(elapsed, q, axis=0, assume_sorted=True)
    reference = curve(t)
    lag_cost = np.array([np.sum((q_interp(t + lag * 1e-3) - reference) ** 2) for lag in LAG_GRID_MS])
    grid_s = grid_ns * 1e-9
    grid_s = grid_s[(grid_s >= elapsed[0]) & (grid_s <= elapsed[-1])]
    uniform = interp1d(elapsed, velocity_error, axis=0, assume_sorted=True)(grid_s)
    jitter = sosfiltfilt(butter(4, 15.0, btype='high', fs=1000.0, output='sos'), uniform, axis=0)

    return dict(weights=_time_weights(elapsed), jitter_weights=_time_weights(grid_s),
                flange_pos=np.column_stack(flange_pos), flange_rot=np.column_stack(flange_rot),
                joint=np.rad2deg(joint_error), jitter=jitter, lag_cost=lag_cost,
                pd=np.asarray(gains['kp']) * joint_error - np.asarray(gains['kd']) * velocity_error)


def _summarize(windows):
    data = {key: np.concatenate([window[key] for window in windows]) for key in windows[0] if key != 'lag_cost'}
    weights = data['weights']
    flange = _scalar_errors(data['flange_pos'], weights, percentile=True)
    joint = _scalar_errors(data['joint'], weights)
    near = [0, 1, 2, 3, 7, 8, 9, 10]
    far = [4, 5, 6, 11, 12, 13]
    lag_cost = np.sum([window['lag_cost'] for window in windows], axis=0)
    lags = LAG_GRID_MS[lag_cost == lag_cost.min()]
    lag = float(lags[np.argmin(np.abs(lags))])
    jitter = _rmse(data['jitter'], data['jitter_weights'])
    return dict(
        samples=len(weights), duration_s=float(weights.sum()),
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
        phase_lag_ms=lag, phase_lag_at_grid_limit=abs(lag) == float(LAG_GRID_MS[-1]),
        jitter_rms_rad_s=float(np.sqrt(np.mean(jitter ** 2))),
        pd_effort_rmse_nm=_scalar_errors(data['pd'], weights)['rmse'],
        per_axis=[
            dict(id=i + 12, joint=name, joint_rmse_deg=float(rms), joint_max_deg=float(peak), jitter_rms_rad_s=float(jit))
            for i, (name, rms, peak, jit) in enumerate(zip(
                NAMES, _rmse(data['joint'], weights), np.max(np.abs(data['joint']), axis=0), jitter,
            ))
        ],
    )


def _acceptance(groups):
    reasons = []
    for name, entries in groups.items():
        for instrument in INSTRUMENTS:
            if not entries[instrument]['valid']:
                reasons.append(f'{name} / {instrument}：{entries[instrument]["invalid_reason"]}')
    if reasons:
        return dict(accepted=False, verdict='INVALID', reasons=reasons)

    target = groups['G3_soft']['overall']
    for name in ('G0_soft', 'G1_soft'):
        for key in ('flange_pos_rmse_mm', 'joint_rmse_deg'):
            base = groups[name]['overall'][key]
            if target[key] >= base:
                reasons.append(f'G3_soft {key}={target[key]:.5g} 须低于 {name} 的 {base:.5g}')
    stiff = groups['G0_stiff']['overall']
    if target['flange_pos_rmse_mm'] > stiff['flange_pos_rmse_mm']:
        reasons.append('G3_soft 的法兰位置 RMSE 高于 G0_stiff')
    if target['jitter_rms_rad_s'] >= stiff['jitter_rms_rad_s']:
        reasons.append('G3_soft 的高频速度残差 RMS 未低于 G0_stiff')
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
            marker = '*' if row['phase_lag_at_grid_limit'] else ''
            lines.append(
                f'| {name} | {row["flange_pos_rmse_mm"]:.3f} / {row["flange_pos_p95_mm"]:.3f} / {row["flange_pos_max_mm"]:.3f}'
                f' | {row["joint_rmse_deg"]:.3f} / {row["joint_max_deg"]:.3f} | {row["phase_lag_ms"]:+.1f}{marker}'
                f' | {row["jitter_rms_rad_s"]:.6f} | {row["pd_effort_rmse_nm"]:.4f} |'
            )
    if any(row.get('phase_lag_at_grid_limit') for entries in report['groups'].values() for row in entries.values()):
        lines.extend(['', '* 相位搜索达到 ±80 ms 网格边界。'])
    if all(group['overall']['valid'] for group in report['groups'].values()):
        lines.extend(['', '## 分轴对比', '',
                      '| SHM ID | ' + ' | '.join(GROUPS) + ' |',
                      '|---|' + '---:|' * len(GROUPS)])
        for i in range(14):
            axes = [report['groups'][group]['overall']['per_axis'][i] for group in GROUPS]
            cells = [f'{row["joint_rmse_deg"]:.3f}° / {row["jitter_rms_rad_s"]:.5f} rad/s' for row in axes]
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
    _, _, feedback_proof = read_verified_stage_capture(plan, capture_dir, robot)
    reader = ObservationReader(capture_dir)
    reference = _read_json(reader.root / 'reference.json')
    curve = BPoly.from_derivatives(np.asarray(reference['knots_ns']) * 1e-9, reference['states'])
    nominal, _ = model_for(robot)

    groups = {}
    for name in GROUPS:
        row = plan['groups'][name]
        windows, entries = [], {}
        for instrument in INSTRUMENTS:
            try:
                window = _window_metrics(reader, row[instrument], curve, row, nominal)
            except ValueError as error:
                entries[instrument] = dict(valid=False, invalid_reason=str(error))
                continue
            windows.append(window)
            entries[instrument] = dict(valid=True, **_summarize([window]))
        entries['overall'] = (dict(valid=True, **_summarize(windows)) if len(windows) == len(INSTRUMENTS)
                              else dict(valid=False, invalid_reason='存在无效子窗'))
        groups[name] = entries
    report = dict(
        **_acceptance(groups), groups=groups,
        delivery_session=plan['delivery_session'],
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
    models, recommended = _feedforward_models(robot, delivery)
    fixture = _read_json(ROOT / 'configs/shm/h1_mock.json')
    fixture.update(urdf=str(delivery.h3_urdf), gravity_world=robot['gravity_world'], stiction_enabled=True)
    fixture['initial_fields']['effort'] = models[-1].gravity(ready_position()).tolist()
    for name, key in (('viscous', 'viscous'), ('coulomb', 'coulomb'), ('static', 'static_friction')):
        fixture[f'friction_{name}'] = recommended['friction'][key]
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
        code, summary, _, _, _ = execute_with_start_replan(
            initial_plan_dir=plan_dir, initial_plan_meta=initial, replan_fn=plan,
            capture_root=verify_dir / 'capture', executor=binary, endpoint=active_endpoint,
            robot_config=robot, hand_background=None, peer=peer,
        )
    if code != 0 or not summary['execution_completed']:
        reason = summary.get('replan_error') or summary['attempts'][-1]['reason'] or f'returncode={code}'
        raise RuntimeError(f'验收执行中止（{reason}），记录见 {verify_dir}')
    _update_latest_link(output_root, verify_dir, 'latest_verify')
    return verify_dir


def main(argv=None):
    parser = argparse.ArgumentParser(description='双臂动力学标定裸法兰闭环验收')
    parser.add_argument('session', nargs='?', type=Path, help='指定标定会话目录')
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
            if args.robot and _read_json(verify_dir.parent / 'session.json')['robot_id'] != args.robot:
                raise ValueError(f'目标会话与 --robot={args.robot} 不一致')
        else:
            robot = load_robot_config(args.robot_config)
            robot_id = args.robot
            if robot_id is None:
                robot_id = (_read_json(args.session / 'session.json')['robot_id'] if args.session
                            else resolve_robot_id(output_root / 'state'))
            delivery = resolve_verified_delivery(args.session, output_root, robot_id, robot)
            if args.plan_only:
                preview = args.output or output_root / 'verify_plan_preview' / str(time.monotonic_ns())
                preview.parent.mkdir(parents=True, exist_ok=True)
                plan = plan_verification(ready_position(), robot, preview, delivery=delivery)
                print(f'五组参考通过离线检查（{plan["duration_ns"] * 1e-9:.2f} s）{preview.resolve()}')
                return 0
            if delivery is None:
                raise ValueError(f'{robot_id} 缺少完整通过的 H1–H3 交付链，请先运行 bash calib/dynamics.sh')
            verify_dir = execute_verification(
                delivery, robot, output_root, executor=args.executor, mock=args.mock, mock_config=args.mock_config,
            )
        report = analyse_verification(verify_dir)
        print(f'验收 {report["verdict"]}，报告 {verify_dir / "report.md"}')
        return 0 if report['accepted'] else 1
    except (HandoffError, KeyError, OSError, RuntimeError, ValueError) as error:
        print(f'验收中止：{error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
