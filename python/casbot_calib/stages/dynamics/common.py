"""Shared helpers for SHM dynamics stages: automatic prior resolution, C3 reference composition, capture verification, and URDF export."""
import csv
import json
from pathlib import Path
import numpy as np
from ...collision import certify_polyline
from ...data import write_json
from ...dynamic_export import check_reload, write_inertial_urdf
from ...model_runtime import collision_for, model_for, urdf_for
from ...rigid_identification import model_from_parameters
from ...shm_data import DIRECTION, NAMES
from ...shm_fieldwork import text_result
from ...shm_observations import ObservationReader
from ...shm_ready import handover_fraction, pd_schedule, preparation, prepend
from ...shm_reference import export_motion
from ...staged_motion import Motion, derivative_bounds, gate_motion


def _load_valid_calibration(cal_path, robot_config):
    cal_path = Path(cal_path)
    if not cal_path.is_file():
        return None
    try:
        cal = json.loads(cal_path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None
    if cal.get('accepted') is False:
        return None
    if cal.get('model_context') != robot_config['model_context']:
        return None
    if len(cal.get('parameters', ())) != 140:
        return None
    for key in ('gravity_world', 'effort_gain', 'effort_zero_nm'):
        if key in cal and not np.allclose(cal[key], robot_config[key], rtol=0, atol=1e-12):
            return None
    return cal


def _find_upstream_calibration(session, stage_candidates, robot_config):
    if session is None:
        return None, 'nominal', None

    # 1. Same session first.
    for st_name in stage_candidates:
        rec = session.stage_record(st_name)
        if (
            rec
            and rec.get('status') == 'completed'
            and rec.get('fit_succeeded')
            and rec.get('accepted', True) is not False
            and rec.get('calibration_json')
        ):
            cal_path = session.root / rec['calibration_json']
            cal = _load_valid_calibration(cal_path, robot_config)
            if cal is not None:
                return cal, f'session_{st_name}', cal_path

    # 2. Latest completed session for the same robot_id.
    robot_dir = session.output_root / session.robot_id
    if robot_dir.is_dir():
        for candidate_dir in sorted((d for d in robot_dir.iterdir() if d.is_dir()), reverse=True):
            if candidate_dir.resolve() == session.root.resolve():
                continue
            sess_file = candidate_dir / 'session.json'
            if not sess_file.is_file():
                continue
            try:
                meta = json.loads(sess_file.read_text(encoding='utf-8'))
            except (OSError, ValueError):
                continue
            if meta.get('plan_only') or meta.get('model_context') != robot_config['model_context']:
                continue
            stages_meta = meta.get('stages') or {}
            for st_name in stage_candidates:
                rec = stages_meta.get(st_name)
                if (
                    rec
                    and rec.get('status') == 'completed'
                    and rec.get('fit_succeeded')
                    and rec.get('accepted', True) is not False
                    and rec.get('calibration_json')
                ):
                    cal_path = candidate_dir / rec['calibration_json']
                    cal = _load_valid_calibration(cal_path, robot_config)
                    if cal is not None:
                        return cal, f'latest_{st_name}', cal_path

    return None, 'nominal', None


def resolve_stage_prior(session, stage_name, robot_config):
    """Automatically resolve H1/H2 upstream calibration without manual CLI paths."""
    nominal, model_source = model_for(robot_config)
    model = nominal.model
    model.gravity_world = np.asarray(robot_config['gravity_world'], dtype=float)
    nominal = type(nominal)(model)
    nominal_rotor = np.array([float(j.rotor_inertia) for j in nominal.model.joints], dtype=float)

    candidates = ('h1',) if stage_name == 'h2' else ('h2', 'h1') if stage_name == 'h3' else ()
    cal, source_kind, source_path = _find_upstream_calibration(session, candidates, robot_config)

    if cal is not None:
        parameters = np.asarray(cal['parameters'], dtype=float)
        viscous = np.asarray(cal.get('viscous', [0.0] * 14), dtype=float)
        coulomb = np.asarray(cal.get('coulomb', [0.0] * 14), dtype=float)
        static_friction = np.asarray(cal.get('static_friction', coulomb), dtype=float)
        smoothing_raw = cal.get('smoothing_velocity', 0.001)
        smoothing = (
            np.asarray(smoothing_raw, dtype=float)
            if isinstance(smoothing_raw, (list, tuple, np.ndarray))
            else float(smoothing_raw)
        )
        stribeck_raw = cal.get('stribeck_velocity', 0.005)
        stribeck = (
            np.asarray(stribeck_raw, dtype=float)
            if isinstance(stribeck_raw, (list, tuple, np.ndarray))
            else float(stribeck_raw)
        )
        rotor_inertia = np.asarray(cal.get('rotor_inertia', nominal_rotor), dtype=float)
    else:
        parameters = np.asarray(nominal.rigid_parameters(), dtype=float)
        viscous = np.zeros(14, dtype=float)
        coulomb = np.zeros(14, dtype=float)
        static_friction = np.zeros(14, dtype=float)
        smoothing = 0.001
        stribeck = 0.005
        rotor_inertia = nominal_rotor.copy()

    engine = model_from_parameters(
        nominal,
        parameters,
        viscous=viscous,
        coulomb=coulomb,
        static=static_friction,
        smoothing_velocity=smoothing,
        stribeck_velocity=stribeck,
        rotor_inertia=rotor_inertia,
    )
    return dict(
        source=source_kind,
        source_path=str(source_path) if source_path is not None else None,
        parameters=parameters,
        viscous=viscous,
        coulomb=coulomb,
        static_friction=static_friction,
        smoothing_velocity=smoothing,
        stribeck_velocity=stribeck,
        rotor_inertia=rotor_inertia,
        engine=engine,
        nominal_engine=nominal,
        model_source=model_source,
    )


def build_feedforward_effort_rows(
    motion,
    engine,
    robot_config,
    preparation_cert=None,
    *,
    mode='gravity',
    step_ns=5_000_000,
):
    if mode not in ('none', 'gravity', 'nominal_reference', 'gravity_friction'):
        raise ValueError(f'未知前馈模式：{mode}')
    if mode == 'none':
        return [dict(t_ns=0, value=[0.0] * 14)]

    step = int(step_ns)
    if step <= 0:
        raise ValueError('step_ns 须为正整数纳秒')

    curve = motion.polynomial()
    last = round(motion.duration * 1e9)
    anchors = [0, last]
    if preparation_cert is not None:
        anchors.extend([
            int(preparation_cert['move_end_ns']),
            int(preparation_cert['handover_end_ns']),
            int(preparation_cert['calibration_begin_ns']),
        ])
    anchors = [t for t in anchors if 0 <= t <= last]
    times = np.unique(np.r_[np.arange(0, last + 1, step, dtype=np.int64), np.asarray(anchors, dtype=np.int64)])

    t_s = times * 1e-9
    q = np.asarray(curve(t_s), dtype=float)
    torque = np.asarray(engine.gravity_batch(q), dtype=float)
    if mode == 'gravity_friction':
        dq = np.asarray(curve(t_s, nu=1), dtype=float)
        viscous = np.array([j.friction.viscous for j in engine.model.joints], dtype=float)
        coulomb = np.array([j.friction.coulomb for j in engine.model.joints], dtype=float)
        torque = torque + viscous[None, :] * dq + coulomb[None, :] * np.tanh(dq / 0.02)

    if preparation_cert is not None:
        torque = torque * handover_fraction(times, preparation_cert)[:, None]

    if not np.isfinite(torque).all() or np.max(np.abs(torque)) > robot_config['feedforward_limit_nm']:
        raise ValueError('重力前馈超出显式力矩上限；不夹取')

    codes = np.trunc(torque * np.asarray(DIRECTION, dtype=float)[None, :] * 100.0).astype(np.int64)
    keep = np.r_[True, np.any(codes[1:] != codes[:-1], axis=1)]
    kept_times = times[keep].tolist()
    kept_torques = torque[keep].tolist()
    return [dict(t_ns=t, value=v) for t, v in zip(kept_times, kept_torques)]


def build_gravity_effort_rows(motion, engine, robot_config, preparation_cert=None, *, enabled=True):
    return build_feedforward_effort_rows(
        motion,
        engine,
        robot_config,
        preparation_cert,
        mode='gravity' if enabled else 'none',
    )


def ensure_leading_hold(motion, min_hold_s):
    end = 0
    for stamp, state in zip(motion.knots[1:], motion.states[1:]):
        if not np.array_equal(state, motion.states[0]):
            break
        end = round(stamp * 1e9)
    extra_ns = max(0, round(min_hold_s * 1e9) - end)
    if not extra_ns:
        return motion, 0
    prefix = Motion(motion.name, motion.stage, motion.home)
    prefix.add(extra_ns * 1e-9, motion.home, label='body_start_hold')
    return prepend(prefix, motion), extra_ns


def compose_stage_reference(
    formal_motion,
    q_start,
    prior_engine,
    robot_config,
    *,
    ready_pose=True,
    gravity_feedforward=True,
    feedforward_mode=None,
    markers=None,
    collision_profile=None,
    max_speed_cap=None,
    max_acceleration_cap=None,
    max_jerk_cap=None,
):
    geometry = collision_for(robot_config) if collision_profile is None else collision_profile
    formal_motion.knots = [round(t * 1e9) * 1e-9 for t in formal_motion.knots]
    hold = robot_config.get('non_arm_hold') or {}
    ramp = hold['ramp_s'] if hold.get('enabled') else 0.
    offset_ns = 0
    if ramp and not ready_pose:
        formal_motion, offset_ns = ensure_leading_hold(formal_motion, ramp)
    proof = gate_motion(geometry, formal_motion, robot_config['scene']['clearance_m'])
    if not proof['passed']:
        raise ValueError(f'轨迹连续碰撞或 URDF 限位检查未通过：{proof.get("reason", "collision_or_limits")}')

    curve = formal_motion.polynomial()
    speed_bounds = derivative_bounds(curve, 1)
    joints = prior_engine.model.joints
    per_joint_v_limit = [
        min(robot_config['maximum_velocity_rad_s'], j.velocity_limit, max_speed_cap or robot_config['maximum_velocity_rad_s'])
        for j in joints
    ]
    if any(speed_bounds[:, idx].max() > lim * (1 + 1e-9) for idx, lim in enumerate(per_joint_v_limit)):
        raise ValueError('参考轨迹连续速度包络超出上限')

    max_acc = float(np.max(derivative_bounds(curve, 2)))
    max_jerk = float(np.max(derivative_bounds(curve, 3)))
    if max_acceleration_cap is not None and max_acc > max_acceleration_cap * (1 + 1e-9):
        raise ValueError(f'参考轨迹连续加速度包络 {max_acc:.3f} 超出上限 {max_acceleration_cap:.3f} rad/s²')
    if max_jerk_cap is not None and max_jerk > max_jerk_cap * (1 + 1e-9):
        raise ValueError(f'参考轨迹连续加加速度包络 {max_jerk:.3f} 超出上限 {max_jerk_cap:.3f} rad/s³')

    prep_cert = None
    if ready_pose:
        prep, prep_cert = preparation(q_start, prior_engine, robot_config['maximum_velocity_rad_s'],
                                      start_hold_s=max(0.5, ramp))
        offset_ns = prep_cert['calibration_begin_ns']
        motion = prepend(prep, formal_motion)
    else:
        motion = formal_motion

    resolved_ff_mode = ('gravity' if gravity_feedforward else 'none') if feedforward_mode is None else str(feedforward_mode)
    effort = build_feedforward_effort_rows(
        motion,
        prior_engine,
        robot_config,
        prep_cert,
        mode=resolved_ff_mode,
    )
    bundle = export_motion(
        motion,
        kp=robot_config['kp'],
        kd=robot_config['kd'],
        effort=effort, non_arm_hold=hold,
        observed_velocity_limit_rad_s=robot_config['maximum_velocity_rad_s'],
    )
    bundle['purpose'] = 'shm_calib_ready_then_collision_checked' if prep_cert else 'shm_calib_collision_checked'
    bundle['observed_velocity_limit_rad_s'] = robot_config['maximum_velocity_rad_s']

    all_markers = []
    if prep_cert is not None:
        bundle['pd_schedule'] = pd_schedule(prep_cert, robot_config['kp'], robot_config['kd'])
        all_markers.extend([
            dict(t_ns=0, event='ready_pose_start'),
            dict(t_ns=prep_cert['move_end_ns'], event='ready_pose_reached'),
            dict(t_ns=prep_cert['handover_end_ns'], event='ready_handover_complete'),
            dict(t_ns=offset_ns, event='calibration_begin'),
        ])
    for row in markers or ():
        all_markers.append(dict(t_ns=offset_ns + int(row['t_ns']), event=row['event']))
    if all_markers:
        # Ensure strictly increasing marker timestamps.
        dedup = {}
        for m in sorted(all_markers, key=lambda item: item['t_ns']):
            dedup[m['t_ns']] = m['event']
        bundle['markers'] = [dict(t_ns=k, event=v) for k, v in sorted(dedup.items())]

    kinematics_summary = dict(
        maximum_velocity_rad_s=float(speed_bounds.max()),
        maximum_acceleration_rad_s2=max_acc,
        maximum_jerk_rad_s3=max_jerk,
    )
    return bundle, motion, proof, prep_cert, offset_ns, kinematics_summary


def resolve_effective_plan_and_capture(plan_dir, capture_dir, *, diagnostic=False):
    plan_dir = Path(plan_dir).resolve()
    capture_dir = Path(capture_dir).resolve()
    wrapper_file = capture_dir / 'result.json'
    if wrapper_file.is_file():
        wrapper = json.loads(wrapper_file.read_text(encoding='utf-8'))
        if isinstance(wrapper.get('effective_capture'), str):
            if not diagnostic and not wrapper.get('execution_completed'):
                raise ValueError('采集未完成，无法生成正式辨识结果')
            if wrapper.get('effective_plan'):
                plan_dir = (capture_dir / wrapper['effective_plan']).resolve()
            capture_dir = (capture_dir / wrapper['effective_capture']).resolve()
    return plan_dir, capture_dir


def stage_capture_complete(native, *, legacy_h1=False):
    return (native.get('execution_completed') == '1'
            and native.get('recording_stopped') == '0'
            and native.get('disable_ack', '1' if legacy_h1 else '0') == '1')


def read_verified_stage_capture(plan_meta, capture_dir, robot_config, *,
                                collision_profile=None, minimum_feedback_rows=10,
                                diagnostic=False, legacy_h1=False):
    reader = ObservationReader(capture_dir, allow_truncated=diagnostic)
    native = text_result(reader.root / 'result.txt') or {}
    if not diagnostic and not stage_capture_complete(native, legacy_h1=legacy_h1):
        raise ValueError(f'原生执行未正常完成或记录中断：{native}')

    arr = reader.arrays()
    if diagnostic:
        with (reader.root / 'events.csv').open(newline='') as events:
            stop_ns = next((int(row['mono_ns']) for row in csv.DictReader(events)
                            if row['event'] == 'soft_stop_return_begin'), None)
        keep = arr['phase'] == 3
        if stop_ns is not None:
            keep &= arr['receive_ns'] < stop_ns
        arr = {key: values[keep] for key, values in arr.items()}
    n_rows = int(len(arr['receive_ns']))
    if n_rows < minimum_feedback_rows:
        raise ValueError('SHM 有效反馈帧数过少')

    calib_begin_ns = int(plan_meta.get('calibration_begin_ns', 0))
    receive_ns = np.asarray(arr['receive_ns'], dtype=np.int64)
    obs_ns = np.asarray(arr['observation_time_ns'], dtype=np.int64)
    rel_ns = obs_ns - np.int64(reader.origin)
    phases = np.asarray(arr['phase'], dtype=np.int64)
    segments = np.asarray(arr['clock_segment'], dtype=np.int64)
    ambiguous = np.asarray(arr['time_ambiguous'], dtype=bool)
    q = np.asarray(arr['q'], dtype=float)
    dq = np.asarray(arr['dq'], dtype=float)
    effort_raw = np.asarray(arr['effort_raw'], dtype=float)
    effort_nm = (effort_raw - np.asarray(robot_config['effort_zero_nm'], dtype=float)) * np.asarray(robot_config['effort_gain'], dtype=float)

    formal_mask = (phases == 3) & (rel_ns >= calib_begin_ns)
    if legacy_h1 and not plan_meta.get('preparation') and not diagnostic:
        formal_mask = np.ones(n_rows, dtype=bool)
    formal_q = q[formal_mask]
    formal_recv_ns = receive_ns[formal_mask]
    if len(formal_q) < 2:
        raise ValueError('正式标定区间没有足够反馈帧')

    geometry = collision_for(robot_config) if collision_profile is None else collision_profile
    received_collision = certify_polyline(
        geometry,
        formal_q,
        (formal_recv_ns - formal_recv_ns[0]) * 1e-9,
        clearance_m=robot_config['scene']['clearance_m'],
        block_samples=1024,
    )
    received_collision.pop('wall_s', None)
    duration_s = float((receive_ns[-1] - receive_ns[0]) * 1e-9) if receive_ns[-1] > receive_ns[0] else 0.0
    obs = dict(
        origin_ns=int(reader.origin),
        receive_ns=receive_ns,
        observation_time_ns=obs_ns,
        relative_ns=rel_ns,
        elapsed_s=rel_ns * 1e-9,
        phase=phases,
        clock_segment=segments,
        time_ambiguous=ambiguous,
        q=q,
        dq=dq,
        effort_raw=effort_raw,
        effort_nm=effort_nm,
        feedback_rows=n_rows,
        formal_rows=int(formal_mask.sum()),
        mean_feedback_hz=(n_rows - 1) / duration_s if duration_s > 0 else None,
    )
    return obs, native, received_collision


def received_clearance_line(collision):
    distance = collision['minimum_sampled_distance_m'] * 1000
    result = '通过' if collision['passed'] else '低于准入裕度或未完成认证'
    return f'- 实测反馈折线碰撞复核：{result}，最小采样间距 {distance:.2f} mm'


def export_stage_candidate(out_dir, candidate_engine, robot_config):
    out_dir = Path(out_dir)
    source_urdf = urdf_for(robot_config)
    candidate_urdf = out_dir / 'candidate.urdf'
    urdf_info = write_inertial_urdf(source_urdf, candidate_engine, candidate_urdf)
    reload_report = check_reload(candidate_urdf, candidate_engine, source_urdf)

    runtime_sidecar = dict(
        schema_version='casbot_shm_runtime_dynamics_v1',
        joint_names=list(NAMES),
        gravity_world=list(robot_config['gravity_world']),
        rotor_inertia=[float(j.rotor_inertia) for j in candidate_engine.model.joints],
        friction={
            key: [float(getattr(j.friction, key)) for j in candidate_engine.model.joints]
            for key in ('viscous', 'coulomb', 'static_friction', 'smoothing_velocity', 'stribeck_velocity')
        },
    )
    write_json(out_dir / 'runtime_dynamics.json', runtime_sidecar)
    write_json(out_dir / 'urdf_reload.json', reload_report)

    with (out_dir / 'bodies.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['joint', 'link', 'mass_kg', 'com_x_m', 'com_y_m', 'com_z_m', 'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'])
        for row in urdf_info['bodies']:
            inertia = row['inertia_com_link_kg_m2']
            writer.writerow([
                row['joint'],
                row['inertial_link'],
                row['mass_kg'],
                *row['com_link_m'],
                inertia[0][0], inertia[0][1], inertia[0][2],
                inertia[1][1], inertia[1][2], inertia[2][2],
            ])
    return dict(
        candidate_urdf='candidate.urdf',
        runtime_dynamics='runtime_dynamics.json',
        reload_passed=reload_report['passed'],
        bodies=urdf_info['bodies'],
    )


def evaluate_stage_acceptance(baseline_rmse, candidate_rmse, *, axis_rows=()):
    """Report holdout regression and reject non-finite fit metrics."""
    baseline = float(baseline_rmse)
    candidate = float(candidate_rmse)
    degraded_axes = []
    for joint_name, b_ax, c_ax in axis_rows:
        if b_ax is None or c_ax is None:
            continue
        b_val = float(b_ax)
        c_val = float(c_ax)
        if (c_val - b_val) > max(0.05, 0.10 * b_val):
            degraded_axes.append(str(joint_name))

    accepted = bool(np.isfinite([baseline, candidate]).all())

    return dict(
        accepted=accepted,
        baseline_rmse_nm=baseline,
        candidate_rmse_nm=candidate,
        improvement_nm=float(baseline - candidate),
        overall_degraded=bool(candidate > baseline * 1.005 + 1e-4),
        degraded_axes=degraded_axes,
        reason='finite_validation_rmse' if accepted else 'nonfinite_validation_rmse',
    )
