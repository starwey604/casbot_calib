"""H1 Static Gravity stage: optional ready pose + smoke jog + paired-approach static gravity & PSD fit."""
from copy import deepcopy
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.interpolate import BPoly
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
import yaml
from ...collision import certify_polyline
from ...data import write_json
from ...field_gravity import write_gravity_candidate, write_static_diagnostics
from ...field_motion import jog_motion, static_motion
from ...model_runtime import collision_for, model_for
from ...pipeline.robot import load_robot_config
from ...pipeline.stage import Stage
from ...rigid_identification import parameter_scales
from ...run_marker import run_marker
from ...shm_data import DIRECTION, NAMES, PARTS, ROOT
from ...shm_fieldwork import text_result
from ...shm_observations import ObservationReader
from ...shm_ready import handover_fraction, pd_schedule, preparation, prepend, ready_position
from ...shm_reference import export_motion
from ...staged_motion import (
    Motion,
    _stationary_pose_cache,
    _stationary_segment_report,
    derivative_bounds,
    gate_motion,
)
from ...staged_motion import _move
from .common import ensure_leading_hold
from .h1_selection import _select_arm, candidate_pool, information, paired_motions, screen_candidates

DEFAULT_H1_CONFIG = ROOT / 'configs/dynamics/h1.yaml'


class TimeMean:
    """Trapezoid exposure per unique observation timestamp; duplicates share group weight."""
    def __init__(self):
        self.current = None
        self.previous = None
        self.total = None
        self.weight = 0.0
        self.rows = 0
        self.groups = 0
        self.first = None
        self.last = None
        self.maximum_gap_ns = 0
        self.error = None
        self.segment = None
        self.low = None
        self.high = None

    def _flush(self, next_ns=None):
        if self.current is None:
            return
        at, values, count = self.current
        before = at - self.previous if self.previous is not None else 0
        after = next_ns - at if next_ns is not None else 0
        w = (before + after) * 0.5
        if w > 0:
            if self.total is None:
                self.total = np.zeros_like(values)
            self.total += w * values / count
            self.weight += w
        self.previous = at
        self.groups += 1

    def add(self, row, values):
        ns = row['observation_time_ns']
        self.rows += 1
        if row['time_ambiguous']:
            self.error = 'wall-clock jump interval or source time reversal'
        if self.segment is not None and self.segment != row['clock_segment']:
            self.error = 'measurement crosses clock segments'
        self.segment = row['clock_segment']
        self.low = np.asarray(values).copy() if self.low is None else np.minimum(self.low, values)
        self.high = np.asarray(values).copy() if self.high is None else np.maximum(self.high, values)
        if self.first is None:
            self.first = ns
        self.last = ns
        if self.current is not None and ns < self.current[0]:
            self.error = 'observation time reversal'
            return
        if self.current is not None and ns == self.current[0]:
            self.current[1] += values
            self.current[2] += 1
            return
        if self.current is not None:
            self.maximum_gap_ns = max(self.maximum_gap_ns, ns - self.current[0])
            self._flush(ns)
        self.current = [ns, np.asarray(values).copy(), 1]

    def result(self):
        self._flush()
        self.current = None
        return None if self.error or not self.weight else self.total / self.weight


class ApproachDisplacement:
    """Observed q displacement over paired_approach using 50ms endpoint exposure means."""
    def __init__(self, trial, metadata):
        k = metadata['labels'].index('paired_approach')
        origin = round(trial['begin_s'] * 1e9)
        self.begin = origin + round(metadata['knots_s'][k] * 1e9)
        self.end = origin + round(metadata['knots_s'][k + 1] * 1e9)
        self.width = min(50_000_000, (self.end - self.begin) // 2)
        self.expected = np.asarray(metadata['states'][k + 1][0]) - metadata['states'][k][0]
        self.first = TimeMean()
        self.last = TimeMean()
        self.segments = set()
        self.ambiguous = False

    def add(self, row, relative_ns):
        if not self.begin <= relative_ns < self.end:
            return
        self.segments.add(row['clock_segment'])
        self.ambiguous |= row['time_ambiguous']
        if relative_ns < self.begin + self.width:
            self.first.add(row, row['q'])
        if relative_ns >= self.end - self.width:
            self.last.add(row, row['q'])

    def result(self):
        first = self.first.result()
        last = self.last.result()
        available = first is not None and last is not None and not self.ambiguous and len(self.segments) == 1
        return dict(
            available=available,
            reference_displacement_rad=self.expected,
            observed_displacement_rad=last - first if available else None,
            endpoint_window_ns=self.width,
            first_rows=self.first.rows,
            last_rows=self.last.rows,
            unavailable_reason=None if available else self.first.error or self.last.error or 'clock ambiguity or missing endpoint exposure',
        )


def _trapezoid_time_mean(obs_ns, seg, ambig, values):
    obs_ns = np.asarray(obs_ns, dtype=np.int64)
    seg = np.asarray(seg, dtype=np.int64)
    ambig = np.asarray(ambig, dtype=bool)
    values = np.asarray(values, dtype=np.float64)
    m = len(obs_ns)
    if m == 0:
        return dict(
            mean=None,
            rows=0,
            groups=0,
            first=None,
            last=None,
            maximum_gap_ns=0,
            error=None,
            low=None,
            high=None,
        )

    squeeze_1d = values.ndim == 1
    if squeeze_1d:
        values = values[:, None]

    low = values.min(axis=0).copy()
    high = values.max(axis=0).copy()
    if squeeze_1d:
        low = float(low[0])
        high = float(high[0])
    first = int(obs_ns[0])
    last = int(obs_ns[-1])

    seg_change = np.zeros(m, dtype=bool)
    if m > 1:
        seg_change[1:] = seg[1:] != seg[:-1]

    running_max = np.maximum.accumulate(obs_ns)
    reversal = np.zeros(m, dtype=bool)
    if m > 1:
        reversal[1:] = obs_ns[1:] < running_max[:-1]

    if np.any(ambig) or np.any(seg_change) or np.any(reversal):
        err_code = np.zeros(m, dtype=np.int8)
        err_code[ambig] = 1
        err_code[seg_change] = 2
        err_code[reversal] = 3
        last_err = int(err_code[np.flatnonzero(err_code)[-1]])
        error = (
            'wall-clock jump interval or source time reversal' if last_err == 1
            else 'measurement crosses clock segments' if last_err == 2
            else 'observation time reversal'
        )
        valid_ns = obs_ns[~reversal]
        is_new = np.r_[True, valid_ns[1:] != valid_ns[:-1]]
        unique_ns = valid_ns[is_new]
        groups = int(len(unique_ns))
        max_gap = int(np.diff(unique_ns).max()) if groups > 1 else 0
        return dict(
            mean=None,
            rows=m,
            groups=groups,
            first=first,
            last=last,
            maximum_gap_ns=max_gap,
            error=error,
            low=low,
            high=high,
        )

    is_new = np.r_[True, obs_ns[1:] != obs_ns[:-1]]
    group_starts = np.flatnonzero(is_new)
    groups = int(len(group_starts))
    if groups < 2:
        return dict(
            mean=None,
            rows=m,
            groups=groups,
            first=first,
            last=last,
            maximum_gap_ns=0,
            error=None,
            low=low,
            high=high,
        )

    unique_ns = obs_ns[group_starts]
    gaps = np.diff(unique_ns)
    max_gap = int(gaps.max())
    counts = np.diff(np.r_[group_starts, m])
    group_sums = np.add.reduceat(values, group_starts, axis=0)
    group_means = group_sums / counts[:, None]
    w = (np.r_[0, gaps] + np.r_[gaps, 0]) * 0.5
    weight = float(w.sum())
    mean = (w[:, None] * group_means).sum(axis=0) / weight if weight > 0.0 else None
    if mean is not None and squeeze_1d:
        mean = float(mean[0])
    return dict(
        mean=mean,
        rows=m,
        groups=groups,
        first=first,
        last=last,
        maximum_gap_ns=max_gap,
        error=None,
        low=low,
        high=high,
    )


def load_h1_recipe(source=DEFAULT_H1_CONFIG, **overrides):
    if isinstance(source, dict):
        c = deepcopy(source)
    else:
        path = Path(source or DEFAULT_H1_CONFIG).resolve()
        c = yaml.safe_load(path.read_text(encoding='utf-8')) or {}

    if 'mode' in overrides and overrides['mode'] is not None:
        c['mode'] = overrides['mode']
    c.setdefault('mode', 'small')
    c.setdefault('ready_pose', True)
    c.setdefault('gravity_feedforward', 'nominal_reference')
    smoke = dict(c.get('smoke_jog') or {})
    smoke.setdefault('enabled', False)
    smoke.setdefault('joint_id', 12)
    smoke.setdefault('amplitude_deg', 10.0)
    smoke.setdefault('anchor_rad', None)
    if overrides.get('smoke_enabled'):
        if c['mode'] != 'smoke':
            smoke['enabled'] = True
    if 'joint_id' in overrides and overrides['joint_id'] is not None:
        smoke['joint_id'] = overrides['joint_id']
    if 'amplitude_deg' in overrides and overrides['amplitude_deg'] is not None:
        smoke['amplitude_deg'] = overrides['amplitude_deg']
    c['smoke_jog'] = smoke

    if c['mode'] not in ('small', 'full', 'smoke'):
        raise ValueError(f'未知 H1 模式：{c["mode"]}（可选：small, full, smoke）')
    if c['gravity_feedforward'] not in ('nominal_reference', 'none'):
        raise ValueError('gravity_feedforward 须为 nominal_reference 或 none')
    if c['mode'] == 'smoke' or smoke['enabled']:
        jid = int(smoke['joint_id'])
        amp = float(smoke['amplitude_deg'])
        if jid not in range(12, 26) or not np.isfinite(amp) or not 0 < amp <= 10.0:
            raise ValueError('H1 往返冒烟须使用 ID12–25、幅度 (0, 10] 度')
        smoke['joint_id'] = jid
        smoke['amplitude_deg'] = amp

    if c['mode'] == 'small':
        small = dict(c.get('small') or {})
        small.setdefault('parallel_arms', True)
        small.setdefault('direct_pose_chain', True)
        small['parallel_arms'] = bool(small['parallel_arms'])
        small['direct_pose_chain'] = bool(small['direct_pose_chain'])
        small.setdefault('joints', ['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint'])
        small.setdefault('target_offset_deg', 15.0)
        small.setdefault('approach_offset_deg', 5.0)
        small.setdefault('timing', dict(
            initial_hold_s=0.5, minimum_transition_s=4.0, minimum_paired_approach_s=2.0,
            settle_s=2.0, measure_s=3.0, final_hold_s=0.5,
        ))
        small.setdefault('planning_envelope', dict(speed_rad_s=0.05, acceleration_rad_s2=0.1, jerk_rad_s3=1.0))
        joints = list(small['joints'])
        if not joints or len(set(joints)) != len(joints) or not set(joints) <= set(NAMES):
            raise ValueError('small.joints 须为不重复的双臂关节名')
        nums = [small['target_offset_deg'], small['approach_offset_deg'], *small['timing'].values(), *small['planning_envelope'].values()]
        if not np.isfinite(nums).all() or min(nums) <= 0:
            raise ValueError('small 静态参数须为正有限值')
        if small['target_offset_deg'] + small['approach_offset_deg'] > 20.0 + 1e-12 or small['planning_envelope']['speed_rad_s'] > 0.05 + 1e-12:
            raise ValueError('small 静态范围（含接近点）不超过 ±20°，参考速度不超过 0.05 rad/s')
        c['small'] = small

    elif c['mode'] == 'full':
        full = dict(c.get('full') or {})
        if not full:
            raise ValueError('full 模式缺少静态采样配置')
        full.setdefault('parallel_arms', True)
        full.setdefault('direct_pose_chain', True)
        full['parallel_arms'] = bool(full['parallel_arms'])
        full['direct_pose_chain'] = bool(full['direct_pose_chain'])
        nums = [full['approach_offset_deg'], *full['timing'].values(), *full['planning_envelope'].values()]
        if not np.isfinite(nums).all() or min(nums) <= 0:
            raise ValueError('full 静态参数须为正有限值')
        if int(full['train_poses_per_arm']) < 14 or int(full['validation_poses_per_arm']) < 1:
            raise ValueError('full 模式每臂至少需要 14 个训练姿态和 1 个验证姿态')
        c['full'] = full

    return c


def reference_effort(
    motion,
    engine,
    robot_config,
    preparation_cert=None,
    *,
    feedforward_mode='nominal_reference',
    step_ns=5_000_000,
):
    if feedforward_mode == 'none':
        return [dict(t_ns=0, value=[0.0] * 14)]
    step_ns = int(step_ns)
    if step_ns <= 0:
        raise ValueError('step_ns 须为正整数纳秒')
    curve = motion.polynomial()
    last = round(motion.duration * 1e9)
    grid = np.arange(0, last + 1, step_ns, dtype=np.int64)
    knot_ns = np.asarray([round(t * 1e9) for t in motion.knots], dtype=np.int64)
    extra = [0, last]
    if preparation_cert is not None:
        extra.extend([
            int(preparation_cert['move_end_ns']),
            int(preparation_cert['handover_end_ns']),
            int(preparation_cert['calibration_begin_ns']),
        ])
    mandatory_ns = np.unique(np.r_[knot_ns, np.asarray(extra, dtype=np.int64)])
    mandatory_ns = mandatory_ns[(mandatory_ns >= 0) & (mandatory_ns <= last)]
    times = np.unique(np.r_[grid, mandatory_ns])
    times = times[(times >= 0) & (times <= last)]
    q = np.ascontiguousarray(curve(times * 1e-9), dtype=np.float64).reshape(len(times), 14)
    torque = engine.gravity_batch(q)
    if preparation_cert is not None:
        torque = torque * handover_fraction(times, preparation_cert)[:, None]
    if not np.isfinite(torque).all() or np.max(np.abs(torque)) > robot_config['feedforward_limit_nm']:
        raise ValueError('名义重力前馈超出显式力矩上限；不夹取')
    direction = np.asarray(DIRECTION, dtype=np.float64)
    codes = np.trunc(torque * direction[None, :] * 100.0).astype(np.int64)
    keep = np.r_[True, np.any(codes[1:] != codes[:-1], axis=1)]
    idx = np.flatnonzero(keep)
    if len(idx) > 15_000:
        mand_flags = np.isin(times[idx], mandatory_ns)
        cand_codes = codes[idx]
        sel = [0]
        last_c = cand_codes[0]
        for pos in range(1, len(idx)):
            c_row = cand_codes[pos]
            if mand_flags[pos]:
                if np.any(c_row != last_c):
                    sel.append(pos)
                    last_c = c_row
            elif int(np.max(np.abs(c_row - last_c))) >= 2:
                sel.append(pos)
                last_c = c_row
        idx = idx[np.asarray(sel, dtype=np.int64)]

    t_sel = torque[idx]
    c_sel = codes[idx]
    v_compact = np.round(t_sel, 2)
    mismatch = np.trunc(v_compact * direction[None, :] * 100.0).astype(np.int64) != c_sel
    if np.any(mismatch):
        v4 = np.round(t_sel, 4)
        v_compact = np.where(mismatch, v4, v_compact)
        still_bad = np.trunc(v_compact * direction[None, :] * 100.0).astype(np.int64) != c_sel
        if np.any(still_bad):
            v_compact = np.where(still_bad, t_sel, v_compact)
    v_compact = np.where(c_sel == 0, 0.0, v_compact)
    return [
        dict(t_ns=int(ns), value=val)
        for ns, val in zip(times[idx].tolist(), v_compact.tolist())
    ]


def _septic_unit_step(u):
    om = 1.0 - u
    om2 = om * om
    u2 = u * u
    return (u2 * u2) * (35.0 * om2 * om + 21.0 * u * om2 + 7.0 * u2 * om + u2 * u)


def _gate_rest_to_rest_motion(geometry, motion, margin=0.01):
    n_seg = len(motion.labels)
    if (
        n_seg < 4
        or motion.stage == 'inertia_probe'
        or 'position_allowance_rad' in motion.conditions
        or motion.conditions.get('refine_position_bounds', False)
        or any(np.any(st[1:]) for st in motion.states)
    ):
        return gate_motion(geometry, motion, margin)

    curve = motion.polynomial()
    lower = np.min(curve.c, axis=(0, 1))
    upper = np.max(curve.c, axis=(0, 1))
    joints = geometry.engine.model.joints
    limits = np.array([[j.lower, j.upper] for j in joints], dtype=float)
    limits_passed = bool(np.all(lower >= limits[:, 0]) and np.all(upper <= limits[:, 1]))
    limit_report = dict(
        passed=limits_passed,
        lower_rad=lower.tolist(),
        upper_rad=upper.tolist(),
        source='nominal URDF travel limits only; manufacturer speed/effort capabilities unconfirmed',
        method='Bernstein coefficient convex hull',
    )
    if not limits_passed:
        return gate_motion(geometry, motion, margin)

    stat_indices = []
    mov_indices = []
    for i in range(n_seg):
        if np.array_equal(motion.states[i][0], motion.states[i + 1][0]):
            stat_indices.append(i)
        else:
            mov_indices.append(i)
    if not stat_indices or not mov_indices:
        return gate_motion(geometry, motion, margin)

    thresholds = np.asarray(
        getattr(geometry, 'pair_clearance_m', np.full(len(geometry.pairs), margin)),
        dtype=float,
    )
    cache = _stationary_pose_cache(geometry)
    native_id = id(geometry.native)
    uncached_keys = []
    uncached_poses = []
    seen_uncached = set()
    stat_keys = {}
    for i in stat_indices:
        q_i = np.ascontiguousarray(motion.states[i][0], dtype=np.float64)
        key = (native_id, q_i.tobytes())
        stat_keys[i] = key
        if key not in cache and key not in seen_uncached:
            seen_uncached.add(key)
            uncached_keys.append(key)
            uncached_poses.append(q_i)

    stat_batch_dt = 0.0
    if uncached_poses:
        t_stat = perf_counter()
        obs_stat = geometry.check(np.array(uncached_poses))
        stat_batch_dt = (perf_counter() - t_stat) / len(uncached_poses)
        for k, key in enumerate(uncached_keys):
            cache[key] = dict(
                distances=obs_stat['distances'][k].copy(),
                worst_pair=int(obs_stat['worst_pair'][k]),
                first_point=obs_stat['first_point'][k].copy(),
                second_point=obs_stat['second_point'][k].copy(),
            )

    fresh_keys = set(uncached_keys)
    stat_reports = {}
    for i in stat_indices:
        t0 = perf_counter()
        key = stat_keys[i]
        was_fresh = key in fresh_keys
        fresh_keys.discard(key)
        rep = _stationary_segment_report(
            geometry,
            motion.states[i][0],
            motion.knots[i],
            motion.knots[i + 1],
            motion.labels[i],
            i,
            cache[key],
            thresholds,
            margin,
            1 if was_fresh else 0,
            (stat_batch_dt if was_fresh else 0.0) + (perf_counter() - t0),
        )
        if not rep['passed']:
            return gate_motion(geometry, motion, margin)
        stat_reports[i] = rep

    n_mov = len(mov_indices)
    q0_mat = np.empty((n_mov, geometry.joints), dtype=np.float64)
    dq_mat = np.empty((n_mov, geometry.joints), dtype=np.float64)
    abs_dq_mat = np.empty((n_mov, geometry.joints), dtype=np.float64)
    states = []
    pending = []
    for pos, orig_i in enumerate(mov_indices):
        a = float(motion.knots[orig_i])
        b = float(motion.knots[orig_i + 1])
        dur = b - a
        if dur <= 0.0:
            return gate_motion(geometry, motion, margin)
        q0 = np.asarray(motion.states[orig_i][0], dtype=np.float64)
        q1 = np.asarray(motion.states[orig_i + 1][0], dtype=np.float64)
        dq = q1 - q0
        abs_dq = np.abs(dq)
        q0_mat[pos] = q0
        dq_mat[pos] = dq
        abs_dq_mat[pos] = abs_dq
        vel_bound = (2.1875 * abs_dq / dur) * (1.0 + 1e-10) + 1e-12
        states.append(dict(
            seg_idx=orig_i,
            a=a,
            b=b,
            duration_s=dur,
            velocity=vel_bound,
            minimum=np.inf,
            lower_min=np.inf,
            witness=None,
            queries=0,
            certified=0,
            status='pending',
            reason=None,
            wall_s=0.0,
        ))
        pending.append((pos, 0.0, 1.0, 0))

    earliest_failed = n_mov
    chunk_size = 512
    max_depth = 16
    radii_t = geometry.radii.T
    zeros_allow = [0.0] * geometry.joints
    path_model = 'analytic reference conditional on supplied global derivative bounds and position allowance'

    while pending:
        pending = [item for item in pending if item[0] < earliest_failed and states[item[0]]['status'] == 'pending']
        if not pending:
            break
        next_round = []
        for offset in range(0, len(pending), chunk_size):
            t0 = perf_counter()
            chunk = [
                item for item in pending[offset:offset + chunk_size]
                if item[0] < earliest_failed and states[item[0]]['status'] == 'pending'
            ]
            if not chunk:
                continue
            pos_arr = np.array([c[0] for c in chunk], dtype=np.int64)
            u_lo = np.array([c[1] for c in chunk], dtype=np.float64)
            u_hi = np.array([c[2] for c in chunk], dtype=np.float64)
            u_mid = 0.5 * (u_lo + u_hi)
            s_lo = _septic_unit_step(u_lo)
            s_mid = _septic_unit_step(u_mid)
            s_hi = _septic_unit_step(u_hi)
            ds_max = np.maximum(s_hi - s_mid, s_mid - s_lo) * (1.0 + 1e-12) + 1e-15

            q = np.ascontiguousarray(q0_mat[pos_arr] + s_mid[:, None] * dq_mat[pos_arr], dtype=np.float64)
            var_bounds = ds_max[:, None] * abs_dq_mat[pos_arr]
            observed = geometry.check(q)
            distances = observed['distances']
            lower_b = distances - var_bounds @ radii_t
            dt_each = (perf_counter() - t0) / len(chunk)

            for k, (pos, ulo_k, uhi_k, depth) in enumerate(chunk):
                st = states[pos]
                if pos >= earliest_failed or st['status'] != 'pending':
                    continue
                st['queries'] += 1
                st['wall_s'] += dt_each
                nearest = int(np.argmin(distances[k] - thresholds))
                distance = float(distances[k, nearest])
                bound = float((lower_b[k] - thresholds + margin).min())
                if distance < st['minimum']:
                    st['minimum'] = distance
                    pair = geometry.pairs[nearest]
                    m_loc = float(u_mid[k] * st['duration_s'])
                    st['witness'] = dict(
                        parameter=m_loc,
                        time=m_loc,
                        q=q[k].tolist(),
                        first=geometry.names[pair[0]],
                        second=geometry.names[pair[1]],
                        distance_m=distance,
                        first_point=(
                            observed['first_point'][k].tolist()
                            if nearest == observed['worst_pair'][k]
                            else None
                        ),
                        second_point=(
                            observed['second_point'][k].tolist()
                            if nearest == observed['worst_pair'][k]
                            else None
                        ),
                    )
                if np.any(distances[k] <= thresholds):
                    st['status'] = 'failed'
                    st['reason'] = 'collision_or_clearance'
                    earliest_failed = min(earliest_failed, pos)
                elif np.all(lower_b[k] > thresholds):
                    st['certified'] += 1
                    st['lower_min'] = min(st['lower_min'], float(lower_b[k].min()))
                elif depth >= max_depth:
                    st['status'] = 'failed'
                    st['reason'] = 'interval_not_certified'
                    earliest_failed = min(earliest_failed, pos)
                else:
                    umid_k = float(u_mid[k])
                    next_round.extend([(pos, ulo_k, umid_k, depth + 1), (pos, umid_k, uhi_k, depth + 1)])

        active_next = {item[0] for item in next_round if item[0] < earliest_failed}
        for pos in range(min(n_mov, earliest_failed)):
            if states[pos]['status'] == 'pending' and pos not in active_next:
                states[pos]['status'] = 'passed'
                states[pos]['reason'] = 'bounded_intervals_certified'
        pending = next_round

    mov_reports = {}
    stop_pos = min(n_mov, earliest_failed + 1)
    for st in states[:stop_pos]:
        orig_i = st['seg_idx']
        if st['status'] == 'passed':
            mov_reports[orig_i] = dict(
                passed=True,
                reason='bounded_intervals_certified',
                minimum_sampled_distance_m=st['minimum'],
                certified_lower_bound_m=st['lower_min'],
                clearance_m=margin,
                witness=st['witness'],
                queries=st['queries'],
                certified_intervals=st['certified'],
                wall_s=st['wall_s'],
                path_model=path_model,
                velocity_bound_rad_s=st['velocity'].tolist(),
                position_allowance_rad=zeros_allow.copy(),
                segment=orig_i,
                label=motion.labels[orig_i],
                start_s=st['a'],
                end_s=st['b'],
            )
        else:
            mov_reports[orig_i] = dict(
                passed=False,
                reason=st['reason'],
                minimum_sampled_distance_m=st['minimum'],
                witness=st['witness'],
                queries=st['queries'],
                wall_s=st['wall_s'],
                path_model=path_model,
                velocity_bound_rad_s=st['velocity'].tolist(),
                position_allowance_rad=zeros_allow.copy(),
                segment=orig_i,
                label=motion.labels[orig_i],
                start_s=st['a'],
                end_s=st['b'],
            )

    if earliest_failed < n_mov:
        orig_fail = mov_indices[earliest_failed]
        rows = [stat_reports[i] if i in stat_reports else mov_reports[i] for i in range(orig_fail + 1)]
        return dict(
            passed=False,
            segments=rows,
            minimum_distance_m=min(r['minimum_sampled_distance_m'] for r in rows),
            queries=sum(r['queries'] for r in rows),
            wall_s=sum(r['wall_s'] for r in rows),
            motion=motion.name,
            joint_limits=limit_report,
            includes=['initial hold', 'approach', 'active motion', 'braking', 'return home', 'final hold'],
        )

    rows = [stat_reports[i] if i in stat_reports else mov_reports[i] for i in range(n_seg)]
    return dict(
        passed=True,
        segments=rows,
        minimum_distance_m=min(r['minimum_sampled_distance_m'] for r in rows),
        queries=sum(r['queries'] for r in rows),
        wall_s=sum(r['wall_s'] for r in rows),
        motion=motion.name,
        joint_limits=limit_report,
        includes=['initial hold', 'approach', 'active motion', 'braking', 'return home', 'final hold'],
    )


def _gate_with_certified_chain(geometry, motion, curve, margin, certified_chain):
    if certified_chain is None or float(certified_chain['clearance_m']) != float(margin):
        return _gate_rest_to_rest_motion(geometry, motion, margin)
    m_len = len(certified_chain['labels'])
    if (
        len(motion.labels) != m_len + 4
        or motion.labels[2:2 + m_len] != certified_chain['labels']
        or not np.allclose(np.diff(motion.knots[2:3 + m_len]), np.diff(certified_chain['knots']), atol=1e-12)
        or not all(np.allclose(motion.states[2 + k], certified_chain['states'][k], atol=1e-12) for k in range(m_len + 1))
    ):
        return _gate_rest_to_rest_motion(geometry, motion, margin)

    lower = np.min(curve.c, axis=(0, 1))
    upper = np.max(curve.c, axis=(0, 1))
    joints = geometry.engine.model.joints
    limits = np.array([[j.lower, j.upper] for j in joints], dtype=float)
    limits_passed = bool(np.all(lower >= limits[:, 0]) and np.all(upper <= limits[:, 1]))
    if not limits_passed:
        return gate_motion(geometry, motion, margin)
    limit_report = dict(
        passed=True,
        lower_rad=lower.tolist(),
        upper_rad=upper.tolist(),
        source='nominal URDF travel limits only; manufacturer speed/effort capabilities unconfirmed',
        method='Bernstein coefficient convex hull',
    )

    pre_m = Motion('prefix_gate', motion.stage, motion.states[0][0])
    for i in (0, 1):
        pre_m.add(motion.knots[i + 1] - motion.knots[i], motion.states[i + 1][0], label=motion.labels[i])
    pre_m.knots = [motion.knots[0], motion.knots[1], motion.knots[2]]
    pre_gate = gate_motion(geometry, pre_m, margin)
    if not pre_gate['passed']:
        return _gate_rest_to_rest_motion(geometry, motion, margin)

    suf_base = 2 + m_len
    suf_t0 = motion.knots[suf_base]
    suf_m = Motion('suffix_gate', motion.stage, motion.states[suf_base][0])
    for i in (suf_base, suf_base + 1):
        suf_m.add(motion.knots[i + 1] - motion.knots[i], motion.states[i + 1][0], label=motion.labels[i])
    suf_m.knots = [0.0, motion.knots[suf_base + 1] - suf_t0, motion.knots[suf_base + 2] - suf_t0]
    suf_gate = gate_motion(geometry, suf_m, margin)
    if not suf_gate['passed']:
        return _gate_rest_to_rest_motion(geometry, motion, margin)

    rows = []
    for i, r in enumerate(pre_gate['segments']):
        rc = dict(r)
        rc['segment'] = i
        rc['start_s'] = motion.knots[i]
        rc['end_s'] = motion.knots[i + 1]
        rows.append(rc)
    for k, r in enumerate(certified_chain['gate']['segments']):
        seg_i = 2 + k
        rc = dict(r)
        rc['segment'] = seg_i
        rc['start_s'] = motion.knots[seg_i]
        rc['end_s'] = motion.knots[seg_i + 1]
        rows.append(rc)
    for k, r in enumerate(suf_gate['segments']):
        seg_i = suf_base + k
        rc = dict(r)
        rc['segment'] = seg_i
        rc['start_s'] = motion.knots[seg_i]
        rc['end_s'] = motion.knots[seg_i + 1]
        rows.append(rc)

    return dict(
        passed=True,
        segments=rows,
        minimum_distance_m=min(r['minimum_sampled_distance_m'] for r in rows),
        queries=sum(r['queries'] for r in rows),
        wall_s=sum(r['wall_s'] for r in rows),
        motion=motion.name,
        joint_limits=limit_report,
        includes=['initial hold', 'approach', 'active motion', 'braking', 'return home', 'final hold'],
    )


class _SinglePoseGateFailure(Exception):
    def __init__(self, pose_id, segment_report):
        super().__init__(f'姿态 {pose_id} 未通过连续碰撞检查')
        self.pose_id = pose_id
        self.segment_report = segment_report


def _select_arm_poses(rows, config, engine):
    settings = config['selection']
    arm = rows[0]['arm']
    active = [k for k, j in enumerate(engine.model.joints) if j.name.startswith(arm + '_')]
    pool = [r for r in rows if r['status'] == 'endpoint_admissible']
    if not pool:
        return [], dict(passed=False, reason='no_endpoint_admissible_candidates')
    q = np.array([r['q_rad'] for r in pool], dtype=float)
    zero = np.zeros_like(q)
    regressors = engine.regressor_batch(q, zero, zero)['regressor'][:, active, :]
    scales = parameter_scales(engine.rigid_parameters())
    pool_info, basis = information(regressors, scales, settings['rank_rtol'])
    blocks = (regressors * scales) @ basis
    grams = np.swapaxes(blocks, 1, 2) @ blocks
    current = np.eye(pool_info['rank']) * settings['information_ridge']
    training = []
    validation = []
    missing = []

    def accept(index, split):
        row = pool[index]
        row.update(status='selected', split=split)
        (training if split == 'train' else validation).append(index)
        return True

    for joint in active:
        for sign in (-1, 1):
            options = [
                i for i, r in enumerate(pool)
                if r['family'] == 'single_axis'
                and r['active_joints'] == [joint]
                and np.sign(r['offsets_deg'][0]) == sign
                and r['status'] == 'endpoint_admissible'
            ]
            options.sort(key=lambda i: (-abs(pool[i]['offsets_deg'][0]), pool[i]['id']))
            if not any(accept(i, 'train') for i in options):
                missing.append(dict(joint=engine.model.joints[joint].name, direction=sign))

    for index in training:
        current += grams[index]

    rng = np.random.default_rng(settings['validation_seed'] + (0 if arm == 'left' else 1))
    validation_order = rng.permutation(len(pool))
    validation_information = np.eye(pool_info['rank']) * settings['information_ridge']
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
    passed = bool(
        not missing
        and len(training) == settings['train_poses_per_arm']
        and len(validation) == settings['validation_poses_per_arm']
        and train_info['rank'] == validation_info['rank'] == pool_info['rank']
    )
    return [pool[i] for i in [*training, *validation]], dict(
        passed=passed,
        endpoint_pool_information=pool_info,
        training_information=train_info,
        validation_information=validation_info,
        missing_signed_diagnostics=missing,
        training_poses=len(training),
        validation_poses=len(validation),
    )


def _pair_arm_poses(poses, approach_offset_rad, geometry, margin, *, parallel_arms=True, home_rad=None):
    poses_copy = [deepcopy(r) for r in poses]
    joints = geometry.engine.model.joints
    limits = np.array([[j.lower, j.upper] for j in joints], dtype=float)

    need_dist = []
    pts_batch = []
    for r in poses_copy:
        if r.get('minimum_endpoint_distance_m') is None:
            q = np.asarray(r['q_rad'], dtype=float)
            act = list(r.get('approach_joints', r['active_joints']))
            nm = q.copy(); nm[act] -= approach_offset_rad
            np_ = q.copy(); np_[act] += approach_offset_rad
            pts3 = np.array([q, nm, np_], dtype=float)
            if np.any(pts3 < limits[:, 0]) or np.any(pts3 > limits[:, 1]):
                raise ValueError(f'静态姿态 {r["id"]} 超出 URDF 行程范围')
            need_dist.append(r)
            pts_batch.extend(pts3)
    if pts_batch:
        d_all = geometry.check(np.array(pts_batch, dtype=float))['distances'].reshape(len(need_dist), 3, -1)
        for r, d3 in zip(need_dist, d_all):
            d_min = float(d3.min())
            if d_min <= margin:
                raise ValueError(f'静态姿态 {r["id"]} 端点碰撞间距不足（{d_min:.4f} m <= {margin:.4f} m）')
            r['minimum_endpoint_distance_m'] = d_min

    if not parallel_arms:
        return poses_copy

    def _pair_endpoints(left_row, right_row):
        q_pair = np.r_[np.asarray(left_row['q_rad'], dtype=float)[:7], np.asarray(right_row['q_rad'], dtype=float)[7:]]
        act = sorted(set(left_row['active_joints']) | set(right_row['active_joints']))
        approach = sorted(set(left_row.get('approach_joints', left_row['active_joints'])) |
                          set(right_row.get('approach_joints', right_row['active_joints'])))
        nm = q_pair.copy(); nm[approach] -= approach_offset_rad
        np_ = q_pair.copy(); np_[approach] += approach_offset_rad
        return q_pair, act, [q_pair, nm, np_]

    def _admissible(pair_min, left_row, right_row):
        l_min = float(left_row['minimum_endpoint_distance_m'])
        r_min = float(right_row['minimum_endpoint_distance_m'])
        return pair_min > margin and pair_min >= min(0.012, l_min - 1e-4, r_min - 1e-4)

    def _make_paired_dict(left_row, right_row, split, q_pair, act, pair_min):
        return dict(
            id=f"{left_row['id']}__{right_row['id']}",
            arm='both',
            split=split,
            q_rad=q_pair.tolist(),
            family='paired_dual_arm',
            active_joints=act,
            approach_joints=sorted(set(left_row.get('approach_joints', left_row['active_joints'])) |
                                   set(right_row.get('approach_joints', right_row['active_joints']))),
            left_pose=deepcopy(left_row),
            right_pose=deepcopy(right_row),
            minimum_endpoint_distance_m=pair_min,
            status='selected',
        )

    mirror = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0], dtype=float)
    splits_order = []
    for r in poses_copy:
        if r['split'] not in splits_order:
            splits_order.append(r['split'])

    result = []
    for split in splits_order:
        split_rows = [r for r in poses_copy if r['split'] == split]
        left_list = [r for r in split_rows if r['arm'] == 'left']
        right_list = [r for r in split_rows if r['arm'] == 'right']
        other_list = [r for r in split_rows if r['arm'] not in ('left', 'right')]
        if not left_list or not right_list:
            result.extend(split_rows)
            continue

        home = (
            np.asarray(home_rad, dtype=float)
            if home_rad is not None
            else np.r_[
                np.asarray(right_list[0]['q_rad'], dtype=float)[:7],
                np.asarray(left_list[0]['q_rad'], dtype=float)[7:],
            ]
        )
        l_rel = (np.array([r['q_rad'][:7] for r in left_list], dtype=float) - home[:7]) * mirror[None, :]
        r_rel = np.array([r['q_rad'][7:] for r in right_list], dtype=float) - home[7:]
        pair_cost = cdist(l_rel, r_rel, metric='chebyshev')
        row_ind, col_ind = linear_sum_assignment(pair_cost)

        init_specs = []
        init_pts = []
        for li, rj in zip(row_ind.tolist(), col_ind.tolist()):
            q_pair, act, pts3 = _pair_endpoints(left_list[li], right_list[rj])
            in_lim = not (np.any(np.asarray(pts3) < limits[:, 0]) or np.any(np.asarray(pts3) > limits[:, 1]))
            init_specs.append((li, rj, q_pair, act, in_lim))
            init_pts.extend(pts3)

        init_dists = geometry.check(np.array(init_pts, dtype=float))['distances'].reshape(len(init_specs), 3, -1)
        matched_l = set()
        matched_r = set()
        for (li, rj, q_pair, act, in_lim), d3 in zip(init_specs, init_dists):
            p_min = float(d3.min())
            if in_lim and _admissible(p_min, left_list[li], right_list[rj]):
                result.append(_make_paired_dict(left_list[li], right_list[rj], split, q_pair, act, p_min))
                matched_l.add(li)
                matched_r.add(rj)

        rem_l = [i for i in range(len(left_list)) if i not in matched_l]
        rem_r = [j for j in range(len(right_list)) if j not in matched_r]
        if rem_l and rem_r:
            cand_specs = []
            cand_pts = []
            for li in rem_l:
                for rj in rem_r:
                    q_pair, act, pts3 = _pair_endpoints(left_list[li], right_list[rj])
                    in_lim = not (np.any(np.asarray(pts3) < limits[:, 0]) or np.any(np.asarray(pts3) > limits[:, 1]))
                    cand_specs.append((li, rj, q_pair, act, in_lim))
                    cand_pts.extend(pts3)
            cand_dists = geometry.check(np.array(cand_pts, dtype=float))['distances'].reshape(len(cand_specs), 3, -1)
            scored = []
            for (li, rj, q_pair, act, in_lim), d3 in zip(cand_specs, cand_dists):
                p_min = float(d3.min())
                if in_lim and _admissible(p_min, left_list[li], right_list[rj]):
                    scored.append((float(pair_cost[li, rj]), -p_min, li, rj, q_pair, act, p_min))
            scored.sort(key=lambda item: (item[0], item[1]))
            for _, _, li, rj, q_pair, act, p_min in scored:
                if li not in matched_l and rj not in matched_r:
                    result.append(_make_paired_dict(left_list[li], right_list[rj], split, q_pair, act, p_min))
                    matched_l.add(li)
                    matched_r.add(rj)

        for li in range(len(left_list)):
            if li not in matched_l:
                result.append(left_list[li])
        for rj in range(len(right_list)):
            if rj not in matched_r:
                result.append(right_list[rj])
        result.extend(other_list)

    return result


def _order_poses_2opt(poses, home_rad):
    n = len(poses)
    if n <= 1:
        return list(poses)
    home = np.asarray(home_rad, dtype=float)
    pts = np.vstack([home[None, :], np.array([r['q_rad'] for r in poses], dtype=float)])
    dist = cdist(pts, pts, metric='chebyshev')

    unvisited = set(range(1, n + 1))
    tour = [0]
    while unvisited:
        cur = tour[-1]
        nxt = min(unvisited, key=lambda j: (float(dist[cur, j]), j))
        tour.append(nxt)
        unvisited.remove(nxt)
    tour.append(0)

    improved = True
    while improved:
        improved = False
        for i in range(1, n):
            for k in range(i + 1, n + 1):
                before = dist[tour[i - 1], tour[i]] + dist[tour[k], tour[k + 1]]
                after = dist[tour[i - 1], tour[k]] + dist[tour[i], tour[k + 1]]
                if after < before - 1e-12:
                    tour[i:k + 1] = tour[i:k + 1][::-1]
                    improved = True

    return [poses[idx - 1] for idx in tour[1:-1]]


def _build_chained_static_motions(ordered_poses, settings, geometry, margin, *, max_speed_rad_s=None):
    home = np.asarray(settings['home_rad'], dtype=float)
    approach_offset_rad = float(settings['approach_offset_rad'])
    timing = settings['timing']
    envelope = settings['planning_envelope']
    return_anchor_s = float(settings.get('return_anchor_s', timing['minimum_transition_s']))

    poses = [deepcopy(r) for r in ordered_poses]
    via_home = [False] * len(poses)

    while True:
        sub_motions = []
        seg_owners = []
        chain_test = Motion('static_chain_gate', 'gravity', home)
        current_q = home.copy()

        for k, row in enumerate(poses):
            q = np.asarray(row['q_rad'], dtype=float)
            active = list(row['active_joints'])
            approach = list(row.get('approach_joints', active))
            split = row['split']
            pose_id = row['id']
            near_minus = q.copy(); near_minus[approach] -= approach_offset_rad
            near_plus = q.copy(); near_plus[approach] += approach_offset_rad
            conditions_minus = dict(
                batch=split,
                pose=pose_id,
                approach=-1,
                active_arm_group=row['arm'],
                active_joints=active,
                approach_joints=approach,
                candidate_id=pose_id,
                target_q_rad=q.tolist(),
                planned_measure_s=timing['measure_s'],
            )
            m_minus = Motion(f'static_{split}_{pose_id}_minus', 'gravity', current_q, conditions_minus)
            roles_minus = []
            at_home = bool(np.allclose(current_q, home, atol=1e-12))
            if via_home[k] and not at_home:
                _move(m_minus, home, timing['minimum_transition_s'], envelope, 'via_home')
                roles_minus.append((k, 'via_home'))
                _move(m_minus, near_minus, timing['minimum_transition_s'], envelope, 'approach')
                roles_minus.append((k, 'from_home_approach'))
            else:
                _move(m_minus, near_minus, timing['minimum_transition_s'], envelope, 'approach')
                roles_minus.append((k, 'from_home_approach' if at_home else 'inter_pose_approach'))
            _move(m_minus, q, timing['minimum_paired_approach_s'], envelope, 'paired_approach')
            roles_minus.append((k, 'local'))
            m_minus.add(timing['settle_s'], q, label='settle')
            roles_minus.append((k, 'local'))
            m_minus.add(timing['measure_s'], q, label='static_measure')
            roles_minus.append((k, 'local'))
            m_minus.knots = [round(t * 1e9) * 1e-9 for t in m_minus.knots]

            m_plus = Motion(f'static_{split}_{pose_id}_plus', 'gravity', q, {**conditions_minus, 'approach': 1})
            _move(m_plus, near_plus, timing['minimum_paired_approach_s'], envelope, 'approach')
            _move(m_plus, q, timing['minimum_paired_approach_s'], envelope, 'paired_approach')
            m_plus.add(timing['settle_s'], q, label='settle')
            m_plus.add(timing['measure_s'], q, label='static_measure')
            m_plus.knots = [round(t * 1e9) * 1e-9 for t in m_plus.knots]
            roles_plus = [(k, 'local')] * 4

            for sub_m, sub_roles in ((m_minus, roles_minus), (m_plus, roles_plus)):
                for i, lbl in enumerate(sub_m.labels):
                    chain_test.add(sub_m.knots[i + 1] - sub_m.knots[i], sub_m.states[i + 1][0], label=lbl)
                seg_owners.extend(sub_roles)
                sub_motions.append(sub_m)

            current_q = q.copy()

        if not np.allclose(current_q, home, atol=1e-12):
            ret_m = Motion('return_preset_anchor', 'gravity', current_q)
            _move(ret_m, home, return_anchor_s, envelope, 'return_preset_anchor')
            ret_m.knots = [round(t * 1e9) * 1e-9 for t in ret_m.knots]
            chain_test.add(ret_m.knots[1] - ret_m.knots[0], home, label='return_preset_anchor')
            seg_owners.append((len(poses) - 1, 'final_return'))

        chain_test.knots = [round(t * 1e9) * 1e-9 for t in chain_test.knots]
        gate = _gate_rest_to_rest_motion(geometry, chain_test, margin)
        if gate['passed']:
            speed = float(np.max(derivative_bounds(chain_test.polynomial())))
            if max_speed_rad_s is not None and speed > max_speed_rad_s * (1.0 + 1e-9):
                raise ValueError(f'静态姿态链参考速度 {speed:.4f} rad/s 超出上限 {max_speed_rad_s:.4f} rad/s')
            certified = dict(
                knots=list(chain_test.knots),
                states=[s.copy() for s in chain_test.states],
                labels=list(chain_test.labels),
                clearance_m=float(margin),
                gate=gate,
            )
            return [dict(m.metadata(), trial_name=m.name) for m in sub_motions], poses, certified

        if not gate['joint_limits']['passed'] or not gate['segments']:
            raise ValueError(f'静态姿态链未通过 URDF 连续包络检查（{gate.get("reason")}）')

        bad_seg = gate['segments'][-1]
        pose_k, role = seg_owners[int(bad_seg['segment'])]
        if role == 'inter_pose_approach' and not via_home[pose_k]:
            via_home[pose_k] = True
            continue

        split_idx = (pose_k - 1) if role == 'via_home' else pose_k
        target_pose = poses[split_idx]
        if target_pose.get('arm') == 'both' and 'left_pose' in target_pose and 'right_pose' in target_pose:
            poses[split_idx:split_idx + 1] = [deepcopy(target_pose['left_pose']), deepcopy(target_pose['right_pose'])]
            via_home[split_idx:split_idx + 1] = [via_home[split_idx], True]
            continue

        raise _SinglePoseGateFailure(target_pose['id'], bad_seg)


def build_small_static_spec(small_cfg, robot_config, geometry):
    home = ready_position()
    return_anchor_s = float(small_cfg['timing']['minimum_transition_s'])
    settings = dict(
        home_rad=home.tolist(),
        approach_offset_rad=float(np.deg2rad(small_cfg['approach_offset_deg'])),
        timing=small_cfg['timing'],
        planning_envelope=small_cfg['planning_envelope'],
        return_anchor_s=return_anchor_s,
    )
    candidates = []
    for joint_name in small_cfg['joints']:
        j = NAMES.index(joint_name)
        arm = 'left' if j < 7 else 'right'
        short = joint_name.replace('_joint', '')
        for split, angle in (('train', float(small_cfg['target_offset_deg'])), ('validation', -float(small_cfg['target_offset_deg']))):
            q = home.copy()
            q[j] += np.deg2rad(angle)
            candidates.append(dict(
                id=f'{short}_{"pos" if angle > 0 else "neg"}',
                arm=arm,
                split=split,
                q_rad=q.tolist(),
                family='single_axis',
                active_joints=[j],
                offsets_deg=[angle],
                status='selected',
            ))

    parallel_arms = bool(small_cfg.get('parallel_arms', True))
    direct_pose_chain = bool(small_cfg.get('direct_pose_chain', True))
    margin = float(robot_config['scene']['clearance_m'])

    if not parallel_arms and not direct_pose_chain:
        motions = []
        for row in candidates:
            for m in paired_motions(row, settings, split=row['split']):
                m.conditions['active_joints'] = list(row['active_joints'])
                m.knots = [round(t * 1e9) * 1e-9 for t in m.knots]
                proof = gate_motion(geometry, m, margin)
                speed = float(np.max(derivative_bounds(m.polynomial())))
                if not proof['passed'] or speed > 0.05 * (1 + 1e-9):
                    raise ValueError(f'small 静态姿态 {m.name} 未通过连续碰撞/URDF/速度检查')
                motions.append(dict(m.metadata(), trial_name=m.name))
        return dict(
            kind='static',
            home_rad=home.tolist(),
            planning_envelope=small_cfg['planning_envelope'],
            motions=motions,
            candidates=candidates,
        )

    home_gate = gate_motion(geometry, Motion('home', 'gravity', home).add(1.0, home), margin)
    if not home_gate['passed']:
        raise ValueError('small 模式准备位锚点未通过碰撞/URDF 检查')

    working = _pair_arm_poses(
        candidates, settings['approach_offset_rad'], geometry, margin, parallel_arms=parallel_arms, home_rad=home
    )
    if direct_pose_chain:
        ordered = _order_poses_2opt(working, home)
        try:
            motions, final_poses, certified = _build_chained_static_motions(
                ordered, settings, geometry, margin, max_speed_rad_s=0.05
            )
        except _SinglePoseGateFailure as exc:
            raise ValueError(f'small 静态姿态 {exc.pose_id} 未通过连续碰撞/URDF/速度检查') from exc
        return dict(
            kind='static',
            home_rad=home.tolist(),
            return_anchor_s=return_anchor_s,
            planning_envelope=small_cfg['planning_envelope'],
            motions=motions,
            candidates=final_poses,
            _certified_chain=certified,
        )

    motions = []
    for row in working:
        for m in paired_motions(row, settings, split=row['split']):
            m.conditions['active_joints'] = list(row['active_joints'])
            m.knots = [round(t * 1e9) * 1e-9 for t in m.knots]
            proof = gate_motion(geometry, m, margin)
            speed = float(np.max(derivative_bounds(m.polynomial())))
            if not proof['passed'] or speed > 0.05 * (1 + 1e-9):
                raise ValueError(f'small 静态姿态 {m.name} 未通过连续碰撞/URDF/速度检查')
            motions.append(dict(m.metadata(), trial_name=m.name))
    return dict(
        kind='static',
        home_rad=home.tolist(),
        planning_envelope=small_cfg['planning_envelope'],
        motions=motions,
        candidates=working,
    )


def build_full_static_spec(full_cfg, robot_config, engine, geometry):
    home = np.deg2rad(np.asarray(full_cfg['home_deg'], dtype=float))
    if home.shape != (14,) or not np.isfinite(home).all():
        raise ValueError('full.home_deg 须为 14 个有限角度')
    margin = float(robot_config['scene']['clearance_m'])
    home_gate = gate_motion(geometry, Motion('home', 'gravity', home).add(1.0, home), margin)
    if not home_gate['passed']:
        raise ValueError('full 模式锚点姿态未通过碰撞/URDF 检查')
    return_anchor_s = float(full_cfg['timing']['minimum_transition_s'])
    scan_cfg = dict(
        home_rad=home.tolist(),
        approach_offset_rad=float(np.deg2rad(full_cfg['approach_offset_deg'])),
        single_axis_offsets_deg=list(full_cfg['single_axis_offsets_deg']),
        combination_offsets_deg=list(full_cfg['combination_offsets_deg']),
        arms={
            arm: dict(
                single_axis_joints=[n for n in NAMES if n.startswith(arm + '_')],
                combination_joints=list(full_cfg['combination_joints'][arm]),
            )
            for arm in ('left', 'right')
        },
        timing=full_cfg['timing'],
        planning_envelope=full_cfg['planning_envelope'],
        return_anchor_s=return_anchor_s,
        selection=dict(
            train_poses_per_arm=int(full_cfg['train_poses_per_arm']),
            validation_poses_per_arm=int(full_cfg['validation_poses_per_arm']),
            validation_seed=int(full_cfg.get('validation_seed', 20261030)),
            information_ridge=float(full_cfg.get('information_ridge', 1e-6)),
            rank_rtol=float(full_cfg.get('rank_rtol', 1e-7)),
        ),
    )
    rows, _ = candidate_pool(scan_cfg, engine)
    # Full H1 measures every joint on the sampled arm, not only the joints
    # varied to generate this target. Reverse stiction on all seven axes.
    for row in rows:
        row['approach_joints'] = list(range(7)) if row['arm'] == 'left' else list(range(7, 14))
    screen_candidates(rows, scan_cfg, geometry, margin)

    parallel_arms = bool(full_cfg.get('parallel_arms', True))
    direct_pose_chain = bool(full_cfg.get('direct_pose_chain', True))

    if not direct_pose_chain and not parallel_arms:
        motions = []
        selected = []
        for arm in ('left', 'right'):
            arm_plans, arm_poses, arm_report = _select_arm(
                [r for r in rows if r['arm'] == arm], scan_cfg, engine, geometry, margin
            )
            if not arm_report['passed']:
                raise ValueError(f'{arm} 臂全姿态采样未满足秩或配额要求：{arm_report}')
            selected.extend(arm_poses)
            for m, _ in arm_plans:
                m.conditions['active_joints'] = list(next(p['active_joints'] for p in arm_poses if p['id'] == m.conditions['candidate_id']))
                m.knots = [round(t * 1e9) * 1e-9 for t in m.knots]
                motions.append(dict(m.metadata(), trial_name=m.name))
        return dict(
            kind='static',
            home_rad=home.tolist(),
            planning_envelope=full_cfg['planning_envelope'],
            motions=motions,
            candidates=selected,
        )

    permanently_rejected = set()
    while True:
        selected = []
        for r in rows:
            if r['id'] in permanently_rejected:
                r['status'] = 'rejected'
            elif r.get('status') == 'selected':
                r['status'] = 'endpoint_admissible'
                r['split'] = None
        for arm in ('left', 'right'):
            arm_poses, arm_report = _select_arm_poses([r for r in rows if r['arm'] == arm], scan_cfg, engine)
            if not arm_report['passed']:
                raise ValueError(f'{arm} 臂全姿态采样未满足秩或配额要求：{arm_report}')
            selected.extend(arm_poses)

        working = _pair_arm_poses(
            selected, scan_cfg['approach_offset_rad'], geometry, margin, parallel_arms=parallel_arms, home_rad=home
        )
        if direct_pose_chain:
            ordered = _order_poses_2opt(working, home)
            try:
                motions, final_poses, certified = _build_chained_static_motions(
                    ordered,
                    scan_cfg,
                    geometry,
                    margin,
                    max_speed_rad_s=float(robot_config['maximum_velocity_rad_s']),
                )
                return dict(
                    kind='static',
                    home_rad=home.tolist(),
                    return_anchor_s=return_anchor_s,
                    planning_envelope=full_cfg['planning_envelope'],
                    motions=motions,
                    candidates=final_poses,
                    _certified_chain=certified,
                )
            except _SinglePoseGateFailure as exc:
                permanently_rejected.add(exc.pose_id)
                continue
        else:
            motions = []
            failed_single_id = None
            for row in working:
                sub_ms = paired_motions(row, scan_cfg, split=row['split'])
                gates = [gate_motion(geometry, m, margin) for m in sub_ms]
                if not all(g['passed'] for g in gates):
                    if row.get('arm') == 'both':
                        for sub_row in (row['left_pose'], row['right_pose']):
                            for m in paired_motions(sub_row, scan_cfg, split=sub_row['split']):
                                m.conditions['active_joints'] = list(sub_row['active_joints'])
                                m.knots = [round(t * 1e9) * 1e-9 for t in m.knots]
                                if not gate_motion(geometry, m, margin)['passed']:
                                    failed_single_id = sub_row['id']
                                    break
                                motions.append(dict(m.metadata(), trial_name=m.name))
                            if failed_single_id is not None:
                                break
                    else:
                        failed_single_id = row['id']
                    if failed_single_id is not None:
                        break
                else:
                    for m in sub_ms:
                        m.conditions['active_joints'] = list(row['active_joints'])
                        m.knots = [round(t * 1e9) * 1e-9 for t in m.knots]
                        motions.append(dict(m.metadata(), trial_name=m.name))
            if failed_single_id is not None:
                permanently_rejected.add(failed_single_id)
                continue
            return dict(
                kind='static',
                home_rad=home.tolist(),
                planning_envelope=full_cfg['planning_envelope'],
                motions=motions,
                candidates=working,
            )


def _concat_rest_motions(first, second):
    if not np.array_equal(first.states[-1], second.states[0]):
        raise ValueError('串联运动段端点状态不连续')
    joined = Motion(second.name, second.stage, first.home, {**first.conditions, **second.conditions})
    offset_ns = round(first.duration * 1e9)
    joined.knots = first.knots[:] + [(offset_ns + round(t * 1e9)) * 1e-9 for t in second.knots[1:]]
    joined.states = first.states[:] + second.states[1:]
    joined.labels = first.labels[:] + second.labels[:]
    return joined, offset_ns * 1e-9


def plan_h1(q_start, robot_config, recipe, plan_dir):
    robot_cfg = load_robot_config(robot_config)
    rec = load_h1_recipe(recipe)
    q_start = np.asarray(q_start, dtype=float)
    if q_start.shape != (14,) or not np.isfinite(q_start).all():
        raise ValueError('起点须为 14 个有限关节角')

    engine, source = model_for(robot_cfg)
    geometry = collision_for(robot_cfg, engine)

    ready_cert = None
    prefix = None
    formal_start = q_start.copy()
    hold = robot_cfg.get('non_arm_hold') or {}
    ramp = hold['ramp_s'] if hold.get('enabled') else 0.
    if rec['ready_pose']:
        prefix, ready_cert = preparation(q_start, engine, robot_cfg['maximum_velocity_rad_s'],
                                         start_hold_s=max(0.5, ramp))
        formal_start = ready_position()

    static_spec = None
    certified_chain = None
    trials = []
    if rec['mode'] == 'smoke':
        smoke = rec['smoke_jog']
        motion, _ = jog_motion(
            robot_cfg,
            formal_start,
            list(NAMES),
            NAMES[smoke['joint_id'] - 12],
            smoke['amplitude_deg'],
            anchor_rad=smoke.get('anchor_rad'),
        )
    else:
        static_spec = (
            build_small_static_spec(rec['small'], robot_cfg, geometry)
            if rec['mode'] == 'small'
            else build_full_static_spec(rec['full'], robot_cfg, engine, geometry)
        )
        certified_chain = static_spec.pop('_certified_chain', None)
        stat_motion, trials = static_motion(robot_cfg, formal_start, static_spec)
        if rec['smoke_jog']['enabled']:
            certified_chain = None
            smoke = rec['smoke_jog']
            smoke_motion, _ = jog_motion(
                robot_cfg,
                formal_start,
                list(NAMES),
                NAMES[smoke['joint_id'] - 12],
                smoke['amplitude_deg'],
                anchor_rad=smoke.get('anchor_rad'),
            )
            smoke_motion.knots = [round(t * 1e9) * 1e-9 for t in smoke_motion.knots]
            motion, shift_s = _concat_rest_motions(smoke_motion, stat_motion)
            for trial in trials:
                for key in ('begin_s', 'end_s', 'measure_begin_s', 'measure_end_s'):
                    trial[key] += shift_s
        else:
            motion = stat_motion

    motion.knots = [round(t * 1e9) * 1e-9 for t in motion.knots]
    extra_ns = 0
    if ramp and ready_cert is None:
        motion, extra_ns = ensure_leading_hold(motion, ramp)
    for trial in trials:
        for key in ('begin_s', 'end_s', 'measure_begin_s', 'measure_end_s'):
            trial[key] = (extra_ns + round(trial[key] * 1e9)) * 1e-9

    plan_dir = Path(plan_dir).resolve()
    plan_dir.mkdir(parents=True, exist_ok=False)

    formal_curve = motion.polynomial()
    gate = _gate_with_certified_chain(
        geometry, motion, formal_curve, robot_cfg['scene']['clearance_m'], certified_chain
    )
    if ready_cert is not None:
        gate.update(scope='formal_calibration_reference_only', execution_offset_ns=ready_cert['calibration_begin_ns'])
        write_json(plan_dir / 'preparation.json', ready_cert)
        write_json(plan_dir / 'calibration_motion.json', motion.metadata(), compact=True)
    write_json(plan_dir / 'collision_reference.json', gate)
    if not gate['passed']:
        raise ValueError(f'H1 正式参考连续碰撞/URDF 检查未通过（{gate.get("reason", "clearance")}）；未发送命令')
    if np.max(derivative_bounds(formal_curve)) > robot_cfg['maximum_velocity_rad_s'] * (1 + 1e-9):
        raise ValueError('H1 正式参考速度超出上限')

    if ready_cert is not None:
        motion = prepend(prefix, motion)
        for trial in trials:
            for key in ('begin_s', 'end_s', 'measure_begin_s', 'measure_end_s'):
                trial[key] = (ready_cert['calibration_begin_ns'] + round(trial[key] * 1e9)) * 1e-9

    effort_rows = reference_effort(
        motion, engine, robot_cfg, ready_cert, feedforward_mode=rec['gravity_feedforward']
    )
    bundle = export_motion(motion, kp=robot_cfg['kp'], kd=robot_cfg['kd'], effort=effort_rows,
                           non_arm_hold=hold, observed_velocity_limit_rad_s=robot_cfg['maximum_velocity_rad_s'])
    bundle.update(
        purpose='shm_calib_ready_then_collision_checked' if ready_cert else 'shm_calib_collision_checked',
        gravity_feedforward=rec['gravity_feedforward'],
        observed_velocity_limit_rad_s=robot_cfg['maximum_velocity_rad_s'],
    )
    if ready_cert is not None:
        bundle.update(
            pd_schedule=pd_schedule(ready_cert, robot_cfg['kp'], robot_cfg['kd']),
            markers=[
                dict(t_ns=0, event='ready_begin'),
                dict(t_ns=ready_cert['move_end_ns'], event='ready_field_handover_begin'),
                dict(t_ns=ready_cert['calibration_begin_ns'], event='h1_calibration_begin'),
            ],
        )
    if trials:
        markers = bundle.setdefault('markers', [])
        for i, trial in enumerate(trials, 1):
            markers.extend([
                dict(t_ns=round(trial['measure_begin_s'] * 1e9), event=f'static_{i:02d}_measure_begin'),
                dict(t_ns=round(trial['measure_end_s'] * 1e9), event=f'static_{i:02d}_measure_end'),
            ])
        markers.sort(key=lambda row: row['t_ns'])

    write_json(plan_dir / 'reference.json', bundle, compact=True)
    write_json(plan_dir / 'motion.json', motion.metadata(), compact=True)

    metadata = dict(
        stage_name='h1',
        stage=motion.stage,
        mode=rec['mode'],
        robot_config=robot_cfg,
        recipe=rec,
        static_motions=static_spec['motions'] if static_spec is not None else [],
        model_context=robot_cfg['model_context'],
        joint_names=list(NAMES),
        source_model_sha256=source['sha256'],
        trials=trials,
        duration_ns=round(motion.duration * 1e9),
        minimum_reference_distance_m=gate.get('minimum_distance_m'),
        **run_marker(),
    )
    if ready_cert is not None:
        metadata['preparation'] = ready_cert
    write_json(plan_dir / 'plan.json', metadata)
    return metadata


def analyse_h1(plan_dir, capture_dir, analysis_dir, *, windows_only=False):
    plan_dir = Path(plan_dir).resolve()
    capture_dir = Path(capture_dir).resolve()
    if (capture_dir / 'result.json').is_file():
        wrapper = json.loads((capture_dir / 'result.json').read_text(encoding='utf-8'))
        if isinstance(wrapper.get('effective_capture'), str):
            if not wrapper.get('execution_completed'):
                raise ValueError('H1 采集未完成，不执行拟合分析')
            if wrapper.get('effective_plan'):
                plan_dir = (capture_dir / wrapper['effective_plan']).resolve()
            capture_dir = (capture_dir / wrapper['effective_capture']).resolve()

    native_dir = capture_dir / 'native' if (capture_dir / 'native').is_dir() else capture_dir
    native = text_result(native_dir / 'result.txt')
    if not native or native.get('execution_completed') != '1':
        raise ValueError('H1 执行未正常完成，记录仅作故障诊断')
    if native.get('recording_stopped') != '0':
        raise ValueError('H1 记录中途停录，不作完整采集分析')

    m = json.loads((plan_dir / 'plan.json').read_text(encoding='utf-8'))
    robot_cfg = m['robot_config']
    engine, source = model_for(robot_cfg)
    if source['sha256'] != m['source_model_sha256']:
        raise ValueError('H1 分析时的模型资产与规划时不一致')

    analysis_dir = Path(analysis_dir).resolve()
    analysis_dir.mkdir(parents=True, exist_ok=False)

    geometry = collision_for(robot_cfg, engine)
    reader = ObservationReader(capture_dir)
    motion = json.loads((plan_dir / 'motion.json').read_text(encoding='utf-8'))
    curve = BPoly.from_derivatives(motion['knots_s'], np.asarray(motion['states']))
    ready_cert = m.get('preparation')
    calibration_begin = ready_cert['calibration_begin_ns'] * 1e-9 if ready_cert else 0.0
    interval = motion['conditions'].get('comparison_window_s', [calibration_begin, m['duration_ns'] * 1e-9])

    arr = reader.arrays()
    receive_ns = arr['receive_ns']
    obs_ns = arr['observation_time_ns']
    elapsed_s = arr['elapsed_s']
    phase = arr['phase']
    seg = arr['clock_segment']
    ambig = arr['time_ambiguous']
    q_all = arr['q']
    dq_all = arr['dq']
    effort_raw = arr['effort_raw']
    count = int(len(receive_ns))

    formal_mask = (
        np.ones(count, dtype=bool)
        if ready_cert is None
        else ((phase == 3) & (elapsed_s >= calibration_begin))
    )
    checked = int(formal_mask.sum())
    excluded = count - checked

    proofs = []
    padded = np.r_[False, formal_mask, False]
    edges = np.flatnonzero(padded[1:] != padded[:-1]).reshape(-1, 2)
    for run_start, run_end in edges:
        run_len = int(run_end - run_start)
        q_run = q_all[run_start:run_end]
        ns_run = receive_ns[run_start:run_end]
        for s in range(0, run_len - 1, 1023):
            q_blk = q_run[s:s + 1024]
            ns_blk = ns_run[s:s + 1024]
            proof = certify_polyline(
                geometry,
                q_blk,
                (ns_blk - ns_blk[0]) * 1e-9,
                clearance_m=robot_cfg['scene']['clearance_m'],
                block_samples=1024,
            )
            if not proof['passed']:
                write_json(analysis_dir / 'collision_received_failure.json', proof)
                raise ValueError('实测接收位置折线未通过连续碰撞检查')
            proofs.append(dict(
                passed=True,
                minimum_distance_m=proof.get('minimum_distance_m', proof.get('minimum_sampled_distance_m')),
                rows=int(len(q_blk)),
            ))

    if count < 2 or checked < 2:
        raise ValueError('H1 正式段缺少足够的完整反馈帧')

    first_receive = int(receive_ns[0])
    last_receive = int(receive_ns[-1])
    relative_ns = obs_ns - np.int64(reader.origin)
    is_monotonic = bool(np.all(obs_ns[1:] >= obs_ns[:-1]))

    tmask = (elapsed_s >= interval[0]) & (elapsed_s < interval[1])
    if np.any(tmask):
        q_ref = np.asarray(curve(elapsed_s[tmask]), dtype=np.float64).reshape(-1, 14)
        err = q_all[tmask] - q_ref
        track_res = _trapezoid_time_mean(obs_ns[tmask], seg[tmask], ambig[tmask], err * err)
    else:
        track_res = _trapezoid_time_mean(obs_ns[:0], seg[:0], ambig[:0], q_all[:0])

    trial_metadata = {row['trial_name']: row for row in m.get('static_motions', [])}
    effort_zero = np.asarray(robot_cfg['effort_zero_nm'], dtype=np.float64)[None, :]
    effort_gain = np.asarray(robot_cfg['effort_gain'], dtype=np.float64)[None, :]

    means = []
    for trial in m['trials']:
        b_s = float(trial['measure_begin_s'])
        e_s = float(trial['measure_end_s'])
        if is_monotonic:
            lo_i = int(np.searchsorted(elapsed_s, b_s, side='left'))
            hi_i = int(np.searchsorted(elapsed_s, e_s, side='left'))
            sl = slice(lo_i, hi_i)
        else:
            sl = (elapsed_s >= b_s) & (elapsed_s < e_s)

        q_w = q_all[sl]
        dq_w = dq_all[sl]
        torque_w = (effort_raw[sl] - effort_zero) * effort_gain
        win_res = _trapezoid_time_mean(obs_ns[sl], seg[sl], ambig[sl], np.hstack([q_w, torque_w]))
        speed = float(np.max(np.abs(dq_w))) if len(q_w) else 0.0

        pa_result = None
        meta_row = trial_metadata.get(trial['name'])
        if meta_row and 'paired_approach' in meta_row['labels']:
            k_lbl = meta_row['labels'].index('paired_approach')
            origin_ns = round(trial['begin_s'] * 1e9)
            pa_begin = origin_ns + round(meta_row['knots_s'][k_lbl] * 1e9)
            pa_end = origin_ns + round(meta_row['knots_s'][k_lbl + 1] * 1e9)
            pa_width = min(50_000_000, (pa_end - pa_begin) // 2)
            pa_expected = np.asarray(meta_row['states'][k_lbl + 1][0]) - meta_row['states'][k_lbl][0]

            if is_monotonic:
                a_lo = int(np.searchsorted(relative_ns, pa_begin, side='left'))
                a_hi = int(np.searchsorted(relative_ns, pa_end, side='left'))
                f_hi = int(np.searchsorted(relative_ns, pa_begin + pa_width, side='left'))
                l_lo = int(np.searchsorted(relative_ns, pa_end - pa_width, side='left'))
                a_sl = slice(a_lo, a_hi)
                f_sl = slice(a_lo, min(a_hi, f_hi))
                l_sl = slice(max(a_lo, l_lo), a_hi)
            else:
                a_sl = (relative_ns >= pa_begin) & (relative_ns < pa_end)
                f_sl = a_sl & (relative_ns < pa_begin + pa_width)
                l_sl = a_sl & (relative_ns >= pa_end - pa_width)

            segs_in_span = np.unique(seg[a_sl])
            ambig_in_span = bool(np.any(ambig[a_sl]))
            first_res = _trapezoid_time_mean(obs_ns[f_sl], seg[f_sl], ambig[f_sl], q_all[f_sl])
            last_res = _trapezoid_time_mean(obs_ns[l_sl], seg[l_sl], ambig[l_sl], q_all[l_sl])
            pa_avail = (
                first_res['mean'] is not None
                and last_res['mean'] is not None
                and not ambig_in_span
                and len(segs_in_span) == 1
            )
            pa_result = dict(
                available=pa_avail,
                reference_displacement_rad=pa_expected,
                observed_displacement_rad=(last_res['mean'] - first_res['mean']) if pa_avail else None,
                endpoint_window_ns=pa_width,
                first_rows=first_res['rows'],
                last_rows=last_res['rows'],
                unavailable_reason=(
                    None if pa_avail
                    else first_res['error'] or last_res['error'] or 'clock ambiguity or missing endpoint exposure'
                ),
            )

        value = win_res['mean']
        available = value is not None
        means.append(dict(
            name=trial['name'],
            conditions=trial['conditions'],
            q=value[:14] if available else None,
            torque=value[14:] if available else None,
            observation_rows=win_res['rows'],
            observation_time_groups=win_res['groups'],
            available=available,
            unavailable_reason=win_res['error'] or (None if available else 'no distinct observation times in window'),
            observed_duration_s=(win_res['last'] - win_res['first']) * 1e-9 if win_res['first'] is not None else 0.0,
            maximum_received_speed_rad_s=speed,
            maximum_observation_gap_s=win_res['maximum_gap_ns'] * 1e-9,
            position_span_rad=win_res['high'][:14] - win_res['low'][:14] if win_res['low'] is not None else None,
            paired_approach=pa_result,
        ))

    position = track_res['mean']
    report = dict(
        stage_name='h1',
        passed=True,
        execution_completed=True,
        collection_complete=True,
        mode=m['mode'],
        stage=m['stage'],
        feedback_rows=count,
        feedback_receive_hz=(count - 1) * 1e9 / (last_receive - first_receive),
        observation_time_origin_ns=reader.origin,
        first_shm_write_mono_ns=int(native['first_shm_write_mono_ns']),
        analysis_available=(position is not None) if m['stage'] == 'position' else False,
        tracking_rmse_rad=np.sqrt(position[:14]) if position is not None else None,
        tracking_unavailable_reason=track_res['error'],
    )
    if ready_cert is not None:
        report.update(preparation=ready_cert, calibration_begin_s=calibration_begin)

    write_json(
        analysis_dir / 'collision_received_interpolant.json',
        dict(
            passed=True,
            chunks=proofs,
            feedback_rows=count,
            collision_checked_rows=checked,
            excluded_rows=excluded,
            execution_begin_s=calibration_begin,
        ),
    )
    write_json(analysis_dir / 'static_windows.json', means)

    if m['stage'] != 'position':
        report['static_diagnostics'] = write_static_diagnostics(means, engine, analysis_dir, m['joint_names'])
        if windows_only:
            report.update(analysis_available=any(r['available'] for r in means), stage='gravity_windows')
        else:
            try:
                fit = write_gravity_candidate(means, robot_cfg, analysis_dir, m)
                report.update(fit)
                report['analysis_available'] = True
            except (ValueError, np.linalg.LinAlgError) as error:
                report.update(analysis_available=False, analysis_unavailable_reason=str(error), fit_succeeded=False)

    write_json(analysis_dir / 'report.json', report)
    return report


class H1GravityStage(Stage):
    name = 'h1'
    title = 'H1 静态重力标定'
    default_config = DEFAULT_H1_CONFIG
    implemented = True

    def load_recipe(self, config_path=None, **overrides):
        return load_h1_recipe(config_path or self.default_config, **overrides)

    def plan(self, q_start, robot_config, recipe, plan_dir, *, session=None):
        return plan_h1(q_start, robot_config, recipe, plan_dir)

    def analyse(self, plan_dir, capture_dir, analysis_dir, *, session=None):
        return analyse_h1(plan_dir, capture_dir, analysis_dir)

    def write_summary(self, stage_dir, plan_meta, capture_summary, report):
        stage_dir = Path(stage_dir).resolve()
        lines = [
            f'# {self.title}报告',
            '',
            f'- 模式：`{plan_meta["mode"]}`（准备位前缀：`{"开启" if plan_meta.get("preparation") else "关闭"}`）',
            f'- 轨迹总时长：{plan_meta["duration_ns"] * 1e-9:.2f} s；静态测量窗口：{len(plan_meta["trials"])} 个',
            f'- 规划最小碰撞间距：{plan_meta.get("minimum_reference_distance_m", 0.0) * 1000:.2f} mm（门限 10.00 mm）',
        ]
        if report is None:
            lines.append('- 运行状态：仅离线规划（`--plan-only`），未连接设备。')
            path = stage_dir / 'report.md'
            path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
            return path

        lines.extend([
            f'- 采集反馈：{report["feedback_rows"]} 帧（平均 {report["feedback_receive_hz"]:.1f} Hz），正式段接收折线碰撞复核通过',
        ])
        if report.get('static_diagnostics'):
            diag = report['static_diagnostics']
            lines.append(
                f'- 可用静态窗口：{diag["available_windows"]}/{diag["total_windows"]}（成对目标 {diag["paired_targets"]} 个）'
            )

        if report.get('fit_succeeded'):
            nom_all = report['overall_nominal_validation_rmse_nm']
            cal_all = report['overall_validation_rmse_nm']
            acc = report.get('acceptance') or {}
            gate_str = '通过（自动采纳）' if acc.get('accepted', True) else f'未通过（{acc.get("reason", "degraded")}，自动回退名义先验）'
            lines.extend([
                f'- PSD 重力拟合：成功（矩阵秩 {report["fit"]["rank"]}/{report["fit"]["columns"]}，URDF 独立重载通过）',
                f'- 有效正反配对留出集力矩 RMSE：名义 **{nom_all:.4f} Nm** → 候选 **{cal_all:.4f} Nm**',
                f'- 防退化验收门禁：{gate_str}',
                f'- 候选模型文件：`{stage_dir / "analysis" / report["candidate_urdf"]}`',
                '',
                '## 逐轴留出验证 RMSE（Nm）',
                '',
                '| SHM ID | 关节 | 有效配对数 | 名义模型 | 拟合候选 | 反向半差最大 |',
                '| ---: | --- | ---: | ---: | ---: | ---: |',
            ])
            nom_vec = np.asarray(report['nominal_validation_rmse_nm'], dtype=float)
            cal_vec = np.asarray(report['validation_rmse_nm'], dtype=float)
            spread_vec = np.asarray(report['paired_approach_half_spread_nm'], dtype=float)
            for j in range(14):
                label = ('左' if j < 7 else '右') + PARTS[j % 7]
                count = report['validation_pairs_per_joint'][j]
                nom = f'{nom_vec[j]:.4f}' if count else '—'
                cal = f'{cal_vec[j]:.4f}' if count else '—'
                lines.append(f'| {j + 12} | {label} (`{NAMES[j]}`) | {count} | {nom} | {cal} | {spread_vec[j]:.4f} |')
        elif m_stage_has_trials := bool(plan_meta['trials']):
            if m_stage_has_trials:
                lines.append(f'- PSD 重力拟合：未生成（{report.get("analysis_unavailable_reason", "无")}）')
        elif report.get('tracking_rmse_rad') is not None:
            rmse_deg = np.rad2deg(np.asarray(report['tracking_rmse_rad'], dtype=float))
            lines.append(f'- 冒烟跟踪最大轴 RMSE：{float(np.max(rmse_deg)):.3f}°')

        path = stage_dir / 'report.md'
        path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
        return path
