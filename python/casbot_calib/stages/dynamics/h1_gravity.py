"""H1 Static Gravity stage: optional ready pose + smoke jog + paired-approach static gravity & PSD fit."""
import bisect
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import BPoly
import yaml
from ...collision import certify_polyline
from ...data import write_json
from ...field_gravity import write_gravity_candidate, write_static_diagnostics
from ...field_motion import jog_motion, static_motion
from ...model_runtime import collision_for, model_for
from ...pipeline.robot import load_robot_config
from ...pipeline.stage import Stage
from ...run_marker import run_marker
from ...shm_data import DIRECTION, NAMES, PARTS, ROOT
from ...shm_fieldwork import text_result
from ...shm_observations import ObservationReader
from ...shm_ready import handover_fraction, pd_schedule, preparation, prepend, ready_position
from ...shm_reference import export_motion
from ...staged_motion import Motion, derivative_bounds, gate_motion
from ...static_field import _select_arm, candidate_pool, paired_motions, screen_candidates

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
        nums = [full['approach_offset_deg'], *full['timing'].values(), *full['planning_envelope'].values()]
        if not np.isfinite(nums).all() or min(nums) <= 0:
            raise ValueError('full 静态参数须为正有限值')
        if int(full['train_poses_per_arm']) < 14 or int(full['validation_poses_per_arm']) < 1:
            raise ValueError('full 模式每臂至少需要 14 个训练姿态和 1 个验证姿态')
        c['full'] = full

    return c


def reference_effort(motion, engine, robot_config, preparation_cert=None, *, feedforward_mode='nominal_reference'):
    if feedforward_mode == 'none':
        return [dict(t_ns=0, value=[0.0] * 14)]
    curve = motion.polynomial()
    last = round(motion.duration * 1e9)
    period = robot_config['command_period_ns']
    rows = []
    previous = None
    for first in range(0, last + period, period * 4096):
        times = np.arange(first, min(last + period, first + period * 4096), period, dtype=np.int64)
        times = np.minimum(times, last)
        torque = engine.gravity_batch(curve(times * 1e-9))
        if preparation_cert is not None:
            torque *= handover_fraction(times, preparation_cert)[:, None]
        if not np.isfinite(torque).all() or np.max(np.abs(torque)) > robot_config['feedforward_limit_nm']:
            raise ValueError('名义重力前馈超出显式力矩上限；不夹取')
        codes = np.trunc(torque * np.asarray(DIRECTION) * 100).astype(np.int64)
        for ns, value, code in zip(times, torque, codes):
            if previous is None or np.any(code != previous):
                rows.append(dict(t_ns=int(ns), value=value.tolist()))
                previous = code.copy()
    return rows


def build_small_static_spec(small_cfg, robot_config, geometry):
    home = ready_position()
    settings = dict(
        home_rad=home.tolist(),
        approach_offset_rad=float(np.deg2rad(small_cfg['approach_offset_deg'])),
        timing=small_cfg['timing'],
        planning_envelope=small_cfg['planning_envelope'],
    )
    motions = []
    candidates = []
    for joint_name in small_cfg['joints']:
        j = NAMES.index(joint_name)
        arm = 'left' if j < 7 else 'right'
        short = joint_name.replace('_joint', '')
        for split, angle in (('train', float(small_cfg['target_offset_deg'])), ('validation', -float(small_cfg['target_offset_deg']))):
            q = home.copy()
            q[j] += np.deg2rad(angle)
            row = dict(
                id=f'{short}_{"pos" if angle > 0 else "neg"}',
                arm=arm,
                split=split,
                q_rad=q.tolist(),
                family='single_axis',
                active_joints=[j],
                offsets_deg=[angle],
                status='selected',
            )
            candidates.append(row)
            for m in paired_motions(row, settings, split=split):
                m.knots = [round(t * 1e9) * 1e-9 for t in m.knots]
                proof = gate_motion(geometry, m, robot_config['scene']['clearance_m'])
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


def build_full_static_spec(full_cfg, robot_config, engine, geometry):
    home = np.deg2rad(np.asarray(full_cfg['home_deg'], dtype=float))
    if home.shape != (14,) or not np.isfinite(home).all():
        raise ValueError('full.home_deg 须为 14 个有限角度')
    margin = float(robot_config['scene']['clearance_m'])
    home_gate = gate_motion(geometry, Motion('home', 'gravity', home).add(1.0, home), margin)
    if not home_gate['passed']:
        raise ValueError('full 模式锚点姿态未通过碰撞/URDF 检查')
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
        selection=dict(
            train_poses_per_arm=int(full_cfg['train_poses_per_arm']),
            validation_poses_per_arm=int(full_cfg['validation_poses_per_arm']),
            validation_seed=int(full_cfg.get('validation_seed', 20261030)),
            information_ridge=float(full_cfg.get('information_ridge', 1e-6)),
            rank_rtol=float(full_cfg.get('rank_rtol', 1e-7)),
        ),
    )
    rows, _ = candidate_pool(scan_cfg, engine)
    screen_candidates(rows, scan_cfg, geometry, margin)
    motions = []
    selected = []
    for arm in ('left', 'right'):
        arm_plans, arm_poses, arm_report = _select_arm([r for r in rows if r['arm'] == arm], scan_cfg, engine, geometry, margin)
        if not arm_report['passed']:
            raise ValueError(f'{arm} 臂全姿态采样未满足秩或配额要求：{arm_report}')
        selected.extend(arm_poses)
        for m, _ in arm_plans:
            m.knots = [round(t * 1e9) * 1e-9 for t in m.knots]
            motions.append(dict(m.metadata(), trial_name=m.name))
    return dict(
        kind='static',
        home_rad=home.tolist(),
        planning_envelope=full_cfg['planning_envelope'],
        motions=motions,
        candidates=selected,
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
    if rec['ready_pose']:
        prefix, ready_cert = preparation(q_start, engine, robot_cfg['maximum_velocity_rad_s'])
        formal_start = ready_position()

    static_spec = None
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
        stat_motion, trials = static_motion(robot_cfg, formal_start, static_spec)
        if rec['smoke_jog']['enabled']:
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
    for trial in trials:
        for key in ('begin_s', 'end_s', 'measure_begin_s', 'measure_end_s'):
            trial[key] = round(trial[key] * 1e9) * 1e-9

    plan_dir = Path(plan_dir).resolve()
    plan_dir.mkdir(parents=True, exist_ok=False)

    gate = gate_motion(geometry, motion, robot_cfg['scene']['clearance_m'])
    if ready_cert is not None:
        gate.update(scope='formal_calibration_reference_only', execution_offset_ns=ready_cert['calibration_begin_ns'])
        write_json(plan_dir / 'preparation.json', ready_cert)
        write_json(plan_dir / 'calibration_motion.json', motion.metadata())
    write_json(plan_dir / 'collision_reference.json', gate)
    if not gate['passed']:
        raise ValueError(f'H1 正式参考连续碰撞/URDF 检查未通过（{gate.get("reason", "clearance")}）；未发送命令')
    if np.max(derivative_bounds(motion.polynomial())) > robot_cfg['maximum_velocity_rad_s'] * (1 + 1e-9):
        raise ValueError('H1 正式参考速度超出上限')

    if ready_cert is not None:
        motion = prepend(prefix, motion)
        for trial in trials:
            for key in ('begin_s', 'end_s', 'measure_begin_s', 'measure_end_s'):
                trial[key] = (ready_cert['calibration_begin_ns'] + round(trial[key] * 1e9)) * 1e-9

    effort_rows = reference_effort(
        motion, engine, robot_cfg, ready_cert, feedforward_mode=rec['gravity_feedforward']
    )
    bundle = export_motion(motion, kp=robot_cfg['kp'], kd=robot_cfg['kd'], effort=effort_rows)
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

    write_json(plan_dir / 'reference.json', bundle)
    write_json(plan_dir / 'motion.json', motion.metadata())

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

    tracking = TimeMean()
    windows = [TimeMean() for _ in m['trials']]
    maximum_speed = [0.0] * len(windows)
    begins = [t['measure_begin_s'] for t in m['trials']]
    ends = [t['measure_end_s'] for t in m['trials']]
    trial_metadata = {row['trial_name']: row for row in m.get('static_motions', [])}
    approaches = []
    approach_map = {}
    for trial in m['trials']:
        meta_row = trial_metadata.get(trial['name'])
        if meta_row and 'paired_approach' in meta_row['labels']:
            span = ApproachDisplacement(trial, meta_row)
            approaches.append(span)
            approach_map[trial['name']] = span
    approach_begins = [span.begin for span in approaches]

    block = []
    proofs = []
    count = 0
    checked = 0
    excluded = 0
    first_receive = None
    last_receive = None

    def prove(rows):
        q = np.array([r['q'] for r in rows])
        ns = np.array([r['receive_ns'] for r in rows], dtype=np.int64)
        proof = certify_polyline(geometry, q, (ns - ns[0]) * 1e-9, clearance_m=robot_cfg['scene']['clearance_m'], block_samples=1024)
        if not proof['passed']:
            write_json(analysis_dir / 'collision_received_failure.json', proof)
            raise ValueError('实测接收位置折线未通过连续碰撞检查')
        proofs.append(dict(
            passed=True,
            minimum_distance_m=proof.get('minimum_distance_m', proof.get('minimum_sampled_distance_m')),
            rows=len(rows),
        ))

    for row in reader.rows():
        count += 1
        if first_receive is None:
            first_receive = row['receive_ns']
        last_receive = row['receive_ns']
        elapsed = row['elapsed_s']
        relative_ns = row['observation_time_ns'] - reader.origin

        approach_index = bisect.bisect_right(approach_begins, relative_ns) - 1
        if 0 <= approach_index < len(approaches):
            approaches[approach_index].add(row, relative_ns)

        if ready_cert is None or (row['phase'] == 3 and elapsed >= calibration_begin):
            checked += 1
            block.append(row)
            if len(block) == 1024:
                prove(block)
                block = [block[-1]]
        else:
            if len(block) > 1:
                prove(block)
            block = []
            excluded += 1

        if interval[0] <= elapsed < interval[1]:
            error = row['q'] - curve(elapsed)
            tracking.add(row, np.r_[error * error, row['q']])

        index = bisect.bisect_right(begins, elapsed) - 1
        if 0 <= index < len(ends) and elapsed < ends[index]:
            torque = (row['effort_raw'] - np.asarray(robot_cfg['effort_zero_nm'])) * np.asarray(robot_cfg['effort_gain'])
            windows[index].add(row, np.r_[row['q'], torque])
            maximum_speed[index] = max(maximum_speed[index], float(np.max(np.abs(row['dq']))))

    if len(block) > 1:
        prove(block)
    if count < 2 or checked < 2:
        raise ValueError('H1 正式段缺少足够的完整反馈帧')

    means = []
    for trial, values, speed in zip(m['trials'], windows, maximum_speed):
        value = values.result()
        available = value is not None
        means.append(dict(
            name=trial['name'],
            conditions=trial['conditions'],
            q=value[:14] if available else None,
            torque=value[14:] if available else None,
            observation_rows=values.rows,
            observation_time_groups=values.groups,
            available=available,
            unavailable_reason=values.error or (None if available else 'no distinct observation times in window'),
            observed_duration_s=(values.last - values.first) * 1e-9 if values.first is not None else 0.0,
            maximum_received_speed_rad_s=speed,
            maximum_observation_gap_s=values.maximum_gap_ns * 1e-9,
            position_span_rad=values.high[:14] - values.low[:14] if values.low is not None else None,
            paired_approach=approach_map[trial['name']].result() if trial['name'] in approach_map else None,
        ))

    position = tracking.result()
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
        tracking_unavailable_reason=tracking.error,
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
            lines.extend([
                f'- PSD 重力拟合：成功（矩阵秩 {report["fit"]["rank"]}/{report["fit"]["columns"]}，URDF 独立重载通过）',
                f'- 留出集整体力矩 RMSE：名义 **{nom_all:.4f} Nm** → 候选 **{cal_all:.4f} Nm**',
                f'- 候选模型文件：`{stage_dir / "analysis" / report["candidate_urdf"]}`',
                '',
                '## 逐轴留出验证 RMSE（Nm）',
                '',
                '| SHM ID | 关节 | 名义模型 | 拟合候选 | 反向半差最大 |',
                '| ---: | --- | ---: | ---: | ---: |',
            ])
            nom_vec = np.asarray(report['nominal_validation_rmse_nm'], dtype=float)
            cal_vec = np.asarray(report['validation_rmse_nm'], dtype=float)
            spread_vec = np.asarray(report['paired_approach_half_spread_nm'], dtype=float)
            for j in range(14):
                label = ('左' if j < 7 else '右') + PARTS[j % 7]
                lines.append(f'| {j + 12} | {label} (`{NAMES[j]}`) | {nom_vec[j]:.4f} | {cal_vec[j]:.4f} | {spread_vec[j]:.4f} |')
        elif m_stage_has_trials := bool(plan_meta['trials']):
            if m_stage_has_trials:
                lines.append(f'- PSD 重力拟合：未生成（{report.get("analysis_unavailable_reason", "无")}）')
        elif report.get('tracking_rmse_rad') is not None:
            rmse_deg = np.rad2deg(np.asarray(report['tracking_rmse_rad'], dtype=float))
            lines.append(f'- 冒烟跟踪最大轴 RMSE：{float(np.max(rmse_deg)):.3f}°')

        path = stage_dir / 'report.md'
        path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
        return path
