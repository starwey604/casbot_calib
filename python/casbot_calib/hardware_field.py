"""Real SDK position acquisition and exploratory static gravity commissioning.

The shared C++ executor owns the command loop. This module plans from measured
start, seals inputs, records raw feedback and fits via the existing PSD solver.
Publication/receive times never become acquisition timestamps or S3 evidence.
"""
import csv
from importlib.resources import files
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import yaml
from .collision import nominal_profile, certify_polyline
from .data import write_json
from .dual_model import nominal_dual_arm
from .staged_motion import Motion, gate_motion
from .static_field import _move as add_bounded_move
from .vendor_identity import sha256
from .vendor_position import executor_binary


def load_profile(path):
    path = Path(path).resolve(); config = yaml.safe_load(path.read_text(encoding='utf-8'))
    if config.get('schema_version') != 'casbot_hardware_field_v1' or config.get('execution_scope') != 'hardware_position_stream':
        raise ValueError('Explicit hardware field profile required')
    if type(config['command_hz']) is not int or config['command_hz'] not in (100, 250, 500):
        raise ValueError('Field command rates are 100/250/500 Hz')
    if config['command_topic'] != '/upper_body_debug/joint_cmd' or config['mode_service'] != '/motion/upper_body_debug' or config['stop_service']:
        raise ValueError('Use SDK upper-body endpoints; no presumed vendor stop service')
    if config['feedback_type'] not in ('sensor_msgs/msg/JointState', 'crb_ros_msg/msg/JointStateData'):
        raise ValueError('Unsupported field feedback type')
    if config['assumptions']['arm_subset'] != 'user_confirmed_14_arm_axes_leave_waist_head_hands_unchanged' or config['assumptions']['disconnect'] != 'user_assumed_robot_self_protection_when_stream_stops':
        raise ValueError('Record the agreed arm subset and assumed disconnect behavior')
    for key in ('initial_hold_s', 'final_hold_s', 'anchor_transition_s', 'max_feedback_age_ns', 'max_mode_age_ns',
                'transition_timeout_ns', 'start_tolerance_rad', 'tracking_limit_rad', 'position_allowance_rad',
                'maximum_velocity_rad_s', 'vel_scale', 'static_velocity_rad_s', 'static_position_range_rad',
                'static_max_gap_s', 'gravity_validation_rmse_nm'):
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise ValueError('Positive finite field limit required: ' + key)
    if config['vel_scale'] > 1 or config['position_allowance_rad'] < config['tracking_limit_rad']:
        raise ValueError('Invalid field speed scale or tracking envelope')
    for key in ('effort_gain', 'effort_zero_nm'):
        value = np.broadcast_to(np.asarray(config[key], dtype=float), (14,))
        if not np.isfinite(value).all() or (key == 'effort_gain' and np.any(value == 0)):
            raise ValueError('Finite nonzero effort scale / finite zero required')
    gravity = np.asarray(config['gravity_world'], dtype=float)
    if gravity.shape != (3,) or not np.isfinite(gravity).all() or np.linalg.norm(gravity) <= 0:
        raise ValueError('Finite nonzero installation gravity vector required')
    config['scene'] = yaml.safe_load((path.parent / config['collision_scene']).read_text(encoding='utf-8'))
    return config


def seal(root):
    root = Path(root)
    write_json(root / 'manifest.json', {str(p.relative_to(root)): sha256(p)
        for p in sorted(root.rglob('*')) if p.is_file() and p != root / 'manifest.json'})


def output_outside(output, *inputs):
    path = Path(output).resolve()
    if any(path == Path(root).resolve() or Path(root).resolve() in path.parents for root in inputs):
        raise ValueError('Field output must be outside frozen input bundles')
    return path


def check_seal(root):
    root = Path(root); manifest = json.loads((root / 'manifest.json').read_text())
    # Static preset manifests wrap the map; field bundles use the map directly.
    entries = manifest.get('files', manifest)
    actual = {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p != root / 'manifest.json'}
    if actual != set(entries): raise ValueError('Field manifest coverage changed')
    for name, digest in entries.items():
        path = (root / name).resolve()
        if not path.is_relative_to(root.resolve()) or sha256(path) != digest:
            raise ValueError('Frozen field input changed: ' + name)


def params(config, names):
    return dict(execution_scope=config['execution_scope'], joint_names=names,
        **{k: config[k] for k in ('joint_topic', 'feedback_type', 'mode_topic', 'command_topic', 'mode_service', 'stop_service',
            'max_feedback_age_ns', 'max_mode_age_ns', 'transition_timeout_ns', 'start_tolerance_rad',
            'tracking_limit_rad', 'position_allowance_rad', 'maximum_velocity_rad_s', 'vel_scale')},
        command_period_ns=1_000_000_000 // config['command_hz'])


def run_executor(config, output, domain_id, *, snapshot=False, plan=None, timeout_s=30, local_remaps=None):
    if not 0 <= domain_id <= 232:
        raise ValueError('Explicit ROS domain 0..232 required')
    engine, _ = nominal_dual_arm(); binary, identity = executor_binary()
    values = params(config, [j.name for j in engine.model.joints]); values['snapshot_only'] = snapshot
    root = Path(output).resolve(); root.mkdir(parents=True, exist_ok=False)
    values['output'] = str(root / 'execution')
    if not snapshot:
        values.update(plan_file=str(Path(plan).resolve()), lower=[j.lower for j in engine.model.joints], upper=[j.upper for j in engine.model.joints])
    command = [str(binary), '--ros-args']
    for key, value in values.items():
        command += ['-p', key + ':=' + ("''" if value == '' else json.dumps(value) if isinstance(value, (bool, list)) else str(value))]
    if local_remaps:
        if any(not target.startswith('/casbot_calib_s2_') for target in local_remaps.values()):
            raise ValueError('Integration remaps require isolated private endpoints')
        for first, second in local_remaps.items(): command += ['-r', first + ':=' + second]
    write_json(root / 'request.json', dict(config=config, domain_id=domain_id, snapshot_only=snapshot,
        executor_sha256=sha256(binary), sdk=identity, local_contract_remaps=local_remaps,
        source_plan_csv_sha256=sha256(plan) if plan is not None else None,
        execution_target='isolated_contract_test' if local_remaps else 'hardware',
        physical_semantics_verified=False, disconnect_protection_verified=False))
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain_id), ROS_LOCALHOST_ONLY='1' if local_remaps else '0')
    observation = yaml.safe_load(Path('configs/vendor_s1.yaml').read_text())
    observation.update(joint_topic=(local_remaps or {}).get(config['joint_topic'], config['joint_topic']),
        joint_type=config['feedback_type'], mode_topic=(local_remaps or {}).get(config['mode_topic'], config['mode_topic']),
        robot_state_topic='', debug_topic='')
    observation_path = root / 'observation.yaml'; observation_path.write_text(yaml.safe_dump(observation))
    import sys
    interruption = None
    with (root / 'recorder.log').open('x') as recorder_log, (root / 'process.log').open('x') as log:
        recorder = subprocess.Popen([sys.executable, '-m', 'casbot_calib.cli', 'vendor-capture', '--config', str(observation_path),
            '--output', str(root / 'raw_state'), '--duration', str(timeout_s + 5), '--domain-id', str(domain_id),
            '--scope', 'localhost_contract_mock' if local_remaps else 'hardware_read_only'], env=environment,
            stdout=recorder_log, stderr=subprocess.STDOUT)
        process = None
        try:
            deadline = time.monotonic() + 10
            while not (root / 'raw_state/raw/recorder.json').exists():
                if recorder.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Raw field recorder failed before any command process started')
                time.sleep(.02)
            process = subprocess.Popen(command, env=environment, stdout=log, stderr=subprocess.STDOUT)
            try:
                execution_deadline = time.monotonic() + timeout_s
                while process.poll() is None:
                    if recorder.poll() is not None:
                        interruption = 'raw_recorder_exited'; break
                    if time.monotonic() > execution_deadline:
                        interruption = 'execution_timeout'; break
                    time.sleep(.02)
            except KeyboardInterrupt:
                interruption = 'user_interrupt'
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: code = process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill(); code = process.wait()
            else: code = process.returncode
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=10)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
                try: recorder.wait(timeout=10)
                except subprocess.TimeoutExpired: recorder.kill(); recorder.wait()
    path = root / 'execution/result.json'
    result = json.loads(path.read_text()) if path.exists() else dict(passed=False, reason='executor_did_not_finish')
    if interruption is not None: result['passed'] = False
    result.update(returncode=code, snapshot_only=snapshot, local_contract_test=bool(local_remaps),
        physical_semantics_verified=False, supervisor_interruption=interruption, source_config_assumptions=config['assumptions'])
    write_json(root / 'result.json', result); seal(root)
    return result


def snapshot_field(profile, output, domain_id):
    return run_executor(load_profile(profile), output, domain_id, snapshot=True)


def _read_snapshot(root):
    check_seal(root); result = json.loads((Path(root) / 'result.json').read_text())
    if not result['passed'] or not result['snapshot_only']:
        raise ValueError('Successful read-only field snapshot required')
    with (Path(root) / 'execution/snapshot.csv').open() as stream: row = next(csv.DictReader(stream))
    q = np.array([float(row['q' + str(j)]) for j in range(14)])
    v = np.array([float(row['v' + str(j)]) for j in range(14)])
    if not np.isfinite(q).all() or not np.isfinite(v).all() or np.max(abs(v)) >= .02:
        raise ValueError('Complete stationary field snapshot required')
    return q


def snapshot_binding(snapshot):
    root = Path(snapshot); request = root / 'request.json'
    return dict(snapshot_manifest_sha256=sha256(root / 'manifest.json'),
        source_domain_id=json.loads(request.read_text())['domain_id'] if request.exists() else None)


def prepare_field(profile, static_plan, snapshot, output, *, max_poses_per_split=None):
    config = load_profile(profile); static_plan = Path(static_plan).resolve(); snapshot = Path(snapshot).resolve()
    check_seal(static_plan); start = _read_snapshot(snapshot)
    source_report = json.loads((static_plan / 'report.json').read_text())
    if not source_report['passed']:
        raise ValueError('Accepted static reference preset required')
    engine, source = nominal_dual_arm(); names = [j.name for j in engine.model.joints]
    if source_report['source_model_sha256'] != source['sha256'] or source_report['joint_names'] != names:
        raise ValueError('Static preset model identity mismatch')
    preset = json.loads((static_plan / 'config.json').read_text())
    if config['scene'] != json.loads((static_plan / 'scene.json').read_text()):
        raise ValueError('Field and static collision scenes differ')
    if max_poses_per_split is not None and (type(max_poses_per_split) is not int or max_poses_per_split < 1):
        raise ValueError('Positive commissioning pose cap required')
    candidates = json.loads((static_plan / 'candidates.json').read_text()); selected = {}
    for arm in ('left', 'right'):
        for split in ('train', 'validation'):
            rows = [r for r in candidates if r['status'] == 'selected' and r['arm'] == arm and r['split'] == split]
            if max_poses_per_split is not None: rows = rows[:max_poses_per_split]
            selected.update({r['id']: r for r in rows})
    root = output_outside(output, static_plan, snapshot); root.mkdir(parents=True, exist_ok=False)
    motion = Motion('hardware_static_sequence', 'gravity', start)
    motion.add(config['initial_hold_s'], start, label='observed_start_hold')
    add_bounded_move(motion, preset['home_rad'], config['anchor_transition_s'], preset['planning_envelope'], 'to_preset_anchor')
    trials = []
    for name in source_report['planned_execution_order']:
        metadata = json.loads((static_plan / 'plans' / name / 'motion.json').read_text())
        if metadata['conditions']['candidate_id'] not in selected: continue
        begin = motion.duration; knots = metadata['knots_s']; labels = metadata['labels']
        for i, label in enumerate(labels):
            state = np.asarray(metadata['states'][i + 1])
            if np.max(abs(state[1:])) > 1e-12: raise ValueError('Static field expects rest-to-rest preset segments')
            motion.add(knots[i + 1] - knots[i], state[0], label=label)
        index = labels.index('static_measure')
        trials.append(dict(name=name, conditions=metadata['conditions'], begin_s=begin, end_s=motion.duration,
            measure_begin_s=begin + knots[index], measure_end_s=begin + knots[index + 1]))
    add_bounded_move(motion, start, config['anchor_transition_s'], preset['planning_envelope'], 'return_observed_start')
    motion.add(config['final_hold_s'], start, label='observed_start_final_hold')
    return finish_plan(root, config, motion, trials, dict(pose_cap_per_arm_split=max_poses_per_split,
        **snapshot_binding(snapshot), preset_manifest_sha256=sha256(static_plan / 'manifest.json')))


def prepare_jog(profile, snapshot, output, joint_name, amplitude_deg=1.):
    config = load_profile(profile); start = _read_snapshot(snapshot); engine, _ = nominal_dual_arm()
    names = [j.name for j in engine.model.joints]
    if joint_name not in names or not np.isfinite(amplitude_deg) or not 0 < amplitude_deg <= 5:
        raise ValueError('Named arm joint and commissioning amplitude in (0,5] degrees required')
    root = output_outside(output, snapshot); root.mkdir(parents=True, exist_ok=False)
    motion = Motion('hardware_direction_jog', 'position', start, dict(joint=names.index(joint_name), amplitude_deg=amplitude_deg))
    motion.add(config['initial_hold_s'], start, label='observed_start_hold')
    envelope = dict(speed_rad_s=.05, acceleration_rad_s2=.1, jerk_rad_s3=1.)
    for sign in (1, -1, 0):
        q = start.copy(); q[names.index(joint_name)] += sign * np.deg2rad(amplitude_deg)
        add_bounded_move(motion, q, config['anchor_transition_s'], envelope, 'direction_jog')
        motion.add(1., q, label='direction_hold')
    motion.add(config['final_hold_s'], start, label='observed_start_final_hold')
    return finish_plan(root, config, motion, [], snapshot_binding(snapshot))


def finish_plan(root, config, motion, trials, bindings):
    engine, source = nominal_dual_arm(); names = [j.name for j in engine.model.joints]
    profile_collision = nominal_profile(engine, config['scene'])
    gate = gate_motion(profile_collision, motion, config['scene']['clearance_m'])
    write_json(root / 'collision_reference.json', gate)
    if not gate['passed']:
        write_json(root / 'failure.json', dict(passed=False, reason='measured_start_or_static_reference_collision', hardware_commands_sent=False))
        seal(root); raise ValueError('Field sequence rejected from measured start; no commands issued')
    curve = motion.polynomial(); period = 1_000_000_000 // config['command_hz']
    # Round up the final stationary hold to the requested command grid.
    last_ns = int(np.ceil(motion.duration * 1e9 / period)) * period
    times = np.arange(0, last_ns + 1, period, dtype=np.int64)
    clipped = np.minimum(times * 1e-9, motion.duration)
    q = curve(clipped); v = curve(clipped, nu=1)
    if np.max(abs(v)) > config['maximum_velocity_rad_s'] * (1 + 1e-9):
        raise ValueError('Field velocity bound exceeded')
    command_gate = certify_polyline(profile_collision, q, times * 1e-9, clearance_m=config['scene']['clearance_m'], block_samples=1000)
    write_json(root / 'collision_command_interpolant.json', command_gate)
    if not command_gate['passed']: raise ValueError('Field sampled-command interpolant rejected')
    np.savetxt(root / 'position_plan.csv', np.column_stack([times, q, v]), delimiter=',',
        header='relative_ns' + ''.join(',' + k + str(j) for k in ('q', 'v') for j in range(14)), comments='', fmt=['%d'] + ['%.17g'] * 28)
    np.savez_compressed(root / 'reference.npz', coefficients=curve.c, breakpoints=curve.x)
    write_json(root / 'motion.json', motion.metadata())
    binary, sdk = executor_binary()
    report = dict(schema_version='casbot_field_plan_v1', passed=True, config=config, joint_names=names,
        stage=motion.stage, source_model_sha256=source['sha256'], observed_start_rad=motion.home.tolist(), duration_s=last_ns * 1e-9,
        command_period_ns=period, trials=trials, **bindings,
        executor_sha256=sha256(binary), sdk_identity=sdk, hardware_command_path_implemented=True,
        physical_motion_qualified=False, tracking_tube_certified=False,
        collision_scope='nominal reference and sampled-command interpolant; tracking guard is runtime commissioning protection, not a certified stop/tracking tube',
        source_assumptions=config['assumptions'])
    write_json(root / 'plan.json', report); seal(root)
    return report


def execute_field(plan, output, domain_id, *, local_remaps=None):
    root = Path(plan).resolve(); check_seal(root); metadata = json.loads((root / 'plan.json').read_text())
    output_outside(output, root)
    if metadata.get('source_domain_id') is not None and metadata['source_domain_id'] != domain_id:
        raise ValueError('Execute on the same ROS domain as the measured snapshot')
    binary, sdk = executor_binary()
    if sha256(binary) != metadata['executor_sha256'] or sdk != metadata['sdk_identity']:
        raise ValueError('Replan after executor/SDK changes')
    engine, source = nominal_dual_arm()
    if source['sha256'] != metadata['source_model_sha256'] or metadata['joint_names'] != [j.name for j in engine.model.joints]:
        raise ValueError('Field model changed')
    return run_executor(metadata['config'], output, domain_id, plan=root / 'position_plan.csv',
        timeout_s=metadata['duration_s'] + 45, local_remaps=local_remaps)


def read_feedback(path, *, require_effort=True):
    with Path(path).open() as stream: rows = list(csv.DictReader(stream))
    if not rows: raise ValueError('No field feedback')
    receive = np.array([int(r['receive_ns']) for r in rows], dtype=np.int64)
    stamps = np.array([int(r['publish_stamp_ns']) for r in rows], dtype=np.int64)
    arrays = {key: np.array([[float(r[prefix + str(j)]) for j in range(14)] for r in rows])
        for key, prefix in [('q', 'q'), ('dq', 'v'), ('effort_raw', 'effort_raw')]}
    checked = ('q', 'dq', 'effort_raw') if require_effort else ('q', 'dq')
    if np.any(np.diff(receive) <= 0) or any(not np.isfinite(arrays[k]).all() for k in checked):
        raise ValueError('Field feedback has invalid fields or nonmonotonic callback times')
    return dict(receive_ns=receive, publish_stamp_ns=stamps, **arrays)


def analyse_field(plan, capture, output):
    from .rigid_identification import fit_rigid, model_from_parameters
    from .dynamic_export import write_inertial_urdf, check_reload
    root = Path(plan).resolve(); capture = Path(capture).resolve(); check_seal(root); check_seal(capture)
    output_outside(output, root, capture)
    metadata = json.loads((root / 'plan.json').read_text()); result = json.loads((capture / 'result.json').read_text())
    if not result['passed']: raise ValueError('Completed field sequence required; failed captures remain diagnostic only')
    request = json.loads((capture / 'request.json').read_text())
    if request['config'] != metadata['config'] or request['executor_sha256'] != metadata['executor_sha256']:
        raise ValueError('Capture differs from frozen field plan')
    if request['source_plan_csv_sha256'] != sha256(root / 'position_plan.csv'):
        raise ValueError('Capture executed a different position plan')
    with (capture / 'execution/commands.csv').open() as stream: commands = list(csv.DictReader(stream))
    if not commands: raise ValueError('Missing executed command history')
    origin = int(commands[0]['planned_ns']) - int(commands[0]['planned_index']) * metadata['command_period_ns']
    if any(int(r['planned_ns']) - int(r['planned_index']) * metadata['command_period_ns'] != origin for r in commands):
        raise ValueError('Field command clock origin changed')
    obs = read_feedback(capture / 'execution/feedback.csv', require_effort=metadata.get('stage') != 'position'); config = metadata['config']
    elapsed = (obs['receive_ns'] - np.int64(origin)) * 1e-9
    means = []; engine, _ = nominal_dual_arm()
    model = engine.model; model.gravity_world = np.asarray(config.get('gravity_world', model.gravity_world)); engine = type(engine)(model)
    actual_collision = certify_polyline(nominal_profile(engine, config['scene']), obs['q'],
        (obs['receive_ns'] - obs['receive_ns'][0]) * 1e-9, clearance_m=config['scene']['clearance_m'],
        max_gap_s=config['static_max_gap_s'])
    if not actual_collision['passed']: raise ValueError('Received field path interpolant failed collision gate')
    if metadata.get('stage') == 'position':
        from .vendor_position_analysis import rates
        from scipy.interpolate import BPoly
        from scipy.optimize import minimize_scalar
        with np.load(root / 'reference.npz', allow_pickle=False) as data: curve = BPoly(data['coefficients'], data['breakpoints'])
        active = (elapsed >= 0) & (elapsed <= curve.x[-1]); t = elapsed[active]; q = obs['q'][active]
        error = q - curve(t); moving = np.ptp(curve(t), axis=0) > .001 if len(t) else np.zeros(14, dtype=bool)
        reference_span = np.ptp(curve(t), axis=0) if len(t) else np.zeros(14)
        observed_span = np.ptp(q, axis=0) if len(t) else np.zeros(14)
        motion_observed = bool(np.any(moving) and np.all(observed_span[moving] >= .5 * reference_span[moving]))
        lag = None; boundary = None
        if len(t) > 20 and np.any(moving):
            objective = lambda delta: float(np.mean((q[:, moving] - curve(np.clip(t - delta, 0, curve.x[-1]))[:, moving]) ** 2))
            fitted = minimize_scalar(objective, bounds=(0., .15), method='bounded', options={'xatol': 1e-6})
            lag = float(fitted.x * 1000); boundary = bool(fitted.x < .0001 or fitted.x > .1499)
        output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
        report = dict(passed=motion_observed, stage='position', source_assumptions=config['assumptions'],
            motion_observed=motion_observed, commanded_span_rad=reference_span,
            command_publish_frequency=rates(np.array([int(r['publish_ns']) for r in commands], dtype=np.int64)),
            feedback_receive_frequency=rates(obs['receive_ns']), observed_span_rad=observed_span,
            tracking_rmse_rad=np.sqrt(np.mean(error ** 2, axis=0)) if len(t) else None,
            aggregate_response_lag_ms=lag, response_lag_boundary_limited=boundary,
            delay_scope='reference-to-callback trajectory shift; includes interpolation/cache/mechanics; not one-way network delay',
            physical_direction_verified=False, mechanical_zero_calibrated=False, actual_sample_rate_known=False,
            source_plan_manifest_sha256=sha256(root / 'manifest.json'), source_capture_manifest_sha256=sha256(capture / 'manifest.json'))
        write_json(output / 'report.json', report); write_json(output / 'collision_received_interpolant.json', actual_collision); seal(output)
        return report
    for trial in metadata['trials']:
        use = (elapsed >= trial['measure_begin_s']) & (elapsed < trial['measure_end_s'])
        count = int(use.sum())
        if count < 2: raise ValueError('No received static window: ' + trial['name'])
        times = elapsed[use]; q = obs['q'][use]; v = obs['dq'][use]
        if (times[-1] - times[0] < .8 * (trial['measure_end_s'] - trial['measure_begin_s']) or
            np.max(np.diff(times)) > config['static_max_gap_s'] or np.max(abs(v)) > config['static_velocity_rad_s'] or
            np.max(np.ptp(q, axis=0)) > config['static_position_range_rad']):
            raise ValueError('Unsettled/incomplete received static window: ' + trial['name'])
        torque = (obs['effort_raw'][use] - config['effort_zero_nm']) * config['effort_gain']
        means.append(dict(name=trial['name'], conditions=trial['conditions'], q=q.mean(axis=0),
            torque=torque.mean(axis=0), received_publications=count, independent_acquisition_count=None,
            sample_time_known=False, time_basis='local callback windows relative to executed plan; not acquisition time'))
    batches = {'train': [], 'validation': []}; by_pose = {}; spreads = []
    for row in means:
        c = row['conditions']; by_pose.setdefault((c['batch'], c['pose']), []).append(row)
    for (split, pose), rows in by_pose.items():
        if len(rows) != 2 or {r['conditions']['approach'] for r in rows} != {-1, 1}:
            raise ValueError('Complete opposite-approach pair required')
        q = np.array([r['q'] for r in rows]); z = np.zeros_like(q)
        y = engine.regressor_batch(q, z, z)['regressor'].mean(axis=0)
        torque = np.mean([r['torque'] for r in rows], axis=0)
        spreads.append(abs(rows[0]['torque'] - rows[1]['torque']) / 2)
        batches[split].append((y, torque, pose))
    if not all(batches.values()): raise ValueError('Disjoint training/validation pairs required')
    y = np.array([r[0] for r in batches['train']]); target = np.array([r[1] for r in batches['train']])
    fit = fit_rigid(y, target, engine.rigid_parameters(), robust_scale=.1)
    vy = np.array([r[0] for r in batches['validation']]); vt = np.array([r[1] for r in batches['validation']])
    prediction = vy @ np.asarray(fit['parameters']); rmse = np.sqrt(np.mean((prediction - vt) ** 2, axis=0))
    nominal_rmse = np.sqrt(np.mean((vy @ engine.rigid_parameters() - vt) ** 2, axis=0))
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output / 'received_observations.npz', **obs)
    write_json(output / 'static_windows.json', means)
    write_json(output / 'collision_received_interpolant.json', actual_collision)
    candidate = model_from_parameters(engine, fit['parameters'])
    write_json(output / 'calibration.json', dict(schema_version='casbot_field_gravity_candidate_v1',
        scope='exploratory commissioning representative; not accepted S4 export', parameters=fit['parameters'],
        joint_names=metadata['joint_names'], gravity_world=np.asarray(engine.model.gravity_world),
        source_model_sha256=metadata['source_model_sha256'], effort_gain=config['effort_gain'], effort_zero_nm=config['effort_zero_nm'],
        source_assumptions=config['assumptions'], friction_identified=False, inertias_identified=False,
        sdk_compensation_modified=False, auto_deploy=False))
    with (output / 'bodies.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream); writer.writerow(['joint', 'mass_kg', 'com_x_m', 'com_y_m', 'com_z_m', 'status'])
        writer.writerows([j.name, j.body.mass, *j.body.com, 'aggregate representative; not uniquely measured'] for j in candidate.model.joints)
    source_path = files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')
    aggregation = write_inertial_urdf(source_path, candidate, output / 'candidate_exploratory.urdf')
    reload_report = check_reload(output / 'candidate_exploratory.urdf', candidate, source_path)
    write_json(output / 'urdf_reload.json', reload_report)
    from .vendor_position_analysis import rates
    command_ns = np.array([int(r['publish_ns']) for r in commands], dtype=np.int64)
    report = dict(passed=bool(reload_report['passed']), fit_succeeded=True, stage='gravity', fit=fit,
        command_publish_frequency=rates(command_ns), feedback_receive_frequency=rates(obs['receive_ns']),
        frequency_scope='local command publication and feedback callback; not motor acquisition or device adoption rate',
        validation_rmse_nm=rmse, nominal_validation_rmse_nm=nominal_rmse,
        paired_approach_half_spread_nm=np.max(spreads, axis=0),
        commissioning_validation_passed=bool(np.max(rmse) <= config['gravity_validation_rmse_nm']),
        training_poses=[r[2] for r in batches['train']], validation_poses=[r[2] for r in batches['validation']],
        scope='exploratory field gravity under recorded assumptions; not qualified S3/S4/P0 acceptance',
        hardware_calibration_accepted=False, physical_stop_verified=False, sample_time_known=False,
        torque_scale_verified=False, source_assumptions=config['assumptions'], aggregation=aggregation,
        source_plan_manifest_sha256=sha256(root / 'manifest.json'), source_capture_manifest_sha256=sha256(capture / 'manifest.json'),
        individual_body_parameters_unique=False, inertias_identified=False, auto_deploy=False)
    write_json(output / 'report.json', report); seal(output)
    return report
