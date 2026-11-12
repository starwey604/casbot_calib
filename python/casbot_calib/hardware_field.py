"""Real SDK position acquisition and exploratory static gravity commissioning.

The shared C++ executor owns the command loop. This module plans from measured
start, seals inputs, records raw feedback and fits via the existing PSD solver.
Publication/receive times never become acquisition timestamps or S3 evidence.
"""
import csv
import json
import os
from pathlib import Path
import numpy as np
import yaml
from .collision import certify_polyline
from .data import write_json
from .model_runtime import model_for, collision_for, urdf_for, check_same_context, context_for
from .staged_motion import Motion, gate_motion
from .static_field import _move as add_bounded_move
from .run_marker import run_marker
from .vendor_identity import sha256
from .vendor_position import executor_binary


class FieldContractError(ValueError):
    """Changed sealed inputs, execution mismatch or failed geometric proof."""


def load_profile(path, *, manual=False):
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
    if not manual:
        for key in ('initial_hold_s', 'final_hold_s', 'anchor_transition_s', 'max_feedback_age_ns', 'max_mode_age_ns',
                    'transition_timeout_ns', 'start_tolerance_rad',
                    'maximum_velocity_rad_s', 'vel_scale'):
            if not np.isfinite(config[key]) or config[key] <= 0:
                raise ValueError('Positive finite field limit required: ' + key)
        if config['vel_scale'] > 1:
            raise ValueError('Invalid field speed scale')
        if config.get('gravity_feedforward','none') not in ('none','nominal'):raise ValueError('Explicit nominal/none bootstrap gravity policy required')
        for key in ('effort_gain', 'effort_zero_nm'):
            value = np.broadcast_to(np.asarray(config[key], dtype=float), (14,))
            if not np.isfinite(value).all() or (key == 'effort_gain' and np.any(value == 0)):
                raise ValueError('Finite nonzero effort scale / finite zero required')
        gravity = np.asarray(config['gravity_world'], dtype=float)
        if gravity.shape != (3,) or not np.isfinite(gravity).all() or np.linalg.norm(gravity) <= 0:
            raise ValueError('Finite nonzero installation gravity vector required')
    config['scene'] = yaml.safe_load((path.parent / config['collision_scene']).read_text(encoding='utf-8'))
    collision_for(config)
    config['model_context']=context_for(config).identity
    if 'connection' in config:
        from .sdk_connection import validate_connection
        validate_connection(config['connection'])
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
    if actual != set(entries): raise FieldContractError('Field manifest coverage changed')
    for name, digest in entries.items():
        path = (root / name).resolve()
        if not path.is_relative_to(root.resolve()) or sha256(path) != digest:
            raise FieldContractError('Frozen field input changed: ' + name)


def params(config, names, *, manual=False):
    if manual:
        return dict(execution_scope=config['execution_scope'],joint_names=names,
            **{k:config[k] for k in ('joint_topic','feedback_type','mode_topic','command_topic','mode_service','stop_service',
                'max_feedback_age_ns','max_mode_age_ns','transition_timeout_ns','maximum_velocity_rad_s')},
            command_period_ns=1_000_000_000//config['command_hz'],vel_scale=config.get('vel_scale',.1))
    experiment={}
    if config.get('gravity_feedforward')=='nominal':
        experiment=dict(gravity_feedforward_enabled=True,
            gravity_model_path=str(urdf_for(config)),
            gravity_world=config['gravity_world'])
    extra=config.get('breakaway_parameters',{})
    allowed=('gravity_feedforward_enabled','gravity_model_path','gravity_world','command_kp','command_kd','feedforward_plan_file','feedforward_limit_nm')
    if any(not name.startswith('breakaway_') and name not in allowed for name in extra):raise ValueError('Only explicit SDK experiment parameters may extend the executor')
    experiment.update(extra)
    return dict(execution_scope=config['execution_scope'], joint_names=names,
        **{k: config[k] for k in ('joint_topic', 'feedback_type', 'mode_topic', 'command_topic', 'mode_service', 'stop_service',
            'max_feedback_age_ns', 'max_mode_age_ns', 'transition_timeout_ns', 'start_tolerance_rad',
            'maximum_velocity_rad_s', 'vel_scale')},
        command_period_ns=1_000_000_000 // config['command_hz'], **experiment)


def print_field_limit(line):
    if not line.startswith('FIELDWORK_LIMIT '): return
    from .fieldwork_status import joint_label
    value = json.loads(line[len('FIELDWORK_LIMIT '):])
    if value['action'] == 'recovered':
        print('反馈已回到观察范围；发送继续。', flush=True); return
    field = dict(position_lower='位置下限',position_upper='位置上限',velocity='速度绝对值')[value['field']]
    action = '仅记录，继续发送' if value['action'] == 'warn' else '已停止发送'
    print(f"{joint_label(value['joint'])}：{field}超限，反馈 {value['value']:.6f} {value['unit']}，阈值 {value['threshold']:.6f} {value['unit']}；{action}。", flush=True)


def run_executor(config, output, domain_id, *, snapshot=False, plan=None, timeout_s=30, local_remaps=None,cancel_check=None, manual_command=None, manual_effort_limit_nm=10., manual_policy=None, on_output=None, recording_pause_file=None):
    if not 0 <= domain_id <= 232:
        raise ValueError('Explicit ROS domain 0..232 required')
    from .sdk_connection import connection_remaps
    local_remaps = connection_remaps(config, domain_id, local_remaps)
    engine, _ = model_for(config); binary, identity = executor_binary()
    if manual_command is not None:
        if snapshot or plan is not None:
            raise ValueError('Manual stream cannot consume a frozen trajectory or snapshot mode')
        config = dict(config, gravity_feedforward='none', breakaway_parameters={})
    elif on_output is None:
        on_output = print_field_limit
    values = params(config, [j.name for j in engine.model.joints],**(dict(manual=True) if manual_policy else {})); values['snapshot_only'] = snapshot
    root = Path(output).resolve(); root.mkdir(parents=True, exist_ok=False)
    values['output'] = str(root / 'execution')
    if not snapshot:
        values.update(lower=[j.lower for j in engine.model.joints], upper=[j.upper for j in engine.model.joints])
        if manual_command is None:
            values['plan_file'] = str(Path(plan).resolve())
        else:
            values.update(manual_stream=True, manual_command_file=str(Path(manual_command).resolve()),
                          manual_effort_limit_nm=manual_effort_limit_nm)
            if manual_policy:values['manual_policy_file']=str(Path(manual_policy).resolve())
    if recording_pause_file:
        if manual_command is None:raise ValueError('Recording pause is exclusive to the manual workspace')
        values['recording_pause_file']=str(recording_pause_file)
    from .sdk_process import ros_arguments,recording_process,run_process
    if local_remaps and any(not target.startswith('/casbot_calib_s2_') for target in local_remaps.values()):
        raise ValueError('Integration remaps require isolated private endpoints')
    command=ros_arguments(binary,values,local_remaps)
    write_json(root / 'request.json', dict(config=config, domain_id=domain_id, snapshot_only=snapshot,
        executor_sha256=sha256(binary), sdk=identity, local_contract_remaps=local_remaps,
        source_plan_csv_sha256=sha256(plan) if plan is not None else None,
        **(dict(manual_stream=True, manual_command_file=str(manual_command), hidden_compensation=False) if manual_command is not None else {}),
        execution_target='isolated_contract_test' if local_remaps else 'hardware',
        physical_semantics_verified=False, disconnect_protection_verified=False))
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain_id), ROS_LOCALHOST_ONLY='1' if local_remaps else '0')
    observation = yaml.safe_load(Path('configs/vendor_s1.yaml').read_text())
    if 'model_context' in config:
        observation.update(model_id=engine.model.model_id,joint_names=[j.name for j in engine.model.joints],model_context=config['model_context'])
    observation.update(joint_topic=(local_remaps or {}).get(config['joint_topic'], config['joint_topic']),
        joint_type=config['feedback_type'], mode_topic=(local_remaps or {}).get(config['mode_topic'], config['mode_topic']),
        robot_state_topic='', debug_topic='/motion/debug/joint_state' if manual_command is not None else '')
    if manual_command is not None and local_remaps:
        observation['debug_topic'] = local_remaps['/motion/debug/joint_state']
    observation_path = root / 'observation.yaml'; observation_path.write_text(yaml.safe_dump(observation))
    import sys
    recorder_command=[sys.executable,'-m','casbot_calib.cli','vendor-capture','--config',str(observation_path),
        '--output',str(root/'raw_state'),'--duration',str(timeout_s+5 if timeout_s is not None else 10),'--domain-id',str(domain_id),
        '--scope','localhost_contract_mock' if local_remaps else 'hardware_read_only']
    if manual_command is not None:recorder_command+=['--recording-flush-period-ms','20']
    if timeout_s is None:recorder_command+=['--until-stopped']
    if recording_pause_file:recorder_command+=['--recording-pause-file',str(recording_pause_file)]
    with recording_process(recorder_command,environment,root/'recorder.log',root/'raw_state/raw/recorder.json',**(dict(cancel_check=cancel_check) if cancel_check else {}),
            **(dict(ready_timeout_s=config.get('manual_recording_start_timeout_s',10.)) if manual_policy else {})) as recorder:
        process_result=run_process(command,environment,root/'process.log',timeout_s=timeout_s,recorder=recorder,cancel_check=cancel_check,**(dict(on_output=on_output) if on_output else {}),
            **(dict(recording_pause_file=recording_pause_file) if recording_pause_file else {}),
            **(dict(stop_timeout_s=config.get('manual_stop_timeout_s',3.)+5) if manual_policy else {}))
    code=process_result.returncode;interruption=process_result.interruption
    path = root / 'execution/result.json'
    result = json.loads(path.read_text()) if path.exists() else dict(passed=False, reason='executor_did_not_finish' if process_result.started else 'executor_not_started',
        **(dict(fault='interrupted') if not process_result.started else {}))
    if interruption is not None: result['passed'] = False
    result.update(returncode=code, snapshot_only=snapshot, local_contract_test=bool(local_remaps), executor_started=process_result.started,
        physical_semantics_verified=False, supervisor_interruption=interruption,
        source_config_assumptions=config['assumptions'], **run_marker())
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


def snapshot_binding(snapshot, config=None):
    root = Path(snapshot); request = root / 'request.json'
    source = json.loads(request.read_text()) if request.exists() else {}
    observed_connection = source.get('config', {}).get('connection')
    if config is not None:check_same_context(source.get('config',{}),config)
    if config is not None and observed_connection != config.get('connection'):
        raise ValueError('Snapshot and plan use different SDK connections')
    return dict(snapshot_manifest_sha256=sha256(root / 'manifest.json'),
        source_domain_id=source.get('domain_id'))


def prepare_field(profile, static_plan, snapshot, output, *, max_poses_per_split=None, candidate_ids=None):
    config = load_profile(profile); static_plan = Path(static_plan).resolve(); snapshot = Path(snapshot).resolve()
    check_seal(static_plan); start = _read_snapshot(snapshot)
    binding = snapshot_binding(snapshot, config)
    source_report = json.loads((static_plan / 'report.json').read_text())
    if not source_report['passed']:
        raise ValueError('Accepted static reference preset required')
    engine, source = model_for(config); names = [j.name for j in engine.model.joints]
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
    if candidate_ids is not None:
        if not candidate_ids or len(set(candidate_ids)) != len(candidate_ids) or not set(candidate_ids) <= set(selected):
            raise ValueError('Explicit distinct selected static candidates required')
        selected = {key: selected[key] for key in candidate_ids}
    root = output_outside(output, static_plan, snapshot); root.mkdir(parents=True, exist_ok=False)
    motions = [dict(json.loads((static_plan / 'plans' / name / 'motion.json').read_text()), trial_name=name)
        for name in source_report['planned_execution_order']]
    motions = [m for m in motions if m['conditions']['candidate_id'] in selected]
    selection = dict(pose_cap_per_arm_split=max_poses_per_split, selected_candidate_ids=list(selected),
        preset_manifest_sha256=sha256(static_plan / 'manifest.json'))
    recipe = dict(kind='static', home_rad=preset['home_rad'], planning_envelope=preset['planning_envelope'],
        motions=motions, selection=selection)
    motion, trials = _static_motion(config, start, recipe)
    return finish_plan(root, config, motion, trials, dict(**selection, **binding, replan_recipe=recipe))


def _static_motion(config, start, recipe):
    """Rebuild the same selected static targets from a newly measured start."""
    motion = Motion('hardware_static_sequence', 'gravity', start)
    motion.add(config['initial_hold_s'], start, label='observed_start_hold')
    add_bounded_move(motion, recipe['home_rad'], config['anchor_transition_s'], recipe['planning_envelope'], 'to_preset_anchor')
    trials = []
    for metadata in recipe['motions']:
        name = metadata['trial_name']
        begin = motion.duration; knots = metadata['knots_s']; labels = metadata['labels']; measure = None
        for i, label in enumerate(labels):
            state = np.asarray(metadata['states'][i + 1])
            if np.max(abs(state[1:])) > 1e-12: raise ValueError('Static field expects rest-to-rest preset segments')
            a = motion.duration
            motion.add(knots[i + 1] - knots[i], state[0], label=label)
            if label == 'static_measure': measure = (a, motion.duration)
        if measure is None: raise ValueError('Missing static measurement window')
        trials.append(dict(name=name, conditions=metadata['conditions'], begin_s=begin, end_s=motion.duration,
            measure_begin_s=measure[0], measure_end_s=measure[1]))
    add_bounded_move(motion, start, config['anchor_transition_s'], recipe['planning_envelope'], 'return_observed_start')
    motion.add(config['final_hold_s'], start, label='observed_start_final_hold')
    return motion, trials


def prepare_jog(profile, snapshot, output, joint_name, amplitude_deg=1., *, anchor_rad=None):
    return _prepare_jog(load_profile(profile), snapshot, output, joint_name, amplitude_deg, anchor_rad=anchor_rad)


def _prepare_jog(config, snapshot, output, joint_name, amplitude_deg, *, anchor_rad=None, bindings=None):
    start = _read_snapshot(snapshot); engine, _ = model_for(config)
    binding = snapshot_binding(snapshot, config)
    names = [j.name for j in engine.model.joints]
    if joint_name not in names or not np.isfinite(amplitude_deg) or not 0 < amplitude_deg <= 5:
        raise ValueError('Named arm joint and commissioning amplitude in (0,5] degrees required')
    root = output_outside(output, snapshot); root.mkdir(parents=True, exist_ok=False)
    motion = Motion('hardware_direction_jog', 'position', start, dict(joint=names.index(joint_name), amplitude_deg=amplitude_deg))
    motion.add(config['initial_hold_s'], start, label='observed_start_hold')
    envelope = dict(speed_rad_s=.05, acceleration_rad_s2=.1, jerk_rad_s3=1.)
    anchor = start if anchor_rad is None else np.asarray(anchor_rad, dtype=float)
    if anchor.shape != (14,) or not np.isfinite(anchor).all(): raise ValueError('Finite 14-axis comparison anchor required')
    if anchor_rad is not None:
        add_bounded_move(motion, anchor, config['anchor_transition_s'], envelope, 'to_comparison_anchor')
    comparison_begin = motion.duration
    comparison = Motion('frequency_reference', 'position', anchor)
    for sign in (1, -1, 0):
        q = anchor.copy(); q[names.index(joint_name)] += sign * np.deg2rad(amplitude_deg)
        add_bounded_move(motion, q, config['anchor_transition_s'], envelope, 'direction_jog')
        motion.add(1., q, label='direction_hold')
        add_bounded_move(comparison, q, config['anchor_transition_s'], envelope, 'direction_jog')
        comparison.add(1., q, label='direction_hold')
    comparison_end = motion.duration
    if anchor_rad is not None:
        add_bounded_move(motion, start, config['anchor_transition_s'], envelope, 'return_measured_start')
    motion.add(config['final_hold_s'], start, label='observed_start_final_hold')
    import hashlib
    definition = dict(joint=joint_name, amplitude_deg=amplitude_deg, anchor_rad=anchor.tolist(),
        knots_s=comparison.knots, states=np.asarray(comparison.states).tolist(), labels=comparison.labels)
    motion.conditions.update(comparison_window_s=[comparison_begin, comparison_end],
        comparison_reference_sha256=hashlib.sha256(json.dumps(definition, sort_keys=True).encode()).hexdigest(),
        comparison_anchor_rad=anchor.tolist())
    recipe = dict(kind='jog', joint=joint_name, amplitude_deg=amplitude_deg,
        anchor_rad=None if anchor_rad is None else anchor.tolist())
    return finish_plan(root, config, motion, [], dict(**binding, **(bindings or {}), replan_recipe=recipe))


def replan_field(plan, snapshot, output):
    """Use sealed H1 recipes, not changed source YAML or an uncertified q offset."""
    root = Path(plan).resolve(); check_seal(root)
    metadata = json.loads((root / 'plan.json').read_text()); recipe = metadata['replan_recipe']
    config = metadata['config']; binding = snapshot_binding(snapshot, config)
    if metadata.get('source_domain_id') is not None and binding['source_domain_id'] != metadata['source_domain_id']:
        raise ValueError('Start replan must use the original ROS domain')
    binary, sdk = executor_binary()
    if sha256(binary) != metadata['executor_sha256'] or sdk != metadata['sdk_identity']:
        raise ValueError('Executor/SDK changed during start replan; create a new plan')
    binding['replanned_from_manifest_sha256'] = sha256(root / 'manifest.json')
    output_outside(output, root, snapshot)
    if recipe['kind'] == 'jog':
        return _prepare_jog(config, snapshot, output, recipe['joint'], recipe['amplitude_deg'],
            anchor_rad=recipe['anchor_rad'], bindings={'replanned_from_manifest_sha256': binding['replanned_from_manifest_sha256']})
    if recipe['kind'] == 'zero':
        from .fieldwork_zero import prepare_zero
        return prepare_zero(config, snapshot, output, recipe['target_rad'], recipe['duration_s'], bindings=binding)
    if recipe['kind'] == 'sdk_wrist_probe':
        from .hardware_probe_capture import replan_probe
        return replan_probe(config,snapshot,output,recipe,binding)
    if recipe['kind'] != 'static': raise ValueError('Unsupported field start-replan recipe')
    start = _read_snapshot(snapshot); destination = Path(output).resolve(); destination.mkdir(parents=True, exist_ok=False)
    motion, trials = _static_motion(config, start, recipe)
    return finish_plan(destination, config, motion, trials,
        dict(**recipe['selection'], **binding, replan_recipe=recipe))


def finish_plan(root, config, motion, trials, bindings):
    engine, source = model_for(config); names = [j.name for j in engine.model.joints]
    profile_collision = collision_for(config,engine)
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
        collision_scope='nominal reference and sampled-command interpolant; no runtime tracking-error gate or certified stop/tracking tube',
        source_assumptions=config['assumptions'])
    write_json(root / 'plan.json', report); seal(root)
    return report


def execute_field(plan, output, domain_id, *, local_remaps=None,cancel_check=None,replan_start=True):
    root = Path(plan).resolve(); check_seal(root); metadata = json.loads((root / 'plan.json').read_text())
    output_outside(output, root)
    if metadata.get('source_domain_id') is not None and metadata['source_domain_id'] != domain_id:
        raise ValueError('Execute on the same ROS domain as the measured snapshot')
    binary, sdk = executor_binary()
    if sha256(binary) != metadata['executor_sha256'] or sdk != metadata['sdk_identity']:
        raise ValueError('Replan after executor/SDK changes')
    engine, source = model_for(metadata['config'])
    if source['sha256'] != metadata['source_model_sha256'] or metadata['joint_names'] != [j.name for j in engine.model.joints]:
        raise ValueError('Field model changed')
    result = run_executor(metadata['config'], output, domain_id, plan=root / 'position_plan.csv',
        timeout_s=metadata['duration_s'] + 45, local_remaps=local_remaps,cancel_check=cancel_check)
    if replan_start and start_replan_required(result):
        if 'replan_recipe' not in metadata:
            print('起点已变化，尚未申请模式或发送指令；旧计划没有重规划配方，请重新快照/规划。', flush=True)
            return result
        return _execute_start_replans(root, Path(output).resolve(), domain_id, result,
            local_remaps=local_remaps, cancel_check=cancel_check)
    return result


def start_replan_required(result):
    """Only a proven pre-control rejection is eligible; never recover SDK faults."""
    return (not result.get('passed') and result.get('requires_start_replan') is True
        and result.get('reason') == 'start_pose_changed' and result.get('mode_requested') is False
        and result.get('published_commands') == 0 and not result.get('fault')
        and not result.get('supervisor_interruption'))


def announce_start_replan(result):
    error = result.get('start_pose_error_rad', [])
    detail = f'；最大偏差 {max(abs(x) for x in error):.6f} rad' if error else ''
    print('起点已变化'+detail+'；尚未申请模式或发送指令，正在重新读取并规划当前单元。Ctrl+C可取消。', flush=True)


def _execute_start_replans(original, output, domain_id, rejected, *, local_remaps, cancel_check):
    # The ordinary capture layout is unchanged unless its start is rejected.
    # Move the completed rejected bundle intact into a collection, never edit it.
    archive = output / 'start_replans/attempt_000/capture'; archive.mkdir(parents=True)
    for path in list(output.iterdir()):
        if path.name != 'start_replans': path.rename(archive / path.name)
    attempts = [dict(plan=str(original), capture=str(archive.relative_to(output)), result=rejected)]
    current = original; result = rejected; effective = {}
    try:
        while start_replan_required(result):
            announce_start_replan(result)
            folder = output / 'start_replans' / f'attempt_{len(attempts):03d}'; folder.mkdir()
            metadata = json.loads((current / 'plan.json').read_text())
            result = run_executor(metadata['config'], folder / 'snapshot', domain_id,
                snapshot=True, local_remaps=local_remaps, cancel_check=cancel_check)
            attempt = dict(snapshot=str((folder / 'snapshot').relative_to(output)))
            attempts.append(attempt)
            if not result['passed']:
                attempt['result'] = result; break
            replan_field(current, folder / 'snapshot', folder / 'plan')
            current = folder / 'plan'
            result = execute_field(current, folder / 'capture', domain_id, local_remaps=local_remaps,
                cancel_check=cancel_check, replan_start=False)
            effective = dict(effective_plan=str(current.relative_to(output)),
                effective_capture=str((folder / 'capture').relative_to(output)))
            attempt.update(plan=effective['effective_plan'], capture=effective['effective_capture'], result=result)
    except (Exception, KeyboardInterrupt) as exc:
        result = dict(passed=False, reason='start_replan_interrupted' if isinstance(exc, KeyboardInterrupt) else 'start_replan_failed', error=str(exc))
        raise
    finally:
        write_json(output / 'start_replans.json', dict(attempts=attempts))
        result = dict(result, schema_version='casbot_field_start_replans_v1', snapshot_only=False,
            original_plan_manifest_sha256=sha256(original / 'manifest.json'), **effective)
        write_json(output / 'result.json', result); seal(output)
    return result


def resolve_field_capture(plan, capture):
    """Keep hardware-analyse's original plan/capture entry points after a replan."""
    plan, capture = Path(plan).resolve(), Path(capture).resolve()
    result = json.loads((capture / 'result.json').read_text())
    if result.get('schema_version') != 'casbot_field_start_replans_v1': return plan, capture
    check_seal(plan); check_seal(capture)
    if sha256(plan / 'manifest.json') != result['original_plan_manifest_sha256']:
        raise ValueError('Start-replan collection belongs to a different original plan')
    if not result['passed']: raise ValueError('Completed field sequence required; start-replan collection did not finish')
    paths = [(capture / result[key]).resolve() for key in ('effective_plan', 'effective_capture')]
    if any(not path.is_relative_to(capture) or path == capture for path in paths):
        raise ValueError('Start-replan artifacts must belong to this capture collection')
    return tuple(paths)


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


def static_means(metadata, obs, elapsed, config):
    means = []
    for trial in metadata['trials']:
        use = (elapsed >= trial['measure_begin_s']) & (elapsed < trial['measure_end_s'])
        count = int(use.sum())
        if not count:
            means.append(dict(name=trial['name'],conditions=trial['conditions'],q=None,torque=None,
                received_publications=0,available=False,sample_time_known=False))
            continue
        times = elapsed[use]; q = obs['q'][use]; v = obs['dq'][use]
        torque = (obs['effort_raw'][use] - config['effort_zero_nm']) * config['effort_gain']
        means.append(dict(name=trial['name'], conditions=trial['conditions'], q=q.mean(axis=0),
            torque=torque.mean(axis=0), received_publications=count, available=True,
            observed_duration_s=float(times[-1]-times[0]),maximum_received_speed_rad_s=float(np.max(abs(v))),
            position_span_rad=np.ptp(q,axis=0),maximum_callback_gap_s=float(np.max(np.diff(times))) if count>1 else None, independent_acquisition_count=None,
            sample_time_known=False, time_basis='local callback windows relative to executed plan; not acquisition time'))
    return means


def field_observations(plan, capture):
    """Shared immutable SDK capture checks; time remains callback availability."""
    root, capture = resolve_field_capture(plan, capture); check_seal(root); check_seal(capture)
    metadata = json.loads((root / 'plan.json').read_text()); result = json.loads((capture / 'result.json').read_text())
    if not result['passed']: raise ValueError('Completed field sequence required; failed captures remain diagnostic only')
    request = json.loads((capture / 'request.json').read_text())
    if request['config'] != metadata['config'] or request['executor_sha256'] != metadata['executor_sha256']:
        raise FieldContractError('Capture differs from frozen field plan')
    if request['source_plan_csv_sha256'] != sha256(root / 'position_plan.csv'):
        raise FieldContractError('Capture executed a different position plan')
    with (capture / 'execution/commands.csv').open() as stream: commands = list(csv.DictReader(stream))
    if not commands: raise ValueError('Missing executed command history')
    origin = int(commands[0]['planned_ns']) - int(commands[0]['planned_index']) * metadata['command_period_ns']
    if any(int(r['planned_ns']) - int(r['planned_index']) * metadata['command_period_ns'] != origin for r in commands):
        raise FieldContractError('Field command clock origin changed')
    obs = read_feedback(capture / 'execution/feedback.csv', require_effort=metadata.get('stage') != 'position'); config = metadata['config']
    elapsed = (obs['receive_ns'] - np.int64(origin)) * 1e-9
    engine, _ = model_for(config)
    model = engine.model; model.gravity_world = np.asarray(config.get('gravity_world', model.gravity_world)); engine = type(engine)(model)
    actual_collision = certify_polyline(collision_for(config,engine), obs['q'],
        (obs['receive_ns'] - obs['receive_ns'][0]) * 1e-9, clearance_m=config['scene']['clearance_m'])
    if not actual_collision['passed']: raise FieldContractError('Received field path interpolant failed collision gate')
    return metadata,commands,origin,obs,elapsed,actual_collision


def analyse_field(plan, capture, output, *, windows_only=False):
    root = Path(plan).resolve(); capture = Path(capture).resolve(); output_outside(output,root,capture)
    root, capture = resolve_field_capture(root, capture)
    metadata,commands,origin,obs,elapsed,actual_collision=field_observations(root,capture)
    config=metadata['config']
    if metadata.get('stage') == 'position':
        from .vendor_position_analysis import rates
        from scipy.interpolate import BPoly
        from scipy.optimize import minimize_scalar
        with np.load(root / 'reference.npz', allow_pickle=False) as data: curve = BPoly(data['coefficients'], data['breakpoints'])
        conditions = json.loads((root / 'motion.json').read_text())['conditions']
        begin, end = conditions.get('comparison_window_s', [0., float(curve.x[-1])])
        active = (elapsed >= begin) & (elapsed <= end); t = elapsed[active]; q = obs['q'][active]
        error = q - curve(t); moving = np.ptp(curve(t), axis=0) > .001 if len(t) else np.zeros(14, dtype=bool)
        reference_span = np.ptp(curve(t), axis=0) if len(t) else np.zeros(14)
        observed_span = np.ptp(q, axis=0) if len(t) else np.zeros(14)
        lag = None; boundary = None
        if len(t) > 20 and np.any(moving):
            objective = lambda delta: float(np.mean((q[:, moving] - curve(np.clip(t - delta, 0, curve.x[-1]))[:, moving]) ** 2))
            fitted = minimize_scalar(objective, bounds=(0., .15), method='bounded', options={'xatol': 1e-6})
            lag = float(fitted.x * 1000); boundary = bool(fitted.x < .0001 or fitted.x > .1499)
        output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
        report = dict(passed=True, analysis_available=True, stage='position', source_assumptions=config['assumptions'],
            commanded_span_rad=reference_span,
            comparison_window_s=[begin, end], comparison_reference_sha256=conditions.get('comparison_reference_sha256'),
            comparison_feedback_receive_frequency=rates(obs['receive_ns'][active]),
            comparison_command_publish_frequency=rates(np.array([int(r['publish_ns']) for r in commands
                if begin <= int(r['planned_index']) * metadata['command_period_ns'] * 1e-9 <= end], dtype=np.int64)),
            command_publish_frequency=rates(np.array([int(r['publish_ns']) for r in commands], dtype=np.int64)),
            feedback_receive_frequency=rates(obs['receive_ns']), observed_span_rad=observed_span,
            tracking_rmse_rad=np.sqrt(np.mean(error ** 2, axis=0)) if len(t) else None,
            aggregate_response_lag_ms=lag, response_lag_boundary_limited=boundary,
            delay_scope='reference-to-callback trajectory shift; includes interpolation/cache/mechanics; not one-way network delay',
            physical_direction_verified=False, mechanical_zero_calibrated=False, actual_sample_rate_known=False,
            source_plan_manifest_sha256=sha256(root / 'manifest.json'), source_capture_manifest_sha256=sha256(capture / 'manifest.json'))
        write_json(output / 'report.json', report); write_json(output / 'collision_received_interpolant.json', actual_collision); seal(output)
        return report
    means = static_means(metadata, obs, elapsed, config)
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output / 'received_observations.npz', **obs)
    write_json(output / 'static_windows.json', means)
    write_json(output / 'collision_received_interpolant.json', actual_collision)
    bindings = dict(source_plan_manifest_sha256=sha256(root / 'manifest.json'), source_capture_manifest_sha256=sha256(capture / 'manifest.json'))
    if windows_only:
        report = dict(passed=True, stage='gravity_windows', windows=len(means), sample_time_known=False,
            hardware_calibration_accepted=False, **bindings)
    else:
        report = write_gravity_candidate(means, config, output, metadata)
        from .vendor_position_analysis import rates
        report.update(command_publish_frequency=rates(np.array([int(r['publish_ns']) for r in commands], dtype=np.int64)),
            feedback_receive_frequency=rates(obs['receive_ns']), **bindings)
    write_json(output / 'report.json', report); seal(output)
    return report


def write_gravity_candidate(means, config, output, metadata):
    """Shared H1/H2 PSD fit and export; caller owns output creation/sealing."""
    from .rigid_identification import fit_rigid, model_from_parameters
    from .dynamic_export import write_inertial_urdf, check_reload
    output = Path(output); engine, _ = model_for(config)
    model = engine.model; model.gravity_world = np.asarray(config.get('gravity_world', model.gravity_world)); engine = type(engine)(model)
    batches = {'train': [], 'validation': []}; by_pose = {}; spreads = []
    for row in means:
        if row.get('q') is None or row.get('torque') is None:continue
        c = row['conditions']; by_pose.setdefault((c['batch'], c['pose']), []).append(row)
    for (split, pose), rows in by_pose.items():
        q = np.array([r['q'] for r in rows]); z = np.zeros_like(q)
        y = engine.regressor_batch(q, z, z)['regressor'].mean(axis=0)
        torque = np.mean([r['torque'] for r in rows], axis=0)
        spreads.append(np.ptp(np.array([r['torque'] for r in rows]),axis=0)/2)
        batches[split].append((y, torque, pose))
    if not all(batches.values()): raise ValueError('Disjoint training/validation pairs required')
    y = np.array([r[0] for r in batches['train']]); target = np.array([r[1] for r in batches['train']])
    fit = fit_rigid(y, target, engine.rigid_parameters(), robust_scale=.1)
    vy = np.array([r[0] for r in batches['validation']]); vt = np.array([r[1] for r in batches['validation']])
    prediction = vy @ np.asarray(fit['parameters']); rmse = np.sqrt(np.mean((prediction - vt) ** 2, axis=0))
    nominal_rmse = np.sqrt(np.mean((vy @ engine.rigid_parameters() - vt) ** 2, axis=0))
    candidate = model_from_parameters(engine, fit['parameters'])
    write_json(output / 'calibration.json', dict(schema_version='casbot_field_gravity_candidate_v1',
        scope='exploratory commissioning representative; not accepted S4 export', parameters=fit['parameters'],
        joint_names=metadata['joint_names'], gravity_world=np.asarray(engine.model.gravity_world),
        source_model_sha256=metadata['source_model_sha256'], effort_gain=config['effort_gain'], effort_zero_nm=config['effort_zero_nm'],
        source_assumptions=config['assumptions'], friction_identified=False, inertias_identified=False,
        sdk_compensation_modified=False, auto_deploy=False,
        **({'model_context':config['model_context']} if 'model_context' in config else {})))
    with (output / 'bodies.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream); writer.writerow(['joint', 'mass_kg', 'com_x_m', 'com_y_m', 'com_z_m', 'status'])
        writer.writerows([j.name, j.body.mass, *j.body.com, 'aggregate representative; not uniquely measured'] for j in candidate.model.joints)
    source_path = urdf_for(config)
    aggregation = write_inertial_urdf(source_path, candidate, output / 'candidate_exploratory.urdf')
    reload_report = check_reload(output / 'candidate_exploratory.urdf', candidate, source_path)
    write_json(output / 'urdf_reload.json', reload_report)
    report = dict(passed=bool(reload_report['passed']), fit_succeeded=True, stage='gravity', fit=fit,
        frequency_scope='local command publication and feedback callback; not motor acquisition or device adoption rate',
        validation_rmse_nm=rmse, nominal_validation_rmse_nm=nominal_rmse,
        paired_approach_half_spread_nm=np.max(spreads, axis=0),
        training_poses=[r[2] for r in batches['train']], validation_poses=[r[2] for r in batches['validation']],
        scope='exploratory field gravity under recorded assumptions; not qualified S3/S4/P0 acceptance',
        hardware_calibration_accepted=False, physical_stop_verified=False, sample_time_known=False,
        torque_scale_verified=False, source_assumptions=config['assumptions'], aggregation=aggregation,
        individual_body_parameters_unique=False, inertias_identified=False, auto_deploy=False)
    return report
