"""Manual five-field tuning sessions, recording and exports; no operator review forms."""
from pathlib import Path
import csv
import json
import time

import numpy as np
import yaml

from . import hardware_field as field
from .hardware_batch import atomic_json, event, events, read
from .run_marker import run_marker

FIELDS = ('position', 'velocity', 'effort', 'kp', 'kd')
DEFAULT_TUNING = dict(kp=20., kd=2., effort_limit_nm=10., kp_max=1000., kd_max=100.)


def tuning_config(config):
    result = dict(DEFAULT_TUNING, **config.get('manual_tuning', {}))
    if set(result) != set(DEFAULT_TUNING) or not all(np.isfinite(x) for x in result.values()):
        raise ValueError('manual_tuning参数须为已定义的有限数值')
    if not (0 <= result['kp'] <= result['kp_max'] <= 1000 and
            0 <= result['kd'] <= result['kd_max'] <= 100 and 0 < result['effort_limit_nm'] <= 100):
        raise ValueError('manual_tuning范围无效')
    return result


def create_session(profile, output, domain_id, scope, *, workspace=False):
    config = field.load_profile(profile,manual=True)
    tuning_config(config)
    if type(domain_id) is not int or not 0 <= domain_id <= 232:
        raise ValueError('必须显式输入现场确认的ROS域号')
    connection = config.get('connection')
    actual_scope = 'localhost_contract_mock' if connection else 'hardware_position_stream'
    if scope != actual_scope or (connection and connection['domain_id'] != domain_id):
        raise ValueError('域号/scope与profile连接不一致；不自动切换目标')
    root = Path(output).resolve(); root.mkdir(parents=True, exist_ok=False)
    protocol = root/'protocol'; protocol.mkdir()
    scene = config.pop('scene'); config['collision_scene'] = 'scene.yaml'
    (protocol/'scene.yaml').write_text(yaml.safe_dump(scene), encoding='utf-8')
    (protocol/'profile.yaml').write_text(yaml.safe_dump(config), encoding='utf-8')
    engine, _ = field.model_for(config)
    metadata = dict(schema_version='casbot_fieldwork_workspace_v1' if workspace else 'casbot_fieldwork_manual_v2', **run_marker(), domain_id=domain_id,
        scope=scope, joint_names=[j.name for j in engine.model.joints], model_context=config['model_context'],
        position_limits_source='bundled_URDF_not_device_readback', mechanical_zero_known=False,
        manual_tuning=tuning_config(config), hidden_compensation=False,
        physical_motion_qualified=False, hardware_calibration_accepted=False)
    atomic_json(protocol/'session.json', metadata); field.seal(protocol)
    event(root, 'session_created')
    return (Workspace(root) if workspace else Session(root)).status()


def initial_commands(q, kp, kd, tuning):
    """q is measured, PD may be read back; measured effort/dq are NOT target readback."""
    q = np.asarray(q, dtype=float)
    if q.shape != (14,) or not np.isfinite(q).all():
        raise ValueError('需要完整14轴首次位置反馈')
    result = dict(position=q.copy(), velocity=np.zeros(14), effort=np.zeros(14))
    source = dict(position='首次位置反馈', velocity='配置默认0；目标速度不可回读', effort='配置默认0；不复制反馈总输出')
    for name, value in [('kp', kp), ('kd', kd)]:
        a = np.asarray(value, dtype=float) if value is not None else None
        if a is not None and a.shape == (14,) and np.isfinite(a).all() and np.all(a >= 0):
            result[name] = a.copy(); source[name] = '首次可用生效增益反馈'
        else:
            result[name] = np.full(14, tuning[name]); source[name] = f'配置默认{tuning[name]:g}；无可用回读'
    return result, source


def validate_commands(values, lower, upper, tuning, velocity_limit):
    if set(values) != set(FIELDS):
        raise ValueError('需要position/velocity/effort/kp/kd五项')
    arrays = {k: np.asarray(values[k], dtype=float) for k in FIELDS}
    if any(a.shape != (14,) or not np.isfinite(a).all() for a in arrays.values()):
        raise ValueError('五字段必须是完整有限的14轴数组')
    if (np.any(arrays['position'] < lower) or np.any(arrays['position'] > upper) or
        np.any(abs(arrays['velocity']) > velocity_limit) or
        np.any(abs(arrays['effort']) > tuning['effort_limit_nm']) or
        np.any(arrays['kp'] < 0) or np.any(arrays['kp'] > tuning['kp_max']) or
        np.any(arrays['kd'] < 0) or np.any(arrays['kd'] > tuning['kd_max'])):
        raise ValueError('目标值超出界面配置范围')
    return arrays


def short_position(anchor, amplitude, duration_s, elapsed_s):
    """Three rest-to-rest C3 segments: +amplitude, -amplitude, return to anchor."""
    u = float(np.clip(elapsed_s/duration_s, 0., 1.))
    index = min(2, int(3*u)); local = 3*u-index
    blend = 35*local**4-84*local**5+70*local**6-20*local**7
    knots = (0., 1., -1., 0.)
    return anchor+amplitude*((1-blend)*knots[index]+blend*knots[index+1])


def write_command(path, generation, values):
    """One latest-value mailbox, atomic replacement; native C++ owns the sending rate."""
    path = Path(path); temporary = path.with_suffix('.pending')
    row = [generation] + [float(x) for name in FIELDS for x in values[name]]
    temporary.write_text(','.join(format(x, '.17g') for x in row)+'\n')
    temporary.replace(path)


class Session:
    schema = 'casbot_fieldwork_manual_v2'
    def __init__(self, root):
        self.root = Path(root).resolve(); field.check_seal(self.root/'protocol')
        self.metadata = read(self.root/'protocol/session.json')
        if self.metadata.get('schema_version') != self.schema:
            raise ValueError('旧验收向导会话不能作为手动调参会话；请新建目录')
        self.config = field.load_profile(self.root/'protocol/profile.yaml',manual=True)
        self.names = self.metadata['joint_names']

    def status(self):
        journal = events(self.root)
        starts = [e['run'] for e in journal if e['event'] in ('stream_started','zero_started')]
        ends = {e['run'] for e in journal if e['event'] in ('stream_finished','zero_finished')}
        fault = any(e['event'] == 'terminal_fault' for e in journal)
        return dict(**self.metadata, runs=starts, blocked_reason='会话故障或未封存，需新建会话' if fault or set(starts)-ends else None)

    def stream(self, relative, command_file, cancel_check=None, initial_values=None, on_event=None, recording_pause_file=None, settings=None, shared_raw=None):
        if self.status()['blocked_reason']:
            raise ValueError(self.status()['blocked_reason'])
        directory = (self.root/relative).resolve()
        if directory.parent != self.root/'runs' or not directory.is_dir():
            raise ValueError('运行目录必须属于本会话')
        if Path(command_file).resolve() != directory/'latest.csv':
            raise ValueError('指令邮箱必须属于本次运行')
        if Path(command_file).exists():
            raise ValueError('连接时邮箱必须为空，先读取再开始发送')
        policy_path=None
        config=self.config
        if settings is not None:
            from . import fieldwork_settings as policy
            engine,_=field.model_for(config)
            lower=[j.lower for j in engine.model.joints];upper=[j.upper for j in engine.model.joints]
            settings=policy.validate(settings,self.names,self.metadata['model_context'],lower,upper)
            atomic_json(directory/'settings_initial.json',settings)
            policy_path=directory/'limits.csv';policy.write_wire(policy_path,settings,lower,upper)
            config=dict(config,manual_stop_timeout_s=settings['general']['stop_timeout_s'],
                manual_recording_start_timeout_s=settings['general']['recording_start_timeout_s'],command_hz=int(settings['general']['command_hz']),
                max_feedback_age_ns=round(settings['general']['feedback_timeout_s']*1e9),max_mode_age_ns=round(settings['general']['mode_timeout_s']*1e9),
                transition_timeout_ns=round(settings['general']['rpc_timeout_s']*1e9),maximum_velocity_rad_s=float(np.max(policy.resolve(settings,lower,upper)['input_velocity'])))
        if initial_values is not None:
            engine,_ = field.model_for(self.config)
            bounds=policy.command_bounds(settings,lower,upper) if settings is not None else (
                [j.lower for j in engine.model.joints],[j.upper for j in engine.model.joints],self.metadata['manual_tuning'],self.config['maximum_velocity_rad_s'])
            values=validate_commands(initial_values,*bounds)
            write_command(command_file,0,values)
        native_events=[];diagnostics=[]
        def output(line):
            from .fieldwork_status import describe
            if line.startswith(('FIELDWORK_EVENT ','FIELDWORK_SETTINGS ')):
                value=describe(json.loads(line.split(' ',1)[1]));native_events.append(value)
                if on_event:on_event(value)
            elif line.startswith('FIELDWORK_NOTICE ') and on_event:
                value=json.loads(line[len('FIELDWORK_NOTICE '):]);on_event(describe(dict(value,action='recording_warning')))
            elif line.strip():
                diagnostics.append(line.strip());del diagnostics[:-8]
        event(self.root, 'stream_started', run=relative)
        try:
            result = field.run_executor(config, directory/'capture', self.metadata['domain_id'],
                manual_command=command_file, manual_effort_limit_nm=self.metadata['manual_tuning']['effort_limit_nm'],
                timeout_s=None if settings is not None else 24*3600,
                **(dict(manual_policy=policy_path) if policy_path else {}), cancel_check=cancel_check, on_output=output,
                **(dict(recording_pause_file=recording_pause_file) if recording_pause_file else {}),
                **(dict(shared_raw=shared_raw) if shared_raw is not None else {}))
            controlled = result.get('fault') == 'interrupted' and result.get('supervisor_interruption') in ('operator_cancel', 'frontend_closed')
            controlled=controlled or (settings is not None and result.get('fault')=='duration_complete')
            event(self.root, 'stream_finished', run=relative, controlled_stop=controlled)
            if not controlled:
                event(self.root, 'terminal_fault', run=relative, reason=result.get('fault') or result['reason'])
            # Archive events, keeping the original initiating fault distinct from stop observations.
            atomic_json(directory/'control_status.json',dict(events=native_events,
                first_fault=next((e for e in native_events if e.get('action')=='end_control'),None)))
            recording_error='控制CSV录制已暂停；已写前缀保留，数据不完整' if result.get('recording_paused') else None
            try:
                if recording_pause_file and Path(recording_pause_file).exists():raise ValueError('缓存已满；原始前缀保留，记录导出不完整')
                if recording_pause_file is None:export_recording(self, directory) # Spools write the small recording manifest only on explicit save.
            except (OSError,ValueError) as exc:
                recording_error=str(exc)
                if on_event:on_event(dict(code='recording_io_error',title='控制已结束，但记录导出不完整',
                    severity='warning',action='recording_warning',effect=str(exc),details=[]))
            return dict(run=relative, controlled_stop=controlled, result=result,recording_error=recording_error,
                        diagnostics=diagnostics,recording_deferred=bool(recording_pause_file),
                        first_fault=next((e for e in native_events if e.get('action')=='end_control'),None))
        except Exception:
            event(self.root, 'terminal_fault', run=relative, reason='stream_or_export_failed')
            raise


class Workspace(Session):
    """Long-lived read-only connection; each explicit start owns a fresh control session."""
    schema = 'casbot_fieldwork_workspace_v1'
    def status(self):
        journal=events(self.root)
        controls=[e['control'] for e in journal if e['event']=='control_created']
        return dict(**self.metadata,controls=controls,runs=[],blocked_reason=None)
    def new_control(self):
        import uuid
        relative='controls/'+uuid.uuid4().hex[:12]
        directory=self.root/relative
        create_session(self.root/'protocol/profile.yaml',directory,self.metadata['domain_id'],self.metadata['scope'])
        event(self.root,'control_created',control=relative)
        return relative


def recording_capture(directory):
    """Resolve the executed zero reference after a pre-control start replan."""
    directory=Path(directory);capture=directory/'capture'
    result=read(capture/'result.json') if (capture/'result.json').exists() else {}
    if result.get('schema_version')=='casbot_field_start_replans_v1' and result.get('passed'):
        return field.resolve_field_capture(directory/'plan',capture)[1]
    return capture


def export_recording(session, directory, *, recording_paused=False):
    """Write a small relocatable manifest; native CSVs remain the only authoritative copy."""
    import os
    capture = recording_capture(directory); output = directory/'recording'; output.mkdir(exist_ok=True)
    files = {name: os.path.relpath(capture/'execution'/name, output)
             for name in ('commands.csv', 'feedback.csv', 'events.csv')}
    init = read(directory/'initial.json') if (directory/'initial.json').exists() else None
    from .raw_archive import capture_raw
    raw,start,end=capture_raw(capture)
    execution_result=read(capture/'execution/result.json') if (capture/'execution/result.json').exists() else {}
    atomic_json(output/'recording.json', dict(schema_version='casbot_fieldwork_recording_v3', files=files, execution_recording_status='paused_prefix' if recording_paused or execution_result.get('recording_paused') else 'finalized_prefix' if execution_result else 'live_prefix',**run_marker(), joint_names=session.names,
        model_context=session.metadata['model_context'], scope=session.metadata['scope'], initial=init,
        timestamp_semantics=dict(publish_ns='sender steady clock', receive_ns='callback steady clock',
            publish_stamp_ns='SDK publication stamp, NOT acquisition'), sample_time_known=False,
        feedback_effort='module estimate; not a command default or independent torque sensor',
        raw_state=os.path.relpath(raw,output),raw_interval_receive_ns=[start,end],hidden_compensation=False))
    return str(output)


def export_readback(directory, output, *, start_s=0., end_s=None, joints=None):
    """Explicit derived CSV; time window is relative to the first joint/debug receipt."""
    from .vendor_data import normalize_packet
    directory=Path(directory).resolve();output=Path(output).expanduser().resolve()
    if directory == output or directory in output.parents:
        raise ValueError('导出文件须位于源运行目录之外')
    if not np.isfinite(start_s) or start_s < 0 or (end_s is not None and (not np.isfinite(end_s) or end_s <= start_s)):
        raise ValueError('时间窗须为非负起点和更大的有限终点')
    indices=list(range(14)) if joints is None else list(joints)
    if not indices or len(set(indices))!=len(indices) or any(type(j) is not int or not 0<=j<14 for j in indices):
        raise ValueError('轴索引须为不重复的0..13')
    # Control and workspace use the same sealed joint order.
    session=Session(directory.parent.parent)
    from .raw_archive import capture_raw,iter_records
    raw,start,end=capture_raw(recording_capture(directory))
    keys=('q','dq','effort_raw','kp','kd');origin=None;count=0
    with output.open('x') as stream:
        writer=csv.writer(stream)
        writer.writerow(['channel','receive_ns','publication_stamp_sec','publication_stamp_nanosec',
                         *[f'{key}{j}' for key in keys for j in indices]])
        for packet,_ in iter_records(raw,start_ns=start,end_ns=end):
            if packet['channel'] not in ('joint','debug'):continue
            received=int(packet['receive_ns'])
            if origin is None:origin=received
            elapsed=received-origin
            if elapsed < round(start_s*1e9) or (end_s is not None and elapsed >= round(end_s*1e9)):continue
            value=normalize_packet(packet,dict(joint_names=session.names))
            if value['q'] is None:continue
            header=packet['message'].get('header',{});stamp=header.get('stamp',header)
            writer.writerow([packet['channel'],received,stamp.get('sec',''),stamp.get('nanosec',''),
                *[value[key][j] if value[key] is not None else '' for key in keys for j in indices]])
            count+=1
    return dict(output=str(output),rows=count,time_origin='first joint/debug receive_ns',joints=indices)


def heartbeat_check(path, *, timeout_s=2., expected_owner=None):
    """Frontend-owned file, checked in the existing SDK process supervisor."""
    from .fieldwork import clock_id
    expected = clock_id()
    def check():
        try:
            record = read(path)
            if expected_owner is not None and record.get('owner_id') != expected_owner:
                return 'frontend_owner_changed'
            if record['clock_id'] != expected:
                return 'frontend_clock_mismatch'
            if record.get('cancel'):
                return record['cancel']
            age = time.monotonic_ns()-record['monotonic_ns']
            if not 0 <= age <= timeout_s*1e9:
                return 'frontend_heartbeat_timeout'
        except (OSError, ValueError, KeyError):
            return 'frontend_heartbeat_unavailable'
        return None
    return check
