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
    config = field.load_profile(profile)
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
        self.config = field.load_profile(self.root/'protocol/profile.yaml')
        self.names = self.metadata['joint_names']

    def status(self):
        journal = events(self.root)
        starts = [e['run'] for e in journal if e['event'] == 'stream_started']
        ends = {e['run'] for e in journal if e['event'] == 'stream_finished'}
        fault = any(e['event'] == 'terminal_fault' for e in journal)
        return dict(**self.metadata, runs=starts, blocked_reason='会话故障或未封存，需新建会话' if fault or set(starts)-ends else None)

    def stream(self, relative, command_file, cancel_check=None, initial_values=None, on_event=None, recording_pause_file=None):
        if self.status()['blocked_reason']:
            raise ValueError(self.status()['blocked_reason'])
        directory = (self.root/relative).resolve()
        if directory.parent != self.root/'runs' or not directory.is_dir():
            raise ValueError('运行目录必须属于本会话')
        if Path(command_file).resolve() != directory/'latest.csv':
            raise ValueError('指令邮箱必须属于本次运行')
        if Path(command_file).exists():
            raise ValueError('连接时邮箱必须为空，先读取再开始发送')
        if initial_values is not None:
            engine,_ = field.model_for(self.config)
            values=validate_commands(initial_values,[j.lower for j in engine.model.joints],
                [j.upper for j in engine.model.joints],self.metadata['manual_tuning'],self.config['maximum_velocity_rad_s'])
            write_command(command_file,0,values)
        native_events=[];diagnostics=[]
        def output(line):
            from .fieldwork_status import describe
            if line.startswith('FIELDWORK_EVENT '):
                value=describe(json.loads(line[len('FIELDWORK_EVENT '):]));native_events.append(value)
                if on_event:on_event(value)
            elif line.startswith('FIELDWORK_NOTICE ') and on_event:
                value=json.loads(line[len('FIELDWORK_NOTICE '):]);on_event(describe(dict(value,action='recording_warning')))
            elif line.strip():
                diagnostics.append(line.strip());del diagnostics[:-8]
        event(self.root, 'stream_started', run=relative)
        try:
            result = field.run_executor(self.config, directory/'capture', self.metadata['domain_id'],
                manual_command=command_file, manual_effort_limit_nm=self.metadata['manual_tuning']['effort_limit_nm'],
                timeout_s=24*3600, cancel_check=cancel_check, on_output=output if on_event else None,
                **(dict(recording_pause_file=recording_pause_file) if recording_pause_file else {}))
            controlled = result.get('fault') == 'interrupted' and result.get('supervisor_interruption') in ('operator_cancel', 'frontend_closed')
            event(self.root, 'stream_finished', run=relative, controlled_stop=controlled)
            if not controlled:
                event(self.root, 'terminal_fault', run=relative, reason=result.get('fault') or result['reason'])
            # Archive events, keeping the original initiating fault distinct from stop observations.
            atomic_json(directory/'control_status.json',dict(events=native_events,
                first_fault=next((e for e in native_events if e.get('action')=='end_control'),None)))
            recording_error=None
            try:
                if recording_pause_file and Path(recording_pause_file).exists():raise ValueError('缓存已满；原始前缀保留，记录导出不完整')
                if recording_pause_file is None:export_recording(self, directory) # Automatic spools derive comparison files only on explicit save.
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


def export_recording(session, directory):
    """Comparison CSVs keep native publish/receive clocks, all frames and missing values."""
    import shutil
    capture = directory/'capture'; output = directory/'recording'; output.mkdir(exist_ok=True)
    for filename in ('commands.csv', 'feedback.csv', 'events.csv'):
        shutil.copyfile(capture/'execution'/filename, output/filename)
    from .vendor_data import normalize_packet
    packets = capture/'raw_state/raw/packets.jsonl'
    with (output/'readback.csv').open('w') as stream:
        writer = csv.writer(stream)
        keys = ('q','dq','effort_raw','kp','kd')
        writer.writerow(['channel','receive_ns','publication_stamp_sec','publication_stamp_nanosec',
                         *[f'{key}{j}' for key in keys for j in range(14)]])
        if packets.exists():
            with packets.open() as raw:
                for line in raw:
                    packet = json.loads(line)
                    if packet['channel'] not in ('joint','debug'): continue
                    value = normalize_packet(packet, dict(joint_names=session.names))
                    if value['q'] is None: continue
                    stamp = packet['message'].get('header',{}).get('stamp',{})
                    writer.writerow([packet['channel'],packet['receive_ns'],stamp.get('sec',''),stamp.get('nanosec',''),
                        *[x for key in keys for x in (value[key] if value[key] is not None else ['']*14)]])
    init = read(directory/'initial.json') if (directory/'initial.json').exists() else None
    atomic_json(output/'recording.json', dict(**run_marker(), joint_names=session.names,
        model_context=session.metadata['model_context'], scope=session.metadata['scope'], initial=init,
        timestamp_semantics=dict(publish_ns='sender steady clock', receive_ns='callback steady clock',
            publish_stamp_ns='SDK publication stamp, NOT acquisition'), sample_time_known=False,
        feedback_effort='module estimate; not a command default or independent torque sensor',
        raw_state='../capture/raw_state', hidden_compensation=False))
    return str(output)


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
