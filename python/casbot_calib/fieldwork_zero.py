"""Shared user pose bookmark and explicit one-way SDK return, for CLI and GUI."""
import json
import csv
import os
from pathlib import Path
import tempfile
import time
import uuid

import numpy as np
import yaml

from . import hardware_field as field
from .data import write_json
from .model_runtime import model_for
from .run_marker import run_marker
from .sdk_connection import connection_remaps
from .staged_motion import Motion
from .static_field import _move


ZERO_FILE = Path(__file__).resolve().parents[2] / 'fieldwork/zero.yaml'


def print_status(message):
    print(message, flush=True)


def operation_directory(parent=None):
    parent = Path(parent) if parent else ZERO_FILE.parent.parent/'artifacts/fieldwork/zero_runs'
    return parent/(time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())+'_'+uuid.uuid4().hex[:8])


def binding(config, domain_id, scope):
    if type(domain_id) is not int or not 0 <= domain_id <= 232:
        raise ValueError('必须显式提供ROS域号 0..232')
    remaps = connection_remaps(config, domain_id)
    expected = 'localhost_contract_mock' if remaps else 'hardware_position_stream'
    if scope != expected:
        raise ValueError('scope与profile连接不一致；不会自动切换目标')
    engine, _ = model_for(config)
    return dict(domain_id=domain_id, scope=scope, model_context=config['model_context'],
                joint_names=[j.name for j in engine.model.joints])


def positions(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (14,) or not np.isfinite(q).all():
        raise ValueError('需要完整、有限的14轴实际位置反馈')
    return q


def save_zero(config, domain_id, scope, q, *, receive_ns, path=ZERO_FILE):
    """Replace only after a complete read; never write robot encoder offsets."""
    metadata = binding(config, domain_id, scope)
    q = positions(q)
    data = dict(schema_version='casbot_fieldwork_zero_v1', **run_marker(), **metadata,
        units='rad', position_rad=dict(zip(metadata['joint_names'], q.tolist())),
        source=dict(kind='received_joint_position', receive_ns=int(receive_ns),
                    clock='host_receive_monotonic_not_device_sample'),
        meaning='user_saved_pose_not_mechanical_or_encoder_zero')
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, prefix='.zero-', suffix='.yaml',
                                          encoding='utf-8', delete=False) as stream:
            temporary = Path(stream.name)
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
            stream.flush(); os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        if temporary and temporary.exists(): temporary.unlink()
    return data


def load_zero(config, domain_id, scope, *, path=ZERO_FILE):
    path = Path(path)
    if not path.exists(): raise ValueError('尚未采集零位：'+str(path))
    try:data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except yaml.YAMLError as exc:raise ValueError('零位YAML格式错误：'+str(path)) from exc
    if not isinstance(data, dict) or data.get('schema_version') != 'casbot_fieldwork_zero_v1' or data.get('units') != 'rad':
        raise ValueError('零位文件格式/单位不正确：'+str(path))
    expected = binding(config, domain_id, scope)
    if any(data.get(key) != value for key, value in expected.items()):
        raise ValueError('零位的模型、14轴顺序、域号或scope与本次连接不同，请在当前连接重新采集')
    values = data.get('position_rad')
    if not isinstance(values, dict) or set(values) != set(expected['joint_names']):
        raise ValueError('零位文件必须含完整14轴命名位置')
    positions([values[name] for name in expected['joint_names']])
    return data


def capture_zero(config, domain_id, scope, *, path=ZERO_FILE, output=None, cancel_check=None):
    """Subscription-only: capture the first complete q, independent of dq/effort."""
    metadata = binding(config, domain_id, scope)
    root = Path(output) if output else operation_directory()
    root.mkdir(parents=True, exist_ok=False)
    observation = yaml.safe_load(Path('configs/vendor_s1.yaml').read_text())
    remaps = connection_remaps(config, domain_id) or {}
    observation.update(model_context=metadata['model_context'], joint_names=metadata['joint_names'],
        model_id=metadata['model_context']['model_id'], joint_type=config['feedback_type'],
        joint_topic=remaps.get(config['joint_topic'], config['joint_topic']),
        mode_topic='', robot_state_topic='', debug_topic='')
    config_path = root/'observation.yaml'; config_path.write_text(yaml.safe_dump(observation))
    received = {}
    from .vendor_data import normalize_packet
    from .vendor_capture import capture
    def line(text):
        if not text.startswith('FIELDWORK_PACKET '): return
        packet = json.loads(text[len('FIELDWORK_PACKET '):])
        if packet['channel'] != 'joint' or received: return
        q = normalize_packet(packet, observation)['q']
        if q is not None:
            received.update(q=positions(q).tolist(), receive_ns=packet['receive_ns'])
    def cancel():
        reason = cancel_check() if cancel_check else None
        return reason or ('zero_position_received' if received else None)
    captured = capture(config_path, root/'capture', duration_s=10., domain_id=domain_id,
        scope='localhost_contract_mock' if remaps else 'hardware_read_only',
        live_callback=line, cancel_check=cancel)
    if (captured or {}).get('user_interrupted'): raise KeyboardInterrupt
    reason = cancel_check() if cancel_check else None
    if reason: raise ValueError('零位采集已取消：'+reason)
    if not received: raise ValueError('未收到完整14轴位置；原零位未改动。记录：'+str(root))
    data = save_zero(config, domain_id, scope, **received, path=path)
    write_json(root/'zero.json', data)
    return dict(passed=True, zero_file=str(path), recording=str(root), zero=data)


def prepare_zero(config, snapshot, output, target, duration_s, *, bindings=None):
    target = positions(target)
    if not np.isfinite(duration_s) or duration_s <= 0: raise ValueError('回零时长必须为正数')
    start = field._read_snapshot(snapshot)
    root = Path(output); root.mkdir(parents=True, exist_ok=False)
    motion = Motion('saved_zero_return', 'position', start)
    motion.add(config['initial_hold_s'], start, label='measured_start')
    _move(motion, target, duration_s, dict(speed_rad_s=min(.3, config['maximum_velocity_rad_s']),
        acceleration_rad_s2=.5, jerk_rad_s3=2.), 'to_saved_zero')
    motion.add(config['final_hold_s'], target, label='saved_zero_final_hold')
    recipe = dict(kind='zero', target_rad=target.tolist(), duration_s=duration_s)
    return field.finish_plan(root, config, motion, [], dict(field.snapshot_binding(snapshot, config),
        **(bindings or {}), replan_recipe=recipe))


def return_zero(config, domain_id, scope, *, duration_s=5., kp=20., kd=2., path=ZERO_FILE,
                zero=None, output=None, cancel_check=None, notify=print_status):
    """Freeze target once; reference/collision checks and native executor are shared."""
    if not np.isfinite(duration_s) or duration_s <= 0: raise ValueError('回零时长必须为正数')
    data = load_zero(config, domain_id, scope, path=path) if zero is None else zero
    if data.get('schema_version') != 'casbot_fieldwork_zero_v1' or data.get('units') != 'rad':
        raise ValueError('零位文件格式/单位不正确')
    # Revalidate even a GUI-frozen bookmark against the current connection.
    expected = binding(config, domain_id, scope)
    if any(data.get(key) != value for key, value in expected.items()): raise ValueError('零位连接绑定不一致')
    target = positions([data['position_rad'][name] for name in expected['joint_names']])
    gains = {}
    for name, value in [('command_kp', kp), ('command_kd', kd)]:
        values = np.broadcast_to(np.asarray(value, dtype=float), (14,))
        maximum = 1000. if name == 'command_kp' else 100.
        if not np.isfinite(values).all() or np.any(values < 0) or np.any(values > maximum):
            raise ValueError(f'{name}需要14轴非负有限值，当前SDK发送器范围0..{maximum:g}')
        gains[name] = values.tolist()
    config = dict(config, gravity_feedforward='none', breakaway_parameters=gains)
    root = Path(output) if output else operation_directory()
    root.mkdir(parents=True, exist_ok=False)
    write_json(root/'zero.json', data)
    notify('正在读取本次回零起点；尚未发送运动指令。')
    result = field.run_executor(config, root/'snapshot', domain_id, snapshot=True, cancel_check=cancel_check)
    if not result['passed']: return dict(passed=False, result=result, recording=str(root))
    notify('正在规划当前位置到已保存零位的单程轨迹，并检查参考碰撞。')
    report = prepare_zero(config, root/'snapshot', root/'plan', target, duration_s)
    if cancel_check and cancel_check(): return dict(passed=False, result=dict(reason='operator_cancel'), recording=str(root))
    notify(f"正在回到保存零位，参考总时长 {report['duration_s']:.2f}s；无重力/摩擦前馈。")
    result = field.execute_field(root/'plan', root/'capture', domain_id, cancel_check=cancel_check)
    effective_capture = root/'capture'
    if result['passed']:
        _, effective_capture = field.resolve_field_capture(root/'plan', effective_capture)
    feedback_file = effective_capture/'execution/feedback.csv'
    final = None
    if feedback_file.exists():
        with feedback_file.open() as stream:
            for row in csv.DictReader(stream): final = row
    observed = None if final is None else [float(final['q'+str(j)]) for j in range(14)]
    summary = dict(passed=result['passed'], result=result, recording=str(root), zero_file=str(path),
        last_received_position_rad=observed,
        final_received_error_rad=None if observed is None else (np.asarray(observed)-target).tolist(),
        completion='reference_sent_and_mode_exited_not_pose_acceptance', hidden_compensation=False)
    write_json(root/'result.json', summary)
    return summary
