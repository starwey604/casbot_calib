"""Orin-native SHM field cases. Standard library only; never runs a control loop."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

from .shm_data import (
    COMMAND_RECORD_DTYPE,
    DIRECTION,
    FEEDBACK_RECORD_DTYPE,
    NAMES,
    OFFSET,
    PARTS,
    ROOT,
    URDF,
)

try:
    import numpy as _np
except ImportError:
    _np = None

try:
    import orjson as _orjson
except ImportError:
    _orjson = None
CASES = {
    'C00': ('全轴被动基线', '不出队、不写命令；原业务退出/主站启停分别保存快照。'),
    'C01': ('首帧与接管', '失能→MIT→按新鲜实际位置使能保持→有限时间后失能。'),
    'C02': ('保持与正常结束', '保持期间观察，再失能退出；hold/disable分别标记观察重点。'),
    'C03': ('双臂单轴对应', 'q0→q0−a→q0+a→q0；其它双臂固定起点，双向URDF检查。'),
    'C04': ('非目标部位关系', '单轴运动并记录全身；可显式改为对侧臂失能，其它背景不变。'),
    'C05': ('停止写命令', '先保持，停止入队但继续出队观察，观察结束再请求失能。'),
    'C06': ('发送进程消失', '保持后终止本次子进程；独立O_RDONLY探针记录，SIGKILL后不补发。'),
    'C07': ('手部专项', '当前仅记录双手原始字段；主动动作待手模式/单位明确。'),
    'C08': ('五字段与尺度', '按既定决定后续核对；本轮不自动加入前馈/零PD尺度试验。'),
}


def _reject_nonfinite(value):
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError('Out of range float values are not JSON compliant')
    elif _np is not None and isinstance(value, _np.ndarray):
        if value.dtype.kind == 'f' and not _np.isfinite(value).all():
            raise ValueError('Out of range float values are not JSON compliant')
    elif _np is not None and isinstance(value, _np.floating):
        if not _np.isfinite(value):
            raise ValueError('Out of range float values are not JSON compliant')
    elif isinstance(value, dict):
        for v in value.values():
            _reject_nonfinite(v)
    elif isinstance(value, (list, tuple)):
        for v in value:
            _reject_nonfinite(v)


def _orjson_default(obj):
    if _np is not None:
        if isinstance(obj, _np.ndarray):
            if obj.dtype.kind in 'biuf':
                return _np.ascontiguousarray(obj)
            return obj.tolist()
        if isinstance(obj, _np.generic):
            return obj.item()
    raise TypeError(f'Object of type {type(obj).__name__} is not JSON serializable')


def dump(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    if _orjson is not None:
        raw = _orjson.dumps(
            value,
            option=_orjson.OPT_SERIALIZE_NUMPY | _orjson.OPT_NON_STR_KEYS | _orjson.OPT_INDENT_2,
            default=_orjson_default,
        )
        if b'null' in raw:
            _reject_nonfinite(value)
        temporary.write_bytes(raw + b'\n')
    else:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def text_result(path):
    if not path.exists():
        return None
    return dict(line.split('=', 1) for line in path.read_text().splitlines() if '=' in line)


def endpoint(value):
    value = '/' + value.lstrip('/')
    if value == '/' or '/' in value[1:]:
        raise argparse.ArgumentTypeError('共享内存名称不能包含目录')
    return value


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError('参数须为有限数值')
    return value


def parser():
    p = argparse.ArgumentParser(description='SHM现场逐例确认；默认预览，一次一例。无需ROS。')
    sub = p.add_subparsers(dest='action', required=True)
    sub.add_parser('list', help='列用例、当前支持范围和建议顺序')
    sub.add_parser('gui', help='本机GUI及渲染，通过SSH连接Orin或私有Mock')
    sub.add_parser('a-evidence', help='A分析，使用SHM观测时间（需要数值Python环境）')
    hold = sub.add_parser('hold-body', help='双臂进入准备位后保持，测试辅助站立')
    hold.add_argument('--duration', type=finite, default=10.)
    hold.add_argument('--scale', type=finite, default=.5)
    hold.add_argument('--execute', action='store_true')
    hold.add_argument('--mock', action='store_true')
    hold.add_argument('--mock-config', type=Path)
    hold.add_argument('--executor', type=Path)
    hold.add_argument('--output', type=Path)
    for action in ('inspect', 'snapshot', 'observe', 'probe', 'prepare', 'activate', 'run', 'execute'):
        a = sub.add_parser(action)
        a.add_argument('--output', type=Path, help='新目录；省略则自动生成')
        a.add_argument('--endpoint', type=endpoint, default='/shm_mc')
        if action != 'prepare':
            a.add_argument('--executor', type=Path, default=ROOT/'install/shm-core/bin/shm_executor')
        if action in ('prepare', 'activate', 'run', 'execute'):
            a.add_argument('--execute', action='store_true', help='显式实施；不加则只预览')
        if action in ('observe', 'snapshot', 'run', 'execute'):
            a.add_argument('--urdf', type=Path, default=None)
            a.add_argument('--mock', action='store_true', help='独占私有物理SHM Mock，不接实机')
            a.add_argument('--mock-config', type=Path, default=ROOT/'configs/shm/mock.json')
        if action in ('observe', 'probe', 'run'):
            a.add_argument('--duration', type=finite, default=10 if action != 'run' else 3)
        if action == 'prepare':
            a.add_argument('--start-only', type=Path, help='沿用已有停业务日志，只复核并启动主站')
        if action == 'run':
            a.add_argument('case', choices=CASES)
            a.add_argument('--joint-id', type=int, choices=range(12, 26), default=12)
            a.add_argument('--amplitude', type=finite, default=.5)
            a.add_argument('--segment-s', type=finite, default=5)
            a.add_argument('--kp', type=finite, default=20)
            a.add_argument('--kd', type=finite, default=2)
            a.add_argument('--quiet-s', type=finite, default=3)
            a.add_argument('--other-arm', choices=('hold', 'disabled'), default='hold')
            a.add_argument('--termination', choices=('sigterm', 'sigkill'), default='sigterm', help='C06：受控终止或硬杀本次发送器')
            a.add_argument('--hand-background', type=Path)
            a.add_argument('--snapshot', type=Path, help='可选：用于离线预览的快照文件或记录目录；执行仍重读起点')
        if action == 'execute':
            a.add_argument('reference',type=Path)
            a.add_argument('--hand-background',type=Path)
            a.set_defaults(case='REF',joint_id=12,amplitude=.5,segment_s=5,kp=20,kd=2,duration=3,
                           quiet_s=3,other_arm='hold',termination='sigterm',snapshot=None)
    for action in ('plan','fields'):
        a=sub.add_parser(action,help='离线生成参考，不连接SHM')
        a.add_argument('--snapshot',type=Path)
        a.add_argument('--joint-id',type=int,choices=range(12,26),default=12)
        a.add_argument('--kp',type=finite,default=20);a.add_argument('--kd',type=finite,default=2)
        a.add_argument('--output',type=Path,required=True)
        if action=='plan':
            a.add_argument('--relative',action='store_true',help='相对执行时新鲜起点；仍重新检查URDF')
            a.add_argument('--amplitude',type=finite,default=.01)
            a.add_argument('--segment-s',type=finite,default=2)
        else:
            a.add_argument('--position',type=finite)
            a.add_argument('--velocity',type=finite,default=0)
            a.add_argument('--effort',type=finite,default=0)
            a.add_argument('--duration',type=finite,default=3)
    a = sub.add_parser('note', help='离线登记用户观察，不连接设备')
    a.add_argument('run', type=Path);a.add_argument('--text', required=True)
    a.add_argument('--phase', default='unspecified')
    a.add_argument('--conclusion', choices=('observed', 'inconclusive', 'contradicted'), default='observed')
    a = sub.add_parser('report', help='离线重分析或恢复完整记录前缀，不连接设备')
    a.add_argument('run', type=Path)
    a = sub.add_parser('status', help='汇总已运行用例与人工观察，不把执行成功自动当验证通过')
    a.add_argument('folder', type=Path, nargs='?', default=ROOT/'artifacts/shm_fieldwork')
    return p


def validate(a):
    if a.action == 'prepare' and a.endpoint != '/shm_mc':
        raise ValueError('prepare仅用于厂家shm_mc；私有夹具不需要停业务')
    if getattr(a,'mock',False) and a.endpoint!='/shm_mc':
        raise ValueError('--mock自动创建独占端点，不同时指定--endpoint')
    if hasattr(a, 'duration') and not 0 < a.duration <= 3600:
        raise ValueError('duration须在(0,3600]秒')
    if a.action not in ('run','execute'):
        return
    if not 0 < a.amplitude <= .5 or a.segment_s < 1 or not 0 <= a.kp <= 100 or not 0 <= a.kd <= 10 or not 0 < a.quiet_s <= 60:
        raise ValueError('幅值(0,.5]，每段≥1秒，Kp[0,100]，Kd[0,10]，quiet-s(0,60]')
    if a.other_arm != 'hold' and a.case != 'C04':
        raise ValueError('背景对照仅在C04用--other-arm disabled显式指定')
    if a.case in ('C07', 'C08') and a.execute:
        raise ValueError(CASES[a.case][1] + '；此入口不发主动命令')
    if a.case == 'C00' and a.mock:
        raise ValueError('C00用于既有业务的被动背景；新建Mock请用observe，只读零发送')
    if a.execute and a.case != 'C00' and not a.mock:
        if not a.hand_background or not a.hand_background.is_file():
            raise ValueError('主动用例需要显式--hand-background（停业务前command.bin，只取手部）')
        data = a.hand_background.read_bytes()
        if len(data) != 744 or struct.unpack_from('<H', data, 10)[0] > 2:
            raise ValueError('手背景须为744字节35槽命令，hand_count≤2')


def state_path(path):
    if path.is_file():
        return path
    for name in ('native/status.bin', 'status.bin'):
        if (path/name).is_file():
            return path/name
    raise ValueError(f'找不到快照：{path}')


def preview(a):
    policy = dict(non_arm='腿0–11/头26–27/腰28：失能0x06，位置填新鲜原始值',
                  other_arms=('测试侧其它轴保持本次实际起点；对侧臂失能' if a.other_arm == 'disabled' else '其它双臂轴固定本次实际起点；增益采用本例Kp/Kd'),
                  hands='仅冻结显式背景命令的双手字段和hand_count', reserved='槽29–34及其它保留字节填0',
                  count=dict(joint_count=29, hand_count='来自显式背景，≤2'),
                  exit='失能并等待状态应答；有界异常退出，不清故障/不resume')
    d = dict(case=a.case, title=CASES.get(a.case,('通用参考/五字段','执行显式冻结bundle，不继承SDK插值'))[0], behavior=CASES.get(a.case,('','执行显式冻结bundle'))[1], endpoint=a.endpoint,
             joint_id=a.joint_id, joint_label=('左' if a.joint_id < 19 else '右') + PARTS[(a.joint_id-12) % 7],
             amplitude_rad=a.amplitude, segment_s=a.segment_s, kp=a.kp, kd=a.kd, effort_nm=0,
             duration_s=a.duration, quiet_s=a.quiet_s, other_arm=a.other_arm,
             termination=a.termination, collision_check='disabled_for_arm_field_cases',
             background=policy, raw_gain=dict(kp=math.trunc(a.kp*10), kd=math.trunc(a.kd*10)),
             urdf=str(a.urdf.resolve()), hand_background=str(a.hand_background.resolve()) if a.hand_background else None,
             phases=['失能', '失能状态切MIT', '起点使能', '用例执行', '失能退出'])
    if a.case in ('C07', 'C08'):
        d['phases'] = ['此入口无主动实现，保留只读字段/待确认事项']
    if a.case == 'C00':
        d['phases'] = ['O_RDONLY/pread快照，无控制']
    if a.case == 'C05':
        d['phases'].insert(-1, f'停止所有写入，继续接收{a.quiet_s:g}秒')
    if a.case == 'C06':
        d['phases'][-1] = f'{a.termination}终止自有发送器，独立被动观察{a.quiet_s:g}秒'
        d['exit_warning'] = 'SIGKILL后不补发失能，旧目标可能无限延续，现场用独立继电器结束；探针不是控制消费者。'
    if a.hand_background and a.hand_background.is_file():
        data = a.hand_background.read_bytes()
        if len(data) != 744:
            raise ValueError('手背景须为744字节命令')
        d['hand_count'] = struct.unpack_from('<H', data, 10)[0]
        d['hand_bytes_hex'] = data[642:722].hex()
    if a.snapshot:
        raw = state_path(a.snapshot).read_bytes()
        if len(raw) != 696:
            raise ValueError('状态快照须为696字节')
        limits = {e.attrib['name']: e.find('limit') for e in ET.parse(a.urdf).getroot().findall('joint')}
        arms = []
        for j, name in enumerate(NAMES):
            r = struct.unpack_from('<HiihhH', raw, 12 + (j+12)*16)
            q = DIRECTION[j]*r[1]*1e-6 + OFFSET[j]
            lim = limits[name]
            lo, hi = float(lim.attrib['lower']), float(lim.attrib['upper'])
            arms.append(dict(id=j+12, name=name, q=q, lower=lo, upper=hi,
                             minus=q-a.amplitude, plus=q+a.amplitude,
                             within_limits=lo <= q-a.amplitude <= q+a.amplitude <= hi,
                             target_raw=math.trunc(DIRECTION[j]*(q-OFFSET[j])*1e6),
                             minus_raw=math.trunc(DIRECTION[j]*(q-a.amplitude-OFFSET[j])*1e6),
                             plus_raw=math.trunc(DIRECTION[j]*(q+a.amplitude-OFFSET[j])*1e6)))
        d['snapshot_arms'] = arms
        d['non_arm_targets_raw'] = {str(j): struct.unpack_from('<i', raw, 12+j*16+2)[0]
                                    for j in list(range(12))+[26, 27, 28]}
        d['preview_notice'] = '此快照只供预览；执行重读所有起点并重新检查，不沿用旧起点。'
    else:
        d['preview_notice'] = '未提供--snapshot，起点/原始位置和URDF结果将在执行时读取；当前预览不连接SHM。'
    if a.action=='execute':
        d.update(collision_check='native_URDF_only; H1 certificates checked by h1 execute wrapper',
                 reference=str(a.reference.resolve()),reference_schema=a.bundle['schema'],duration_s=a.duration,
                 origin=a.bundle.get('origin','absolute'),joint_id=None,joint_label='双臂14轴',
                 input_notice='直接五字段不隐式平滑' if a.bundle['schema']=='casbot_shm_fields_v1' else '同一条七次Hermite参考的q/v')
        for key in ('amplitude_rad','segment_s','kp','kd','effort_nm','raw_gain','other_arm','termination'):
            d.pop(key,None)
        d['background']['other_arms']='双臂按bundle逐轴目标；进入阶段先保持新鲜实际起点'
        d['kp']=a.bundle['kp'];d['kd']=a.bundle['kd']
        if 'pd_schedule' in a.bundle:
            d['pd_schedule']=a.bundle['pd_schedule'];d['pd_interpolation']='quintic smoothstep between schedule knots; hold last'
        if 'markers' in a.bundle:d['reference_markers']=a.bundle['markers']
        if a.bundle['schema']=='casbot_shm_fields_v1':
            d['fields']={key:a.bundle[key] for key in ('position','velocity','effort')}
            first=a.bundle['effort']
        else:
            d['effort_table_count']=len(a.bundle['effort'])
            d['effort_table_first']=a.bundle['effort'][0]
            d['effort_table_last']=a.bundle['effort'][-1]
            first=a.bundle['effort'][0]['value']
        d['encoded_initial_effort_nm']=[DIRECTION[j]*math.trunc(DIRECTION[j]*v*100)*.01 for j,v in enumerate(first)]
    if a.mock:
        d['background']['hands']='私有Mock明确生成双手零字节协议夹具；不重放实机手背景' if a.hand_background is None else '采用显式手背景，仅在私有Mock使用'
        d.update(endpoint='new_private_physics_endpoint', mock_config=str(a.mock_config.resolve()), support='fixed_base_physics; no hidden holding')
    return d


def new_run(a):
    root = a.output if a.output else ROOT/'artifacts/shm_fieldwork'/f'{time.monotonic_ns()}_{getattr(a, "case", a.action)}_{uuid.uuid4().hex[:6]}'
    root = root.resolve();root.mkdir(parents=True, exist_ok=False)
    return root


def supervise(command, root, *, disappear=False, termination='sigterm', quiet=3, timeout=60,
              peer=None, defer_hup_restore=False, soft_stop=False, soft_stop_timeout=60.0):
    """Only signals Popen handles owned by this invocation, never a discovered PID."""
    root = Path(root)
    with (root/'terminal.log').open('wb') as log:
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=soft_stop)
        probe = None
        previous = {}
        stopping = False
        soft_requested = False
        soft_sent = False
        meta = dict(command=command, pid=child.pid, started_mono_ns=time.monotonic_ns(), termination_sent=None)
        def handler(signum, _frame):
            nonlocal stopping, soft_requested
            if signum == signal.SIGHUP:meta['hangup_received'] = True
            # One terminal hangup can reach the shell and its job more than once.
            # Do not interrupt the bounded cleanup/metadata commit a second time.
            if stopping:return
            if soft_stop and (signum == signal.SIGUSR1 or (signum == signal.SIGINT and not soft_requested)):
                if not soft_requested:
                    soft_requested = True
                    meta['soft_stop_requested'] = True
                    meta['soft_stop_signal'] = signal.Signals(signum).name
                return
            stopping = True
            meta['interrupt_signal'] = signal.Signals(signum).name
            if child.poll() is None:
                child.send_signal(signal.SIGTERM)
            if probe is not None and probe.poll() is None:
                probe.send_signal(signal.SIGTERM)
            raise KeyboardInterrupt
        signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
        if soft_stop:
            signals += (signal.SIGUSR1, signal.SIGQUIT)
        for sig in signals:
            previous[sig] = signal.signal(sig, handler)
        dump(root/'process.json', meta)
        limit = time.monotonic()+timeout
        def stop_owned(process):
            if process.poll() is not None:
                return
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                meta.setdefault('cleanup_hard_killed_pids', []).append(process.pid)
                process.kill();process.wait(timeout=5)
        last_event = None
        last_limit = None
        def progress():
            nonlocal last_event, last_limit
            limits = root/'native/observed_limits.csv'
            if limits.exists():
                with limits.open('rb') as stream:
                    stream.seek(max(0, limits.stat().st_size-2048))
                    tail = stream.read().decode(errors='replace').splitlines()
                if tail and tail[-1] != last_limit and not tail[-1].startswith('receive_mono_ns'):
                    last_limit = tail[-1]
                    print('反馈限制：' + last_limit, flush=True)
            path = root/'native/events.csv'
            if not path.exists():
                return
            with path.open('rb') as stream:
                stream.seek(max(0, path.stat().st_size-2048))
                tail = stream.read().decode(errors='replace').splitlines()
            if tail and tail[-1] != last_event and not tail[-1].startswith('mono_ns'):
                last_event = tail[-1]
                print('阶段：' + last_event, flush=True)
        try:
            if disappear:
                ready = root/'native/kill_ready.txt'
                while child.poll() is None and not ready.exists():
                    if peer is not None and peer.poll() is not None:
                        meta['mock_process_failed']=peer.returncode;stop_owned(child);break
                    if time.monotonic()>limit:
                        meta['supervisor_timeout'] = True;stop_owned(child);break
                    progress();time.sleep(.01)
                if child.poll() is None and ready.exists():
                    probe_command = [command[0], '--passive-observe', '--shm', command[command.index('--shm')+1],
                                     '--duration', str(quiet+.2), '--output', str(root/'probe')]
                    with (root/'probe.log').open('wb') as probe_log:
                        probe = subprocess.Popen(probe_command, stdout=probe_log, stderr=subprocess.STDOUT)
                        deadline = time.monotonic()+1.5
                        while probe.poll() is None and not (root/'probe/passive_ready.txt').exists() and time.monotonic()<deadline:
                            time.sleep(.01)
                        if not (root/'probe/passive_ready.txt').exists() or probe.poll() is not None:
                            child.terminate()
                            meta['probe_failed_before_termination'] = True
                        elif child.poll() is None:
                            meta['termination_sent'] = termination
                            meta['termination_mono_ns'] = time.monotonic_ns()
                            dump(root/'process.json', meta)
                            child.send_signal(signal.SIGKILL if termination == 'sigkill' else signal.SIGTERM)
                        child.wait(timeout=5)
                        probe.wait(timeout=quiet+5)
            while child.poll() is None:
                # Native creates settings.txt after installing its signal handlers.
                if soft_requested and not soft_sent and (root/'native/settings.txt').is_file():
                    child.send_signal(signal.SIGUSR1)
                    soft_sent = True
                    limit = time.monotonic() + soft_stop_timeout
                    dump(root/'process.json', meta)
                    print('收到软停请求，正在复位后失能（再按一次 Ctrl-C 或 Ctrl-\\ 立即失能）', flush=True)
                if peer is not None and peer.poll() is not None:
                    meta['mock_process_failed']=peer.returncode;stop_owned(child);break
                if time.monotonic()>limit:
                    meta['supervisor_timeout'] = True;stop_owned(child);break
                progress();time.sleep(.05)
            child.wait();progress()
        except KeyboardInterrupt:
            stop_owned(child)
            meta['operator_interrupted'] = True
        finally:
            # This is ownership cleanup, not robot recovery. Native TERM attempts disable.
            stopping = True
            try:
                stop_owned(child)
                if probe is not None:
                    stop_owned(probe)
                meta['returncode'] = child.returncode
                meta['ended_mono_ns'] = time.monotonic_ns()
                if probe is not None:
                    meta['probe_returncode'] = probe.returncode
                dump(root/'process.json', meta)
            finally:
                for sig, old in previous.items():
                    # Keep late/repeat HUP from killing the offline report writer.
                    # _native_action restores the caller's HUP handler at its end.
                    signal.signal(sig, signal.SIG_IGN if sig == signal.SIGHUP and defer_hup_restore else old)
    return meta


def records(path, size):
    """Yield complete binary records; caller reports trailing bytes separately."""
    if not path.exists():
        return
    with path.open('rb') as stream:
        while True:
            data = stream.read(size)
            if len(data) < size:
                break
            yield data


def span_update(s, value):
    if not s:
        s.update(first=value, last=value, min=value, max=value)
    else:
        s['last']=value;s['min']=min(s['min'], value);s['max']=max(s['max'], value)


def _span_from_array(arr):
    if len(arr) == 0:
        return {}
    return dict(
        first=arr[0].item(),
        last=arr[-1].item(),
        min=arr.min().item(),
        max=arr.max().item(),
    )


def _unique_sorted_int(col, *, exclude_zero=False):
    if len(col) == 0:
        return []
    if exclude_zero and not _np.any(col):
        return []
    compact = col if len(col) <= 1 else _np.r_[col[:1], col[1:][col[1:] != col[:-1]]]
    vals = _np.unique(compact)
    if exclude_zero:
        return [int(x) for x in vals if x != 0]
    return [int(x) for x in vals]


def _affine_span(raw_span, d, off):
    if not raw_span:
        return {}
    f_first = float(d * raw_span['first'] * 1e-6 + off)
    f_last = float(d * raw_span['last'] * 1e-6 + off)
    a = float(d * raw_span['min'] * 1e-6 + off)
    b = float(d * raw_span['max'] * 1e-6 + off)
    return dict(first=f_first, last=f_last, min=min(a, b), max=max(a, b))


def summarize_feedback(folder, codec=None):
    folder = Path(folder)
    direction = (codec or {}).get('direction', DIRECTION)
    offset_rad = (codec or {}).get('offset_rad', OFFSET)
    path = folder/'feedback.bin'
    size = path.stat().st_size if path.exists() else 0
    result = dict(count=0, trailing_bytes=size % 704 if path.exists() else 0,
                  receive_gap_ns={}, source_step_ms={}, repeated_stamp=0, backward_stamp=0,
                  repeated_counter=0, counter_gap=0, backward_counter=0,
                  joints=[dict(id=j, raw_position={}, q_rad={} if 12 <= j <= 25 else None,
                               raw_velocity={}, raw_torque={}, statuses=[], modes=[], errors=[]) for j in range(35)],
                  hands=[dict(hand=h, errors=[], global_statuses=[], items=[dict(position={}, velocity={}, current={}, statuses=[]) for _ in range(6)]) for h in range(2)])
    count = size // 704
    if count == 0:
        return result
    if _np is not None and FEEDBACK_RECORD_DTYPE is not None:
        recs = _np.fromfile(path, dtype=FEEDBACK_RECORD_DTYPE, count=count)
        result['count'] = int(count)
        if count > 1:
            recv = recs['receive_mono_ns'].astype(_np.int64)
            result['receive_gap_ns'] = _span_from_array(_np.diff(recv))
            stamps = recs['frame']['timestamp']
            if int(stamps.max()) <= _np.iinfo(_np.int64).max:
                result['source_step_ms'] = _span_from_array(_np.diff(stamps.astype(_np.int64)))
            else:
                steps = [int(b) - int(a) for a, b in zip(stamps[:-1], stamps[1:])]
                result['source_step_ms'] = dict(first=steps[0], last=steps[-1], min=min(steps), max=max(steps))
            result['repeated_stamp'] = int(_np.sum(stamps[1:] == stamps[:-1]))
            result['backward_stamp'] = int(_np.sum(stamps[1:] < stamps[:-1]))
            counters = recs['frame']['frame_counter']
            result['repeated_counter'] = int(_np.sum(counters[1:] == counters[:-1]))
            result['backward_counter'] = int(_np.sum(counters[1:] < counters[:-1]))
            inc = counters[1:] > counters[:-1]
            if _np.any(inc):
                result['counter_gap'] = int(_np.sum(counters[1:][inc] - counters[:-1][inc] - _np.uint64(1)))
        joints = recs['frame']['joints']
        pos_all = joints['position']
        vel_all = joints['velocity']
        trq_all = joints['torque']
        sw_all = joints['status_word']
        md_all = joints['mode']
        err_all = joints['error_code']
        pos_min, pos_max = pos_all.min(axis=0), pos_all.max(axis=0)
        vel_min, vel_max = vel_all.min(axis=0), vel_all.max(axis=0)
        trq_min, trq_max = trq_all.min(axis=0), trq_all.max(axis=0)
        has_joint_err = bool(_np.any(err_all))
        for j, s in enumerate(result['joints']):
            raw_pos = dict(first=pos_all[0, j].item(), last=pos_all[-1, j].item(), min=pos_min[j].item(), max=pos_max[j].item())
            s['raw_position'] = raw_pos
            s['raw_velocity'] = dict(first=vel_all[0, j].item(), last=vel_all[-1, j].item(), min=vel_min[j].item(), max=vel_max[j].item())
            s['raw_torque'] = dict(first=trq_all[0, j].item(), last=trq_all[-1, j].item(), min=trq_min[j].item(), max=trq_max[j].item())
            if 12 <= j <= 25:
                s['q_rad'] = _affine_span(raw_pos, direction[j - 12], offset_rad[j - 12])
            s['statuses'] = _unique_sorted_int(sw_all[:, j])
            s['modes'] = _unique_sorted_int(md_all[:, j])
            s['errors'] = _unique_sorted_int(err_all[:, j], exclude_zero=True) if has_joint_err else []
        hands = recs['frame']['hands']
        hj_all = hands['joints']
        hp_min, hp_max = hj_all['position'].min(axis=0), hj_all['position'].max(axis=0)
        hv_min, hv_max = hj_all['velocity'].min(axis=0), hj_all['velocity'].max(axis=0)
        hc_min, hc_max = hj_all['current'].min(axis=0), hj_all['current'].max(axis=0)
        for h, s in enumerate(result['hands']):
            s['global_statuses'] = _unique_sorted_int(hands['global_status'][:, h])
            s['errors'] = _unique_sorted_int(hands['error_code'][:, h], exclude_zero=True)
            hj = hj_all[:, h, :]
            for item_idx, item in enumerate(s['items']):
                item['position'] = dict(first=hj['position'][0, item_idx].item(), last=hj['position'][-1, item_idx].item(), min=hp_min[h, item_idx].item(), max=hp_max[h, item_idx].item())
                item['velocity'] = dict(first=hj['velocity'][0, item_idx].item(), last=hj['velocity'][-1, item_idx].item(), min=hv_min[h, item_idx].item(), max=hv_max[h, item_idx].item())
                item['current'] = dict(first=hj['current'][0, item_idx].item(), last=hj['current'][-1, item_idx].item(), min=hc_min[h, item_idx].item(), max=hc_max[h, item_idx].item())
                item['statuses'] = _unique_sorted_int(hj['status'][:, item_idx])
        return result
    last = None
    status_sets = [set() for _ in range(35)];mode_sets=[set() for _ in range(35)];errors=[set() for _ in range(35)]
    hands = [[set(), set(), [set() for _ in range(6)]] for _ in range(2)]
    for data in records(path, 704):
        received, counter = struct.unpack_from('<qQ', data)
        stamp = struct.unpack_from('<Q', data, 8+680)[0]
        result['count'] += 1
        if last:
            span_update(result['receive_gap_ns'], received-last[0]);span_update(result['source_step_ms'], stamp-last[2])
            result['repeated_stamp'] += stamp == last[2];result['backward_stamp'] += stamp < last[2]
            result['repeated_counter'] += counter == last[1];result['backward_counter'] += counter < last[1]
            if counter > last[1]+1:
                result['counter_gap'] += counter-last[1]-1
        last = (received, counter, stamp)
        for j, s in enumerate(result['joints']):
            status, q, v, torque, mode, error = struct.unpack_from('<HiihhH', data, 8+12+j*16)
            span_update(s['raw_position'], q);span_update(s['raw_velocity'], v);span_update(s['raw_torque'], torque)
            if 12 <= j <= 25:
                span_update(s['q_rad'], direction[j-12]*q*1e-6+offset_rad[j-12])
            status_sets[j].add(status);mode_sets[j].add(mode)
            if error:
                errors[j].add(error)
        for h in range(2):
            offset = 8+572+h*52
            global_status, error = struct.unpack_from('<HH', data, offset)
            hands[h][0].add(global_status)
            if error:
                hands[h][1].add(error)
            for j, item in enumerate(result['hands'][h]['items']):
                q, v, current, status = struct.unpack_from('<hhhH', data, offset+4+j*8)
                span_update(item['position'], q);span_update(item['velocity'], v);span_update(item['current'], current);hands[h][2][j].add(status)
    for j, s in enumerate(result['joints']):
        s.update(statuses=sorted(status_sets[j]), modes=sorted(mode_sets[j]), errors=sorted(errors[j]))
    for h, s in enumerate(result['hands']):
        s.update(global_statuses=sorted(hands[h][0]), errors=sorted(hands[h][1]))
        for j, item in enumerate(s['items']):
            item['statuses'] = sorted(hands[h][2][j])
    return result


def summarize_commands(folder, codec=None):
    folder = Path(folder)
    direction = (codec or {}).get('direction', DIRECTION)
    offset_rad = (codec or {}).get('offset_rad', OFFSET)
    path = folder/'commands.bin'
    size = path.stat().st_size if path.exists() else 0
    result = dict(count=0, trailing_bytes=size % 752 if path.exists() else 0,
                  send_gap_ns={}, arms=[dict(id=j+12, q_rad={}) for j in range(14)])
    count = size // 752
    if count == 0:
        return result
    if _np is not None and COMMAND_RECORD_DTYPE is not None:
        recs = _np.fromfile(path, dtype=COMMAND_RECORD_DTYPE, count=count)
        result['count'] = int(count)
        if count > 1:
            result['send_gap_ns'] = _span_from_array(_np.diff(recs['send_mono_ns'].astype(_np.int64)))
        arm_pos = recs['frame']['joints']['position'][:, 12:26]
        arm_min, arm_max = arm_pos.min(axis=0), arm_pos.max(axis=0)
        for j, arm in enumerate(result['arms']):
            raw_span = dict(first=arm_pos[0, j].item(), last=arm_pos[-1, j].item(), min=arm_min[j].item(), max=arm_max[j].item())
            arm['q_rad'] = _affine_span(raw_span, direction[j], offset_rad[j])
        return result
    last = None
    for data in records(path, 752):
        at = struct.unpack_from('<q', data)[0];result['count'] += 1
        if last is not None:
            span_update(result['send_gap_ns'], at-last)
        last = at
        for j, arm in enumerate(result['arms']):
            q = struct.unpack_from('<i', data, 8+12+(j+12)*18+2)[0]
            span_update(arm['q_rad'], direction[j]*q*1e-6+offset_rad[j])
    return result


def timing_summary(folder):
    folder = Path(folder)
    result = dict(events=[], clock_pairs=[], streaming_send_gap_ns={})
    event_path = folder/'events.csv'
    if event_path.exists():
        with event_path.open() as stream:
            result['events'] = [r for r in csv.DictReader(stream) if r.get('mono_ns') and r.get('event')]
    path = folder/'clock_pairs.csv'
    if path.exists():
        with path.open() as stream:
            result['clock_pairs'] = [r for r in csv.DictReader(stream) if all(r.get(k) for k in
                ('mono_before_ns', 'wall_ns', 'mono_after_ns', 'offset_ns', 'clock_segment', 'jump_detected'))]
    cmd_bin = folder/'commands.bin'
    if _np is not None and COMMAND_RECORD_DTYPE is not None and cmd_bin.is_file() and result['events']:
        count = cmd_bin.stat().st_size // 752
        if count > 1:
            sends = _np.fromfile(cmd_bin, dtype=COMMAND_RECORD_DTYPE, count=count)['send_mono_ns'].astype(_np.int64)
            intervals = []
            start_ns = None
            for ev in result['events']:
                p = ev.get('phase')
                t = int(ev['mono_ns'])
                if p == '3' and start_ns is None:
                    start_ns = t
                elif p != '3' and start_ns is not None:
                    intervals.append((start_ns, t))
                    start_ns = None
            if start_ns is not None:
                intervals.append((start_ns, _np.iinfo(_np.int64).max))
            gaps = []
            for lo, hi in intervals:
                i0 = int(_np.searchsorted(sends, lo, side='left'))
                i1 = int(_np.searchsorted(sends, hi, side='left'))
                if i1 - i0 > 1:
                    gaps.append(_np.diff(sends[i0:i1]))
            if gaps:
                result['streaming_send_gap_ns'] = _span_from_array(_np.concatenate(gaps))
        return result
    path = folder/'commands.csv';last = None
    if path.exists():
        with path.open() as stream:
            for r in csv.DictReader(stream):
                if not r.get('send_mono_ns') or r.get('phase') != '3':
                    last = None;continue
                at = int(r['send_mono_ns'])
                if last is not None:
                    span_update(result['streaming_send_gap_ns'], at-last)
                last = at
    return result


def snapshot_summary(folder):
    path = folder/'status.bin'
    if not path.exists():
        return None
    data = path.read_bytes()
    if len(data) != 696:
        raise ValueError('状态快照长度错误')
    result = dict(source_timestamp_ms=struct.unpack_from('<Q', data, 680)[0],
                  frame_counter=struct.unpack_from('<Q', data)[0], joints=[])
    for j in range(35):
        word, q, v, effort, mode, error = struct.unpack_from('<HiihhH', data, 12+j*16)
        result['joints'].append(dict(id=j, raw_position=q, raw_velocity=v, raw_torque=effort,
                                     status=word, mode=mode, error=error))
    return result


def summarize_limits(folder):
    result = dict(count=0, actions={}, last_by_joint_field={})
    path = Path(folder)/'observed_limits.csv'
    if path.exists():
        with path.open() as stream:
            for row in csv.DictReader(stream):
                result['count'] += 1
                action = row['action']
                result['actions'][action] = result['actions'].get(action, 0)+1
                result['last_by_joint_field'][row['joint_id']+':'+row['field']] = row
    return result


def report(root):
    root = Path(root).resolve()
    case_file = root/'case.json'
    case = json.loads(case_file.read_text()) if case_file.exists() else {}
    process = json.loads((root/'process.json').read_text()) if (root/'process.json').exists() else {}
    folder = root/'native' if (root/'native').is_dir() else root
    native = text_result(folder/'result.txt')
    feedback = summarize_feedback(folder, case.get('codec'));commands = summarize_commands(folder, case.get('codec'))
    timing = timing_summary(folder)
    limits = summarize_limits(folder)
    snapshot = snapshot_summary(folder)
    observations = [json.loads(row) for row in (root/'observations.jsonl').read_text().splitlines()] if (root/'observations.jsonl').exists() else []
    planned_kill = process.get('termination_sent') == 'sigkill' and process.get('returncode') == -signal.SIGKILL
    planned_term = process.get('termination_sent') == 'sigterm' and native and native.get('reason') == 'operator_interrupt'
    probe_result = text_result(root/'probe/result.txt')
    experiment_completed = bool((planned_kill or planned_term) and probe_result and probe_result.get('execution_completed') == '1' and process.get('probe_returncode') == 0)
    mock_exit = json.loads((root/'mock/exit.json').read_text()) if (root/'mock/exit.json').exists() else None
    setup = case.get('case') in ('prepare', 'activate')
    completed = (bool(native and native.get('execution_completed') == '1' and process.get('returncode', 0) == 0) or experiment_completed) and not process.get('supervisor_timeout', False) and 'mock_process_failed' not in process
    if mock_exit is not None and (mock_exit['returncode']!=0 or mock_exit.get('cleanup_errors')):
        completed=False
    if setup:
        completed = process.get('returncode') == 0
    reasons = []
    if mock_exit is not None and (mock_exit['returncode']!=0 or mock_exit.get('cleanup_errors')):
        reasons.append(f'私有物理Mock退出码={mock_exit["returncode"]}；资源回收错误={mock_exit.get("cleanup_errors", [])}，本例未完成。')
    if 'mock_process_failed' in process:
        reasons.append('自有物理Mock异常退出，已结束发送器；未重启或回退到实机。')
    if process.get('supervisor_timeout') or process.get('cleanup_hard_killed_pids'):
        reasons.append('监督超时/有界退出失败：已终止自有进程，物理输出状态未知；未恢复或重试。')
    if planned_kill:
        reasons.append('计划内SIGKILL：发送器无结束封存；仅恢复完整原始前缀，不补发失能，物理停止须现场处理。')
    elif not native and not setup:
        reasons.append('缺少原生结束结果：未封存，只恢复完整前缀，不允许resume。')
    if native and native.get('recording_stopped') == '1':
        reasons.append('原生记录中途停录，保留前缀；执行结果与记录完整性分开。')
    if feedback['trailing_bytes'] or commands['trailing_bytes']:
        reasons.append('二进制尾部存在不完整记录，未作为完整帧分析。')
    if setup:
        reasons.append('此例是接管/握手步骤，不含运动或持续反馈；步骤退出成功不作物理停止证明。')
    result = dict(schema='casbot_shm_field_case_v1', case=case, process=process, native=native,
                  execution_completed=completed, conclusion=observations[-1]['conclusion'] if observations else ('inconclusive' if process or native else 'not_run'),
                  observation_recorded=bool(observations), observations=observations,
                  feedback=feedback, commands=commands, mock_exit=mock_exit, timing=timing, snapshot=snapshot, observed_limits=limits, recording_notes=reasons,
                  probe=summarize_feedback(root/'probe', case.get('codec')) if (root/'probe').is_dir() else None,
                  physical_stop_observed='unknown')
    # Human observations establish only what their text actually says, not an automatic pass.
    dump(root/'result.json', result)
    lines = ['# SHM现场用例报告', '', f'- 用例：{case.get("case", "旧原型/未标记")} {case.get("title", "")}',
             f'- 软件执行完成：{completed}；原因：{native.get("reason") if native else ("准备步骤，见退出码/日志" if setup else "无原生封存")}',
             f'- 人工观察：{"已记录，见下文" if observations else "尚未记录"}；结论不由执行返回码自动生成。',
             f'- 成功命令：{commands["count"]}；反馈记录：{feedback["count"]}',
             f'- 最大成功发送间隔（含计划静默）：{commands["send_gap_ns"].get("max", "无")} ns',
             f'- STREAM阶段最大成功发送间隔：{timing["streaming_send_gap_ns"].get("max", "无")} ns',
             f'- stamp重复/倒退：{feedback["repeated_stamp"]}/{feedback["backward_stamp"]}；帧号缺口：{feedback["counter_gap"]}',
             '- stamp为主站毫秒墙钟；接收时刻另存，不作驱动采样时刻或单向执行延迟。', '']
    if limits['count']:
        lines += [f'- 反馈限制事件：{limits["count"]}条，动作计数{limits["actions"]}；完整轴/值/阈值见native/observed_limits.csv。']
    if timing['clock_pairs']:
        lines += [f'- 时钟配对：{len(timing["clock_pairs"])}次；分段：{sorted({r["clock_segment"] for r in timing["clock_pairs"]})}；原始stamp量化1ms。']
    if timing['events']:
        lines += ['', '## 阶段与事件', '']
        lines += [f'- {r["mono_ns"]} ns：phase={r["phase"]} {r["event"]}' for r in timing['events']]
    if case.get('case') == 'C06':
        lines += [f'- 计划终止：{process.get("termination_sent")}；探针退出码：{process.get("probe_returncode")}',
                  '- 探针是被动快照，重复数据仍保留，不证明主站/电机持续更新。']
    lines += reasons + ['', '## 双臂指令与反馈', '', '|ID|关节|指令范围 rad|反馈范围 rad|末次反馈 rad|错误码|', '|---|---|---|---|---|---|']
    def six(value):
        return f'{value:.6f}'.replace('-0.000000', '0.000000')
    def bounds(s):
        return f'{six(s["min"])}～{six(s["max"])}' if s else '无记录'
    def raw_span(s):
        return f'{s["first"]} → {s["last"]} [{s["min"]}, {s["max"]}]' if s else '无记录'
    def values(items, transform=str):
        return ' / '.join(map(transform, items)) if items else '无'

    for j in range(14):
        arm = feedback['joints'][j+12];q = arm['q_rad']
        lines.append(f'|{j+12}|{("左" if j<7 else "右")+PARTS[j%7]}|{bounds(commands["arms"][j]["q_rad"])}|{bounds(q)}|{six(q["last"]) if q else "无"}|{values(arm["errors"])}|')
    lines += ['', '## 全槽背景变化（原始刻度）', '', '|槽|位置首/末/范围|状态字集合|模式|错误码|', '|---|---|---|---|---|']
    for s in feedback['joints']:
        lines.append(f'|{s["id"]}|{raw_span(s["raw_position"])}|{values(s["statuses"], hex)}|{values(s["modes"])}|{values(s["errors"])}|')
    lines += ['', '## 双手原始反馈', '']
    for hand in feedback['hands']:
        lines += [f'- 手{hand["hand"]}：global_status={values(hand["global_statuses"])}，error={values(hand["errors"])}', '',
                  '|原始项|位置首→末 [min,max]|速度首→末 [min,max]|电流首→末 [min,max]|状态|',
                  '|---|---|---|---|---|']
        for j, item in enumerate(hand['items']):
            lines.append(f'|{j}|{raw_span(item["position"])}|{raw_span(item["velocity"])}|{raw_span(item["current"])}|{values(item["statuses"])}|')
        lines.append('')
    if not feedback['count'] and snapshot:
        lines += ['', '## 起点/被动快照（非持续观测）', '', f'- source_timestamp_ms={snapshot["source_timestamp_ms"]}，frame_counter={snapshot["frame_counter"]}']
        for row in snapshot['joints']:
            lines.append(f'- ID{row["id"]}：position_raw={row["raw_position"]}，velocity_raw={row["raw_velocity"]}，torque_raw={row["raw_torque"]}，status={hex(row["status"])}，mode={row["mode"]}，error={row["error"]}')
    if not feedback['count'] and (folder/'status.bin').exists():
        lines += ['', '此例仅有单次status.bin快照，不含持续反馈；不能据此报告更新率或运行期间行为。']
    lines += ['', '## 用户观察', '']
    for row in observations:
        lines += [f'- [{row["phase"]}] {row["conclusion"]}：{row["text"]}']
    if not observations:
        lines.append('待现场观察后用note登记。')
    (root/'report.md').write_text('\n'.join(lines)+'\n')
    print(f'报告：{root}/report.md\n记录：{root}\n软件执行完成={completed}；人工观察={bool(observations)}')
    return result


def run_body_hold(a):
    from .model_runtime import model_for
    from .pipeline.native import maybe_mock_context, native_execute_once, native_snapshot
    from .pipeline.preflight import check_hardware_environment, check_native_binaries
    from .pipeline.robot import load_robot_config
    from .pipeline.session import DEFAULT_OUTPUT_ROOT
    from .pipeline.takeover import ensure_ready_channel
    from .shm_ready import ready_position
    from .shm_reference import save_bundle
    from .staged_motion import Motion
    from .stages.dynamics.common import compose_stage_reference

    if not 0 < a.duration <= 3600 or not 0 < a.scale <= 1:
        raise ValueError('站立冒烟需要 0 < duration <= 3600、0 < scale <= 1')
    robot = load_robot_config(hold_body=True)
    for fields in robot['non_arm_hold']['slots'].values():
        fields['kp'] *= a.scale
        fields['kd'] *= a.scale
    if not a.execute:
        print(f'仅预览：15 个非臂槽以 {a.scale:g} 倍增益保持，双臂进入准备位后停留 {a.duration:g} s')
        return 0

    endpoint = '/shm_mc'
    binary = check_native_binaries(executor=a.executor, mock=a.mock, endpoint=endpoint)
    root = new_run(a)
    print(f'站立冒烟记录：{root}', flush=True)
    if not a.mock:
        check_hardware_environment(endpoint=endpoint)
        channel = ensure_ready_channel(DEFAULT_OUTPUT_ROOT / 'state', binary, endpoint=endpoint)
    with maybe_mock_context(a.mock, root, binary, robot, mock_config=a.mock_config) as mock:
        endpoint = mock.endpoint if mock else channel['endpoint']
        background = mock.background if mock else channel['hand_background']
        peer = mock.process if mock else None
        q = native_snapshot(binary, endpoint, robot, root / 'snapshot', peer=peer)
        ready = ready_position()
        motion = Motion('hold_body', 'smoke', ready)
        motion.add(a.duration, ready, label='hold')
        engine, _ = model_for(robot)
        bundle, *_ = compose_stage_reference(motion, q, engine, robot, ready_pose=True, feedforward_mode='none')
        reference = save_bundle(root / 'plan', bundle)
        code, result = native_execute_once(binary, endpoint, robot, reference, background,
                                           root / 'capture', duration_s=bundle['knots_ns'][-1]*1e-9, peer=peer)
    native = result.get('native') or {}
    print(f"结束原因 {native.get('reason')}，失能应答 {native.get('disable_ack')}")
    return code


def run_action(a):
    if a.action == 'hold-body':
        return run_body_hold(a)
    if a.action in ('plan','fields'):
        from .shm_reference import snapshot_positions,jog_bundle,fields_bundle,save_bundle
        if a.action=='plan' and a.relative:
            if a.snapshot:raise ValueError('relative不同时使用旧快照起点')
            q=[0.]*14
        elif a.snapshot:q=snapshot_positions(a.snapshot)
        else:raise ValueError('绝对参考/五字段需要--snapshot；相对jog可用--relative')
        bundle=jog_bundle(q,a.joint_id,a.amplitude,a.segment_s,a.kp,a.kd,relative=a.relative) if a.action=='plan' else fields_bundle(
            q,a.joint_id,position=a.position,velocity=a.velocity,effort=a.effort,kp=a.kp,kd=a.kd,duration=a.duration)
        print('离线参考：'+str(save_bundle(a.output,bundle)));return 0
    if a.action=='execute':
        a.bundle=json.loads(a.reference.read_text())
        if a.bundle.get('schema') not in ('casbot_shm_reference_v1','casbot_shm_fields_v1'):raise ValueError('Unsupported SHM reference bundle')
        a.duration=(a.bundle['duration_ns'] if a.bundle['schema']=='casbot_shm_fields_v1' else a.bundle['knots_ns'][-1])*1e-9
    if hasattr(a, 'urdf'):
        if getattr(a, 'mock', False) and a.urdf is None:
            config=json.loads(a.mock_config.read_text());a.urdf=(a.mock_config.resolve().parent/config['urdf']).resolve()
        a.urdf=a.urdf or URDF
    validate(a)
    if a.action == 'list':
        for key, (title, behavior) in CASES.items():
            print(f'{key}  {title}：{behavior}')
        print('\n顺序：inspect → prepare --execute → activate --execute → observe → C01/C02 → C03/C04 → C05 → C06。')
        print('主动用例run默认预览；追加--execute才发命令。C03只测双臂12–25。C07/C08尚无主动入口。')
        return 0
    if a.action == 'note':
        if not a.run.is_dir():
            raise ValueError('记录目录不存在')
        row = dict(recorded_wall_ns=time.time_ns(), recorded_mono_ns=time.monotonic_ns(), phase=a.phase,
                   conclusion=a.conclusion, text=a.text)
        with (a.run/'observations.jsonl').open('a') as stream:
            stream.write(json.dumps(row, ensure_ascii=False)+'\n')
        with (a.run/'observations.md').open('a') as stream:
            stream.write(f'\n- [{a.phase}] {a.conclusion}：{a.text}\n')
        report(a.run);return 0
    if a.action == 'report':
        report(a.run);return 0
    if a.action == 'status':
        for file in sorted(a.folder.rglob('case.json')):
            case = json.loads(file.read_text());out = file.parent/'result.json'
            result = json.loads(out.read_text()) if out.exists() else {}
            print(f'{case.get("case", "setup")} completed={result.get("execution_completed", False)} '
                  f'observation={result.get("observation_recorded", False)} {file.parent}')
        return 0
    if a.action in ('run','execute'):
        recipe = preview(a)
        print(json.dumps(recipe, ensure_ascii=False, indent=2))
        if not a.execute:
            print('仅预览：未连接SHM、未创建运行目录、未发命令。');return 0
    elif a.action in ('prepare', 'activate') and not a.execute:
        print('仅预览：' + ('停独立服务→厂家stop→只启主站，不发关节命令。' if a.action == 'prepare' else '仅对全新主站post一次启动握手；不用于解卡/恢复。'))
        return 0
    root = new_run(a)
    case = recipe if a.action in ('run','execute') else dict(case='C00' if a.action in ('inspect', 'snapshot', 'observe', 'probe') else a.action,
                                               title=a.action, endpoint=a.endpoint)
    case.update(schema='casbot_shm_field_case_v1', created_wall_ns=time.time_ns(), created_mono_ns=time.monotonic_ns(),
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                support=('not_applicable_private_endpoint' if a.endpoint.startswith('/casbot_calib_test_') else '用户已确认固定支撑，现场负责继电器与试验顺序') if a.action in ('run','execute') else None,
                codec=dict(profile='orin_35_compat_20261113', direction=DIRECTION, offset_rad=OFFSET,
                           arm_names=NAMES, position_rad_per_lsb=1e-6, velocity_rad_s_per_lsb=1e-6, torque_nm_per_lsb=.01))
    dump(root/'case.json', case)
    print(f'本例记录：{root}', flush=True)
    if getattr(a, 'mock', False):
        from .shm_mock import SHMMock
        try:
            with SHMMock(root/'mock',a.executor,a.mock_config,urdf=a.urdf) as mock:
                a.endpoint=mock.endpoint
                if a.action in ('run','execute') and a.hand_background is None:a.hand_background=mock.background
                case.update(endpoint=a.endpoint,kind='private_shm_physics',support='fixed_base_with_free_unpowered_joints',
                            hand_background=str(getattr(a,'hand_background',None)) if getattr(a,'hand_background',None) else None)
                dump(root/'case.json',case)
                return _native_action(a,root,peer=mock.process,peer_close=mock.close)
        except (OSError,RuntimeError,ValueError,subprocess.SubprocessError) as error:
            if not (root/'process.json').exists():dump(root/'process.json',dict(returncode=2,mock_startup_error=str(error)))
            report(root)
            print('私有物理Mock失败：'+str(error),file=sys.stderr)
            return 2
    return _native_action(a,root)



def prepare_subprocess(command):
    child = subprocess.Popen(command)
    meta = dict(command=command, pid=child.pid, operator_interrupted=False)
    try:meta['returncode'] = child.wait()
    except KeyboardInterrupt:
        meta['operator_interrupted'] = True
        if child.poll() is None:child.send_signal(signal.SIGINT)
        try:child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            child.kill();child.wait(timeout=3);meta['cleanup_timeout'] = True
        meta['returncode'] = 130
    return meta


def _native_action(a,root,peer=None,peer_close=None):
    # A lost terminal must not interrupt the offline report after native cleanup.
    previous_hup = signal.getsignal(signal.SIGHUP)
    try:return _native_action_impl(a,root,peer=peer,peer_close=peer_close)
    finally:signal.signal(signal.SIGHUP, previous_hup)


def _native_action_impl(a,root,peer=None,peer_close=None):
    if a.action == 'prepare':
        command = [sys.executable, '-m', 'casbot_calib.shm_takeover']
        command += ['--start-only', str(a.start_only.resolve())] if a.start_only else [str(root/'takeover')]
        # Vendor stop may ask interactively: retain stdin/terminal and log via script.
        meta = prepare_subprocess(command)
        code = meta['returncode']
        dump(root/'process.json', meta)
        if code == 0:print('主站未自动握手。查看takeover/station.log，确认后单独activate --execute。')
        else:print('准备未完成；查看takeover/prepare_result.json及diagnosis.txt，不继续activate或重试运动。')
        report(root);return code
    if a.action == 'activate':
        command = [str(a.executor.resolve().with_name('shm_channel_start')), '--fresh-master', a.endpoint]
    else:
        mode = {'inspect':'--inspect', 'snapshot':'--snapshot', 'observe':'--observe', 'probe':'--passive-observe'}.get(a.action)
        if a.action in ('run','execute'):
            mode = '--inspect' if a.case == 'C00' else '--execute'
        command = [str(a.executor.resolve()), mode, '--shm', a.endpoint, '--output', str(root/'native')]
        if mode in ('--snapshot', '--execute'):
            command += ['--urdf', str(a.urdf.resolve())]
        if mode in ('--observe', '--passive-observe'):
            command += ['--duration', str(a.duration)]
        if mode == '--execute':
            background = root/'hand_background_source.bin'
            background.write_bytes(a.hand_background.read_bytes())
            if a.action=='execute':
                command += ['--reference',str(a.reference.resolve()),'--hand-background',str(background)]
            else:
                command += ['--case', a.case, '--joint-id', str(a.joint_id), '--hand-background', str(background),
                            '--amplitude', str(a.amplitude), '--segment-s', str(a.segment_s), '--kp', str(a.kp), '--kd', str(a.kd),
                            '--duration', str(a.duration), '--quiet-s', str(a.quiet_s), '--other-arm', a.other_arm]
    try:
        meta = supervise(command, root, disappear=a.action == 'run' and a.case == 'C06',
                         termination=getattr(a, 'termination', 'sigterm'), quiet=getattr(a, 'quiet_s', 3),
                         peer=peer, defer_hup_restore=True, timeout=((3*a.segment_s+.5 if a.case in ('C03', 'C04') else a.duration+a.quiet_s)+20)
                         if a.action in ('run','execute') else getattr(a, 'duration', 3)+10)
    except OSError as error:
        dump(root/'process.json', dict(command=command, spawn_error=str(error), returncode=127))
        report(root)
        raise ValueError(f'启动失败：{error}；先用--with-core构建安装shm_executor') from error
    if a.action == 'activate':
        print((root/'terminal.log').read_text());report(root);return meta['returncode']
    if peer_close:peer_close()
    result = report(root)
    if peer is not None and (peer.returncode != 0 or result['mock_exit'].get('cleanup_errors')):
        return meta['returncode'] or 1
    if a.action == 'inspect' or (a.action == 'run' and a.case == 'C00'):
        print(f'手部背景文件：{root}/native/command.bin（只用于冻结手部，非整帧回放）')
    if a.action == 'run' and a.case == 'C06' and result['execution_completed']:
        if a.termination == 'sigkill':
            print('发送器已硬杀，未补发失能；'+('自有Mock将在观察后退出。' if getattr(a,'mock',False) else '现场用继电器处理持续输出。'))
        return 0
    return meta['returncode'] if meta['returncode'] >= 0 else 128-meta['returncode']


def main(argv=None):
    values=list(sys.argv[1:] if argv is None else argv)
    if values and values[0] in ('a-evidence','gui'):
        try:
            if values[0]=='gui':from .shm_gui import main as implementation
            else:from .shm_a_evidence import main as implementation
        except ImportError as error:
            print('GUI/A分析需要数值Python环境：'+str(error),file=sys.stderr);return 2
        return implementation(values[1:])
    a = parser().parse_args(values)
    try:
        return run_action(a)
    except (ValueError, OSError, KeyError, RuntimeError, ET.ParseError, subprocess.SubprocessError, TypeError, IndexError, OverflowError) as error:
        print(f'错误：{error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
