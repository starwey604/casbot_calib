"""Read-only field evidence analysis. No ROS, process launch or motion imports."""
from collections import Counter
import json
import math
from pathlib import Path


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def read_json(path, default=None):
    return json.loads(Path(path).read_text(encoding='utf-8')) if Path(path).exists() else default


def json_lines(path, problems=None):
    """Only complete records are evidence; an unfinished tail is reported by caller."""
    if Path(path).exists():
        with Path(path).open(encoding='utf-8') as stream:
            for line in stream:
                try:
                    if not line.endswith('\n'):
                        raise ValueError(f'Unfinished JSONL record: {path}')
                    yield json.loads(line)
                except ValueError as exc:
                    if problems is None:
                        raise
                    problems.append(str(exc))


def finite_array(values, size):
    return (isinstance(values, list) and len(values) == size and
            all(type(v) in (int, float) and math.isfinite(v) for v in values))


def position_errors(packet, expected):
    msg = packet.get('message', {})
    names = msg.get('name', [])
    if (not isinstance(names, list) or not names or
            any(not isinstance(n, str) or not n for n in names) or len(set(names)) != len(names)):
        return ['invalid_or_duplicate_names']
    errors = []
    if not finite_array(msg.get('position'), len(names)):
        errors.append('unavailable_position')
    if not set(expected) <= set(names):
        errors.append('missing_arm_axes')
    return errors


def receive_rates(times):
    delta = [b - a for a, b in zip(times, times[1:])]
    if not delta:
        return dict(count=len(times), hz=None, interval_ms=None, monotonic=True)
    if min(delta) <= 0:
        return dict(count=len(times), hz=None, interval_ms=None, monotonic=False)
    ordered = sorted(delta)
    def percentile(fraction):
        offset = (len(ordered) - 1) * fraction
        lower = int(offset)
        upper = min(lower + 1, len(ordered) - 1)
        return (ordered[lower] + (offset - lower) * (ordered[upper] - ordered[lower])) / 1e6
    return dict(count=len(times), hz=(len(times) - 1) * 1e9 / (times[-1] - times[0]), monotonic=True,
                interval_ms=dict(p50=percentile(.5), p95=percentile(.95), p99=percentile(.99), maximum=max(delta)/1e6))


class ConnectionMonitor:
    """Actual polling-time detection, distinct from callback reception and sampling."""
    def __init__(self, expected, timeout_ns, emit):
        self.expected = expected
        self.timeout_ns = timeout_ns
        self.emit = emit
        self.last_valid_ns = None
        self.stale = False
        self.started_ns = None
        self.waiting_reported = False

    def packet(self, packet, observed_ns):
        if packet.get('channel') != 'joint' or position_errors(packet, self.expected):
            return
        received = packet['receive_ns']
        if self.last_valid_ns is None:
            self.emit(dict(event='first_valid_feedback', observed_ns=observed_ns, receive_ns=received))
        elif self.stale:
            self.emit(dict(event='valid_feedback_resumed', observed_ns=observed_ns, receive_ns=received,
                           previous_receive_ns=self.last_valid_ns, gap_ns=received-self.last_valid_ns))
        self.stale = False
        self.last_valid_ns = received

    def tick(self, observed_ns):
        if self.started_ns is None:
            self.started_ns = observed_ns
        if self.last_valid_ns is None:
            if not self.waiting_reported and observed_ns-self.started_ns > self.timeout_ns:
                self.waiting_reported = True
                self.emit(dict(event='initial_feedback_timeout', observed_ns=observed_ns,
                               monitor_started_ns=self.started_ns))
            return
        if self.last_valid_ns is not None and not self.stale and observed_ns-self.last_valid_ns > self.timeout_ns:
            self.stale = True
            self.emit(dict(event='valid_feedback_timeout', observed_ns=observed_ns,
                           last_valid_receive_ns=self.last_valid_ns,
                           receive_age_ns=observed_ns-self.last_valid_ns))


def analyse_capture(root, session):
    root = Path(root)
    config = session['config']
    capture_root = root / 'capture'
    raw = capture_root / 'raw'
    channels = {}
    modes = []
    previous_mode = {}
    evidence_errors = []
    for packet in json_lines(raw / 'packets.jsonl', evidence_errors):
        channel = packet['channel']
        if channel == 'imu':
            continue  # Raw IMU is retained; a-evidence summarizes it, not mode events.
        if channel not in ('joint', 'debug'):
            item = dict(channel=channel, receive_ns=packet['receive_ns'], message=packet['message'])
            if previous_mode.get(channel) != item['message']:
                modes.append(item)
                previous_mode[channel] = item['message']
            continue
        data = channels.setdefault(channel, dict(times=[], valid_times=[], invalid=0, errors=Counter(),
            axes={}, layouts=[], optional_unavailable=Counter(), samples=[]))
        data['times'].append(packet['receive_ns'])
        errors = position_errors(packet, config['joint_names'])
        if errors:
            data['invalid'] += 1
            data['errors'].update(errors)
            if len(data['samples']) < 8:
                data['samples'].append(dict(receive_sequence=packet['receive_sequence'], errors=errors))
        msg = packet['message']
        names = msg.get('name', [])
        valid_names = isinstance(names, list) and all(isinstance(n, str) and n for n in names) and len(set(names)) == len(names)
        if not valid_names:
            continue
        if not data['layouts'] or data['layouts'][-1]['names'] != names:
            data['layouts'].append(dict(receive_ns=packet['receive_ns'], names=names))
        for field in ('velocity', 'effort'):
            if not finite_array(msg.get(field), len(names)):
                data['optional_unavailable'][field] += 1
        if not errors:
            data['valid_times'].append(packet['receive_ns'])
        # Preserve statistics for every named axis, including non-target axes.
        if finite_array(msg.get('position'), len(names)):
            for name, value in zip(names, msg['position']):
                axis = data['axes'].setdefault(name, dict(name=name, target_arm=name in config['joint_names'],
                    count=0, first_rad=value, last_rad=value, minimum_rad=value, maximum_rad=value))
                axis['count'] += 1
                axis['last_rad'] = value
                axis['minimum_rad'] = min(axis['minimum_rad'], value)
                axis['maximum_rad'] = max(axis['maximum_rad'], value)
    summary = read_json(raw / 'summary.json', {})
    capture = read_json(capture_root / 'capture.json', {})
    recorder = read_json(raw / 'recorder.json', {})
    primary = channels.get('joint', {})
    times = primary.get('valid_times', [])
    timeout = session['timeout_ns']
    gaps = [dict(last_valid_receive_ns=a, first_restored_receive_ns=b, gap_ns=b-a)
            for a, b in zip(times, times[1:]) if b-a > timeout]
    tail_stale = bool(times and summary.get('end_ns', times[-1])-times[-1] > timeout)
    snapshots = list(json_lines(raw / 'discovery.jsonl', evidence_errors))
    operator_events = list(json_lines(root/'events/operator.jsonl', evidence_errors))
    monitor_events = list(json_lines(root/'events/monitor.jsonl', evidence_errors))
    endpoints = [s.get('monitored_topics', {}).get(config['joint_topic'], []) for s in snapshots]
    matched = [[p for p in publishers if p['type'] == config['joint_type']] for publishers in endpoints]
    has_publisher = any(endpoints)
    has_type = any(matched)
    # Recorder subscriptions are volatile. Reliable reader + best-effort writer
    # is a definite incompatibility; unknown middleware policies stay unknown.
    compatible = any(any(not config['reliable'] or p['qos_reliability'] != 2 for p in group) for group in matched)
    if times:
        connection = 'data_received'
    elif not snapshots:
        connection = 'discovery_evidence_unavailable'
    elif not has_publisher:
        connection = 'publisher_not_discovered'
    elif not has_type:
        connection = 'message_type_mismatch'
    elif not compatible:
        connection = 'qos_reliability_incompatible'
    elif primary.get('times'):
        connection = 'invalid_position_feedback'
    else:
        connection = 'publisher_discovered_no_data'
    checks = []
    def check(item_id, status, conclusion, next_action, evidence):
        checks.append(dict(item_id=item_id, status=status, conclusion=conclusion,
                           next_action=next_action, evidence=evidence))
    environment = read_json(root / 'environment.json', {})
    environment_ok = environment.get('ros_distro') == 'humble' and environment.get('recorder_available', False)
    check('environment', 'passed' if environment_ok else 'blocked',
          'Humble与只读记录器已找到' if environment_ok else 'Humble环境或vendor记录器不可用',
          '继续查看端点与数据' if environment_ok else '加载Humble环境并构建vendor overlay', ['environment.json'])
    deliberate_finish = bool(capture.get('returncode') == 0 and summary.get('reason') == 'interrupted' and
                             any(e['event'] == 'operator_finish' for e in operator_events))
    finished = capture.get('completed') or deliberate_finish
    check('capture', 'passed' if finished else 'failed',
          '操作者结束采集，已保存记录' if deliberate_finish else '采集正常结束' if finished else '采集未正常完成，保留已接收记录',
          '查看采集数据' if finished else '检查capture/process.log及capture_error.json；新建会话重试',
          ['capture/capture.json', 'capture/process.log', 'capture/raw/summary.json'])
    from .vendor_data import raw_fingerprints
    try:
        source_integrity = capture.get('source_sha256') == raw_fingerprints(capture_root)
    except OSError:
        source_integrity = False
    source_session = read_json(capture_root/'session.json', {})
    source_binding = (source_session.get('config') == config and source_session.get('domain_id') == session['domain_id'] and
                      source_session.get('connection_scope') == session['scope'])
    check('evidence', 'passed' if source_integrity and source_binding and not evidence_errors else 'failed',
          '原始采集摘要与配置/域号/scope绑定一致' if source_integrity and source_binding and not evidence_errors else
          '原始证据未封存、被改动、绑定不符或含不完整记录',
          '继续分析完整记录' if source_integrity and source_binding and not evidence_errors else '检查原文件及capture_error.json；当前测量不可作为通过证据',
          ['session.json', 'capture/session.json', 'capture/capture.json'])
    ambiguous = any(len(group) > 1 for group in matched)
    check('discovery', 'blocked' if ambiguous or not snapshots else 'passed' if has_type else 'blocked',
          '同一目标topic发现多个同类型发布者，目标不明确' if ambiguous else
          '已保存目标端点及QoS快照' if has_type else '未发现目标类型端点或缺少发现证据',
          '现场核对目标、其他机器人/仿真及发布者' if ambiguous else
          '核对反馈数据' if has_type else '检查显式域号、网卡/网络和topic类型；不要扫描域号',
          ['capture/raw/discovery.jsonl'])
    valid = len(times) >= 2
    check('feedback', 'passed' if valid else 'blocked',
          '已收到至少两帧完整14轴具名位置反馈' if valid else '完整14轴有效位置反馈不足：' + connection,
          '查看全轴表；实物映射待F2确认' if valid else '按连接诊断检查网络、类型和QoS；字段错误查看原始消息',
          ['capture/raw/packets.jsonl'])
    invalid = primary.get('invalid', 0)
    monotonic = receive_rates(primary.get('times', []))['monotonic']
    check('position_structure', 'failed' if invalid or not monotonic else 'passed' if valid else 'blocked',
          f'无效位置记录{invalid}条；本地接收时刻严格递增={monotonic}',
          '检查错误样例，保留原始数据' if invalid or not monotonic else '继续查看接收节奏', ['capture/raw/packets.jsonl'])
    continuity = not gaps and not tail_stale
    check('continuity', 'passed' if valid and (session['command'] == 'reconnect' or continuity) else 'blocked',
          f'有效位置反馈超时空窗{len(gaps)}段；结束时超时={tail_stale}',
          '查看空窗和恢复证据；空窗不等于网络丢包', ['capture/raw/packets.jsonl', 'events/monitor.jsonl'])
    if session['command'] == 'reconnect':
        check('recovery', 'passed' if gaps and not tail_stale else 'blocked',
              '已在同一采集进程观察到有效位置反馈中断后恢复收数' if gaps and not tail_stale else '尚未观察到完整中断—恢复周期或结束时仍超时，是否需重启尚不能判断',
              '核对人工拔插标记与恢复前后全轴/端点；不恢复运动',
              ['events/operator.jsonl', 'events/monitor.jsonl', 'capture/raw/discovery.jsonl'])
    status = 'failed' if any(c['status'] == 'failed' for c in checks) else 'blocked' if any(c['status'] == 'blocked' for c in checks) else 'passed'
    for data in channels.values():
        data['receive'] = receive_rates(data.pop('times'))
        data['valid_position_receive'] = receive_rates(data.pop('valid_times'))
        data['axes'] = list(data['axes'].values())
        for axis in data['axes']:
            axis['span_rad'] = axis['maximum_rad']-axis['minimum_rad']
        data['errors'] = dict(data['errors'])
        data['optional_unavailable'] = dict(data['optional_unavailable'])
    return dict(schema_version='casbot_fieldwork_report_v1', run_id=session['run_id'], created_utc=session['created_utc'],
        command=session['command'], domain_id=session['domain_id'], scope=session['scope'], target_label=session['target_label'],
        read_only=True, status=status, automated_checks_passed=status == 'passed', checks=checks,
        connection_diagnosis=connection, channels=channels, gaps=gaps, stale_at_end=tail_stale,
        recovery_observation=dict(same_recorder_recovered=bool(gaps) and not tail_stale,
            restart_required='not_needed_for_observed_cycle' if gaps and not tail_stale else 'not_determined', auto_restart_performed=False),
        recorder_clock_id=recorder.get('clock_id'), source=str(root.resolve()),
        topology_scope='本次会话观测；端点GID不作为设备身份',
        timing_scope='本地回调接收与轮询检出；非设备采样或单向网络延迟',
        non_target_observation=dict(names=[axis['name'] for axis in channels.get('joint', {}).get('axes', []) if not axis['target_arm']],
            coverage='仅覆盖收到的具名轴；缺失的非目标轴未知', hold_verified=False),
        mode_events=modes, operator_events=operator_events, monitor_events=monitor_events, evidence_errors=evidence_errors,
        manual_confirmation=dict(status='needs_manual_confirmation', conclusion='目标实物与物理方向须在GUI逐轴核对；只读报告不授予运动资格'),
        unavailable=dict(sample_time='not_supported_by_current_interface', device_sequence='not_supported_by_current_interface',
            cache_age='not_supported_by_current_interface', actual_sample_hz=None, one_way_network_delay_ms=None,
            independent_torque=None, effort_scale='unknown'),
        motion_allowed=False, physical_stop_verified=False, hardware_calibration_accepted=False)


STATUS_LABELS = dict(passed='通过', failed='失败', blocked='条件不足', needs_manual_confirmation='待人工确认')
ITEM_LABELS = dict(environment='运行环境', capture='采集结束', evidence='证据绑定', discovery='端点发现',
                   feedback='有效位置反馈', position_structure='字段与时钟', continuity='反馈连续性', recovery='恢复收数')


def markdown_report(report):
    def safe(value):
        return str(value).replace('|', '\\|').replace('\n', ' ')
    text = ['# 现场只读联调报告', '', f"运行：{report['run_id']}；域号：{report['domain_id']}；范围：{report['scope']}",
            f"自动检查：{STATUS_LABELS[report['status']]}。实物目标与方向待人工核对，运动未开放。", '',
            '## 结论与下一步', '', '| 项目 | 结论 | 下一步 |', '| --- | --- | --- |']
    for check in report['checks']:
        text.append(f"| {ITEM_LABELS[check['item_id']]}（{STATUS_LABELS[check['status']]}） | {safe(check['conclusion'])} | {safe(check['next_action'])} |")
    text += ['', '## 接收频率与全轴位置', '', report['timing_scope'], '']
    text += ['非目标轴：' + ', '.join(report['non_target_observation']['names']),
             report['non_target_observation']['coverage'] + '。位置稳定本身不证明驱动保持。', '']
    for name, channel in report['channels'].items():
        rate = channel['receive']
        text += [f"### {name}", '', f"回调记录{rate['count']}条；接收频率{rate['hz']} Hz；间隔毫秒：{rate['interval_ms']}",
                 '', '| 关节名称 | 目标臂轴 | 有效位置数 | 首值rad | 末值rad | 跨度rad |', '| --- | --- | --- | --- | --- | --- |']
        for axis in channel['axes']:
            text.append('| ' + ' | '.join(safe(axis[key]) for key in ('name', 'target_arm', 'count', 'first_rad', 'last_rad', 'span_rad')) + ' |')
    text += ['', '## 中断与恢复', '', f"超时阈值由session.json封存；完整有效位置空窗{len(report['gaps'])}段。",
             '空窗包含无消息或无效位置记录；不据此判定网络丢包。重复数值未去重。', '',
             '```json', json.dumps(report['gaps'], ensure_ascii=False, indent=2), '```', '',
             '人工事件见events/operator.jsonl，实际轮询检出见events/monitor.jsonl；端点变化见capture/raw/discovery.jsonl。', '',
             '## 证据', '', '路径相对于原始会话：' + report['source'], '']
    for check in report['checks']:
        text.append(f"- {check['item_id']}：" + '、'.join(check['evidence']))
    text += ['', '发布stamp、回调接收时间均不是设备采样时间。估算effort原样保留，尺度未知；不生成电流、采样序号或物理停止证明。', '']
    return '\n'.join(text)


def save_report(root, output=None):
    root = Path(root).resolve()
    session = read_json(root / 'session.json')
    if not session or session.get('schema_version') != 'casbot_fieldwork_session_v1':
        raise ValueError('Expected a fieldwork session with session.json')
    report = analyse_capture(root, session)
    destination = Path(output).resolve() if output is not None else root
    if output is not None:
        if destination == root or root in destination.parents:
            raise ValueError('Reanalysis output must be outside the source session')
        destination.mkdir(parents=True, exist_ok=False)
    for name in ('report.json', 'report.md'):
        if (destination/name).exists():
            raise FileExistsError(f'Report already exists: {destination/name}')
    write_json(destination/'report.json', report)
    (destination/'report.md').write_text(markdown_report(report), encoding='utf-8')
    return report
