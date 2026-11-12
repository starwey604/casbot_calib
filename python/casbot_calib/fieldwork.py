"""F1 session orchestration. Only the subscription-only vendor recorder is launched."""
import json
import math
import os
from pathlib import Path
import platform
import select
import shutil
import subprocess
import sys
import time

from .fieldwork_analysis import ConnectionMonitor, read_json, save_report, write_json
from .run_marker import run_marker


def clock_id():
    return ('linux_monotonic:' + Path('/proc/sys/kernel/random/boot_id').read_text().strip() +
            ':' + str(Path('/proc/self/ns/time').readlink()))


class JsonTail:
    def __init__(self, path):
        self.path = Path(path)
        self.offset = 0

    def read(self):
        if not self.path.exists():
            return []
        rows = []
        with self.path.open(encoding='utf-8') as stream:
            stream.seek(self.offset)
            while True:
                line = stream.readline()
                if not line or not line.endswith('\n'):
                    break
                rows.append(json.loads(line))
                self.offset = stream.tell()
        return rows


def environment_snapshot():
    prefix = Path(os.environ.get('CASBOT_VENDOR_INSTALL', 'install/vendor')).resolve()
    result = dict(python=sys.version, executable=sys.executable, platform=platform.platform(),
        ros_distro=os.environ.get('ROS_DISTRO'), ros2=shutil.which('ros2'), vendor_install=str(prefix),
        recorder_available=(prefix/'casbot_calib_vendor/lib/casbot_calib_vendor/state_recorder').is_file(),
        environment={key: os.environ.get(key) for key in ('RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY', 'ROS_DOMAIN_ID',
            'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE')}, topology_scope='本次会话观测')
    result['interfaces'] = {}
    for name, command in [('addresses', ['ip', '-j', 'address', 'show']), ('routes', ['ip', '-j', 'route', 'show'])]:
        try:
            process = subprocess.run(command, capture_output=True, text=True, timeout=3, check=False)
            result['interfaces'][name] = dict(returncode=process.returncode,
                data=json.loads(process.stdout) if process.returncode == 0 else None, stderr=process.stderr)
        except (OSError, subprocess.TimeoutExpired, ValueError) as exc:
            result['interfaces'][name] = dict(unavailable=str(exc))
    return result


class LiveObservation:
    def __init__(self, root, session, notify, interactive=False, input_stream=None):
        self.root = Path(root)
        self.session = session
        self.notify = notify
        self.input = (sys.stdin if input_stream is None else input_stream) if interactive else None
        self.input_buffer = b''
        self.packets = JsonTail(self.root/'capture/raw/packets.jsonl')
        self.discovery = JsonTail(self.root/'capture/raw/discovery.jsonl')
        self.monitor = ConnectionMonitor(session['config']['joint_names'], session['timeout_ns'], self.emit)
        self.checked_clock = False
        self.graph_signature = None
        self.expected_clock = clock_id()

    def append(self, name, event):
        with (self.root/'events'/name).open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+'\n')

    def emit(self, event):
        event['clock_id'] = self.expected_clock
        self.append('monitor.jsonl', event)
        self.notify(event)

    def operator(self, line):
        text = line.strip()
        if not text:
            return
        code, _, note = text.partition(' ')
        event = dict(event={'d': 'operator_disconnect', 'r': 'operator_reconnect', 'q': 'operator_finish'}.get(code, 'operator_note'),
                     note=note if code in ('d', 'r', 'q', 'n') else text, observed_ns=time.monotonic_ns(),
                     clock_id=self.expected_clock, timestamp_semantics='operator_key_entry_not_physical_event')
        self.append('operator.jsonl', event)
        self.notify(event)
        if code == 'q':
            raise KeyboardInterrupt

    def poll(self, *, accept_input=True):
        metadata = read_json(self.root/'capture/raw/recorder.json')
        if metadata and not self.checked_clock:
            if metadata['clock_id'] != self.expected_clock:
                raise ValueError('Recorder and monitor monotonic clock identities differ')
            self.checked_clock = True
        if self.checked_clock:
            # Drain received records before checking freshness: frontend
            # polling latency must not create a fictitious feedback timeout.
            for packet in self.packets.read():
                self.monitor.packet(packet, time.monotonic_ns())
            self.monitor.tick(time.monotonic_ns())
            for snapshot in self.discovery.read():
                publishers = snapshot['monitored_topics'].get(self.session['config']['joint_topic'], [])
                signature = sorted((p['gid'], p['type']) for p in publishers)
                if signature != self.graph_signature:
                    self.emit(dict(event='publisher_set_observed', observed_ns=time.monotonic_ns(),
                                   graph_observed_ns=snapshot['observed_ns'], publishers=publishers))
                    self.graph_signature = signature
        if accept_input and self.input is not None:
            if select.select([self.input], [], [], 0)[0]:
                # Avoid TextIOWrapper read-ahead: after two rapid input lines,
                # select() may see no fd data while readline() still buffers q.
                chunk = os.read(self.input.fileno(), 4096)
                self.input_buffer += chunk
                lines = self.input_buffer.split(b'\n')
                self.input_buffer = lines.pop()
                if not chunk:
                    self.input = None
                    if self.input_buffer:
                        lines.append(self.input_buffer)
                        self.input_buffer = b''
                for line in lines:
                    self.operator(line.decode('utf-8'))


def collect(config_path, output, *, command, domain_id, scope, duration_s, timeout_s=None,
            target_label='', interactive=False, notify=lambda event: None, imu_topic=None):
    from .vendor_data import load_config
    from .vendor_capture import capture
    config = load_config(config_path)
    if imu_topic is not None:config['imu_topic'] = imu_topic
    if command not in ('preflight', 'observe', 'reconnect'):
        raise ValueError('Unknown read-only field command')
    if type(domain_id) is not int or not 0 <= domain_id <= 232:
        raise ValueError('An explicit integer ROS domain 0..232 is required')
    if scope not in ('hardware_read_only', 'localhost_contract_mock'):
        raise ValueError('An explicit read-only connection scope is required')
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError('Duration must be positive and finite')
    timeout_ns = config['stale_after_ns'] if timeout_s is None else round(timeout_s * 1e9) if math.isfinite(timeout_s) else 0
    if timeout_ns <= 0:
        raise ValueError('Timeout must be positive and finite')
    if scope == 'localhost_contract_mock' and any(
            config.get(key) and not config[key].startswith(('/casbot_calib_s1_', '/casbot_calib_s2_'))
            for key in ('joint_topic', 'debug_topic', 'mode_topic', 'robot_state_topic', 'imu_topic')):
        raise ValueError('Mock scope requires private S1/S2 topic names')
    root = Path(output).resolve()
    root.mkdir(parents=True, exist_ok=False)
    (root/'events').mkdir()
    for name in ('operator.jsonl', 'monitor.jsonl'):
        (root/'events'/name).touch()
    session = dict(schema_version='casbot_fieldwork_session_v1', **run_marker(),
        command=command, domain_id=domain_id, scope=scope, duration_s=duration_s, timeout_ns=timeout_ns,
        target_label=target_label, target_label_semantics='operator_description_not_device_identity',
        config=config, config_source=str(Path(config_path).resolve()), interactive=interactive,
        read_only=True, motion_allowed=False)
    write_json(root/'session.json', session)
    # Freeze the exact configuration passed to capture; later edits to the
    # caller's YAML must not change the semantics of the running session.
    import yaml
    frozen = root/'observation.yaml'
    frozen.write_text(yaml.safe_dump(config, allow_unicode=True), encoding='utf-8')
    environment = environment_snapshot()
    environment['effective_capture_environment'] = dict(ROS_DOMAIN_ID=str(domain_id),
        ROS_LOCALHOST_ONLY='1' if scope == 'localhost_contract_mock' else '0')
    write_json(root/'environment.json', environment)
    live = LiveObservation(root, session, notify, interactive)
    deadline = time.monotonic() + duration_s + 15
    def poll():
        if time.monotonic() > deadline:
            raise TimeoutError('Read-only recorder exceeded duration plus 15s startup/shutdown allowance')
        live.poll()
    try:
        capture(frozen, root/'capture', duration_s=duration_s, domain_id=domain_id, scope=scope,
                record_discovery=True, poll_callback=poll)
        live.poll(accept_input=False)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, KeyboardInterrupt) as exc:
        write_json(root/'capture_error.json', dict(error=type(exc).__name__, message=str(exc)))
    return save_report(root)
