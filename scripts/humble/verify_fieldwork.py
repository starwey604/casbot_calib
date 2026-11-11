#!/usr/bin/env python3
"""F1 acceptance over private localhost DDS only. Never discovers physical robots."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    cases = [('all_axes', 0), ('custom_axes', 0), ('publisher_restart', 0), ('qos_mismatch', 2),
             ('wrong_type', 2), ('silent', 2), ('absent', 2), ('invalid', 1), ('ambiguous', 2), ('interrupt', 1), ('interactive', 0)]
    parser.add_argument('--case', choices=[name for name, _ in cases], help='Run only one state-only case; default also tests unified SDK Mock')
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    domain = 180 + int(uuid.uuid4().hex[:2], 16) % 30
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1')
    from casbot_calib.sdk_mock import SDKMockSession
    from casbot_calib.fieldwork_analysis import read_json, write_json
    from verify_sdk_mock import MockAdmin

    checks = {}
    baseline = yaml.safe_load(Path('configs/vendor_s1.yaml').read_text())
    def config_file(name, endpoint, custom=False):
        config = dict(baseline, joint_topic=endpoint+'/joint_states', debug_topic='',
            mode_topic=endpoint+'/motion/status', robot_state_topic='',
            joint_type='crb_ros_msg/msg/JointStateData' if custom else 'sensor_msgs/msg/JointState')
        path = root/(name+'.yaml')
        path.write_text(yaml.safe_dump(config))
        return path, config

    def launch(name, config, *, command='observe', duration=4, human=False, interactive=False):
        log = (root/(name+'.log')).open('x')
        argv = [sys.executable, '-m', 'casbot_calib.fieldwork_cli', command,
            '--config', str(config), '--domain-id', str(domain), '--scope', 'localhost_contract_mock',
            '--output', str(root/name), '--duration', str(duration)]
        if not human:
            argv.append('--json')
        if interactive:
            argv.append('--interactive')
        return subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                                stdin=subprocess.PIPE if interactive else subprocess.DEVNULL, text=True), log

    def finish(name, child, log, expected):
        try:
            code = child.wait(timeout=15)
        finally:
            if child.poll() is None:
                child.terminate(); child.wait(timeout=5)
            log.close()
        report = read_json(root/name/'report.json')
        checks[name] = code == expected and report is not None and report['read_only'] and not report['motion_allowed']
        if not checks[name]:
            raise RuntimeError(f'{name}: unexpected result {code}; inspect {root/name} and {name}.log')
        return report

    # Actual unified mechanical SDK Mock, without acquiring its motion mode.
    if args.case is None:
        with SDKMockSession('configs/sdk_mock.yaml', root/'sdk_mock', domain) as mock:
            admin = MockAdmin(mock)
            try:
                config_path, _ = config_file('sdk', mock.endpoint)
                child, log = launch('sdk_preflight', config_path, command='preflight', human=True)
                finish('sdk_preflight', child, log, 0)
                child, log = launch('sdk_reconnect', config_path, command='reconnect', duration=6)
                try:
                    deadline = time.monotonic()+8
                    while not (root/'sdk_reconnect/capture/raw/packets.jsonl').exists():
                        if time.monotonic() > deadline or child.poll() is not None:
                            raise RuntimeError('SDK read-only recorder did not start')
                        time.sleep(.03)
                    time.sleep(1.)
                    admin.set('feedback_enabled', False)
                    time.sleep(.9)
                    admin.set('feedback_enabled', True)
                    report = finish('sdk_reconnect', child, log, 0)
                    checks['sdk_recovery_observed'] = bool(report['gaps'] and report['monitor_events'])
                    checks['sdk_no_control'] = admin['received'] == 0 and admin['mode_calls'] == 0
                finally:
                    if child.poll() is None:
                        child.terminate(); child.wait(timeout=5); log.close()
            finally:
                admin.close()

    # State-only protocol fixtures cover full non-target arrays and negative
    # cases absent from the mechanical Mock; they do not simulate mechanics.
    import rclpy
    from rclpy.impl.implementation_singleton import rclpy_implementation
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from crb_ros_msg.msg import JointStateData
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('fieldwork_state_only_fixture', enable_rosout=False, start_parameter_services=False)
    prefix = '/casbot_calib_s1_'+uuid.uuid4().hex[:10]
    try:
        for case, expected in cases:
            if args.case is not None and case != args.case:
                continue
            endpoint = prefix+'/'+case
            path, config = config_file(case, endpoint, custom=case == 'custom_axes')
            cls = JointStateData if case == 'custom_axes' else String if case == 'wrong_type' else JointState
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT if case == 'qos_mismatch' else ReliabilityPolicy.RELIABLE)
            pub = None if case == 'absent' else node.create_publisher(cls, config['joint_topic'], qos)
            extra = node.create_publisher(cls, config['joint_topic'], qos) if case == 'ambiguous' else None
            child, log = launch(case, path, duration=20 if case == 'interactive' else 5,
                command='reconnect' if case in ('publisher_restart', 'interactive') else 'observe',
                interactive=case == 'interactive', human=case == 'interactive')
            began = time.monotonic()
            destroyed = restored = interrupted = False
            no_publishers = no_clients = True
            try:
                while child.poll() is None:
                    elapsed = time.monotonic()-began
                    if elapsed > 12:
                        raise TimeoutError(case)
                    rclpy.spin_once(node, timeout_sec=0)
                    if case == 'publisher_restart':
                        if elapsed > 1.6 and not destroyed:
                            node.destroy_publisher(pub); pub = None; destroyed = True
                        if elapsed > 3.0 and not restored:
                            pub = node.create_publisher(cls, config['joint_topic'], qos); restored = True
                    if case == 'interactive':
                        if elapsed > 1.6 and not destroyed:
                            child.stdin.write('d\n'); child.stdin.flush(); destroyed = True
                        if elapsed > 2.8 and not restored:
                            child.stdin.write('r\n'); child.stdin.flush(); restored = True
                        if elapsed > 4.0 and not interrupted:
                            child.stdin.write('n protocol fixture\n'); child.stdin.flush()
                            child.stdin.write('q\n'); child.stdin.flush(); interrupted = True
                    if pub is not None and case != 'silent' and not (case == 'interactive' and destroyed and not restored):
                        if cls == String:
                            msg = String(data='wrong feedback type')
                        else:
                            names = ['waist_yaw_joint'] + config['joint_names'] + ['left_thumb_proximal_joint']
                            if restored:
                                names = list(reversed(names))
                            q = {name: .01*i for i, name in enumerate(config['joint_names'])}
                            msg = cls(name=names, position=[q.get(n, .2) for n in names], velocity=[], effort=[])
                            msg.header.stamp.sec = 999  # publication field, never sampling time
                            if case == 'invalid':
                                msg.name[1] = msg.name[2]
                        pub.publish(msg)
                        if extra:
                            extra.publish(msg)
                    if case == 'interrupt' and elapsed > 2.0 and not interrupted:
                        child.send_signal(signal.SIGINT); interrupted = True
                    for topic, _ in node.get_topic_names_and_types():
                        no_publishers &= all(p.node_name != 'casbot_vendor_state_recorder'
                                             for p in node.get_publishers_info_by_topic(topic))
                    try:
                        no_clients &= not node.get_client_names_and_types_by_node('casbot_vendor_state_recorder', '/')
                    except rclpy_implementation.NodeNameNonExistentError:
                        pass  # recorder not discovered yet, or already exited
                    time.sleep(.01)
                report = finish(case, child, log, expected)
                checks[case+'_subscription_only'] = no_publishers and no_clients
                if case == 'interactive':
                    names = [e['event'] for e in report['operator_events']]
                    checks['operator_marks_preserved'] = names == ['operator_disconnect', 'operator_reconnect', 'operator_note', 'operator_finish']
                    checks['deliberate_finish_saved'] = read_json(root/case/'capture/raw/summary.json')['reason'] == 'interrupted'
                    checks['actual_timeout_and_recovery'] = any(e['event'] == 'valid_feedback_timeout' for e in report['monitor_events']) and any(e['event'] == 'valid_feedback_resumed' for e in report['monitor_events'])
                if case in ('all_axes', 'custom_axes', 'publisher_restart'):
                    checks[case+'_full_axes'] = len(report['channels']['joint']['axes']) == 16
                    checks[case+'_missing_optional_retained'] = report['channels']['joint']['optional_unavailable']['effort'] > 10
                    raw = root/case/'capture/raw'
                    cdr = (raw/'cdr.bin').read_bytes()
                    first = json.loads((raw/'packets.jsonl').read_text().splitlines()[0])
                    decoded = deserialize_message(cdr[first['cdr_offset']:first['cdr_offset']+first['cdr_size']], cls)
                    checks[case+'_cdr'] = list(decoded.name) == first['message']['name'] and len(decoded.name) == 16
                if case == 'publisher_restart':
                    checks['restart_layout_and_gap'] = len(report['channels']['joint']['layouts']) == 2 and len(report['gaps']) >= 1
                    graph = [json.loads(row) for row in (root/case/'capture/raw/discovery.jsonl').read_text().splitlines()]
                    counts = [len(row['monitored_topics'][config['joint_topic']]) for row in graph]
                    checks['restart_endpoint_absence_recorded'] = 0 in counts and 1 in counts
                if case == 'qos_mismatch':
                    checks['qos_cause_reported'] = report['connection_diagnosis'] == 'qos_reliability_incompatible'
                if case == 'wrong_type':
                    checks['type_cause_reported'] = report['connection_diagnosis'] == 'message_type_mismatch'
            finally:
                if child.poll() is None:
                    child.terminate(); child.wait(timeout=5); log.close()
                if pub is not None:
                    node.destroy_publisher(pub)
                if extra is not None:
                    node.destroy_publisher(extra)
                if child.stdin is not None:
                    child.stdin.close()
        # Reanalysis goes through the public CLI, without launching a recorder.
        if (root/'all_axes').exists():
            process = subprocess.run([sys.executable, '-m', 'casbot_calib.fieldwork_cli', 'report', str(root/'all_axes'),
                '--output', str(root/'offline_analysis'), '--json'], capture_output=True, text=True, timeout=10)
            checks['offline_cli'] = process.returncode == 0 and json.loads(process.stdout)['channels'] == read_json(root/'all_axes/report.json')['channels']
    finally:
        node.destroy_node(); rclpy.shutdown()
    result = dict(passed=all(checks.values()),
        scope='private localhost DDS: unified SDK mechanical Mock + state-only fixtures' if args.case is None else 'private localhost DDS: state-only protocol fixture '+args.case,
        unified_sdk_mock_tested=args.case is None,
        hardware_tested=False, physical_network_tested=False, checks=checks)
    write_json(root/'verification.json', result)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
