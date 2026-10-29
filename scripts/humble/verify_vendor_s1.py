#!/usr/bin/env python3
"""S1 real local DDS contract fixture. Publishes state only, never robot commands."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args()
    root = args.output.resolve(); root.mkdir(parents=True, exist_ok=False)
    # Domain and all application topics are private; no physical robot discovery.
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(180 + int(uuid.uuid4().hex[:2], 16) % 30)
    import rclpy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from crb_ros_msg.msg import JointStateData
    from std_msgs.msg import String
    from casbot_calib.vendor_data import normalize_session, raw_fingerprints
    from casbot_calib.vendor_viewer import VendorLiveSource
    from casbot_calib.ros_state import RosStateSource
    prefix = '/casbot_calib_s1_'+uuid.uuid4().hex[:10]
    rclpy.init()
    node = rclpy.create_node('vendor_s1_contract_fixture', enable_rosout=False, start_parameter_services=False)
    baseline = yaml.safe_load(Path('configs/vendor_s1.yaml').read_text())
    checks = {}; reports = {}
    try:
        for case, cls in [('standard', JointState), ('custom', JointStateData)]:
            config = dict(baseline, joint_topic=prefix+'/'+case+'/joint', debug_topic=prefix+'/'+case+'/debug',
                          mode_topic=prefix+'/'+case+'/mode', robot_state_topic=prefix+'/'+case+'/state',
                          joint_type='sensor_msgs/msg/JointState' if case=='standard' else 'crb_ros_msg/msg/JointStateData')
            config_path = root/(case+'.yaml'); config_path.write_text(yaml.safe_dump(config))
            pub = node.create_publisher(cls, config['joint_topic'], 10)
            debug = node.create_publisher(JointStateData, config['debug_topic'], 10)
            mode = node.create_publisher(String, config['mode_topic'], 10)
            state = node.create_publisher(String, config['robot_state_topic'], 10)
            capture_root = root/case
            with (root/(case+'-process.log')).open('w') as log:
                child = subprocess.Popen([sys.executable, '-m', 'casbot_calib.cli', 'vendor-capture',
                    '--config', str(config_path), '--output', str(capture_root), '--duration', '6',
                    '--domain-id', os.environ['ROS_DOMAIN_ID'], '--scope', 'localhost_contract_mock'], stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic()+8
                    while pub.get_subscription_count()==0 or debug.get_subscription_count()==0:
                        if time.monotonic()>deadline or child.poll() is not None:
                            raise RuntimeError('Recorder failed to match state-only fixture; see process log')
                        rclpy.spin_once(node, timeout_sec=.02)
                    live = VendorLiveSource(capture_root)
                    ros_live = RosStateSource(config,config['joint_topic'],config['joint_type'],scope='localhost_contract_mock')
                    try:
                        names = [config['joint_names'][0], 'left_thumb_proximal_joint', 'waist_yaw_joint']+config['joint_names'][1:]
                        base_q = {n: .03*(i+1) for i,n in enumerate(config['joint_names'])}
                        pd = {n: (10.+i, .5+i*.1) for i,n in enumerate(n for n in names if 'thumb' not in n)}
                        started = time.monotonic(); next_at = started; sent = 0; stale_seen = False; live_seen = False
                        read_only_graph = True; ros_seen = False; ros_stale = False
                        while time.monotonic()-started < 3.6:
                            elapsed = time.monotonic()-started
                            rclpy.spin_once(node, timeout_sec=0)
                            frame = live.read()
                            live_seen |= frame is not None
                            ros_frame = ros_live.read()
                            ros_seen |= ros_frame is not None and ros_frame.current is None and ros_frame.sample_ns==-1
                            if 1.45 < elapsed < 1.6:
                                stale_seen |= live.is_stale()
                                ros_stale |= ros_live.is_stale()
                            # Explicit silence, then reappearance. No simulated mechanics.
                            if 1.0 < elapsed < 1.6:
                                time.sleep(.002); next_at = time.monotonic(); continue
                            msg = cls(); raw_names = names if elapsed<.5 else list(reversed(names))
                            msg.name = raw_names
                            msg.position = [base_q.get(n, 0.)+.08*np.sin(2*np.pi*.5*elapsed) for n in raw_names]
                            msg.velocity = [.08*np.pi*np.cos(2*np.pi*.5*elapsed)]*len(raw_names)
                            msg.effort = [1.+i*.1 for i in range(len(raw_names))]
                            msg.header.stamp.sec = 100 if elapsed<2.0 else 0  # unknown/backward header stays raw
                            if isinstance(msg, JointStateData):
                                pd_names = [n for n in raw_names if 'thumb' not in n]
                                msg.kp = [pd[n][0] for n in pd_names]; msg.kd = [pd[n][1] for n in pd_names]
                            phase = int((elapsed-1.6)/.12) if elapsed>=1.6 else -1
                            if phase==0:
                                msg.name[0] = msg.name[1]  # duplicate name
                            elif phase==1:
                                msg.name = msg.name[:-1]; msg.position = msg.position[:-1]
                                msg.velocity = msg.velocity[:-1]; msg.effort = msg.effort[:-1]
                            elif phase==2:
                                msg.position = []
                            elif phase==3:
                                msg.velocity = []; msg.effort = []
                            elif phase==4:
                                values = list(msg.position); values[0] = float('nan'); msg.position = values
                            elif phase==5:
                                msg.velocity = [0.]
                            elif phase==6 and isinstance(msg, JointStateData):
                                msg.kp = [1.]*len(raw_names); msg.kd = [1.]*len(raw_names)  # wrong hand placeholders
                            pub.publish(msg); sent += 1
                            if sent%25==0:
                                mode.publish(String(data='mode: READY' if elapsed>2.0 else 'mode: UPPER_BODY_DEBUG, state: STAND'))
                                state.publish(String(data='只读契约样本，不是机器人主机'))
                                d = JointStateData(name=names, position=[base_q.get(n,0.) for n in names],
                                                  velocity=[0.]*len(names), effort=[1.]*len(names),
                                                  kp=[pd[n][0] for n in names if 'thumb' not in n],
                                                  kd=[pd[n][1] for n in names if 'thumb' not in n])
                                debug.publish(d)
                            if sent%100==0:
                                for topic, _ in node.get_topic_names_and_types():
                                    read_only_graph &= all(i.node_name!='casbot_vendor_state_recorder'
                                                           for i in node.get_publishers_info_by_topic(topic))
                            next_at += .002
                            time.sleep(max(0., next_at-time.monotonic()))
                        node.destroy_publisher(pub); node.destroy_publisher(debug)
                        node.destroy_publisher(mode); node.destroy_publisher(state)
                        code = child.wait(timeout=10)
                    finally:
                        live.close()
                        ros_live.close()
                    if code: raise RuntimeError(f'Recorder exited {code}; see {case}-process.log')
                finally:
                    if child.poll() is None:
                        child.terminate(); child.wait(timeout=5)
            before = raw_fingerprints(capture_root)
            report = normalize_session(capture_root, root/(case+'-normalized')); reports[case] = report
            diagnostics = json.loads((root/(case+'-normalized')/'diagnostics.json').read_text())
            errors = [e for r in diagnostics if r['channel']=='joint' for e in r['errors']]
            checks[case] = dict(read_only_graph=read_only_graph, live_pose=live_seen, live_stale=stale_seen,
                                direct_ros_pose=ros_seen, direct_ros_stale=ros_stale,
                                invalid_position_rejected=report['invalid_position_packets']>0,
                                stale_gap_reported=report['stale_gaps']>=1, stale_end_reported=report['stale_at_capture_end'],
                                unknown_time=report['unavailable']['sample_time'], dynamics_blocked=not report['dynamics_qualified'],
                                duplicate_rejected='invalid_or_duplicate_names' in errors,
                                missing_rejected=any(e.startswith('missing_axes:') for e in errors),
                                empty_position_rejected='unavailable_position' in errors,
                                missing_velocity_recorded='unavailable_velocity' in errors,
                                raw_unchanged=before==raw_fingerprints(capture_root))
            graph = [json.loads(s) for s in (capture_root/'raw/publishers.jsonl').read_text().splitlines()]
            checks[case]['endpoint_qos_recorded'] = any(g['topic']==config['joint_topic'] and g['type']==config['joint_type'] for g in graph)
            # Independently decode the received CDR bytes with ROS generated types.
            cdr = (capture_root/'raw/cdr.bin').read_bytes()
            packets = [json.loads(s) for s in (capture_root/'raw/packets.jsonl').read_text().splitlines()]
            decoded = 0; compact = None
            for packet in packets:
                if packet['channel'] not in ('joint', 'debug'): continue
                payload = cdr[packet['cdr_offset']:packet['cdr_offset']+packet['cdr_size']]
                native = deserialize_message(payload, JointState if packet['type']=='sensor_msgs/msg/JointState' else JointStateData)
                assert native.name == packet['message']['name']
                for actual, stored in zip(native.position, packet['message']['position'], strict=True):
                    assert (not np.isfinite(actual) and stored is None) or actual==stored
                decoded += 1
                if packet['channel']=='debug' and compact is None:
                    from casbot_calib.vendor_data import normalize_packet
                    compact = normalize_packet(packet, config)
            checks[case]['cdr_roundtrip'] = decoded>100
            checks[case]['compact_pd_by_name'] = compact['kp']==[pd[n][0] for n in config['joint_names']]
            if case=='custom': checks[case]['hand_placeholder_pd_rejected'] = 'unavailable_compact_pd' in errors
        # Interrupt a live capture through its Python launcher. Raw files survive.
        import signal
        config = dict(baseline, joint_topic=prefix+'/interrupt/joint', debug_topic='',
                      mode_topic='', robot_state_topic='')
        config_path=root/'interrupt.yaml'; config_path.write_text(yaml.safe_dump(config))
        pub=node.create_publisher(JointState, config['joint_topic'], 10)
        with (root/'interrupt-process.log').open('w') as log:
            child=subprocess.Popen([sys.executable,'-m','casbot_calib.cli','vendor-capture',
                '--config',str(config_path),'--output',str(root/'interrupt'),'--duration','30',
                '--domain-id',os.environ['ROS_DOMAIN_ID'],'--scope','localhost_contract_mock'],stdout=log,stderr=subprocess.STDOUT)
            try:
                deadline=time.monotonic()+8
                while pub.get_subscription_count()==0:
                    if time.monotonic()>deadline or child.poll() is not None: raise RuntimeError('Interrupted fixture did not match')
                    rclpy.spin_once(node,timeout_sec=.02)
                for _ in range(25):
                    pub.publish(JointState(name=config['joint_names'],position=[.05]*14))
                    rclpy.spin_once(node,timeout_sec=.01)
                child.send_signal(signal.SIGINT); code=child.wait(timeout=8)
            finally:
                node.destroy_publisher(pub)
                if child.poll() is None: child.terminate(); child.wait(timeout=5)
        interrupted=normalize_session(root/'interrupt',root/'interrupt-normalized')
        checks['interruption_preserved'] = bool(code==1 and interrupted['capture_reason']=='interrupted' and interrupted['valid_position_packets']>0)
        # Explicit source-order oracle, independent of publisher's reversed arrays.
        with np.load(root/'standard-normalized/observations.npz') as data:
            q = data['q'][np.flatnonzero(data['position_valid'])[0]]
            checks['named_mapping'] = bool(np.allclose(np.diff(q), .03, atol=1e-12))
        if args.render:
            from casbot_calib.vendor_viewer import view_vendor
            checks['viewer'] = view_vendor(root/'standard-normalized', root/'presentation')['passed']
        passed = all(all(v.values()) if isinstance(v,dict) else v for v in checks.values())
        result = dict(passed=passed, scope='localhost DDS with synthetic state-only protocol fixture; no robot mechanics',
                      hardware_tested=False, physical_network_tested=False, checks=checks,
                      receive_rates={name: r['receive_hz'] for name,r in reports.items()},
                      note='Rates include deliberate silence, invalid packets and logging; no 500Hz hardware qualification')
        (root/'verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if passed else 1
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
