#!/usr/bin/env python3
"""Private localhost sensor fixture for the optional IMU recording path."""
import argparse
import json
import os
from pathlib import Path
import threading
import time
import uuid
import yaml


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--domain-id', type=int, required=True)
    a = p.parse_args()
    if not 0 <= a.domain_id <= 232:
        raise ValueError('Explicit ROS domain required')
    os.environ.update(ROS_DOMAIN_ID=str(a.domain_id), ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState, Imu
    from casbot_calib.vendor_capture import capture
    from casbot_calib.fieldwork_a_evidence import imu_summary
    from casbot_calib.data import write_json
    root = a.output.resolve(); root.mkdir(parents=True, exist_ok=False)
    topic = '/casbot_calib_s2_imu_'+uuid.uuid4().hex
    config = yaml.safe_load(Path('configs/vendor_s1.yaml').read_text())
    config.update(joint_topic=topic+'/joint_states', imu_topic=topic+'/imu', debug_topic='', mode_topic='', robot_state_topic='')
    path = root/'profile.yaml'; path.write_text(yaml.safe_dump(config))
    rclpy.init(); node = rclpy.create_node('fieldwork_imu_fixture_'+uuid.uuid4().hex[:8])
    joint = node.create_publisher(JointState, config['joint_topic'], 10)
    imu = node.create_publisher(Imu, config['imu_topic'], QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    stop = threading.Event()
    def publish():
        while not stop.is_set():
            q = JointState(); q.header.stamp = node.get_clock().now().to_msg()
            q.name = config['joint_names']; q.position = [0.]*14; q.velocity = [0.]*14; q.effort = [0.]*14
            m = Imu(); m.header = q.header; m.header.frame_id = 'torso_sensor_fixture'
            m.orientation.w = 1.; m.angular_velocity.x = .125; m.linear_acceleration.z = 9.81
            m.orientation_covariance[0] = -1.  # Preserve unavailable orientation declaration.
            imu.publish(m); joint.publish(q); stop.wait(.01)
    thread = threading.Thread(target=publish); thread.start()
    try:
        result = capture(path, root/'capture', duration_s=3., domain_id=a.domain_id, scope='localhost_contract_mock')
    finally:
        stop.set(); thread.join(); node.destroy_node(); rclpy.shutdown()
    return verify(root)


def verify(root):
    from casbot_calib.fieldwork_a_evidence import imu_summary
    from casbot_calib.data import write_json
    result = json.loads((root/'capture/capture.json').read_text())
    raw = root/'capture/raw'; packets = [json.loads(x) for x in (raw/'packets.jsonl').read_text().splitlines()]
    rows = [x for x in packets if x['channel'] == 'imu']; info = imu_summary(root)
    read_clock = json.loads((raw/'recorder.json').read_text())['clock_id']
    summary = json.loads((raw/'summary.json').read_text())
    checks = dict(capture_completed=result['completed'], imu_received=len(rows)>0,
        values_preserved=bool(rows) and all(r['message']['angular_velocity']==[.125,0.,0.] and r['message']['linear_acceleration']==[0.,0.,9.81] for r in rows),
        missing_orientation_preserved=bool(rows) and all(r['message']['orientation_covariance'][0]==-1 for r in rows),
        shared_callback_clock=bool(rows) and read_clock.startswith('linux_monotonic:') and all(summary['start_ns'] <= r['receive_ns'] <= summary['end_ns'] for r in packets),
        original_cdr_retained=bool(rows) and all(r['cdr_offset']+r['cdr_size']<=(raw/'cdr.bin').stat().st_size for r in rows),
        summary_reads_raw=len(info)==1 and info[0]['channels']['angular_velocity']['valid_rows']==len(rows))
    write_json(root/'verification.json', dict(passed=all(checks.values()), checks=checks, imu_rows=len(rows),
        fixture='private ROS known sensor messages, not physical standing dynamics', imu_summary=info))
    print(checks)
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
