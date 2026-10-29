"""Optional ROS latest-state input for the common viewer. No motor commands."""
from threading import Event, Lock, Thread
from time import monotonic_ns
import uuid
import numpy as np
from .telemetry import StateFrame
from .vendor_data import normalize_packet

MESSAGE_TYPES = ('sensor_msgs/msg/JointState', 'crb_ros_msg/msg/JointStateData',
                 'casbot_calib_interfaces/msg/JointObservation')


def message_frame(msg, message_type, config, receive_sequence, receive_ns, *, source_model_id=None):
    """Message-specific edge adapters converge on StateFrame, not on fake fields."""
    if message_type not in MESSAGE_TYPES:
        raise ValueError('Unsupported ROS state type')
    if message_type != MESSAGE_TYPES[2]:
        message = dict(header=dict(sec=int(msg.header.stamp.sec), nanosec=int(msg.header.stamp.nanosec),
                                   frame_id=msg.header.frame_id), name=list(msg.name), position=list(msg.position),
                       velocity=list(msg.velocity), effort=list(msg.effort))
        if message_type == MESSAGE_TYPES[1]: message.update(kp=list(msg.kp), kd=list(msg.kd))
        row = normalize_packet(dict(message=message, channel='joint', type=message_type,
                                    receive_sequence=receive_sequence, receive_ns=receive_ns), config)
        if row['q'] is None: raise ValueError(','.join(row['errors']))
        return StateFrame(config['model_id'], 'ROS vendor feedback, mapping unverified', receive_sequence,
                          -1, receive_ns, tuple(config['joint_names']), row['q'], row['dq'], None, row['q'],
                          status=None, clock_domain='vendor_sampling_clock_unknown')
    if not source_model_id or msg.model_id != source_model_id:
        raise ValueError('Mock source model ID differs from the explicitly expected model')
    names = list(msg.joint_names)
    if len(set(names)) != len(names) or set(names) != set(config['joint_names']):
        raise ValueError('Mock observation joint identity mismatch')
    order = [names.index(n) for n in config['joint_names']]
    def array(key):
        values = np.asarray(getattr(msg, key))
        if values.shape != (14,) or not np.isfinite(values).all():
            raise ValueError('Invalid mock observation '+key)
        return values[order]
    known = bool(msg.sample_time_known)
    if known and (not msg.sample_clock_id or msg.sample_ns < 0):
        raise ValueError('Known sample time requires its own declared clock domain')
    return StateFrame(config['model_id'], 'ROS native Mock observations', int(msg.publish_sequence),
                      int(msg.sample_ns) if known else -1, receive_ns, tuple(config['joint_names']),
                      array('position'), array('velocity'), array('current'), array('position'),
                      status=int(np.bitwise_or.reduce(array('status').astype(np.uint32))),
                      clock_domain=msg.sample_clock_id if known else 'mock_sampling_clock_unknown')


class RosStateSource:
    """Separate subscription thread with one immutable latest pose.

    Rendering never blocks acquisition and does not drive the physics clock.
    Receive age uses this subscriber's monotonic clock, even across hosts.
    """
    def __init__(self, config, topic, message_type, *, source_model_id=None, scope='hardware_read_only'):
        if message_type not in MESSAGE_TYPES: raise ValueError('Unsupported state message type')
        if message_type == MESSAGE_TYPES[2] and not source_model_id:
            raise ValueError('Mock input requires an explicit source model identity')
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        message_class = get_message(message_type)
        self.config = config; self.scope = scope; self.source_model_id = source_model_id
        self._lock = Lock(); self._frame = None; self._arrival_ns = None; self.error = None
        self.received = 0; self.rejected = 0; self._stop = Event(); self._remote_identity = None
        self._rclpy = rclpy; self._context = Context(); rclpy.init(context=self._context)
        self.node = rclpy.create_node('casbot_read_only_view_'+uuid.uuid4().hex[:8], context=self._context,
                                     enable_rosout=False, start_parameter_services=False)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        def receive(msg):
            at = monotonic_ns()
            with self._lock:
                sequence = self.received; self.received += 1
            try:
                frame = message_frame(msg, message_type, config, sequence, at, source_model_id=source_model_id)
                if message_type == MESSAGE_TYPES[2]:
                    identity = (msg.model_id, msg.clock_id, msg.sample_clock_id)
                    if self._remote_identity is not None and identity != self._remote_identity:
                        raise ValueError('Mock source clock identity changed; reconnect explicitly')
                    if self._frame is not None and frame.sequence <= self._frame.sequence:
                        raise ValueError('Old or duplicate Mock publication sequence')
                    self._remote_identity = identity
                with self._lock:
                    self._frame = frame; self._arrival_ns = at; self.error = None
            except (ValueError, TypeError, KeyError, AttributeError) as exc:
                with self._lock:
                    self.rejected += 1; self.error = str(exc)
        self._subscription = self.node.create_subscription(message_class, topic, receive, qos)
        self._executor = SingleThreadedExecutor(context=self._context); self._executor.add_node(self.node)
        self._thread = Thread(target=self._consume, daemon=True, name='readonly-ros-state')
        self._thread.start()

    def _consume(self):
        try:
            while not self._stop.is_set() and self._context.ok(): self._executor.spin_once(timeout_sec=.05)
        except Exception as exc:
            with self._lock: self.error = str(exc)

    def read(self, presentation_ns=0):
        with self._lock: return self._frame

    def is_stale(self):
        with self._lock:
            return self._arrival_ns is None or monotonic_ns()-self._arrival_ns > self.config['stale_after_ns']

    def close(self):
        self._stop.set(); self._thread.join(timeout=2)
        self._executor.shutdown(); self.node.destroy_node()
        self._rclpy.shutdown(context=self._context)


def view_ros(config_path, output, *, topic, message_type, domain_id, scope, source_model_id=None,
             window=False, duration_s=5., fps=12):
    import json
    import os
    from pathlib import Path
    from .vendor_data import load_config
    from .vendor_viewer import present_pose_source
    if not 0 <= domain_id <= 232 or scope not in ('localhost_ros_mock', 'hardware_read_only'):
        raise ValueError('Explicit ROS domain and scope are required')
    if scope=='localhost_ros_mock' and not topic.startswith(('/casbot_calib_bench/', '/casbot_calib_s1_')):
        raise ValueError('Mock viewing requires private calibration topics')
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1' if scope=='localhost_ros_mock' else '0'
    source = RosStateSource(load_config(config_path), topic, message_type, source_model_id=source_model_id, scope=scope)
    try:
        result = present_pose_source(source, output, live=True, window=window, duration_s=duration_s, fps=fps)
        metadata = dict(topic=topic, message_type=message_type, source_model_id=source_model_id, scope=scope,
                        domain_id=domain_id, received=source.received, rejected=source.rejected, last_error=source.error,
                        qos='BestEffort/Volatile/KeepLast(1); latest display only, not raw acquisition',
                        command_publishers=0, mode_clients=0)
        (Path(output)/'ros_input.json').write_text(json.dumps(metadata,indent=2)+'\n')
        return result
    finally:
        source.close()
