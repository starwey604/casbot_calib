"""S1 vendor observations. Missing fields remain missing; no identification or ROS."""
import json
from pathlib import Path
import numpy as np
import yaml
from .vendor_identity import sha256


def load_config(path):
    config = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    names = config['joint_names']
    if (config.get('schema_version') != 'casbot_vendor_s1_v1' or config.get('scope') != 'read_only'
            or len(names) != 14 or len(set(names)) != 14 or any(not isinstance(n, str) or not n for n in names)):
        raise ValueError('S1 requires a read-only configuration with 14 unique named axes')
    if config.get('position_unit') != 'rad' or config.get('velocity_unit') != 'rad/s':
        raise ValueError('S1 currently requires explicitly configured rad and rad/s')
    if config.get('sample_timestamp_semantics') != 'unknown':
        raise ValueError('S1 preserves header stamps but does not qualify sampling time')
    if config.get('effort_source') != 'unknown' or config.get('effort_unit') != 'unknown':
        raise ValueError('Effort scale/source qualification belongs to S3')
    if config.get('model_mapping_verified') is not False:
        raise ValueError('S1 uses an explicitly unverified model mapping')
    if config.get('joint_type') not in ('sensor_msgs/msg/JointState', 'crb_ros_msg/msg/JointStateData'):
        raise ValueError('Unsupported primary feedback type')
    if not config.get('joint_topic') or config.get('depth', 0) < 1 or config.get('stale_after_ns', 0) <= 0:
        raise ValueError('Feedback topic, positive depth and stale threshold are required')
    return config


def _finite(values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and np.isfinite(v) for v in values)


def is_hand_joint(name):
    return name.startswith(('left_', 'right_')) and any(
        '_'+finger+'_' in name for finger in ('thumb', 'index', 'middle', 'ring', 'pinky'))


def normalize_packet(packet, config):
    """Validate arrays independently. A bad effort channel does not invalidate q.

    receive_sequence is only local callback order. Header is never sample_ns.
    Returns a diagnostic even for rejected position packets.
    """
    msg = packet['message']; names = msg.get('name', [])
    result = dict(receive_sequence=packet['receive_sequence'], receive_ns=packet['receive_ns'],
                  channel=packet['channel'], header=msg.get('header'), errors=[], q=None, dq=None,
                  effort_raw=None, kp=None, kd=None, sample_ns=None, sample_time_known=False,
                  device_sequence=None, device_status=None, current=None, independent_torque=None)
    if (not isinstance(names, list) or not names or any(not isinstance(n, str) or not n for n in names)
            or len(set(names)) != len(names)):
        result['errors'].append('invalid_or_duplicate_names'); return result
    missing = sorted(set(config['joint_names']) - set(names))
    if missing:
        result['errors'].append('missing_axes:'+','.join(missing)); return result
    order = [names.index(n) for n in config['joint_names']]
    for key, target in [('position', 'q'), ('velocity', 'dq'), ('effort', 'effort_raw')]:
        values = msg.get(key, [])
        if not isinstance(values, list) or len(values) != len(names) or not _finite(values):
            result['errors'].append('unavailable_'+key); continue
        result[target] = [values[i] for i in order]
    if packet['type'] == 'crb_ros_msg/msg/JointStateData':
        pd_names = [n for n in names if not is_hand_joint(n)]
        kp, kd = msg.get('kp', []), msg.get('kd', [])
        if (len(kp) == len(kd) == len(pd_names) and _finite(kp) and _finite(kd)
                and all(v >= 0 for v in kp+kd)):
            pd_order = [pd_names.index(n) for n in config['joint_names']]
            result['kp'], result['kd'] = ([values[i] for i in pd_order] for values in (kp, kd))
        else:
            result['errors'].append('unavailable_compact_pd')
    return result


def raw_fingerprints(root):
    root = Path(root)
    names = ['session.json', 'raw/recorder.json', 'raw/packets.jsonl', 'raw/cdr.bin', 'raw/summary.json', 'raw/publishers.jsonl']
    return {name: sha256(root/name) for name in names}


def normalize_session(folder, output):
    root = Path(folder); output = Path(output)
    session = json.loads((root/'session.json').read_text())
    config = session['config']
    before = raw_fingerprints(root)
    captured = json.loads((root/'capture.json').read_text())
    if captured.get('source_sha256') != before:
        raise ValueError('Raw capture no longer matches the completion manifest')
    recorder = json.loads((root/'raw/recorder.json').read_text())
    rows, diagnostics, events = [], [], []
    sequence = 0; previous_ns = -1; offset = 0
    with (root/'raw/packets.jsonl').open() as stream:
        for line in stream:
            packet = json.loads(line)
            if (packet['receive_sequence'] != sequence or packet['receive_ns'] < previous_ns
                    or packet['cdr_offset'] != offset or packet['cdr_size'] <= 0):
                raise ValueError('Raw callback journal is discontinuous or CDR offsets changed')
            sequence += 1; previous_ns = packet['receive_ns']; offset += packet['cdr_size']
            if packet['channel'] not in ('joint', 'debug'):
                events.append(packet); continue
            row = normalize_packet(packet, config)
            diagnostics.append({k: row[k] for k in ('receive_sequence', 'receive_ns', 'channel', 'errors')})
            if row['channel'] == 'joint': rows.append(row)
    if offset != (root/'raw/cdr.bin').stat().st_size:
        raise ValueError('Trailing or missing CDR bytes; preserve interrupted capture separately')
    summary = json.loads((root/'raw/summary.json').read_text())
    if summary['receive_packets'] != sequence:
        raise ValueError('Raw summary and journal counts disagree')
    if not rows:
        raise ValueError('No primary joint feedback received; original raw capture preserved')
    receive = np.array([r['receive_ns'] for r in rows], dtype=np.int64)
    gaps = np.diff(receive); valid = np.array([r['q'] is not None for r in rows])
    def array(key):
        return np.array([r[key] if r[key] is not None else [np.nan]*14 for r in rows], dtype=float)
    arrays = dict(receive_ns=receive, receive_sequence=np.array([r['receive_sequence'] for r in rows], dtype=np.int64),
                  position_valid=valid, q=array('q'), dq=array('dq'), effort_raw=array('effort_raw'),
                  kp=array('kp'), kd=array('kd'),
                  header_ns=np.array([r['header']['sec']*1_000_000_000+r['header']['nanosec']
                                      if r['header'] is not None else -1 for r in rows], dtype=np.int64))
    for key in ('dq', 'effort_raw', 'kp', 'kd'):
        arrays[key+'_available'] = np.isfinite(arrays[key]).all(axis=1)
    span = int(receive[-1])-int(receive[0])
    mode_events = [e for e in events if e['channel'] == 'mode']
    elapsed_end = int(summary['end_ns']) - int(receive[-1])
    report = dict(schema_version='casbot_vendor_observations_v1', read_only=True,
                  position_observations_available=bool(np.any(valid)), joint_names=config['joint_names'],
                  model_id=config['model_id'], model_mapping_verified=config['model_mapping_verified'],
                  packets=len(rows), valid_position_packets=int(valid.sum()), invalid_position_packets=int((~valid).sum()),
                  receive_hz=(len(rows)-1)*1e9/span if span > 0 else None,
                  receive_interval_ms=dict(p50=float(np.median(gaps))*1e-6, p99=float(np.quantile(gaps, .99))*1e-6,
                                           maximum=float(max(gaps))*1e-6) if len(gaps) else None,
                  stale_gaps=int(np.count_nonzero(gaps > config['stale_after_ns'])),
                  stale_at_capture_end=bool(elapsed_end > config['stale_after_ns']),
                  valid_pose_stale_at_capture_end=bool(not np.any(valid) or int(summary['end_ns'])-int(receive[valid][-1]) > config['stale_after_ns']),
                  mode_messages=len(mode_events), last_mode=mode_events[-1]['message'] if mode_events else None,
                  latest_debug=[r for r in diagnostics if r['channel']=='debug'][-1:] or None,
                  unavailable=dict(sample_time=True, device_sequence=True, current=True, torque_scale=True,
                                   device_status=True, command_adoption=True, one_way_latency=True),
                  clock_id=recorder['clock_id'], effort_source='unknown', effort_unit='unknown',
                  rates_scope='local callback arrivals including file logging load; not device sampling/adoption',
                  expected_receive_hz=config['expected_receive_hz'], dynamics_qualified=False, hardware_executable=False,
                  hardware_scope_requested=session['connection_scope'] == 'hardware_read_only', hardware_identity_verified=False,
                  source_sha256=before, capture_reason=summary['reason'])
    capabilities = dict(schema_version='casbot_vendor_capabilities_v1', source='S1 observed fields, not hardware qualification',
                        position=dict(observed_packets=int(valid.sum()), units='configured rad; hardware mapping unverified'),
                        velocity=dict(observed_packets=int(arrays['dq_available'].sum()), source='unknown'),
                        effort=dict(observed_packets=int(arrays['effort_raw_available'].sum()), unit='unknown', source='unknown'),
                        pd=dict(source_interface='conditional SDK fields; readback does not prove control authority'),
                        raw_current='not_exposed_in_selected_messages', sample_time='unknown', device_sequence='not_exposed',
                        saturation='unknown', temperature='not_exposed_in_selected_messages', command_adoption='not_exposed',
                        control=dict(enabled=False, position='SDK_interface_only', torque='unknown', mixed_pd='conditional_unverified'),
                        stages={stage: dict(allowed=False, reason='S1 is read-only; hardware mapping/scale/timing/control not qualified')
                                for stage in ('direction_motion', 'trical_motion', 'gravity', 'friction', 'breakaway', 'inertia', 'control_comparison')})
    if before != raw_fingerprints(root):
        raise ValueError('Raw capture changed during normalization')
    output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output/'observations.npz', **arrays)
    for name, obj in [('report.json', report), ('capabilities.json', capabilities), ('diagnostics.json', diagnostics), ('status_events.json', events)]:
        (output/name).write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (output/'session.json').write_bytes((root/'session.json').read_bytes())
    names = ['observations.npz', 'report.json', 'capabilities.json', 'diagnostics.json', 'status_events.json', 'session.json']
    (output/'manifest.json').write_text(json.dumps({name: sha256(output/name) for name in names}, indent=2)+'\n')
    return report
