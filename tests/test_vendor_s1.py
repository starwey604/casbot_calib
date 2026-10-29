import copy
import json
import sys
from pathlib import Path
import numpy as np
import pytest
from casbot_calib.vendor_data import load_config, normalize_packet, normalize_session, raw_fingerprints
from casbot_calib.vendor_viewer import VendorReplaySource, VendorLiveSource
from casbot_calib.telemetry import StateFrame


@pytest.fixture
def config():
    return load_config(Path(__file__).resolve().parents[1]/'configs/vendor_s1.yaml')


def packet(config, sequence=0, *, custom=False):
    # Interleave a hand and waist, with axes in reverse order.
    names = list(reversed(config['joint_names']))
    names.insert(1, 'left_thumb_proximal_joint'); names.insert(3, 'waist_yaw_joint')
    values = {name: i*.1 for i,name in enumerate(config['joint_names'])}
    msg = dict(header=dict(sec=200, nanosec=5, frame_id='unverified'), name=names,
               position=[values.get(n,8.) for n in names], velocity=[2.]*len(names), effort=[3.]*len(names))
    if custom:
        pd_names = [n for n in names if 'thumb' not in n]
        msg.update(kp=[10.+values.get(n,8.) for n in pd_names], kd=[1.]*len(pd_names))
    return dict(receive_sequence=sequence, receive_ns=1_000_000_000+sequence*2_000_000,
                channel='joint', type='crb_ros_msg/msg/JointStateData' if custom else 'sensor_msgs/msg/JointState',
                cdr_offset=sequence, cdr_size=1, message=msg)


def make_capture(root, config, packets):
    root.mkdir(); (root/'raw').mkdir()
    (root/'session.json').write_text(json.dumps(dict(config=config, connection_scope='localhost_contract_mock')))
    (root/'raw/recorder.json').write_text(json.dumps(dict(clock_id='unknown_device_is_not_local_clock')))
    (root/'raw/packets.jsonl').write_text(''.join(json.dumps(p)+'\n' for p in packets))
    (root/'raw/cdr.bin').write_bytes(b'x'*len(packets))
    (root/'raw/summary.json').write_text(json.dumps(dict(receive_packets=len(packets), end_ns=packets[-1]['receive_ns']+300_000_000,
                                                       reason='duration_complete')))
    (root/'raw/publishers.jsonl').write_text('')
    (root/'capture.json').write_text(json.dumps(dict(source_sha256=raw_fingerprints(root))))


def test_named_position_and_compact_pd_have_independent_oracle(config):
    row = normalize_packet(packet(config, custom=True), config)
    np.testing.assert_allclose(row['q'], np.arange(14)*.1)
    np.testing.assert_allclose(row['kp'], 10.+np.arange(14)*.1)
    assert row['current'] is None and row['sample_ns'] is None and row['device_sequence'] is None
    assert row['device_status'] is None and row['independent_torque'] is None


@pytest.mark.parametrize('mutation,reason', [
    ('duplicate','invalid_or_duplicate_names'), ('missing','missing_axes:'),
    ('empty_position','unavailable_position'), ('nan','unavailable_position'),
    ('infinite','unavailable_position')])
def test_reject_invalid_pose_but_keep_raw(config, mutation, reason):
    p = packet(config); msg = p['message']
    if mutation=='duplicate': msg['name'][0] = msg['name'][1]
    if mutation=='missing': msg['name'][0] = 'other_joint'
    if mutation=='empty_position': msg['position'] = []
    if mutation in ('nan','infinite'): msg['position'][0] = None if mutation=='nan' else float('inf')
    before = copy.deepcopy(p)
    row = normalize_packet(p, config)
    assert row['q'] is None and any(e.startswith(reason) for e in row['errors'])
    assert p == before


def test_empty_or_malformed_nonposition_arrays_do_not_invent_values(config):
    p = packet(config, custom=True); p['message'].update(velocity=[], effort=[1.], kp=[], kd=[])
    row = normalize_packet(p, config)
    assert row['q'] is not None and row['dq'] is row['effort_raw'] is row['kp'] is row['kd'] is None


def test_pd_hand_placeholders_are_not_misindexed(config):
    p = packet(config, custom=True)
    p['message']['kp'] = [10.]*len(p['message']['name']); p['message']['kd'] = [1.]*len(p['message']['name'])
    row = normalize_packet(p, config)
    assert row['kp'] is None and row['q'] is not None and 'unavailable_compact_pd' in row['errors']


def test_normalization_keeps_header_unknown_and_masks_missing_channels(tmp_path, config):
    packets = [packet(config,i) for i in range(3)]
    packets[1]['message']['velocity'] = []; packets[1]['message']['effort'] = []
    packets[1]['message']['header']['sec'] = 0
    packets[2]['receive_ns'] += 400_000_000
    root = tmp_path/'capture'; make_capture(root, config, packets)
    original = raw_fingerprints(root); out = tmp_path/'normalized'
    report = normalize_session(root,out)
    assert report['stale_gaps']==1 and report['stale_at_capture_end'] and not report['dynamics_qualified']
    with np.load(out/'observations.npz') as d:
        assert d['header_ns'][1]==5 and np.isnan(d['dq'][1]).all() and np.isnan(d['effort_raw'][1]).all()
        assert 'sample_ns' not in d and 'current' not in d and 'status' not in d and 'sequence' not in d
    capabilities = json.loads((out/'capabilities.json').read_text())
    assert all(not stage['allowed'] for stage in capabilities['stages'].values())
    assert raw_fingerprints(root)==original
    replay = VendorReplaySource(out)
    assert replay.read(-1) is None
    frame = replay.read(3_000_000)
    assert frame.sample_ns==-1 and frame.current is None and frame.dq is None and frame.status is None
    assert not replay.is_stale()
    replay.read(350_000_000); assert replay.is_stale()
    with pytest.raises(FileExistsError): normalize_session(root,out)


def test_rejected_position_holds_previous_pose_and_stays_stale(tmp_path, config):
    packets = [packet(config,i) for i in range(3)]
    for i in (1,2):
        packets[i]['message']['position']=[]; packets[i]['receive_ns'] += 400_000_000
    root = tmp_path/'capture'; make_capture(root,config,packets)
    normalize_session(root,tmp_path/'normalized')
    source = VendorReplaySource(tmp_path/'normalized'); frame = source.read(500_000_000)
    assert frame.sequence==0 and source.is_stale() and source.error is not None


def test_raw_tampering_or_unfinished_journal_is_rejected(tmp_path, config):
    root=tmp_path/'capture'; make_capture(root,config,[packet(config)])
    (root/'raw/cdr.bin').write_bytes(b'y')
    with pytest.raises(ValueError,match='completion manifest'): normalize_session(root,tmp_path/'normalized')


def test_callback_gaps_are_not_forged_as_device_continuity(tmp_path, config):
    p=packet(config,2); p['cdr_offset']=0
    root=tmp_path/'capture'; make_capture(root,config,[p])
    with pytest.raises(ValueError,match='journal'): normalize_session(root,tmp_path/'normalized')


def test_normalized_pose_tamper_is_rejected(tmp_path, config):
    root=tmp_path/'capture'; make_capture(root,config,[packet(config)])
    normalize_session(root,tmp_path/'normalized')
    (tmp_path/'normalized/observations.npz').write_bytes(b'wrong')
    with pytest.raises(ValueError,match='Normalized'): VendorReplaySource(tmp_path/'normalized')


@pytest.mark.skipif(not sys.platform.startswith('linux'),reason='ROS S1 live clock identity is Linux-only')
def test_live_tail_retains_partial_lines_and_does_not_fill_current(tmp_path, config):
    root=tmp_path/'capture'; make_capture(root,config,[packet(config)])
    clock='linux_monotonic:'+Path('/proc/sys/kernel/random/boot_id').read_text().strip()+':'+str(Path('/proc/self/ns/time').readlink())
    (root/'raw/recorder.json').write_text(json.dumps(dict(clock_id=clock)))
    line=json.dumps(packet(config))
    (root/'raw/packets.jsonl').write_text(line[:50])
    source=VendorLiveSource(root)
    try:
        assert source.read() is None
        with (root/'raw/packets.jsonl').open('a') as stream: stream.write(line[50:]+'\n')
        frame=source.read(); assert frame is not None and frame.current is None and frame.sample_ns==-1
        assert source.is_stale()
    finally: source.close()


@pytest.mark.skipif(not sys.platform.startswith('linux'),reason='ROS S1 live clock identity is Linux-only')
def test_live_rejects_foreign_monotonic_clock(tmp_path, config):
    root=tmp_path/'capture'; make_capture(root,config,[packet(config)])
    with pytest.raises(ValueError,match='local monotonic clock'): VendorLiveSource(root).read()


def test_pose_only_shared_frame_keeps_missing_fields_explicit():
    frame=StateFrame('nominal_unverified','vendor',0,-1,100,('a',),[.1],None,None,None,status=None)
    assert frame.current is None and frame.dq is None and not frame.q.flags.writeable


def test_capture_scope_rejects_public_mock_topics_before_launch(tmp_path):
    from casbot_calib.vendor_capture import capture
    with pytest.raises(ValueError,match='private'):
        capture(Path(__file__).resolve().parents[1]/'configs/vendor_s1.yaml',tmp_path/'unused',
                duration_s=1,domain_id=200,scope='localhost_contract_mock')
    assert not (tmp_path/'unused').exists()


def test_s1_rejects_self_declared_scale_or_clock_qualification(tmp_path, config):
    import yaml
    for key,value in [('effort_source','independent_torque'),('effort_unit','Nm'),
                      ('sample_timestamp_semantics','acquisition'),('model_mapping_verified',True)]:
        path=tmp_path/'config.yaml'; path.write_text(yaml.safe_dump(dict(config,**{key:value})))
        with pytest.raises(ValueError): load_config(path)


def test_empty_primary_capture_does_not_claim_readiness(tmp_path, config):
    p=packet(config); p['channel']='mode'; p['message']={'data':'mode: READY'}
    root=tmp_path/'capture'; make_capture(root,config,[p])
    with pytest.raises(ValueError,match='No primary'): normalize_session(root,tmp_path/'normalized')
    assert (root/'raw/cdr.bin').exists() and not (tmp_path/'normalized').exists()
