import json
import time
import numpy as np
import pytest
from casbot_calib.telemetry import JointMapping,TelemetryAdapter,ReplaySource,LatestStateSource
from casbot_calib.live import JsonlTelemetrySource


def packet(sequence=1):
    return dict(robot_model_id="bench",source="sdk_recording",sequence=sequence,sample_ns=1_000_000,
        receive_ns=2_000_000,joint_names=["a","b"],raw_position=[12.,18.],raw_velocity=[4.,-6.],
        raw_current=[.2,.3],target_q=[.4,.5],clock_domain="device_dc_unmapped")


def adapter():
    return TelemetryAdapter("bench",[JointMapping("b","b_joint",.1,-1,20.,2.,-1),JointMapping("a","a_joint",.2,1,10.)])


def test_raw_mapping_reorders_all_channels_and_copies_input():
    raw=packet();frame=adapter().convert(raw)
    assert frame.joint_names==("b","a")
    np.testing.assert_allclose(frame.q,[.2,.4]);np.testing.assert_allclose(frame.dq,[.6,.8])
    np.testing.assert_allclose(frame.current,[-.6,.2]);np.testing.assert_allclose(frame.target_q,[.5,.4])
    raw["raw_position"][1]=999.
    assert frame.raw_position[0]==18. and not frame.q.flags.writeable
    assert frame.clock_domain=="device_dc_unmapped"


@pytest.mark.parametrize("change",[{"robot_model_id":"other"},{"joint_names":["a","a"]},
    {"sample_ns":1.5},{"raw_position":[np.nan,1.]},{"raw_velocity":[1.]}])
def test_bad_packets_are_rejected(change):
    with pytest.raises(ValueError):adapter().convert({**packet(),**change})


def test_latest_only_buffer_is_clock_independent_and_rejects_old_packets():
    source=LatestStateSource(adapter(),stale_after_ns=100)
    assert source.is_stale(1000)
    assert source.publish(packet(4),arrival_ns=1000)
    assert not source.publish(packet(3),arrival_ns=2000)
    assert source.read(0).sequence==4
    assert not source.is_stale(1099) and source.is_stale(1101)
    for i in range(5,100):source.publish(packet(i),arrival_ns=1200+i)
    assert source.read(0).sequence==99


def test_replay_holds_acquired_samples_and_does_not_fill_gaps():
    a=TelemetryAdapter("bench",[JointMapping("a","observed_joint",.1,-1,10.)])
    o=dict(q=np.array([.1,.2,.3]),dq=np.ones(3),current=np.zeros(3),
        sequence=np.array([0,1,4],dtype=np.int64),sample_ns=np.array([0,1_000_000,4_000_000],dtype=np.int64),
        receive_ns=np.array([100,1_000_100,4_000_100],dtype=np.int64),status=np.zeros(3,dtype=np.uint32))
    s=ReplaySource(o,a)
    assert s.read(0) is None
    frame=s.read(3_000_000)
    assert frame.sequence==1 and frame.q[0]==pytest.approx(.2)
    assert s.read(5_000_000).sequence==4
    assert frame.target_q is None
    assert frame.raw_position is None


def test_bad_viewer_sign_is_not_cancelled_by_mock_encoder():
    from casbot_calib.demo import mock_encoder_channels
    o={"q":np.array([.15]),"dq":np.array([.4]),"current":np.array([.1])}
    raw=mock_encoder_channels(o)
    correct=JointMapping("a","a",2*np.pi/16384,-1,8192)
    wrong=JointMapping("a","a",2*np.pi/16384,1,8192)
    assert correct.position(raw["raw_position"])[0]==pytest.approx(.15)
    assert wrong.position(raw["raw_position"])[0]==pytest.approx(-.15)


def test_jsonl_follow_retains_partial_line_and_reports_bad_input(tmp_path):
    p=tmp_path/"stream.jsonl";p.write_text("")
    source=JsonlTelemetrySource(p,adapter(),follow=True)
    try:
        encoded=json.dumps(packet())
        with p.open("a") as f:f.write(encoded[:20]);f.flush()
        time.sleep(.04)
        assert source.read(0) is None and source.error is None
        with p.open("a") as f:f.write(encoded[20:]+"\n");f.flush()
        deadline=time.monotonic()+2
        while source.read(0) is None and time.monotonic()<deadline:time.sleep(.01)
        assert source.read(0).sequence==1
        with p.open("a") as f:f.write("invalid\n");f.flush()
        deadline=time.monotonic()+2
        while source.error is None and time.monotonic()<deadline:time.sleep(.01)
        assert source.error is not None
    finally:source.close()


def test_invalid_mapping_does_not_hide_direction_in_negative_scale():
    with pytest.raises(ValueError):JointMapping("a","a",-.1)
    with pytest.raises(ValueError):TelemetryAdapter("bench",[JointMapping("a","a"),JointMapping("a","b")])
