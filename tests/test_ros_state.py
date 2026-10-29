from pathlib import Path
from types import SimpleNamespace as Obj
import numpy as np
import pytest
from casbot_calib.vendor_data import load_config
from casbot_calib.ros_state import message_frame, MESSAGE_TYPES


@pytest.fixture
def config():
    return load_config(Path(__file__).resolve().parents[1]/'configs/vendor_s1.yaml')


def mock_message(config):
    return Obj(model_id='explicit_native_model', joint_names=list(reversed(config['joint_names'])),
               sample_time_known=True, sample_clock_id='host_sim_clock', sample_ns=8_000_000_000,
               publish_sequence=7, position=list(reversed(np.arange(14)*.1)), velocity=[.2]*14,
               current=[.3]*14, status=[0]*13+[1])


def test_native_mock_ros_pose_keeps_declared_clock_and_current(config):
    frame=message_frame(mock_message(config),MESSAGE_TYPES[2],config,1,200,source_model_id='explicit_native_model')
    np.testing.assert_allclose(frame.q,np.arange(14)*.1)
    assert frame.sample_ns==8_000_000_000 and frame.receive_ns==200 and frame.sequence==7
    assert frame.clock_domain=='host_sim_clock' and frame.status==1
    np.testing.assert_allclose(frame.current,.3)


def test_vendor_ros_pose_does_not_promote_header_or_effort(config):
    msg=Obj(name=config['joint_names'],position=[.1]*14,velocity=[],effort=[3.]*14,
            header=Obj(stamp=Obj(sec=9,nanosec=7),frame_id='unknown'))
    frame=message_frame(msg,MESSAGE_TYPES[0],config,4,200)
    assert frame.sample_ns==-1 and frame.current is None and frame.dq is None and frame.status is None
    assert frame.sequence==4 and frame.receive_ns==200


@pytest.mark.parametrize('mutation', ['model','missing_clock','missing_axis','nonfinite'])
def test_invalid_native_model_or_pose_is_rejected(config, mutation):
    msg=mock_message(config)
    if mutation=='model': msg.model_id='different'
    if mutation=='missing_clock': msg.sample_clock_id=''
    if mutation=='missing_axis': msg.joint_names[0]='unknown_joint'
    if mutation=='nonfinite': msg.position[0]=float('nan')
    with pytest.raises(ValueError):
        message_frame(msg,MESSAGE_TYPES[2],config,1,200,source_model_id='explicit_native_model')


def test_unknown_mock_sample_time_is_not_replaced_by_receive_time(config):
    msg=mock_message(config); msg.sample_time_known=False
    frame=message_frame(msg,MESSAGE_TYPES[2],config,1,200,source_model_id='explicit_native_model')
    assert frame.sample_ns==-1 and frame.receive_ns==200
