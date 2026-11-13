import numpy as np
import pytest

from casbot_calib.trical_sdk import SimulatedIndicators,write_segment,run_point
from casbot_calib.dvt2_trical import configuration
from casbot_calib.kinematics import ball_world


def test_mailbox_atomic_fields(tmp_path):
    path=tmp_path/'segment.csv';q=np.arange(14,dtype=float)*.01
    write_segment(path,1,1,q,q+.001,2.)
    values=np.loadtxt(path,delimiter=',')
    np.testing.assert_array_equal(values[:3],[1,1,2])
    np.testing.assert_allclose(values[3:17],q)
    assert not list(tmp_path.glob('*.tmp'))
    with pytest.raises(ValueError):write_segment(path,2,1,q,[float('nan')]*14,2.)


def test_gauges_follow_feedback_have_independent_conversions():
    c=configuration();visit=c['workflow']['visits'][0];q=np.asarray(visit['nominal_q_rad'])
    sensor=SimulatedIndicators(c);sensor.select(visit['frame'],visit['ball_id'],ball_world(c,visit['ball_id']))
    first=sensor.read(q,100);second=sensor.read(q,200)
    assert second['sequence']==first['sequence']+1
    np.testing.assert_array_equal(first['readings'],second['readings'])
    q=q.copy();q[0]+=.001
    assert np.linalg.norm(sensor.read(q,300)['readings']-first['readings'])>1e-6
    sensor.fault='disconnected'
    with pytest.raises(RuntimeError,match='indicator_disconnected'):sensor.read(q,400)


def test_no_hardware_connection_or_output(tmp_path):
    with pytest.raises(ValueError,match='Explicit local SDK Mock'):
        run_point('configs/hardware_field.yaml',tmp_path/'forbidden',86)
    assert not (tmp_path/'forbidden').exists()


def test_bad_measured_start_never_starts_executor(tmp_path,monkeypatch):
    from casbot_calib import hardware_field,trical_sdk
    from casbot_calib.sdk_connection import mock_connection
    config=hardware_field.load_profile('configs/hardware_field.yaml')
    config['connection']=mock_connection('/casbot_calib_s2_k5_test',86,'a'*64)
    monkeypatch.setattr(trical_sdk,'load_profile',lambda _:config)
    monkeypatch.setattr(hardware_field,'snapshot_field',lambda *a:None)
    monkeypatch.setattr(hardware_field,'_read_snapshot',lambda _:np.full(14,99.))
    def forbidden(*a):raise AssertionError('Executor must not start')
    monkeypatch.setattr(trical_sdk.RealtimeDevice,'start',forbidden)
    result=run_point('unused',tmp_path/'rejected',86)
    assert not result['passed'] and result['failure']=='measured_start_reference_rejected'
    assert result['execution'] is None
