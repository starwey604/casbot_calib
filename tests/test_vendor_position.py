import copy
import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib.vendor_position import position_config,validate_position_config,endpoint_params,plan_position,execute_position
from casbot_calib.vendor_data import load_config
from casbot_calib.vendor_position_analysis import rates

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def config():return position_config(ROOT/'configs/vendor_s2.yaml')


@pytest.mark.parametrize('change', [dict(scope='hardware'),dict(hardware_execution_enabled=True),dict(control_hz=1000),
                                   dict(collision_allowance_rad=.001),dict(start_tolerance_rad=0),dict(vel_scale=2)])
def test_unqualified_execution_profile_is_rejected(config,change):
    with pytest.raises(ValueError):validate_position_config(dict(config,**change))


def test_missing_waist_or_stop_semantics_cannot_enable_stream(config):
    for key in ('waist_drive','arm_subset','stop','watchdog','time_ref','default_pd'):
        changed=copy.deepcopy(config);changed['mock_contract'][key]='unknown'
        with pytest.raises(ValueError,match='Unconfirmed'):validate_position_config(changed)


def test_public_or_ambiguous_mock_endpoint_is_rejected(config):
    for endpoint in ('/upper_body_debug','/casbot_calib_s2_x//other'):
        with pytest.raises(ValueError):endpoint_params(config,endpoint)


def test_published_rate_is_measured_in_local_ns_and_not_configured_hz():
    base=8_000_000_000_000_000
    result=rates(np.array([base,base+2_000_000,base+4_000_000],dtype=np.int64))
    assert result['hz']==pytest.approx(500) and result['interval_ms']['p99']==pytest.approx(2)
    assert rates([])['hz'] is None


def make_plan(tmp_path,config,q=None):
    folder=tmp_path/'plan';folder.mkdir();(folder/'snapshot').mkdir()
    home=np.array([0,.45,0,-.9,0,0,0,0,-.45,0,-.9,0,0,0.]) if q is None else q
    (folder/'snapshot/snapshot.csv').write_text('unit fixture\n')
    (folder/'snapshot/result.json').write_text('{"passed":true}')
    obs=load_config(ROOT/'configs/vendor_s1.yaml')
    metadata=plan_position(config,obs,home,np.zeros(14),folder,{}, {},'test_binary',200)
    return folder,metadata


def test_reference_has_observed_start_stationary_boundaries_and_real_collision_gate(tmp_path,config):
    folder,metadata=make_plan(tmp_path,config)
    commands=np.genfromtxt(folder/'position_plan.csv',delimiter=',',skip_header=1)
    np.testing.assert_allclose(commands[0,1:15],metadata['initial_q'],atol=0)
    np.testing.assert_allclose(commands[-1,1:15],metadata['initial_q'],atol=1e-12)
    np.testing.assert_allclose(commands[[0,-1],15:],0.,atol=1e-12)
    assert np.min(np.ptp(commands[:,1:15],axis=0))>.03
    report=json.loads((folder/'collision_reference.json').read_text())
    assert report['passed'] and report['tracking_envelope']['position_allowance_rad']==[.003]*14
    assert metadata['hardware_executable'] is False


def test_invalid_snapshot_or_reference_limits_are_rejected(tmp_path,config):
    obs=load_config(ROOT/'configs/vendor_s1.yaml')
    with pytest.raises(ValueError,match='stationary'):
        plan_position(config,obs,np.zeros(14),np.ones(14),tmp_path,{}, {},'',200)


def test_tampered_position_csv_is_rejected_before_publisher_creation(tmp_path,config):
    folder,_=make_plan(tmp_path,config)
    with (folder/'position_plan.csv').open('a') as f:f.write('changed\n')
    with pytest.raises(ValueError,match='Frozen'):execute_position(folder,tmp_path/'run')
    assert not (tmp_path/'run').exists()


def test_identical_continuous_excitation_is_used_at_all_control_rates(tmp_path,config):
    coefficients=[]
    for hz in (100,250,500):
        path=tmp_path/str(hz);path.mkdir()
        folder,_=make_plan(path,dict(config,control_hz=hz))
        with np.load(folder/'reference.npz') as data:coefficients.append(data['coefficients'])
    np.testing.assert_array_equal(coefficients[0],coefficients[1]);np.testing.assert_array_equal(coefficients[1],coefficients[2])


def test_irreducible_tracking_box_fails_without_exponential_time_refinement(tmp_path,config):
    with pytest.raises(ValueError,match='reference/envelope rejected'):
        make_plan(tmp_path,dict(config,collision_allowance_rad=.03))
    report=json.loads((tmp_path/'plan/collision_reference.json').read_text())
    assert report['tracking_envelope']['reason']=='tracking_allowance_not_certifiable'
    assert report['tracking_envelope']['queries']<30
    assert not (tmp_path/'plan/position_plan.csv').exists()


def test_stop_rejection_requires_negative_server_event_and_never_claims_rest_confirmed():
    from casbot_calib.vendor_position_analysis import rejected_stop_verified
    event=np.array([('stop_rejected',)],dtype=[('event','U30')])
    capture=dict(passed=False,stop_ack=False,reason='stop_unconfirmed',fault='stale_feedback')
    assert rejected_stop_verified(capture,True,event)
    assert not rejected_stop_verified(capture,False,event)
    assert not rejected_stop_verified(dict(capture,stop_ack=True),True,event)
    assert not rejected_stop_verified(capture,True,None)
