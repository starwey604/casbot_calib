from copy import deepcopy
import hashlib,json
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib.dual_excitation import direction_reference,periodic_reference
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.dual_preflight import mock_encoder_channels,check_direction,pulse_commands,identify_driver_timing
from casbot_calib.dual_workflow import _save_run


def config():return yaml.safe_load((Path(__file__).parents[1]/'configs/dual_arm_dynamics.yaml').read_text(encoding='utf-8'))


def test_large_multisine_and_smooth_isolated_jogs():
    cfg=config();motion=cfg['motion'];t=np.arange(0,motion['period_s'],.002)
    q,v,a,jerk=periodic_reference(t,motion['center'],motion['period_s'],motion['amplitude'])
    assert np.degrees(np.ptp(q,axis=0)).min()>20.
    center=np.zeros(14);times=np.array([3.37,4.71,6.52,9.83]);h=1e-5
    actual=direction_reference(times,center);plus=direction_reference(times+h,center);minus=direction_reference(times-h,center)
    for d in range(3):np.testing.assert_allclose(actual[d+1],(plus[d]-minus[d])/(2*h),atol=1e-7)
    for knot in (3.,4.2,6.6,7.8,9.):
        left=direction_reference(np.array([knot-1e-8]),center);right=direction_reference(np.array([knot+1e-8]),center)
        for d in range(4):np.testing.assert_allclose(left[d],right[d],atol=1e-5)
    q=direction_reference(np.array([4.2]),center)[0][0]
    assert q[0]>.44 and np.count_nonzero(q[1:])==0


def test_encoder_sign_and_zero_faults_do_not_cancel_against_the_producer():
    cfg=config();names=[j.name for j in nominal_dual_arm()[0].model.joints];pre=cfg['preflight']
    t=np.arange(0,pre['initial_hold_s']+14*pre['direction_slot_s'],.01)
    q,v,_,_=direction_reference(t,cfg['motion']['center'],pre['direction_amplitude_rad'],pre['direction_slot_s'],pre['initial_hold_s'])
    obs=dict(sample_ns=np.rint(t*1e9).astype(np.int64),q=q,dq=v,current=np.zeros_like(q))
    raw=mock_encoder_channels(obs)
    assert check_direction(obs,raw,q,names,cfg)['passed']
    flipped=deepcopy(cfg);flipped['preflight']['encoder_directions'][0]*=-1
    assert not check_direction(obs,raw,q,names,flipped)['passed']
    shifted=deepcopy(cfg);shifted['preflight']['encoder_zero_ticks'][7]+=1024
    result=check_direction(obs,raw,q,names,shifted)
    assert not result['passed'] and result['axes'][7]['maximum_mapping_error_rad']>.04
    assert 'unconfirmed' in result['physical_zero']


def test_fourteen_distinct_current_constants_and_delays_predict_new_control_rates(tmp_path):
    """Independent sum-of-step analytic oracle; configuration truth is deliberately wrong."""
    cfg=config();names=[j.name for j in nominal_dual_arm()[0].model.joints]
    cfg['joint_names']=names;cfg['drive_assumptions']['current_tau']=.005;cfg['drive_assumptions']['command_delay_ns']=4_500_000
    true_tau=np.linspace(.0006,.0018,14);true_delay=np.linspace(.0008,.0022,14);gain=2.
    class ZeroBaseline:
        def gravity(self,q):return np.zeros(14)
    obs_times=np.arange(0,320_000_001,1_000_000,dtype=np.int64)
    for name,period in [('driver_timing_train',1_000_000),('driver_timing_validation_2ms',2_000_000),('driver_timing_validation_4ms',4_000_000)]:
        times=np.arange(0,320_000_000,period,dtype=np.int64)
        commands,reference=pulse_commands(times,ZeroBaseline(),np.zeros(14),validation=name!='driver_timing_train')
        current=np.empty((len(obs_times),14))
        for j in range(14):
            increments=np.diff(np.r_[0.,commands[:,j,2]/gain])
            elapsed=np.maximum(obs_times[:,None]*1e-9-times[None,:]*1e-9-true_delay[j],0.)
            current[:,j]=(1-np.exp(-elapsed/true_tau[j]))@increments
        zero=np.zeros_like(current);obs=dict(sample_ns=obs_times,receive_ns=obs_times+700_000,
            sequence=np.arange(len(obs_times),dtype=np.int64),q=zero,dq=zero,current=current,status=np.zeros_like(current,dtype=np.int32))
        _save_run(tmp_path/name,obs,{},times,commands,reference,names,dict(control_period_ns=period))
    report=identify_driver_timing(tmp_path,cfg)
    assert report['passed'] and report['parameters_frozen_for_validation']
    np.testing.assert_allclose([row['fit']['current_tau_s'] for row in report['axes']],true_tau,atol=3e-5)
    np.testing.assert_allclose([row['fit']['command_delay_s'] for row in report['axes']],true_delay,atol=3e-5)
    assert report['feedback_transport_delay_ms']==pytest.approx(.7)
    # A PD torque changes with the plant state, so its sent reference is not a valid current target.
    path=tmp_path/'driver_timing_train/commands.npz'
    with np.load(path) as data:arrays={k:data[k].copy() for k in data.files}
    arrays['commands'][:,0,3]=1.;np.savez_compressed(path,**arrays)
    metadata=tmp_path/'driver_timing_train/metadata.json';record=json.loads(metadata.read_text())
    record['commands_sha256']=hashlib.sha256(path.read_bytes()).hexdigest();metadata.write_text(json.dumps(record))
    with pytest.raises(ValueError,match='torque-only'):identify_driver_timing(tmp_path,cfg)
