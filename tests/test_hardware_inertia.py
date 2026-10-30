"""H4 observation math, physical fit and stage contracts; no devices."""
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.dynamics_problem import fourier_projection
from casbot_calib.hardware_periodic import periodic_problem,prediction_metrics
from casbot_calib.hardware_inertia import settings,gravity_constraints
from casbot_calib.rigid_identification import fit_rigid,model_from_parameters

ROOT=Path(__file__).resolve().parents[1]


def config():return yaml.safe_load((ROOT/'configs/hardware_inertia.yaml').read_text())


def test_callback_fourier_projection_has_independent_analytic_derivatives_and_offset():
    origin=8000000000000000;t=np.arange(0.,6.,.002);w=2*np.pi
    time=origin+np.rint(t*1e9).astype(np.int64);y=(.3*np.sin(w*t)+.2*np.cos(2*w*t))[:,None]
    result,meta=fourier_projection(time,y,origin,1.,4,128,True,weights=np.linspace(.5,1.5,len(time)),phase_offset_s=.003)
    phase=meta['phase_s']+.003
    expected=[.3*np.sin(w*phase)+.2*np.cos(2*w*phase),.3*w*np.cos(w*phase)-.4*w*np.sin(2*w*phase),
        -.3*w*w*np.sin(w*phase)-.8*w*w*np.cos(2*w*phase)]
    for actual,value in zip(result,expected):np.testing.assert_allclose(actual[:,0],value,atol=2e-9)


def observations():
    e,_=nominal_dual_arm();c=config();c.update(period_s=4.,periods=4,discard_periods=2,harmonics=12)
    t=np.arange(0.,16.,.002);q=np.tile(c['home_rad'],(len(t),1));v=np.zeros_like(q);a=v.copy()
    for j in (0,7):
        w=2*np.pi/4.;q[:,j]+=.07*np.sin(w*t)+.02*np.cos(3*w*t)
        v[:,j]=.07*w*np.cos(w*t)-.06*w*np.sin(3*w*t);a[:,j]=-.07*w*w*np.sin(w*t)-.18*w*w*np.cos(3*w*t)
    torque=e.evaluate_batch(q,v,a)['total'];obs=dict(receive_ns=6000000000000000+np.rint(t*1e9).astype(np.int64),q=q,dq=v,effort_raw=torque)
    motion=dict(labels=['periodic'],knots_s=[0.,16.]);friction=dict(viscous=[0.]*14,coulomb=[0.]*14,static_friction=[0.]*14)
    return e,c,t,obs,motion,friction


def test_periodic_sdk_data_uses_measured_motion_and_preserves_unknown_sampling():
    e,c,t,obs,motion,f=observations();p=periodic_problem(obs,t,motion,e,c,dict(effort_zero_nm=0.,effort_gain=1.),f)
    report=prediction_metrics(p,e.rigid_parameters(),[0,7],dict(c,validation_absolute_rmse_nm=.001,validation_normalized_rmse=.01))
    assert report['passed'] and not p['preprocessing']['sample_time_known']
    assert p['preprocessing']['independent_acquisition_count'] is None
    assert p['preprocessing']['temporal_rank']==25


def test_callback_gaps_and_asynchronous_velocity_are_rejected():
    e,c,t,obs,motion,f=observations();obs['receive_ns'][6000:]+=100000000
    with pytest.raises(ValueError,match='gap'):periodic_problem(obs,t,motion,e,c,dict(effort_zero_nm=0.,effort_gain=1.),f)
    obs['receive_ns'][6000:]-=100000000;obs['dq'][:,0]+=.2
    with pytest.raises(ValueError,match='velocity'):periodic_problem(obs,t,motion,e,c,dict(effort_zero_nm=0.,effort_gain=1.),f)


@pytest.mark.parametrize('change',[lambda c:c.update(discard_periods=3),lambda c:c['pose_offsets_rad'].update(validation=c['pose_offsets_rad']['train']),
    lambda c:c.update(validation_seeds=c['train_seeds']),lambda c:c['control'].update(kp_factors=[1.])])
def test_invalid_h4_design_fails_before_io(tmp_path,change):
    e,_=nominal_dual_arm();c=config();change(c);path=tmp_path/'h4.yaml';path.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError):settings(path,[j.name for j in e.model.joints])


def test_physical_fit_uses_pinocchio_and_preserves_independent_gravity():
    e,_=nominal_dual_arm();model=e.model;joints=model.joints
    for joint in joints:
        b=joint.body;b.inertia_com=b.inertia_com*1.5;joint.body=b
    model.joints=joints;plant=type(e)(model)
    rng=np.random.default_rng(9201);q=np.tile(config()['home_rad'],(80,1))+rng.uniform(-.1,.1,(80,14));v=rng.normal(0,.1,(80,14));a=rng.normal(0,.5,(80,14))
    y=e.regressor_batch(q,v,a)['regressor'];target=plant.evaluate_batch(q,v,a)['total']
    frozen=gravity_constraints(e,config()['home_rad'])
    fit=fit_rigid(y,target,e.rigid_parameters(),equalities=frozen,ridge=config()['ridge'],physical_length_scale=.1,lift_residuals=True)
    candidate=model_from_parameters(e,fit['parameters'])
    assert fit['rank']<140 and min(fit['minimum_pseudo_inertia_eigenvalues'])>0
    check,_=gravity_constraints(e,config()['home_rad'],9402)
    assert np.max(abs(check@(candidate.rigid_parameters()-e.rigid_parameters())))<1e-6
    assert np.sqrt(np.mean((y@candidate.rigid_parameters()-target)**2))<.001


@pytest.mark.parametrize('kind,inertia_ok,expected',[('control_tuning',False,'inertia_rejected'),('control_validation',True,'controller_rejected')])
def test_failed_stage_blocks_control_before_snapshot_or_sdk_io(tmp_path,monkeypatch,kind,inertia_ok,expected):
    from types import SimpleNamespace
    from casbot_calib import hardware_batch as batch,hardware_inertia as h4,hardware_control as control
    job=dict(id='blocked',kind=kind);protocol=dict(jobs=[job],connection_scope='hardware')
    monkeypatch.setattr(batch,'load_batch',lambda *a,**k:(tmp_path,protocol))
    monkeypatch.setattr(batch,'events',lambda root:[])
    monkeypatch.setattr(batch,'outcomes',lambda *a:({'blocked':[]},[]))
    monkeypatch.setattr(batch,'summarize',lambda root,state:dict(state=state,passed=False))
    monkeypatch.setattr(h4,'freeze_inertia',lambda *a:inertia_ok)
    monkeypatch.setattr(control,'freeze_selection',lambda *a:False)
    def forbidden(*a,**k):raise AssertionError('SDK must not be accessed before stage acceptance')
    backend=SimpleNamespace(local_remaps=None,snapshot=forbidden,execute=forbidden,clear_fault=forbidden)
    assert batch.run_batch(tmp_path,backend=backend)['state']==expected


def test_nonpositive_fourier_exposure_weights_are_rejected():
    t=np.arange(0,4000000000,1000000,dtype=np.int64)
    with pytest.raises(ValueError,match='weights'):fourier_projection(t,np.zeros((len(t),1)),0,1.,4,128,weights=np.zeros(len(t)))


@pytest.mark.parametrize('seed',[401,501,601,701])
def test_h4_c3_envelope_keeps_entry_and_braking_inside_multisine_range(seed):
    from casbot_calib.staged_motion import periodic_motion,restricted_coefficients,derivative_bounds
    c=config();home=np.array(c['home_rad']);amplitude=np.full(14,.08)
    recipe=dict(dynamic_period_s=5.,dynamic_periods=3,pose_amplitude_rad=amplitude,settle_s=3.,
        dynamic_transition_s=4.,smooth_periodic_envelope=True,dynamic_ramp_s=2.)
    motion=periodic_motion('bounded','inertia',home,home,recipe,seed,seed!=401)
    curve=motion.polynomial();coefficients=restricted_coefficients(curve)
    assert np.max(coefficients-home)<=1.2*.08+1e-8
    assert np.min(coefficients-home)>=-1.2*.08-1e-8
    assert np.max(derivative_bounds(curve,2))<=c['maximum_acceleration_rad_s2']
    assert np.max(derivative_bounds(curve,3))<=c['maximum_jerk_rad_s3']
    assert motion.metadata()['initial_final_home']


def test_small_control_difference_is_not_reported_as_stable_benefit():
    from casbot_calib.hardware_control import benefit_interpretation
    report=benefit_interpretation(.000015,dict(prior=.000074,identified=.000052))
    assert report['model_difference_smaller_than_observed_repeat_change']
    assert not report['stable_control_benefit_demonstrated'] and report['control_benefit_status'].startswith('inconclusive')


@pytest.mark.parametrize('enabled',[False,True])
def test_missing_onset_cannot_be_promoted_from_a_disabled_stage_pass(tmp_path,monkeypatch,enabled):
    from casbot_calib import hardware_batch as batch,hardware_field as field
    from casbot_calib.hardware_inertia import snapshot_prior
    from casbot_calib.data import write_json
    from casbot_calib.vendor_identity import sha256
    folder=tmp_path/'report';folder.mkdir()
    write_json(folder/'report.json',dict(passed=True,moving_friction=dict(passed=True),breakaway=dict(passed=True,available=False)))
    field.seal(folder);write_json(tmp_path/'latest.json',dict(report='report',manifest_sha256=sha256(folder/'manifest.json')))
    p=dict(workflow='h3_friction',h3=dict(breakaway=dict(enabled=enabled)))
    monkeypatch.setattr(batch,'load_batch',lambda source:(tmp_path,p));monkeypatch.setattr(batch,'outcomes',lambda *a:({},[]))
    with pytest.raises(ValueError,match='Accepted H3'):snapshot_prior(tmp_path,{}, {},tmp_path/'not_created')
    assert not (tmp_path/'not_created').exists()
