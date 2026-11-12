"""Roll screen reuses the recipe, not the pitch model or its qualification."""
import numpy as np
import pytest
from casbot_calib.hardware_probe_capture import design_roll_screen,inspect_observations
from casbot_calib.hardware_probe_campaign import design as campaign_design,source_binding
from casbot_calib.hardware_probe import settings
from casbot_calib.hardware_wrist_exploration import read
from test_hardware_probe_campaign import source_fixture
from test_hardware_probe import fixture,rotor


def test_roll_screen_is_joint13_two_trials_and_does_not_require_pitch_candidate(tmp_path):
    source=tmp_path/'pair';source_fixture(source)
    probes=tmp_path/'points.npz';np.savez(probes,q=np.zeros((128,14)),dq=np.ones((128,14)),ddq=np.ones((128,14)))
    campaign=tmp_path/'campaign';campaign_design(source,campaign,probes)
    assert not (campaign/'analysis/development_candidate').exists()
    output=tmp_path/'roll';p=design_roll_screen(campaign,output)
    assert p['joint']==13 and p['joint_label']=='右腕 roll'
    assert [(j['id'],j['direction']) for j in p['jobs']]==[('c00_right_roll_neg',-1),('c00_right_roll_pos',1)]
    assert p['pose_rad'][12:14]==[.2,.3] and p['probe_config']['cache_period_ns']==5000000
    assert not p['pitch_candidate_used_for_roll'] and not p['full_rigid_fit_planned']
    assert p['original_inertia_relative_threshold']==.1 and p['maximum_trials']==2
    assert (output/'protocol/base_profile.yaml').exists() and not (output/'mock').exists()
    # This role cannot silently enter the existing right-pitch campaign.
    with pytest.raises(ValueError,match='two-probe'):source_binding(output)


def test_inspection_uses_roll_channel_and_keeps_pitch_among_held_axes():
    obs,t,meta=fixture();meta['conditions']['joint']=13
    for key in ('q','dq','effort_raw'):
        obs[key][:,13]=obs[key][:,4];obs[key][:,4]=0.
    prior=dict(viscous=[.035]*14,coulomb=[.04]*14,static_friction=[.04]*14)
    report,problem,_=inspect_observations(obs,t,meta,rotor(),settings(),dict(effort_gain=1.,effort_zero_nm=0.),prior)
    assert report['probe_preprocessing_passed'] and problem['joint']==13
    assert '12' in report['held_joint_maximum_speed_rad_s'] and '13' not in report['held_joint_maximum_speed_rad_s']
    assert report['observed_minus_frozen_gravity_friction_5hz_rms_nm']>1e-4
    assert abs(report['inertia_diagnostic']['inertia_kg_m2']/5e-5-1)<.02


def test_phase_mixing_formula_explains_apparent_inertia_without_model_refit():
    J=2.7e-4;b=.01;omega=10*np.pi;delta=.005
    t=np.arange(0,2.,.001);v=.18+.004*omega*np.cos(omega*t);a=-.004*omega**2*np.sin(omega*t)
    shifted=J*(-.004*omega**2*np.sin(omega*(t+delta)))+b*(.18+.004*omega*np.cos(omega*(t+delta)))+.005
    observed=np.linalg.lstsq(np.column_stack([a,v,np.ones(len(t))]),shifted,rcond=None)[0][0]
    expected=J*np.cos(omega*delta)+b/omega*np.sin(omega*delta)
    assert observed==pytest.approx(expected,rel=1e-12)
    assert abs(observed/J-1)>.1
