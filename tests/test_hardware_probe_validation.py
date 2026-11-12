"""Frozen parameter scoring and finite-window effort timing diagnostics."""
import numpy as np
import pytest
from casbot_calib.hardware_probe import probe_problem,settings
from casbot_calib.hardware_probe_validation import score,design
from casbot_calib.hardware_wrist_exploration import read
from casbot_calib import hardware_field as field
from casbot_calib.data import write_json
from casbot_calib.hardware_probe_campaign import design as campaign_design,export_candidate
from test_hardware_probe_campaign import source_fixture
from test_hardware_probe import rotor,fixture


def test_effort_diagnostic_offset_changes_force_only_and_zero_preserves_default():
    e=rotor();obs,t,meta=fixture();c=settings();profile=dict(effort_gain=1.,effort_zero_nm=0.)
    p=probe_problem(obs,t,meta,e,c,profile);zero=probe_problem(obs,t,meta,e,c,profile,effort_relative_offset_s=0.)
    offset=probe_problem(obs,t,meta,e,c,profile,effort_relative_offset_s=.005)
    for k in ('regression','extra','rotor','q','dq','ddq','observed_torque_nm','target'):np.testing.assert_array_equal(p[k],zero[k])
    for k in ('regression','extra','rotor','q','dq','ddq'):np.testing.assert_array_equal(p[k],offset[k])
    assert np.max(abs(p['observed_torque_nm']-offset['observed_torque_nm']))>1e-5
    np.testing.assert_allclose(offset['target'],offset['observed_torque_nm']-offset['rotor'],atol=1e-15)
    with pytest.raises(ValueError,match='trimmed'):probe_problem(obs,t,meta,e,c,profile,effort_relative_offset_s=.3)


def test_frozen_prediction_and_rigid_inertia_account_for_rotor_once(monkeypatch):
    from casbot_calib import rigid_identification
    def forbidden(*args,**kwargs):raise AssertionError('No candidate fitting during validation')
    monkeypatch.setattr(rigid_identification,'fit_rigid',forbidden)
    e=rotor();model=e.model;joints=model.joints;joints[4].rotor_inertia=2e-5;model.joints=joints;e=type(e)(model)
    obs,t,meta=fixture();sample=np.floor((t-.0013)/.005)*.005+.0013
    obs['effort_raw'][:,4]+=2e-5*(-.004*(10*np.pi)**2*np.sin(10*np.pi*sample))
    p=probe_problem(obs,t,meta,e,settings(),dict(effort_gain=1.,effort_zero_nm=0.))
    candidate=dict(parameters=e.rigid_parameters().tolist(),viscous=[.035]*14,coulomb=[.04]*14,static_friction=[.04]*14)
    before=np.array(candidate['parameters'])
    result,prediction,error=score(p,candidate,e,dict(maximum_relative_inertia_disagreement=.1,maximum_torque_rmse_nm=.002))
    assert result['passed'] and result['relative_inertia_disagreement']<.02
    np.testing.assert_array_equal(before,candidate['parameters'])
    assert result['frozen_candidate_mean_rigid_inertia_kg_m2']==pytest.approx(5e-5)


def test_third_pose_freezes_candidate_and_remains_independent_of_training_folder(tmp_path):
    pair=tmp_path/'pair';source_fixture(pair)
    probes=tmp_path/'points.npz';np.savez(probes,q=np.zeros((128,14)),dq=np.ones((128,14)),ddq=np.ones((128,14)))
    campaign=tmp_path/'train';old=campaign_design(pair,campaign,probes)
    with np.load(campaign/'protocol/coordinates.npz') as d:theta=d['prior'].tolist()
    analysis=campaign/'analysis';analysis.mkdir()
    write_json(analysis/'report.json',dict(all_operator_deletions_passed=True,variants=[dict(fit=dict(parameters=theta))]))
    export_candidate(campaign,old,analysis,theta,True);field.seal(analysis)
    root=tmp_path/'validation';p=design(campaign,root)
    assert p['pose_rad'][12]==0 and p['pose_rad'][13]==pytest.approx(.225)
    assert all(p['pose_rad']!=pose for pose in old['poses'].values())
    assert len(p['jobs'])==2 and p['acceptance']['maximum_relative_inertia_disagreement']==.1
    assert not p['candidate_parameter_updates_allowed'] and not p['diagnostics_used_for_selection']
    frozen=read(root/'protocol/candidate/candidate.json')
    assert frozen['parameters']==theta
    # The new bundle remains complete after its original source is unavailable.
    campaign.rename(tmp_path/'train_unavailable')
    field.check_seal(root/'protocol');assert read(root/'protocol/candidate/candidate.json')['parameters']==theta
    assert not (root/'mock').exists()
