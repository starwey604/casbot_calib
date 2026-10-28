"""Independent spectral/component attribution oracles and failure examples."""
import numpy as np
import pytest
from casbot_calib.staged_residuals import energy_spectrum, frequency_budget, aggregate_benefit, residual_statistics, same_band, raw_residual_budget, matched_components
from casbot_calib.staged_inertia import periodic_problem
from casbot_calib.dual_model import nominal_dual_arm


def policy():
    return dict(minimum_speed_bin_rms_rad_s=.001,minimum_acceleration_bin_rms_rad_s2=.001,
        minimum_qualified_bins=1,band_non_degradation_factor=1.05,band_non_degradation_additive_nm=.002,
        minimum_inertia_band_improvement_fraction=.01)


def signals(joints=2):
    t=np.arange(1024)/1024;wave=np.sin(2*np.pi*3*t)[:,None]*np.ones((1,joints))
    return wave


def budget(observed,predictions,proxy=None):
    wave=signals(observed.shape[1])
    return frequency_budget(observed,predictions,wave,wave,wave,wave if proxy is None else proxy,[3],8,
        np.full((8,observed.shape[1]),1e-4),policy(),period_s=1.)


def test_one_sided_parseval_dc_excitation_and_other_energy_close():
    t=np.arange(1024)/1024
    value=(.3+2*np.sin(6*np.pi*t)+.2*np.cos(10*np.pi*t))[:,None]
    power=energy_spectrum(value)
    np.testing.assert_allclose(power[[0,3,5],0],[.09,2.,.02],atol=1e-14)
    assert abs(power.sum()-np.mean(value**2))<1e-14
    # Nyquist is single weighted, unlike interior positive-frequency bins.
    assert energy_spectrum(((-1.)**np.arange(1024))[:,None])[-1,0]==1.


def test_only_bias_improvement_cannot_become_inertia_band_benefit():
    wave=signals();observed=wave
    predictions=dict(A=wave+.4+.02*wave,B=wave+.02*wave,C=wave+.02*wave,D=wave+.02*wave)
    stats=residual_statistics(observed,predictions)
    assert np.mean(stats['B']['rmse_nm'])<np.mean(stats['A']['rmse_nm'])*.1
    result=aggregate_benefit({'trial':budget(observed,predictions)},policy())
    assert result['status']=='prediction_equivalence'
    assert not result['inertia_benefit_claim_supported']


def test_only_friction_improvement_is_joint_benefit():
    wave=signals();observed=wave
    result=aggregate_benefit({'trial':budget(observed,dict(A=1.1*wave,B=1.1*wave,C=1.1*wave,D=wave))},policy())
    assert result['status']=='joint_model_benefit_only'
    assert not result['inertia_benefit_claim_supported']


def test_analytic_known_inertia_perturbation_has_band_benefit():
    # tau = J * qdd, independent of Mock, Pinocchio and the regressor.
    acceleration=signals();observed=.02*acceleration
    result=aggregate_benefit({'trial':budget(observed,dict(A=.025*acceleration,B=observed,C=observed,D=observed))},policy())
    assert result['inertia_benefit_claim_supported']
    assert result['improvement_fraction']['B']>.999999


def test_weak_axis_remains_in_report_and_blocks_full_domain_benefit():
    wave=signals();proxy=wave.copy();proxy[:,1]*=1e-6
    row=budget(wave,dict(A=wave*1.1,B=wave,C=wave,D=wave),proxy)
    result=aggregate_benefit({'trial':row},policy())
    assert row['axis_sufficient']==[True,False]
    assert result['qualified_subset_inertia_improvement'] and not result['inertia_benefit_claim_supported']
    assert result['missing_axis_evidence']==[dict(trial='trial',joint=1)]
    assert result['status']=='insufficient_signal'


def test_same_band_enforced_before_comparison():
    with pytest.raises(ValueError,match='identical'):same_band(dict(period_s=1.,harmonics=40),dict(period_s=1.,harmonics=20))
    wave=signals();signatures={k:dict(period_s=1.,phase_points=1024,harmonics=8) for k in 'ABCD'};signatures['B']['harmonics']=7
    with pytest.raises(ValueError,match='identical'):
        frequency_budget(wave,{k:wave for k in 'ABCD'},wave,wave,wave,wave,[3],8,np.full((8,2),1e-4),policy(),period_s=1.,signatures=signatures)


def test_outside_band_raw_energy_not_lost_and_repeated_noise_not_cancelled():
    t=np.arange(1024)/1024;low=signals();high=.2*np.sin(200*np.pi*t)[:,None]*np.ones((1,2))
    currents=np.stack([low+high,low-high]);raw=dict(current_coefficients=np.fft.rfft(currents,axis=1)/1024,
        samples_per_period=1024,periods=2,nyquist_hz=512.)
    result=raw_residual_budget(raw,{k:low for k in 'ABCD'},8)
    np.testing.assert_allclose(result['models']['D']['raw_full_period_rmse_nm'],.2/np.sqrt(2),atol=1e-14)
    np.testing.assert_allclose(result['models']['D']['raw_outside_retained_rmse_nm'],.2/np.sqrt(2),atol=1e-14)


def test_nonlinear_features_precede_projection_and_rotor_is_counted_once():
    nominal,_=nominal_dual_arm();t=np.arange(16000)/4000
    q=(.005/(2*np.pi)*(-np.cos(2*np.pi*t))-.1/(80*2*np.pi)*np.cos(80*2*np.pi*t))[:,None]*np.ones((1,14))
    velocity=(.005*np.sin(2*np.pi*t)+.1*np.sin(80*2*np.pi*t))[:,None]*np.ones((1,14))
    times=np.arange(len(t),dtype=np.int64)*250000
    obs=dict(sample_ns=times,receive_ns=times,sequence=np.arange(len(t)),status=np.zeros((len(t),14),dtype=np.int64),q=q,dq=velocity,current=np.zeros_like(q))
    meta=dict(knots_s=[0.,4.],labels=['periodic'],conditions=dict(period_s=1.))
    p=periodic_problem(nominal,obs,meta,1.,np.full(14,.1),harmonics=4,phase_points=256)
    raw_feature=np.sign(velocity)*(1-np.exp(-(velocity/.002)**2))
    coeff=np.fft.rfft(raw_feature[8000:].reshape(2,4000,14),axis=1).mean(axis=0)/4000
    padded=np.zeros((129,14),complex);padded[:5]=coeff[:5]
    expected=np.fft.irfft(padded*256,n=256,axis=0)
    np.testing.assert_allclose(p['extra'][:,:,14],np.column_stack([expected[:,0],np.zeros((256,13))]),atol=1e-12)
    wrong=np.sign(p['dq'])*(1-np.exp(-(p['dq']/.002)**2))
    assert np.max(abs(expected-wrong))>.2
    friction=np.r_[np.full(14,.01),np.full(14,.02)]
    parts,pred,audit=matched_components(nominal,nominal.rigid_parameters(),friction,p,4)
    np.testing.assert_allclose(parts['rigid_inertia']+parts['rotor'],
        __import__('casbot_calib.staged_inertia',fromlist=['_project'])._project(nominal.evaluate_batch(p['q'],p['dq'],p['ddq'])['inertia'],4),atol=1e-12)
    assert audit['maximum_matched_closure_nm']<1e-10
