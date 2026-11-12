"""Local pair scope and public signal inspection; no device processes."""
import numpy as np
from casbot_calib.hardware_probe_capture import design,tone,inspect_observations
from casbot_calib.hardware_probe import settings
from test_hardware_probe import fixture,rotor


def test_pair_design_is_exactly_c00_pitch_two_directions_with_5ms_cache(tmp_path):
    p=design(tmp_path/'pair')
    assert [(j['id'],j['direction']) for j in p['jobs']]==[('c00_right_pitch_neg',-1),('c00_right_pitch_pos',1)]
    assert p['joint']==12 and p['pose_rad'][12:14]==[.2,.3]
    assert p['probe_config']['cache_period_ns']==5000000
    assert p['original_inertia_relative_threshold']==.1
    assert p['maximum_trials']==2 and not p['automatic_expansion']
    assert not p['formal_test_consumed'] and not p['physical_device_tested']
    assert not (tmp_path/'pair/mock').exists()


def test_tone_separates_quadratic_trend_from_5hz_force():
    t=np.arange(0.,1.6,.001);t=t+np.random.default_rng(8).uniform(0,.0002,len(t))
    y=.04+.01*t-.002*t*t+.003*np.sin(10*np.pi*t)+.004*np.cos(10*np.pi*t)
    result=tone(t,y,5.)
    np.testing.assert_allclose(result['rms'],.005/np.sqrt(2),atol=1e-12)
    assert result['non_tone_residual_rms'][0]<1e-12


def test_raw_speed_hold_and_tone_are_reported_when_strict_probe_quality_rejects():
    obs,t,meta=fixture();obs['dq'][:,0]=.002
    prior=dict(viscous=[.035]*14,coulomb=[.04]*14,static_friction=[.04]*14)
    report,problem,traces=inspect_observations(obs,t,meta,rotor(),settings(),dict(effort_gain=1.,effort_zero_nm=0.),prior)
    assert problem is None and not report['probe_preprocessing_passed']
    assert not report['raw_checks']['other_axes_held'] and report['maximum_held_speed_rad_s']==.002
    assert report['estimated_effort_5hz_rms_nm']>0
    assert len(traces['dynamic_nm'])>1000
    assert report['independent_acquisition_count'] is None


def test_failed_snapshot_logs_noop_clear_fault_terminates_pair_and_closes_mock(tmp_path,monkeypatch):
    import json
    import pytest
    from types import SimpleNamespace
    from casbot_calib import hardware_probe_capture as capture
    root=tmp_path/'pair';design(root);called=[]
    class Mock:
        def __init__(self,*args):pass
        def __enter__(self):return SimpleNamespace(client_profile='private_mock')
        def __exit__(self,*args):called.append('closed')
    monkeypatch.setattr(capture,'SDKMockSession',Mock)
    def failed_snapshot(*args):
        called.append('snapshot');return dict(passed=False,fault='feedback_stale')
    monkeypatch.setattr(capture.field,'snapshot_field',failed_snapshot)
    def forbidden(*args,**kwargs):raise AssertionError('No commands after failed snapshot')
    monkeypatch.setattr(capture.field,'execute_field',forbidden)
    with pytest.raises(RuntimeError,match='terminate whole session'):capture.collect(root,227)
    assert called==['snapshot','closed']
    result=json.loads((root/'collection.json').read_text())
    assert result['terminal_fault'] and not result['resume_permitted']
    assert result['clear_fault']['status']=='not_implemented' and not result['clear_fault']['performed']
    assert not (root/'c00_right_pitch_pos').exists()
