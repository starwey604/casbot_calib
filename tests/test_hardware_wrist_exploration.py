"""Local wrist H4 fixture and sensitivity conventions; never device I/O."""
import json
import numpy as np
import pytest
from casbot_calib.hardware_wrist_exploration import (
    FLAGS, WRISTS, design, require_local, transfer, perturb, mass_difference, make_motion)
from casbot_calib.sdk_connection import mock_connection
from casbot_calib.staged_motion import derivative_bounds


def test_exploration_rejects_hardware_and_binds_domain():
    with pytest.raises(ValueError, match='isolated local'):
        require_local({}, 227)
    profile = dict(connection=mock_connection('/casbot_calib_s2_unit_test', 227, '0'*64))
    assert all(v.startswith('/casbot_calib_s2_unit_test/') for v in require_local(profile,227).values())
    with pytest.raises(ValueError, match='domain'):
        require_local(profile, 228)


def test_periodic_shift_advances_and_lowpass_has_both_phase_and_amplitude():
    t=np.arange(256)/256*8; w=2*np.pi*3/8
    signal=np.sin(w*t)[:,None]
    assert np.max(abs(transfer(signal,8,shift_s=.015)[:,0]-np.sin(w*(t+.015))))<1e-12
    tau=.02
    expected=np.sin(w*t-np.arctan(w*tau))/np.sqrt(1+(w*tau)**2)
    assert np.max(abs(transfer(signal,8,tau_s=tau)[:,0]-expected))<1e-12


def test_perturb_changes_only_unique_observed_torque_and_rebuilds_target():
    v=np.sin(np.arange(256)/256*2*np.pi)[:,None]*np.ones((1,14))
    problem=dict(observed_torque_nm=v, rotor=np.ones_like(v)*.2,friction=np.ones_like(v)*.1,
                 target=v-.3,regression=np.ones((256,14,140)),moving=np.ones_like(v,dtype=bool))
    out=perturb(problem,8,dict(shift_s=.015))
    assert out['regression'] is problem['regression'] and out['moving'] is problem['moving']
    np.testing.assert_allclose(out['target'],out['observed_torque_nm']-.3,atol=1e-15)
    np.testing.assert_array_equal(problem['observed_torque_nm'],v)


def test_operator_comparison_uses_full_reference_and_original_floor():
    base=np.ones((128,14)); other=base.copy();other[17,12]=1.1032
    result=mass_difference(base,other,1e-6)
    assert result['maximum_relative']==pytest.approx(.1032)
    assert result['worst_joint']==12 and result['worst_point']==17
    assert result['wrist_maximum_relative']==pytest.approx(.1032)


def test_design_freezes_train_validation_and_reports_no_h3_acceptance(tmp_path):
    probes=tmp_path/'probes.npz'
    np.savez(probes,q=np.zeros((128,14)),dq=np.ones((128,14)),ddq=np.ones((128,14)))
    root=tmp_path/'experiment'; p=design(root,probes)
    assert len(p['jobs'])==6 and len(p['cases'])==2 and len(p['variations'])==9
    assert p['cases'][1]==dict(id='cache_5ms',cache_period_ns=5000000)
    train={j['seed'] for j in p['jobs'] if j['split']=='train'}
    validation={j['seed'] for j in p['jobs'] if j['split']=='validation'}
    assert not train & validation
    assert all(p[k] is False for k in FLAGS)
    assert 'not tested' in p['prior_status']
    assert not p['sensitivity_used_for_selection'] and not p['command_control_comparison']
    assert (root/'protocol/manifest.json').exists()
    assert not (root/'cache_3ms').exists()  # design is offline, no SDK session.
    for job in p['jobs']:
        pose=np.array(p['poses'][job['center']]); home=np.array(p['h4']['home_rad'])
        motion=make_motion(home,pose,p['h4'],job); curve=motion.polynomial()
        assert np.max(derivative_bounds(curve,2))<=1.5
        assert np.max(derivative_bounds(curve,3))<=8
        times=np.linspace(0,motion.duration,300)
        held=[j for j in range(14) if j not in WRISTS]
        np.testing.assert_allclose(curve(times,nu=1)[:,held],0.,atol=1e-12)
        np.testing.assert_allclose(curve(motion.duration),home,atol=1e-12)
