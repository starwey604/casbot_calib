"""Wave 1B verification: CVXPY conic engine, O(1) septic peaks, batched collision gates, and CMA-ES excitation."""
import numpy as np
import pytest

pytest.importorskip('cvxpy')
pytest.importorskip('cma')

from casbot_calib.collision import certify_reference, nominal_profile
from casbot_calib.dual_excitation import (
    optimize_excitation_phases,
    periodic_reference,
    stratified_harmonics,
)
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.rigid_identification import fit_rigid, model_from_parameters
from casbot_calib.staged_motion import Motion, derivative_bounds, gate_motion
from casbot_calib.staged_motion import _SEPTIC_ANALYTIC_PEAKS, _SEPTIC_BERNSTEIN8_PEAKS, _move


def _perturbed_dual_arm_engine(nominal, scale_seed=19):
    rng = np.random.default_rng(scale_seed)
    theta_nom = nominal.rigid_parameters().reshape(14, 10).copy()
    masses = theta_nom[:, 0] * rng.uniform(0.88, 1.15, size=14)
    theta_true = theta_nom.copy()
    for j in range(14):
        ratio = masses[j] / theta_nom[j, 0]
        theta_true[j] *= ratio
        theta_true[j, 1:4] += masses[j] * rng.uniform(-0.008, 0.008, size=3)
        theta_true[j, [4, 6, 9]] += masses[j] * 0.002
    true_engine = model_from_parameters(nominal, theta_true.ravel())
    return true_engine, theta_true.ravel()


def test_cvxpy_huber_rejects_gross_torque_outliers_and_sample_weights_work():
    nominal, _ = nominal_dual_arm()
    true_engine, theta_true = _perturbed_dual_arm_engine(nominal)
    prior = nominal.rigid_parameters()

    rng = np.random.default_rng(42)
    center = np.deg2rad([0.0, 35.0, 0.0, -40.0, 0.0, 0.0, 0.0, 0.0, -35.0, 0.0, -40.0, 0.0, 0.0, 0.0])
    q_train = center + rng.uniform(-0.35, 0.35, (60, 14))
    q_val = center + rng.uniform(-0.35, 0.35, (30, 14))
    z_train = np.zeros_like(q_train)
    z_val = np.zeros_like(q_val)

    y_train = nominal.regressor_batch(q_train, z_train, z_train)['regressor']
    y_val = nominal.regressor_batch(q_val, z_val, z_val)['regressor']
    clean_train = true_engine.gravity_batch(q_train)
    clean_val = true_engine.gravity_batch(q_val)

    noisy_train = clean_train + rng.normal(0.0, 0.015, size=clean_train.shape)
    flat_noisy = noisy_train.ravel().copy()
    outlier_idx = rng.choice(len(flat_noisy), size=int(0.05 * len(flat_noisy)), replace=False)
    flat_noisy[outlier_idx] += rng.choice([-4.0, 4.0], size=len(outlier_idx))

    fit_l2 = fit_rigid(y_train, flat_noisy, prior, robust_scale=None, ridge=1e-4)
    fit_huber = fit_rigid(y_train, flat_noisy, prior, robust_scale=0.05, ridge=1e-4)

    rmse_l2 = float(np.sqrt(np.mean((y_val @ np.asarray(fit_l2['parameters']) - clean_val) ** 2)))
    rmse_huber = float(np.sqrt(np.mean((y_val @ np.asarray(fit_huber['parameters']) - clean_val) ** 2)))
    assert rmse_huber < 0.4 * rmse_l2
    assert min(fit_huber['minimum_pseudo_inertia_eigenvalues']) > 0.0

    weights = np.ones_like(flat_noisy)
    weights[outlier_idx] = 0.0
    fit_weighted = fit_rigid(
        y_train,
        flat_noisy,
        prior,
        robust_scale=0.05,
        ridge=1e-4,
        sample_weights=weights,
    )
    rmse_weighted = float(np.sqrt(np.mean((y_val @ np.asarray(fit_weighted['parameters']) - clean_val) ** 2)))
    assert rmse_weighted < rmse_huber
    with pytest.raises(ValueError, match='sample weights'):
        fit_rigid(y_train, flat_noisy, prior, sample_weights=-np.ones_like(flat_noisy))


def test_dpp_ridge_grid_selects_optimal_ridge():
    nominal, _ = nominal_dual_arm()
    true_engine, _ = _perturbed_dual_arm_engine(nominal, scale_seed=73)
    prior = nominal.rigid_parameters()

    rng = np.random.default_rng(108)
    center = np.deg2rad([0.0, 35.0, 0.0, -40.0, 0.0, 0.0, 0.0, 0.0, -35.0, 0.0, -40.0, 0.0, 0.0, 0.0])
    q_train = center + rng.uniform(-0.3, 0.3, (80, 14))
    q_val = center + rng.uniform(-0.3, 0.3, (32, 14))
    y_train = nominal.regressor_batch(q_train, np.zeros_like(q_train), np.zeros_like(q_train))['regressor']
    y_val = nominal.regressor_batch(q_val, np.zeros_like(q_val), np.zeros_like(q_val))['regressor']

    signs = rng.choice([-1.0, 1.0], size=(80, 14))
    hyst_true = np.full(14, 0.12)
    extra_train = np.zeros((80, 14, 14), dtype=float)
    idx = np.arange(14)
    extra_train[:, idx, idx] = signs
    target_train = true_engine.gravity_batch(q_train) + signs * hyst_true + rng.normal(0.0, 0.04, size=(80, 14))
    target_val = true_engine.gravity_batch(q_val)

    aux = (extra_train.reshape(-1, 14), np.zeros(14), np.zeros(14), np.full(14, 3.0))
    grid = [1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1]

    fit = fit_rigid(
        y_train,
        target_train,
        prior,
        auxiliary=aux,
        robust_scale=0.05,
        ridge_grid=grid,
        validation_data=(y_val, target_val),
    )
    assert len(fit['ridge_scan']) == len(grid)
    assert fit['scaled_prior_ridge'] in grid
    assert fit['validation_rmse_nm'] < float(np.sqrt(np.mean((y_val @ prior - target_val) ** 2)))
    np.testing.assert_allclose(fit['auxiliary_parameters'], hyst_true, atol=0.03)


def test_static_move_matches_bernstein_bounds():
    assert all(b >= a for b, a in zip(_SEPTIC_BERNSTEIN8_PEAKS, _SEPTIC_ANALYTIC_PEAKS))
    envelope = dict(speed_rad_s=0.6, acceleration_rad_s2=1.8, jerk_rad_s3=14.0)
    rng = np.random.default_rng(55)
    for _ in range(12):
        q0 = rng.uniform(-0.5, 0.5, 14)
        q1 = rng.uniform(-0.5, 0.5, 14)
        m_fast = Motion('fast', 'gravity', q0)
        _move(m_fast, q1, 0.4, envelope, 'approach')

        unit = Motion('unit', 'gravity', q0).add(1.0, q1).polynomial()
        peaks = [float(np.max(derivative_bounds(unit, k))) for k in (1, 2, 3)]
        limits = [envelope['speed_rad_s'], envelope['acceleration_rad_s2'], envelope['jerk_rad_s3']]
        expected_dur = max(0.4, *(float((p / lim) ** (1.0 / k)) for k, (p, lim) in enumerate(zip(peaks, limits), 1))) * (1.0 + 1e-10)
        assert m_fast.duration == pytest.approx(expected_dur, rel=1e-14, abs=1e-15)
        poly = m_fast.polynomial()
        for k, lim in enumerate(limits, 1):
            assert np.max(derivative_bounds(poly, k)) <= lim * (1.0 + 1e-8)


def test_gate_motion_stationary_short_circuit_and_batched_equivalence():
    engine, _ = nominal_dual_arm()
    profile = nominal_profile(engine)
    if hasattr(profile, '_stationary_pose_cache'):
        profile._stationary_pose_cache.clear()

    home = np.deg2rad([0.0, 25.0, 0.0, -20.0, 0.0, 0.0, 0.0, 0.0, -25.0, 0.0, -20.0, 0.0, 0.0, 0.0])
    near = home.copy()
    near[0] += 0.25
    pose = home.copy()
    pose[0] += 0.20

    motion = Motion('safe_trial', 'gravity', home)
    motion.add(0.5, home, label='initial_hold')
    motion.add(2.0, near, label='approach')
    motion.add(0.8, pose, label='paired_approach')
    motion.add(1.0, pose, label='settle')
    motion.add(1.0, pose, label='static_measure')
    motion.add(2.0, home, label='return_home')
    motion.add(0.5, home, label='final_hold')

    gate = gate_motion(profile, motion, clearance_m=0.01)
    assert gate['passed'] and len(gate['segments']) == 7
    # Stationary holds: initial_hold queries home once (1), settle queries pose once (1),
    # static_measure reuses pose (0), final_hold reuses home (0).
    assert gate['segments'][0]['queries'] == 1
    assert gate['segments'][3]['queries'] == 1
    assert gate['segments'][4]['queries'] == 0
    assert gate['segments'][6]['queries'] == 0

    # Repeat gate on cached poses -> 0 stationary queries.
    hold_only = Motion('cached_home', 'gravity', home).add(1.0, home, label='hold')
    hold_gate = gate_motion(profile, hold_only, clearance_m=0.01)
    assert hold_gate['passed'] and hold_gate['queries'] == 0

    # Moving segments match certify_reference exactly.
    curve = motion.polynomial()
    bounds = derivative_bounds(curve)
    for seg_idx in (1, 2, 5):
        a, b = motion.knots[seg_idx], motion.knots[seg_idx + 1]
        ref = certify_reference(
            profile,
            lambda t, a=a: curve(t + a),
            b - a,
            bounds[seg_idx],
            clearance_m=0.01,
            initial_step_s=b - a,
        )
        got = gate['segments'][seg_idx]
        assert got['passed'] == ref['passed']
        assert got['queries'] == ref['queries']
        assert got['certified_intervals'] == ref['certified_intervals']
        assert got['minimum_sampled_distance_m'] == pytest.approx(ref['minimum_sampled_distance_m'], abs=1e-14)
        assert got['certified_lower_bound_m'] == pytest.approx(ref['certified_lower_bound_m'], abs=1e-14)
        assert got['witness']['first'] == ref['witness']['first']
        assert got['witness']['second'] == ref['witness']['second']

    # Colliding trajectory (arms Straight Down at 0 rad collide with hips) stops at first failing segment.
    bad_target = np.zeros(14)
    bad_motion = Motion('colliding_trial', 'gravity', home)
    bad_motion.add(0.5, home, label='initial_hold')
    bad_motion.add(2.0, bad_target, label='into_hip_collision')
    bad_motion.add(1.0, bad_target, label='never_reached_hold')
    bad_motion.add(2.0, home, label='return_home')
    bad_gate = gate_motion(profile, bad_motion, clearance_m=0.01)
    assert not bad_gate['passed']
    assert len(bad_gate['segments']) == 2
    assert bad_gate['segments'][0]['passed'] and not bad_gate['segments'][1]['passed']
    assert bad_gate['segments'][1]['reason'] == 'collision_or_clearance'


@pytest.mark.slow
def test_stratified_harmonics_and_cma_es_phase_optimization():
    t = np.linspace(0.0, 4.0, 65)
    center = np.deg2rad([0.0, 45.0, 0.0, -45.0, 0.0, 0.0, 0.0, 0.0, -45.0, 0.0, -45.0, 0.0, 0.0, 0.0])
    q0, v0, a0, j0 = periodic_reference(t, center, period_s=4.0, amplitude=0.1, seed=27, validation=False, active='both')
    eps = 1e-6
    qp, vp, ap, _ = periodic_reference(t + eps, center, period_s=4.0, amplitude=0.1, seed=27, validation=False, active='both')
    qm, vm, am, _ = periodic_reference(t - eps, center, period_s=4.0, amplitude=0.1, seed=27, validation=False, active='both')
    np.testing.assert_allclose((qp - qm) / (2 * eps), v0, atol=1e-7)
    np.testing.assert_allclose((vp - vm) / (2 * eps), a0, atol=1e-6)
    np.testing.assert_allclose((ap - am) / (2 * eps), j0, atol=1e-5)

    h_train = stratified_harmonics(14, validation=False)
    h_val = stratified_harmonics(14, validation=True)
    assert h_train.shape == (14, 4) and h_val.shape == (14, 4)
    for j in (0, 1, 2, 3, 7, 8, 9, 10):
        assert list(h_train[j]) == [1, 2, 3, 5]
        assert list(h_val[j]) == [1, 3, 4, 6]
    for j in (4, 5, 6, 11, 12, 13):
        assert list(h_train[j]) == [2, 4, 6, 9]
        assert list(h_val[j]) == [2, 5, 7, 10]

    engine, _ = nominal_dual_arm()
    profile = nominal_profile(engine)
    opt = optimize_excitation_phases(
        engine,
        profile,
        center,
        period_s=6.0,
        amplitude=0.15,
        seed=401,
        clearance_m=0.01,
        velocity_limits=0.5,
        acceleration_limit=2.5,
        jerk_limit=12.0,
        sample_points=40,
        popsize=10,
        max_generations=14,
    )
    assert opt['optimized_condition_number'] < opt['initial_condition_number']
    assert opt['minimum_distance_m'] >= opt['clearance_m']

    dense_t = np.linspace(0.0, 6.0, 500)
    q, dq, ddq, jrk = periodic_reference(
        dense_t,
        center,
        period_s=6.0,
        amplitude=0.15,
        seed=401,
        phases=opt['phases'],
        harmonics_override=opt['harmonics'],
        amplitude_scales=opt['amplitude_scales'],
    )
    limits = np.array([[jt.lower, jt.upper] for jt in engine.model.joints])
    assert np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1])
    assert float(np.max(np.abs(dq))) <= 0.5 + 1e-9
    assert float(np.max(np.abs(ddq))) <= 2.5 + 1e-9
    assert float(np.max(np.abs(jrk))) <= 12.0 + 1e-9


def test_bernstein_motion_has_consistent_derivatives_and_valid_segment_bounds():
    from casbot_calib.staged_motion import Motion, derivative_bounds

    m=Motion('test','oracle',np.zeros(2));m.add(1.,[.3,-.2]).add(.5,[.4,-.1],velocity=[.1,.1]).add(1.,[0,0])
    c=m.polynomial();d=c.derivative();bound=np.max(abs(d.c),axis=0)
    for i,(a,b) in enumerate(zip(m.knots[:-1],m.knots[1:])):
        t=np.linspace(a,b,501);assert np.all(abs(c(t,nu=1))<=bound[i]+1e-12)
        mid=(a+b)/2;eps=1e-5
        np.testing.assert_allclose((c(mid+eps)-c(mid-eps))/(2*eps),c(mid,nu=1),rtol=1e-6,atol=1e-8)
    for i,t in enumerate(m.knots):
        for order in range(4):np.testing.assert_allclose(c(t,nu=order),m.states[i][order],atol=1e-10)
    for order in (1,2,3):
        tightened=derivative_bounds(c,order)
        for i,(a,b) in enumerate(zip(m.knots[:-1],m.knots[1:])):
            assert np.all(abs(c(np.linspace(a,b,501),nu=order))<=tightened[i]+1e-10)
        assert np.max(tightened)<=np.max(abs(c.derivative(order).c))*(1+1e-8)
