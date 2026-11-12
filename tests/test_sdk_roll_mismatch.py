"""Algebraic attribution and identifiability, without device or capture I/O."""
import numpy as np
import pytest
from casbot_calib.sdk_roll_mismatch import (
    PARAMETERS, normalized, hybrid, fit, shapley, filter_fit, local_information)


def columns(delta=.001, tau=0.):
    w = 2*np.pi*np.array([2.5, 10.])
    v = np.array([.1*np.exp(.7j), .04*np.exp(-.2j)])
    a = 1j*w*v; c = .016/(1j*w)*v
    y = np.exp(1j*w*delta)/(1+1j*w*tau)*(.00027*a+.01*v+c)
    return np.array([y, a, v, c])


def test_phase_normalization_preserves_original_nm_fit():
    z = columns(); n = normalized(z)
    out = hybrid(n, n, 0)
    np.testing.assert_allclose(abs(out), abs(z), atol=1e-16)
    original, rotated = fit(z, [2.5, 10]), fit(out, [2.5, 10])
    for key in PARAMETERS:
        assert rotated[key] == pytest.approx(original[key], rel=1e-7, abs=1e-12)
    assert original['inertia_kg_m2'] == pytest.approx(.00027, rel=1e-7)
    assert original['effective_advance_s'] == pytest.approx(.001, abs=1e-9)


def test_hybrid_endpoints_and_single_factor_selection():
    a = normalized(columns()); b = normalized(columns(-.002, .005))
    np.testing.assert_allclose(hybrid(a, b, 0), hybrid(a, a, 127))
    np.testing.assert_allclose(hybrid(a, b, 127), hybrid(b, b, 0))
    changed = normalized(hybrid(a, b, 1))
    assert abs(changed['h'][0]) == pytest.approx(abs(b['h'][0]))
    assert np.angle(changed['h'][0]) == pytest.approx(np.angle(a['h'][0]))
    np.testing.assert_allclose(changed['h'][1], a['h'][1])
    with pytest.raises(ValueError, match='velocity'):
        normalized(np.zeros((4, 2), complex))


def test_shapley_additive_and_interaction_allocation():
    effects = np.array([[2, -1], [3, 4], [5, 6.]])
    values = np.array([sum((effects[k] for k in range(3) if mask & (1 << k)), start=np.zeros(2))
                       + (np.array([8, 12]) if mask & 3 == 3 else 0) for mask in range(8)])
    actual = shapley(values)
    expected = effects.copy(); expected[:2] += [4, 6]
    np.testing.assert_allclose(actual, expected)
    np.testing.assert_allclose(actual.sum(axis=0), values[-1]-values[0])
    with pytest.raises(ValueError):
        shapley(values[:-1])


@pytest.mark.parametrize('tau', [0., .002, .005, .01])
def test_fixed_filter_recovers_known_parameters_without_reweighting(tau):
    result = filter_fit(columns(tau=tau), [2.5, 10.], tau)
    assert result['solver_converged']
    for key, value in zip(PARAMETERS, [.00027, .01, .001]):
        assert result[key] == pytest.approx(value, rel=1e-6, abs=1e-10)
    assert result['residual_complex_norm_nm'] < 1e-10


def test_delay_filter_degeneracy_survives_third_tone_at_zero_tau():
    for f in ([2.5, 10.], [2.5, 5., 10.]):
        info = local_information(f, 0.)
        assert info['rank'] == 3 and info['null_dimensions'] == 1
        assert info['phase_filter_column_sum_norm'] == 0
    two = local_information([2.5, 10.], .005, free_gain=True)
    three = local_information([2.5, 5., 10.], .005, free_gain=True)
    assert two['rank'] == 4 and two['null_dimensions'] == 1
    assert three['rank'] == 5 and three['real_observations'] == 6
