"""Independent linear oracle for the bounded robust estimator, without a plant."""
import numpy as np
import pytest
from casbot_calib.dual_identification import fit_parameters


@pytest.mark.parametrize('active', [None, [0, 1]])
def test_robust_jacobian_does_not_corrupt_the_residual_matrix(active):
    rng = np.random.default_rng(81)
    features = rng.normal(size=(48, 2, 6))
    truth = np.array([1.3, .7, .06, .04, .12, .08])
    start = truth.copy() if active is not None else np.array([1., 1., .01, .01, .02, .02])
    start[:2] = 1.
    original = features.copy()
    torque = features @ truth
    result = fit_parameters(features, torque, start, active)
    assert result['success'] and result['rmse_nm'] < 1e-8
    np.testing.assert_allclose(result['parameters'], truth, atol=1e-8)
    np.testing.assert_array_equal(features, original)
