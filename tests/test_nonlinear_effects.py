"""One-pass Pinocchio nonlinear term preserves separate gravity/friction APIs."""
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.arm_model import load_arm_model, planar_two_link_config
from casbot_calib.dual_model import nominal_dual_arm


@pytest.mark.parametrize('full_tree',[False, True])
def test_one_pass_nonlinear_equals_existing_decomposition_and_inverse(full_tree):
    plant=nominal_dual_arm()[0] if full_tree else _core.ArmDynamics(load_arm_model(planar_two_link_config()))
    n=len(plant.model.joints);workspace=plant.workspace();rng=np.random.default_rng(20261106)
    for _ in range(20):
        q=rng.uniform(-.3,.3,n);v=rng.uniform(-.5,.5,n)
        expected=plant.gravity(q)+plant.velocity_bias(q,v)
        for backend in (plant,workspace):
            np.testing.assert_allclose(backend.nonlinear_effects(q,v),expected,atol=2e-14,rtol=1e-13)
            np.testing.assert_allclose(backend.nonlinear_effects(q,v),backend.inverse(q,v,np.zeros(n))-backend.friction(v),atol=2e-14,rtol=1e-13)
            np.testing.assert_allclose(backend.nonlinear_effects(q,np.zeros(n)),backend.gravity(q),atol=2e-14,rtol=1e-13)
    with pytest.raises(ValueError):workspace.nonlinear_effects(np.zeros(n-1),np.zeros(n))
    with pytest.raises(ValueError):workspace.nonlinear_effects(np.zeros(n),np.full(n,np.nan))
