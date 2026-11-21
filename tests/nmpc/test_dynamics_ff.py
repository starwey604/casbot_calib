import json
from pathlib import Path

import numpy as np
import pytest

pytestmark = pytest.mark.slow


def test_delivery_inverse_dynamics_and_limit(tmp_path):
    from casbot_calib.model_context import load_context
    from casbot_calib.nmpc.dynamics_ff import Feedforward

    ctx = load_context()
    (tmp_path / 'calibrated.urdf').symlink_to(ctx.urdf)
    runtime = dict(joint_names=list(ctx.joint_names), gravity_world=[0, 0, -9.81],
                   rotor_inertia=[.08]*14, friction=dict(viscous=[.12]*14, coulomb=[.2]*14,
                   smoothing_velocity=[.005]*14, stribeck_velocity=[.1]*14, static_friction=[.2]*14))
    (tmp_path/'runtime_dynamics.json').write_text(json.dumps(runtime))
    ff = Feedforward(dict(smoothing_floor_rad_s=.03), tmp_path)
    q, v, a = ctx.home, np.full(14, .1), np.full(14, .5)
    nominal = ctx.engine.inverse(q, v, a)
    expected = nominal + .08*a + .12*v + .2*np.tanh(v/.03)
    np.testing.assert_allclose(ff.effort(np.r_[q, v, a]), expected, atol=1e-10)
    assert ff.engine.model.joints[0].friction.smoothing_velocity == .03
    with pytest.raises(ValueError, match='力矩'):
        ff.effort(np.r_[q, v, a*1e6])
    ff.enabled = False
    np.testing.assert_array_equal(ff.effort(np.r_[q, v, a]), np.zeros(14))
    runtime['joint_names'].reverse()
    (tmp_path/'runtime_dynamics.json').write_text(json.dumps(runtime))
    with pytest.raises(ValueError, match='轴序'):
        Feedforward(dict(smoothing_floor_rad_s=.03), tmp_path)


def test_nominal_placeholders_and_gravity(tmp_path, monkeypatch):
    from casbot_calib.model_context import load_context
    from casbot_calib.nmpc import dynamics_ff

    monkeypatch.setattr(dynamics_ff, 'ROOT', tmp_path)
    ff = dynamics_ff.Feedforward(dict(smoothing_floor_rad_s=.03,
                                      nominal_dynamics=dict(viscous=.1, rotor_inertia=.2)))
    q = load_context().home
    z = np.zeros(14)
    np.testing.assert_allclose(ff.effort(np.r_[q,z,z]), load_context().engine.gravity(q))
    assert ff.engine.model.joints[0].rotor_inertia == .2
    assert ff.engine.model.joints[-1].friction.viscous == .1
    with pytest.raises((FileNotFoundError, OSError)):
        dynamics_ff.Feedforward(dict(smoothing_floor_rad_s=.03), Path(tmp_path)/'missing')
