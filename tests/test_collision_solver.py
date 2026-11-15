"""Coal solver settings retain separated geometry at an H3 entry pose."""
import numpy as np
from casbot_calib.model_runtime import collision_for
from casbot_calib.pipeline.robot import load_robot_config


def test_h3_separated_base_and_shoulder_distance_stays_positive():
    # At this pose Coal 3.0.3 with GJK/EPA tolerance 1e-12 returned zero.
    q = np.array([
        -0.0006361332770272488, 0.7871615537255316, 0.007675944754551947,
        -0.7676729265884908, -0.012233515089802544, -0.003973914239518665,
        -0.001829845154906846, -0.01380547295868708, -0.7697597586433719,
        0.00750245169214483, -0.772231321523758, 0.0011814508664195862,
        -0.019122701353646923, 0.019512782299712896,
    ])
    profile = collision_for(load_robot_config())
    first = profile.names.index('robot_base_link_0')
    second = profile.names.index('robot_left_shoulder_roll_link_0')
    pair = list(profile.pairs).index((first, second))
    single = profile.check(q)['distances'][0, pair]
    assert 0.25 < single < 0.26
    batch = profile.check(np.tile(q, (32, 1)))['distances'][:, pair]
    np.testing.assert_allclose(batch, single, rtol=0, atol=1e-12)
