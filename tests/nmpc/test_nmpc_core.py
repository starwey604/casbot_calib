"""Offline native runtime, certified stopping and kinematic viewer acceptance."""
from importlib.util import find_spec
from time import perf_counter, sleep

import numpy as np
import pytest

pytestmark = pytest.mark.slow


@pytest.fixture
def controller():
    from casbot_calib.nmpc.runtime import BUILD, Controller

    if not (BUILD/'libcasbot_nmpc_runtime.so').is_file() or find_spec('acados_template') is None:
        pytest.skip('Use the host NMPC environment and run scripts/nmpc/run.sh build')
    ctl = Controller()
    try:
        yield ctl
    finally:
        ctl.close()


def test_native_fk_and_distance_gradients(controller):
    from casbot_calib.nmpc.model import forward
    from casbot_calib.model_context import load_context

    ctl = controller
    profile = load_context().collision_profile()
    q = ctl.x[:14] + np.random.default_rng(8).uniform(-.12, .12, 14)
    gaps, gradients, poses = ctl.native.query(q, gradient=True, flange=True)
    np.testing.assert_allclose(poses, forward(ctl.spec, q)[1], atol=1e-12)
    convex = np.array([p['modes']['simulation']['geometry'] == 'convex' for p in ctl.spec['pairs']])
    exact = profile.check(q)['distances'][0]
    np.testing.assert_allclose(gaps[convex], exact[convex], atol=1e-9, rtol=0)
    assert np.all(gaps[~convex] <= exact[~convex] + 1e-9)
    differences = []
    for delta in 1e-6*np.eye(14):
        hi = ctl.native.query(q+delta)[0]
        lo = ctl.native.query(q-delta)[0]
        differences.append((hi-lo)/2e-6)
    np.testing.assert_allclose(gradients, np.array(differences).T, atol=1e-6, rtol=0)


def test_between_substep_clearance(controller):
    from casbot_calib.nmpc.runtime import advance, pointer

    ctl = controller
    q = ctl.x[:14].copy()
    lo, hi = 0., q[1]
    for _ in range(42):
        q[1] = (lo+hi)/2
        if np.min(ctl.native.query(q)[0]-ctl.margin) <= 0:
            lo = q[1]
        else:
            hi = q[1]
    # Parabolic dip across the clearance boundary at t=.5, between .4 and .6.
    q[1] = hi - .00002 + .01*.5**2
    x = np.r_[q, np.zeros(28)]
    x[15], x[29] = -.01, .02
    jerk = np.zeros((1, 14))
    for t in np.linspace(0., 1., 6):
        assert np.min(ctl.native.query(advance(x, jerk[0], t)[:14])[0]-ctl.margin) > 0
    states = np.stack([x, advance(x, jerk[0], 1.)])
    assert ctl.native.lib.nmpc_certify(ctl.native.handle, pointer(states), pointer(jerk),
                                       1, 1., pointer(ctl.native.query(q)[0]), pointer(ctl.radii), pointer(ctl.margin)) == 0


def test_candidate_rejection_keeps_committed_stop(controller, monkeypatch):
    ctl = controller
    target = ctl.poses.copy()
    target[:, 0, 3] += .07
    for _ in range(8):
        ctl.propose(target)
        ctl.commit()
    assert np.max(abs(ctl.x[14:28])) > .001
    old_state, old_backup = ctl.x.copy(), ctl.backup.copy()
    monkeypatch.setattr(ctl, 'check', lambda proposal: False)
    ctl.propose(target)
    assert ctl.mode == 'backup'
    np.testing.assert_array_equal(ctl.x, old_state)
    np.testing.assert_array_equal(ctl.backup, old_backup)
    with pytest.raises(RuntimeError, match='awaits confirmation'):
        ctl.propose(target)
    ctl.commit()
    np.testing.assert_array_equal(ctl.backup[:-1], old_backup[1:])
    for _ in range(len(old_backup)):
        ctl.propose(target, stop=True)
        ctl.commit()
    assert np.max(abs(ctl.x[14:])) < 1e-8


@pytest.mark.parametrize('probe', ['reach', 'cross', 'chest'])
def test_tracking_and_stopping_probes(controller, probe):
    from casbot_calib.nmpc.bridge import filtered_target
    from casbot_calib.model_context import load_context

    ctl = controller
    profile = load_context().collision_profile()
    goal = ctl.poses.copy()
    initial = goal.copy()
    if probe == 'reach':
        goal[:, 0, 3] += .075
    elif probe == 'cross':
        goal[:, 1, 3] = [-.15, .15]
        goal[:, 0, 3] = .42
        goal[:, 2, 3] = 1.28
    else:
        goal[:, :3, 3] = [0, 0, 1.22]
    filtered = initial
    timings = []
    minimum = np.inf
    inter_arm_minimum = np.inf
    parents = np.array([g['parent'] for g in ctl.spec['geometries']])
    arms = parents[np.array([p['indices'] for p in ctl.spec['pairs']])] // 7
    inter_arm = (arms.min(axis=1) == 0) & (arms.max(axis=1) == 1)
    for _ in range(round(4.5/ctl.dt)):
        start = perf_counter()
        filtered = filtered_target(filtered, goal, ctl.dt, ctl.spec['config'])
        ctl.propose(filtered)
        ctl.commit()
        timings.append(1000*(perf_counter()-start))
        slack = ctl.gaps-ctl.margin
        minimum = min(minimum, np.min(slack))
        inter_arm_minimum = min(inter_arm_minimum, np.min(slack[inter_arm]))
        assert np.all(profile.check(ctl.x[:14])['distances'][0] > ctl.margin)
        sleep(max(0., ctl.dt - (perf_counter()-start)))
    assert minimum > 0
    assert np.linalg.norm((ctl.poses-initial)[:, :3, 3]) > .04
    if probe == 'reach':
        assert np.linalg.norm((ctl.poses-goal)[:, :3, 3], axis=1).max() < .012
    else:
        assert np.linalg.norm((ctl.poses-goal)[:, :3, 3]) > .05
    if probe == 'cross':
        assert inter_arm_minimum < .001
    overruns = np.count_nonzero(np.array(timings) > 1000*ctl.dt)
    print(f'{probe}: min slack {1e3*minimum:.3f} mm; median/p95/max ms',
          np.round(np.percentile(timings, [50, 95, 100]), 2),
          f'overruns {overruns}/{len(timings)}')
    for _ in range(len(ctl.backup)):
        ctl.propose(goal, stop=True)
        ctl.commit()
        assert np.all(ctl.gaps > ctl.margin)
    assert np.max(np.abs(ctl.x[14:])) < 1e-8
