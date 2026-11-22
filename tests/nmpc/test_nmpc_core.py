"""Offline native runtime, certified stopping and kinematic viewer acceptance."""
from importlib.util import find_spec

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
    from casbot_calib.nmpc.model import fixture_profile
    profile = fixture_profile(load_context())
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


@pytest.mark.parametrize('probe', ['reach', 'cross', 'chest', 'fixture'])
def test_tracking_and_stopping_probes(controller, probe):
    from casbot_calib.nmpc.bridge import filtered_target
    from casbot_calib.model_context import load_context

    ctl = controller
    from casbot_calib.nmpc.model import fixture_profile
    profile = fixture_profile(load_context())
    goal = ctl.poses.copy()
    initial = goal.copy()
    if probe == 'reach':
        goal[:, 0, 3] += .075
    elif probe == 'cross':
        goal[:, 1, 3] = [-.15, .15]
        goal[:, 0, 3] = -.12
        goal[:, 2, 3] = .95
    elif probe == 'fixture':
        goal[0, :3, 3] = [.193, 0., .96]
    else:
        goal[:, :3, 3] = [0, 0, 1.22]
    filtered = initial
    minimum = np.inf
    fixture = np.array([any(profile.names[j].startswith(('fixed:', 'ball:', 'stem:')) for j in p)
                        for p in profile.pairs])
    fixture_minimum = np.inf
    inter_arm_minimum = np.inf
    parents = np.array([g['parent'] for g in ctl.spec['geometries']])
    arms = parents[np.array([p['indices'] for p in ctl.spec['pairs']])] // 7
    inter_arm = (arms.min(axis=1) == 0) & (arms.max(axis=1) == 1)
    for _ in range(round(4.5/ctl.dt)):
        filtered = filtered_target(filtered, goal, ctl.dt, ctl.spec['config'])
        ctl.propose(filtered)
        ctl.commit()
        slack = ctl.gaps-ctl.margin
        minimum = min(minimum, np.min(slack))
        fixture_minimum = min(fixture_minimum, np.min(slack[fixture]))
        inter_arm_minimum = min(inter_arm_minimum, np.min(slack[inter_arm]))
        assert np.all(profile.check(ctl.x[:14])['distances'][0] > ctl.margin)
    assert minimum > 0
    assert np.linalg.norm((ctl.poses-initial)[:, :3, 3]) > .04
    if probe == 'reach':
        assert np.linalg.norm((ctl.poses-goal)[:, :3, 3], axis=1).max() < .012
    else:
        assert np.linalg.norm((ctl.poses-goal)[:, :3, 3]) > .05
    if probe == 'fixture':
        assert fixture_minimum < .005
    if probe == 'cross':
        assert inter_arm_minimum < .001
    for _ in range(len(ctl.backup)):
        ctl.propose(goal, stop=True)
        ctl.commit()
        assert np.all(ctl.gaps > ctl.margin)
    assert np.max(np.abs(ctl.x[14:])) < 1e-8


def test_ready_return_rejects_blocked_path(controller, monkeypatch):
    ctl = controller
    start = ctl.x.copy()
    monkeypatch.setattr(ctl, 'check', lambda plan, dt=None: False)
    ctl.propose(ctl.poses, return_ready=True)
    ctl.commit()
    assert ctl.mode == 'home_blocked'
    np.testing.assert_array_equal(ctl.x, start)


def test_runtime_orientation_switch(controller):
    from scipy.spatial.transform import Rotation

    ctl = controller
    goal = ctl.poses.copy()
    goal[0, :3, :3] = goal[0, :3, :3] @ Rotation.from_rotvec([.25, 0., 0.]).as_matrix()
    initial = ctl.x.copy()
    ctl.set_orientation_tracking(False)
    for _ in range(10):
        ctl.propose(goal)
        ctl.commit()
    np.testing.assert_allclose(ctl.x, initial, atol=1e-8)
    ctl.set_orientation_tracking(True)
    for _ in range(35):
        ctl.propose(goal)
        ctl.commit()
    residual = Rotation.from_matrix(goal[0, :3, :3].T @ ctl.poses[0, :3, :3]).magnitude()
    assert residual < .14


def test_release_snap_and_tether():
    from casbot_calib.nmpc.interaction import ReleaseSnap, tether

    pose = np.eye(4)
    snap = ReleaseSnap()
    assert not snap.update(True, pose, 0.)
    assert not snap.update(True, pose, .8)
    assert not snap.update(False, pose, .9)
    pose[0, 3] += .025
    assert not snap.update(False, pose, 1.31)
    assert not snap.update(False, pose, 1.65)
    assert snap.update(False, pose, 1.72)
    assert not snap.update(False, pose, 2.2)
    assert not snap.update(True, pose, 2.3)
    assert not snap.update(False, pose, 2.4)
    assert not snap.update(True, pose, 2.5)
    assert not snap.update(True, pose, 3.1)
    np.testing.assert_allclose(tether(np.array([.3, .4, 0]), np.zeros(3)), [.06, .08, 0.])


def test_limit_indicators(controller):
    from casbot_calib.nmpc.interaction import LimitIndicators

    ctl = controller
    indicators = LimitIndicators(ctl)
    q, gaps = ctl.x[:14].copy(), ctl.gaps.copy()
    q[5] = ctl.upper[5] - .01
    right_only = next(i for i in indicators.by_arm[1] if i not in indicators.by_arm[0])
    gaps[right_only] = ctl.planning_margin[right_only]
    assert indicators.read(ctl, q, gaps) == [5, -1, -1, right_only]


def test_other_arm_keeps_moving_when_one_arm_hits_boundary(controller):
    from casbot_calib.nmpc.bridge import filtered_target

    ctl = controller
    ctl.set_orientation_tracking(False)
    goal = ctl.poses.copy()
    goal[0, :3, 3] = [0.6, 0.4, 1.5]
    filtered = ctl.poses.copy()
    for _ in range(90):
        filtered = filtered_target(filtered, goal, ctl.dt, ctl.spec['config'])
        ctl.propose(filtered)
        ctl.commit()
    right_start = ctl.poses[1, :3, 3].copy()
    goal[1, :3, 3] = right_start + np.array([0.08, -0.08, 0.08])
    for _ in range(35):
        filtered = filtered_target(filtered, goal, ctl.dt, ctl.spec['config'])
        ctl.propose(filtered)
        ctl.commit()
        assert np.all(ctl.gaps > ctl.planning_margin)
    assert np.linalg.norm(ctl.poses[1, :3, 3] - right_start) > 0.08


def test_ready_return_plans_around_blocked_direct_chord(controller):
    from casbot_calib.nmpc.bridge import filtered_target

    ctl = controller
    ctl.set_orientation_tracking(False)
    goal = ctl.poses.copy()
    goal[:, :3, 3] = [0.0, 0.0, 1.22]
    filtered = ctl.poses.copy()
    for _ in range(60):
        filtered = filtered_target(filtered, goal, ctl.dt, ctl.spec['config'])
        ctl.propose(filtered)
        ctl.commit()
    for _ in range(400):
        ctl.propose(ctl.poses, return_ready=True)
        ctl.commit()
        assert np.all(ctl.gaps > ctl.planning_margin)
        if ctl.mode in ('home_done', 'home_blocked'):
            break
    assert ctl.mode == 'home_done'
    np.testing.assert_allclose(ctl.x[:14], ctl.spec['q0'], atol=1e-7)
