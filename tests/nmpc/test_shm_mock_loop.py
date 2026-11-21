"""Live physics, remote enqueue confirmation and disable on a private SHM endpoint."""
import json
from importlib.util import find_spec
from time import monotonic, sleep

import numpy as np
import pytest

pytestmark = pytest.mark.slow


@pytest.fixture
def session(tmp_path, monkeypatch):
    from casbot_calib.nmpc import dynamics_ff
    from casbot_calib.nmpc.runtime import BUILD
    from casbot_calib.nmpc.shm_loop import make_client, SHMLoop

    if not (BUILD/'libcasbot_nmpc_runtime.so').is_file() or find_spec('acados_template') is None:
        pytest.skip('Build the local NMPC environment')
    # Pair the nominal physics fixture with its nominal controller on any workstation.
    monkeypatch.setattr(dynamics_ff, 'ROOT', tmp_path)

    def create(name, ff=True):
        return SHMLoop(make_client(tmp_path/name), feedforward=ff)
    return create


def result(loop):
    from pathlib import Path

    root = Path(loop.client.config['output'])
    mock = json.loads((root/'mock/exit.json').read_text())
    assert mock['endpoint'].startswith('/casbot_calib_test_')
    assert not mock['cleanup_errors'] and not Path('/dev/shm', mock['endpoint'][1:]).exists()
    return json.loads((root/'native/result.json').read_text())


def track(loop, goal, seconds):
    from casbot_calib.nmpc.bridge import filtered_target

    ctl = loop.controller
    target = ctl.poses.copy()
    errors, gaps = [], []
    for _ in range(round(seconds/ctl.dt)):
        start = monotonic()
        target = filtered_target(target, goal, ctl.dt, ctl.spec['config'])
        last = loop.last_applied
        q = loop.step(target)
        assert loop.last_applied > last
        errors.append(q-ctl.x[:14])
        gaps.append(loop.measured_gaps-ctl.margin)
        sleep(max(0., ctl.dt-(monotonic()-start)))
    return np.asarray(errors), np.asarray(gaps)


def test_feedforward_tracking_comparison(session):
    errors = []
    for ff in (False, True):
        with session(str(ff), ff) as loop:
            loop.start()
            goal = loop.controller.poses.copy()
            goal[:, 0, 3] += .06
            err, gap = track(loop, goal, 2.)
            errors.append(np.sqrt(np.mean(err**2)))
            assert gap.min() > 0
        assert loop.disable_ack and result(loop)['disable_ack']
    print('五字段跟踪 RMSE rad, 关闭/开启前馈:', errors)
    assert errors[1] < .5*errors[0]


def test_physical_cross_and_backup(session):
    with session('cross') as loop:
        ctl = loop.controller
        loop.start()
        goal = ctl.poses.copy()
        goal[:, :3, 3] = [[.42,-.15,1.28],[.42,.15,1.28]]
        _, gaps = track(loop, goal, 5.)
        parents = np.array([g['parent'] for g in ctl.spec['geometries']])
        arms = parents[np.array([p['indices'] for p in ctl.spec['pairs']])]//7
        inter_arm = (arms.min(axis=1) == 0) & (arms.max(axis=1) == 1)
        assert gaps.min() > 0
        assert gaps[-1,inter_arm].min() < .04
        for _ in range(len(ctl.backup)):
            start = monotonic()
            loop.step(goal, stop=True)
            sleep(max(0., ctl.dt-(monotonic()-start)))
        assert np.max(np.abs(ctl.x[14:])) < 1e-8
    assert loop.disable_ack and result(loop)['disable_ack']


@pytest.mark.parametrize('fault', ['late_ack', 'slow_send', 'disconnect', 'drive'])
def test_unknown_progress_disables_without_commit(session, monkeypatch, fault, tmp_path):
    from casbot_calib.project_paths import ROOT

    loop = session(fault)
    if fault == 'drive':
        config = json.loads((ROOT/'configs/shm/gui_mock.json').read_text())
        config['fault_after_commands'] = 170
        path = tmp_path/'drive-fault.json'
        path.write_text(json.dumps(config))
        loop.client.config['mock_config'] = str(path)
    with pytest.raises((RuntimeError, TimeoutError)):
        with loop:
            loop.start()
            c = loop.controller
            goal = c.poses.copy()
            goal[:, 0, 3] += .06
            initial, backup = c.x.copy(), c.backup.copy()
            if fault == 'late_ack':
                poll = loop.poll
                old = loop.client.revision
                def delayed():
                    s = poll()
                    return dict(s, revision=old) if s['revision'] > old else s
                monkeypatch.setattr(loop, 'poll', delayed)
                loop.step(goal)
            elif fault == 'slow_send':
                send = loop.client.send
                def delayed_send(kind, **payload):
                    if kind == 'fields':
                        sleep(loop.timeout+.02)
                    return send(kind, **payload)
                monkeypatch.setattr(loop.client, 'send', delayed_send)
                loop.step(goal)
            elif fault == 'disconnect':
                loop.client.display.terminate()
                loop.client.display.wait(timeout=2)
                loop.step(goal)
            else:
                while True:
                    initial, backup = c.x.copy(), c.backup.copy()
                    loop.step(goal)
                    sleep(c.dt)
    outcome = result(loop)
    # The persistent drive fault stays at 0x238 instead of acknowledging ready (0x21).
    assert loop.disable_ack == outcome['disable_ack'] == (fault != 'drive')
    np.testing.assert_array_equal(c.x, initial)
    np.testing.assert_array_equal(c.backup, backup)
    if fault == 'drive':
        assert 'drive' in outcome['reason'] or 'mode_lost' in outcome['reason']


def test_hardware_geometry_policy_offline():
    from casbot_calib.model_context import load_context
    from casbot_calib.nmpc.runtime import BUILD, Controller

    if not (BUILD/'libcasbot_nmpc_runtime.so').is_file() or find_spec('acados_template') is None:
        pytest.skip('Build the local NMPC environment')
    ctl = Controller(mode='hardware', physical=True)
    try:
        profile = load_context().collision_profile()
        actual = profile.check(ctl.x[:14])['distances'][0]
        rows = [p['modes']['hardware'] for p in ctl.spec['pairs']]
        convex = np.array([p['geometry'] == 'convex' for p in rows])
        np.testing.assert_equal(ctl.margin, [p['pair_clearance_m'] for p in rows])
        np.testing.assert_allclose(ctl.gaps[convex], actual[convex], atol=1e-9)
        assert np.all(ctl.gaps[~convex] <= actual[~convex]+1e-9)
        assert np.all(ctl.gaps > ctl.planning_margin)
    finally:
        ctl.close()
