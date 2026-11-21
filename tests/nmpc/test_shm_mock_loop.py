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
        goal[:, :3, 3] = [[.193,-.05,.96],[.193,.05,.96]]
        _, gaps = track(loop, goal, 5.)
        names = [g['name'] for g in ctl.spec['geometries']]
        fixture = [i for i, pair in enumerate(ctl.spec['pairs'])
                   if any(names[j].startswith(('fixed:', 'ball:', 'stem:')) for j in pair['indices'])]
        assert gaps.min() > 0
        assert gaps[:,fixture].min() < .04
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
        from casbot_calib.nmpc.model import fixture_profile
        profile = fixture_profile(load_context())
        actual = profile.check(ctl.x[:14])['distances'][0]
        rows = [p['modes']['hardware'] for p in ctl.spec['pairs']]
        convex = np.array([p['geometry'] == 'convex' for p in rows])
        np.testing.assert_equal(ctl.margin, [p['pair_clearance_m'] for p in rows])
        np.testing.assert_allclose(ctl.gaps[convex], actual[convex], atol=1e-9)
        assert np.all(ctl.gaps[~convex] <= actual[~convex]+1e-9)
        assert np.all(ctl.gaps > ctl.planning_margin)
    finally:
        ctl.close()


def test_ready_motion_from_lowered_arms(session, tmp_path, capsys):
    from casbot_calib.model_context import load_context
    from casbot_calib.project_paths import ROOT
    from casbot_calib.shm_ready import ready_position

    loop = session('unfold')
    config = json.loads((ROOT/'configs/shm/gui_mock.json').read_text())
    q = np.array([.074, .092, .032, -.124, .009, -.003, -.002,
                  .058, -.099, -.009, -.179, -.027, 0., -.003])
    config['initial_q'] = config['initial_fields']['position'] = q.tolist()
    config['initial_fields']['effort'] = load_context().engine.gravity(q).tolist()
    path = tmp_path/'lowered.json'
    path.write_text(json.dumps(config))
    loop.client.config['mock_config'] = str(path)
    with loop:
        assert loop.needs_ready
        loop.cfg['tracking_error_rad'] = 1e-6
        loop.start()
        assert '准备段跟踪报警 ID' in capsys.readouterr().err
        proof = json.loads((loop.client.root/'preparation.json').read_text())
        assert proof['fixture']['passed']
        assert loop.last_applied > 0 and loop.client.control
        assert np.max(abs(loop.measured_q-ready_position())) < .05
        assert np.all(loop.measured_gaps > loop.controller.margin)
        loop.next_tracking_warning = 0.
        revision = loop.client.revision
        loop.step(loop.controller.poses.copy())
        assert 'NMPC跟踪报警 ID' in capsys.readouterr().err
        assert loop.client.revision == revision + 1
    assert loop.disable_ack and result(loop)['disable_ack']


def test_ready_return_while_tracking(session):
    with session('return-to-ready') as loop:
        loop.start()
        ctl = loop.controller
        goal = ctl.poses.copy()
        goal[:, 0, 3] += .055
        track(loop, goal, .8)
        assert np.max(abs(ctl.x[14:])) > 1e-4
        modes = []
        for _ in range(250):
            begin = monotonic()
            loop.step(goal, return_ready=True)
            modes.append(ctl.mode)
            if ctl.mode == 'home_done':
                break
            assert ctl.mode != 'home_blocked'
            sleep(max(0., ctl.dt - (monotonic()-begin)))
        assert modes[0] == 'stopping' and 'homing' in modes and modes[-1] == 'home_done'
        np.testing.assert_allclose(ctl.x, np.r_[ctl.spec['q0'], np.zeros(28)], atol=1e-9)
        assert np.max(abs(loop.measured_q-ctl.x[:14])) < .05
    assert loop.disable_ack


def test_measured_deflection_warns_without_abort(session, capsys):
    with session('deflection') as loop:
        loop.start()
        ctl = loop.controller
        s = loop.poll()
        # Simulate external tug past URDF soft limit and inside virtual clearance bubble.
        tugged_limit = np.asarray(s['q'], dtype=float)
        tugged_limit[5] = ctl.upper[5] + 0.05
        loop.feedback(dict(s, q=tugged_limit.tolist()), ctl.x[:14])
        err_limit = capsys.readouterr().err
        assert 'NMPC实测越软限位 ID17' in err_limit

        tugged_margin = np.asarray(s['q'], dtype=float)
        tugged_margin[1] = 0.0  # inward shoulder_roll below ready clearance margin
        loop.feedback(dict(s, q=tugged_margin.tolist()), ctl.x[:14])
        err_margin = capsys.readouterr().err
        assert 'NMPC实测碰撞间距' in err_margin

        # Loop remains alive and keeps accepting commands; non-finite feedback still aborts.
        rev = loop.client.revision
        loop.step(ctl.poses.copy())
        assert loop.client.revision == rev + 1
        bad = np.asarray(s['q'], dtype=float)
        bad[0] = np.nan
        with pytest.raises(RuntimeError, match='非有限值'):
            loop.feedback(dict(s, q=bad.tolist()), ctl.x[:14])
    assert loop.disable_ack and result(loop)['disable_ack']
