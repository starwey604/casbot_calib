"""Phase 0 acceptance on the host. Run pytest -o addopts='' tests/nmpc."""
import json
from pathlib import Path
import time

import numpy as np
import pytest
import yaml

pytestmark = pytest.mark.slow


@pytest.fixture(scope='module')
def imported():
    from casbot_calib.model_context import load_context
    from casbot_calib.nmpc.import_dvt2 import CONFIG, import_model

    ctx = load_context()
    return ctx, import_model(ctx, yaml.safe_load(CONFIG.read_text()))


def test_flange_poses(imported):
    from casbot_calib.nmpc.model import FLANGES, forward

    context, model = imported
    engine = context.engine
    bounds = np.asarray(model['bounds'])
    qs = np.random.default_rng(42).uniform(*bounds.T, (100, 14))
    for q in qs:
        _, flanges = forward(model, q)
        for pose, name in zip(flanges, FLANGES):
            np.testing.assert_allclose(pose, engine.frame_pose(q, name), rtol=0, atol=1e-10)


def test_viewer_contains_only_installed_tube(imported, tmp_path):
    import mujoco
    from casbot_calib.nmpc.mocap_viewer import make_scene

    ctx, spec = imported
    scene = mujoco.MjModel.from_xml_path(str(make_scene(ctx, spec, tmp_path)))
    names = [scene.geom(i).name for i in range(scene.ngeom)]
    assert [n for n in names if n.startswith(('fixture_', 'datum_'))] == ['fixture_square_tube']


def test_exact_pairs_and_proxies(imported):
    from casbot_calib import _core

    ctx, model = imported
    from casbot_calib.nmpc.model import fixture_profile
    profile = fixture_profile(ctx)
    pairs = model['pairs']
    assert [tuple(p['indices']) for p in pairs] == profile.pairs
    assert len(pairs) == 518
    fixture_names = [n for n in profile.names if n.startswith(('fixed:', 'stem:', 'ball:'))]
    assert fixture_names == ['fixed:10010002000']
    tube = profile.names.index(fixture_names[0])
    assert sum(tube in pair for pair in profile.pairs) == 16
    assert model['scene_geometry_sha256'] == profile.metadata['geometry_sha256']
    assert max(g['max_vertex_residual_m'] for g in model['geometries']) <= 1e-10
    starts = np.array([model['q0'], model['home']])
    exact = profile.check(starts)['distances']
    for i, p in enumerate(pairs):
        np.testing.assert_allclose(p['convex_gap_m'], exact[:, i], atol=1e-12, rtol=0)
        # Decreasing signed gap is guaranteed while the hulls are separated.
        assert np.all(np.array(p['capsule_gap_m']) <= exact[:, i] + 1e-10)
        for policy in p['modes'].values():
            margin = policy['pair_clearance_m']
            assert 0 < margin < exact[:, i].min()
            assert (policy['geometry'] == 'capsule') == (min(p['capsule_gap_m']) > margin)
        if any(policy['geometry'] == 'convex' for policy in p['modes'].values()):
            geometries = [profile.geometry[j] for j in p['indices']]
            single = _core.CollisionChecker(profile.engine, geometries, [])
            result = single.check_batch(starts)
            lengths = np.linalg.norm(result['first_point'] - result['second_point'], axis=1)
            np.testing.assert_allclose(lengths, exact[:, i], atol=1e-9, rtol=0)

    pair = tuple(profile.names.index('robot_right_' + part + '_link_0')
                 for part in ('shoulder_yaw', 'wrist_yaw'))
    audit = pairs[profile.pairs.index(pair)]
    assert audit['convex_gap_m'][1] > .015 and audit['capsule_gap_m'][1] < 0
    assert audit['modes']['simulation']['geometry'] == 'convex'


def test_host_mock_feedback(tmp_path):
    from casbot_calib.project_paths import ROOT
    from casbot_calib.shm_gui_client import RemoteClient

    executor = ROOT / 'build/nmpc/shm-host/bin/shm_executor'
    if not executor.is_file():
        pytest.skip('Build the host SHM binaries with scripts/nmpc/setup.sh')
    remote_root = tmp_path / 'remote'
    client = RemoteClient(dict(mock=True, executor=str(executor), output=str(remote_root)),
                          tmp_path / 'client')
    try:
        client.connect()
        end = time.monotonic() + 12
        while not client.failure and time.monotonic() < end:
            if client.latest and client.latest['master_frame'] > 1:
                break
            time.sleep(.01)
        assert client.failure is None, client.failure
        assert client.ready['mock'] and client.ready['endpoint'].startswith('/casbot_calib_test_')
        assert client.latest['master_frame'] > 1 and client.latest['receive_ns'] > 0
        assert client.latest['writes'] == 0
        assert np.isfinite(client.latest['q']).all()
    finally:
        client.close()
    result = json.loads((remote_root / 'mock/exit.json').read_text())
    assert result['returncode'] == 0 and not result['cleanup_errors']
    assert not Path('/dev/shm', result['endpoint'][1:]).exists()
