"""One startup policy across SHM calibration pipeline and GUI tools."""
import struct
import numpy as np
import pytest
from casbot_calib.field_defaults import start_velocity_limit
from casbot_calib.pipeline.native import read_start_state
from casbot_calib.pipeline.robot import load_robot_config


@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf'), True])
def test_invalid_startup_threshold(value):
    with pytest.raises(ValueError):
        start_velocity_limit(dict(start_velocity_rad_s=value))


@pytest.mark.parametrize('speed,allowed', [(0.1, True), (0.199999, True), (0.200001, False), (0.21, False)])
def test_shm_measured_start_uses_shared_threshold(tmp_path, speed, allowed):
    c = load_robot_config()
    root = tmp_path / 'snapshot'
    (root / 'native').mkdir(parents=True)
    raw = bytearray(696)
    struct.pack_into('<i', raw, 12 + 12 * 16 + 6, round(speed * 1e6))
    (root / 'native/status.bin').write_bytes(raw)
    if allowed:
        assert len(read_start_state(root, c)) == 14
    else:
        with pytest.raises(ValueError, match='0.2rad/s'):
            read_start_state(root, c)


@pytest.mark.parametrize('speed,starts', [(0.1, True), (0.2, False)])
def test_gui_start_uses_same_default_without_graphics(tmp_path, speed, starts):
    from types import SimpleNamespace
    from casbot_calib.shm_gui import Desk, parser

    root = tmp_path / 'gui'
    root.mkdir()
    desk = Desk(root, parser().parse_args([]), render=False)
    calls = []
    try:
        desk.client = SimpleNamespace(
            hello=dict(version=2, hand_background=True, lower=[-3.0] * 14, upper=[3.0] * 14, velocity=[1.5] * 14),
            failure=None,
            send=lambda *args, **kw: calls.append((args, kw)),
        )
        desk.model_ok = True
        desk.feedback = dict(receive_ns=1, phase=-1, q=[0.0] * 14, dq=[speed] * 14)
        desk.commands = dict(
            position=np.zeros(14), velocity=np.zeros(14), effort=np.zeros(14),
            kp=np.full(14, 20.0), kd=np.full(14, 2.0),
        )
        desk.start()
        assert bool(calls) == starts and desk.start_waiting == (not starts)
        if not starts:
            assert '0.2rad/s' in desk.message
    finally:
        desk.client = None
        desk.close()
