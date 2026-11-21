"""Arm encoding constants and wire35 single-frame layouts."""
from .model_sources import model_directory

from .project_paths import ROOT
URDF = model_directory() / 'casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2/model.urdf'
DIRECTION = (1, -1, -1, -1, -1, 1, 1, -1, -1, -1, 1, -1, -1, 1)
OFFSET = (-.019698, .00643665, .005416, .007078, .026015, -.025037, -.007323,
          -.013976, -.00224065, -.01494833, .007841, -.01723733, -.01549655, .00541233)
PARTS = ('肩pitch', '肩roll', '肩yaw', '肘pitch', '腕yaw', '腕pitch', '腕roll')
NAMES = tuple(f'{side}_{part}_joint' for side in ('left', 'right') for part in
              ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow_pitch', 'wrist_yaw', 'wrist_pitch', 'wrist_roll'))

try:
    import numpy as _np
except ImportError:  # Bare system python3 before venv bootstrap
    JOINT_COMMAND_DTYPE = None
    JOINT_STATUS_DTYPE = None
    HAND_JOINT_COMMAND_DTYPE = None
    HAND_COMMAND_DTYPE = None
    HAND_JOINT_STATUS_DTYPE = None
    HAND_STATUS_DTYPE = None
    COMMAND_FRAME_DTYPE = None
    STATUS_FRAME_DTYPE = None
else:
    JOINT_COMMAND_DTYPE = _np.dtype([
        ('control_word', '<u2'),
        ('position', '<i4'),
        ('velocity', '<i4'),
        ('torque', '<i2'),
        ('mode', '<i2'),
        ('kp', '<i2'),
        ('kd', '<i2'),
    ], align=False)

    JOINT_STATUS_DTYPE = _np.dtype([
        ('status_word', '<u2'),
        ('position', '<i4'),
        ('velocity', '<i4'),
        ('torque', '<i2'),
        ('mode', '<i2'),
        ('error_code', '<u2'),
    ], align=False)

    HAND_JOINT_COMMAND_DTYPE = _np.dtype([
        ('target', '<i2'),
        ('param', '<i2'),
        ('mode', 'u1'),
        ('options', 'u1'),
    ], align=False)

    HAND_COMMAND_DTYPE = _np.dtype([
        ('control_word', '<u2'),
        ('hand_id', 'u1'),
        ('reserved', 'u1'),
        ('joints', HAND_JOINT_COMMAND_DTYPE, (6,)),
    ], align=False)

    HAND_JOINT_STATUS_DTYPE = _np.dtype([
        ('position', '<i2'),
        ('velocity', '<i2'),
        ('current', '<i2'),
        ('status', '<u2'),
    ], align=False)

    HAND_STATUS_DTYPE = _np.dtype([
        ('global_status', '<u2'),
        ('error_code', '<u2'),
        ('joints', HAND_JOINT_STATUS_DTYPE, (6,)),
    ], align=False)

    COMMAND_FRAME_DTYPE = _np.dtype([
        ('frame_counter', '<u8'),
        ('joint_count', 'u1'),
        ('reserved0', 'u1'),
        ('hand_count', '<u2'),
        ('joints', JOINT_COMMAND_DTYPE, (35,)),
        ('hands', HAND_COMMAND_DTYPE, (2,)),
        ('reserved1', 'u1', (6,)),
        ('timestamp', '<u8'),
        ('crc', '<u4'),
        ('reserved2', 'u1', (4,)),
    ], align=False)

    STATUS_FRAME_DTYPE = _np.dtype([
        ('frame_counter', '<u8'),
        ('joint_count', 'u1'),
        ('reserved0', 'u1'),
        ('hand_count', '<u2'),
        ('joints', JOINT_STATUS_DTYPE, (35,)),
        ('hands', HAND_STATUS_DTYPE, (2,)),
        ('reserved1', 'u1', (4,)),
        ('timestamp', '<u8'),
        ('crc', '<u4'),
        ('reserved2', 'u1', (4,)),
    ], align=False)

    assert JOINT_COMMAND_DTYPE.itemsize == 18
    assert JOINT_STATUS_DTYPE.itemsize == 16
    assert HAND_JOINT_COMMAND_DTYPE.itemsize == 6
    assert HAND_COMMAND_DTYPE.itemsize == 40
    assert HAND_JOINT_STATUS_DTYPE.itemsize == 8
    assert HAND_STATUS_DTYPE.itemsize == 52
    assert COMMAND_FRAME_DTYPE.itemsize == 744
    assert STATUS_FRAME_DTYPE.itemsize == 696
