"""Frozen 35-slot raw-record decoding metadata for the 2026-11-13 profile."""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
URDF = Path(__file__).resolve().parent / 'assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2/model.urdf'
DIRECTION = (1, -1, -1, -1, -1, 1, 1, -1, -1, -1, 1, -1, -1, 1)
OFFSET = (-.019698, .00643665, .005416, .007078, .026015, -.025037, -.007323,
          -.013976, -.00224065, -.01494833, .007841, -.01723733, -.01549655, .00541233)
PARTS = ('肩pitch', '肩roll', '肩yaw', '肘pitch', '腕yaw', '腕pitch', '腕roll')
NAMES = tuple(f'{side}_{part}_joint' for side in ('left', 'right') for part in
              ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow_pitch', 'wrist_yaw', 'wrist_pitch', 'wrist_roll'))
