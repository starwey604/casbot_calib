#!/usr/bin/env python3
"""Extract and certify compact DVT2 reference clips from local 100 Hz songs."""

import argparse
import csv
from math import comb
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
from scipy.interpolate import BPoly, make_interp_spline
from scipy.optimize import bisect
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation

from casbot_calib.data import write_json
from casbot_calib.model_runtime import collision_for
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.pipeline.verify_dynamics import (
    CLIP_ASSET, CLIP_STATE_RESOLUTION, load_clips, plan_verification,
)
from casbot_calib.shm_data import NAMES, ROOT
from casbot_calib.shm_ready import ready_position
from casbot_calib.staged_motion import Motion, derivative_bounds, gate_motion

HERTZ = 100
DRUM_AXES = [3, 4, 5, 6, 10, 11, 12, 13]
QUAD_X, QUAD_W = np.polynomial.legendre.leggauss(7)
FADE = np.polynomial.Polynomial([0, 0, 0, 10, -15, 6])
KEYBOARD_TIME_SCALE = 1.4
KEYBOARD_FLANGE_Y_M = 0.308


def _read_song(path):
    with path.open(newline='', encoding='utf-8') as stream:
        rows = csv.reader(stream)
        header = next(rows)
        waist = next(name for name in header if name in ('waist_yaw', 'waist_yaw_joint'))
        columns = [header.index(name) for name in (*NAMES, waist)]
        data = np.array([[float(row[index]) for index in columns] for row in rows if row])
    return data[:, :14], data[:, 14]


def _candidate_states(samples, *, cutoff, fade_s, drum):
    raw_time = np.arange(len(samples)) / HERTZ
    duration = raw_time[-1]
    filtered = sosfiltfilt(butter(4, cutoff, fs=HERTZ, output='sos'), samples, axis=0)
    spline = make_interp_spline(raw_time, filtered, k=5, axis=0)
    speed_estimate = np.max(np.abs(spline(raw_time, nu=1)))
    step = 0.08 / max(1.0, speed_estimate / 1.32) if drum else 0.10
    t = np.linspace(0, duration, round(duration / step) + 1)
    states = np.stack([spline(t, nu=d) for d in range(4)], axis=1)
    # Blend to the filtered endpoint pose. The pose difference supplies the
    # fourth zero at each endpoint, making the join stationary through jerk.
    for endpoint, direction in ((0.0, 1), (duration, -1)):
        u = direction * (t - endpoint) / fade_s
        mask = u < 1
        source = states[mask].copy()
        anchor = spline(endpoint)
        source[:, 0] -= anchor
        for d in range(4):
            states[mask, d] = sum(
                comb(d, k) * (FADE.deriv(k)(u[mask]) * (direction / fade_s) ** k)[:, None]
                * source[:, d - k] for k in range(d + 1)
            )
        states[mask, 0] += anchor
    states[[0, -1], 1:] = 0.0
    return t, states


def _score(curve, drum):
    dt = np.diff(curve.x)
    t = curve.x[:-1, None] + dt[:, None] * (QUAD_X + 1) / 2
    derivative = curve(t, nu=2 if drum else 1)
    axis_rms = ((derivative ** 2 * (dt[:, None] * QUAD_W)[..., None]).sum(axis=(0, 1))
                / (2 * curve.x[-1]))
    if drum:
        return float(np.sqrt(axis_rms[DRUM_AXES].mean()))
    left, right = (np.sqrt(side.mean()) for side in np.split(axis_rms, 2))
    return float(min(left, right) + 0.5 * (left + right))


def _clip(samples, drum, source):
    t, states = _candidate_states(samples, cutoff=4.0 if drum else 8.0,
                                  fade_s=0.5 if drum else 0.6, drum=drum)
    scale = 1.0
    if drum:
        speed = derivative_bounds(BPoly.from_derivatives(t, states)).max()
        max_scale = 9.5 / t[-1]
        if speed / max_scale > 1.319:
            return None
        if speed > 1.319:
            # Uniform retiming scales the certified velocity bound by 1/s.
            scale = bisect(lambda s: speed / s - 1.319, 1.0, max_scale) + 1e-10
    knots = np.rint(t * scale * 1e9).astype(np.int64)
    states = states / scale ** np.arange(4)[None, :, None]
    encoded = np.rint(states / CLIP_STATE_RESOLUTION).astype(np.int64)
    decoded = encoded * CLIP_STATE_RESOLUTION
    curve = BPoly.from_derivatives(knots * 1e-9, decoded)
    if derivative_bounds(curve).max() > (1.32 if drum else 1.25):
        return None
    return _score(curve, drum), dict(knots_ns=knots, states=encoded,
                                     source={**source, 'time_scale': scale, 'lowpass_hz': 4.0 if drum else 8.0})


def _ranked_clips(data_dir, geometry, instrument):
    drum = instrument == 'drum'
    lengths = (200, 250, 300, 350) if drum else (700,)
    candidates = []
    for path in sorted(data_dir.glob(f'*{instrument}1*.data')):
        q, waist = _read_song(path)
        clear = waist == 0.0
        for start in range(0, len(q), 512):
            stop = start + 512
            if clear[start:stop].any():
                clear[start:stop] &= geometry.check(q[start:stop])['distances'].min(axis=1) >= 0.0115
        blocked = np.r_[0, np.cumsum(~clear)]
        for length in lengths:
            # Adjacent candidates overlap heavily. Search starts every 0.5 s.
            starts = np.flatnonzero(blocked[length + 1:] == blocked[:-(length + 1)])
            for start in starts[starts % 50 == 0]:
                stop = start + length + 1
                result = _clip(q[start:stop], drum, dict(file=path.name, first_frame=int(start), last_frame=int(stop - 1)))
                if result is not None:
                    candidates.append(result)
        print(f'{path.name} 已筛查', flush=True)
    return [clip for _, clip in sorted(candidates, key=lambda item: -item[0])]


def _safe_clips(candidates, geometry, name):
    for clip in candidates:
        states = clip['states'] * CLIP_STATE_RESOLUTION
        motion = Motion(name, 'verification', states[0, 0])
        motion.states = list(states)
        motion.knots = (clip['knots_ns'] * 1e-9).tolist()
        motion.labels = [name] * (len(states) - 1)
        if gate_motion(geometry, motion, 0.01)['passed']:
            yield clip


def _relocate_keyboard(clip, engine):
    """Translate both flange paths laterally, then refit their joint curves."""
    knots = np.asarray(clip['knots_ns']) * 1e-9
    t = np.linspace(0, knots[-1], round(knots[-1] * HERTZ) + 1)
    original = BPoly.from_derivatives(knots, clip['states'] * CLIP_STATE_RESOLUTION)(t)
    q = original.copy()
    lower = np.array([joint.lower for joint in engine.model.joints])
    upper = np.array([joint.upper for joint in engine.model.joints])
    offsets = []
    weights = np.array([1., 1., 1., .25, .25, .25])
    regularization = .002 ** 2
    for start, side, sign in ((0, 'left', 1), (7, 'right', -1)):
        axes = slice(start, start + 7)
        frame = f'{side}_flange_frame'
        target = engine.frame_batch(original, frame)['poses']
        offset = sign * KEYBOARD_FLANGE_Y_M - target[:, 1, 3].mean()
        target[:, 1, 3] += offset
        offsets.append(float(offset))
        posture = original[:, axes].copy()
        posture[:, 1] += sign * .45
        lo, hi = lower[axes] + .005, upper[axes] - .005
        posture = np.clip(posture, lo, hi)
        q[:, axes] = posture
        for _ in range(35):
            current = engine.frame_batch(q, frame)
            position = target[:, :3, 3] - current['poses'][:, :3, 3]
            relative = target[:, :3, :3] @ current['poses'][:, :3, :3].transpose(0, 2, 1)
            rotation = Rotation.from_matrix(relative).as_rotvec()
            error = np.column_stack([position, rotation]) * weights
            if np.max(np.linalg.norm(error, axis=1)) < 5e-5:
                break
            jacobian = current['jacobians'][:, :, axes] * weights[None, :, None]
            transpose = jacobian.transpose(0, 2, 1)
            rhs = (transpose @ error[:, :, None])[:, :, 0] + regularization * (posture - q[:, axes])
            step = np.linalg.solve(transpose @ jacobian + regularization * np.eye(7), rhs)
            q[:, axes] = np.clip(q[:, axes] + np.clip(step, -.15, .15), lo, hi)
        else:
            return None

    stationary = [(1, np.zeros(14)), (2, np.zeros(14))]
    spline = make_interp_spline(t, q, k=5, bc_type=(stationary, stationary))
    scale = KEYBOARD_TIME_SCALE
    states = np.stack([spline(knots, nu=d) / scale ** d for d in range(4)], axis=1)
    states[[0, -1], 1:] = 0.
    return dict(
        knots_ns=np.rint(knots * scale * 1e9).astype(np.int64),
        states=np.rint(states / CLIP_STATE_RESOLUTION).astype(np.int64),
        source={**clip['source'], 'time_scale': scale, 'fixture': '1105_03', 'flange_lateral_offset_m': offsets},
    )


def extract(data_dir, output):
    robot = load_robot_config()
    geometry = collision_for(robot)
    keyboard = _ranked_clips(data_dir, geometry, 'keyboard')
    drum = _ranked_clips(data_dir, geometry, 'drum')
    for keys in _safe_clips(keyboard, geometry, 'keyboard'):
        keys = _relocate_keyboard(keys, geometry.engine)
        if keys is None:
            continue
        for beats in _safe_clips(drum, geometry, 'drum'):
            with TemporaryDirectory(prefix='casbot-band-clips-') as temp:
                staging = Path(temp) / 'clips.json'
                write_json(staging, dict(keyboard_clip=keys, drum_clip=beats), compact=True)
                try:
                    plan = plan_verification(ready_position(), robot, Path(temp) / 'plan', clips=load_clips(staging))
                except ValueError as error:
                    print(f'完整参考未通过：{error}', flush=True)
                    continue
                output.write_bytes(staging.read_bytes())
                print(f'{output} ({output.stat().st_size} 字节，参考 {plan["duration_ns"] * 1e-9:.2f} 秒)', flush=True)
                return
    raise ValueError('找不到通过完整连续检查的电子琴与架子鼓片段组合')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', type=Path, default=ROOT / 'references/datas')
    parser.add_argument('--output', type=Path, default=CLIP_ASSET)
    args = parser.parse_args()
    extract(args.data_dir, args.output)
