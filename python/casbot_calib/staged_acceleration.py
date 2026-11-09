"""Preregistered wrist acceleration and child-posture coverage for P0 development.

Reference design only: no new mechanics, measured acceleration or fit weights.
Protocols without these optional fields retain their original references.
"""
from math import comb
import numpy as np
from scipy.interpolate import BPoly


def validate_design(config):
    p0 = config['p0']
    tone = p0.get('acceleration_tone')
    anchors = p0.get('center_design', {}).get('fixed_joint_offsets_rad', {})
    if tone is not None:
        if not config.get('smooth_periodic_envelope') or config.get('dynamic_ramp_s', 0) <= 0:
            raise ValueError('Acceleration tone requires a bounded C3 entry/braking envelope')
        if set(tone) != {'joints', 'amplitude_rad', 'harmonic_by_split'}:
            raise ValueError('Unknown acceleration-tone setting')
        axes, amplitude, harmonics = tone['joints'], tone['amplitude_rad'], tone['harmonic_by_split']
        if not axes or len(set(axes)) != len(axes) or any(type(j) is not int or not 0 <= j < 14 for j in axes):
            raise ValueError('Explicit unique 14-axis tone indices required')
        if not np.isfinite(amplitude) or amplitude <= 0:
            raise ValueError('Positive finite acceleration-tone amplitude required')
        if set(harmonics) != {'train', 'selection', 'test'} or any(
                type(h) is not int or not 1 <= h <= p0['preprocessing']['harmonics'] for h in harmonics.values()):
            raise ValueError('Tone harmonic must be within the declared matched band for every role')
        if len(set(harmonics.values())) != 3:
            raise ValueError('Independent role tone harmonics required')
        for split, harmonic in harmonics.items():
            if harmonic in ([1, 2, 3, 5] if split == 'train' else [1, 3, 4, 6]):
                raise ValueError('Additional tone duplicates a base harmonic')
    known = {f'{split}_c{i:02}' for split, role in p0['roles'].items() for i in range(role['centers'])}
    if not isinstance(anchors, dict) or set(anchors) - known:
        raise ValueError('Unknown fixed-posture center')
    for entries in anchors.values():
        if not isinstance(entries, dict) or not entries or any(
                type(j) is not str or j not in {str(i) for i in range(14)} or not np.isfinite(v) for j, v in entries.items()):
            raise ValueError('Fixed center joint offsets require finite radians and canonical string indices (JSON stable)')


def center_pool(pool, home, center_id, policy):
    offsets = policy.get('fixed_joint_offsets_rad', {}).get(center_id)
    if offsets is None:
        return pool
    result = np.array(pool, copy=True)
    for joint, offset in offsets.items():
        index = int(joint)
        result[:, index] = home[index] + offset
    return result


def add_tone(motion, config, split, seed):
    """Add analytic tone jets to the existing C3-envelope motion's knot states."""
    spec = config['p0'].get('acceleration_tone')
    if spec is None:
        return motion
    active = motion.conditions['active']
    axes = [j for j in spec['joints'] if active == 'both' or (j < 7) == (active == 'left')]
    harmonic = spec['harmonic_by_split'][split]
    omega = 2 * np.pi * harmonic / config['dynamic_period_s']
    # Independent of the base multisine RNG, so its bins remain unchanged.
    phase = np.random.default_rng(seed + 104729).uniform(-np.pi, np.pi, 14)
    start = motion.windows('periodic')[0][0]
    end = motion.windows('periodic')[-1][1]
    ramp = config['dynamic_ramp_s']
    envelope = BPoly.from_derivatives([0., ramp], [[0., 0., 0., 0.], [1., 0., 0., 0.]])
    for time, state in zip(motion.knots, motion.states):
        if time < start - ramp - 1e-9 or time > end + ramp + 1e-9:
            continue
        if time < start:
            weights = [float(envelope(np.clip(time - (start - ramp), 0, ramp), nu=k)) for k in range(4)]
        elif time <= end:
            weights = [1., 0., 0., 0.]
        else:
            w = [float(envelope(np.clip(time - end, 0, ramp), nu=k)) for k in range(4)]
            weights = [1 - w[0], *[-x for x in w[1:]]]
        angle = omega * (time - start) + phase
        jets = [spec['amplitude_rad'] * omega**k * np.sin(angle + k * np.pi / 2) for k in range(4)]
        for order in range(4):
            state[order, axes] += sum(comb(order, k) * weights[k] * jets[order - k][axes] for k in range(order + 1))
    motion.conditions['acceleration_tone'] = dict(
        harmonic=harmonic, amplitude_rad=spec['amplitude_rad'], joints=axes, phase_seed=seed + 104729,
        scope='reference only, same C3 amplitude envelope; not a measured-acceleration channel')
    return motion
