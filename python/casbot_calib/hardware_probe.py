"""Non-reversing SDK wrist observations on callback availability time.

A finite-window Fourier basis includes the nonperiodic position drift. Reported
velocity is fitted before analytic differentiation; cached values are never
finite-differenced or deduplicated. Force and all balance columns share one
projection. No device/ROS I/O is implemented here.
"""
from pathlib import Path
from copy import deepcopy
from math import factorial
import numpy as np
import yaml
from .staged_motion import wrist_probe_motion


def settings(path='configs/sdk_wrist_probe.yaml'):
    c = yaml.safe_load(Path(path).read_text())
    if c.get('schema_version') not in ('casbot_sdk_wrist_probe_v1','casbot_sdk_wrist_probe_v2'):
        raise ValueError('Explicit SDK non-reversing wrist configuration required')
    if type(c['cache_period_ns']) is not int or c['cache_period_ns']<=0:
        raise ValueError('Positive integer cache period required')
    p=c['processing']; probe=c['probe']
    if (p['polynomial_degree']!=2 or not 0<p['fit_band_hz'] or
        not 0<p['edge_trim_s']<probe['duration_s']/2 or
        not 0<p['quadrature_period_s']<.5/p['fit_band_hz'] or
        (legacy_quality(c) and any(not np.isfinite(p[k]) or p[k]<=0 for k in (
            'maximum_callback_gap_s','minimum_signed_velocity_rad_s',
            'maximum_held_velocity_rad_s','maximum_velocity_consistency_rad_s')))):
        raise ValueError('Invalid finite-window wrist processing settings')
    return c



from .probe_projection import legacy_quality


def field_probe_config(config):
    """New field observations keep numerical processing, not empirical gates."""
    c=deepcopy(config);c['schema_version']='casbot_sdk_wrist_probe_v2'
    for key in ('maximum_callback_gap_s','minimum_signed_velocity_rad_s','maximum_held_velocity_rad_s','maximum_velocity_consistency_rad_s'):
        c['processing'].pop(key,None)
    return c

def make_probe(name, home, pose, joint, direction, *, config=None, transition_s=4., settle_s=3.):
    """Use the existing C3 waveform and its original dedicated probe envelope."""
    c=settings() if config is None else config
    recipe=dict(wrist_probe=c['probe'],transition_s=transition_s,settle_s=settle_s)
    motion=wrist_probe_motion(name,home,pose,recipe,joint,direction,
        dict(refine_position_bounds=True,observation_protocol='sdk_callback_nonreversing_v1'))
    # The native recipe carries a4kHz planning hint. It is NOT an SDK sampling
    # period and must not leak into the callback observation contract.
    motion.conditions.pop('sample_period_ns')
    return motion


from .probe_projection import basis,exposure,least_squares,probe_problem,inertia_balance
