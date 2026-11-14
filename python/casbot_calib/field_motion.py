"""Measured-start H1 recipes, shared without SDK launch or record schemas."""
import json
import numpy as np
from .staged_motion import Motion
from .static_field import _move as add_bounded_move


def static_motion(config, start, recipe):
    """Rebuild the same selected static targets from a newly measured start."""
    motion = Motion('hardware_static_sequence', 'gravity', start)
    motion.add(config['initial_hold_s'], start, label='observed_start_hold')
    add_bounded_move(motion, recipe['home_rad'], config['anchor_transition_s'], recipe['planning_envelope'], 'to_preset_anchor')
    trials = []
    for metadata in recipe['motions']:
        name = metadata['trial_name']
        begin = motion.duration; knots = metadata['knots_s']; labels = metadata['labels']; measure = None
        for i, label in enumerate(labels):
            state = np.asarray(metadata['states'][i + 1])
            if np.max(abs(state[1:])) > 1e-12: raise ValueError('Static field expects rest-to-rest preset segments')
            a = motion.duration
            motion.add(knots[i + 1] - knots[i], state[0], label=label)
            if label == 'static_measure': measure = (a, motion.duration)
        if measure is None: raise ValueError('Missing static measurement window')
        trials.append(dict(name=name, conditions=metadata['conditions'], begin_s=begin, end_s=motion.duration,
            measure_begin_s=measure[0], measure_end_s=measure[1]))
    add_bounded_move(motion, start, config['anchor_transition_s'], recipe['planning_envelope'], 'return_observed_start')
    motion.add(config['final_hold_s'], start, label='observed_start_final_hold')
    return motion, trials



def jog_motion(config,start,names,joint_name,amplitude_deg,*,anchor_rad=None):
    motion = Motion('hardware_direction_jog', 'position', start, dict(joint=names.index(joint_name), amplitude_deg=amplitude_deg))
    motion.add(config['initial_hold_s'], start, label='observed_start_hold')
    envelope = dict(speed_rad_s=.05, acceleration_rad_s2=.1, jerk_rad_s3=1.)
    anchor = start if anchor_rad is None else np.asarray(anchor_rad, dtype=float)
    if anchor.shape != (14,) or not np.isfinite(anchor).all(): raise ValueError('Finite 14-axis comparison anchor required')
    if anchor_rad is not None:
        add_bounded_move(motion, anchor, config['anchor_transition_s'], envelope, 'to_comparison_anchor')
    comparison_begin = motion.duration
    comparison = Motion('frequency_reference', 'position', anchor)
    for sign in (1, -1, 0):
        q = anchor.copy(); q[names.index(joint_name)] += sign * np.deg2rad(amplitude_deg)
        add_bounded_move(motion, q, config['anchor_transition_s'], envelope, 'direction_jog')
        motion.add(1., q, label='direction_hold')
        add_bounded_move(comparison, q, config['anchor_transition_s'], envelope, 'direction_jog')
        comparison.add(1., q, label='direction_hold')
    comparison_end = motion.duration
    if anchor_rad is not None:
        add_bounded_move(motion, start, config['anchor_transition_s'], envelope, 'return_measured_start')
    motion.add(config['final_hold_s'], start, label='observed_start_final_hold')
    import hashlib
    definition = dict(joint=joint_name, amplitude_deg=amplitude_deg, anchor_rad=anchor.tolist(),
        knots_s=comparison.knots, states=np.asarray(comparison.states).tolist(), labels=comparison.labels)
    motion.conditions.update(comparison_window_s=[comparison_begin, comparison_end],
        comparison_reference_sha256=hashlib.sha256(json.dumps(definition, sort_keys=True).encode()).hexdigest(),
        comparison_anchor_rad=anchor.tolist())
    recipe = dict(kind='jog', joint=joint_name, amplitude_deg=amplitude_deg,
        anchor_rad=None if anchor_rad is None else anchor.tolist())
    return motion, recipe
