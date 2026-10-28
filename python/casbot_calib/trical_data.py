"""Immutable SI-normalised three-indicator observations, independent of Mock truth.

Rows are measurement records, not a declaration of simultaneous acquisition.
Each indicator retains its own clock, sequence and status. Unknown sample time=-1.
"""
from enum import IntEnum
from hashlib import sha256
import json
from pathlib import Path
import numpy as np
from .data import write_json
from .kinematics import calibration, digest, validate_config


class IndicatorStatus(IntEnum):
    VALID = 0
    NO_CONTACT = 1
    OUT_OF_RANGE = 2
    STALE = 3
    DISCONNECTED = 4


ARRAYS = {
    'q_rad', 'dq_rad_s', 'joint_sequence', 'joint_sample_ns', 'joint_receive_ns',
    'joint_sample_known', 'indicator_readings_m', 'indicator_sequence', 'indicator_sample_ns',
    'indicator_receive_ns', 'indicator_sample_known', 'indicator_status', 'ball_id', 'tool_frame',
}


def validate_observations(arrays, metadata):
    if set(arrays) != ARRAYS:
        raise ValueError(f'Unexpected observation fields: {set(arrays) ^ ARRAYS}')
    if metadata['schema_version'] != 1 or metadata['schema'] != 'trical_observations_v1':
        raise ValueError('Unsupported TriCal observation schema')
    expected_units = dict(position='m', angle='rad', angular_velocity='rad/s', time='ns')
    if metadata['units'] != expected_units:
        raise ValueError('TriCal observations must use explicit SI units')
    names = metadata['joint_names']
    if len(names) != 14 or len(set(names)) != 14:
        raise ValueError('Expected 14 distinct joints in declared model order')
    if names != metadata['geometry']['joint_names'] or metadata['geometry_sha256'] != metadata['geometry']['geometry_sha256']:
        raise ValueError('Joint order or geometry identity mismatch')
    if names != [j['name'] for j in metadata['geometry']['geometry']['joints']]:
        raise ValueError('Declared joint order differs from geometry content')
    if digest(metadata['geometry']['geometry']) != metadata['geometry_sha256']:
        raise ValueError('Geometry content digest mismatch')
    config = metadata['profile']; validate_config(config)
    if metadata['profile_sha256'] != digest(config):
        raise ValueError('Measurement profile identity mismatch')
    geometry = metadata['geometry']
    if geometry['robot_model_id'] != config['robot_model_id'] or geometry['urdf_sha256'] != config['urdf_sha256']:
        raise ValueError('Measurement profile and robot source mismatch')
    if metadata['indicator_channels'] != ['x', 'y', 'z'] or metadata['status_codes'] != {x.name: int(x) for x in IndicatorStatus}:
        raise ValueError('Indicator channel/status convention mismatch')
    a = {key: np.asarray(value) for key, value in arrays.items()}; n = len(a['q_rad'])
    if n == 0:
        raise ValueError('Empty observation batch')
    for key in ARRAYS:
        width = 14 if key in {'q_rad', 'dq_rad_s'} else 3 if key.startswith('indicator_') else None
        shape = (n,) if width is None else (n, width)
        if a[key].shape != shape or a[key].dtype.hasobject:
            raise ValueError(f'Invalid shape/dtype for {key}')
    for key in ('q_rad', 'dq_rad_s', 'indicator_readings_m'):
        if a[key].dtype.kind != 'f':
            raise ValueError(f'{key} must be floating-point SI values')
    if not np.isfinite(a['q_rad']).all() or not np.isfinite(a['dq_rad_s']).all():
        raise ValueError('Invalid joint observation')
    for key in ('joint_sequence', 'joint_sample_ns', 'joint_receive_ns', 'indicator_sequence', 'indicator_sample_ns', 'indicator_receive_ns'):
        if a[key].dtype != np.dtype('int64'):
            raise ValueError(f'{key} requires signed int64, without timestamp coercion')
    for key in ('joint_sample_known', 'indicator_sample_known'):
        if a[key].dtype != np.dtype('bool'):
            raise ValueError(f'{key} requires boolean values')
    if a['indicator_status'].dtype != np.dtype('uint8') or not np.isin(a['indicator_status'], list(IndicatorStatus)).all():
        raise ValueError('Unknown indicator status')
    clocks = metadata['clocks']
    if set(clocks) != {'joint_sample', 'indicator_sample', 'receive'} or len(clocks['indicator_sample']) != 3:
        raise ValueError('Declare joint, three indicator and receive clock domains')
    for clock in [clocks['joint_sample'], *clocks['indicator_sample'], clocks['receive']]:
        if not isinstance(clock, str) or not clock:
            raise ValueError('Clock domains must be nonempty strings; use unknown explicitly')
    if clocks['receive'] == 'unknown':
        raise ValueError('Reception clock must be declared')
    for prefix in ('joint', 'indicator'):
        known = a[prefix + '_sample_known']; sample = a[prefix + '_sample_ns']; received = a[prefix + '_receive_ns']
        if np.any(sample[~known] != -1) or np.any(sample[known] < 0) or np.any(received < 0):
            raise ValueError('Unknown sample time must be -1; known/reception times must be nonnegative')
        if np.any(a[prefix + '_sequence'] < 0) or np.any(np.diff(received, axis=0) < 0):
            raise ValueError('Negative sequence or reordered receive time')
    if clocks['joint_sample'] == 'unknown' and a['joint_sample_known'].any():
        raise ValueError('Unknown joint clock cannot expose known sampling times')
    for channel, clock in enumerate(clocks['indicator_sample']):
        if clock == 'unknown' and a['indicator_sample_known'][:, channel].any():
            raise ValueError('Unknown indicator clock cannot expose known sampling times')
    for sample, received, known, clock in [(a['joint_sample_ns'], a['joint_receive_ns'], a['joint_sample_known'], clocks['joint_sample'])] + [
            (a['indicator_sample_ns'][:, i], a['indicator_receive_ns'][:, i], a['indicator_sample_known'][:, i], clock)
            for i, clock in enumerate(clocks['indicator_sample'])]:
        if clock == clocks['receive'] and np.any(sample[known] > received[known]):
            raise ValueError('Sample cannot follow reception in the same clock domain')
    valid = a['indicator_status'] == IndicatorStatus.VALID
    readings = a['indicator_readings_m']; cal = calibration(config)
    if not np.isfinite(readings[valid]).all():
        raise ValueError('Valid readings must be finite')
    low = np.array(config['indicators']['minimum_readings_m']); high = np.array(config['indicators']['maximum_readings_m'])
    if np.any(((readings < low) | (readings > high)) & valid):
        raise ValueError('Out-of-range readings cannot be marked valid')
    if a['ball_id'].dtype.kind != 'U' or not set(a['ball_id']).issubset({b['id'] for b in config['fixture']['balls']}):
        raise ValueError('Unknown datum ball ID')
    if a['tool_frame'].dtype.kind != 'U' or not set(a['tool_frame']).issubset({t['name'] for t in config['tools']}):
        raise ValueError('Unknown measurement tool frame')
    if not np.isfinite(cal.zero).all():
        raise ValueError('Invalid mastering')
    return dict(records=n, fully_valid_records=int(valid.all(axis=1).sum()),
        unknown_indicator_sample_times=int((~a['indicator_sample_known']).sum()),
        synchronized=False, note='No interpolation, clock mapping or stable-window acceptance is implied by K1 schema validation')


def observation_metadata(manifest, config, *, run_id, clocks, source):
    return dict(schema_version=1, schema='trical_observations_v1', run_id=run_id, source=source,
        joint_names=manifest['joint_names'], geometry_sha256=manifest['geometry_sha256'], geometry=manifest,
        profile_sha256=digest(config), profile=config, clocks=clocks,
        units=dict(position='m', angle='rad', angular_velocity='rad/s', time='ns'),
        indicator_channels=['x', 'y', 'z'], status_codes={x.name: int(x) for x in IndicatorStatus})


def save_observations(folder, arrays, metadata):
    summary = validate_observations(arrays, metadata)
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(folder / 'observations.npz', **arrays)
    write_json(folder / 'metadata.json', metadata)
    write_json(folder / 'manifest.json', dict(schema_version=1,
        observations_sha256=sha256((folder / 'observations.npz').read_bytes()).hexdigest(),
        metadata_sha256=sha256((folder / 'metadata.json').read_bytes()).hexdigest(), validation=summary))
    return summary


def load_observations(folder):
    folder = Path(folder); manifest = json.loads((folder / 'manifest.json').read_text(encoding='utf-8'))
    for filename, key in [('observations.npz', 'observations_sha256'), ('metadata.json', 'metadata_sha256')]:
        if sha256((folder / filename).read_bytes()).hexdigest() != manifest[key]:
            raise ValueError('Immutable TriCal record digest mismatch')
    metadata = json.loads((folder / 'metadata.json').read_text(encoding='utf-8'))
    with np.load(folder / 'observations.npz', allow_pickle=False) as raw:
        arrays = {key: raw[key] for key in raw.files}
    validate_observations(arrays, metadata)
    return arrays, metadata
