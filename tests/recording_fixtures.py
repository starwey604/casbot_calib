"""Construct native-format capture tables for offline analysis tests."""
from itertools import islice

import numpy as np

from casbot_calib import _core
from casbot_calib.shm_data import DIRECTION, OFFSET


def write_recording(path, rows=(), *, feedback=True, frame_rows=256):
    header = _core._recording_header(feedback)
    columns = {name: i for i, name in enumerate(header.rstrip('\n').split(','))}
    empty_row = ['0'] * len(columns)
    _core._write_csv_zst(path, header)
    rows = iter(rows)
    while batch := list(islice(rows, frame_rows)):
        lines = []
        for row in batch:
            values = empty_row.copy()
            for name, value in row.items():
                values[columns[name]] = str(value)
            lines.append(','.join(values)+'\n')
        _core._write_csv_zst(path, ''.join(lines), append=True)


def arm_columns(name, values):
    return {f'{name}{j}': value for j, value in enumerate(values)}


def feedback_rows(receive, *, stamps=None, observations=None, phase=None, counters=None,
                  position=None, velocity=None, torque=None):
    """Return rows with matching raw registers and converted arm quantities."""
    for i, at in enumerate(receive):
        row = dict(receive_mono_ns=at, frame_counter=i+1 if counters is None else counters[i])
        if stamps is not None:
            row['source_timestamp_ms'] = stamps[i]
        if observations is not None:
            row['observation_time_ns'] = observations[i]
        if phase is not None:
            row['phase'] = phase[i]
        if position is not None:
            row.update(arm_columns('q', position[i]*np.asarray(DIRECTION)*1e-6+OFFSET))
            row.update({f'pos_raw{j+12}': int(value) for j, value in enumerate(position[i])})
        if velocity is not None:
            row.update(arm_columns('v', velocity[i]*np.asarray(DIRECTION)*1e-6))
            row.update({f'vel_raw{j+12}': int(value) for j, value in enumerate(velocity[i])})
        if torque is not None:
            row.update(arm_columns('effort', torque[i]*np.asarray(DIRECTION)*.01))
            row.update({f'trq_raw{j+12}': int(value) for j, value in enumerate(torque[i])})
        yield row
