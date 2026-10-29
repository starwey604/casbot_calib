"""One total output-torque observation, without a current-shaped alias."""
import numpy as np


def output_torque(observations,gain=None):
    if 'torque_nm' in observations:
        if 'current' in observations:raise ValueError('Select one torque observation channel, not current plus derived torque')
        return np.asarray(observations['torque_nm'])
    if gain is None or not np.isfinite(gain) or gain==0:
        raise ValueError('Legacy current observations require a declared output gain')
    return np.asarray(observations['current'])*gain
