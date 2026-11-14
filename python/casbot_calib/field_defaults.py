"""Shared startup defaults for new field tests (H1--H4, K and manual tools)."""
import math

START_VELOCITY_RAD_S=0.2


def start_velocity_limit(config=None):
    value=(config or {}).get('start_velocity_rad_s',START_VELOCITY_RAD_S)
    if isinstance(value,bool) or not math.isfinite(value) or value<=0:
        raise ValueError('起点停稳速度须为正有限rad/s')
    return float(value)
