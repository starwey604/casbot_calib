"""Declared synthetic workflow settings, saved before any acquisition begins."""
from copy import deepcopy

DEFAULTS=dict(
    plant=dict(position_noise=1e-6,current_noise=.0002,seed=20261020,
               current_tau=.002,command_delay_ns=1_300_000),
    identification=dict(gain=1.7),
    added_inertia=.04,
    repeat_noise=dict(position_noise=1e-6,current_noise=.0002,position_quantum=1e-6,
                      current_quantum=1e-4,drop_probability=.01,repeat_probability=.01),
    acceptance=dict(torque_rmse_nm=.01,current_rmse_a=.005,
                    repeat_relative_limit=.05,minimum_repeat_passes=9),
)


def workflow_config(overrides=None):
    result=deepcopy(DEFAULTS)
    for name,value in (overrides or {}).items():
        if name not in result:
            raise ValueError(f"Unknown workflow configuration: {name}")
        if isinstance(result[name],dict):
            if not isinstance(value,dict):raise ValueError(f"Expected mapping for {name}")
            if name in ("acceptance","repeat_noise") and set(value)-set(result[name]):
                raise ValueError(f"Unknown {name} fields: {set(value)-set(result[name])}")
            result[name].update(value)
        else:result[name]=value
    for key,value in result["acceptance"].items():
        if not isinstance(value,(int,float)) or not 0<value<float("inf"):
            raise ValueError(f"Invalid acceptance threshold: {key}")
    count=result["acceptance"]["minimum_repeat_passes"]
    if not isinstance(count,int) or not 1<=count<=10:
        raise ValueError("minimum_repeat_passes must be an integer from 1 to 10")
    if not 0<result["added_inertia"]<float("inf"):
        raise ValueError("Known added inertia must be positive finite")
    return result
