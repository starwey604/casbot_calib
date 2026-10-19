"""Batch virtual-device boundary. Simulator truth never enters identification."""
from __future__ import annotations
from time import perf_counter
import numpy as np
from . import _core

def simulate(plant: dict, times_ns: np.ndarray, commands: np.ndarray, duration_ns: int):
    config = _core.SimulationConfig()
    for key, value in plant.items():
        if not hasattr(config, key):
            raise ValueError(f"Unknown simulation parameter: {key}")
        setattr(config, key, value)
    start = perf_counter()
    observations, truth = _core.SimulationRunner(config).run(times_ns, commands, int(duration_ns))
    return observations, truth, {"simulation_wall_s": perf_counter()-start,
                                 "virtual_duration_s": duration_ns*1e-9}

def default_plant() -> dict:
    """Public synthetic plant fixture, not an ENCOS specification."""
    return dict(inertia=.08, damping=.12, coulomb=.06, spring=3., gain=2.,
                spring_zero=0., friction_velocity=.03, integration_ns=50_000,
                sample_period_ns=1_000_000, torque_limit=20., current_limit=10., current_tau=.001)
