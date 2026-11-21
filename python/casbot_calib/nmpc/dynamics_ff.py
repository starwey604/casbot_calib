"""Inverse dynamics for the five-field SHM command in DVT2 joint order."""
import json
from pathlib import Path

import numpy as np

from .. import _core
from ..dynamic_export import _restore_runtime
from ..model_context import load_context
from ..pipeline.robot import load_robot_config
from ..project_paths import ROOT


class Feedforward:
    def __init__(self, cfg, delivery=None, *, enabled=True):
        self.enabled = enabled
        context = load_context()
        robot = load_robot_config()
        self.limit = robot['feedforward_limit_nm']
        self.engine = context.engine
        default = ROOT / 'artifacts/calib/latest_dynamics/delivery'
        path = Path(delivery) if delivery is not None else default if default.is_dir() else None
        self.source = str(path) if path is not None else '名义模型'
        floor = cfg['smoothing_floor_rad_s']
        if path is not None:
            runtime = json.loads((path / 'runtime_dynamics.json').read_text())
            engine = _core.ArmDynamics.from_urdf(str(path / 'calibrated.urdf'))
            if (runtime['joint_names'] != list(context.joint_names)
                    or [j.name for j in engine.model.joints] != runtime['joint_names']
                    or runtime['gravity_world'] != robot['gravity_world']):
                raise ValueError('动力学交付的轴序或重力与 DVT2 不符')
        else:
            joints = self.engine.model.joints
            friction = {k: [getattr(j.friction, k) for j in joints] for k in
                        ('viscous', 'coulomb', 'static_friction', 'smoothing_velocity', 'stribeck_velocity')}
            runtime = dict(rotor_inertia=[j.rotor_inertia for j in joints],
                           gravity_world=robot['gravity_world'], friction=friction)
            for key, value in cfg.get('nominal_dynamics', {}).items():
                if key == 'rotor_inertia':
                    runtime[key] = np.broadcast_to(value, (14,))
                elif key in friction:
                    friction[key] = np.broadcast_to(value, (14,))
                else:
                    raise ValueError(f'未知名义动力学参数 {key}')
            engine = self.engine
        runtime['friction']['smoothing_velocity'] = np.maximum(runtime['friction']['smoothing_velocity'], floor)
        self.engine = _restore_runtime(engine, runtime)

    def effort(self, state):
        if not self.enabled:
            return np.zeros(14)
        effort = self.engine.inverse(*np.split(state, 3))
        if not np.isfinite(effort).all() or np.max(np.abs(effort)) > self.limit:
            raise ValueError('前馈力矩超过限制或含非有限值')
        return effort
