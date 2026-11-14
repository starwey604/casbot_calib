"""Robot hardware parameters and native executor resolution."""
from copy import deepcopy
import os
from pathlib import Path
import numpy as np
import yaml
from ..field_defaults import start_velocity_limit
from ..model_runtime import context_for, dynamics_scene
from ..shm_data import ROOT

DEFAULT_ROBOT_CONFIG = ROOT / 'configs/robot/dvt2.yaml'
DEFAULT_EXECUTOR = ROOT / 'install/shm-core/bin/shm_executor'
COMMAND_PERIOD_NS = 1_000_000
START_TOLERANCE_RAD = 0.005


def find_executor(explicit=None):
    if explicit is not None:
        binary = Path(explicit).resolve()
        if not binary.is_file():
            raise ValueError(f'找不到原生执行器：{binary}')
        return binary
    env = os.environ.get('CASBOT_SHM_EXECUTOR')
    if env:
        binary = Path(env).resolve()
        if not binary.is_file():
            raise ValueError(f'CASBOT_SHM_EXECUTOR 不存在：{binary}')
        return binary
    for candidate in (
        DEFAULT_EXECUTOR,
        ROOT / 'build/shm-core/cpp/src/shm/shm_executor',
        ROOT / 'build/humble/cpp/src/shm/shm_executor',
    ):
        if candidate.is_file():
            return candidate.resolve()
    raise ValueError(f'找不到 shm_executor（默认位置 {DEFAULT_EXECUTOR}），请先编译安装 SHM 组件')


def load_robot_config(source=DEFAULT_ROBOT_CONFIG):
    if isinstance(source, dict):
        c = deepcopy(source)
    else:
        path = Path(source).resolve()
        c = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    c.setdefault('transport', 'shm_mc')
    c.setdefault('command_period_ns', COMMAND_PERIOD_NS)
    c.setdefault('start_tolerance_rad', START_TOLERANCE_RAD)
    c.setdefault('initial_hold_s', 0.5)
    c.setdefault('final_hold_s', 0.5)
    c.setdefault('anchor_transition_s', 4.0)
    c.setdefault('maximum_velocity_rad_s', 1.5)
    c.setdefault('feedforward_limit_nm', 100.0)
    c.setdefault('kp', 20.0)
    c.setdefault('kd', 2.0)
    c.setdefault('effort_gain', 1.0)
    c.setdefault('effort_zero_nm', 0.0)
    c.setdefault('gravity_world', [0.0, 0.0, -9.81])

    if c['command_period_ns'] != COMMAND_PERIOD_NS or c['start_tolerance_rad'] != START_TOLERANCE_RAD:
        raise ValueError('SHM 原生周期固定为 1ms、起点容差固定为 0.005 rad')
    c['start_velocity_rad_s'] = start_velocity_limit(c)

    for key in ('initial_hold_s', 'final_hold_s', 'anchor_transition_s', 'maximum_velocity_rad_s', 'feedforward_limit_nm'):
        if not np.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'机器人配置项须为正有限值：{key}')
    if c['maximum_velocity_rad_s'] > 1.5 or c['feedforward_limit_nm'] > 100.0:
        raise ValueError('速度上限（≤1.5 rad/s）或前馈上限（≤100 Nm）超出接口范围')

    for key, maximum in (('kp', 500.0), ('kd', 50.0)):
        values = np.broadcast_to(np.asarray(c[key], dtype=float), (14,)).copy()
        if not np.isfinite(values).all() or np.any(values < 0) or np.any(values > maximum):
            raise ValueError(f'逐轴 {key} 超出有效范围 [0, {maximum:g}]')
        c[key] = values.tolist()

    for key in ('effort_gain', 'effort_zero_nm'):
        values = np.broadcast_to(np.asarray(c[key], dtype=float), (14,)).copy()
        if not np.isfinite(values).all() or (key == 'effort_gain' and np.any(values == 0)):
            raise ValueError(f'力矩标定参数无效：{key}')
        c[key] = values.tolist()

    gravity = np.asarray(c['gravity_world'], dtype=float)
    if gravity.shape != (3,) or not np.isfinite(gravity).all() or np.linalg.norm(gravity) == 0:
        raise ValueError('gravity_world 须为非零三维向量')
    c['gravity_world'] = gravity.tolist()

    context = context_for(c)
    c['model_context'] = context.identity
    scene = dynamics_scene(context)
    if 'scene' in c and c['scene'] != scene:
        raise ValueError('配置中的碰撞场景与当前 DVT2 模型不一致')
    c['scene'] = scene
    return c
