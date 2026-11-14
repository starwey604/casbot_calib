"""Standardized SHM calibration pipeline."""
from .robot import DEFAULT_EXECUTOR, DEFAULT_ROBOT_CONFIG, find_executor, load_robot_config
from .session import Session

__all__ = [
    'DEFAULT_EXECUTOR',
    'DEFAULT_ROBOT_CONFIG',
    'Session',
    'find_executor',
    'load_robot_config',
    'main',
    'run_pipeline',
    'run_preflight',
]


def __getattr__(name):
    if name in ('main', 'run_pipeline'):
        from . import runner
        return getattr(runner, name)
    if name == 'run_preflight':
        from .preflight import run_preflight
        return run_preflight
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
