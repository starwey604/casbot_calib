"""Calibration stage protocol and stage registry."""
from pathlib import Path


class Stage:
    name = ''
    title = ''
    default_config = Path()
    implemented = True

    def load_recipe(self, config_path=None, **overrides):
        raise NotImplementedError

    def plan(self, q_start, robot_config, recipe, plan_dir, *, session=None):
        raise NotImplementedError

    def analyse(self, plan_dir, capture_dir, analysis_dir, *, session=None):
        raise NotImplementedError

    def write_summary(self, stage_dir, plan_meta, capture_summary, report):
        raise NotImplementedError


def stage_registry():
    from ..stages.dynamics import H1GravityStage, H2FrictionStage, H3InertiaStage
    return {
        'h1': H1GravityStage(),
        'h2': H2FrictionStage(),
        'h3': H3InertiaStage(),
    }


def resolve_stages(spec='h1,h2,h3'):
    registry = stage_registry()
    if isinstance(spec, str):
        names = [part.strip().lower() for part in spec.split(',') if part.strip()]
    else:
        names = [str(part).strip().lower() for part in spec if str(part).strip()]
    if not names or len(set(names)) != len(names):
        raise ValueError('请指定非空且不重复的阶段列表（可选：h1, h2, h3）')
    unknown = [name for name in names if name not in registry]
    if unknown:
        raise ValueError(f'未知标定阶段：{", ".join(unknown)}（可选：h1, h2, h3）')
    return [registry[name] for name in names]
