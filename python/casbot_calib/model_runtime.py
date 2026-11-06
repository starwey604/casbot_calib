"""Resolve a frozen workflow model, scene and resource source together.

During N3 preparation untagged existing profiles retain their original contract.
Tagged DVT2 profiles never fall back to those legacy assets.
"""
from importlib.resources import files
from pathlib import Path
import numpy as np
from .dual_model import nominal_dual_arm


def context_for(config):
    if 'model_context' not in config:return None
    from .model_context import resolve_context
    return resolve_context(config['model_context'])


def model_for(config):
    engine,source=nominal_dual_arm(context_for(config))
    if 'gravity_world' in config:
        model=engine.model;model.gravity_world=np.asarray(config['gravity_world']);engine=type(engine)(model)
    return engine,source


def urdf_for(config):
    context=context_for(config)
    return context.urdf if context else Path(str(files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')))


def dynamics_scene(context):
    scene=context.metadata['scene']
    scene.update(clearance_m=.01,fixture_geometry_measured=False,model_context=context.identity,
        coverage='DVT2 bare-flange full robot and floor; no TriCal tool/table or measured waist clamp')
    return scene


def collision_for(config,engine=None,scene=None):
    context=context_for(config)
    if context is None:
        from .collision import nominal_profile
        return nominal_profile(engine,scene if scene is not None else config.get('scene'))
    scene=config.get('scene') if scene is None else scene
    if scene!=dynamics_scene(context):raise ValueError('DVT2 workflow collision scene differs from its frozen model context')
    profile=context.collision_profile(engine=engine)
    from . import _core
    profile.metadata={**profile.metadata,'scene':scene,'model_context':context.identity,
        'backend':_core.CollisionChecker.backend_version(),'actual_geometry_count':len(profile.geometry)}
    return profile


def check_same_context(first,second):
    if first.get('model_context')!=second.get('model_context'):
        raise ValueError('Workflow model contexts differ; regenerate the profile/plan, do not mix model generations')
