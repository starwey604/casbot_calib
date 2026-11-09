"""Resolve a frozen workflow model, scene and resource source together.

DVT2 is the sole default. Explicit historical model identities are rejected;
no downloaded or legacy asset fallback exists.
"""
import numpy as np
from .dual_model import nominal_dual_arm


def context_for(config):
    from .model_context import load_context,resolve_context
    context=resolve_context(config['model_context']) if 'model_context' in config else load_context()
    if 'model' in config and config['model']!=context.model_id:
        raise ValueError('Workflow model differs from DVT2 default; historical records require their original code version')
    return context


def model_for(config):
    engine,source=nominal_dual_arm(context_for(config))
    if 'gravity_world' in config:
        model=engine.model;model.gravity_world=np.asarray(config['gravity_world']);engine=type(engine)(model)
    return engine,source


def urdf_for(config):
    context=context_for(config)
    return context.urdf


def dynamics_scene(context):
    scene=context.metadata['scene']
    scene.update(clearance_m=.01,fixture_geometry_measured=False,model_context=context.identity,
        coverage='DVT2 bare-flange full robot and floor; no TriCal tool/table or measured waist clamp')
    return scene


def collision_for(config,engine=None,scene=None):
    context=context_for(config)
    scene=config.get('scene',dynamics_scene(context)) if scene is None else scene
    if scene!=dynamics_scene(context):raise ValueError('DVT2 workflow collision scene differs from its frozen model context')
    profile=context.collision_profile(engine=engine)
    from . import _core
    profile.metadata={**profile.metadata,'scene':scene,'model_context':context.identity,
        'backend':_core.CollisionChecker.backend_version(),'actual_geometry_count':len(profile.geometry)}
    return profile


def check_same_context(first,second):
    if first.get('model_context')!=second.get('model_context'):
        raise ValueError('Workflow model contexts differ; regenerate the profile/plan, do not mix model generations')
    context_for(first);context_for(second)  # Validate explicit identities; never auto-tag serialized inputs.
