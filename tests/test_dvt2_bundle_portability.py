"""ARM fixed-frame roundoff restores the same frozen identity, not new geometry."""
import json
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.dvt2_bundle import _bind_frozen_frames
from casbot_calib.kinematics import geometry_manifest
from casbot_calib.model_context import BUNDLE_RELATIVE
from casbot_calib.model_sources import model_directory


@pytest.fixture
def frozen():
    root=model_directory()/BUNDLE_RELATIVE
    return _core.ArmDynamics.from_urdf(str(root/'model.urdf')),json.loads((root/'model.json').read_text())['geometry']


def test_single_ulp_fixed_frame_restores_exact_frozen_identity(frozen):
    engine,metadata=frozen;model=engine.model;frames=model.frames
    placement=frames[55].placement;matrix=placement.rotation.copy()
    matrix[0,2]=np.nextafter(matrix[0,2],0.);placement.rotation=matrix;frames[55].placement=placement;model.frames=frames
    result=_bind_frozen_frames(_core.ArmDynamics(model),metadata)
    assert geometry_manifest(result,'dvt2_locked_v1')['geometry_sha256']==metadata['geometry_sha256']


@pytest.mark.parametrize('change',['frame_transform','frame_name','frame_parent','moving_geometry'])
def test_material_or_topological_geometry_change_still_rejected(frozen,change):
    engine,metadata=frozen;model=engine.model
    if change=='moving_geometry':
        joints=model.joints;p=joints[0].origin;p.translation=p.translation+np.array([1e-12,0,0]);joints[0].origin=p;model.joints=joints
    else:
        frames=model.frames
        if change=='frame_name':frames[55].name+='changed'
        elif change=='frame_parent':frames[55].parent=0 if frames[55].parent==-1 else -1
        else:
            p=frames[55].placement;p.translation=p.translation+np.array([1e-12,0,0]);frames[55].placement=p
        model.frames=frames
    with pytest.raises(ValueError,match='Derived geometry differs'):_bind_frozen_frames(_core.ArmDynamics(model),metadata)
