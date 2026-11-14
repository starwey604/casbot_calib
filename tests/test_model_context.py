"""N3 explicit DVT2 integration, including the atomic default consumers."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.model_context import load_context, resolve_context, MODEL_ID
from casbot_calib.dual_model import nominal_dual_arm, physical_model
from casbot_calib.kinematics import nominal_kinematics, default_config
from casbot_calib.rigid_identification import model_from_parameters
from casbot_calib.artifact_io import sha256
from casbot_calib.urdf_resources import resource_elements, resolved_resource


@pytest.fixture(scope='module')
def context():return load_context()


def assert_resources(path):
    resources=resource_elements(ET.parse(path).getroot())
    assert resources
    assert all(Path(resolved_resource(item.get('filename'),path)).is_file() for item in resources)


def test_context_is_explicit_fresh_and_source_bound(context):
    engine,source=nominal_dual_arm(context)
    assert engine.model.model_id==MODEL_ID
    assert nominal_dual_arm()[1]['sha256']==source['sha256']
    assert tuple(j.name for j in engine.model.joints)==context.joint_names
    assert len(engine.frame_names())==155
    model=engine.model;model.model_id='changed';engine=_core.ArmDynamics(model)
    assert context.engine.model.model_id==MODEL_ID
    home=context.home;home[:]=0
    assert np.linalg.norm(context.home)>1
    metadata=context.metadata;metadata['profile']['home_rad'][0]=99
    assert context.home[0]!=99
    wrong=copy.deepcopy(context.identity);wrong['urdf_sha256']='wrong'
    with pytest.raises(ValueError,match='context differs'):resolve_context(wrong,context)
    assert resolve_context(context.identity).urdf==context.urdf
    transport=context.transport_parameters()
    assert _core.ArmDynamics.from_urdf(transport['model_path']).model.model_id==transport['model_id']
    assert transport['joint_names']==list(context.joint_names)








def write(path,data):path.write_text(json.dumps(data)+'\n')


def seal(path):
    write(path/'manifest.json',{str(p.relative_to(path)):sha256(p) for p in path.rglob('*') if p.is_file() and p.name!='manifest.json'})


def synthetic_analysis(context,path):
    """Accepted-format synthetic fixture, not the result of an S4 experiment."""
    scaled=physical_model(context.engine,np.linspace(.9,1.1,14),.02,.03)
    candidate=model_from_parameters(context.engine,scaled.rigid_parameters(),.02,.03,.03)
    fit=dict(parameters=candidate.rigid_parameters().tolist(),identifiability='synthetic test representative',rank=28,columns=140,unique_body_parameters=False)
    (path/'gravity').mkdir(parents=True);(path/'inertia').mkdir()
    write(path/'analysis.json',dict(schema_version='casbot_vendor_s4_analysis_v1',passed=True,scope='synthetic_test_fixture',
        stages=dict(gravity=dict(passed=True),inertia=dict(passed=True)),source_model_sha256=context.source['sha256']))
    write(path/'gravity/report.json',dict(passed=True,fit=fit))
    write(path/'inertia/report.json',dict(passed=True,joint_refinement=dict(fit,auxiliary_parameters=[.02]*14+[.03]*14),
        prior_stage_acceptance=dict(static_friction_frozen=[.03]*14)))
    write(path/'inertia/candidate.json',dict(model_version='tree_full_physical_inertia_v1',source_sha256=context.source['sha256'],
        joint_names=list(context.joint_names),gain_status='synthetic_declared',parameters=fit['parameters'],
        viscous=[.02]*14,coulomb=[.03]*14,static_friction=[.03]*14))
    seal(path);return candidate






def test_runtime_context_does_not_import_python_pinocchio_or_coal():
    code="""
import sys
sys.modules['pinocchio']=None
sys.modules['coal']=None
from casbot_calib.model_context import load_context
from casbot_calib.kinematics import nominal_kinematics, default_config
context=load_context()
engine,_=nominal_kinematics(context.kinematic_config())
assert len(engine.model.joints)==14
assert len(context.collision_profile().pairs)>0
"""
    subprocess.run([sys.executable,'-c',code],check=True,capture_output=True,text=True)


