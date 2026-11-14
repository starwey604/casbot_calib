import copy
from importlib.resources import files
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.rigid_identification import model_from_parameters
from casbot_calib.dynamic_export import check_reload, write_inertial_urdf
from casbot_calib.artifact_io import sha256

SOURCE=__import__('casbot_calib.model_context',fromlist=['load_context']).load_context().urdf


def write(path,data):Path(path).write_text(json.dumps(data)+'\n')


def seal(path):
    write(path/'manifest.json',{str(p.relative_to(path)):sha256(p) for p in path.rglob('*') if p.is_file() and p.name!='manifest.json'})


@pytest.fixture
def accepted(tmp_path):
    nominal,source=nominal_dual_arm();model=nominal.model;joints=model.joints
    for index,joint in enumerate(joints):
        body=joint.body;body.mass*=.9+.02*index;body.com=body.com+np.array([.001,-.002,.003])
        body.inertia_com=body.inertia_com*(1.03+.01*index);joint.body=body
    # A valid fitted wrist aggregate may be lighter than the original palm.
    body=joints[6].body;body.mass=.4;body.com=[.01,-.015,.02]
    body.inertia_com=np.array([[.001,.0001,-.00003],[.0001,.0012,.00002],[-.00003,.00002,.0014]]);joints[6].body=body
    model.joints=joints;changed=_core.ArmDynamics(model)
    b=np.linspace(.02,.04,14);fc=np.linspace(.03,.05,14);fs=fc+.02
    engine=model_from_parameters(nominal,changed.rigid_parameters(),b,fc,fs)
    p=engine.rigid_parameters().tolist()
    fit=dict(parameters=p,identifiability='test physical representative, not unique components',rank=28,columns=140,unique_body_parameters=False)
    root=tmp_path/'analysis';(root/'gravity').mkdir(parents=True);(root/'inertia').mkdir()
    write(root/'analysis.json',dict(schema_version='casbot_vendor_s4_analysis_v1',passed=True,scope='synthetic_observations',
        stages=dict(gravity=dict(passed=True),inertia=dict(passed=True)),source_model_sha256=source['sha256']))
    write(root/'gravity/report.json',dict(passed=True,fit=fit))
    write(root/'inertia/report.json',dict(passed=True,joint_refinement=dict(fit,auxiliary_parameters=np.r_[b,fc].tolist()),
        prior_stage_acceptance=dict(static_friction_frozen=fs.tolist())))
    write(root/'inertia/candidate.json',dict(model_version='tree_full_physical_inertia_v1',source_sha256=source['sha256'],
        joint_names=[j.name for j in model.joints],gain_status='synthetic_declared',parameters=p,
        viscous=b.tolist(),coulomb=fc.tolist(),static_friction=fs.tolist()))
    seal(root);return root,engine
















def test_runtime_sidecar_restores_armature_once(accepted,tmp_path):
    _,engine=accepted;model=engine.model;joints=model.joints
    for joint in joints:joint.rotor_inertia=.002
    model.joints=joints;expected=_core.ArmDynamics(model)
    write_inertial_urdf(SOURCE,expected,tmp_path/'candidate.urdf')
    report=check_reload(tmp_path/'candidate.urdf',expected,SOURCE,samples=2)
    assert report['passed'] and report['checks']['runtime_inverse_nm'] and report['checks']['runtime_mass_matrix_kg_m2']


def test_geometry_changes_cannot_hide_in_an_inertial_export(accepted,tmp_path):
    _,engine=accepted;model=engine.model;joints=model.joints;origin=joints[0].origin
    origin.translation=np.asarray(origin.translation)+[.01,0,0];joints[0].origin=origin;model.joints=joints
    with pytest.raises(ValueError,match='geometry differs'):write_inertial_urdf(SOURCE,_core.ArmDynamics(model),tmp_path/'wrong.urdf')


def test_axis_change_at_zero_configuration_is_still_rejected(accepted,tmp_path):
    _,engine=accepted;model=engine.model;joints=model.joints;joints[0].axis=[0,0,1];model.joints=joints
    with pytest.raises(ValueError,match='geometry/constraints'):write_inertial_urdf(SOURCE,_core.ArmDynamics(model),tmp_path/'wrong_axis.urdf')
