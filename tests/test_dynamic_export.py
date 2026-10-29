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
from casbot_calib.dynamic_export import export_dynamics_urdf,load_exported_dynamics,verify_dynamics_export,check_reload,write_inertial_urdf
from casbot_calib.vendor_identity import sha256

SOURCE=files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')


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


def test_export_is_lossless_and_inputs_immutable(accepted,tmp_path):
    analysis,expected=accepted;before={p:sha256(p) for p in analysis.rglob('*') if p.is_file()}
    source_hash=sha256(SOURCE);result=export_dynamics_urdf(analysis,tmp_path/'export')
    assert result['passed'] and not result['hardware_executable'] and not result['individual_body_parameters_unique']
    loaded,meta=load_exported_dynamics(tmp_path/'export')
    assert not meta['hardware_executable'] and meta['inertia_estimation_stage_included']
    np.testing.assert_allclose(loaded.rigid_parameters(),expected.rigid_parameters(),atol=1e-13)
    assert sha256(SOURCE)==source_hash and all(sha256(p)==digest for p,digest in before.items())
    assert verify_dynamics_export(tmp_path/'export',tmp_path/'recheck')['passed']
    with pytest.raises(FileExistsError):export_dynamics_urdf(analysis,tmp_path/'export')


def test_fixed_hand_aggregate_not_counted_twice_and_xml_has_com_inertia(accepted,tmp_path):
    analysis,expected=accepted;export_dynamics_urdf(analysis,tmp_path/'export')
    root=ET.parse(tmp_path/'export/candidate.urdf').getroot()
    assert root.find("link[@name='left_palm_base_link']/inertial") is None
    assert root.find("joint[@name='left_hand_joint']") is not None
    inertial=root.find("link[@name='left_wrist_roll_link']/inertial")
    assert float(inertial.find('mass').get('value'))==pytest.approx(.4)
    np.testing.assert_allclose(np.fromstring(inertial.find('origin').get('xyz'),sep=' '),[.01,-.015,.02],atol=1e-15)
    assert inertial.find('origin').get('rpy')=='0 0 0'
    assert float(inertial.find('inertia').get('ixy'))==pytest.approx(.0001)
    assert root.find("link[@name='waist_yaw_link']/inertial/mass").get('value')=='11.255'
    assert root.find("link[@name='left_palm_base_link']/visual") is not None
    original=ET.parse(SOURCE).getroot()
    root.find("link[@name='left_palm_base_link']").append(copy.deepcopy(original.find("link[@name='left_palm_base_link']/inertial")))
    wrong=tmp_path/'double_hand.urdf';ET.ElementTree(root).write(wrong)
    report=check_reload(wrong,expected,SOURCE,samples=2)
    assert not report['passed'] and not report['checks']['rigid_parameters']


def test_gravity_export_does_not_claim_inertia_or_friction_identification(accepted,tmp_path):
    analysis,_=accepted;result=export_dynamics_urdf(analysis,tmp_path/'export','gravity')
    assert result['passed']
    loaded,metadata=load_exported_dynamics(tmp_path/'export')
    assert not metadata['inertia_estimation_stage_included'] and 'not identified' in metadata['friction_status']
    np.testing.assert_array_equal(loaded.friction(np.ones(14)*.2),0.)


@pytest.mark.parametrize('failure',['rejected_analysis','rejected_stage','changed_candidate','unsealed_candidate','wrong_geometry'])
def test_no_export_from_rejected_or_unbound_input(accepted,tmp_path,failure):
    analysis,_=accepted
    if failure=='rejected_analysis':
        data=json.loads((analysis/'analysis.json').read_text());data['passed']=False;write(analysis/'analysis.json',data)
    if failure=='rejected_stage':
        data=json.loads((analysis/'inertia/report.json').read_text());data['passed']=False;write(analysis/'inertia/report.json',data)
    if failure=='changed_candidate':
        data=json.loads((analysis/'inertia/candidate.json').read_text());data['parameters'][0]*=1.01;write(analysis/'inertia/candidate.json',data)
    if failure=='wrong_geometry':
        data=json.loads((analysis/'analysis.json').read_text());data['source_model_sha256']='0'*64;write(analysis/'analysis.json',data)
    seal(analysis)
    if failure=='unsealed_candidate':
        data=json.loads((analysis/'manifest.json').read_text());del data['inertia/candidate.json'];write(analysis/'manifest.json',data)
    with pytest.raises(ValueError):export_dynamics_urdf(analysis,tmp_path/'export')
    assert not (tmp_path/'export').exists()


def test_legacy_gravity_only_analysis_needs_source_geometry_binding(accepted,tmp_path):
    analysis,_=accepted;data=json.loads((analysis/'analysis.json').read_text());del data['source_model_sha256']
    data['stages'].pop('inertia');write(analysis/'analysis.json',data);seal(analysis)
    with pytest.raises(ValueError,match='source geometry'):export_dynamics_urdf(analysis,tmp_path/'export','gravity')


@pytest.mark.parametrize('file',['candidate.urdf','calibration.json','report.json'])
def test_export_tampering_is_rejected(accepted,tmp_path,file):
    analysis,_=accepted;export_dynamics_urdf(analysis,tmp_path/'export')
    with (tmp_path/'export'/file).open('a') as stream:stream.write(' ')
    with pytest.raises(ValueError,match='changed'):load_exported_dynamics(tmp_path/'export')


def test_resealed_geometry_change_is_still_rejected(accepted,tmp_path):
    analysis,_=accepted;export_dynamics_urdf(analysis,tmp_path/'export');path=tmp_path/'export/candidate.urdf'
    tree=ET.parse(path);tree.getroot().find("joint[@name='left_elbow_pitch_joint']/axis").set('xyz','0 0 1');tree.write(path)
    meta=json.loads((tmp_path/'export/calibration.json').read_text());meta['urdf_sha256']=sha256(path)
    write(tmp_path/'export/calibration.json',meta);seal(tmp_path/'export')
    with pytest.raises(ValueError,match='geometry differs'):load_exported_dynamics(tmp_path/'export')


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
