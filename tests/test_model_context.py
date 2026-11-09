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
from casbot_calib.kinematic_parameters import KinematicParameters
from casbot_calib.kinematic_export import export_urdf
from casbot_calib.dynamic_export import export_dynamics_urdf, load_exported_dynamics, verify_dynamics_export
from casbot_calib.rigid_identification import model_from_parameters
from casbot_calib.vendor_identity import sha256
from casbot_calib.vendor_viewer import scene_for_vendor
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


def test_actual_four_ball_kinematic_geometry_reuses_source_tcps(context):
    config=context.kinematic_config();engine,manifest=nominal_kinematics(config)
    assert manifest['model_context']==context.identity
    assert [ball['id'] for ball in config['fixture']['balls']]==list('ABCD')
    assert engine.frame_names()==context.engine.frame_names()
    chart=KinematicParameters(engine);assert chart.size==60
    unchanged=chart.build(np.zeros(chart.size))
    for frame in engine.frame_names():
        np.testing.assert_allclose(unchanged.frame_pose(context.home,frame),engine.frame_pose(context.home,frame),atol=1e-14)
    config['tools'][0]['translation_m'][0]+=.001
    with pytest.raises(ValueError,match='tool differs'):nominal_kinematics(config)


def test_collision_accepts_inertia_not_geometry_changes(context):
    candidate=physical_model(context.engine,1.1,.02,.03)
    profile=context.collision_profile(engine=candidate)
    np.testing.assert_allclose(profile.engine.rigid_parameters(),candidate.rigid_parameters())
    contact=context.collision_profile(active_tool='left',ball_id='A',contact=True)
    clone=context.collision_profile(active_tool='left',ball_id='A',contact=True,engine=candidate)
    assert clone.pairs==contact.pairs
    np.testing.assert_array_equal(clone.pair_clearance_m,contact.pair_clearance_m)
    parameters=KinematicParameters(context.engine);x=np.zeros(parameters.size);x[0]=.2
    with pytest.raises(ValueError,match='rebuilt collision'):context.collision_profile(engine=parameters.build(x))
    with pytest.raises(ValueError,match='active tool'):context.collision_profile(contact=True)


def test_kinematic_export_keeps_all_frames_dynamics_and_resources(context,tmp_path):
    config=context.kinematic_config();nominal,_=nominal_kinematics(config)
    chart=KinematicParameters(nominal);candidate=chart.build(np.linspace(-.15,.15,chart.size))
    path=tmp_path/'candidate.urdf';assert export_urdf(nominal,candidate,config,path)['passed']
    loaded=_core.ArmDynamics.from_urdf(str(path));assert_resources(path)
    links=[link.get('name') for link in ET.parse(path).getroot().findall('link')]
    assert all(links.count(tool['name'])==1 for tool in config['tools'])
    rng=np.random.default_rng(20261106)
    for q in rng.uniform(-.4,.4,(4,14)):
        for frame in candidate.frame_names():
            np.testing.assert_allclose(loaded.frame_pose(q,frame),candidate.frame_pose(q,frame),atol=1e-12)
        np.testing.assert_allclose(loaded.mass_matrix(q),candidate.mass_matrix(q),atol=1e-12)
        np.testing.assert_allclose(loaded.gravity(q),candidate.gravity(q),atol=1e-12)
    with pytest.raises(FileExistsError):export_urdf(nominal,candidate,config,path)


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


def test_dynamics_export_reload_uses_bound_source_without_caller_context(context,tmp_path):
    analysis=tmp_path/'analysis';candidate=synthetic_analysis(context,analysis)
    output=tmp_path/'export'
    result=export_dynamics_urdf(analysis,output)
    assert export_dynamics_urdf(analysis,tmp_path/'explicit_export',context=context)['passed']
    assert result['passed'] and result['checks']['geometry_xml_unchanged']
    loaded,metadata=load_exported_dynamics(output)
    assert metadata['model_context']==context.identity
    assert not metadata['hardware_executable']
    assert_resources(output/'candidate.urdf')
    np.testing.assert_allclose(loaded.rigid_parameters(),candidate.rigid_parameters(),atol=1e-13)
    np.testing.assert_allclose(loaded.inverse(context.home,np.ones(14)*.1,np.ones(14)*.2),
                               candidate.inverse(context.home,np.ones(14)*.1,np.ones(14)*.2),atol=1e-12)
    assert verify_dynamics_export(output,tmp_path/'verify')['passed']
    # Legitimate rebasing must not hide substitution of a different mesh.
    tree=ET.parse(output/'candidate.urdf');meshes=tree.getroot().findall('.//mesh')
    different=next(m.get('filename') for m in meshes if m.get('filename')!=meshes[0].get('filename'))
    meshes[0].set('filename',different);tree.write(output/'candidate.urdf')
    metadata['urdf_sha256']=sha256(output/'candidate.urdf');write(output/'calibration.json',metadata);seal(output)
    with pytest.raises(ValueError,match='geometry differs'):load_exported_dynamics(output)
    metadata['model_context']['model_id']='wrong';write(output/'calibration.json',metadata);seal(output)
    with pytest.raises(ValueError,match='context differs'):load_exported_dynamics(output)


@pytest.mark.parametrize('tool',[None,'left','right'])
def test_viewer_resources_and_identity(context,tmp_path,tool):
    config=dict(model_id=context.model_id,joint_names=list(context.joint_names),active_tool=tool)
    path=tmp_path/'scene.xml';adapter,engine=scene_for_vendor(config,path,context)
    assert adapter.robot_model_id==engine.model.model_id==context.model_id
    root=ET.parse(path).getroot()
    assert all((path.parent/mesh.get('file')).is_file() for mesh in root.findall('./asset/mesh'))
    fixtures=[g for g in root.iter('geom') if g.get('name','').startswith(('fixture_','datum_'))]
    assert bool(fixtures)==(tool is not None)
    config['model_id']='old'
    with pytest.raises(ValueError,match='identity mismatch'):scene_for_vendor(config,tmp_path/'wrong.xml',context)


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


def test_default_export_reuses_tcp_without_mutating_caller(tmp_path):
    nominal,_=nominal_dual_arm();config=default_config()
    before=nominal.frame_names()
    assert all(tool['name'] in before for tool in config['tools'])
    path=tmp_path/'legacy_candidate.urdf'
    assert export_urdf(nominal,nominal,config,path)['passed']
    assert nominal.frame_names()==before
    loaded=_core.ArmDynamics.from_urdf(str(path))
    assert all(tool['name'] in loaded.frame_names() for tool in config['tools'])
