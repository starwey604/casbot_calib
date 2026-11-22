"""N3 explicit DVT2 integration, including the atomic default consumers."""
import copy
import json
import os
import shutil
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from casbot_calib import _core
from casbot_calib.dvt2_bundle import _bind_frozen_frames, robot_profile
from casbot_calib.model_context import BUNDLE_RELATIVE, load_context, resolve_context, MODEL_ID
from casbot_calib.model_sources import model_directory
from casbot_calib.dual_model import nominal_dual_arm, physical_model
from casbot_calib.kinematics import nominal_kinematics, default_config, geometry_manifest
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
sys.modules['mujoco']=None
from casbot_calib.model_context import load_context
from casbot_calib.kinematics import nominal_kinematics, default_config
context=load_context()
engine,_=nominal_kinematics(context.kinematic_config())
assert len(engine.model.joints)==14
assert context.collision_profile().check(context.home)['distances'].min()>.01
"""
    subprocess.run([sys.executable,'-c',code],check=True,capture_output=True,text=True)


def test_deprecated_assets_are_not_runtime_options():
    from importlib.resources import files
    from casbot_calib.model_runtime import model_for
    assert not files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf').is_file()
    with pytest.raises(ValueError, match='historical records'):
        model_for(dict(model='casbot_dual_arm_benchmark_nominal'))


@pytest.mark.parametrize('file,kind',[('model.urdf','bytes'),('collision.npz','bytes'),
    ('collision.json','exclude'),('trical_scene.json','scene'),('model.json','profile')])
def test_changed_profile_geometry_exclusions_or_scene_is_rejected(tmp_path,file,kind):
    # Small descriptors/cache only: relative mesh paths are not read by native FK.
    root=tmp_path/'copy';shutil.copytree(model_directory() / BUNDLE_RELATIVE,root,ignore=shutil.ignore_patterns('visual_meshes','*.xml'))
    path=root/file
    if kind=='bytes':path.write_bytes(path.read_bytes()+b'\n')
    else:
        payload=json.loads(path.read_text())
        if kind=='profile':payload['profile']['gravity_world_m_s2'][0]=1.
        elif kind=='exclude':payload['excluded_pairs'].clear()
        else:payload['axes_sensor']['left'][0][0]+=.1
        path.write_text(json.dumps(payload))
    with pytest.raises(ValueError):robot_profile(root)


def test_unhydrated_lfs_collision_cache_gives_an_actionable_error(tmp_path):
    root=tmp_path/'copy';shutil.copytree(model_directory() / BUNDLE_RELATIVE,root,ignore=shutil.ignore_patterns('visual_meshes','*.xml'))
    (root/'collision.npz').write_text('version https://git-lfs.github.com/spec/v1\noid sha256:example\nsize 1\n')
    with pytest.raises(ValueError,match='git lfs pull'):robot_profile(root)


@pytest.fixture(scope="module")
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


def project_paths(env, cwd):
    code = '''
import json
from pathlib import Path
from casbot_calib import project_paths, shm_data, shm_mock
from casbot_calib.model_sources import model_directory
print(json.dumps(dict(root=str(project_paths.ROOT), data=str(shm_data.ROOT),
                     mock=str(shm_mock.ROOT), models=str(model_directory()),
                     package_models=str(Path(project_paths.__file__).parent/'assets/models'),
                     default=str(Path(project_paths.__file__).resolve().parents[2]))))
'''
    return json.loads(subprocess.check_output([sys.executable, '-c', code], env=env, cwd=cwd, text=True))


def test_source_default_is_independent_of_working_directory(tmp_path):
    env = dict(os.environ)
    env.pop('CASBOT_PROJECT_ROOT', None)
    result = project_paths(env, tmp_path)
    assert result['root'] == result['data'] == result['mock'] == result['default']


def test_installed_project_root_keeps_package_resources(tmp_path):
    env = dict(os.environ, CASBOT_PROJECT_ROOT=str(tmp_path))
    result = project_paths(env, tmp_path)
    assert result['root'] == result['data'] == result['mock'] == str(tmp_path)
    packaged = Path(result['package_models'])
    expected = packaged if packaged.is_dir() else tmp_path / 'python/casbot_calib/assets/models'
    assert Path(result['models']) == expected


def test_model_resolution_prefers_packaged_directory_then_project_root(tmp_path, monkeypatch):
    from casbot_calib import model_sources, project_paths
    package = tmp_path / 'package'
    project = tmp_path / 'project'
    monkeypatch.setattr(model_sources, 'files', lambda _: package)
    monkeypatch.setattr(project_paths, 'ROOT', project)
    assert model_sources.model_directory() == project / 'python/casbot_calib/assets/models'
    (package / 'assets/models').mkdir(parents=True)
    assert model_sources.model_directory() == package / 'assets/models'
