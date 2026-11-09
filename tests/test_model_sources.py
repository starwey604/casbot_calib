"""N1 source discovery, resources and explicit assembly selection."""
from dataclasses import replace
import json
from pathlib import Path
import pytest
from casbot_calib import _core
from casbot_calib.model_sources import MODEL_ID,source_catalog,dvt2_source,inspect_source,model_directory


@pytest.mark.parametrize('variant,count',[('a2',51),('a2_ff',29),('b1',40),('b2',51),('b3',29)])
def test_all_sources_and_meshes_load_without_changing_the_default(variant,count):
    report=inspect_source(variant);source=dvt2_source(variant)
    assert report['passed'] and report['source_only'] and not report['default_model_changed']
    joints=_core.ArmDynamics.from_urdf(str(source.urdf)).model.joints
    assert len(joints)==count
    assert tuple(j.name for j in joints if j.name in source.calibration_joint_names)==source.calibration_joint_names
    assert len(source.calibration_joint_names)==14 and report['urdf_meshes']>0
    assert len(report['files'])==report['referenced_files']
    from casbot_calib.dual_model import nominal_dual_arm
    engine,old=nominal_dual_arm()
    assert len(engine.model.joints)==14 and 'bare_flange_1105_03_gauge_v2' in old['source']


def test_source_paths_are_independent_of_cwd_and_accept_an_explicit_install_root(tmp_path,monkeypatch):
    source=dvt2_source('a2_ff');monkeypatch.chdir(tmp_path)
    assert dvt2_source('a2_ff',models_directory=model_directory())==source
    assert source.resolve_asset('collision/CASBOTHL12/base_link.obj').is_file()
    assert '/下载/' not in str(source.directory)


@pytest.mark.parametrize('variant',[None,'latest','nominal','../a2'])
def test_no_implicit_default_or_silent_old_model_fallback(variant):
    with pytest.raises(ValueError,match='explicit'):dvt2_source(variant)


@pytest.mark.parametrize('uri',['../robot.urdf','/tmp/mesh.obj','C:/mesh.obj','package://unknown/mesh.obj','visual\\mesh.obj'])
def test_assets_are_resolved_only_within_the_declared_root(uri):
    with pytest.raises(ValueError):dvt2_source('a2_ff').resolve_asset(uri)


def test_missing_mesh_and_lfs_pointer_are_reported_instead_of_loaded(tmp_path):
    source=replace(dvt2_source('a2_ff'),asset_root=tmp_path)
    with pytest.raises(FileNotFoundError):source.resolve_asset('mesh.obj')
    (tmp_path/'mesh.obj').write_text('version https://git-lfs.github.com/spec/v1\noid sha256:example\nsize 1\n')
    with pytest.raises(ValueError,match='git lfs pull'):source.resolve_asset('mesh.obj')


def test_changed_urdf_identity_is_not_accepted(tmp_path):
    catalog=source_catalog();root=tmp_path/MODEL_ID;root.mkdir();(root/'catalog.json').write_text(json.dumps(catalog))
    urdf=root/catalog['variants']['a2_ff']['urdf'];urdf.parent.mkdir(parents=True)
    urdf.write_text('<robot name="different"/>')
    with pytest.raises(ValueError,match='differs from its catalog'):dvt2_source('a2_ff',models_directory=tmp_path)
