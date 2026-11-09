"""Default selection is one source, not a legacy fallback or a test-result alias."""
import copy
import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from casbot_calib import _core
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.model_context import load_context
from casbot_calib.model_runtime import context_for,model_for,urdf_for,collision_for
from casbot_calib.kinematics import default_config,workflow_config,nominal_kinematics
from casbot_calib.kinematic_design import k3_config
from casbot_calib.staged_development import accepted_development
from casbot_calib.data import write_json

ROOT=Path(__file__).resolve().parents[1]


def test_all_default_factories_bind_one_model_and_fixture():
    context=load_context();engine,source=nominal_dual_arm()
    assert source['model_context']==context.identity and engine.model.model_id==context.model_id
    assert context_for({}).identity==context.identity and urdf_for({})==context.urdf
    assert model_for({})[1]==source and collision_for({}).metadata['model_context']==context.identity
    k1=default_config();k2=workflow_config();k3=k3_config()
    assert all(c['model_context']==context.identity for c in (k1,k2,k3))
    assert k2==k3 and k2['fixture']==k1['fixture'] and len(k1['fixture']['balls'])==4
    assert k2['routes']==json.loads((ROOT/'configs/trical_1105_release_turn_reference.json').read_text())['results']
    assert nominal_kinematics()[1]['model_context']==context.identity
    assert len(engine.frame_names())==155


def test_deprecated_assets_are_not_runtime_options():
    from importlib.resources import files
    assert not files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf').is_file()
    with pytest.raises(ValueError,match='historical records'):model_for(dict(model='casbot_dual_arm_benchmark_nominal'))
    with pytest.raises(ValueError,match='DVT2 K1'):default_config(ROOT/'configs/kinematics_trical_mock.yaml')


@pytest.mark.parametrize('name',['staged_dual_arm.yaml','staged_dual_arm_p0.yaml','sdk_mock.yaml','hardware_field.yaml','hardware_friction.yaml','hardware_inertia.yaml'])
def test_compatibility_config_names_resolve_same_context(name):
    old_entry=yaml.safe_load((ROOT/'configs'/name).read_text());explicit=yaml.safe_load((ROOT/'configs/dvt2'/name).read_text())
    assert context_for(old_entry).identity==context_for(explicit).identity==load_context().identity
    for key in set(old_entry)&{'home_rad','initial_q','control_period_ns','command_hz'}:assert old_entry[key]==explicit[key]


def test_r2_cannot_silently_promote_rejected_r1(tmp_path):
    root=tmp_path/'root';root.mkdir();state=dict(round='R1',passed=False);write_json(root/'workflow.json',state)
    with pytest.raises(ValueError,match='development-verification'):accepted_development(root)
    assert json.loads((root/'workflow.json').read_text())==state
    write_json(root/'workflow.json',dict(round='R1',passed=True))
    assert accepted_development(root)['source']=='original accepted R1 checkpoint'


def test_serialized_untagged_snapshot_is_not_rebound_to_the_new_default(tmp_path):
    from casbot_calib import hardware_field as field
    from casbot_calib.model_runtime import check_same_context
    profile=field.load_profile(ROOT/'configs/hardware_field.yaml')
    old=copy.deepcopy(profile);old.pop('model_context')
    # A frozen old request is not a new config: missing identity must not be inferred.
    snapshot=tmp_path/'snapshot';snapshot.mkdir()
    write_json(snapshot/'request.json',dict(config=old,domain_id=227));write_json(snapshot/'manifest.json',{})
    with pytest.raises(ValueError,match='contexts differ'):field.snapshot_binding(snapshot,profile)
    # A new config using the current CAD scene can be bound before its first capture.
    config_path=tmp_path/'fresh.yaml';old.pop('scene');old['collision_scene']=str(ROOT/'configs/collision_scene.yaml')
    config_path.write_text(yaml.safe_dump(old));assert field.load_profile(config_path)['model_context']==profile['model_context']
    with pytest.raises(ValueError,match='context differs'):check_same_context(dict(model_context={'old':'model'}),dict(model_context={'old':'model'}))


def test_fresh_mock_config_binds_default_before_recording(tmp_path):
    from casbot_calib.sdk_mock import mock_config
    c=yaml.safe_load((ROOT/'configs/sdk_mock.yaml').read_text());c.pop('model_context')
    path=tmp_path/'fresh_mock.yaml';path.write_text(yaml.safe_dump(c))
    assert mock_config(path)['model_context']==load_context().identity
