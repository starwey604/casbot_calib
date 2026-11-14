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
from casbot_calib.kinematics import default_config,nominal_kinematics
from casbot_calib.data import write_json

ROOT=Path(__file__).resolve().parents[1]


def test_deprecated_assets_are_not_runtime_options():
    from importlib.resources import files
    assert not files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf').is_file()
    with pytest.raises(ValueError,match='historical records'):model_for(dict(model='casbot_dual_arm_benchmark_nominal'))


@pytest.mark.parametrize('name',['staged_dual_arm.yaml'])
def test_compatibility_config_names_resolve_same_context(name):
    old_entry=yaml.safe_load((ROOT/'configs'/name).read_text());explicit=yaml.safe_load((ROOT/'configs/dvt2'/name).read_text())
    assert context_for(old_entry).identity==context_for(explicit).identity==load_context().identity
    for key in set(old_entry)&{'home_rad','initial_q','control_period_ns','command_hz'}:assert old_entry[key]==explicit[key]




