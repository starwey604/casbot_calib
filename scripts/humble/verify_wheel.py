"""Verify a non-editable wheel with source redirects and Python physics libraries disabled."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import zipfile


def verify(wheel, output):
    output.mkdir(parents=True, exist_ok=False)
    unpacked = (output / 'unpacked').resolve()
    with zipfile.ZipFile(wheel) as archive:
        names = archive.namelist()
        assert any(name.startswith('casbot_calib/_core.') and name.endswith('.so') for name in names)
        assert not any(name.startswith(('include/', 'lib/')) for name in names), 'Core development files leaked into Python wheel'
        archive.extractall(unpacked)
    probe = '''
import json,sys
from pathlib import Path
sys.meta_path = [finder for finder in sys.meta_path if "_casbot_calib_editable" not in type(finder).__module__]
sys.path.insert(0,sys.argv[1])
for name in ("pinocchio","coal","mujoco"):sys.modules[name]=None
import numpy as np
import casbot_calib
from casbot_calib import _core
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.kinematics import nominal_kinematics,default_config,calibration,workflow_config
from casbot_calib.trical_fixture import fixture_profile
from casbot_calib.kinematic_parameters import KinematicParameters
from casbot_calib.kinematic_design import k3_config
from casbot_calib.kinematic_acceptance import repeatability
from casbot_calib.kinematic_metrology import UNITS
from casbot_calib.kinematic_presentation import scene as write_trical_scene
from casbot_calib.model_sources import source_catalog,dvt2_source,inspect_source,model_directory
from casbot_calib.dvt2_bundle import load_bundle,robot_profile,viewer_adapter
assert Path(casbot_calib.__file__).is_relative_to(sys.argv[1])
assert Path(_core.__file__).is_relative_to(sys.argv[1])
catalog=source_catalog();source_variants=catalog["variants"]
for variant in source_variants:
    model_source=dvt2_source(variant)
    assert model_source.directory.is_relative_to(sys.argv[1])
    assert inspect_source(variant)["passed"]
    assert len(_core.ArmDynamics.from_urdf(str(model_source.urdf)).model.joints)==len(source_variants[variant]["active_joint_names"])
asset_files=[p for p in model_source.asset_root.rglob("*") if p.is_file()]
assert len(asset_files)==catalog["asset_inventory"]["files"]
assert sum(p.stat().st_size for p in asset_files)==catalog["asset_inventory"]["bytes"]
candidate=model_directory()/"casbot_hl12p1_dvt2/derived/a2ff_stock_hands_candidate_v1"
new,descriptor=load_bundle(candidate)
assert len(new.model.joints)==14 and not descriptor["assembly_confirmed"]
assert len(viewer_adapter(candidate).mappings)==14
assert robot_profile(candidate).check(descriptor["profile"]["home_rad"])["distances"].min()>.01
assert all((candidate/mesh.get("file")).resolve().is_file() for mesh in __import__("xml.etree.ElementTree",fromlist=["parse"]).parse(candidate/"scene.xml").findall("./asset/mesh") if mesh.get("file"))
engine,source=nominal_dual_arm();q=np.zeros(14)
kinematics,manifest=nominal_kinematics()
assert "left_trical_tcp" in kinematics.frame_names() and "right_trical_tcp" in kinematics.frame_names()
assert kinematics.frame_batch(np.zeros((2,14)),"left_trical_tcp")["jacobians"].shape==(2,6,14)
sensor=_core.ArmPose();delta=np.array([.001,-.002,.003]);cal=calibration(default_config())
assert np.max(np.abs(_core.trical_readings(sensor,delta,cal)-delta))<1e-14
workflow=workflow_config();measurement,_=nominal_kinematics(workflow)
scene=fixture_profile(measurement,workflow)
assert "ball_1_stem" in scene.names
geometry=measurement.point_geometry_batch(q[None,:],"left_trical_tcp",np.zeros((1,3)))
assert geometry["placement_jacobians"].shape==(1,3,84)
parameters=KinematicParameters(measurement);assert parameters.size==60
assert np.max(np.abs(parameters.build(np.zeros(60)).frame_pose(q,"left_trical_tcp")-measurement.frame_pose(q,"left_trical_tcp")))<1e-12
geometric_profile=k3_config();assert len(geometric_profile["fixture"]["balls"])==6
assert geometric_profile["parameter_policy"]["fit_enabled"] is True
geometric_engine,_=nominal_kinematics(geometric_profile)
assert np.max(np.abs(geometric_engine.frame_pose(q,"left_trical_tcp")-measurement.frame_pose(q,"left_trical_tcp")))<1e-12
assert repeatability(parameters,np.zeros((3,60)),np.zeros((2,14)),np.array(["left_trical_tcp","right_trical_tcp"]))["passed"]
assert UNITS["position_meter_m"]=="m" and callable(write_trical_scene)
indicator=_core.TricalMockConfig();indicator.noise_std=0.;indicator.quantum=0.
drive=_core.ArmSimulationConfig();drive.gain=np.full(14,2.)
virtual=_core.TricalMockSession(measurement,drive,indicator,q,q)
virtual.select_target("left_trical_tcp","ball_1",measurement.frame_pose(q,"left_trical_tcp")[:3,3])
assert len(virtual.advance_to(10_000_000)[1])==3
config=_core.ArmSimulationConfig();config.gain=np.full(14,2.)
config.current_tau=0.;config.command_delay_ns=0;config.position_noise=0.;config.current_noise=0.
commands=np.zeros((1,14,5));commands[0,:,2]=engine.gravity(q)
observations,_=_core.ArmSimulationRunner(engine,config).run(np.array([0],dtype=np.int64),commands,2_000_000,q,q)
assert observations["q"].shape==(3,14) and np.isfinite(observations["q"]).all()
assert np.max(np.abs(observations["q"]))<1e-10
print(json.dumps(dict(passed=True,core_path=_core.__file__,backend=engine.backend_version(),joints=14,dvt2_source_variants=list(source_variants),dvt2_default_changed=False,dvt2_candidate_loaded=True,python_physics_imports_disabled=True,editable_disabled=True)))
'''
    result = json.loads(subprocess.check_output([sys.executable, '-I', '-c', probe, str(unpacked)], text=True))
    result.update(wheel=wheel.name, wheel_bytes=wheel.stat().st_size)
    (output / 'verification.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print('Independent wheel import, five DVT2 sources, derived candidate, existing default and 14-axis native Mock: PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheel', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    verify(args.wheel, args.output)
