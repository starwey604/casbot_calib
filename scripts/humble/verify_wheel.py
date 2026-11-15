"""Verify the small non-editable wheel against external project model assets."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[2]


def verify(wheel, output):
    output.mkdir(parents=True, exist_ok=False)
    unpacked = (output / 'unpacked').resolve()
    with zipfile.ZipFile(wheel) as archive:
        names = archive.namelist()
        assert wheel.stat().st_size < 4 * 1024**2
        assert any(name.startswith('casbot_calib/_core.') and name.endswith('.so') for name in names)
        assert not any(name.startswith(('include/', 'lib/', 'casbot_calib/assets/models/',
                                       'casbot_calib/assets/fieldwork_display/')) for name in names)
        for name in ('trical_measurement_defaults.json', 'trical_1105_release_turn_reference.json'):
            assert 'casbot_calib/assets/' + name in names
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
from casbot_calib.model_context import load_context
from casbot_calib.dvt2_bundle import robot_profile
from casbot_calib.trical_assembly import asset_directory,collision_assets
assert Path(casbot_calib.__file__).is_relative_to(sys.argv[1])
assert Path(_core.__file__).is_relative_to(sys.argv[1])
context=load_context();engine=context.engine;q=np.zeros(14)
from casbot_calib.shm_data import URDF
assert URDF == context.urdf and URDF.is_file()
np.testing.assert_allclose(engine.workspace().nonlinear_effects(q,np.ones(14)*.2),engine.gravity(q)+engine.velocity_bias(q,np.ones(14)*.2),atol=1e-13)
assert robot_profile(context.bundle).check(context.home)["distances"].min()>.01
collision_assets(asset_directory(context.bundle))
cal=_core.TricalCalibration();pose=_core.ArmPose();offset=np.array([.001,-.002,.003])
np.testing.assert_allclose(_core.trical_offset(_core.trical_readings(pose,offset,cal),cal),offset,atol=1e-14)
from casbot_calib.kinematics import nominal_kinematics
kin,_=nominal_kinematics()
geometry=kin.point_geometry_batch(q[None,:],"left_trical_tcp",np.zeros((1,3)))
np.testing.assert_allclose(geometry["positions"][0],kin.frame_pose(q,"left_trical_tcp")[:3,3],atol=1e-14)
print(json.dumps(dict(passed=True,core_path=_core.__file__,backend=engine.backend_version(),joints=14)))
'''
    result = json.loads(subprocess.check_output([sys.executable, '-I', '-c', probe, str(unpacked)], text=True,
                       env=dict(os.environ, CASBOT_PROJECT_ROOT=str(ROOT))))
    result.update(wheel=wheel.name, wheel_bytes=wheel.stat().st_size)
    (output / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print('Small installed wheel, external collision assets and TriCal: PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheel', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    verify(args.wheel, args.output)
