"""Verify a non-editable wheel with source redirects and Python physics libraries disabled."""
import argparse
import hashlib
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
from casbot_calib.kinematics import nominal_kinematics,default_config,calibration
assert Path(casbot_calib.__file__).is_relative_to(sys.argv[1])
assert Path(_core.__file__).is_relative_to(sys.argv[1])
engine,source=nominal_dual_arm();q=np.zeros(14)
kinematics,manifest=nominal_kinematics()
assert "left_trical_tcp" in kinematics.frame_names() and "right_trical_tcp" in kinematics.frame_names()
assert kinematics.frame_batch(np.zeros((2,14)),"left_trical_tcp")["jacobians"].shape==(2,6,14)
sensor=_core.ArmPose();delta=np.array([.001,-.002,.003]);cal=calibration(default_config())
assert np.max(np.abs(_core.trical_readings(sensor,delta,cal)-delta))<1e-14
config=_core.ArmSimulationConfig();config.gain=np.full(14,2.)
config.current_tau=0.;config.command_delay_ns=0;config.position_noise=0.;config.current_noise=0.
commands=np.zeros((1,14,5));commands[0,:,2]=engine.gravity(q)
observations,_=_core.ArmSimulationRunner(engine,config).run(np.array([0],dtype=np.int64),commands,2_000_000,q,q)
assert observations["q"].shape==(3,14) and np.isfinite(observations["q"]).all()
assert np.max(np.abs(observations["q"]))<1e-10
print(json.dumps(dict(passed=True,core_path=_core.__file__,backend=engine.backend_version(),joints=14,python_physics_imports_disabled=True,editable_disabled=True)))
'''
    result = json.loads(subprocess.check_output([sys.executable, '-I', '-c', probe, str(unpacked)], text=True))
    result.update(wheel=wheel.name, wheel_sha256=hashlib.sha256(wheel.read_bytes()).hexdigest(), wheel_bytes=wheel.stat().st_size)
    (output / 'verification.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print('Independent wheel import, bundled nominal URDF and 14-axis native Mock: PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wheel', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    verify(args.wheel, args.output)
