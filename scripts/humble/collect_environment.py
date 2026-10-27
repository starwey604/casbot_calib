"""Record the actual Humble native environment, rather than infer it from a lock."""
import argparse
import hashlib
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def output(*command):
    return subprocess.check_output(command, text=True).strip()


def collect():
    import coal
    import numpy as np
    import pinocchio
    import rclpy
    from casbot_calib import _core
    assert sys.version_info[:2] == (3, 10)
    assert os.environ.get('ROS_DISTRO') == 'humble'
    assert not os.environ.get('CONDA_PREFIX')
    paths = list(sys.path) + [str(_core.__file__)]
    libraries = output('ldd', _core.__file__)
    forbidden = ('/miniforge', '/miniconda', '/anaconda', '/.conda/')
    assert not any(token in str(paths) + libraries for token in forbidden), 'Conda dependency leaked into Humble'
    first = coal.CollisionObject(coal.Sphere(.25), coal.Transform3s())
    second = coal.CollisionObject(coal.Sphere(.25), coal.Transform3s(np.eye(3), np.array([1., 0., 0.])))
    distance = coal.distance(first, second, coal.DistanceRequest(), coal.DistanceResult())
    assert abs(distance - .5) < 1e-12
    versions = {name: metadata.version(name) for name in ('numpy', 'scipy', 'matplotlib', 'pybind11', 'scikit-build-core', 'pytest', 'mujoco', 'Pillow')}
    return dict(passed=True, python=sys.version, executable=sys.executable,
        platform=platform.platform(), ros_distro=os.environ['ROS_DISTRO'],
        backend=_core.ArmDynamics.backend_version(), integration_backend=_core.SimulationRunner.integration_backend(),
        pinocchio=pinocchio.__version__, coal=getattr(coal, '__version__', 'see apt manifest'),
        rclpy_path=rclpy.__file__, core_path=_core.__file__, python_versions=versions,
        cmake=output('cmake', '--version').splitlines()[0], compiler=output('c++', '--version').splitlines()[0],
        system_packages=output('dpkg-query', '-W', '-f=${binary:Package}\t${Version}\n'),
        python_packages=output(sys.executable, '-m', 'pip', 'freeze'),
        core_sha256=hashlib.sha256(Path(_core.__file__).read_bytes()).hexdigest(),
        core_libraries=libraries, conda_paths_absent=True, coal_primitive_distance_m=distance)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = collect()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print('Native Humble imports, linked-library isolation and Coal distance: PASS')
