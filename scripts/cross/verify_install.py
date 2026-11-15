#!/usr/bin/env python3
"""Exercise first/update prebuilt installs in an isolated ARM64 project, offline."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import re
import sys
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first', type=Path, default=ROOT / 'dist/orin-aarch64')
    parser.add_argument('--update', type=Path, default=ROOT / 'dist/orin-aarch64')
    args = parser.parse_args()
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    stage = ROOT / 'build' / f'orin-install-{stamp}'
    report = ROOT / 'artifacts/orin-install' / stamp
    report.mkdir(parents=True)
    for directory in ('calib', 'configs', 'scripts', 'python'):
        shutil.copytree(ROOT / directory, stage / directory,
                        ignore=shutil.ignore_patterns('assets', '__pycache__'))
    sys.path.insert(0, str(ROOT / 'scripts/shm'))
    from runtime_assets import FILES
    for relative in FILES:
        (stage / relative).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / relative, stage / relative)
    packages = stage / 'artifacts/prebuilt_aarch64'
    shutil.copytree(args.first, packages)
    wheelhouse = ROOT / 'artifacts/wheelhouse_aarch64'
    test_prefix = Path(json.loads((args.update / 'manifest.json').read_text())['test_prefix'])
    base = ['docker', 'run', '--rm', '--platform', 'linux/arm64', '--network', 'none',
            '--user', f'{os.getuid()}:{os.getgid()}',
            '-v', f'{stage}:{stage}', '-v', f'{test_prefix}:{test_prefix}:ro', '-v', f'{wheelhouse}:{wheelhouse}:ro', '-w', str(stage),
            'casbot-calib/orin-arm64-runtime:humble', 'env', '-u', 'PYTHONPATH', '-u', 'LD_LIBRARY_PATH',
            'OPENBLAS_NUM_THREADS=1', 'OMP_NUM_THREADS=4', 'MPLBACKEND=Agg', f'MPLCONFIGDIR={stage}/.mpl']
    times = {}
    def run(name, command):
        start = time.perf_counter()
        with (report / f'{name}.log').open('w') as log:
            subprocess.run(base + command, stdout=log, stderr=subprocess.STDOUT, check=True)
        times[name] = round(time.perf_counter() - start, 3)
        phases = dict((key, float(value)) for key, value in re.findall(r'SETUP_TIMING (\w+) ([0-9.]+)', (report / f'{name}.log').read_text()))
        if phases:
            times[name + '-phases'] = phases
        print(f'{name}: {times[name]}s {phases}', flush=True)
        (report / 'timings.json').write_text(json.dumps(times, indent=2) + '\n')
    setup = ['bash', 'calib/setup.sh', '--prebuilt', '--wheelhouse', str(wheelhouse)]
    run('first', setup)
    run('first-check', ['bash', 'calib/setup.sh', '--check'])
    # Retain operator configuration while replacing installed binaries/Python files.
    sentinel = stage / 'configs/operator-note.txt'
    sentinel.write_text('retain on update\n')
    obsolete = [stage / 'install/shm-core' / relative for relative in (
        'bin/shm_joint_test', 'bin/old_tests', 'lib/libcalib_core.a',
        'include/casbot_calib/simulator.hpp', 'lib/cmake/casbot_calib/old.cmake')]
    for path in obsolete:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('previous development install\n')
    shutil.copytree(args.update, packages, dirs_exist_ok=True)
    run('update', setup)
    assert sentinel.read_text() == 'retain on update\n'
    assert all(not path.exists() for path in obsolete)
    run('update-check', ['bash', 'calib/setup.sh', '--check'])
    code = '''
from pathlib import Path
from casbot_calib import _core
from casbot_calib.project_paths import ROOT
import sys
assert Path(_core.__file__).is_relative_to(Path(sys.prefix)), _core.__file__
assert ROOT == Path.cwd(), ROOT
print(_core.__file__, ROOT)
'''
    run('installed-path', [str(stage / '.venv-shm/bin/python'), '-c', code])
    run('h1-plan', ['env', 'PYTHONVERBOSE=1', 'bash', 'calib/dynamics_h1.sh', '--plan-only', '--robot', 'QEMU_INSTALL'])
    cli_log = (report / 'h1-plan.log').read_text()
    assert str(stage / '.venv-shm/lib/python3.10/site-packages/casbot_calib/__init__.py') in cli_log
    assert str(stage / '.venv-shm/lib/python3.10/site-packages/casbot_calib/_core.') in cli_log
    run('openmp', [str(test_prefix / 'bin/openmp_probe'), '4'])
    print(f'安装验证通过：{report}\n项目：{stage}', flush=True)


if __name__ == '__main__':
    main()
