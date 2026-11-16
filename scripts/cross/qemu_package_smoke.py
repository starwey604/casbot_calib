#!/usr/bin/env python3
"""Exercise the installed ARM64 numerical stack, packaged binaries and offline workflow."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

ROOT = Path(__file__).resolve().parents[2]


def run(*args, env=None):
    started = time.perf_counter()
    subprocess.run(list(map(str, args)), check=True, env=env)
    return round(time.perf_counter() - started, 3)


def main():
    cpp, report = Path(sys.argv[1]) / 'cpp', Path(sys.argv[2])
    tests = Path((Path(sys.argv[1]) / 'test-prefix.txt').read_text())
    pytest_args = sys.argv[4:] if len(sys.argv) > 3 and sys.argv[3] == '--pytest' else None
    started = time.perf_counter()
    sys.path.insert(0, str(ROOT / 'scripts/shm'))
    from link_math_bindings import link_bindings
    link_bindings()
    cases = ((1, 'core-first'), (1, 'math-first'), (4, 'core-first'), (4, 'math-first'))
    def check_order(case):
        threads, order = case
        return run(sys.executable, ROOT / 'scripts/cross/qemu_math.py', report / f'math-{threads}-{order}', order,
            env=dict(os.environ, OMP_NUM_THREADS=str(threads)))
    def check_cpp():
        urdf = ROOT / 'python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2/model.urdf'
        times = {}
        times['core_unit'] = run(tests / 'bin/core_unit_tests', urdf)
        times['shm_unit'] = run(tests / 'bin/shm_unit_tests', ROOT / 'tests/fixtures/shm')
        times['abi'] = run(cpp / 'bin/shm_abi_check')
        for threads in (1, 4):
            times[f'openmp_{threads}'] = run(tests / 'bin/openmp_probe', threads, env=dict(os.environ, OMP_NUM_THREADS=str(threads)))
        return times
    with ThreadPoolExecutor(max_workers=6) as pool:
        pytest_future = None
        if pytest_args is not None:
            env = dict(os.environ, OMP_NUM_THREADS='1', CASBOT_SHM_TEST_BIN=str(cpp / 'bin'),
                       CASBOT_SHM_EXECUTOR=str(cpp / 'bin/shm_executor'), CASBOT_QEMU_REPORT=str(report))
            pytest_future = pool.submit(run, sys.executable, ROOT / 'scripts/cross/pytest_qemu.py', *pytest_args, env=env)
        futures = [pool.submit(check_order, case) for case in cases]
        futures.append(pool.submit(check_cpp))
        imports_started = time.perf_counter()
        import numpy as np
        from casbot_calib import _core
        import casbot_calib
        import pinocchio, coal
        for module in (casbot_calib, _core):
            assert Path(module.__file__).is_relative_to(sys.prefix), module.__file__
        versions = dict(pinocchio=pinocchio.__version__, coal=coal.__version__)
        imports_seconds = round(time.perf_counter() - imports_started, 3)
        cli_started = time.perf_counter()
        cli = subprocess.run(['bash', str(ROOT / 'calib/dynamics_h1.sh'), '--help'],
                             env=dict(os.environ, PYTHONVERBOSE='1'), capture_output=True, text=True, check=True)
        (report / 'cli-imports.log').write_text(cli.stderr)
        assert str(Path(casbot_calib.__file__)) in cli.stderr
        assert str(Path(_core.__file__)) in cli.stderr

        cli_seconds = round(time.perf_counter() - cli_started, 3)
        commands = {f'math_{threads}_{order}': future.result() for (threads, order), future in zip(cases, futures)}
        commands.update(futures[-1].result())
        comparisons_started = time.perf_counter()
        comparisons = {}
        with np.load(report / 'math-1-core-first.npz') as baseline:
            for threads, order in ((1, 'math-first'), (4, 'core-first'), (4, 'math-first')):
                with np.load(report / f'math-{threads}-{order}.npz') as actual:
                    for key in ('torque', 'distances'):
                        error = float(np.max(np.abs(actual[key] - baseline[key])))
                        assert error < 1e-12, (threads, order, key, error)
                        comparisons[f'{threads}-{order}-{key}'] = error
                    np.testing.assert_array_equal(actual['worst_pair'], baseline['worst_pair'])
                    reference_error = float(np.max(np.abs(actual['distances'] - baseline['reference_distances'])))
                    assert reference_error < 1e-12, (threads, order, reference_error)
                    comparisons[f'{threads}-{order}-coal-reference'] = reference_error
        for program in ('shm_executor', 'shm_trical_executor', 'shm_mock_server'):
            libraries = subprocess.check_output(['ldd', str(cpp / 'bin' / program)], text=True)
            assert 'not found' not in libraries, libraries
            assert str(cpp / 'bin/../lib/libomp.so.5') in libraries, libraries
            assert 'libpinocchio_default.so.4.0.0' in libraries and 'libcoal.so.3.0.3' in libraries, libraries
        timings = dict(commands, parent_imports=imports_seconds, cli=cli_seconds,
                       comparisons_and_ldd=round(time.perf_counter() - comparisons_started, 3))
        result = dict(status='passed', extension=_core.__file__, packages=versions,
                      comparison_max_errors=comparisons, timings_seconds=timings,
                      smoke_seconds=round(time.perf_counter() - started, 3))
        (report / 'smoke.json').write_text(json.dumps(result, indent=2) + '\n')
        print('完整 ARM64/QEMU 冒烟通过:', json.dumps(result), flush=True)
        if pytest_future is not None:
            pytest_future.result()



if __name__ == '__main__':
    main()
