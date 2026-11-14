"""A diagnostics on mapped SHM observation time (dev diagnostic tool)."""
import argparse
from pathlib import Path
import time
import numpy as np
from .data import write_json
from .pipeline.robot import DEFAULT_ROBOT_CONFIG, load_robot_config
from .probe_diagnostics import window_a
from .shm_data import ROOT
from .shm_observations import ObservationReader


def analyse(capture, output=None, *, robot_config=DEFAULT_ROBOT_CONFIG, window, joint_id=12, band=30.0, clock_segment=None):
    if joint_id not in range(12, 26):
        raise ValueError('使用双臂真实 ID12–25')
    if output is None:
        output = ROOT / 'artifacts/dev' / f'a_evidence_{time.strftime("%Y%m%d_%H%M%S")}_{time.monotonic_ns() % 1000000:06d}'
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    config = load_robot_config(robot_config)
    report = dict(
        source=str(Path(capture).resolve()),
        joint_id=joint_id,
        window_s=list(window),
        band_hz=band,
        source_timestamp_quantum_ns=1_000_000,
        effort_resolution_nm=0.01,
    )
    try:
        reader = ObservationReader(capture)
        obs, elapsed, scope = reader.window(window, clock_segment)
        report.update(scope, first_command_mono_ns=reader.origin)
        result = window_a(obs, elapsed, window, joint_id - 12, config, band, observation_time_ns=obs['observation_time_ns'])
        result['body_motion'] = dict(available=False, note='SHM 记录不含 IMU')
        report.update(available=True, A=result)
    except (ValueError, KeyError, np.linalg.LinAlgError) as error:
        report.update(available=False, unavailable_reason=str(error))
    write_json(output / 'report.json', report)
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture', type=Path)
    p.add_argument('--output', type=Path, default=None)
    p.add_argument('--robot-config', type=Path, default=DEFAULT_ROBOT_CONFIG)
    p.add_argument('--window', nargs=2, type=float, required=True)
    p.add_argument('--joint-id', type=int, choices=range(12, 26), default=12)
    p.add_argument('--band', type=float, default=30.0)
    p.add_argument('--clock-segment', type=int)
    a = p.parse_args(argv)
    try:
        out = a.output or (ROOT / 'artifacts/dev' / f'a_evidence_{time.strftime("%Y%m%d_%H%M%S")}_{time.monotonic_ns() % 1000000:06d}')
        report = analyse(a.capture, out, robot_config=a.robot_config, window=a.window, joint_id=a.joint_id, band=a.band, clock_segment=a.clock_segment)
        status = '可计算' if report['available'] else f'不可计算：{report["unavailable_reason"]}'
        print(f'A分析：{status}；{Path(out).resolve() / "report.json"}')
        return 0
    except (ValueError, OSError) as error:
        print('A错误：' + str(error), file=__import__('sys').stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
