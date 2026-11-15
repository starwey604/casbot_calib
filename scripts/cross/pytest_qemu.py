#!/usr/bin/env python3
"""Non-realtime selection for the installed ARM64 wheel, sharing pytest slow markers."""
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[2]

WALL_CLOCK = {
    'test_600k_frame_binary_read_and_summary_benchmark',
    'test_dpp_ridge_grid_selects_optimal_ridge_under_250ms',
    'test_static_move_matches_bernstein_bounds_and_runs_fast',
    'test_h1_full_offline_planning_benchmark',
    'test_h2_full_offline_planning_and_duration_gate',
    'test_run_logged_does_not_wait_for_inherited_stdout',
}
LIVE_SHM = {
    'test_dynamics_h1_shell_end_to_end_and_resume',
    'test_three_stage_closed_loop_with_physical_shm_mock',
    'test_live_native_executor_binary_vs_csv_exact_match',
    'test_eight_windows_physics_fit',
    'test_only_zero_write_no_fault_start_mismatch_can_replan',
    'test_fault_during_execution_ends_collection_without_replan',
    'test_native_observed_velocity_ends_and_records_trigger',
    'test_one_enable_cycle_ready_then_h1_physical_capture_and_analysis',
    'test_preparation_fault_disables_without_entering_formal_reference',
    'test_desk_control_remote_mock_and_settings',
    'test_bridge_zero_reference_full_protocol_with_explicit_zero_gravity_fixture',
    'test_relative_reference_physical_loop_is_not_target_echo',
    'test_physics_observer_has_no_command_sender',
    'test_direct_effort_is_quantized_and_stops_with_disable',
    'test_physics_drive_fault_ends_no_reset_or_reenable',
    'test_owner_cleans_hard_killed_mock_without_touching_another',
    'test_mock_accepts_shared_inertia_viscous_settings_and_rejects_implicit_dry_friction',
}


def exclusion(name, fixtures, filename):
    if filename == 'test_orin_cross_scripts.py':
        return 'amd64 Docker build orchestration (tested on host)'
    if name == 'test_preview_widgets_without_graphics_or_control':
        return 'host GUI extra (imgui_bundle)'
    if name in WALL_CLOCK:
        return 'native wall-clock performance limit'
    if name in LIVE_SHM or {'master', 'owner', 'client'} & set(fixtures):
        return 'live SHM / physical Mock scheduling'
    return None


@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(config, items):
    kept, removed, reasons = [], [], {}
    for item in items:
        reason = exclusion(item.originalname or item.name, item.fixturenames, Path(item.path).name)
        if reason:
            removed.append(item)
            reasons[item.nodeid] = reason
        else:
            kept.append(item)
    report = Path(os.environ['CASBOT_QEMU_REPORT'])
    previous = sorted(path for path in report.parent.glob('*/pytest.xml') if path.parent.name < report.name)
    durations = {}
    if previous:
        for case in ET.parse(previous[-1]).iter('testcase'):
            nodeid = case.get('classname').replace('.', '/') + '.py::' + case.get('name')
            durations[nodeid] = float(case.get('time', '0'))
        ordered = sorted(kept, key=lambda item: -durations.get(item.nodeid, 0.0))
        # xdist prefetches at least two tests per worker. Pair long/short tests
        # so its initial queues spread the expensive kernels across CPUs.
        kept = []
        for index in range((len(ordered) + 1) // 2):
            kept.append(ordered[index])
            opposite = len(ordered) - 1 - index
            if opposite > index:
                kept.append(ordered[opposite])
    items[:] = kept
    config.hook.pytest_deselected(items=removed)
    if os.environ.get('PYTEST_XDIST_WORKER', 'gw0') == 'gw0':
        report = Path(os.environ['CASBOT_QEMU_REPORT'])
        (report / 'pytest-selection.json').write_text(json.dumps(
            dict(selected=[item.nodeid for item in kept], excluded=reasons,
                 duration_history=str(previous[-1]) if previous else None), indent=2) + '\n')


def main():
    from casbot_calib import _core
    report = Path(os.environ['CASBOT_QEMU_REPORT'])
    print(f'ARM64 regression extension: {_core.__file__}', flush=True)
    os.chdir(ROOT)
    sys.path.insert(0, str(ROOT))
    workers = min(4, len(os.sched_getaffinity(0)))
    args = ['-p', 'xdist.plugin', '-p', 'scripts.cross.pytest_qemu', '-n', str(workers), '--maxschedchunk=1', '-q', '--durations=15', '-p', 'no:cacheprovider', '--junitxml', str(report / 'pytest.xml')]
    args += sys.argv[1:] or ['tests']
    result = pytest.main(args)
    (report / 'pytest-result.json').write_text(json.dumps(dict(exit_code=int(result))) + '\n')
    raise SystemExit(result)


if __name__ == '__main__':
    main()
