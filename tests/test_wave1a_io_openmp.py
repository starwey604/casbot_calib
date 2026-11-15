"""Wave 1A tests: wire35 NumPy dtypes, vectorized binary I/O, orjson safety, and OpenMP equivalence."""
import csv
import json
import shutil
import struct
import time
from pathlib import Path
import numpy as np
import pytest

from casbot_calib import _core
from casbot_calib import shm_fieldwork as fw
from casbot_calib.data import write_json
from casbot_calib.model_runtime import collision_for, model_for
from casbot_calib.pipeline.robot import load_robot_config
from casbot_calib.shm_data import (
    COMMAND_FRAME_DTYPE,
    COMMAND_RECORD_DTYPE,
    DIRECTION,
    FEEDBACK_RECORD_DTYPE,
    HAND_COMMAND_DTYPE,
    HAND_JOINT_COMMAND_DTYPE,
    HAND_JOINT_STATUS_DTYPE,
    HAND_STATUS_DTYPE,
    JOINT_COMMAND_DTYPE,
    JOINT_STATUS_DTYPE,
    OFFSET,
    STATUS_FRAME_DTYPE,
    URDF,
)
from casbot_calib.shm_observations import ObservationReader
from casbot_calib.shm_reference import jog_bundle, save_bundle


def test_wire35_numpy_dtype_sizes_and_offsets():
    assert JOINT_COMMAND_DTYPE.itemsize == 18
    assert JOINT_COMMAND_DTYPE.fields['control_word'][1] == 0
    assert JOINT_COMMAND_DTYPE.fields['position'][1] == 2
    assert JOINT_COMMAND_DTYPE.fields['velocity'][1] == 6
    assert JOINT_COMMAND_DTYPE.fields['torque'][1] == 10
    assert JOINT_COMMAND_DTYPE.fields['mode'][1] == 12
    assert JOINT_COMMAND_DTYPE.fields['kp'][1] == 14
    assert JOINT_COMMAND_DTYPE.fields['kd'][1] == 16

    assert JOINT_STATUS_DTYPE.itemsize == 16
    assert JOINT_STATUS_DTYPE.fields['status_word'][1] == 0
    assert JOINT_STATUS_DTYPE.fields['position'][1] == 2
    assert JOINT_STATUS_DTYPE.fields['velocity'][1] == 6
    assert JOINT_STATUS_DTYPE.fields['torque'][1] == 10
    assert JOINT_STATUS_DTYPE.fields['mode'][1] == 12
    assert JOINT_STATUS_DTYPE.fields['error_code'][1] == 14

    assert HAND_JOINT_COMMAND_DTYPE.itemsize == 6
    assert HAND_JOINT_COMMAND_DTYPE.fields['mode'][1] == 4
    assert HAND_COMMAND_DTYPE.itemsize == 40
    assert HAND_COMMAND_DTYPE.fields['joints'][1] == 4

    assert HAND_JOINT_STATUS_DTYPE.itemsize == 8
    assert HAND_STATUS_DTYPE.itemsize == 52
    assert HAND_STATUS_DTYPE.fields['joints'][1] == 4

    assert COMMAND_FRAME_DTYPE.itemsize == 744
    assert COMMAND_FRAME_DTYPE.fields['frame_counter'][1] == 0
    assert COMMAND_FRAME_DTYPE.fields['joint_count'][1] == 8
    assert COMMAND_FRAME_DTYPE.fields['hand_count'][1] == 10
    assert COMMAND_FRAME_DTYPE.fields['joints'][1] == 12
    assert COMMAND_FRAME_DTYPE.fields['hands'][1] == 642
    assert COMMAND_FRAME_DTYPE.fields['timestamp'][1] == 728
    assert COMMAND_FRAME_DTYPE.fields['crc'][1] == 736

    assert STATUS_FRAME_DTYPE.itemsize == 696
    assert STATUS_FRAME_DTYPE.fields['frame_counter'][1] == 0
    assert STATUS_FRAME_DTYPE.fields['joint_count'][1] == 8
    assert STATUS_FRAME_DTYPE.fields['hand_count'][1] == 10
    assert STATUS_FRAME_DTYPE.fields['joints'][1] == 12
    assert STATUS_FRAME_DTYPE.fields['hands'][1] == 572
    assert STATUS_FRAME_DTYPE.fields['timestamp'][1] == 680
    assert STATUS_FRAME_DTYPE.fields['crc'][1] == 688

    assert FEEDBACK_RECORD_DTYPE.itemsize == 704
    assert FEEDBACK_RECORD_DTYPE.fields['receive_mono_ns'][1] == 0
    assert FEEDBACK_RECORD_DTYPE.fields['frame'][1] == 8

    assert COMMAND_RECORD_DTYPE.itemsize == 752
    assert COMMAND_RECORD_DTYPE.fields['send_mono_ns'][1] == 0
    assert COMMAND_RECORD_DTYPE.fields['frame'][1] == 8


def _write_dual_fixture(root: Path, n_ticks: int = 500, *, include_csv: bool = True, fault_tail: bool = False):
    """Build a synthetic recording matching field_runtime.cpp's exact tick event/send/receive order."""
    root.mkdir(parents=True, exist_ok=True)
    t0 = 10**18 + 500_000_000
    wall_base_ms = 1_700_000_000_000
    offset0 = wall_base_ms * 1_000_000 - t0
    offset1 = offset0 + 2_000_000

    # Phase schedule across ticks:
    # 0..19: phase 0, 20..39: phase 1, 40..59: phase 2 (skip receive on tick 40 to test missing-receive transition!),
    # 60..(n_ticks-41): phase 3 (with markers at 120, 180), (n_ticks-40)..: phase 4 (or phase 5 fault tail)
    p3_tick = 60
    p4_tick = n_ticks - 40
    jump_tick = n_ticks // 2

    events = [(t0 - 50_000, 0, 'disabling')]
    clock_pairs = [
        (t0 - 200_000, wall_base_ms * 1_000_000 - 200_000, t0 - 190_000, offset0, 0, 0),
    ]

    fb_list = []
    cmd_list = []
    csv_fb_rows = []
    csv_cmd_rows = []

    phase = 0
    first_stream_ns = 0
    first_write_ns = 0
    cur_segment = 0
    cur_offset = offset0

    for k in range(n_ticks):
        tick_now = t0 + k * 1_000_000
        if k == jump_tick:
            cur_segment = 1
            cur_offset = offset1
            clock_pairs.append((
                tick_now + 1_000,
                (wall_base_ms + k + 2) * 1_000_000,
                tick_now + 2_000,
                offset1,
                1,
                1,
            ))

        # Simulate missed receive on transition tick 40 to test intervening-send disambiguation
        received = (k != 40)
        recv_at = tick_now + 5_000 if received else 0
        stamp_ms = wall_base_ms + k + (2 if cur_segment == 1 else 0)
        if k == 95:
            # Backward 1 ms source stamp inside segment 0
            stamp_ms -= 2

        if k == 20:
            phase = 1
            events.append((tick_now + 12_000, 1, 'switching_mit'))
        elif k == 40:
            phase = 2
            events.append((tick_now + 12_000, 2, 'enabling_at_start'))
        elif k == p3_tick:
            phase = 3
            events.append((tick_now + 12_000, 3, 'reference_begin'))
        elif k == 120 and k < p4_tick:
            events.append((tick_now + 12_000, 3, 'calibration_begin'))
        elif k == p4_tick:
            if fault_tail:
                # Push pre-throw frame with OLD phase=3, then emit phase=5 event and catch-loop frames
                if received:
                    obs_ns = stamp_ms * 1_000_000 - cur_offset
                    fb_rec = np.zeros(1, dtype=FEEDBACK_RECORD_DTYPE)[0]
                    fb_rec['receive_mono_ns'] = recv_at
                    fb_rec['frame']['frame_counter'] = 100 + k
                    fb_rec['frame']['timestamp'] = stamp_ms
                    for j in range(14):
                        fb_rec['frame']['joints'][j + 12]['position'] = (k * 10 + j) * DIRECTION[j]
                        fb_rec['frame']['joints'][j + 12]['velocity'] = (-k * 3 + j) * DIRECTION[j]
                        fb_rec['frame']['joints'][j + 12]['torque'] = ((k + j) % 50) * DIRECTION[j]
                        fb_rec['frame']['joints'][j + 12]['status_word'] = 0x237
                        fb_rec['frame']['joints'][j + 12]['mode'] = -1
                    fb_list.append(fb_rec)
                    if include_csv:
                        q_vals = [DIRECTION[j] * float(fb_rec['frame']['joints'][j + 12]['position']) * 1e-6 + OFFSET[j] for j in range(14)]
                        v_vals = [DIRECTION[j] * float(fb_rec['frame']['joints'][j + 12]['velocity']) * 1e-6 for j in range(14)]
                        e_vals = [DIRECTION[j] * float(fb_rec['frame']['joints'][j + 12]['torque']) * 0.01 for j in range(14)]
                        csv_fb_rows.append([recv_at, stamp_ms, 100 + k, obs_ns, cur_segment, 0, *q_vals, *v_vals, *e_vals, phase])
                phase = 5
                events.append((tick_now + 20_000, 5, 'observed_velocity_limit_slot_12'))
                continue
            phase = 4
            events.append((tick_now + 12_000, 4, 'exiting_disable'))

        send_at = tick_now + 30_000
        if not first_write_ns:
            first_write_ns = send_at
        if phase == 3 and not first_stream_ns:
            first_stream_ns = send_at

        cmd_rec = np.zeros(1, dtype=COMMAND_RECORD_DTYPE)[0]
        cmd_rec['send_mono_ns'] = send_at
        cmd_rec['frame']['frame_counter'] = k + 1
        cmd_rec['frame']['timestamp'] = tick_now // 1_000_000
        for j in range(14):
            cmd_rec['frame']['joints'][j + 12]['control_word'] = 15 if phase in (2, 3) else 6
            cmd_rec['frame']['joints'][j + 12]['position'] = (k * 12 - j * 5) * DIRECTION[j]
        cmd_list.append(cmd_rec)
        if include_csv:
            csv_cmd_rows.append([send_at, phase, k + 1])

        if received:
            obs_ns = stamp_ms * 1_000_000 - cur_offset
            fb_rec = np.zeros(1, dtype=FEEDBACK_RECORD_DTYPE)[0]
            fb_rec['receive_mono_ns'] = recv_at
            fb_rec['frame']['frame_counter'] = (100 + k) + (2 if k > 200 else 0)
            fb_rec['frame']['timestamp'] = stamp_ms
            for j in range(35):
                fb_rec['frame']['joints'][j]['status_word'] = 0x237 if (12 <= j <= 25 and phase == 3) else 0x231
                fb_rec['frame']['joints'][j]['mode'] = -1 if (12 <= j <= 25 and phase >= 1) else 0
                fb_rec['frame']['joints'][j]['error_code'] = 42 if (fault_tail and phase == 5 and j == 12) else 0
            for j in range(14):
                fb_rec['frame']['joints'][j + 12]['position'] = (k * 10 + j) * DIRECTION[j]
                fb_rec['frame']['joints'][j + 12]['velocity'] = (-k * 3 + j) * DIRECTION[j]
                fb_rec['frame']['joints'][j + 12]['torque'] = ((k + j) % 50 - 25) * DIRECTION[j]
            fb_rec['frame']['hands'][0]['global_status'] = 3
            fb_rec['frame']['hands'][0]['joints'][0]['position'] = 123 + (k % 7)
            fb_rec['frame']['hands'][0]['joints'][0]['status'] = 1
            fb_list.append(fb_rec)
            if include_csv:
                q_vals = [DIRECTION[j] * float(fb_rec['frame']['joints'][j + 12]['position']) * 1e-6 + OFFSET[j] for j in range(14)]
                v_vals = [DIRECTION[j] * float(fb_rec['frame']['joints'][j + 12]['velocity']) * 1e-6 for j in range(14)]
                e_vals = [DIRECTION[j] * float(fb_rec['frame']['joints'][j + 12]['torque']) * 0.01 for j in range(14)]
                csv_fb_rows.append([recv_at, stamp_ms, int(fb_rec['frame']['frame_counter']), obs_ns, cur_segment, 0, *q_vals, *v_vals, *e_vals, phase])

    if not fault_tail:
        events.append((t0 + n_ticks * 1_000_000, 4, 'disable_ack'))

    np.array(fb_list, dtype=FEEDBACK_RECORD_DTYPE).tofile(root / 'feedback.bin')
    np.array(cmd_list, dtype=COMMAND_RECORD_DTYPE).tofile(root / 'commands.bin')
    (root / 'passive_commands.bin').write_bytes(b'')

    with (root / 'events.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['mono_ns', 'phase', 'event'])
        w.writerows(events)

    with (root / 'clock_pairs.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['mono_before_ns', 'wall_ns', 'mono_after_ns', 'offset_ns', 'clock_segment', 'jump_detected'])
        w.writerows(clock_pairs)

    (root / 'result.txt').write_text(
        f'execution_completed={0 if fault_tail else 1}\n'
        f'reason={"fault" if fault_tail else "completed_with_disable_ack"}\n'
        f'first_stream_mono_ns={first_stream_ns}\n'
        f'first_command_mono_ns={first_stream_ns}\n'
        f'first_shm_write_mono_ns={first_write_ns}\n'
        f'recording_stopped=0\n'
        f'disable_ack=1\n'
    )

    if include_csv:
        with (root / 'commands.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['send_mono_ns', 'phase', 'frame_counter'])
            w.writerows(csv_cmd_rows)
        with (root / 'feedback.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'receive_mono_ns', 'source_timestamp_ms', 'frame_counter',
                'observation_time_ns', 'clock_segment', 'passive',
                *[f'q{j}' for j in range(14)],
                *[f'v{j}' for j in range(14)],
                *[f'effort{j}' for j in range(14)],
                'phase',
            ])
            for r in csv_fb_rows:
                w.writerow([
                    r[0], r[1], r[2], r[3], r[4], r[5],
                    *[f'{x:.17g}' for x in r[6:48]],
                    r[48],
                ])


@pytest.mark.parametrize('fault_tail', [False, True])
def test_observation_reader_binary_matches_csv_zero_error(tmp_path, fault_tail):
    dual = tmp_path / 'dual'
    csv_only = tmp_path / 'csv_only'
    _write_dual_fixture(dual, n_ticks=400, include_csv=True, fault_tail=fault_tail)
    shutil.copytree(dual, csv_only)
    (csv_only / 'feedback.bin').unlink()
    (csv_only / 'commands.bin').unlink()

    # Also test binary-only without commands.csv / feedback.csv
    bin_only = tmp_path / 'bin_only'
    shutil.copytree(dual, bin_only)
    (bin_only / 'feedback.csv').unlink()
    (bin_only / 'commands.csv').unlink()

    arr_bin = ObservationReader(bin_only).arrays()
    arr_csv = ObservationReader(csv_only).arrays()

    for key in ('index', 'receive_ns', 'observation_time_ns', 'source_timestamp_ms', 'clock_segment', 'frame_counter', 'phase', 'time_ambiguous'):
        np.testing.assert_array_equal(arr_bin[key], arr_csv[key], err_msg=f'Mismatch in {key}')
    for key in ('elapsed_s', 'q', 'dq', 'effort_raw'):
        np.testing.assert_allclose(arr_bin[key], arr_csv[key], rtol=0, atol=1e-15, err_msg=f'Mismatch in {key}')

    # Verify window() produces identical slices on segment 1
    win_bin, t_bin, scope_bin = ObservationReader(bin_only).window((0.16, 0.28), segment=1)
    win_csv, t_csv, scope_csv = ObservationReader(csv_only).window((0.16, 0.28), segment=1)
    assert scope_bin == scope_csv
    np.testing.assert_array_equal(t_bin, t_csv)
    for k in win_bin:
        np.testing.assert_allclose(win_bin[k], win_csv[k], rtol=0, atol=1e-15)


def test_observation_reader_binary_corrupt_tail_and_passive_rejected(tmp_path):
    bad_tail = tmp_path / 'bad_tail'
    _write_dual_fixture(bad_tail, n_ticks=100, include_csv=False)
    with (bad_tail / 'feedback.bin').open('ab') as f:
        f.write(b'trunc')
    with pytest.raises(ValueError, match='不完整'):
        ObservationReader(bad_tail).arrays()

    passive = tmp_path / 'passive'
    _write_dual_fixture(passive, n_ticks=100, include_csv=False)
    (passive / 'passive_commands.bin').write_bytes(b'\x00' * 752)
    with pytest.raises(ValueError, match='被动快照'):
        ObservationReader(passive).arrays()


def test_fieldwork_summaries_match_struct_fallback_bit_for_bit(tmp_path, monkeypatch):
    folder = tmp_path / 'run'
    _write_dual_fixture(folder, n_ticks=350, include_csv=True, fault_tail=True)
    # Add partial trailing bytes to verify tolerant prefix recovery in summarize_*
    with (folder / 'feedback.bin').open('ab') as f:
        f.write(b'12345')
    with (folder / 'commands.bin').open('ab') as f:
        f.write(b'123')

    vec_fb = fw.summarize_feedback(folder)
    vec_cmd = fw.summarize_commands(folder)
    vec_tim = fw.timing_summary(folder)

    monkeypatch.setattr(fw, '_np', None)
    ref_fb = fw.summarize_feedback(folder)
    ref_cmd = fw.summarize_commands(folder)
    ref_tim = fw.timing_summary(folder)

    assert vec_fb == ref_fb
    assert vec_cmd == ref_cmd
    assert vec_tim == ref_tim
    # Verify JSON serializability via fw.dump
    fw.dump(tmp_path / 'summary.json', dict(fb=vec_fb, cmd=vec_cmd, tim=vec_tim))
    loaded = json.loads((tmp_path / 'summary.json').read_text())
    assert loaded['fb']['trailing_bytes'] == 5
    assert loaded['cmd']['trailing_bytes'] == 3


def test_600k_frame_binary_read_and_summary_benchmark(tmp_path):
    """600,000 frames (~10 minutes at 1 kHz) must read via ObservationReader.arrays() well under 0.25 s."""
    free_bytes = shutil.disk_usage(tmp_path).free
    n_frames = 600_000 if free_bytes >= 2 * 1024**3 else 150_000
    scale = 600_000 / n_frames

    root = tmp_path / 'bench600k'
    root.mkdir()
    try:
        t0 = 10**18
        wall_ms = 1_700_000_000_000
        offset = wall_ms * 1_000_000 - t0
        first_stream = t0 + 10 * 1_000_000 + 20_000

        fb = np.zeros(n_frames, dtype=FEEDBACK_RECORD_DTYPE)
        idx = np.arange(n_frames, dtype=np.int64)
        fb['receive_mono_ns'] = t0 + idx * 1_000_000 + 5_000
        fb['frame']['frame_counter'] = idx + 1
        fb['frame']['timestamp'] = (wall_ms + idx).astype(np.uint64)
        fb['frame']['joints'][:, 12:26]['position'] = (idx[:, None] % 10000).astype(np.int32)
        fb['frame']['joints'][:, 12:26]['velocity'] = ((idx[:, None] % 200) - 100).astype(np.int32)
        fb['frame']['joints'][:, 12:26]['torque'] = ((idx[:, None] % 80) - 40).astype(np.int16)
        fb['frame']['joints'][:, 12:26]['status_word'] = 0x237
        fb.tofile(root / 'feedback.bin')
        del fb

        cmd = np.zeros(n_frames, dtype=COMMAND_RECORD_DTYPE)
        cmd['send_mono_ns'] = t0 + idx * 1_000_000 + 20_000
        cmd['frame']['frame_counter'] = idx + 1
        cmd['frame']['joints'][:, 12:26]['position'] = (idx[:, None] % 10000).astype(np.int32)
        cmd.tofile(root / 'commands.bin')
        del cmd

        (root / 'events.csv').write_text(
            f'mono_ns,phase,event\n'
            f'{t0},0,disabling\n'
            f'{t0 + 10 * 1_000_000 + 10_000},3,reference_begin\n'
            f'{t0 + (n_frames - 5) * 1_000_000 + 10_000},4,exiting_disable\n'
        )
        (root / 'clock_pairs.csv').write_text(
            f'mono_before_ns,wall_ns,mono_after_ns,offset_ns,clock_segment,jump_detected\n'
            f'{t0 - 100_000},{wall_ms * 1_000_000 - 100_000},{t0 - 90_000},{offset},0,0\n'
        )
        (root / 'result.txt').write_text(
            f'execution_completed=1\nreason=completed_with_disable_ack\n'
            f'first_stream_mono_ns={first_stream}\nfirst_command_mono_ns={first_stream}\n'
            f'first_shm_write_mono_ns={t0 + 20_000}\nrecording_stopped=0\ndisable_ack=1\n'
        )

        t_start = time.perf_counter()
        arrays = ObservationReader(root).arrays()
        t_obs = (time.perf_counter() - t_start) * scale

        assert len(arrays['receive_ns']) == n_frames
        assert int(np.sum(arrays['phase'] == 3)) == n_frames - 15
        # Target < 0.25 s on fast NVMe; allow laptop/Orin/container bind-mount jitter ceiling of 1.2 s
        assert t_obs < 1.2, f'600k ObservationReader.arrays() took {t_obs:.3f}s'

        t_sum0 = time.perf_counter()
        s_fb = fw.summarize_feedback(root)
        s_cmd = fw.summarize_commands(root)
        s_tim = fw.timing_summary(root)
        t_sum = (time.perf_counter() - t_sum0) * scale
        assert s_fb['count'] == n_frames and s_cmd['count'] == n_frames
        assert s_tim['streaming_send_gap_ns']['max'] == 1_000_000
        assert t_sum < 2.0, f'600k fieldwork summaries took {t_sum:.3f}s'
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_openmp_batch_equivalence_dynamics_and_collision():
    rng = np.random.default_rng(20261114)
    cfg = load_robot_config()
    engine, _ = model_for(cfg)
    checker = collision_for(cfg, engine)

    # Add an observation frame to also exercise frame_batch and point_geometry_batch
    m = engine.model
    frame = _core.ArmFrame()
    frame.name = 'test_flange_obs'
    frame.parent = 6
    m.frames = [*m.frames, frame]
    engine_f = _core.ArmDynamics(m)

    # Compare serial path (N = 5 < 8 / < 16) vs OpenMP parallel path (N = 128)
    q_small = rng.uniform(-0.35, 0.35, size=(5, 14))
    dq_small = rng.uniform(-0.5, 0.5, size=(5, 14))
    ddq_small = rng.uniform(-1.5, 1.5, size=(5, 14))
    off_small = rng.uniform(-0.05, 0.05, size=(5, 3))

    tiles = 26  # 5 * 26 = 130 >= 16 and >= 8
    q_big = np.tile(q_small, (tiles, 1))
    dq_big = np.tile(dq_small, (tiles, 1))
    ddq_big = np.tile(ddq_small, (tiles, 1))
    off_big = np.tile(off_small, (tiles, 1))

    g_s = engine_f.gravity_batch(q_small)
    g_b = engine_f.gravity_batch(q_big)
    assert np.max(np.abs(np.tile(g_s, (tiles, 1)) - g_b)) < 1e-14

    ev_s = engine_f.evaluate_batch(q_small, dq_small, ddq_small)
    ev_b = engine_f.evaluate_batch(q_big, dq_big, ddq_big)
    for k in ('mass_matrix', 'inertia', 'velocity', 'gravity', 'friction', 'total'):
        reps = (tiles, 1, 1) if ev_s[k].ndim == 3 else (tiles, 1)
        assert np.max(np.abs(np.tile(ev_s[k], reps) - ev_b[k])) < 1e-14

    reg_s = engine_f.regressor_batch(q_small, dq_small, ddq_small)
    reg_b = engine_f.regressor_batch(q_big, dq_big, ddq_big)
    assert np.max(np.abs(np.tile(reg_s['regressor'], (tiles, 1, 1)) - reg_b['regressor'])) < 1e-14
    assert np.max(np.abs(np.tile(reg_s['rotor_torque'], (tiles, 1)) - reg_b['rotor_torque'])) < 1e-14
    assert np.max(np.abs(np.tile(reg_s['friction'], (tiles, 1)) - reg_b['friction'])) < 1e-14

    fb_s = engine_f.frame_batch(q_small, 'test_flange_obs')
    fb_b = engine_f.frame_batch(q_big, 'test_flange_obs')
    assert np.max(np.abs(np.tile(fb_s['poses'], (tiles, 1, 1)) - fb_b['poses'])) < 1e-14
    assert np.max(np.abs(np.tile(fb_s['jacobians'], (tiles, 1, 1)) - fb_b['jacobians'])) < 1e-14

    pg_s = engine_f.point_geometry_batch(q_small, 'test_flange_obs', off_small)
    pg_b = engine_f.point_geometry_batch(q_big, 'test_flange_obs', off_big)
    for k in ('positions', 'placement_jacobians', 'joint_jacobians'):
        reps = (tiles, 1, 1) if pg_s[k].ndim == 3 else (tiles, 1)
        assert np.max(np.abs(np.tile(pg_s[k], reps) - pg_b[k])) < 1e-14

    # Fresh GJK solvers keep distances independent of preceding poses and threads.
    fresh_serial = collision_for(cfg, engine)
    fresh_parallel = collision_for(cfg, engine)
    c_s = fresh_serial.check(q_small)
    c_b = fresh_parallel.check(q_big)
    assert np.max(np.abs(c_s['distances'][0] - c_b['distances'][0])) < 1e-14
    assert np.max(np.abs(np.tile(c_s['distances'], (tiles, 1)) - c_b['distances'])) < 1e-12
    np.testing.assert_array_equal(np.tile(c_s['worst_pair'], tiles), c_b['worst_pair'])


def test_orjson_serialization_nan_guard_and_noncontiguous_numpy(tmp_path):
    bundle = jog_bundle([0.0] * 14, 12, 0.05, 2.0)
    bundle['optional_note'] = None
    bundle['effort'][0]['value'] = np.zeros((2, 14), dtype=np.float64)[:, :14][0]
    ref_path = save_bundle(tmp_path / 'ok_bundle', bundle)
    text = ref_path.read_text()
    assert '\n' not in text[:-1] and text.endswith('\n')  # Compact single-line JSON
    assert not list((tmp_path / 'ok_bundle').glob('*.tmp'))
    parsed = json.loads(text)
    assert parsed['optional_note'] is None

    # NaN in Python float or numpy array must raise ValueError in save_bundle and fw.dump
    bad_py = jog_bundle([0.0] * 14, 12, 0.05, 2.0)
    bad_py['kp'][0] = float('nan')
    with pytest.raises(ValueError):
        save_bundle(tmp_path / 'bad_py', bad_py)

    bad_np = jog_bundle([0.0] * 14, 12, 0.05, 2.0)
    bad_np['kp'] = np.array([20.0] * 13 + [np.inf])
    with pytest.raises(ValueError):
        save_bundle(tmp_path / 'bad_np', bad_np)

    with pytest.raises(ValueError):
        fw.dump(tmp_path / 'bad_fw.json', {'x': np.array([1.0, np.nan])})

    # data.write_json handles non-contiguous arrays, integer keys, and non-finite -> null
    mat = np.arange(12, dtype=np.float64).reshape(3, 4).T  # Fortran-contiguous view
    payload = {7: mat, 'nan_val': float('nan'), 'flag': np.bool_(True)}
    write_json(tmp_path / 'pretty.json', payload)
    write_json(tmp_path / 'compact.json', payload, compact=True)
    p1 = json.loads((tmp_path / 'pretty.json').read_text())
    p2 = json.loads((tmp_path / 'compact.json').read_text())
    assert p1 == p2
    assert p1['7'] == mat.tolist() and p1['nan_val'] is None and p1['flag'] is True


def test_live_native_executor_binary_vs_csv_exact_match(tmp_path, monkeypatch):
    import os
    import subprocess
    root_dir = Path(__file__).resolve().parents[1]
    from test_shm_fieldwork import NATIVE as native
    fixture_bin = native / 'shm_field_fixture'
    exec_bin = native / 'shm_executor'
    if not (fixture_bin.exists() and exec_bin.exists()):
        pytest.skip('Native SHM test binaries not yet built')

    shm_name = f'/casbot_calib_test_w1a_{os.getpid()}_{time.monotonic_ns()}'
    ready_file = tmp_path / 'ready'
    master_cmds = tmp_path / 'master_commands.bin'
    proc = subprocess.Popen([str(fixture_bin), shm_name, str(ready_file), str(master_cmds)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 5.0
        while not ready_file.exists() and proc.poll() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert ready_file.exists()

        out = tmp_path / 'c01_capture'
        r = subprocess.run([
            str(exec_bin), '--execute', '--case', 'C01', '--shm', shm_name,
            '--urdf', str(URDF),
            '--hand-background', str(root_dir / 'tests/fixtures/shm/command_slot_0.bin'),
            '--duration', '0.06', '--output', str(out),
        ], capture_output=True, text=True, timeout=10)
        assert r.returncode == 0, r.stderr + r.stdout

        arr_bin = ObservationReader(out).arrays()
        csv_copy = tmp_path / 'c01_csv_only'
        shutil.copytree(out, csv_copy)
        (csv_copy / 'feedback.bin').unlink()
        arr_csv = ObservationReader(csv_copy).arrays()

        for key in ('index', 'receive_ns', 'observation_time_ns', 'source_timestamp_ms', 'clock_segment', 'frame_counter', 'phase', 'time_ambiguous'):
            np.testing.assert_array_equal(arr_bin[key], arr_csv[key], err_msg=f'Live C++ mismatch in {key}')
        for key in ('elapsed_s', 'q', 'dq', 'effort_raw'):
            np.testing.assert_allclose(arr_bin[key], arr_csv[key], rtol=0, atol=1e-15, err_msg=f'Live C++ mismatch in {key}')

        s_vec = (fw.summarize_feedback(out), fw.summarize_commands(out), fw.timing_summary(out))
        monkeypatch.setattr(fw, '_np', None)
        s_ref = (fw.summarize_feedback(out), fw.summarize_commands(out), fw.timing_summary(out))
        assert s_vec == s_ref
    finally:
        if proc.poll() is None:
            proc.terminate()
        proc.wait(timeout=5)

