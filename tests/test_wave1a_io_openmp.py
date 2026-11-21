"""Wave 1A tests: wire35 NumPy dtypes, compressed recording I/O, orjson safety, and OpenMP equivalence."""
import csv
import json
import subprocess
import sys
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
from recording_fixtures import write_recording


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


def _synthetic_recording(root, ticks, *, rich=False):
    root.mkdir()
    origin = 10**18 + 500_000_000
    wall_ms = 1_700_000_000_000
    offset = wall_ms*1_000_000-origin
    commands = (dict(send_mono_ns=origin+k*1_000_000, phase=3 if k < ticks-5 else 4,
                     frame_counter=k+1) for k in range(ticks))
    write_recording(root/'commands.csv.zst', commands, feedback=False)

    def feedback():
        for k in range(ticks):
            raw = k%10000
            row = dict(receive_mono_ns=origin+k*1_000_000+5000, phase=3 if k < ticks-5 else 4,
                       frame_counter=k+1, source_timestamp_ms=wall_ms+k,
                       observation_time_ns=origin+k*1_000_000, pos_raw12=raw, q0=raw*1e-6+OFFSET[0])
            if rich:
                row.update(status12=0x237, mode12=-1, error12=42 if k == ticks-1 else 0,
                           hand0_global_status=3, hand0_pos0=123+k%7, hand0_status0=1)
            yield row

    write_recording(root/'feedback.csv.zst', feedback())
    (root/'clock_pairs.csv').write_text(
        'mono_before_ns,wall_ns,mono_after_ns,offset_ns,clock_segment,jump_detected\n'
        f'{origin-1000},{wall_ms*1_000_000-1000},{origin-1000},{offset},0,0\n'
    )
    (root/'result.txt').write_text(
        f'execution_completed=1\nfirst_command_mono_ns={origin}\nfirst_shm_write_mono_ns={origin}\n'
        'recording_stopped=0\ndisable_ack=1\n'
    )
    return origin


def test_observation_reader_uses_native_time_phase_and_registers(tmp_path):
    root = tmp_path/'capture'
    origin = _synthetic_recording(root, 50, rich=True)
    data = ObservationReader(root).arrays()
    np.testing.assert_array_equal(data['observation_time_ns'], origin+np.arange(50)*1_000_000)
    np.testing.assert_array_equal(data['phase'], [3]*45+[4]*5)
    np.testing.assert_array_equal(data['q'][:, 0], np.arange(50)*1e-6+OFFSET[0])
    assert data['frame_counter'].dtype == np.uint64
    assert not data['time_ambiguous'].any()
    window, elapsed, metadata = ObservationReader(root).window((.01, .04))
    assert len(window['q']) == 32 and metadata['clock_segment'] == 0
    assert elapsed[0] == 0 and elapsed[1] == .01


@pytest.mark.parametrize('passive_source', ['feedback', 'commands', 'events'])
def test_observation_reader_rejects_passive_capture(tmp_path, passive_source):
    root = tmp_path/'capture'
    origin = _synthetic_recording(root, 30)
    if passive_source == 'feedback':
        write_recording(root/'feedback.csv.zst', [dict(receive_mono_ns=origin, passive=1)])
    elif passive_source == 'commands':
        write_recording(root/'passive_commands.csv.zst', [dict(send_mono_ns=origin)], feedback=False)
    else:
        (root/'events.csv').write_text(f'mono_ns,phase,event\n{origin},-1,passive_observe_begin\n')
    with pytest.raises(ValueError, match='被动快照'):
        ObservationReader(root).arrays()


def test_fieldwork_summary_preserves_complete_prefix_after_incomplete_zstd_frame(tmp_path):
    root = tmp_path/'run'
    _synthetic_recording(root, 50, rich=True)
    for name in ('feedback', 'commands'):
        path = root/f'{name}.csv.zst'
        size = path.stat().st_size
        _core._write_csv_zst(path, 'interrupted page\n', append=True)
        with path.open('rb+') as stream:
            stream.truncate(size+9)
    fb = fw.summarize_feedback(root)
    cmd = fw.summarize_commands(root)
    assert fb['count'] == cmd['count'] == 50
    assert fb['truncated_tail'] and cmd['truncated_tail']
    assert fb['joints'][12]['errors'] == [42]
    assert fb['hands'][0]['items'][0]['position'] == dict(first=123, last=123, min=123, max=129)
    with pytest.raises(ValueError, match='命令记录尾部不完整'):
        ObservationReader(root).arrays()
    fw.dump(tmp_path/'summary.json', dict(fb=fb, cmd=cmd, timing=fw.timing_summary(root)))


@pytest.mark.slow
@pytest.mark.skipif(sys.platform != 'linux', reason='Linux process peak RSS')
def test_large_csv_zstd_recording_read_and_summary(tmp_path, record_property):
    """Measure whole-table and bounded video reads in separate processes."""
    ticks = 600_000
    root = tmp_path/'recording'
    _synthetic_recording(root, ticks)
    reader = """
import json, sys, time
from pathlib import Path
from casbot_calib import _core
from casbot_calib.shm_observations import ObservationReader
root = Path(sys.argv[1])
start = time.perf_counter()
if sys.argv[2] == 'video':
    rows = sum(len(times) for times, _ in _core.iter_feedback_chunks_zst(root/'feedback.csv.zst'))
else:
    observations = ObservationReader(root).arrays()
    rows = len(observations['receive_ns'])
    assert (observations['phase'] == 3).sum() == rows-5
seconds = time.perf_counter()-start
status = Path('/proc/self/status').read_text().splitlines()
peak_rss_kib = int(next(line.split()[1] for line in status if line.startswith('VmHWM:')))
print(json.dumps(dict(rows=rows, seconds=seconds, peak_rss_kib=peak_rss_kib)))
"""
    for route in ('observations', 'video'):
        metrics = json.loads(subprocess.check_output([sys.executable, '-c', reader, str(root), route]))
        assert metrics['rows'] == ticks
        print(f'Recording {route}: {metrics}')
        for name in ('seconds', 'peak_rss_kib'):
            record_property(f'{route}_{name}', metrics[name])
    assert fw.summarize_feedback(root)['count'] == ticks
    assert fw.summarize_commands(root)['count'] == ticks
    assert fw.timing_summary(root)['streaming_send_gap_ns']


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


@pytest.mark.slow
def test_live_native_executor_zstdcat_matches_arrays(tmp_path):
    import os
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

        path = out/'feedback.csv.zst'
        subprocess.run(['zstd', '-t', path], check=True, capture_output=True)
        table = subprocess.check_output(['zstdcat', path], text=True)
        rows = list(csv.DictReader(table.splitlines()))
        data = ObservationReader(out).arrays()
        records = _core.read_feedback_csv_zst(path)
        assert len(rows) == len(data['receive_ns']) and not records['truncated_tail']
        for column in ('observation_time_ns', 'source_timestamp_ms', 'clock_segment', 'frame_counter', 'phase'):
            np.testing.assert_array_equal(data[column], [int(row[column]) for row in rows])
        for column, array in (('q', 'q'), ('v', 'dq'), ('effort', 'effort_raw')):
            expected = [[float(row[f'{column}{j}']) for j in range(14)] for row in rows]
            np.testing.assert_array_equal(data[array], expected)
        for column, array in (('pos_raw', 'pos_raw'), ('vel_raw', 'vel_raw'), ('trq_raw', 'trq_raw'),
                              ('status', 'status'), ('mode', 'mode'), ('error', 'error')):
            expected = [[int(row[f'{column}{j}']) for j in range(35)] for row in rows]
            np.testing.assert_array_equal(records[array], expected)
        assert not (out/'feedback.bin').exists() and not (out/'commands.bin').exists()
        assert not (out/'feedback.csv').exists() and not (out/'commands.csv').exists()
        assert _core.last_feedback_csv_zst(path) == rows[-1]

        # The native writer closes pages as the run progresses. An archive can
        # truncate a partial last frame and preserve the preceding pages verbatim.
        truncated = tmp_path/'live.csv.zst'
        with path.open('rb') as stream:
            truncated.write_bytes(stream.read(path.stat().st_size-3))
        snapshot = tmp_path/'snapshot.csv.zst'
        copied = _core.copy_complete_csv_zst_prefix(truncated, snapshot)
        assert 0 < copied < len(rows)
        assert _core.read_feedback_csv_zst(truncated)['truncated_tail']
        prefix = subprocess.check_output(['zstdcat', snapshot], text=True)
        assert list(csv.DictReader(prefix.splitlines())) == rows[:copied]
        assert _core.last_feedback_csv_zst(truncated) == rows[copied-1]
    finally:
        if proc.poll() is None:
            proc.terminate()
        proc.wait(timeout=5)

