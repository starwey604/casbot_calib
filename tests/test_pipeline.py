"""End-to-end tests for calib/dynamics*.sh, Session/--resume, and automatic boot takeover."""
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import pytest
import yaml
from casbot_calib.pipeline.runner import build_parser, run_pipeline
from casbot_calib.pipeline.takeover import CHANNEL_BYTES, QUEUE_MAGIC, STATUS_OFFSET, ensure_ready_channel
from casbot_calib.shm_data import ROOT
from casbot_calib.shm_takeover import SERVICES

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='POSIX SHM only')
NATIVE = Path(os.environ.get('CASBOT_SHM_TEST_BIN', ROOT / 'build/humble/cpp/src/shm')) / 'shm_executor'


def test_operator_help_hides_dev_and_mock_flags():
    help_text = build_parser().format_help()
    for visible in ('--stages', '--resume', '--plan-only', '--robot', '--config'):
        assert visible in help_text
    for hidden in ('--mock', '--endpoint', '--executor', '--hand-background', '--output-root'):
        assert hidden not in help_text


def test_invalid_stage_or_mode_fails_fast_before_touching_shm(tmp_path):
    res_smoke = subprocess.run(
        ['bash', str(ROOT / 'calib/dynamics_h2.sh'), '--mode', 'smoke', '--output-root', str(tmp_path / 'out_smoke')],
        capture_output=True, text=True,
    )
    assert res_smoke.returncode == 2
    assert '仅适用于 H1' in res_smoke.stderr
    assert not (tmp_path / 'out_smoke').exists()

    res_unknown = subprocess.run(
        ['bash', str(ROOT / 'calib/dynamics.sh'), '--stages', 'h1,h9', '--output-root', str(tmp_path / 'out_unknown')],
        capture_output=True, text=True,
    )
    assert res_unknown.returncode == 2
    assert '未知标定阶段' in res_unknown.stderr
    assert not (tmp_path / 'out_unknown').exists()


def test_plan_only_runs_offline_and_persists_robot_id(tmp_path):
    out_root = tmp_path / 'calib_out'
    res = subprocess.run(
        ['bash', str(ROOT / 'calib/dynamics_h1.sh'), '--plan-only', '--robot', 'SN_TEST_01', '--output-root', str(out_root)],
        capture_output=True, text=True,
    )
    assert res.returncode == 0, res.stderr + res.stdout
    assert (out_root / 'state/robot_id.txt').read_text().strip() == 'SN_TEST_01'
    latest = (out_root / 'latest').resolve()
    session = json.loads((latest / 'session.json').read_text())
    assert session['robot_id'] == 'SN_TEST_01' and session['plan_only']
    assert session['stages']['h1']['status'] == 'planned'
    assert (latest / 'h1/plan/reference.json').is_file()
    assert (latest / 'h1/report.md').is_file()


@pytest.mark.skipif(not NATIVE.exists(), reason='Build SHM physics executables')
def test_dynamics_h1_shell_end_to_end_and_resume(tmp_path):
    fast_robot = tmp_path / 'robot.yaml'
    fast_robot.write_text(yaml.safe_dump(dict(initial_hold_s=0.05, final_hold_s=0.05, anchor_transition_s=0.1)))
    fast_h1 = tmp_path / 'h1.yaml'
    fast_h1.write_text(yaml.safe_dump(dict(
        mode='small',
        ready_pose=True,
        small=dict(
            joints=['left_shoulder_pitch_joint'],
            target_offset_deg=0.1,
            approach_offset_deg=0.03,
            timing=dict(initial_hold_s=0.05, minimum_transition_s=0.1, minimum_paired_approach_s=0.1, settle_s=0.05, measure_s=0.1, final_hold_s=0.05),
            planning_envelope=dict(speed_rad_s=0.05, acceleration_rad_s2=0.1, jerk_rad_s3=1.0),
        ),
    )))
    out_root = tmp_path / 'runs'
    res = subprocess.run(
        [
            'bash', str(ROOT / 'calib/dynamics_h1.sh'),
            '--robot', 'ORIN_01',
            '--config', str(fast_h1),
            '--robot-config', str(fast_robot),
            '--output-root', str(out_root),
            '--executor', str(NATIVE),
            '--mock',
        ],
        capture_output=True, text=True,
    )
    assert res.returncode == 0, res.stderr + res.stdout
    latest = (out_root / 'latest').resolve()
    session = json.loads((latest / 'session.json').read_text())
    assert session['robot_id'] == 'ORIN_01'
    h1_rec = session['stages']['h1']
    assert h1_rec['status'] == 'completed' and h1_rec['fit_succeeded']
    assert (latest / h1_rec['candidate_urdf']).is_file()
    assert (latest / h1_rec['report_md']).is_file()
    assert '留出集整体力矩 RMSE' in (latest / h1_rec['report_md']).read_text()

    # Resuming an already completed h1 stage skips re-execution cleanly.
    res_resume = run_pipeline(
        stages='h1',
        resume=True,
        output_root=out_root,
        robot_config=fast_robot,
        stage_config=fast_h1,
        executor=NATIVE,
        mock=True,
    )
    assert res_resume['completed'] and res_resume['skipped_all']


def test_auto_takeover_captures_background_handshakes_and_skips_same_boot(tmp_path):
    dev_shm = tmp_path / 'dev_shm'
    dev_shm.mkdir()
    proc = tmp_path / 'proc'
    (proc / 'sys/kernel/random').mkdir(parents=True)
    (proc / 'sys/kernel/random/boot_id').write_text('boot-abc-123\n')

    def write_fake_shm(cmd_pub, status_pub):
        shm_file = dev_shm / 'shm_mc'
        with shm_file.open('wb') as f:
            f.truncate(CHANNEL_BYTES)
            f.seek(0)
            f.write(struct.pack('<iiqqqq', QUEUE_MAGIC, 2, 0, cmd_pub, cmd_pub, cmd_pub))
            f.seek(STATUS_OFFSET)
            f.write(struct.pack('<iiqqqq', QUEUE_MAGIC, 2, 0, status_pub, status_pub, status_pub))
        for suffix in ('_front', '_back', '_start'):
            (dev_shm / f'sem.shm_mc{suffix}').touch()

    # Initially vendor stack is up and has published command frames.
    write_fake_shm(cmd_pub=10, status_pub=10)

    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    fake_exec = bin_dir / 'shm_executor'
    fake_exec.write_text(
        '#!/usr/bin/env python3\n'
        'import pathlib, struct, sys\n'
        'out = pathlib.Path(sys.argv[sys.argv.index("--output") + 1])\n'
        'out.mkdir(parents=True, exist_ok=True)\n'
        'buf = bytearray(744); struct.pack_into("<H", buf, 10, 2); buf[642] = 0x5A\n'
        '(out / "command.bin").write_bytes(buf)\n'
    )
    fake_exec.chmod(0o755)

    shm_file_path = dev_shm / 'shm_mc'
    fake_start = bin_dir / 'shm_channel_start'
    fake_start.write_text(
        f'#!/usr/bin/env python3\n'
        f'import pathlib, struct\n'
        f'p = pathlib.Path({str(shm_file_path)!r})\n'
        f'with p.open("r+b") as f:\n'
        f'    f.seek({STATUS_OFFSET})\n'
        f'    f.write(struct.pack("<iiqqqq", {QUEUE_MAGIC}, 2, 0, 1, 0, 1))\n'
    )
    fake_start.chmod(0o755)

    class FakeOrinHost:
        def __init__(self):
            self.proc = proc
            self.vendor = tmp_path / 'vendor'
            self.vendor.mkdir()
            for name in ('launcher.bash', 'run_ethercat_station.bash'):
                (self.vendor / name).touch()
            self.svc = {u: 'active' for u in SERVICES}
            self.procs = [dict(pid=100, name='ethercat_station', start_ticks='1', threads=[dict(tid=100, state='S')])]
            self.takeovers = 0

        def services(self):
            return dict(self.svc)

        def processes(self):
            return list(self.procs)

        def terminate_master(self, _p):
            self.procs = []

        def diagnostics(self, root):
            (root / 'diagnosis.json').write_text('{}')

        def start(self, _root):
            self.takeovers += 1
            self.procs = [dict(pid=200, name='ethercat_station', start_ticks='2', threads=[dict(tid=200, state='S')])]
            write_fake_shm(cmd_pub=0, status_pub=0)

    host = FakeOrinHost()
    import casbot_calib.shm_takeover as st
    orig_runner = st.run_logged

    def fake_runner(cmd, log_path, **_kw):
        Path(log_path).write_text('ok\n')
        if cmd[-1] == 'stop':
            host.svc = {u: 'inactive' for u in SERVICES}
        return 0

    st.run_logged = fake_runner
    # Rebind prepare default runner through monkeypatch-safe wrapper:
    import casbot_calib.pipeline.takeover as pt
    orig_prepare = pt.prepare_takeover
    pt.prepare_takeover = lambda root, host=host: st.prepare(root, host=host, runner=fake_runner)
    try:
        state_root = tmp_path / 'state'
        first = ensure_ready_channel(state_root, fake_exec, host=host, dev_shm=dev_shm, search_roots=())
        assert first['takeover_performed'] and first['handshake_performed']
        assert first['hand_background'].read_bytes()[642] == 0x5A
        assert host.takeovers == 1

        # Second call in same boot skips takeover & handshake immediately!
        second = ensure_ready_channel(state_root, fake_exec, host=host, dev_shm=dev_shm, search_roots=())
        assert not second['takeover_performed'] and not second['handshake_performed']
        assert second['hand_background'] == first['hand_background']
        assert host.takeovers == 1
    finally:
        st.run_logged = orig_runner
        pt.prepare_takeover = orig_prepare
