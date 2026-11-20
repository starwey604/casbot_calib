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

from test_shm_fieldwork import NATIVE as _NATIVE_DIR

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='POSIX SHM only')
NATIVE = _NATIVE_DIR / 'shm_executor'


def test_operator_help_hides_dev_and_mock_flags():
    help_text = build_parser().format_help()
    for visible in ('--stages', '--resume', '--plan-only', '--robot', '--config'):
        assert visible in help_text
    for hidden in ('--mock', '--endpoint', '--executor', '--hand-background', '--output-root'):
        assert hidden not in help_text


@pytest.mark.slow
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


@pytest.mark.slow
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


@pytest.mark.slow
def test_dynamics_h1_resume_skips_completed_session(tmp_path):
    from casbot_calib.pipeline.session import Session
    from casbot_calib.pipeline.robot import load_robot_config
    config = load_robot_config()
    session = Session.create(output_root=tmp_path, robot_id='ORIN_01',
                             model_context=config['model_context'], stages_requested=['h1'])
    session.mark_stage('h1', 'completed', fit_succeeded=True)
    result = run_pipeline(stages='h1', resume=True, output_root=tmp_path)
    assert result['completed'] and result['skipped_all']
    assert result['session'].root == session.root


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


def test_body_hold_complete_slots_and_motion_offsets():
    import numpy as np
    from casbot_calib.model_runtime import model_for
    from casbot_calib.pipeline.robot import load_robot_config
    from casbot_calib.shm_ready import ready_position
    from casbot_calib.staged_motion import Motion
    from casbot_calib.stages.dynamics.common import compose_stage_reference

    cfg = load_robot_config(hold_body=True)
    engine, _ = model_for(cfg)
    anchor = ready_position()
    motion = Motion('test', 'h2', anchor)
    motion.add(.5, anchor, label='initial_hold')
    target = anchor.copy()
    target[0] += .03
    motion.add(2., target)
    start = anchor.copy()
    start[0] -= .02
    for ready in (False, True):
        bundle, joined, _, _, offset, _ = compose_stage_reference(
            motion, start if ready else anchor, engine, cfg, ready_pose=ready,
            markers=[dict(t_ns=500_000_000, event='sweep')])
        assert len(bundle['non_arm_hold']['slots']) == 15 and bundle['schema'].endswith('_v2')
        np.testing.assert_allclose(joined.at([0, .5, 1.])[0], [joined.home]*3, atol=1e-12)
        if not ready:
            assert offset == 500_000_000
        assert dict((m['event'], m['t_ns']) for m in bundle['markers'])['sweep'] == offset + 500_000_000
    del cfg['non_arm_hold']['slots']['11']
    with pytest.raises(ValueError, match='15'):
        load_robot_config(cfg)


@pytest.mark.slow
def test_hold_body_h1_cli_and_offline_reanalysis(tmp_path):
    output = tmp_path / 'sessions'
    base = ['bash', str(ROOT / 'calib/dynamics_h1.sh'), '--mode', 'smoke', '--output-root', str(output)]
    run = subprocess.run([*base, '--mock', '--hold-body', '--executor', str(NATIVE), '--robot', 'BODY_TEST'],
                         text=True, capture_output=True, timeout=120)
    assert run.returncode == 0, run.stderr + run.stdout
    stage = (output / 'latest').resolve() / 'h1'
    bundle = json.loads((stage / 'plan/reference.json').read_text())
    assert bundle['schema'].endswith('_v2') and len(bundle['non_arm_hold']['slots']) == 15
    again = subprocess.run([*base, '--reanalyse'], text=True, capture_output=True, timeout=60)
    assert again.returncode == 0, again.stderr + again.stdout
    assert list(stage.glob('analysis.previous_*'))


@pytest.mark.slow
def test_body_hold_stage_windows_without_ready_pose(tmp_path):
    recipes = {stage: yaml.safe_load((ROOT / f'configs/dynamics/{stage}.yaml').read_text())
               for stage in ('h2', 'h3')}
    for recipe in recipes.values():
        recipe['ready_pose'] = False
    plans = []
    for enabled in (False, True):
        result = run_pipeline(stages='h2,h3', stage_config=recipes, plan_only=True, hold_body=enabled,
                              output_root=tmp_path / str(enabled))
        plans.append(result['session'].root)
    for stage in recipes:
        plain, body = [root / stage / 'plan' for root in plans]
        original, shifted = [json.loads((root / 'reference.json').read_text()) for root in (plain, body)]
        assert original['schema'] == 'casbot_shm_reference_v1' and 'non_arm_hold' not in original
        assert shifted['schema'] == 'casbot_shm_reference_v2' and len(shifted['non_arm_hold']['slots']) == 15
        original_meta, shifted_meta = [json.loads((root / 'plan.json').read_text()) for root in (plain, body)]
        offset = shifted_meta['calibration_begin_ns']
        assert offset == 500_000_000
        assert shifted['markers'] == [dict(row, t_ns=row['t_ns'] + offset) for row in original['markers']]
        def windows(meta):
            if stage == 'h2':
                return [t for s in meta['sweeps'] for p in s['passes']
                        for key in ('ramp_ns', 'plateau_ns', 'brake_ns') for t in p[key]]
            return [t for b in meta['blocks'] for key in ('periodic_ns', 'steady_ns') for t in b[key]]
        assert windows(shifted_meta) == [t + offset for t in windows(original_meta)]
