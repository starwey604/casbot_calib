"""Fieldwork orchestration uses private native SHM fixtures only, never shm_mc."""
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import pytest
from casbot_calib import _core

from casbot_calib import shm_fieldwork as fw

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='POSIX SHM fieldwork requires Linux')

ROOT = Path(__file__).resolve().parents[1]


def _default_native_dir() -> Path:
    if 'CASBOT_SHM_TEST_BIN' in os.environ:
        return Path(os.environ['CASBOT_SHM_TEST_BIN'])
    build_cands = [
        cand for cand in (
            ROOT / 'build/shm-python/cpp/src/shm',
            ROOT / 'build/shm-core/cpp/src/shm',
            ROOT / 'build/humble/cpp/src/shm',
        )
        if (cand / 'shm_executor').exists()
    ]
    if build_cands:
        # Prefer directories with test fixtures and the most recently compiled shm_executor.
        return max(
            build_cands,
            key=lambda p: ((p / 'shm_field_fixture').exists(), (p / 'shm_executor').stat().st_mtime_ns),
        )
    if (ROOT / 'install/shm-core/bin/shm_executor').exists():
        return ROOT / 'install/shm-core/bin'
    return ROOT / 'build/shm-python/cpp/src/shm'


NATIVE = _default_native_dir()


def cli(*args, timeout=15):
    return subprocess.run([sys.executable, '-m', 'casbot_calib.shm_fieldwork', *map(str, args)],
                          text=True, capture_output=True, timeout=timeout)


@pytest.fixture
def master(tmp_path):
    executable = NATIVE/'shm_field_fixture'
    if not executable.exists():
        pytest.skip('Build shm_field_fixture and shm_executor for private IPC tests')
    processes = []
    def start(fault=None):
        folder = tmp_path/f'master{len(processes)}';folder.mkdir()
        name = f'/casbot_calib_test_fw_{os.getpid()}_{time.monotonic_ns()}'
        command = [str(executable), name, str(folder/'ready'), str(folder/'commands.bin')]
        if fault is not None:
            command.append(str(fault))
        log = (folder/'log').open('wb')
        process = subprocess.Popen(command, stdout=log, stderr=log)
        processes.append((process, log))
        deadline = time.monotonic()+5
        while not (folder/'ready').exists() and process.poll() is None and time.monotonic()<deadline:
            time.sleep(.01)
        assert (folder/'ready').exists(), (folder/'log').read_text()
        return name, folder
    yield start
    for process, log in processes:
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=5);log.close()


def flags(name, output):
    return ['--endpoint', name, '--executor', NATIVE/'shm_executor', '--output', output]


def active(master, tmp_path, case, *extra, fault=None):
    name, source = master(fault)
    output = tmp_path/('run_'+case)
    completed = cli('run', case, '--execute', *flags(name, output), '--hand-background',
                    ROOT/'tests/fixtures/shm/command_slot_0.bin', '--duration', '.05', '--quiet-s', '.1', *extra)
    result = json.loads((output/'result.json').read_text())
    return completed, result, output, source


def test_preview_does_not_need_binary_shm_or_output(tmp_path):
    output = tmp_path/'must_not_exist'
    result = cli('run', 'C03', '--joint-id', '13', '--executor', tmp_path/'no_binary',
                 '--endpoint', '/casbot_calib_test_missing', '--output', output)
    assert result.returncode == 0, result.stderr
    assert '仅预览' in result.stdout and not output.exists()
    for cmd in ('prepare', 'activate'):
        assert cli(cmd, '--output', output).returncode == 0
        assert not output.exists()


@pytest.mark.parametrize('case', ['C07', 'C08'])
def test_deferred_cases_never_launch(case, tmp_path):
    out = tmp_path/'no_run'
    result = cli('run', case, '--execute', '--output', out)
    assert result.returncode == 2 and not out.exists()


def test_shell_entry_from_elsewhere(tmp_path):
    result = subprocess.run(['bash', ROOT/'fieldwork/shm/run.sh', 'list'], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0 and 'C06' in result.stdout


@pytest.mark.slow
def test_observe_zero_commands_clock_fields_and_note(master, tmp_path):
    name, source = master()
    out = tmp_path/'observe'
    result = cli('observe', '--duration', '.08', *flags(name, out))
    assert result.returncode == 0, result.stderr + result.stdout
    report = json.loads((out/'result.json').read_text())
    assert report['commands']['count'] == 0
    assert source.joinpath('commands.bin').stat().st_size == 0
    assert report['feedback']['count'] > 1
    assert report['feedback']['hands'][0]['items'][0]['position']['first'] == 123
    import csv
    pairs = list(csv.DictReader((out/'native/clock_pairs.csv').open()))
    feedback = _core.read_feedback_csv_zst(out/'native/feedback.csv.zst')
    assert pairs and not feedback['truncated_tail']
    assert (feedback['observation_time_ns'] ==
            feedback['source_timestamp_ms'].astype('int64')*1_000_000-int(pairs[0]['offset_ns'])).all()
    assert cli('note', out, '--text', '固定支撑下未见异常', '--phase', 'observe').returncode == 0
    assert json.loads((out/'result.json').read_text())['observations'][0]['text'] == '固定支撑下未见异常'
    assert source.joinpath('commands.bin').stat().st_size == 0


@pytest.mark.slow
def test_c03_jog_and_c04_disabled_opposite_background(master, tmp_path):
    for case in ('C03', 'C04'):
        extra = ['--joint-id', '13', '--amplitude', '.001', '--segment-s', '1']
        if case == 'C04':
            extra += ['--other-arm', 'disabled']
        completed, result, out, _ = active(master, tmp_path, case, *extra)
        assert completed.returncode == 0, completed.stdout + completed.stderr
        assert result['commands']['arms'][1]['q_rad']['max']-result['commands']['arms'][1]['q_rad']['min'] > .0019
        control = _core.read_commands_csv_zst(out/'native/commands.csv.zst')['cmd_cw']
        assert (control[:, [*range(12), 26, 27, 28]] == 6).all()
        if case == 'C04':
            assert (control[:, 19:26] == 6).all()



@pytest.mark.slow
def test_limit_skip_and_fault_abort(master, tmp_path):
    skipped, result, _, source = active(master, tmp_path, 'C03', '--joint-id', '15')
    assert skipped.returncode == 4 and result['commands']['count'] == 0
    assert source.joinpath('commands.bin').stat().st_size == 0
    failed, result, out, _ = active(master, tmp_path, 'C01', fault=2)
    assert failed.returncode != 0 and not result['execution_completed']
    assert 'drive_error_slot_12_42' == result['native']['reason']
    control = _core.read_commands_csv_zst(out/'native/commands.csv.zst')['cmd_cw'][:, 12]
    assert control[-1] == 6
    assert (control != 0x80).all()


@pytest.mark.slow
def test_c05_silence_does_not_replay(master, tmp_path):
    completed, result, out, _ = active(master, tmp_path, 'C05')
    assert completed.returncode == 0, completed.stdout + completed.stderr
    import csv
    events = list(csv.DictReader((out/'native/events.csv').open()))
    begin = int(next(e['mono_ns'] for e in events if e['event'] == 'command_writes_stopped'))
    end = int(next(e['mono_ns'] for e in events if e['event'] == 'quiet_observation_end_disable'))
    stamps = _core.read_commands_csv_zst(out/'native/commands.csv.zst')['send_mono_ns']
    assert not any(begin <= stamp < end for stamp in stamps)
    assert end > begin
    assert result['native']['disable_ack'] == '1'


@pytest.mark.slow
@pytest.mark.parametrize('termination', ['sigterm', 'sigkill'])
def test_c06_own_sender_termination_passive_probe(master, tmp_path, termination):
    completed, result, out, source = active(master, tmp_path, 'C06', '--termination', termination)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert result['execution_completed'] and result['probe']['count'] > 0
    assert result['process']['termination_sent'] == termination
    if termination == 'sigkill':
        assert result['native'] is None
        assert result['process']['returncode'] == -signal.SIGKILL
        assert struct.unpack_from('<H', source.joinpath('commands.bin').read_bytes()[-744:], 12+12*18)[0] == 15
        assert not len(_core.read_commands_csv_zst(out/'probe/commands.csv.zst')['send_mono_ns'])
        assert len(_core.read_commands_csv_zst(out/'probe/passive_commands.csv.zst')['send_mono_ns']) > 0
    else:
        assert result['native']['disable_ack'] == '1'


@pytest.mark.slow
def test_owner_conflict_does_not_steal_channel(master, tmp_path):
    name, source = master()
    out = tmp_path/'first'
    process = subprocess.Popen([str(NATIVE/'shm_executor'), '--observe', '--shm', name,
                                '--duration', '2', '--output', str(out)], stdout=subprocess.DEVNULL)
    try:
        deadline = time.monotonic()+3
        while not (out/'feedback.csv.zst').exists() and process.poll() is None and time.monotonic()<deadline:
            time.sleep(.01)
        assert process.poll() is None
        conflict = cli('observe', '--duration', '.05', *flags(name, tmp_path/'conflict'))
        assert conflict.returncode != 0 and 'Another fieldwork owner' in (tmp_path/'conflict/native/result.txt').read_text()
        assert source.joinpath('commands.bin').stat().st_size == 0
    finally:
        process.terminate();process.wait(timeout=5)


def test_partial_records_reported_and_not_resumed(tmp_path):
    folder = tmp_path/'native';folder.mkdir()
    from recording_fixtures import write_recording
    path = folder/'feedback.csv.zst'
    write_recording(path, [dict(receive_mono_ns=1000, source_timestamp_ms=123)])
    with path.open('ab') as stream:
        stream.write(b'partial')
    result = fw.report(tmp_path)
    assert result['feedback']['count'] == 1 and result['feedback']['truncated_tail']
    assert not result['execution_completed']


def test_setup_report_and_prepare_private_rejected_without_side_effects(tmp_path, monkeypatch):
    out = tmp_path/'setup'
    called=[]
    monkeypatch.setattr(fw, 'prepare_subprocess', lambda command: called.append(command) or dict(command=command,returncode=0))
    assert fw.main(['prepare', '--execute', '--output', str(out)]) == 0
    assert called and json.loads((out/'result.json').read_text())['execution_completed']
    assert '未封存' not in (out/'report.md').read_text()
    assert fw.main(['prepare','--execute','--endpoint','/casbot_calib_test_no_stop', '--output',str(tmp_path/'never')]) == 2
    assert len(called)==1 and not (tmp_path/'never').exists()


def test_supervisor_timeout_only_terminates_owned_child(tmp_path):
    meta = fw.supervise([sys.executable, '-c', 'import time;time.sleep(10)'], tmp_path, timeout=.05)
    assert meta['supervisor_timeout'] and meta['returncode']==-signal.SIGTERM


@pytest.mark.slow
def test_pty_close_preserves_native_exit_and_wrapper_report(master,tmp_path):
    import pty,select,shlex
    name,_=master();out=tmp_path/'pty_hangup'
    command=[sys.executable,'-m','casbot_calib.shm_fieldwork','run','C01','--execute',
        *map(str,flags(name,out)),'--hand-background',str(ROOT/'tests/fixtures/shm/command_slot_0.bin'),'--duration','5']
    pid,fd=pty.fork()
    if pid==0:
        os.execvp('bash',['bash','--noprofile','--norc','-i'])
    try:
        os.write(fd,(shlex.join(command)+'\n').encode())
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            if select.select([fd],[],[],.01)[0]:os.read(fd,16384)
            events=out/'native/events.csv'
            if events.exists() and 'hold_begin' in events.read_text():break
        else:pytest.fail('Private fixture never reached hold phase')
        os.close(fd);fd=None
        deadline=time.monotonic()+5
        while not (out/'result.json').exists() and time.monotonic()<deadline:time.sleep(.01)
        assert (out/'result.json').exists()
        result=json.loads((out/'result.json').read_text())
        assert result['native']['reason']=='operator_interrupt' and result['native']['disable_ack']=='1'
        assert result['process']['interrupt_signal']=='SIGHUP' and 'ended_mono_ns' in result['process']
    finally:
        if fd is not None:os.close(fd)
        done,_=os.waitpid(pid,os.WNOHANG)
        if not done:
            os.kill(pid,signal.SIGTERM);os.waitpid(pid,0)


@pytest.mark.slow
def test_hangup_during_offline_report_keeps_completed_metadata(master,tmp_path):
    name,_=master();out=tmp_path/'report_hangup'
    code='''import sys
from pathlib import Path
from casbot_calib import shm_fieldwork as fw
original=fw.report
def slow_report(root):
    Path(root,'offline_report_entered').touch()
    sys.stdin.readline()
    return original(root)
fw.report=slow_report
raise SystemExit(fw.main(sys.argv[1:]))
'''
    with (tmp_path/'report_wrapper.log').open('wb') as log:
        process=subprocess.Popen([sys.executable,'-c',code,'run','C01','--execute',
            *map(str,flags(name,out)),'--hand-background',str(ROOT/'tests/fixtures/shm/command_slot_0.bin'),'--duration','.05'],
            stdin=subprocess.PIPE,stdout=log,stderr=log)
        try:
            deadline=time.monotonic()+4
            while not (out/'offline_report_entered').exists() and process.poll() is None and time.monotonic()<deadline:time.sleep(.01)
            assert (out/'offline_report_entered').exists()
            assert 'ended_mono_ns' in json.loads((out/'process.json').read_text())
            process.send_signal(signal.SIGHUP)
            process.stdin.close()
            process.wait(timeout=5)
            assert (out/'result.json').exists()
            assert json.loads((out/'result.json').read_text())['execution_completed']
        finally:
            if process.poll() is None:process.terminate();process.wait(timeout=5)


@pytest.mark.slow
@pytest.mark.parametrize(('injection', 'expected', 'ack'), [
    ({}, 'completed_with_disable_ack', '1'),
    ({'stop_status_after_commands': 140}, 'cst_feedback_stale', '0'),
    ({'stop_command_consume_after': 140}, 'cst_command_stale', '0'),
    ({'fault_after_commands': 140}, 'drive_error_slot_12_42', '0'),
    ({'non_arm_test_motion': {'4': {'vel_raw': 1600000}}}, 'observed_velocity_limit_slot_4', '1'),
    ({}, 'operator_interrupt', '1'),
])
def test_body_hold_smoke_raw_frames(tmp_path, injection, expected, ack):
    import csv
    import numpy as np
    from casbot_calib.shm_data import URDF

    config = json.loads((ROOT / 'configs/shm/h1_mock.json').read_text())
    config.update(urdf=str(URDF), non_arm_test_motion={
        '0': {'pos_raw_offset': 1400},
        '4': {'pos_raw_offset': 600000, 'vel_raw': 100000},
        '5': {'pos_raw_offset': -600000},
        '10': {'vel_raw': -200000},
    })
    config.update(injection)
    path = tmp_path / 'mock.json'
    path.write_text(json.dumps(config))
    output = tmp_path / 'run'
    cmd = ['bash', str(ROOT / 'calib/dev.sh'), 'hold-body', '--mock', '--execute',
           '--duration', '2', '--scale', '1', '--mock-config', str(path),
           '--executor', str(NATIVE / 'shm_executor'), '--output', str(output)]
    native = output / 'capture/native'
    with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as process:
        if expected == 'operator_interrupt':
            deadline = time.monotonic() + 15
            events = native / 'events.csv'
            while process.poll() is None and time.monotonic() < deadline:
                if events.exists() and 'reference_begin' in events.read_text():
                    process.send_signal(signal.SIGINT)
                    break
                time.sleep(.01)
        log, _ = process.communicate(timeout=25)
    assert (process.returncode == 0) == (expected == 'completed_with_disable_ack'), log
    result = fw.text_result(native / 'result.txt')
    assert (result['reason'], result['disable_ack']) == (expected, ack), log
    bundle = json.loads((native / 'reference.json').read_text())
    assert bundle['schema'] == 'casbot_shm_reference_v2'
    assert len(bundle['non_arm_hold']['slots']) == 15 and bundle['observed_velocity_limit_rad_s'] > 0
    records = _core.read_commands_csv_zst(native / 'commands.csv.zst')
    joints = {field: records[column] for field, column in (
        ('control_word', 'cmd_cw'), ('position', 'cmd_pos_raw'), ('velocity', 'cmd_vel_raw'),
        ('torque', 'cmd_trq_raw'), ('mode', 'cmd_mode'), ('kp', 'cmd_kp_raw'), ('kd', 'cmd_kd_raw'),
    )}
    if expected != 'cst_command_stale':
        for field in ('control_word', 'torque', 'kp', 'kd', 'velocity'):
            assert np.all(joints[field][-1] == (6 if field == 'control_word' else 0))
    else:
        assert int(result['send_skipped']) > 0
    if expected in ('cst_command_stale', 'cst_feedback_stale'):
        with (native / 'events.csv').open() as stream:
            failure = next(int(row['mono_ns']) for row in csv.DictReader(stream) if row['event'] == expected)
        if expected == 'cst_command_stale':
            last = records['send_mono_ns'][-1]
        else:
            status = _core.read_feedback_csv_zst(native / 'feedback.csv.zst')
            last = status['receive_mono_ns'][-1]
        assert failure - last >= 19_000_000
    if expected == 'completed_with_disable_ack':
        enabled = joints['control_word'][:, 4] == 15
        active = {field: values[enabled] for field, values in joints.items()}
        assert np.all(active['control_word'] == 15)
        assert np.all(active['position'][:, 0] == 0)  # injected feedback moves after enabling
        gains = active['kp'][:, 0]
        assert gains[0] == 0 and gains[-1] == 4000 and np.all(np.diff(gains) >= 0)
        for slot in (4, 5, 10, 11):
            assert np.all(active['mode'][:, slot] == 10)
            for field in ('position', 'kp', 'kd', 'velocity'):
                assert np.all(active[field][:, slot] == 0)
        assert active['torque'][-1, 4] == -3000 and active['torque'][-1, 5] == 3000
        assert active['torque'][-1, 10] in (59, 60)
        first_stream = int(result['first_stream_mono_ns'])
        middle = np.searchsorted(records['send_mono_ns'], first_stream + 500_000_000)
        assert 1900 < joints['kp'][middle, 0] < 2100
        assert 25 <= joints['torque'][middle, 10] <= 35
