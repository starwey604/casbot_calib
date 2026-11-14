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

from casbot_calib import shm_fieldwork as fw

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='POSIX SHM fieldwork requires Linux')

ROOT = Path(__file__).resolve().parents[1]
NATIVE = Path(os.environ.get('CASBOT_SHM_TEST_BIN', ROOT/'build/humble/cpp/src/shm'))


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


def test_shell_entry_from_elsewhere_and_ros_aliases(tmp_path):
    result = subprocess.run(['bash', ROOT/'fieldwork/shm/run.sh', 'list'], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0 and 'C06' in result.stdout
    script = (ROOT/'fieldwork/ros/run.sh').read_text()
    assert '/../..' in script and 'scripts/humble/container.sh' in script
    assert '/ros/run.sh' in (ROOT/'fieldwork/run.sh').read_text()
    for name in ('capture_zero.sh', 'return_zero.sh'):
        assert '/ros/'+name in (ROOT/'fieldwork'/name).read_text()


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
    rows = list(csv.DictReader((out/'native/feedback.csv').open()))
    assert pairs
    for row in rows:
        assert int(row['observation_time_ns']) == int(row['source_timestamp_ms'])*1_000_000-int(pairs[0]['offset_ns'])
    assert cli('note', out, '--text', '固定支撑下未见异常', '--phase', 'observe').returncode == 0
    assert json.loads((out/'result.json').read_text())['observations'][0]['text'] == '固定支撑下未见异常'
    assert source.joinpath('commands.bin').stat().st_size == 0


@pytest.mark.parametrize('case', ['C01', 'C02'])
def test_hold_enable_and_disable(master, tmp_path, case):
    completed, result, out, _ = active(master, tmp_path, case)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert result['execution_completed'] and result['native']['disable_ack'] == '1'
    assert not result['observation_recorded']
    assert result['feedback']['joints'][12]['statuses'] == [0x231, 0x237]
    assert (out/'native/background.bin').stat().st_size == 744


def test_c03_jog_and_c04_disabled_opposite_background(master, tmp_path):
    for case in ('C03', 'C04'):
        extra = ['--joint-id', '13', '--amplitude', '.001', '--segment-s', '1']
        if case == 'C04':
            extra += ['--other-arm', 'disabled']
        completed, result, out, _ = active(master, tmp_path, case, *extra)
        assert completed.returncode == 0, completed.stdout + completed.stderr
        assert result['commands']['arms'][1]['q_rad']['max']-result['commands']['arms'][1]['q_rad']['min'] > .0019
        for data in fw.records(out/'native/commands.bin', 752):
            for j in list(range(12))+[26, 27, 28]:
                assert struct.unpack_from('<H', data, 8+12+j*18)[0] == 6
            if case == 'C04':
                assert all(struct.unpack_from('<H', data, 8+12+j*18)[0] == 6 for j in range(19,26))


def test_limit_skip_and_fault_abort(master, tmp_path):
    skipped, result, _, source = active(master, tmp_path, 'C03', '--joint-id', '15')
    assert skipped.returncode == 4 and result['commands']['count'] == 0
    assert source.joinpath('commands.bin').stat().st_size == 0
    failed, result, out, _ = active(master, tmp_path, 'C01', fault=2)
    assert failed.returncode != 0 and not result['execution_completed']
    assert 'drive_error_slot_12_42' == result['native']['reason']
    commands = list(fw.records(out/'native/commands.bin', 752))
    assert struct.unpack_from('<H', commands[-1], 8+12+12*18)[0] == 6
    assert all(struct.unpack_from('<H', r, 8+12+12*18)[0] != 0x80 for r in commands)


def test_c05_silence_does_not_replay(master, tmp_path):
    completed, result, out, _ = active(master, tmp_path, 'C05')
    assert completed.returncode == 0, completed.stdout + completed.stderr
    import csv
    events = list(csv.DictReader((out/'native/events.csv').open()))
    begin = int(next(e['mono_ns'] for e in events if e['event'] == 'command_writes_stopped'))
    end = int(next(e['mono_ns'] for e in events if e['event'] == 'quiet_observation_end_disable'))
    stamps = [struct.unpack_from('<q', d)[0] for d in fw.records(out/'native/commands.bin', 752)]
    assert not any(begin <= stamp < end for stamp in stamps)
    assert end-begin >= 95_000_000
    assert result['native']['disable_ack'] == '1'


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
        assert not (out/'probe/commands.bin').stat().st_size
        assert (out/'probe/passive_commands.bin').stat().st_size > 0
    else:
        assert result['native']['disable_ack'] == '1'


def test_owner_conflict_does_not_steal_channel(master, tmp_path):
    name, source = master()
    out = tmp_path/'first'
    process = subprocess.Popen([str(NATIVE/'shm_executor'), '--observe', '--shm', name,
                                '--duration', '2', '--output', str(out)], stdout=subprocess.DEVNULL)
    try:
        deadline = time.monotonic()+3
        while not (out/'feedback.bin').exists() and process.poll() is None and time.monotonic()<deadline:
            time.sleep(.01)
        assert process.poll() is None
        conflict = cli('observe', '--duration', '.05', *flags(name, tmp_path/'conflict'))
        assert conflict.returncode != 0 and 'Another fieldwork owner' in (tmp_path/'conflict/native/result.txt').read_text()
        assert source.joinpath('commands.bin').stat().st_size == 0
    finally:
        process.terminate();process.wait(timeout=5)


def test_partial_records_reported_and_not_resumed(tmp_path):
    folder = tmp_path/'native';folder.mkdir()
    state = bytearray(696);struct.pack_into('<Q', state, 680, 123)
    (folder/'feedback.bin').write_bytes(struct.pack('<q', 1000)+state+b'partial')
    result = fw.report(tmp_path)
    assert result['feedback']['count'] == 1 and result['feedback']['trailing_bytes'] == 7
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


def test_hangup_to_wrapper_disables_owned_native_sender(master, tmp_path):
    name, _ = master();out=tmp_path/'hangup'
    with (tmp_path/'wrapper.log').open('wb') as log:
        process = subprocess.Popen([sys.executable, '-m', 'casbot_calib.shm_fieldwork', 'run', 'C01', '--execute',
                                    *map(str,flags(name,out)), '--hand-background', str(ROOT/'tests/fixtures/shm/command_slot_0.bin'),
                                    '--duration', '5'], stdout=log, stderr=log)
        try:
            deadline=time.monotonic()+5
            while process.poll() is None and time.monotonic()<deadline:
                events=out/'native/events.csv'
                if events.exists() and 'hold_begin' in events.read_text():
                    break
                time.sleep(.01)
            assert process.poll() is None
            process.send_signal(signal.SIGHUP);process.wait(timeout=7)
            result=json.loads((out/'result.json').read_text())
            assert result['native']['reason']=='operator_interrupt' and result['native']['disable_ack']=='1'
            assert result['process']['operator_interrupted']
        finally:
            if process.poll() is None:
                process.terminate();process.wait(timeout=7)


def test_repeated_hangup_still_finalizes_exit_and_report(master, tmp_path):
    """SSH/PTTY can hang up both the shell and its job; cleanup must finish once."""
    name, _ = master();out=tmp_path/'repeated_hangup'
    with (tmp_path/'wrapper.log').open('wb') as log:
        process=subprocess.Popen([sys.executable,'-m','casbot_calib.shm_fieldwork','run','C01','--execute',
            *map(str,flags(name,out)),'--hand-background',str(ROOT/'tests/fixtures/shm/command_slot_0.bin'),
            '--duration','5'],stdout=log,stderr=log)
        try:
            deadline=time.monotonic()+4
            while process.poll() is None and time.monotonic()<deadline:
                events=out/'native/events.csv'
                if events.exists() and 'hold_begin' in events.read_text():break
                time.sleep(.01)
            assert process.poll() is None
            for _ in range(40):
                if process.poll() is not None:break
                process.send_signal(signal.SIGHUP);time.sleep(.002)
            process.wait(timeout=7)
            assert (out/'result.json').exists(),(tmp_path/'wrapper.log').read_text()
            result=json.loads((out/'result.json').read_text())
            assert result['native']['reason']=='operator_interrupt' and result['native']['disable_ack']=='1'
            assert result['process']['operator_interrupted'] and 'ended_mono_ns' in result['process']
            assert result['process']['returncode']==1
        finally:
            if process.poll() is None:process.terminate();process.wait(timeout=7)



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


def test_hangup_during_offline_report_keeps_completed_metadata(master,tmp_path):
    name,_=master();out=tmp_path/'report_hangup'
    code='''import sys,time
from pathlib import Path
from casbot_calib import shm_fieldwork as fw
original=fw.report
def slow_report(root):
    Path(root,'offline_report_entered').touch()
    time.sleep(.15)
    return original(root)
fw.report=slow_report
raise SystemExit(fw.main(sys.argv[1:]))
'''
    with (tmp_path/'report_wrapper.log').open('wb') as log:
        process=subprocess.Popen([sys.executable,'-c',code,'run','C01','--execute',
            *map(str,flags(name,out)),'--hand-background',str(ROOT/'tests/fixtures/shm/command_slot_0.bin'),'--duration','.05'],
            stdout=log,stderr=log)
        try:
            deadline=time.monotonic()+4
            while not (out/'offline_report_entered').exists() and process.poll() is None and time.monotonic()<deadline:time.sleep(.01)
            assert (out/'offline_report_entered').exists()
            assert 'ended_mono_ns' in json.loads((out/'process.json').read_text())
            process.send_signal(signal.SIGHUP);process.wait(timeout=5)
            assert (out/'result.json').exists()
            assert json.loads((out/'result.json').read_text())['execution_completed']
        finally:
            if process.poll() is None:process.terminate();process.wait(timeout=5)
