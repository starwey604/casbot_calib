#!/usr/bin/env python3
"""Standard-library private IPC smoke test, usable on Orin without pytest/ROS."""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'python'))
from casbot_calib import shm_fieldwork as fw


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bin-dir', type=Path, required=True)
    a = p.parse_args();binaries = a.bin_dir.resolve()
    fixture = binaries/'shm_field_fixture';executor = binaries/'shm_executor'
    if not fixture.is_file() or not executor.is_file():
        raise SystemExit('Build shm_executor and shm_field_fixture first')
    with tempfile.TemporaryDirectory(prefix='casbot-shm-field-') as temporary:
        root = Path(temporary);name = f'/casbot_calib_test_verify_{os.getpid()}_{time.monotonic_ns()}'
        with (root/'fixture.log').open('wb') as log:
            master = subprocess.Popen([str(fixture), name, str(root/'ready'), str(root/'master_commands.bin')], stdout=log, stderr=log)
            try:
                deadline=time.monotonic()+5
                while not (root/'ready').exists() and master.poll() is None and time.monotonic()<deadline:
                    time.sleep(.01)
                assert (root/'ready').exists(), 'Private fixture did not start'
                def run(action, label, *extra, expected=0):
                    out = root/label
                    command=[sys.executable, '-m', 'casbot_calib.shm_fieldwork', action, *map(str, extra),
                             '--executor', str(executor), '--endpoint', name, '--output', str(out)]
                    env=dict(os.environ, PYTHONPATH=str(ROOT/'python'))
                    child=subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, timeout=15)
                    assert child.returncode==expected, child.stdout
                    return json.loads((out/'result.json').read_text()), out
                result, _ = run('observe', 'observe', '--duration', '.08')
                assert result['feedback']['count']>1 and result['commands']['count']==0
                assert (root/'master_commands.bin').stat().st_size==0, 'Observe wrote a command'
                for case in ('C01','C02','C03','C04','C05','C06'):
                    options=[case,'--execute','--duration','.05','--quiet-s','.1','--hand-background',str(ROOT/'tests/fixtures/shm/command_slot_0.bin')]
                    if case in ('C03','C04'):
                        options+=['--joint-id','13','--amplitude','.001','--segment-s','1']
                    if case=='C04':
                        options+=['--other-arm','disabled']
                    if case=='C06':
                        options+=['--termination','sigterm']
                    result,_=run('run',case,*options)
                    assert result['execution_completed'], case
                result,out=run('inspect','C00')
                assert result['snapshot'] and (out/'native/command.bin').stat().st_size==744
                # Hard death is deliberately last: no reset/post recovery afterward.
                result,_=run('run','C06kill','C06','--execute','--duration','.05','--quiet-s','.1',
                             '--termination','sigkill','--hand-background',ROOT/'tests/fixtures/shm/command_slot_0.bin')
                assert result['execution_completed'] and result['process']['returncode']==-signal.SIGKILL
                latest=(root/'master_commands.bin').read_bytes()[-744:]
                assert struct.unpack_from('<H',latest,12+12*18)[0]==15, 'Probe unexpectedly sent disable'
            finally:
                if master.poll() is None:
                    master.terminate()
                master.wait(timeout=5)
    print('SHM fieldwork private protocol: observe zero-write, C00–C06, opposite-arm disable and passive hard-death probe passed. Not a physics Mock.')


if __name__=='__main__':
    main()
