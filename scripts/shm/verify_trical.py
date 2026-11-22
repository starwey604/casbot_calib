#!/usr/bin/env python3
"""Private SHM TriCal execution, block telemetry, shutdown and recording checks."""
import argparse
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'python'))
from casbot_calib.shm_mock import SHMMock


def run_case(binary, root, *, eof=False, fault=False):
    config = json.loads((ROOT / 'configs/shm/h1_mock.json').read_text())
    config['urdf'] = str((ROOT / 'configs/shm' / config['urdf']).resolve())
    if fault:
        config['fault_after_commands'] = 250
    path = root / 'mock.json'
    path.write_text(json.dumps(config))
    with SHMMock(root / 'master', binary, path) as mock:
        command = [str(binary), '--shm', mock.endpoint, '--urdf', config['urdf'],
                   '--output', str(root / 'native'), '--bare-flange', '--tool-side', 'left',
                   '--kp', ','.join(map(str, [65, 65, 30, 30, 30, 20, 20] * 2)),
                   '--kd', ','.join(map(str, [5, 5, 4, 2, 2, 2, 2] * 2))]
        with (root / 'stderr.log').open('w') as errors:
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=errors, text=True, bufsize=1)
            events = queue.Queue()
            def read():
                for line in process.stdout:
                    events.put(json.loads(line))
                events.put(None)
            reader = threading.Thread(target=read)
            reader.start()
            history = []
            def wait(kind, identifier=None, timeout=10):
                deadline = time.monotonic() + timeout
                while True:
                    event = events.get(timeout=max(.01, deadline - time.monotonic()))
                    assert event is not None, (root / 'stderr.log').read_text()
                    history.append(event)
                    if event['type'] == kind and (identifier is None or event.get('id') == identifier):
                        return event
                    if time.monotonic() >= deadline:
                        raise TimeoutError(kind)
            def send(message):
                process.stdin.write(json.dumps(message) + '\n')
                process.stdin.flush()
            try:
                ready = wait('ready')
                q0 = ready['q0']
                if fault:
                    finished = wait('finished')
                    assert not finished['ok'] and not finished['disable_ack']
                    assert finished['reason'].startswith('drive_error')
                elif eof:
                    process.stdin.close()
                    finished = wait('finished')
                    assert not finished['ok'] and finished['disable_ack']
                else:
                    bad = q0.copy(); bad[0] += .05
                    send(dict(id=1, type='segment', duration_ns=1_000_000_000,
                              q_from=q0, q_to=bad, windup=True))
                    reply = wait('reply', 1)
                    assert not reply['ok'] and reply['reason'] == 'spring_windup_limit'
                    first = wait('state')
                    assert wait('state')['end_ns'] > first['end_ns']
                    target = q0.copy(); target[0] += .002
                    for identifier, start, end in ((2, q0, target), (3, target, q0)):
                        settle_deadline = time.monotonic() + 10
                        while True:
                            state = wait('state', timeout=max(.01, settle_deadline - time.monotonic()))
                            if state['holding'] and all(
                                max(abs(lo - goal), abs(hi - goal)) < .010 and hi - lo < .001
                                for lo, hi, goal in zip(state['q_min'], state['q_max'], end)
                            ):
                                break
                            if time.monotonic() >= settle_deadline:
                                raise AssertionError(f'TriCal mock did not settle at segment {identifier}: {state}')
                        send(dict(id=identifier, type='segment', duration_ns=200_000_000,
                                  q_from=start, q_to=end, windup=True))
                        reply = wait('reply', identifier)
                        assert reply['ok'], reply
                        wait('segment_done', identifier)
                    send(dict(id=4, type='finish'))
                    finished = wait('finished')
                    assert finished['ok'] and finished['disable_ack']
                assert process.wait(timeout=5) == (1 if eof or fault else 0)
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)
                reader.join(timeout=5)
                process.stdout.close()
                if not process.stdin.closed:
                    process.stdin.close()
            (root / 'events.json').write_text(json.dumps(history, indent=2))
    result = dict(line.split('=', 1) for line in (root / 'native/result.txt').read_text().splitlines())
    assert result['disable_ack'] == ('0' if fault else '1')
    assert result['recording_stopped'] == '0'
    assert result['execution_completed'] == ('0' if eof or fault else '1')
    if not eof and not fault:
        blocks = [e for e in history if e['type'] == 'state']
        assert blocks
        assert all(a['end_ns'] < b['start_ns'] for a, b in zip(blocks, blocks[1:]))
        assert all(b['count'] > 0 and len(b['q_min']) == len(b['q_max']) == 14 for b in blocks)
        from casbot_calib import _core
        commands = _core.read_commands_csv_zst(root / 'native/commands.csv.zst')
        assert not commands['truncated_tail']
        assert (commands['hand_count'] == 0).all()
        assert (commands['cmd_cw'][:, :12] == 6).all()
    print(f'TriCal {root.name}: PASS', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bin-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    temporary = tempfile.TemporaryDirectory(prefix='casbot-trical-') if args.output is None else None
    root = Path(temporary.name) if temporary else args.output.resolve()
    root.mkdir(parents=True, exist_ok=True)
    try:
        for name, options in [('segments', {}), ('eof', dict(eof=True)), ('fault', dict(fault=True))]:
            directory = root / name; directory.mkdir()
            run_case(args.bin_dir.resolve() / 'shm_trical_executor', directory, **options)
    finally:
        if temporary:
            temporary.cleanup()


if __name__ == '__main__':
    main()
