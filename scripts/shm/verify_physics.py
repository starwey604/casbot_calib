#!/usr/bin/env python3
"""Standard-library SHM physical-loop acceptance. Private endpoints only."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'python'))
from casbot_calib.shm_reference import jog_bundle,save_bundle


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bin-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,help='Optional new evidence directory; otherwise temporary')
    args=p.parse_args();binary=args.bin_dir.resolve()/'shm_executor'
    temporary=None
    if args.output:
        root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    else:
        temporary=tempfile.TemporaryDirectory(prefix='casbot-shm-physics-');root=Path(temporary.name)
    try:
        reference=save_bundle(root/'plan',jog_bundle([0.]*14,12,.01,1,relative=True))
        command=[sys.executable,'-m','casbot_calib.shm_fieldwork','execute',str(reference),'--mock','--execute',
                 '--executor',str(binary),'--output',str(root/'reference')]
        env=dict(os.environ,PYTHONPATH=str(ROOT/'python'))
        done=subprocess.run(command,capture_output=True,text=True,env=env,timeout=20)
        (root/'reference_cli.log').write_text(done.stdout+done.stderr)
        assert done.returncode==0,done.stdout+done.stderr
        result=json.loads((root/'reference/result.json').read_text())
        assert result['execution_completed'] and result['native']['disable_ack']=='1' and result['mock_exit']['returncode']==0
        observed=result['feedback']['joints'][12];sent=result['commands']['arms'][0]['q_rad']
        assert observed['raw_torque']['min']!=0 or observed['raw_torque']['max']!=0
        mismatch=max(abs(observed['q_rad'][key]-sent[key]) for key in ('min','max','last'))
        assert mismatch>1e-4,'Physics feedback unexpectedly equals target positions'
        endpoint=result['case']['endpoint'];assert not Path('/dev/shm'+endpoint).exists()
        silent=[sys.executable,'-m','casbot_calib.shm_fieldwork','run','C05','--mock','--execute','--duration','.1','--quiet-s','.15',
                '--executor',str(binary),'--output',str(root/'silence')]
        done=subprocess.run(silent,capture_output=True,text=True,env=env,timeout=15)
        (root/'silence_cli.log').write_text(done.stdout+done.stderr)
        assert done.returncode==0,done.stdout+done.stderr
        r=json.loads((root/'silence/result.json').read_text());assert r['execution_completed']
        events=r['timing']['events']
        begin=int(next(e['mono_ns'] for e in events if e['event']=='command_writes_stopped'))
        end=int(next(e['mono_ns'] for e in events if e['event']=='quiet_observation_end_disable'))
        with (root/'silence/native/commands.csv').open() as stream:
            assert not any(begin<=int(row['send_mono_ns'])<end for row in csv.DictReader(stream))
        with (root/'silence/native/feedback.csv').open() as stream:
            held=[row for row in csv.DictReader(stream) if begin<int(row['receive_mono_ns'])<end]
        assert held and all(int(row['status12'])&0x006f==0x27 for row in held),'Mock invented a silence watchdog'
        summary=dict(passed=True,scope='private_shared_physics',reference_mismatch_rad=mismatch,
                     feedback_count=result['feedback']['count'],silence_feedback_count=len(held),hardware_tested=False)
        (root/'verification.json').write_text(json.dumps(summary,indent=2)+'\n')
        print('Private SHM physics/reference/quantized feedback/silence/cleanup: PASS')
        if args.output:print(root/'verification.json')
    finally:
        if temporary:temporary.cleanup()


if __name__=='__main__':main()
