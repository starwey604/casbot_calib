#!/usr/bin/env python3
"""Installed native preparation -> H1 acceptance, private physical SHM only."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'python'))
from casbot_calib.shm_h1 import DEFAULT_PROFILE,load_profile,snapshot,prepare,execute,analyse
from casbot_calib.model_runtime import model_for,urdf_for
from casbot_calib.data import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bin-dir',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    native=args.bin_dir.resolve()/'shm_executor';config=load_profile(DEFAULT_PROFILE);engine,_=model_for(config)
    # Explicit held synthetic fixture at the measured near-leg hanging angles.
    # No collision/contact forces or hidden gravity compensation are invented.
    q=np.array([-.037817,.10047065,-.132868,-.016,.023154,.045726,-.12043,
                -.014166,-.11496665,-.12538533,-.054911,.18399067,.03161545,-.05161767])
    fixture=json.loads((ROOT/'configs/shm/h1_mock.json').read_text());fixture['urdf']=str(urdf_for(config))
    fixture['initial_q']=q.tolist();fixture['initial_fields']['position']=q.tolist();fixture['initial_fields']['effort']=engine.gravity(q).tolist()
    write_json(root/'fixture.json',fixture)
    snapshot(DEFAULT_PROFILE,root/'start',executor=native,mock=True,mock_config=root/'fixture.json')
    plan=prepare(DEFAULT_PROFILE,root/'start',root/'plan',executor=native,with_ready=True)
    assert execute(root/'plan',root/'capture',executor=native,mock=True,do_execute=True)==0
    analysis=analyse(root/'plan',root/'capture',root/'analysis')
    collection=json.loads((root/'capture/result.json').read_text());capture=root/'capture'/collection['effective_capture']
    result=json.loads((capture/'result.json').read_text())
    with (capture/'native/events.csv').open() as f:events=[r['event'] for r in csv.DictReader(f)]
    assert events.count('enabling_at_start')==1 and events.count('h1_calibration_begin')==1
    assert result['native']['disable_ack']=='1' and analysis['analysis_available']
    collision=json.loads((root/'analysis/collision_received_interpolant.json').read_text())
    with (capture/'native/commands.csv').open() as f:
        for row in csv.DictReader(f):
            if row['phase']=='3':
                relative=int(row['send_mono_ns'])-int(result['native']['first_command_mono_ns'])
                if relative<plan['preparation']['move_end_ns']:
                    assert all(float(row['effort'+str(j)])==0 and float(row['kp'+str(j)])==20 and float(row['kd'+str(j)])==2 for j in range(14))
    verification=dict(passed=True,hardware_tested=False,scope='private_physics_scripted_ready_then_H1_ID12_1deg',
        duration_ns=plan['duration_ns'],preparation=plan['preparation'],enable_cycles=1,disable_ack=True,
        feedback_rows=analysis['feedback_rows'],formal_collision_checked_rows=collision['collision_checked_rows'],
        excluded_preparation_entry_exit_rows=collision['excluded_rows'],analysis_available=analysis['analysis_available'])
    write_json(root/'verification.json',verification);print(json.dumps(verification,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
