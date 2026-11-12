#!/usr/bin/env python3
"""Private fixed-state ROS fixture: record once and continue without quality gates."""
import argparse
import json
import os
from pathlib import Path
import sys

import yaml

from verify_hardware_start_replan import executing, rows, wait_for


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--domain-id',type=int,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
    sys.path.insert(0,str(Path('tests').resolve()))
    from test_hardware_field import make_inputs
    from verify_fieldwork_round1 import ProtocolFixture
    from casbot_calib import hardware_field as field,hardware_batch as batch
    preset,_=make_inputs(root)
    fixture=ProtocolFixture(root/'fixture',args.domain_id);fixture.velocity=[0.]*14;fixture.enable_service()
    fixture.timer.timer_period_ns=80_000_000;fixture.timer.reset()
    config=yaml.safe_load(fixture.client_profile.read_text())
    config.update(initial_hold_s=.05,final_hold_s=.05,anchor_transition_s=.1,gravity_feedforward='none')
    profile=root/'profile.yaml';profile.write_text(yaml.safe_dump(config))
    settings=yaml.safe_load(Path('configs/hardware_batch.yaml').read_text())
    settings['direction'].update(joints=[fixture.names[0]],amplitude_deg=.05)
    settings['frequency'].update(joints=[fixture.names[0]],rates_hz=[500],repeats=1,amplitude_deg=.05)
    settings['inter_job_pause_s']=0.
    path=root/'batch.yaml';path.write_text(yaml.safe_dump(settings))
    counts=dict(executions=0,clears=0)
    class Backend(batch.SDKBatchBackend):
        def execute(self,plan,output,domain_id):
            counts['executions']+=1
            metadata=batch.read(Path(plan)/'plan.json')
            try:
                with executing(metadata['config'],output,domain_id,Path(plan)/'position_plan.csv') as (thread,future):
                    wait_for(lambda:any(r.get('state')=='STREAMING' for r in rows(Path(output)/'execution/events.csv')))
                    fixture.velocity[0]=.2
                    thread.join(timeout=metadata['duration_s']+10)
                return future['value']
            finally:fixture.velocity=[0.]*14
        def clear_fault(self,reason):
            counts['clears']+=1
            return super().clear_fault(reason)
    try:
        from casbot_calib.sdk_connection import connection_remaps
        backend=Backend(connection_remaps(config,args.domain_id))
        batch.plan_batch(path,profile,root/'session',preset=preset,domain_id=args.domain_id)
        report=batch.run_batch(root/'session',backend=backend)
        history,_=batch.outcomes(root/'session',batch.load_batch(root/'session')[1])
        windows=[w for directory in (root/'session/attempts').glob('static_*/attempt_000')
                 for w in batch.read(directory/'inspection/static_windows.json')]
        checks=dict(workflow_completed=report['passed'] and report['collection_complete'],
            no_review_pause=report['state']=='completed',each_job_executed_once=all(len(r)==1 for r in history.values()),
            recorded_without_quality_acceptance=all(r[0]['status']=='recorded' for r in history.values()),
            static_speed_0_2_retained=any(w['maximum_received_speed_rad_s']>=.2 for w in windows),
            callback_gaps_over_50ms_retained=any(w['maximum_callback_gap_s'] and w['maximum_callback_gap_s']>.05 for w in windows),
            no_fault_clear=counts['clears']==0,all_four_jobs_executed=counts['executions']==4,
            formal_qualification_unchanged=not report['formal_hardware_calibration_accepted'])
    finally:fixture.close()
    result=dict(passed=all(checks.values()),checks=checks,scope='private localhost injected SDK fields; fixed q, no physical dynamics or GUI render')
    (root/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
