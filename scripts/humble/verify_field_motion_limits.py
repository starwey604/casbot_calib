#!/usr/bin/env python3
"""Nonvisual ROS: field position warns, speed still ends, no tracking cutoff."""
import argparse
import json
import os
from pathlib import Path
import time

import numpy as np

from verify_hardware_start_replan import executing, read, rows, wait_for


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--domain-id',type=int,required=True)
    args = parser.parse_args(); root = args.output.resolve(); root.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
    from verify_fieldwork_round1 import ProtocolFixture
    from casbot_calib import hardware_field as field,fieldwork_settings as policy
    from casbot_calib.fieldwork_control import initial_commands,tuning_config,write_command
    from casbot_calib.staged_motion import Motion
    fixture = ProtocolFixture(root/'fixture',args.domain_id); fixture.velocity=[0.]*14; fixture.enable_service()
    checks = {}
    try:
        config = field.load_profile(fixture.client_profile); engine,_ = field.model_for(config)
        home = np.array(fixture.q); outside = engine.model.joints[5].upper+.03
        field.run_executor(config,root/'snapshot',args.domain_id,snapshot=True)
        motion = Motion('deliberately_untracked','position',home)
        target = home.copy(); target[0]+=.08
        motion.add(.3,home,label='hold').add(1.,target,label='move').add(2.2,target,label='hold')
        motion.add(1.,home,label='return').add(.3,home,label='hold')
        plan = root/'untracked_plan'; plan.mkdir()
        field.finish_plan(plan,config,motion,[],field.snapshot_binding(root/'snapshot',config))
        output = root/'untracked_capture'
        with executing(config,output,args.domain_id,plan/'position_plan.csv') as (thread,future):
            wait_for(lambda:any(float(r.get('q0',home[0]))>home[0]+.04 for r in rows(output/'execution/commands.csv')))
            count = fixture.commands
            # Injected SDK fields only: the fixture deliberately does not follow qref.
            fixture.q[5]=outside;fixture.velocity[5]=1.
            time.sleep(1.05)
            checks['position_outside_urdf_and_speed_1_continues'] = thread.is_alive() and fixture.commands>count
            fixture.q[5]=float(home[5]);fixture.velocity[5]=0.
            thread.join(timeout=10)
        result = future['value']; native = read(output/'execution/result.json')
        log = rows(output/'execution/observed_limits.csv')
        checks.update(untracked_reference_finishes=result['passed'],
            tracking_gate_absent=not native['tracking_gate_enabled'],
            position_action_recorded=native['position_limit_action']=='warn',
            diagnostic_has_axis_value_threshold=any(r['joint']==fixture.names[5] and r['field']=='position_upper'
                and r['action']=='warn' and abs(float(r['value'])-outside)<1e-12 for r in log),
            recovery_recorded=any(r['action']=='recovered' for r in log),
            actual_feedback_preserved=any(float(r['q5'])>engine.model.joints[5].upper for r in rows(output/'execution/feedback.csv')))
        wait_for(lambda:not fixture.active)
        hold = root/'hold_plan'; hold.mkdir()
        field.finish_plan(hold,config,Motion('hold','position',home).add(8.,home,label='hold'),[],{})
        output = root/'overspeed_capture'
        with executing(config,output,args.domain_id,hold/'position_plan.csv') as (thread,future):
            wait_for(lambda:any(r.get('state')=='STREAMING' for r in rows(output/'execution/events.csv')))
            fixture.velocity[0]=1.6
            wait_for(lambda:any(r.get('state')=='STOPPING' for r in rows(output/'execution/events.csv')))
            fixture.velocity[0]=0.;thread.join(timeout=6)
        result = future['value']; log = rows(output/'execution/observed_limits.csv')
        stop = next(int(r['receive_ns']) for r in rows(output/'execution/events.csv') if r['state']=='STOPPING')
        checks.update(speed_1_6_ends=result['fault']=='observed_limit',
            speed_threshold_is_1_5=any(r['field']=='velocity' and float(r['threshold'])==1.5 and r['action']=='end_control' for r in log),
            no_publish_after_speed_stop=all(int(r['publish_ns'])<=stop for r in rows(output/'execution/commands.csv')))
        wait_for(lambda:not fixture.active)
        manual = root/'manual';manual.mkdir()
        settings = policy.defaults(config,fixture.names,config['model_context'])
        settings['general']['maximum_duration_s']=1.2
        settings['overrides'][fixture.names[5]]=dict(observed_action=1)
        lower = [j.lower for j in engine.model.joints]; upper = [j.upper for j in engine.model.joints]
        policy.write_wire(manual/'limits.csv',settings,lower,upper)
        values,_ = initial_commands(home,None,None,tuning_config(config))
        write_command(manual/'latest.csv',0,values)
        output = manual/'capture'
        with executing(config,output,args.domain_id,manual_command=manual/'latest.csv',manual_policy=manual/'limits.csv') as (thread,future):
            wait_for(lambda:any(r.get('state')=='STREAMING' for r in rows(output/'execution/events.csv')))
            fixture.q[5]=outside
            thread.join(timeout=8)
            fixture.q[5]=float(home[5])
        result = future['value']; log = rows(output/'execution/observed_limits.csv')
        checks.update(manual_position_does_not_end_even_with_end_action=result['fault']=='duration_complete',
            manual_warning_archived=any(r['field']=='position_upper' and r['action']=='warn' for r in log),
            manual_async_archive_complete=not result['recording_paused'])
    finally: fixture.close()
    report = dict(passed=all(checks.values()),checks=checks,visual_inspection=False,
        scope='private localhost fixed-state/injected SDK fields; not physical motion or dynamics qualification')
    (root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
