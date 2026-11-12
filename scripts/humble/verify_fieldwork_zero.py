#!/usr/bin/env python3
"""Private ROS, no render: readonly bookmark and GUI worker one-way completion."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

from verify_hardware_start_replan import read, rows, wait_for


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--domain-id',type=int,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
    from verify_fieldwork_round1 import ProtocolFixture
    from casbot_calib import hardware_field as field,fieldwork_zero as zero
    from casbot_calib.fieldwork_control import create_session,Session
    from casbot_calib.fieldwork_ipc import Child
    from casbot_calib.fieldwork import clock_id
    from casbot_calib.hardware_batch import batch_lock
    fixture=ProtocolFixture(root/'fixture',args.domain_id);fixture.enable_service()
    checks={};child=None
    try:
        config=field.load_profile(fixture.client_profile);home=np.array(fixture.q)
        saved=zero.capture_zero(config,args.domain_id,'localhost_contract_mock',path=root/'zero.yaml',output=root/'capture_zero')
        checks['capture_with_no_velocity_or_effort_gate']=saved['passed'] and fixture.commands==0 and fixture.calls==0
        np.testing.assert_allclose(list(saved['zero']['position_rad'].values()),home,atol=1e-12)
        # A second capture updates exactly the same file, not a caller-named version.
        fixture.q[0]+=.002
        saved=zero.capture_zero(config,args.domain_id,'localhost_contract_mock',path=root/'zero.yaml',output=root/'recapture_zero')
        checks['recapture_updates_fixed_file']=abs(list(saved['zero']['position_rad'].values())[0]-fixture.q[0])<1e-12
        target=np.array(fixture.q);fixture.q[0]+=.025;fixture.velocity=[0.]*14
        workspace=root/'workspace';create_session(fixture.client_profile,workspace,args.domain_id,'localhost_contract_mock',workspace=True)
        from casbot_calib.fieldwork_control import Workspace
        control=workspace/Workspace(workspace).new_control();output=control/'runs/zero_test'
        request=root/'request.json';request.write_text(json.dumps(dict(action='zero_return',values=dict(
            zero=saved['zero'],output=str(output),duration_s=1.,kp=[0.]+[20.]*13,kd=[0.]+[2.]*13))))
        owner='zero_verify';clock=clock_id()
        # Keep the workspace observer lock held, just like the real persistent observer.
        with batch_lock(workspace):
            child=Child([sys.executable,'-m','casbot_calib.fieldwork_worker','--session',str(control),
                '--request',str(request),'--heartbeat',str(root/'unused_heartbeat.json'),'--owner-id',owner,
                '--output',str(root/'worker_result.json'),'--pipe-heartbeat'],root/'worker.log')
            end=time.monotonic()+35;injected=False;messages=[]
            while not child.done() and time.monotonic()<end:
                child.pulse(dict(owner_id=owner,clock_id=clock,monotonic_ns=time.monotonic_ns(),cancel=None))
                messages.extend(child.drain())
                if any(r.get('state')=='STREAMING' for r in rows(output/'capture/execution/events.csv')):
                    fixture.velocity[0]=.2;injected=True
                time.sleep(.025)
            if not child.done():raise RuntimeError('zero worker timed out')
            messages.extend(child.drain());child.close();child=None
        response=read(root/'worker_result.json');result=response['result']
        commands=rows(output/'capture/execution/commands.csv');events=rows(output/'capture/execution/events.csv')
        checks.update(gui_worker_owns_separate_control_without_observer_lock_conflict=response['completed'],
            no_return_position_or_velocity_acceptance=injected and result['passed'] and result['result']['reason']=='trajectory_sent_and_mode_exited',
            no_settling_phase=not any(r['state']=='SETTLING' for r in events),
            single_way_reference=abs(float(commands[0]['q0'])-float(commands[-1]['q0']))>.02,
            final_command_is_saved_target=all(abs(float(commands[-1]['q'+str(j)])-target[j])<1e-10 for j in range(14)),
            nonzero_actual_error_recorded=abs(result['final_received_error_rad'][0])>.02,
            exited_mode=not fixture.active,
            explicit_pd_only=not result['hidden_compensation'] and all(float(r['effort0'])==0 and float(r['kp0'])==0 and float(r['kd0'])==0 and float(r['kp1'])==20 and float(r['kd1'])==2 for r in commands),
            no_session_resume=Session(control).status()['runs']==['runs/zero_test'])
        # Supervisor cancel must stop the operation and not send a new trajectory.
        result=zero.return_zero(config,args.domain_id,'localhost_contract_mock',path=root/'zero.yaml',
            output=root/'cancel_return',cancel_check=lambda:'operator_cancel',notify=lambda _:None)
        checks['cancel_does_not_send_motion']=not result['passed'] and not fixture.active
    finally:
        fixture.velocity=[0.]*14
        if child:
            child.process.stdin.close();child.process.wait(timeout=15);child.close()
        fixture.close()
    report=dict(passed=all(checks.values()),checks=checks,visual_inspection=False,
        scope='private localhost injected SDK feedback; fixed q, not physical zero accuracy or dynamics evidence')
    (root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
