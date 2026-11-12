#!/usr/bin/env python3
"""Nonvisual protocol test: silent mode topic must still terminate manual control."""
import argparse
import csv
import json
import os
from pathlib import Path
from types import SimpleNamespace
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--domain-id',type=int,required=True)
    args=p.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1',CASBOT_FIELDWORK_CACHE=str(root/'cache'))
    from verify_fieldwork_round1 import ProtocolFixture
    from casbot_calib import fieldwork_gui as gui
    from casbot_calib.fieldwork_control import create_session
    fixture=ProtocolFixture(root/'fixture',args.domain_id);fixture.velocity=[0.]*14;fixture.enable_service()
    create_session(fixture.client_profile,root/'workspace',args.domain_id,'localhost_contract_mock',workspace=True)
    ui=argparse.Namespace(mock=False,session=root/'workspace',recover=None,preview=False,output=None,
        profile=fixture.client_profile,domain_id=None,scope=None,frames=None,screenshot=None)
    original=gui.Desk.gui;state={'phase':'connect','start':time.monotonic()};checks={};publisher=fixture.mode
    def draw(desk):
        original(desk);now=time.monotonic()
        if now-state['start']>40:raise RuntimeError('mode silence verification timeout: '+state['phase'])
        if state['phase']=='connect':desk.connect();state['phase']='feedback'
        elif state['phase']=='feedback' and desk.connected:
            desk.begin_sending();state['phase']='sending'
        elif state['phase']=='sending' and desk.sending:
            checks['default_mode_timeout_3s']=desk.limits.pinned['general']['mode_timeout_s']==3.
            state['run']=desk.control_root/desk.active_run
            fixture.mode=SimpleNamespace(publish=lambda _:None);state['silenced']=now;state['receive']=desk.receive_ns;state['phase']='fault'
        elif state['phase']=='fault' and not desk.busy:
            fault=desk.operator.first_fault or {}
            checks.update(real_silence_terminates=fault.get('code')=='stale_mode',no_automatic_resume=not desk.sending and desk.needs_reinitialize,
                joint_feedback_continues=desk.receive_ns>state['receive'],observer_continues=desk.observing,
                threshold_reported=any(x.get('threshold')==3000 for x in fault.get('observations',[])))
            state['fault']=fault;state['phase']='done';ui.frames=desk.frame+1
    gui.Desk.gui=draw
    try:gui.run_gui(ui)
    finally:gui.Desk.gui=original;fixture.mode=publisher;fixture.close()
    run=state['run'];result=json.loads((run/'capture/execution/result.json').read_text())
    commands=list(csv.DictReader((run/'capture/execution/commands.csv').open()));events=list(csv.DictReader((run/'capture/execution/events.csv').open()))
    stop=next(int(row['receive_ns']) for row in events if row['state']=='STOPPING')
    checks.update(new_async_path=result['execution_recording_mode']=='bounded_async_single_producer',
        no_commands_after_stop=all(int(row['publish_ns'])<stop for row in commands),recording_complete=not result['recording_paused'])
    report=dict(passed=all(checks.values()),checks=checks,fault=state['fault'],scope='fixed-state private protocol fixture; no physics/physical robot',visual_inspection=False)
    (root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
