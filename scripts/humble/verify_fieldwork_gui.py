#!/usr/bin/env python3
"""Exercise real ImGui sliders against private SDK Mock and preserve motion evidence."""
import argparse
import csv
import json
import os
from pathlib import Path
import time
import uuid

import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scenario',choices=('sliders','close','heartbeat'),default='sliders')
    args=parser.parse_args(); root=args.output.resolve(); root.mkdir(parents=True,exist_ok=False)
    domain=180+int(uuid.uuid4().hex[:2],16)%30
    os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1')
    from casbot_calib.sdk_mock import SDKMockSession
    from casbot_calib.fieldwork_control import create_session,Session,Workspace
    from casbot_calib.fieldwork_analysis import read_json,write_json
    from casbot_calib import fieldwork_gui
    from imgui_bundle import imgui
    with SDKMockSession('configs/sdk_mock.yaml',root/'mock',domain) as mock:
        create_session(mock.client_profile,root/'desk',domain,'localhost_contract_mock',workspace=True)
        ui=argparse.Namespace(session=root/'desk',output=None,preview=False,profile=mock.client_profile,
            domain_id=None,scope=None,frames=20000,screenshot=root/'window.png')
        original=fieldwork_gui.Desk.gui
        state=dict(stage='connect',at=0,start=time.monotonic(),actions=[],checks={},frames_seen=0)
        pending=[]
        def click(desk,key,ratio=None):
            if ratio is None: point=desk.controls[key]
            else:
                a,b,c,d=desk.control_bounds[key]; point=(a+(c-a)*ratio,(b+d)/2)
            frame=desk.frame
            pending.extend([(frame+1,'pos',point),(frame+2,'down',None),(frame+4,'up',None)])
            state['actions'].append(dict(key=key,ratio=ratio,frame=frame));state['at']=frame
        def drag(desk,key,field,delta,lo,hi):
            a,b,c,d=desk.control_bounds[key]
            grab=imgui.get_style().grab_min_size
            usable=(c-a)-4-grab
            value=float(desk.commands[field][4])
            x=a+2+grab/2+usable*(value-lo)/(hi-lo)
            end=x+usable*delta/(hi-lo)
            frame=desk.frame
            pending.extend([(frame+1,'pos',(x,(b+d)/2)),(frame+2,'down',None),
                            (frame+4,'pos',(end,(b+d)/2)),(frame+6,'up',None)])
            state['actions'].append(dict(key=key,delta=delta,frame=frame));state['at']=frame
        def stage(name): state['stage']=name;state['at_time']=time.monotonic()
        def draw(desk):
            original(desk)
            io=imgui.get_io()
            for item in list(pending):
                frame,kind,value=item
                if frame<=desk.frame:
                    if kind=='pos': io.add_mouse_pos_event(*value)
                    else: io.add_mouse_button_event(0,kind=='down')
                    pending.remove(item)
            if time.monotonic()-state['start']>600:
                state['timeout']=state['stage'];ui.frames=min(ui.frames,desk.frame+1);return
            step=state['stage'];dt=desk.frame-state['at']
            if step=='connect' and desk.frame>8:
                click(desk,'connect');stage('feedback')
            elif step=='feedback' and desk.commands is not None and desk.feedback['kp'] is not None and dt>8:
                state['first_q']=desk.first_q.tolist();state['initial_commands']={k:v.tolist() for k,v in desk.commands.items()}
                state['checks']['first_q_initialization']=np.array_equal(desk.commands['position'],desk.first_q)
                state['checks']['kp_readback_initialization']=np.array_equal(desk.commands['kp'],desk.feedback['kp'])
                state['checks']['effort_not_feedback_copy']=bool(np.all(desk.commands['effort']==0))
                click(desk,'gravity');stage('gravity')
            elif step=='gravity' and dt>10:
                state['checks']['visible_gravity_fill']=bool(np.max(abs(desk.commands['effort']))>1)
                click(desk,'send');stage('mode')
            elif step=='mode' and desk.sending and dt>35:
                if args.scenario=='close':
                    state['checks']['close_requested']=True;ui.frames=desk.frame+1;stage('closing')
                elif args.scenario=='heartbeat':
                    state['before_freeze_receive_ns']=desk.receive_ns
                    state['checks']['freeze_triggered']=True;time.sleep(3.5);stage('stopping')
                else:
                    state['view_before']=desk.rendered_frames
                    drag(desk,'position_4','position',.04,float(desk.lower[4]),float(desk.upper[4]));stage('q')
            elif step=='q' and dt>12:
                state['q_after']=None if desk.q is None else desk.q.tolist()
                state['checks']['wrist_feedback_moved']=bool(desk.q is not None and abs(desk.q[4]-state['first_q'][4])>.01)
                state['checks']['live_view_updated']=desk.rendered_frames>state['view_before']
                drag(desk,'velocity','velocity',.02,-.75,.75);stage('velocity')
            elif step=='velocity' and dt>12:
                drag(desk,'kp','kp',15.,0,1000);stage('kp')
            elif step=='kp' and dt>12:
                drag(desk,'kd','kd',.2,0,100);stage('kd')
            elif step=='kd' and dt>12:
                drag(desk,'effort','effort',.05,-10,10);stage('effort')
            elif step=='effort' and dt>20:
                state['tuned']={k:v.tolist() for k,v in desk.commands.items()}
                click(desk,'short');stage('short')
            elif step=='short' and time.monotonic()-state['at_time']>desk.short_duration+.5 and desk.short is None:
                click(desk,'stop');stage('stopping')
            elif step=='stopping' and not desk.busy:
                state['response']=desk.response
                if args.scenario=='heartbeat':
                    state['checks']['heartbeat_cause_visible']=bool(desk.operator.displayed and desk.operator.displayed['code']=='frontend_heartbeat_timeout')
                    state['checks']['readonly_survives_ui_stall']=desk.observing and desk.receive_ns>state['before_freeze_receive_ns']
                if args.scenario=='sliders': click(desk,'video');stage('video')
                else: ui.frames=desk.frame+3;stage('done')
            elif step=='video' and not desk.busy and desk.response and desk.response['action']=='video':
                state['checks']['video_exported']=desk.response['completed']
                state['video']=desk.response;ui.frames=desk.frame+5;stage('done')
            state['frames_seen']=desk.rendered_frames
        fieldwork_gui.Desk.gui=draw
        try: fieldwork_gui.run_gui(ui)
        finally: fieldwork_gui.Desk.gui=original
        workspace_status=Workspace(root/'desk').status();runs=list((root/'desk').glob('controls/*/runs/*'))
        status=Session(runs[-1].parents[1]).status() if runs else workspace_status
        checks=state['checks']
        commands=[];feedback=[];capture_result={}
        if runs:
            capture=runs[-1]/'capture';capture_result=read_json(capture/'result.json') or {}
            with (capture/'execution/commands.csv').open() as stream: commands=list(csv.DictReader(stream))
            with (capture/'execution/feedback.csv').open() as stream: feedback=list(csv.DictReader(stream))
            with (capture/'execution/events.csv').open() as stream: events=list(csv.DictReader(stream))
            stops=[int(row['receive_ns']) for row in events if row['state']=='STOPPING']
            checks['no_commands_after_stop']=bool(stops and all(int(row['publish_ns'])<stops[0] for row in commands))
            checks['recorded_raw_and_readback']=bool((runs[-1]/'recording/readback.csv').exists() and (capture/'raw_state/raw/cdr.bin').stat().st_size>0)
            if args.scenario=='sliders':
                for key,column in [('position','q4'),('velocity','v4'),('effort','effort4'),('kp','kp4'),('kd','kd4')]:
                    values=np.array([float(row[column]) for row in commands])
                    checks['published_'+key+'_changed']=bool(len(values)>0 and np.ptp(values)>1e-4)
                checks['normal_stop_no_fault_lock']=status['blocked_reason'] is None
                checks['short_triggered']=any(row['key']=='short' for row in state['actions']) and len(commands)>2000
                checks['same_sdk_five_field_shape']=all(len([k for k in row if k.startswith('q')])==14 for row in commands)
            elif args.scenario=='heartbeat': checks['heartbeat_terminal']=bool(status['blocked_reason'] and capture_result.get('supervisor_interruption')=='frontend_heartbeat_timeout')
            else: checks['close_controlled']=bool(status['blocked_reason'] is None and capture_result.get('supervisor_interruption')=='frontend_closed')
        checks['actual_sdk_commands']=len(commands)>30
        checks['completed_without_timeout']='timeout' not in state
        # Inspect public commands as received by the SDK Mock, not only the sender log.
        with (root/'mock/evaluation/server/server_commands.csv').open() as stream: received=list(csv.DictReader(stream))
        checks['mock_received_commands']=len(received)>30
        if args.scenario=='sliders':
            for field in ('q4','v4','effort4'):
                checks['SDK_received_'+field+'_changed']=bool(received and np.ptp([float(row[field]) for row in received])>1e-4)
            with (root/'mock/evaluation/plant/device_commands.csv').open() as stream:
                applied=list(csv.DictReader(stream))
            for field in ('kp4','kd4'):
                checks['native_applied_'+field+'_changed']=bool(applied and np.ptp([float(row[field]) for row in applied])>1e-4)
        result=dict(passed=all(checks.values()),scenario=args.scenario,checks=checks,trace=state,
            sent_count=len(commands),received_count=len(received),feedback_count=len(feedback),capture_result=capture_result,
            scope='Xvfb + private localhost SDK Mock; representative left wrist yaw',physical_robot_tested=False)
        write_json(root/'verification.json',result);print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
