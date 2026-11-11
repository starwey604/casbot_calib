#!/usr/bin/env python3
"""Actual ImGui wait/cancel/recovery and original operator-fault reproduction, localhost only."""
import argparse
import json
import os
from pathlib import Path
from threading import Thread
import time
import uuid

import yaml


def screenshot(path):
    from PIL import ImageGrab
    # Capture the displayed X window, not GL_FRONT (black on this Xvfb backend).
    ImageGrab.grab(bbox=(0,0,1440,980),xdisplay=os.environ['DISPLAY']).save(path)


class ProtocolFixture:
    """Fixed q with injected dq: protocol state-machine evidence, NOT mechanical evidence."""
    def __init__(self,root,domain):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import SetBool
        from crb_ros_msg.msg import UpperJointData
        from casbot_calib.hardware_field import load_profile,model_for
        from casbot_calib.sdk_connection import mock_connection
        from casbot_calib.vendor_identity import sha256
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);self.rclpy=rclpy
        self.endpoint='/casbot_calib_s2_r1_'+uuid.uuid4().hex[:10]
        config=load_profile('configs/hardware_field.yaml');engine,_=model_for(config)
        self.names=[j.name for j in engine.model.joints]
        self.q=yaml.safe_load(Path('configs/sdk_mock.yaml').read_text())['initial_q']
        self.velocity=[]  # Initially incomplete feedback; no valid dq is invented.
        self.active=False;self.commands=0;self.calls=0;self.last_command=0
        self.node=rclpy.create_node('fieldwork_protocol_fixture',namespace=self.endpoint,
                                    enable_rosout=False,start_parameter_services=False)
        self.pub=self.node.create_publisher(JointState,self.endpoint+'/joint_states',10)
        self.mode=self.node.create_publisher(String,self.endpoint+'/motion/status',10)
        def command(msg):self.commands+=1;self.last_command=time.monotonic()
        def mode(req,resp):
            self.calls+=1;self.active=req.data;self.last_command=time.monotonic();resp.success=True;return resp
        self.sub=self.node.create_subscription(UpperJointData,self.endpoint+'/upper_body_debug/joint_cmd',command,10)
        self.mode_handler=mode;self.service=None
        def publish():
            if self.active and time.monotonic()-self.last_command>.1:self.active=False
            msg=JointState();msg.header.stamp=self.node.get_clock().now().to_msg();msg.name=self.names
            msg.position=self.q;msg.velocity=self.velocity;msg.effort=[0.]*14;self.pub.publish(msg)
            status=String();status.data='mode: UPPER_BODY_DEBUG, state: STAND' if self.active else 'mode: READY, state: READY';self.mode.publish(status)
        self.timer=self.node.create_timer(.01,publish)
        self.running=True
        def spin():
            while self.running:rclpy.spin_once(self.node,timeout_sec=.01)
        self.thread=Thread(target=spin);self.thread.start()
        root.mkdir();descriptor=root/'fixture.json'
        descriptor.write_text(json.dumps(dict(scope='protocol fixture, fixed position / injected velocity; no mechanics',endpoint=self.endpoint)))
        config.pop('scene');config.update(collision_scene=str(Path('configs/collision_scene.yaml').resolve()),
              connection=mock_connection(self.endpoint,domain,sha256(descriptor)))
        self.client_profile=root/'profile.yaml';self.client_profile.write_text(yaml.safe_dump(config))
    def enable_service(self):
        from std_srvs.srv import SetBool
        self.service=self.node.create_service(SetBool,self.endpoint+'/motion/upper_body_debug',self.mode_handler)
    def close(self):
        self.running=False;self.thread.join(timeout=2);self.node.destroy_node();self.rclpy.shutdown()


def create_desk(profile,root,domain):
    from casbot_calib.fieldwork_control import create_session
    create_session(profile,root,domain,'localhost_contract_mock',workspace=True)
    return argparse.Namespace(session=root,output=None,preview=False,profile=profile,domain_id=None,scope=None,
                              frames=100000,screenshot=root.parent/'window.png')


def run_case(args,profile,domain,fixture=None):
    from casbot_calib import fieldwork_gui
    from casbot_calib.fieldwork_control import Session,Workspace
    from imgui_bundle import imgui
    ui=create_desk(profile,args.output/'desk',domain);original=fieldwork_gui.Desk.gui
    trace=dict(stage='connect',at=0,started=time.monotonic(),checks={},actions=[],events=[])
    queue=[]
    def stage(value):trace['stage']=value;trace['at_time']=time.monotonic()
    def click(desk,key):
        frame=desk.frame;queue.extend([(frame+1,'pos',desk.controls[key]),(frame+2,'down',None),(frame+4,'up',None)])
        trace['at']=frame;trace['actions'].append(key)
    def draw(desk):
        original(desk);io=imgui.get_io()
        for event in list(queue):
            frame,kind,value=event
            if frame<=desk.frame:
                if kind=='pos':io.add_mouse_pos_event(*value)
                else:io.add_mouse_button_event(0,kind=='down')
                queue.remove(event)
        if time.monotonic()-trace['started']>95:
            trace['timeout']=trace['stage'];ui.frames=min(ui.frames,desk.frame+1);return
        event=desk.operator.current
        if event and (not trace['events'] or (trace['events'][-1]['code'],trace['events'][-1].get('phase'))!=(event['code'],event.get('phase'))):trace['events'].append(event)
        step=trace['stage'];dt=desk.frame-trace['at'];elapsed=time.monotonic()-trace.get('at_time',trace['started'])
        checks=trace['checks']
        if step=='connect' and desk.frame>8:click(desk,'connect');stage('observe')
        elif step=='observe' and desk.commands is not None and dt>8:
            click(desk,'send');stage('wait_missing' if fixture else 'fault')
        elif fixture:
            if step=='wait_missing' and event and event['code']=='waiting_feedback' and elapsed>.7:
                checks['incomplete_feedback_waits_without_control']=fixture.calls==0 and fixture.commands==0 and desk.busy
                fixture.velocity=[0.,.03]+[0.]*5+[-.04]+[0.]*6;stage('wait_first')
            elif step=='wait_first' and event and event['code']=='waiting_stationary':stage('long_wait')
            elif step=='long_wait' and elapsed>10.5:
                checks['wait_beyond_old_10s']=desk.busy and desk.pending_start
                checks['all_waiting_axes_explained']=len(event['observations'])==2 and len(event['details'])==2
                checks['no_mode_or_commands_before_rest']=fixture.calls==0 and fixture.commands==0
                screenshot(args.output/'waiting.png');click(desk,'stop');stage('cancel')
            elif step=='cancel' and not desk.busy:
                fixture.velocity=[0.]*14;trace['cancel_receive_ns']=desk.receive_ns;stage('after_cancel')
            elif step=='after_cancel' and elapsed>.6:
                checks['cancel_cannot_start_later']=fixture.calls==0 and fixture.commands==0
                checks['observe_after_cancel']=desk.observing and desk.receive_ns>trace['cancel_receive_ns']
                click(desk,'refresh');stage('new_start')
            elif step=='new_start' and dt>8:
                fixture.velocity=[0.,.03]+[0.]*12;click(desk,'send');stage('wait_second')
            elif step=='wait_second' and event and event['code']=='waiting_stationary':
                fixture.velocity=[0.]*14;stage('wait_service')
            elif step=='wait_service' and event and event['code']=='waiting_mode_service' and elapsed>.5:
                checks['undiscovered_service_waits_without_rpc']=fixture.calls==0 and fixture.commands==0 and desk.busy
                fixture.enable_service();stage('auto_continue')
            elif step=='auto_continue' and desk.sending and fixture.commands>30:
                checks['automatic_continue_without_second_start']=fixture.calls==1
                checks['new_control_session_not_resume']=len(desk.control_roots)==2
                click(desk,'stop');stage('stop_second')
            elif step=='stop_second' and not desk.busy:
                import resource
                # Limit ONLY this test's readonly recorder. Its explicitly
                # tolerant live mode converts EFBIG into a recording notice.
                observations=list((args.output/'desk/observations').glob('*/capture/raw'))
                raw=observations[-1].resolve();wanted='output:='+str(raw)
                matches=[]
                for path in Path('/proc').glob('[0-9]*/cmdline'):
                    try:fields=path.read_bytes().decode().split('\0')
                    except (OSError,UnicodeError):continue
                    if fields and fields[0].endswith('/state_recorder') and wanted in fields:matches.append(int(path.parent.name))
                assert len(matches)==1,matches
                threshold=max(p.stat().st_size for p in raw.iterdir() if p.is_file())+8192
                _,hard=resource.prlimit(matches[0],resource.RLIMIT_FSIZE)
                resource.prlimit(matches[0],resource.RLIMIT_FSIZE,(threshold,hard))
                trace['before_io_receive_ns']=desk.receive_ns;stage('io_failure')
            elif step=='io_failure' and desk.recording_error and desk.receive_ns>trace['before_io_receive_ns'] and elapsed>.5:
                checks['recording_failure_visible']=True;checks['live_feedback_survives_disk_error']=desk.observing
                screenshot(args.output/'recording_error.png');ui.frames=desk.frame+5;stage('done')
        else:
            if step=='fault' and desk.operator.first_fault and not desk.busy:
                trace['fault']=desk.operator.first_fault;trace['after_fault_ns']=desk.receive_ns;stage('after_fault')
            elif step=='after_fault' and elapsed>1.:
                checks['original_fault_not_overwritten']=desk.operator.displayed['code']=='observed_limit'
                checks['axis_and_threshold_visible']=bool(desk.operator.displayed['details'] and all('阈值' in d or '限' in d for d in desk.operator.displayed['details']))
                checks['feedback_continues_after_control_fault']=desk.observing and desk.receive_ns>trace['after_fault_ns']
                checks['control_fault_stays_terminal']=bool(Session(desk.control_root).status()['blocked_reason'])
                checks['workspace_still_usable']=Workspace(desk.root).status()['blocked_reason'] is None
                screenshot(args.output/'fault.png');ui.frames=desk.frame+5;stage('done')
    fieldwork_gui.Desk.gui=draw
    try:fieldwork_gui.run_gui(ui)
    finally:fieldwork_gui.Desk.gui=original
    trace['checks']['completed_without_timeout']='timeout' not in trace
    controls=Workspace(args.output/'desk').status()['controls']
    trace['controls']=controls
    return trace


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scenario',choices=('waiting','fault'),default='waiting')
    args=parser.parse_args();args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    domain=180+int(uuid.uuid4().hex[:2],16)%30
    os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1')
    if args.scenario=='waiting':
        fixture=ProtocolFixture(args.output/'fixture',domain)
        try:trace=run_case(args,fixture.client_profile,domain,fixture)
        finally:fixture.close()
        scope='actual ImGui + localhost SDK protocol fixture; no mechanical claims'
    else:
        from casbot_calib.sdk_mock import SDKMockSession
        with SDKMockSession('configs/sdk_mock.yaml',args.output/'mock',domain) as mock:
            trace=run_case(args,mock.client_profile,domain)
        scope='actual ImGui + native SDK mechanical Mock; zero front-feedforward original startup failure'
    result=dict(passed=all(trace['checks'].values()),scenario=args.scenario,checks=trace['checks'],trace=trace,scope=scope,physical_robot_tested=False)
    (args.output/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
