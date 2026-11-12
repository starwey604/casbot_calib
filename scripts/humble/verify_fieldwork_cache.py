#!/usr/bin/env python3
"""Exercise auto-cache/owned Mock, live GUI save, quota and stale-target recovery."""
import argparse
import csv
import json
import os
from pathlib import Path
from threading import Thread
import time
import uuid


def crash_case(root,domain):
    import subprocess,sys
    command=[sys.executable,'-m','casbot_calib.fieldwork_cli','gui','--mock','--mock-diagnostics','--domain-id',str(domain),'--cache-limit-mib','32']
    with (root/'gui.log').open('x') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+45;frame=None
            while time.monotonic()<deadline:
                frames=list((root/'cache').glob('sessions/*/workspace/view/rendered.json'))
                if frames:frame=frames[0];break
                if child.poll() is not None:raise RuntimeError('GUI提前退出，查看gui.log')
                time.sleep(.05)
            if frame is None:raise RuntimeError('未得到GUI实际渲染帧')
            cache=frame.parents[2]
            import signal
            child.send_signal(signal.SIGSTOP)
            deadline=time.monotonic()+20
            while not (cache/'recording_paused.json').exists() and time.monotonic()<deadline:time.sleep(.05)
            quota_survives_pause=(cache/'recording_paused.json').exists()
            time.sleep(.3)
            sizes={p:p.stat().st_size for p in (cache/'mock').glob('evaluation/**/*.csv')};time.sleep(1)
            stopped_append=all(p.stat().st_size==size for p,size in sizes.items())
            child.kill();child.wait(timeout=5)
            deadline=time.monotonic()+15
            while not (cache/'mock/shutdown.json').exists() and time.monotonic()<deadline:time.sleep(.05)
            from casbot_calib.fieldwork_storage import cached_sessions,recover_workspace
            checks=dict(quota_monitor_survives_sigstop=quota_survives_pause,diagnostics_stop_without_gui_pulses=stopped_append,owned_mock_closes_after_parent_kill=(cache/'mock/shutdown.json').exists()
                and json.loads((cache/'mock/shutdown.json').read_text())['returncodes']==[0],
                cache_survives_parent_kill=(cache/'workspace/protocol/session.json').exists(),
                crashed_cache_is_recoverable=recover_workspace(cache.name)==cache/'workspace')
        finally:
            if child.poll() is None:child.kill();child.wait(timeout=5)
    result=dict(passed=all(checks.values()),checks=checks,scope='SIGKILL only our GUI; EOF cleanup only its owned localhost Mock',physical_robot_tested=False)
    (root/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['passed'] else 1


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scenario',choices=('workspace','crash'),default='workspace')
    args=p.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    domain=190+int(uuid.uuid4().hex[:2],16)%20
    os.environ.update(CASBOT_FIELDWORK_CACHE=str(root/'cache'),ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=str(domain))
    if args.scenario=='crash':return crash_case(root,domain)
    from casbot_calib.fieldwork_cli import parser
    from casbot_calib import fieldwork_gui
    from casbot_calib.fieldwork_storage import cache_for,cache_home
    from casbot_calib.fieldwork_control import Workspace
    from imgui_bundle import imgui,hello_imgui
    from PIL import ImageGrab
    checks={};trace={};original=fieldwork_gui.Desk.gui
    class CommandObserver:
        def __init__(self,endpoint):
            import rclpy
            from rclpy.signals import SignalHandlerOptions
            from crb_ros_msg.msg import UpperJointData
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO);self.rclpy=rclpy
            self.node=rclpy.create_node('cache_command_observer',enable_rosout=False,start_parameter_services=False)
            self.count=0
            def receive(msg):self.count+=1
            self.sub=self.node.create_subscription(UpperJointData,endpoint+'/upper_body_debug/joint_cmd',receive,10)
            self.running=True
            def run():
                while self.running:rclpy.spin_once(self.node,timeout_sec=.02)
            self.thread=Thread(target=run);self.thread.start()
        def close(self):self.running=False;self.thread.join(timeout=2);self.node.destroy_node();self.rclpy.shutdown()
    def shot(name):ImageGrab.grab(bbox=(0,0,1440,980),xdisplay=os.environ['DISPLAY']).save(root/(name+'.png'))
    def run(ui,scenario):
        state=dict(step='connect' if scenario!='snapshot' else 'snapshot',at=time.monotonic(),start=time.monotonic());queue=[];counter=None
        def stage(name):state['step']=name;state['at']=time.monotonic()
        def click(desk,key):
            f=desk.frame;queue.extend([(f+1,'pos',desk.controls[key]),(f+2,'down',None),(f+4,'up',None)])
        def draw(desk):
            nonlocal counter
            original(desk)
            if not trace.get('workspace') and scenario=='live':
                trace['workspace']=str(desk.root);trace['cache']=str(cache_for(desk.root))
                counter=CommandObserver(Workspace(desk.root).config['connection']['endpoint_root'])
            io=imgui.get_io()
            for item in list(queue):
                frame,kind,value=item
                if desk.frame>=frame:
                    if kind=='pos':io.add_mouse_pos_event(*value)
                    else:io.add_mouse_button_event(0,kind=='down')
                    queue.remove(item)
            elapsed=time.monotonic()-state['at'];step=state['step']
            if time.monotonic()-state['start']>75:
                checks[scenario+'_timeout']=False;ui.frames=desk.frame+1;return
            if scenario=='snapshot':
                if desk.frame>70 and step!='done' and desk.q is not None and desk.rendered_frames>0:
                    checks['saved_snapshot_is_readonly']=desk.read_only and not desk.observing and not desk.sending
                    checks['saved_feedback_is_displayed']=desk.q is not None and desk.rendered_frames>0
                    checks['saved_commands_are_restored']=desk.commands is not None and desk.first_q is not None
                    shot('snapshot');ui.frames=desk.frame+1;stage('done')
                return
            if step=='connect' and desk.frame>8:
                if scenario=='recover':checks['recovery_does_not_auto_connect']=not desk.observing and desk.q is not None
                click(desk,'connect');stage('feedback')
            elif scenario=='recover' and step=='feedback' and elapsed>3 and desk.connection_channels.get('joint',{}).get('publishers')==0:
                checks['stale_profile_has_zero_publishers']=desk.receive_ns is None and not desk.sending
                checks['actual_stale_endpoint_visible']=desk.connection_channels['joint']['topic'].startswith(Workspace(desk.root).config['connection']['endpoint_root'])
                shot('stale_target');click(desk,'recover_menu');stage('recover_menu')
            elif scenario=='recover' and step=='recover_menu' and 'recover_'+trace['other_cache_id'] in desk.controls:
                click(desk,'recover_'+trace['other_cache_id']);stage('reopen')
            elif scenario=='recover' and step=='reopen' and str(desk.root)==trace['other_workspace']:
                checks['gui_cache_menu_opens_another_workspace']=not desk.observing and not desk.sending
                ui.frames=desk.frame+1;stage('done')
            elif scenario=='live':
                if step=='feedback' and elapsed>.6 and desk.commands is not None and desk.feedback['kp'] is not None and desk.connection_channels.get('joint',{}).get('publishers')==1:
                    checks['automatic_mock_profile_matches']=desk.connection_channels.get('joint',{}).get('publishers')==1
                    checks['gravity_control_removed']='gravity' not in desk.controls
                    click(desk,'send');stage('sending')
                elif step=='sending' and desk.sending and counter.count>100:
                    trace['before_save_commands']=counter.count;trace['before_save_feedback']=desk.receive_ns
                    click(desk,'save_session');stage('save_form')
                elif step=='save_form' and desk.save_open and elapsed>.3:
                    desk.save_path=str(root/'saved_live');click(desk,'save_apply');stage('saved')
                elif step=='saved' and (root/'saved_live/saved_session.json').exists() and desk.save_job is None:
                    checks['save_while_sending']=desk.sending and counter.count>trace['before_save_commands'] and desk.receive_ns>trace['before_save_feedback']
                    checks['saved_protocol_valid']=Workspace(root/'saved_live').status()['schema_version']=='casbot_fieldwork_workspace_v1'
                    checks['comparison_files_generated_on_save']=bool(list((root/'saved_live').glob('controls/*/runs/*/recording/readback.csv')))
                    shot('live_saved')
                    # Explicit test injection: lower ONLY this cache's monitor quota.
                    desk.cache_monitor.limit=desk.cache_monitor.used+512*1024;stage('quota')
                elif step=='quota' and desk.cache_monitor.paused:
                    trace['paused_commands']=counter.count;trace['paused_feedback']=desk.receive_ns
                    trace['paused_sizes']={str(p.relative_to(desk.root)):p.stat().st_size for p in desk.root.glob('**/packets.jsonl')}
                    stage('after_quota')
                elif step=='after_quota' and elapsed>2.:
                    checks['quota_keeps_control_and_feedback']=desk.sending and counter.count>trace['paused_commands']+100 and desk.receive_ns>trace['paused_feedback']
                    checks['quota_stops_raw_append']=all((desk.root/name).stat().st_size==size for name,size in trace['paused_sizes'].items())
                    checks['quota_visible']=bool(desk.recording_error and desk.short is None)
                    shot('capacity');click(desk,'stop');stage('stop')
                elif step=='stop' and not desk.busy:
                    checks['normal_stop_after_quota']=not desk.operator.first_fault and desk.observing
                    click(desk,'save_session');stage('save_paused_form')
                elif step=='save_paused_form' and desk.save_open and elapsed>.3:
                    desk.save_path=str(root/'saved_paused');click(desk,'save_apply');stage('save_paused')
                elif step=='save_paused' and (root/'saved_paused/saved_session.json').exists() and desk.save_job is None:
                    checks['repeated_save_reports_success']=desk.save_message.startswith('会话已保存')
                    checks['capacity_preserved_in_archive']=json.loads((root/'saved_paused/saved_session.json').read_text())['recording_capacity_reached']
                    ui.frames=desk.frame+2;stage('done')
        fieldwork_gui.Desk.gui=draw
        try:fieldwork_gui.run_gui(ui)
        finally:
            fieldwork_gui.Desk.gui=original
            if counter:counter.close()
        checks[scenario+'_completed']=state['step']=='done'
    live=parser().parse_args(['gui','--mock','--domain-id',str(domain),'--frames','20000'])
    run(live,'live')
    cache=Path(trace['cache']);workspace=Path(trace['workspace'])
    checks['owned_mock_stopped_on_close']=json.loads((cache/'mock/shutdown.json').read_text())['returncodes']==[0]
    checks['cache_retained_after_close']=workspace.exists()
    # Decode every archived CDR row independently against its stored message.
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from crb_ros_msg.msg import JointStateData
    count=0
    for packets in (root/'saved_live').glob('**/packets.jsonl'):
        data=packets.with_name('cdr.bin').read_bytes()
        for line in packets.read_text().splitlines():
            packet=json.loads(line)
            if packet['channel'] not in ('joint','debug') or packet['cdr_offset'] is None:continue
            offset=packet['cdr_offset'];size=packet['cdr_size'];assert offset+size<=len(data)
            message=deserialize_message(data[offset:offset+size],JointState if packet['channel']=='joint' else JointStateData)
            assert list(message.name)==packet['message']['name']
            assert list(message.position)==packet['message']['position'];count+=1
    checks['saved_cdr_independently_decodes']=count>100;trace['decoded_packets']=count
    from casbot_calib.fieldwork_storage import new_cache
    from casbot_calib.fieldwork_control import create_session
    other=new_cache();create_session(cache/'mock/client_profile.yaml',other/'workspace',domain,'localhost_contract_mock',workspace=True)
    trace['other_cache_id']=other.name;trace['other_workspace']=str(other/'workspace')
    recover=parser().parse_args(['gui','--recover',cache.name,'--frames','10000'])
    run(recover,'recover')
    snapshot=parser().parse_args(['gui','--session',str(root/'saved_live'),'--frames','10000'])
    run(snapshot,'snapshot')
    result=dict(passed=all(checks.values()),checks=checks,trace=trace,physical_robot_tested=False,
        scope='actual ImGui + GUI-owned private localhost SDK Mock; public command observer and independent CDR decode')
    (root/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
