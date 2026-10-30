#!/usr/bin/env python3
"""Run actual hardware-path code against isolated ROS protocol fixtures only."""
import argparse
import json
import os
from pathlib import Path
from threading import Thread
import time
import uuid
import numpy as np
import yaml

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    domain=220+int(uuid.uuid4().hex[:2],16)%10
    os.environ['ROS_DOMAIN_ID']=str(domain);os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from crb_ros_msg.msg import UpperJointData
    from casbot_calib.data import write_json
    from casbot_calib.dual_model import nominal_dual_arm
    from casbot_calib.static_field import paired_motions
    from casbot_calib.hardware_field import load_profile,run_executor,prepare_jog,prepare_field,execute_field,analyse_field,seal
    e,source=nominal_dual_arm();names=[j.name for j in e.model.joints]
    preset=yaml.safe_load(Path('configs/static_field_nominal.yaml').read_text())
    preset['approach_offset_rad']=.001
    preset['timing']=dict(initial_hold_s=.05,minimum_transition_s=.1,minimum_paired_approach_s=.05,
                         settle_s=.1,measure_s=.2,final_hold_s=.05)
    home=np.array(preset['home_rad']);ref=root/'fixture_reference';ref.mkdir();candidates=[];order=[]
    for k,split in enumerate(('train','validation')):
        q=home.copy();q[0]+=.006 if k==0 else -.006
        row=dict(id=f'fixture_{k}',arm='left',status='selected',split=split,q_rad=q.tolist(),
                 family='single_axis',active_joints=[0],offsets_deg=[1 if k==0 else -1])
        candidates.append(row)
        for motion in paired_motions(row,preset,split=split):
            folder=ref/'plans'/motion.name;folder.mkdir(parents=True)
            write_json(folder/'motion.json',motion.metadata());order.append(motion.name)
    write_json(ref/'config.json',preset);write_json(ref/'candidates.json',candidates)
    scene=yaml.safe_load(Path('configs/collision_scene.yaml').read_text());write_json(ref/'scene.json',scene)
    write_json(ref/'report.json',dict(passed=True,source_model_sha256=source['sha256'],joint_names=names,
        planned_execution_order=order,scope='small ROS contract fixture; not static observability design'))
    seal(ref)
    cfg=yaml.safe_load(Path('configs/hardware_field.yaml').read_text())
    cfg.update(collision_scene=str(Path('configs/collision_scene.yaml').resolve()),anchor_transition_s=.1,
               initial_hold_s=.05,final_hold_s=.05)
    config_path=root/'hardware.yaml';config_path.write_text(yaml.safe_dump(cfg));config=load_profile(config_path)
    prefix='/casbot_calib_s2_field_'+uuid.uuid4().hex[:8]
    remaps={'/joint_states':prefix+'/joint_states','/motion/status':prefix+'/status',
            '/upper_body_debug/joint_cmd':prefix+'/upper_cmd','/motion/upper_body_debug':prefix+'/upper_debug'}
    rclpy.init();node=rclpy.create_node('field_protocol_fixture',enable_rosout=False,start_parameter_services=False)
    feedback=node.create_publisher(JointState,remaps['/joint_states'],10)
    status=node.create_publisher(String,remaps['/motion/status'],10)
    state=dict(q=home.copy(),v=np.zeros(14),target=home.copy(),previous=home.copy(),begin=time.monotonic(),
               cache_at=0.,active=False,received=0,mode_calls=0,watchdog=False,drop=False,fail_after=None,duplicates=0,last_q=None)
    def upper(msg):
        if not state['active']:return
        if list(msg.name)!=names or len(msg.position)!=14 or any(msg.effort) or msg.kp or msg.kd:
            raise RuntimeError('Hardware executor emitted wrong subset/effort/PD')
        state['previous']=state['q'].copy();state['target']=np.array(msg.position)
        state['begin']=time.monotonic();state['received']+=1
    def mode(req,resp):
        state['active']=req.data;state['mode_calls']+=1;state['begin']=time.monotonic();resp.success=True
        if not req.data:state['v']=np.zeros(14)
        return resp
    sub=node.create_subscription(UpperJointData,remaps['/upper_body_debug/joint_cmd'],upper,1)
    service=node.create_service(SetBool,remaps['/motion/upper_body_debug'],mode)
    def tick():
        now=time.monotonic()
        if state['active'] and now-state['begin']>.1:
            state['active']=False;state['v']=np.zeros(14);state['watchdog']=True
        if now-state['cache_at']>=.003:
            dt=max(now-state['cache_at'],.001);state['cache_at']=now
            old=state['q'].copy()
            ratio=min(max((now-state['begin'])/.002,0.),1.)
            state['q']=state['previous']+ratio*(state['target']-state['previous'])
            state['v']=(state['q']-old)/dt
        if state['fail_after'] and state['received']>=state['fail_after']:return
        msg=JointState();msg.header.stamp=node.get_clock().now().to_msg();msg.name=names+['waist_yaw_joint']
        msg.position=[*state['q'],.17];msg.velocity=[*state['v'],0.]
        msg.effort=[*e.gravity(state['q']),0.]
        feedback.publish(msg)
        if state['last_q'] is not None and np.array_equal(state['last_q'],state['q']):state['duplicates']+=1
        state['last_q']=state['q'].copy()
    timer=node.create_timer(.001,tick)
    mode_timer=node.create_timer(.02,lambda:status.publish(String(data='mode: UPPER_BODY_DEBUG, state: STAND' if state['active'] else 'mode: READY, state: READY')))
    executor=SingleThreadedExecutor();executor.add_node(node)
    running=True
    def spin():
        while running:executor.spin_once(timeout_sec=.01)
    thread=Thread(target=spin);thread.start()
    checks={}
    try:
        snap=run_executor(config,root/'snapshot',domain,snapshot=True,local_remaps=remaps)
        checks['snapshot_read_only']=snap['passed'] and state['received']==state['mode_calls']==0
        prepare_jog(config_path,root/'snapshot',root/'jog_plan',names[0],.2)
        jog=execute_field(root/'jog_plan',root/'jog_capture',domain,local_remaps=remaps)
        checks['single_axis_jog']=jog['passed']
        jog_report=analyse_field(root/'jog_plan',root/'jog_capture',root/'jog_analysis')
        checks['jog_response_diagnostic']=jog_report['passed'] and jog_report['aggregate_response_lag_ms'] is not None
        plan=prepare_field(config_path,ref,root/'snapshot',root/'plan')
        run=execute_field(root/'plan',root/'capture',domain,local_remaps=remaps)
        checks['real_executor_normal_path']=run['passed'] and run['returncode']==0
        if not run['passed']:raise RuntimeError('Field executor normal case failed; inspect capture/execution/events.csv')
        analysed=analyse_field(root/'plan',root/'capture',root/'analysis')
        checks['gravity_and_urdf_reload']=analysed['passed']
        checks['assumptions_not_promoted']=not analysed['hardware_calibration_accepted'] and not analysed['sample_time_known']
        checks['repeated_cache_seen']=state['duplicates']>0
        before=state['received'];state['fail_after']=before+130;state['watchdog']=False
        failed=execute_field(root/'plan',root/'feedback_gap',domain,local_remaps=remaps)
        count=state['received'];time.sleep(.15)
        checks['feedback_gap_stops_commands']=not failed['passed'] and state['received']==count and state['watchdog']
        checks['no_fabricated_stop_ack']=not failed.get('stop_ack',True) and not failed.get('physical_stop_verified',True)
        write_json(root/'verification.json',dict(passed=all(checks.values()),checks=checks,domain=domain,
            local_only=True,physical_device_tested=False,hardware_commands_on_public_network=False,
            fixture='position-only causal ramp toward latest command; cached state refresh about3ms; publication stamps; no mechanical simulation or vendor behavior certification',
            received_commands=state['received'],duplicate_publications=state['duplicates']))
        return 0 if all(checks.values()) else 1
    finally:
        running=False;thread.join(timeout=2);executor.shutdown();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
