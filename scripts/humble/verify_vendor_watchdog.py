#!/usr/bin/env python3
"""Independent SDK Mock command-silence check at a certified stationary pose."""
import argparse
from importlib.resources import files
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
import numpy as np
import yaml
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.collision import nominal_profile,certify_reference,certify_polyline
from casbot_calib.vendor_position_analysis import rows
from verify_vendor_s2 import stop


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_DOMAIN_ID']=str(150+int(uuid.uuid4().hex[:2],16)%25)
    import rclpy
    from sensor_msgs.msg import JointState
    from crb_ros_msg.msg import UpperJointData
    from std_srvs.srv import SetBool
    model,identity=nominal_dual_arm();names=[j.name for j in model.model.joints];model_id='casbot_20251015_nominal:'+identity['sha256'][:12]
    home=np.array([0,.45,0,-.9,0,0,0,0,-.45,0,-.9,0,0,0])
    profile=nominal_profile(scene=yaml.safe_load(Path('configs/collision_scene.yaml').read_text()))
    reference=certify_reference(profile,lambda t:np.broadcast_to(home,(len(t),14)),1.,np.zeros(14))
    if not reference['passed']:raise ValueError('Stationary watchdog reference rejected')
    endpoint='/casbot_calib_s2_watchdog_'+uuid.uuid4().hex[:8]
    prefix=Path(os.environ['CASBOT_VENDOR_INSTALL'])/'casbot_calib_vendor/lib/casbot_calib_vendor'
    source=str(files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf'))
    commands=[['install/humble-ros/casbot_calib_transport/lib/casbot_calib_transport/mock_host','--ros-args',
        '-r','__ns:='+endpoint+'/plant','-p','model_id:='+model_id,'-p','experiment_id:=watchdog',
        '-p','model_path:='+source,'-p','output_dir:='+str(root),'-p','initial_q:='+json.dumps(home.tolist())],
        [str(prefix/'mock_sdk_bridge'),'--ros-args','-p','endpoint_root:='+endpoint,'-p','model_id:='+model_id,
         '-p','experiment_id:=watchdog','-p','model_path:='+source,'-p','joint_names:='+json.dumps(names),'-p','output:='+str(root)]]
    processes=[];logs=[];rclpy.init();node=rclpy.create_node('sdk_watchdog_fixture');observed=[]
    sub=node.create_subscription(JointState,endpoint+'/joint_states',lambda msg:observed.append((time.monotonic_ns(),msg)),10)
    pub=node.create_publisher(UpperJointData,endpoint+'/upper_cmd',1);client=node.create_client(SetBool,endpoint+'/upper_debug')
    try:
        for i,command in enumerate(commands):
            log=(root/f'process_{i}.log').open('w');logs.append(log);processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT))
        deadline=time.monotonic()+10
        while not observed or not client.service_is_ready() or not pub.get_subscription_count():
            if time.monotonic()>deadline:raise RuntimeError('Watchdog fixture startup timeout')
            rclpy.spin_once(node,timeout_sec=.02)
        msg=observed[-1][1];pose=[msg.position[msg.name.index(n)] for n in names]
        future=client.call_async(SetBool.Request(data=True));rclpy.spin_until_future_complete(node,future,timeout_sec=2)
        if not future.done() or not future.result().success:raise RuntimeError('Mode rejected')
        for _ in range(25):
            pub.publish(UpperJointData(name=names,position=pose,velocity=[0.]*14,effort=[0.]*14,vel_scale=.1))
            rclpy.spin_once(node,timeout_sec=.002);time.sleep(.002)
        silence=time.monotonic_ns()
        while time.monotonic_ns()-silence<600000000:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        for process in reversed(processes):stop(process)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
    events=rows(root/'bridge_events.csv');watch=events[events['event']=='mock_watchdog_hold']
    accepted=rows(root/'bridge_commands.csv')
    observed_delay=(int(watch['receive_ns'][0])-int(accepted['receive_ns'][-1]))*1e-6 if len(watch)==1 and accepted is not None else None
    q=np.array([[msg.position[msg.name.index(n)] for n in names] for _,msg in observed])
    v=np.array([[msg.velocity[msg.name.index(n)] for n in names] for _,msg in observed]);times=np.array([t for t,_ in observed],dtype=np.int64)
    actual=certify_polyline(profile,q,(times-times[0])*1e-9)
    last=times>silence+300000000
    passed=bool(len(watch)==1 and accepted is not None and len(accepted)>=10 and observed_delay>=100 and
                np.any(last) and np.max(abs(v[last]))<.02 and np.max(abs(q[last]-pose))<.0005 and actual['passed'])
    result=dict(passed=passed,scope='stationary local Mock command silence, not hardware brake certification',
                watchdog_events=len(watch),explicit_stop_service_called=False,hardware_executable=False,
                last_accepted_command_to_watchdog_ms=observed_delay,
                reference=reference,observed_interpolant=actual,maximum_tail_speed_rad_s=float(np.max(abs(v[last]))),
                source_model_id=model_id,domain_id=int(os.environ['ROS_DOMAIN_ID']))
    np.savez_compressed(root/'callback_observations.npz',receive_ns=times,q=q,dq=v)
    (root/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(passed=passed,watchdog_events=len(watch))))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
