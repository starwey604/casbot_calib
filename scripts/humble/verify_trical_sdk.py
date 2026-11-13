#!/usr/bin/env python3
"""K5: real private ROS roundtrip, synthetic gauges, no GUI or physical robot."""
import argparse
import csv
import json
import os
from pathlib import Path
import time

import numpy as np


def read(path):return json.loads(Path(path).read_text())
def rows(path):
    with Path(path).open() as stream:return list(csv.DictReader(stream))


def inspect_point(root):
    report=read(root/'report.json');motions=read(root/'motions.json');live=read(root/'live.json')
    commands=rows(root/'capture/execution/commands.csv')
    times=np.array([int(r['publish_ns']) for r in commands],dtype=np.int64)
    measurement=read(root/'measurements.json')[0]
    by_generation={m['ack']['generation']:m for m in motions}
    maximum_error=0.
    from scipy.interpolate import BPoly
    curves={g:BPoly.from_derivatives(m['motion']['knots_s'],m['motion']['states']) for g,m in by_generation.items()}
    for row in commands:
        g=int(row['segment_generation'])
        if g not in curves:continue  # Initial measured hold, before the first motion.
        motion=by_generation[g];curve=curves[g]
        t=np.clip((int(row['source_evaluation_ns'])-motion['ack']['start_receive_clock_ns'])*1e-9,0,motion['motion']['duration_s'])
        for key,order in [('q',0),('v',1)]:
            maximum_error=max(maximum_error,float(np.max(abs(np.array([float(row[key+str(j)]) for j in range(14)])-curve(t,nu=order)))))
    phases=[r['state'] for r in rows(root/'capture/execution/events.csv')]
    known=np.load(root/'observations.npz',allow_pickle=False)
    result=dict(single_point_completed=report['passed'] and report['measurements']==1,
        correction_was_exercised=any(e['state']=='correct' for e in read(root/'events.json')),
        centered=max(abs(np.array(measurement['offset_sensor_m'])))<=30e-6,
        one_mode_entry_exit=phases.count('ENTERING')==1 and phases.count('EXITING')==1 and phases[-1]=='COMPLETE',
        native_curve_matches_scipy=maximum_error<1e-10,
        all_reference_proofs_passed=all(m['reference']['passed'] for m in motions),
        no_watchdog_sized_publish_gap=int(np.diff(times).max())<100_000_000,
        planning_pause_keeps_sending=sum(int(r['segment_generation'])==2 for r in commands)>350,
        joint_sample_times_remain_unknown=not known['joint_sample_known'].any() and (known['joint_sample_ns']==-1).all(),
        raw_full_messages_preserved=(root/'capture/raw_state/raw/cdr.bin').stat().st_size>0,
        actual_pd_and_gravity_recorded=all(float(r['kp0'])==300. and float(r['kd0'])==25. for r in commands) and any(abs(float(r['effort1']))>.01 for r in commands),
        no_formal_or_hardware_qualification=not report['formal_kinematic_acceptance'] and not report['hardware_executable'])
    return result,dict(maximum_native_curve_error=maximum_error,maximum_publish_gap_ms=float(np.diff(times).max())/1e6,
        published_commands=len(commands),final_centering_error_um=float(max(abs(np.array(measurement['offset_sensor_m']))))*1e6)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain-id',type=int,required=True)
    p.add_argument('--point',type=Path,help='复用同实现已完成的单点记录，只补故障检查')
    args=p.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
    root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    from casbot_calib.sdk_mock import SDKMockSession
    from casbot_calib.trical_sdk import run_point,RealtimeDevice
    from casbot_calib.dvt2_trical import configuration
    from casbot_calib.hardware_field import load_profile,params
    from casbot_calib.kinematics import nominal_kinematics,ball_world
    from casbot_calib.sdk_process import ros_arguments,run_process
    from casbot_calib.sdk_binary import executor_binary
    from verify_sdk_mock import MockAdmin
    checks={};faults={}
    with SDKMockSession('configs/sdk_mock.yaml',root/'mock',args.domain_id,diagnostics=False) as mock:
        point=args.point.resolve() if args.point else root/'point'
        if not args.point:
            normal=run_point(mock.client_profile,point,args.domain_id,planner_pause_s=1.)
            if not normal['passed']:raise RuntimeError('K5 normal point failed: '+str(normal['failure']))
        checks,metrics=inspect_point(point)
        admin=MockAdmin(mock)
        try:
            for kind in ('operator_cancel','indicator_disconnected','control_mode_lost','stale_feedback','native_nonlocal_denied'):
                config=configuration();config.pop('mock');field=load_profile(mock.client_profile)
                config['workflow']['max_joint_age_ns']=200_000_000
                engine,_=nominal_kinematics(config);home=np.array(config['workflow']['home_rad'])
                field['max_feedback_age_ns']=200_000_000;field['max_mode_age_ns']=500_000_000
                field['breakaway_parameters']=dict(command_kp=list(map(float,config['drive']['kp'])),command_kd=list(map(float,config['drive']['kd'])))
                output=root/kind;output.mkdir()
                if kind=='native_nonlocal_denied':
                    binary,_=executor_binary();values=params(field,[j.name for j in engine.model.joints])
                    values.update(output=str(output/'execution'),trical_stream_file=str(output/'unused.csv'))
                    environment=dict(os.environ,ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
                    # Bypass Python connection validation deliberately; native must
                    # reject the default public endpoints before any mode request.
                    result=run_process(ros_arguments(binary,values),environment,output/'process.log')
                    checks[kind]=result.returncode!=0 and 'private localhost' in (output/'process.log').read_text()
                    continue
                device=RealtimeDevice(config,field,output,args.domain_id)
                before=len(admin.rows('server_events.csv'))
                try:
                    device.start(home)
                    if kind=='operator_cancel':device.cancel_reason='operator_cancel'
                    elif kind=='indicator_disconnected':
                        device.sensor.fault='disconnected';visit=config['workflow']['visits'][0]
                        device.select(visit['frame'],visit['ball_id'],ball_world(config,visit['ball_id']))
                    elif kind=='control_mode_lost':admin.set('lose_mode',True)
                    elif kind=='stale_feedback':admin.set('feedback_enabled',False)
                    deadline=time.monotonic()+8
                    while device.result is None and time.monotonic()<deadline:
                        try:device.check()
                        except RuntimeError:break
                        time.sleep(.02)
                finally:
                    device.close(home,False)
                    admin.set('lose_mode',False);admin.set('feedback_enabled',True)
                result=device.result;faults[kind]=dict(result=result,root_cause=str(device.error),cancel=device.cancel_reason)
                events=admin.rows('server_events.csv')[before:]
                checks[kind]=not result['passed'] and sum(r['event']=='enter_requested' for r in events)==1
                if kind=='indicator_disconnected':checks[kind]&='indicator_disconnected' in str(device.error)
                if kind=='control_mode_lost':checks[kind]&=result['fault']=='control_mode_lost'
                if kind=='stale_feedback':checks[kind]&=result['fault']=='stale_feedback'
                # No resume or motion is initiated by clearing a test injection.
                count=len(admin.rows('server_commands.csv'));time.sleep(.2)
                checks[kind+'_no_recovery_commands']=len(admin.rows('server_commands.csv'))==count
        finally:admin.close()
    checks['owned_mock_cleaned']=all(code==0 for code in read(root/'mock/shutdown.json')['returncodes'])
    checks={key:bool(value) for key,value in checks.items()}
    report=dict(passed=all(checks.values()),checks=checks,metrics=metrics,fault_cases=faults,point_source=str(point),
        scope='one left_A SDK Mock point; synthetic instrument from received q; no physical hardware or formal K5 metrology',visual_inspection=False)
    (root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
