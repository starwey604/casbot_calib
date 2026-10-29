#!/usr/bin/env python3
"""Local S2 SDK position workflow against the existing C++ physical Mock."""
import argparse
from importlib.resources import files
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import numpy as np
import yaml
from casbot_calib.vendor_position import prepare_position,execute_position,position_config
from casbot_calib.vendor_position_analysis import analyse_position,rows,rejected_stop_verified
from casbot_calib.dual_model import nominal_dual_arm


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--first-only',action='store_true');parser.add_argument('--render',action='store_true')
    parser.add_argument('--only-case',choices=('stop_rejected','mode_loss','feedback_gap'))
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOCALHOST_ONLY']='1';domain=180+int(uuid.uuid4().hex[:2],16)%25;os.environ['ROS_DOMAIN_ID']=str(domain)
    baseline=position_config('configs/vendor_s2.yaml');engine,source=nominal_dual_arm();names=[j.name for j in engine.model.joints]
    model_id='casbot_20251015_nominal:'+source['sha256'][:12];model_path=str(files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf'))
    native=Path('install/humble-ros/casbot_calib_transport/lib/casbot_calib_transport/mock_host').resolve()
    vendor=Path(os.environ.get('CASBOT_VENDOR_INSTALL','install/vendor'))/'casbot_calib_vendor/lib/casbot_calib_vendor'
    rng=np.random.default_rng(20261029);cases=[dict(name=f'normal_{hz}_{rep}',hz=hz,fault='',repeat=rep) for rep in range(2) for hz in (100,250,500)]
    cases=[cases[i] for i in rng.permutation(len(cases))]
    if args.first_only:cases=[dict(name='normal_500_0',hz=500,fault='',repeat=0)]
    else:
        cases += [dict(name=f,hz=500,fault=f,repeat=0) for f in ('reject_mode','mode_no_confirm','feedback_gap','invalid_feedback','mode_loss','stop_rejected','client_interrupt','competing_publisher','start_changed')]
    if args.only_case:cases=[dict(name=args.only_case,hz=500,fault=args.only_case,repeat=0)]
    results=[];saved_plan=None;saved_endpoint=None
    for case in cases:
        folder=root/case['name'];folder.mkdir();endpoint='/casbot_calib_s2_'+uuid.uuid4().hex[:10]
        if case['fault']=='start_changed':endpoint=saved_endpoint
        home=np.array([0,.45,0,-.9,0,0,0,0,-.45,0,-.9,0,0,0],dtype=float);home[0]=.01*case['repeat']
        if case['fault']=='start_changed':home[0]=.06
        # Public nominal model selects fixed synthetic servo gains, not a second
        # mechanics implementation or calibration truth. Same rule at all rates.
        kp=np.full(14,100.);kd=np.full(14,14.)
        common=['--ros-args','-r','__ns:='+endpoint+'/plant','-p','experiment_id:=sdk_s2_mock','-p','model_id:='+model_id,
                '-p','model_path:='+model_path,'-p','output_dir:='+str(folder),'-p','initial_q:='+json.dumps(home.tolist())]
        bridge=[str(vendor/'mock_sdk_bridge'),'--ros-args','-p','endpoint_root:='+endpoint,'-p','joint_names:='+json.dumps(names),
                '-p','model_id:='+model_id,'-p','experiment_id:=sdk_s2_mock','-p','output:='+str(folder),'-p','fault:='+
                (case['fault'] if case['fault'] and case['fault'] not in ('client_interrupt','competing_publisher','start_changed') else "''")]
        bridge+=['-p','kp:='+json.dumps(kp.tolist()),'-p','kd:='+json.dumps(kd.tolist()),'-p','model_path:='+model_path]
        (folder/'mock_servo.json').write_text(json.dumps(dict(acceleration_kp_s2=kp.tolist(),acceleration_kd_s=kd.tolist(),
            source='declared synthetic computed-torque servo through existing Pinocchio workspace; no identification or additional integrator'),indent=2)+'\n')
        processes=[];logs=[];ros_node=None
        try:
            for label,command in [('host',[str(native),*common]),('bridge',bridge)]:
                log=(folder/(label+'.log')).open('w');logs.append(log);processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT))
            config=dict(baseline,control_hz=case['hz']);config_path=folder/'config.yaml';config_path.write_text(yaml.safe_dump(config))
            print('S2 '+case['name']+': snapshot / collision / execute',flush=True)
            if case['fault']=='start_changed':plan=saved_plan
            else:
                plan=folder/'plan';prepare_position(config_path,plan,endpoint_root=endpoint,domain_id=domain)
                if saved_plan is None:saved_plan=plan;saved_endpoint=endpoint
            if case['fault'] in ('client_interrupt','competing_publisher'):
                with (folder/'launcher.log').open('w') as log:
                    launcher=subprocess.Popen([sys.executable,'-m','casbot_calib.cli','vendor-execute-position',str(plan),'--output',str(folder/'run')],stdout=log,stderr=subprocess.STDOUT)
                    try:
                        deadline=time.monotonic()+15
                        events=folder/'run/execution/events.csv'
                        while not events.exists() or 'STREAMING' not in events.read_text():
                            if launcher.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Executor never entered stream')
                            time.sleep(.01)
                        time.sleep(.25)
                        if case['fault']=='client_interrupt':launcher.send_signal(signal.SIGINT)
                        else:
                            import rclpy
                            from crb_ros_msg.msg import UpperJointData
                            rclpy.init();ros_node=rclpy.create_node('s2_competing_publisher_fixture')
                            competing=ros_node.create_publisher(UpperJointData,endpoint+'/upper_cmd',1)
                        launcher.wait(timeout=10)
                    finally:stop(launcher)
                capture=json.loads((folder/'run/capture.json').read_text())
            else:capture=execute_position(plan,folder/'run')
            analysis=analyse_position(folder/'run',folder/'analysis')
            if case['fault']:
                expected={'reject_mode':'mode_rejected','mode_no_confirm':'mode_transition_timeout','feedback_gap':'stale_feedback',
                          'invalid_feedback':'invalid_feedback','mode_loss':'control_mode_lost','stop_rejected':'control_mode_lost',
                          'client_interrupt':'interrupted','competing_publisher':'competing_command_publisher','start_changed':'start_pose_changed'}[case['fault']]
                passed=not capture['passed'] and capture.get('fault')==expected and analysis['no_commands_after_fault']
                if case['fault']=='stop_rejected':passed = rejected_stop_verified(capture,analysis['no_commands_after_fault'],rows(folder/'bridge_events.csv'))
                elif case['fault'] not in ('reject_mode','start_changed'):passed &= analysis['stop_confirmed']
                if case['fault'] in ('reject_mode','start_changed'):passed &= rows(folder/'run/execution/commands.csv') is None
            else:passed=analysis['passed']
            protocol_verified=bool(passed if case['fault'] else
                (capture['passed'] and analysis['no_commands_after_fault'] and analysis.get('observed_interpolant',{}).get('passed')) or
                (not capture['passed'] and capture.get('fault') in ('stale_feedback','stale_mode','tracking_envelope','aged_tracking_envelope')
                 and analysis['no_commands_after_fault'] and analysis['stop_confirmed']))
            row=dict(case=case,passed=bool(passed),result=capture,command_publish=analysis['command_publish'],
                     protocol_behaviour_verified=protocol_verified,
                     response_lag_ms=analysis.get('aggregate_response_lag_ms'),maximum_tracking_error_rad=analysis.get('maximum_tracking_error_rad'),
                     observed_interpolant_passed=analysis.get('observed_interpolant',{}).get('passed'))
            results.append(row);(folder/'verification.json').write_text(json.dumps(row,indent=2)+'\n')
            print(f"S2 {case['name']}: {'PASS' if passed else 'FAIL'} {capture.get('fault') or capture['reason']}",flush=True)
        finally:
            if ros_node is not None:ros_node.destroy_node();rclpy.shutdown()
            for process in reversed(processes):stop(process)
            for log in logs:log.close()
    relative=[]
    for row in results:
        if row['case']['fault']:continue
        meta=json.loads((root/row['case']['name']/'plan/plan.json').read_text())
        relative.append(np.asarray(meta['motion']['states'])[:,0,:]-np.asarray(meta['initial_q']))
    same=all(np.allclose(relative[0],r,atol=1e-12,rtol=0) for r in relative[1:])
    normal=[r for r in results if not r['case']['fault']]
    report=dict(passed=all(r['protocol_behaviour_verified'] for r in results) and (not normal or any(r['passed'] for r in normal)) and same,
                passed_scope='execution/guard implementation; every captured trial retains its own pass and qualification',
                all_requested_profiles_qualified=all(r['passed'] for r in normal) if normal else None,operating_frequency_recommendation=None,
                scope='local SDK bridge + existing native Mock, position only',
                hardware_tested=False,hardware_executable=False,physical_network_tested=False,
                relative_continuous_motion_identical_across_rates=same,randomized_frequency_order=True,seed=20261029,cases=results)
    import csv
    with (root/'summary.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['case','requested_hz','actual_publish_hz','maximum_tracking_error_rad','aggregate_lag_ms','expected_case_passed','capture_passed','fault'])
        for row in results:writer.writerow([row['case']['name'],row['case']['hz'],row['command_publish']['hz'],row['maximum_tracking_error_rad'],row['response_lag_ms'],row['passed'],row['result']['passed'],row['result'].get('fault')])
    if args.render:
        from casbot_calib.vendor_viewer import view_vendor
        from casbot_calib.vendor_position_analysis import plot_position_summary
        plot_position_summary(report,root/'frequency_response.png')
        eligible=[r for r in results if not r['case']['fault'] and r['passed']]
        if eligible:
            chosen=max(eligible,key=lambda r:r['case']['hz'])
            report['presentation']=view_vendor(root/chosen['case']['name']/'analysis/observations',root/'presentation',fps=12)
            report['presentation_case']=chosen['case']['name']
    (root/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
