#!/usr/bin/env python3
"""Nonvisual 1kHz private ROS storage, exact CDR decode, live save and hard-kill recovery."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

from verify_fieldwork_storage import wait


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain-id',type=int,required=True);args=p.parse_args()
    root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
    from verify_fieldwork_round1 import ProtocolFixture
    from casbot_calib.fieldwork_control import Workspace,Session,create_session,initial_commands,export_readback
    from casbot_calib.fieldwork_settings import defaults
    from casbot_calib.fieldwork_observer import observe
    from casbot_calib.fieldwork_storage import attach_budget,CacheMonitor,recording_pause_file,save_snapshot,last_observation,directory_bytes
    from casbot_calib.raw_archive import iter_records,chunk_streams,encoded_frames,capture_raw
    from casbot_calib.fieldwork_a_evidence import run as evidence
    from casbot_calib.sdk_process import SharedRecorder
    from sensor_msgs.msg import JointState,Imu
    from std_msgs.msg import String
    from crb_ros_msg.msg import JointStateData
    from rclpy.serialization import deserialize_message
    fixture=ProtocolFixture(root/'fixture',args.domain_id);fixture.velocity=[0.]*14;fixture.enable_service()
    fixture.timer.cancel();tick=[0]
    debug=fixture.node.create_publisher(JointStateData,fixture.endpoint+'/motion/debug/joint_state',10)
    imu=fixture.node.create_publisher(Imu,fixture.endpoint+'/motion/imu',10)
    def publish():
        if fixture.active and time.monotonic()-fixture.last_command>.1:fixture.active=False
        tick[0]+=1;k=tick[0];stamp=fixture.node.get_clock().now().to_msg()
        msg=JointState();msg.header.stamp=stamp;msg.name=fixture.names+['waist_yaw_joint']
        msg.position=fixture.q+[.17];msg.velocity=[0.]*15
        msg.effort=[math.sin(.031*k+j)*.02 for j in range(15)];fixture.pub.publish(msg)
        d=JointStateData();d.header=msg.header;d.name=msg.name;d.position=msg.position;d.velocity=msg.velocity;d.effort=msg.effort
        d.kp=[20.]*14+[0.];d.kd=[2.]*14+[0.];debug.publish(d)
        if k%5==0:
            m=Imu();m.header.stamp=stamp;m.header.frame_id='private_torso';m.orientation.w=1.
            m.angular_velocity.x=math.sin(k*.01);m.linear_acceleration.z=9.81
            m.orientation_covariance[0]=-1.
            if k%100==0:m.linear_acceleration.x=float('nan')
            imu.publish(m)
        if k%10==0:
            m=String();m.data='mode: UPPER_BODY_DEBUG, state: STAND' if fixture.active else 'mode: READY, state: READY';fixture.mode.publish(m)
    timer=fixture.node.create_timer(.001,publish)
    workspace=root/'workspace';create_session(fixture.client_profile,workspace,args.domain_id,'localhost_contract_mock',workspace=True)
    attach_budget(workspace);monitor=CacheMonitor(workspace);pause=recording_pause_file(workspace)
    latest={};failures=[];results={};checks={};stop=threading.Event();observation_stop=threading.Event();fault_stop=threading.Event();fault_thread=None
    def event(kind,**values):
        if kind=='feedback':latest.update(values)
    def observing():
        try:results['observation']=observe(workspace,'observations/one',lambda:'operator_disconnect' if observation_stop.is_set() else None,event)
        except Exception as exc:failures.append(str(exc))
    observer=threading.Thread(target=observing);observer.start();sender=None;crash=None
    raw=workspace/'observations/one/capture/raw'
    try:
        wait(lambda:latest.get('q') is not None)
        control=workspace/Workspace(workspace).new_control();session=Session(control);run=control/'runs/one';run.mkdir(parents=True)
        values,_=initial_commands(latest['q'],None,None,session.metadata['manual_tuning'])
        settings=defaults(session.config,session.names,session.metadata['model_context'])
        def sending():
            try:results['control']=session.stream('runs/one',run/'latest.csv',initial_values=values,settings=settings,shared_raw=raw,
                recording_pause_file=pause,cancel_check=lambda:'operator_cancel' if stop.is_set() else None)
            except Exception as exc:failures.append(str(exc))
        sender=threading.Thread(target=sending);sender.start();wait(lambda:fixture.commands>500)
        time.sleep(1.)
        checks['one_raw_archive_no_control_raw_copy']=len(list(workspace.rglob('format.json')))==1 and not list((run/'capture').rglob('cdr.bin')) and not (run/'capture/raw_state').exists()
        saved=root/'live_saved';save_snapshot(workspace,saved)
        checks['live_snapshot_seals_every_chunk']=bool(list(saved.rglob('*.bin.gz'))) and not list(saved.rglob('*.open.gz'))
        moved=root/'relocated';saved.rename(moved);moved_run=moved/run.relative_to(workspace)
        r,start,end=capture_raw(moved_run/'capture');records=list(iter_records(r,start_ns=start,end_ns=end))
        checks['relocated_shared_reference_reads']=bool(records) and all(start<=p['receive_ns'] for p,c in records)
        readback=export_readback(moved_run,root/'selected.csv',end_s=1.,joints=[0,7]);checks['selected_readback']=readback['rows']>0
        checks['recover_pose_from_compressed_observation']=last_observation(moved,session.names)['q']==fixture.q
        # Independently deserialize actual received CDR, including the extra axis and NaN bits.
        types={'joint':JointState,'debug':JointStateData,'mode':String,'imu':Imu};counts={};nonfinite=False;decoded=True
        for packet,cdr in iter_records(raw):
            channel=packet['channel'];counts[channel]=counts.get(channel,0)+1
            m=deserialize_message(cdr,types[channel]);j=packet['message']
            if channel in ('joint','debug'):
                decoded &= list(m.name)==j['name'] and list(m.position)==j['position'] and list(m.effort)==j['effort'] and len(m.name)==15
            elif channel=='imu':
                if math.isnan(m.linear_acceleration.x):nonfinite |= j['linear_acceleration'][0] is None
                decoded &= m.header.frame_id==j['header']['frame_id']
        checks['all_axes_and_cdr_decode_exact']=bool(decoded and nonfinite and set(counts)==set(types))
        results['received_channels']=counts
        # Quota must pause storage without ending the shared recorder or sender.
        monitor.limit=1;monitor.check();wait(lambda:pause.exists());time.sleep(.5)
        size=directory_bytes(raw/'chunks');commands=fixture.commands;received=latest['receive_ns'];time.sleep(.7)
        checks['quota_stops_compressed_append']=directory_bytes(raw/'chunks')==size
        checks['quota_keeps_feedback_and_sending']=fixture.commands>commands and latest['receive_ns']>received
        stop.set();sender.join(timeout=20)
        checks['normal_control_stop']=not sender.is_alive() and results['control']['controlled_stop']
        from casbot_calib import fieldwork_zero as zero
        zero.save_zero(session.config,args.domain_id,'localhost_contract_mock',fixture.q,receive_ns=latest['receive_ns'],path=root/'zero.yaml')
        zero_result=zero.return_zero(session.config,args.domain_id,'localhost_contract_mock',path=root/'zero.yaml',
            output=root/'zero_run',shared_raw=raw,recording_pause_file=pause,notify=lambda _:None)
        checks['zero_uses_shared_raw_through_snapshot_and_execute']=zero_result['passed'] and len(list((root/'zero_run').rglob('raw_reference.json')))>=2 and not list((root/'zero_run').rglob('raw_state'))
        observation_stop.set();observer.join(timeout=20)
        checks['observer_finishes_all_chunks']=not observer.is_alive() and not list((raw/'chunks').glob('*.open.gz'))
        flat=0;frames=0
        with chunk_streams(raw) as chunks:
            for path,stream,n in chunks:
                for encoded,cdr in encoded_frames(stream,n,sealed=True):flat+=len(encoded)+1+len(cdr);frames+=1
        compressed=sum(p.stat().st_size for p in (raw/'chunks').glob('*.gz'))
        results['storage']=dict(frames=frames,single_flat_json_plus_cdr_bytes=flat,compressed_bytes=compressed,
            compressed_to_single_flat_ratio=compressed/flat,
            comparator='same received frames JSON text + CDR bytes, before compression; no second subscription counted')
        checks['lossless_compression_reduces_size']=compressed<flat*.5
        # Kill only a new recorder owned by this test while its gzip footer is open.
        crash_raw=root/'crash/raw';crash_raw.parent.mkdir()
        binary=Path('install/vendor/casbot_calib_vendor/lib/casbot_calib_vendor/state_recorder').resolve()
        command=[str(binary),'--ros-args']
        for k,v in dict(output=str(crash_raw),duration_s=60.,until_stopped=True,chunked_archive=True,recording_flush_period_ms=20,
            joint_topic=fixture.endpoint+'/joint_states',joint_type='sensor_msgs/msg/JointState',debug_topic=fixture.endpoint+'/motion/debug/joint_state',
            mode_topic=fixture.endpoint+'/motion/status',robot_state_topic="''",imu_topic=fixture.endpoint+'/motion/imu').items():
            command+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
        with (root/'crash.log').open('w') as log:
            crash=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
            wait(lambda:(crash_raw/'recorder.json').exists())
            wait(lambda:any(p.stat().st_size>2000 for p in (crash_raw/'chunks').glob('*.open.gz')))
            meta=json.loads((crash_raw/'recorder.json').read_text());watch=SharedRecorder(meta)
            checks['live_owner_identity_matches']=watch.poll() is None
            fault_control=root/'crash/control';create_session(fixture.client_profile,fault_control,args.domain_id,'localhost_contract_mock')
            fault_session=Session(fault_control);fault_run=fault_control/'runs/one';fault_run.mkdir(parents=True)
            def fault_sending():
                try:results['fault_control']=fault_session.stream('runs/one',fault_run/'latest.csv',initial_values=values,
                    settings=settings,shared_raw=crash_raw,recording_pause_file=pause,
                    cancel_check=lambda:'operator_cancel' if fault_stop.is_set() else None)
                except Exception as exc:failures.append(str(exc))
            before=fixture.commands;fault_thread=threading.Thread(target=fault_sending);fault_thread.start()
            wait(lambda:fixture.commands>before+100)
            crash.kill();crash.wait(timeout=5)
            fault_thread.join(timeout=20)
            checks['recorder_death_ends_control_without_resume']=not fault_thread.is_alive() and results.get('fault_control',{}).get('result',{}).get('supervisor_interruption')=='raw_recorder_exited' and bool(fault_session.status()['blocked_reason'])
            recovered=list(iter_records(crash_raw));checks['killed_open_chunk_recovers_exact_prefix']=bool(recovered) and [p['receive_sequence'] for p,c in recovered]==list(range(len(recovered)))
            checks['shared_recorder_death_detected']=watch.poll() is not None
            save_snapshot(root/'crash',root/'crash_saved')
            checks['crashed_snapshot_resealed']=list(iter_records(root/'crash_saved/raw'))==recovered
        report=evidence(moved,root/'analysis');checks['analysis_has_one_control_and_one_imu']=len(report['records'])==1 and len(report['imu'])==1
    finally:
        stop.set();observation_stop.set();fault_stop.set()
        if fault_thread:fault_thread.join(timeout=20)
        if sender:sender.join(timeout=20)
        observer.join(timeout=20);monitor.close()
        if crash and crash.poll() is None:crash.kill();crash.wait(timeout=5)
        fixture.close()
    checks['no_worker_errors']=not failures
    report=dict(passed=all(checks.values()),checks=checks,results=results,errors=failures,
        scope='nonvisual private ROS protocol fixture, 1kHz requested publish timer; not mechanics/sample-time evidence')
    (root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
