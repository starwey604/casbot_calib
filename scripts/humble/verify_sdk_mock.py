#!/usr/bin/env python3
"""H1/H2 through the unified SDK server and existing native mechanical host."""
import argparse
import csv
import json
import os
from pathlib import Path
from threading import Thread
import time
import uuid
import numpy as np
import yaml


class MockAdmin:
    """Test-only fault controls/diagnostics; never supplied to identification."""
    def __init__(self, session):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rcl_interfaces.srv import SetParameters
        from rclpy.executors import SingleThreadedExecutor
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.rclpy=rclpy;self.session=session;self.root=session.root/'evaluation/server'
        self.node=rclpy.create_node('sdk_mock_test_admin',enable_rosout=False,start_parameter_services=False)
        self.client=self.node.create_client(SetParameters,session.endpoint+'/casbot_sdk_mock_server/set_parameters')
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        self.running=True;self.generation=0;self.workers=[];self.watchdog_at=0
        self.thread=Thread(target=self.spin);self.thread.start()
        if not self.client.wait_for_service(timeout_sec=5):raise RuntimeError('Mock admin service unavailable')
    def spin(self):
        while self.running:self.executor.spin_once(timeout_sec=.01)
    def set(self,name,value):
        from rclpy.parameter import Parameter
        from rcl_interfaces.srv import SetParameters
        request=SetParameters.Request();request.parameters=[Parameter(name,value=value).to_parameter_msg()]
        future=self.client.call_async(request)
        deadline=time.monotonic()+5
        while not future.done():
            if time.monotonic()>deadline:raise TimeoutError('Mock parameter update timeout')
            time.sleep(.005)
        if not all(r.successful for r in future.result().results):raise RuntimeError('Mock fault injection rejected')
    def rows(self,name):
        path=self.root/name
        with path.open() as stream:
            lines=stream.readlines()
        return list(csv.DictReader(line for line in lines if line.endswith('\n')))
    def __getitem__(self,key):
        if key=='received':return len(self.rows('server_commands.csv'))
        if key=='duplicates':return sum(int(r['duplicate_cache']) for r in self.rows('server_publications.csv'))
        if key=='mode_calls':return sum(r['event'] in ('enter_requested','exit_after_hold') for r in self.rows('server_events.csv'))
        if key=='watchdog':return any(r['event']=='assumed_watchdog_hold' for r in self.rows('server_events.csv')[self.watchdog_at:])
        raise KeyError(key)
    def __setitem__(self,key,value):
        if key=='unstable_static':self.set('unstable_static_pairs',1 if value else 0)
        elif key=='watchdog':self.watchdog_at=len(self.rows('server_events.csv'))
        elif key=='fail_after':
            self.generation+=1;generation=self.generation
            if value is None:self.set('feedback_enabled',True);return
            def inject():
                deadline=time.monotonic()+30
                while self.running and generation==self.generation and time.monotonic()<deadline:
                    if self['received']>=value:self.set('feedback_enabled',False);return
                    time.sleep(.02)
            worker=Thread(target=inject);self.workers.append(worker);worker.start()
        else:raise KeyError(key)
    def close(self):
        self.generation+=1
        for worker in self.workers:worker.join(timeout=1)
        self.running=False;self.thread.join(timeout=2)
        self.executor.shutdown();self.node.destroy_node();self.rclpy.shutdown()


def reference(root,engine,source):
    from casbot_calib.static_field import paired_motions
    from casbot_calib.data import write_json
    from casbot_calib.hardware_field import seal
    names=[j.name for j in engine.model.joints]
    preset=yaml.safe_load(Path('configs/static_field_nominal.yaml').read_text())
    preset['approach_offset_rad']=.001
    preset['timing']=dict(initial_hold_s=.1,minimum_transition_s=1.,minimum_paired_approach_s=.8,
                         settle_s=3.,measure_s=.4,final_hold_s=.1)
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
        planned_execution_order=order,scope='small SDK physical Mock integration fixture; not static observability design'))
    seal(ref);return ref


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--h2',action='store_true');parser.add_argument('--render',action='store_true')
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    domain=220+int(uuid.uuid4().hex[:2],16)%10
    os.environ['ROS_DOMAIN_ID']=str(domain);os.environ['ROS_LOCALHOST_ONLY']='1'
    from casbot_calib.sdk_mock import SDKMockSession
    from casbot_calib.sdk_connection import connection_remaps
    from casbot_calib.hardware_field import load_profile,run_executor,prepare_jog,prepare_field,execute_field,analyse_field
    from casbot_calib.dual_model import nominal_dual_arm
    from casbot_calib.data import write_json
    engine,source=nominal_dual_arm();names=[j.name for j in engine.model.joints]
    preset=reference(root,engine,source)
    base=yaml.safe_load(Path('configs/hardware_field.yaml').read_text())
    base.update(collision_scene=str(Path('configs/collision_scene.yaml').resolve()),anchor_transition_s=.3,
                initial_hold_s=.15,final_hold_s=.15)
    profile=root/'base.yaml';profile.write_text(yaml.safe_dump(base))
    checks={}
    with SDKMockSession('configs/sdk_mock.yaml',root/'mock',domain,profile=profile) as session:
        admin=MockAdmin(session)
        try:
            profile=session.client_profile;config=load_profile(profile)
            remaps=connection_remaps(config,domain)
            snap=run_executor(config,root/'snapshot',domain,snapshot=True)
            checks['snapshot_read_only']=snap['passed'] and admin['received']==admin['mode_calls']==0
            prepare_jog(profile,root/'snapshot',root/'jog_plan',names[0],.2)
            jog=execute_field(root/'jog_plan',root/'jog_capture',domain)
            checks['single_axis_jog']=jog['passed']
            if not jog['passed']:raise RuntimeError('Physical SDK Mock jog failed; inspect retained capture')
            jr=analyse_field(root/'jog_plan',root/'jog_capture',root/'jog_analysis')
            checks['jog_response_diagnostic']=jr['passed'] and jr['aggregate_response_lag_ms'] is not None
            prepare_field(profile,preset,root/'snapshot',root/'plan')
            run=execute_field(root/'plan',root/'capture',domain)
            checks['real_executor_normal_path']=run['passed'] and run['returncode']==0
            if not run['passed']:raise RuntimeError('Physical SDK Mock static failed; inspect retained capture')
            analysis=analyse_field(root/'plan',root/'capture',root/'analysis')
            checks['gravity_and_urdf_reload']=analysis['passed'] and analysis['commissioning_validation_passed']
            checks['assumptions_not_promoted']=not analysis['hardware_calibration_accepted'] and not analysis['sample_time_known']
            checks['repeated_cache_seen']=admin['duplicates']>0
            before=admin['received'];admin['fail_after']=before+130;admin['watchdog']=False
            failed=execute_field(root/'plan',root/'feedback_gap',domain)
            time.sleep(.2);count=admin['received'];time.sleep(.15)
            checks['feedback_gap_stops_commands']=not failed['passed'] and admin['received']==count and admin['watchdog']
            checks['no_fabricated_stop_ack']=not failed.get('stop_ack',True) and not failed.get('physical_stop_verified',True)
            if args.h2:checks.update(verify_h2(root,admin,profile,preset,domain,remaps,names))
            admin['fail_after']=None
            debug_config=dict(config,joint_topic='/motion/debug/joint_state',feedback_type='crb_ros_msg/msg/JointStateData')
            debug=run_executor(debug_config,root/'debug_snapshot',domain,snapshot=True)
            checks['same_client_custom_debug_feedback']=debug['passed']
            services=[name for name,types in admin.node.get_service_names_and_types()]
            checks['no_mock_stop_service']=not any(name.startswith(session.endpoint) and name.endswith('/stop') for name in services)
        finally:admin.close()
    publications=np.genfromtxt(root/'mock/evaluation/server/server_publications.csv',delimiter=',',names=True,ndmin=1,dtype=None,encoding='utf-8')
    references=np.genfromtxt(root/'mock/evaluation/server/server_references.csv',delimiter=',',names=True,ndmin=1)
    checks['mock_1000hz_declared_and_measured']=len(references)>100 and 850<1e9/np.mean(np.diff(references['tick_ns']))<1150
    checks['cache_repeat_has_private_sequence_evidence']=bool(np.any(publications['duplicate_cache']))
    if args.h2:checks.update(verify_truth_isolation(root)['checks'])
    # The client's raw/normal records still carry no acquisition-time claim.
    write_json(root/'verification.json',dict(passed=all(checks.values()),checks=checks,domain=domain,
        local_only=True,physical_device_tested=False,hardware_commands_on_public_network=False,
        fixture='SDK server + existing ArmSimulationSession host; ordinary PD + configured internal gravity; position-only linear ramp',
        native_truth_used_by_client=False,public_stamp_is_acquisition=False,
        reference_tick_hz=1e9/np.mean(np.diff(references['tick_ns']))))
    return 0 if all(checks.values()) else 1


def verify_truth_isolation(root):
    import subprocess
    import sys
    from casbot_calib.data import write_json
    root=Path(root).resolve();session=root/'h2_batch'
    code='''import os,sys
from pathlib import Path
def guard(event,args):
    if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
        raise RuntimeError('evaluation access forbidden')
sys.addaudithook(guard)
try:open('/__casbot_guard_probe__/evaluation/truth.npz','rb')
except RuntimeError:pass
else:raise RuntimeError('truth isolation guard was not active')
from casbot_calib.cli import main
raise SystemExit(main(['hardware-analyse-batch',sys.argv[1],'--output',sys.argv[2]]))
'''
    with (root/'truth_denied_reanalysis.log').open('x') as stream:
        run=subprocess.run([sys.executable,'-c',code,str(session),str(root/'truth_denied_analysis')],stdout=stream,stderr=subprocess.STDOUT,timeout=60)
    latest=json.loads((session/'latest.json').read_text())
    original=json.loads((session/latest['report']/'gravity/calibration.json').read_text())
    checks=dict(fresh_process_analysis_with_evaluation_denied=run.returncode==0)
    difference=None
    if run.returncode==0:
        candidate=json.loads((root/'truth_denied_analysis/gravity/calibration.json').read_text())
        difference=float(np.max(np.abs(np.asarray(original['parameters'])-candidate['parameters'])))
        checks['truth_denied_parameters_identical']=difference<1e-10
    report=dict(passed=all(checks.values()),checks=checks,parameter_max_difference=difference,device_writes=0)
    write_json(root/'truth_isolation.json',report);return report


def verify_h2(root,state,profile,preset,domain,remaps,names):
    from threading import Thread
    import signal
    from casbot_calib import hardware_batch as batch
    from casbot_calib import hardware_field as field
    from casbot_calib.data import write_json
    checks={};state['fail_after']=None;state['unstable_static']=False
    config=yaml.safe_load(Path('configs/hardware_batch.yaml').read_text())
    config['direction'].update(joints=[names[0]],amplitude_deg=.2)
    config['frequency'].update(joints=[names[0]],repeats=1,amplitude_deg=.2)
    config['inter_job_pause_s']=.1
    path=root/'h2_config.yaml';path.write_text(yaml.safe_dump(config))
    folder=root/'h2_batch';batch.plan_batch(path,profile,folder,preset=preset,domain_id=domain,connection_scope='localhost_contract_mock')
    class FixtureBackend(batch.SDKBatchBackend):
        def __init__(self):super().__init__(remaps);self.inject_quality=True
        def execute(self,plan,output,domain_id):
            metadata=batch.read(Path(plan)/'plan.json')
            inject=self.inject_quality and metadata['stage']=='gravity'
            state['unstable_static']=inject
            try:return super().execute(plan,output,domain_id)
            finally:
                state['unstable_static']=False
                if inject:self.inject_quality=False
    backend=FixtureBackend()
    first=batch.run_batch(folder)
    checks['h2_waits_for_bound_human_review']=first['state']=='awaiting_direction_review'
    review=root/'h2_direction_review.yaml';batch.write_direction_template(folder,review)
    data=yaml.safe_load(review.read_text());data['observer']='isolated ROS protocol fixture; not physical direction evidence'
    for row in data['joints']:row['observed_same_direction']=True
    review.write_text(yaml.safe_dump(data));batch.import_direction_review(folder,review)
    final=batch.run_batch(folder,resume=True,backend=backend)
    checks['h2_complete']=final['passed'] and final['gravity']['available']
    history,_=batch.outcomes(folder,batch.load_batch(folder)[1])
    checks['h2_quality_retry_retained']=any(any(r['status']=='quality_retry' for r in rows) and batch.accepted(rows) for rows in history.values())
    checks['h2_same_reference_across_tiers']=final['frequency_references_consistent'] and len({r['comparison_reference_sha256'] for r in final['frequency']})==1
    from casbot_calib.hardware_batch_viewer import render_batch
    view=render_batch(folder,root/'h2_replay',fps=4,max_seconds_per_attempt=.4)
    checks['h2_geometry_only_replay']=view['passed'] and view['device_writes']==0
    interrupted_config=json.loads(json.dumps(config));interrupted_config['direction']['require_review']=False
    interrupted_config['frequency']['enabled']=False;interrupted_config['static']['enabled']=False
    path2=root/'h2_interrupt.yaml';path2.write_text(yaml.safe_dump(interrupted_config))
    paused=root/'h2_interrupted';batch.plan_batch(path2,profile,paused,domain_id=domain,connection_scope='localhost_contract_mock')
    class InterruptBackend(batch.SDKBatchBackend):
        def __init__(self):super().__init__(remaps);self.once=True
        def execute(self,plan,output,domain_id):
            if not self.once:return super().execute(plan,output,domain_id)
            self.once=False;cancel=[False]
            def interrupt_when_streaming():
                deadline=time.monotonic()+20
                while not cancel[0] and time.monotonic()<deadline:
                    events=Path(output)/'execution/events.csv'
                    if events.exists() and 'STREAMING' in events.read_text():
                        time.sleep(.06);os.kill(os.getpid(),signal.SIGINT);return
                    time.sleep(.02)
            worker=Thread(target=interrupt_when_streaming);worker.start()
            try:return super().execute(plan,output,domain_id)
            finally:cancel[0]=True;worker.join(timeout=1)
    interrupted_backend=InterruptBackend();before=batch.run_batch(paused,backend=interrupted_backend)
    after=batch.run_batch(paused,resume=True,backend=interrupted_backend)
    checks['h2_controlled_interrupt_resume']=before['state']=='interrupted' and after['passed']
    state['fail_after']=state['received']+130
    failed=root/'h2_terminal';batch.plan_batch(path2,profile,failed,domain_id=domain,connection_scope='localhost_contract_mock')
    bad=batch.run_batch(failed,backend=batch.SDKBatchBackend(remaps));count=state['received']
    refused=False
    try:batch.run_batch(failed,resume=True,backend=batch.SDKBatchBackend(remaps))
    except RuntimeError:refused=True
    checks['h2_terminal_fault_no_recovery_or_resume']=bad['state']=='faulted' and refused and state['received']==count
    fault=list((failed/'faults').glob('*/clear_fault.json'))
    checks['h2_clear_fault_is_log_only']=len(fault)==1 and batch.read(fault[0])['performed'] is False
    write_json(root/'h2_verification.json',dict(passed=all(checks.values()),checks=checks,local_only=True,physical_device_tested=False))
    return checks


if __name__=='__main__':raise SystemExit(main())
