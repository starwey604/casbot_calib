"""K5 first slice: continuous native SDK stream and feedback-driven fake gauges.

Only private localhost SDK Mock connections are accepted. Host receipt times are
never promoted to joint sampling times. K1--K4 remain virtual-time workflows.
"""
import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np

from .data import write_json
from .dvt2_trical import configuration, routed_controller
from .hardware_field import load_profile, run_executor, seal
from .kinematics import nominal_kinematics, calibration
from .run_marker import run_marker
from .sdk_connection import validate_connection
from .trical_motion import segment
from .trical_workflow import MeasurementController, MeasurementFailure, stable_window


def write_segment(path, generation, action, start, goal, duration_s=0.):
    """Atomic, bounded mailbox. The native sender owns its own monotonic start."""
    values=np.r_[generation,action,duration_s,start,goal]
    if values.shape!=(31,) or not np.isfinite(values).all():raise ValueError('Invalid TriCal segment')
    temporary=Path(str(path)+'.tmp')
    temporary.write_text(','.join(format(v,'.17g') for v in values)+'\n')
    temporary.replace(path)


class SimulatedIndicators:
    """Synthetic instrument evaluated from delivered SDK q, not private plant truth.

    Bias is an explicit test excitation; no claim of physical contact detection.
    A new synthetic conversion has its own sequence/host time, even for equal q.
    """
    def __init__(self, config, bias_m=(.00012,-.00008,.00006), fault=None):
        if fault not in (None,'disconnected','out_of_range','frozen','no_contact'):raise ValueError('Unknown simulated instrument fault')
        self.engine,_=nominal_kinematics(config);self.cal=calibration(config)
        self.bias=np.asarray(bias_m);self.fault=fault;self.sequence=0
        self.frozen=None
        self.frame=None;self.ball=None;self.target=None

    def select(self, frame, ball, target):
        self.frame,self.ball,self.target=frame,ball,np.asarray(target)

    def read(self,q,now):
        if self.frame is None:return None
        if self.fault=='disconnected':raise MeasurementFailure('indicator_disconnected')
        if self.fault=='frozen' and self.frozen is not None:return self.frozen
        self.sequence+=1
        pose=self.engine.frame_pose(q,self.frame)
        offset=pose[:3,:3].T@(self.target-pose[:3,3])
        reading=np.asarray(self.cal.axes)@offset+np.asarray(self.cal.zero)+self.bias
        status=np.zeros(3,dtype=np.uint8)
        status[np.abs(reading)>.005]=1  # Synthetic no-contact outside modelled stroke.
        if self.fault=='out_of_range':status[:]=2
        if self.fault=='no_contact':status[:]=1
        self.frozen=dict(readings=reading,status=status,sequence=self.sequence,sample_ns=now)
        return self.frozen


class RealtimeDevice:
    """Planner-facing facade; the existing native executor alone publishes SDK commands."""
    def __init__(self,config,profile,root,domain_id,*,fault=None,cancel_check=None):
        self.config=config;self.profile=profile;self.root=Path(root);self.domain=domain_id
        validate_connection(profile.get('connection',{}))
        self.sensor=SimulatedIndicators(config,fault=fault);self.lock=threading.RLock()
        self.latest=None;self.rows=[];self.row_times=[];self.command_times=[]
        self.frame=None;self.target=None;self.generation=0;self.acks={};self.result=None;self.error=None
        self.cancel_reason=None;self.external_cancel=cancel_check;self.last_phase=None
        self.mailbox=self.root/'segment.csv';self.thread=None;self.live=[]

    @property
    def now(self):return time.monotonic_ns()

    def select(self,frame,ball,target):
        with self.lock:
            self.frame,self.target=frame,ball;self.sensor.select(frame,ball,target)

    def line(self,line):
        try:self._line(line)
        except Exception as exc:
            if self.error is None:self.error=exc
            self.cancel_reason='k5_observation_failed'

    def _line(self,line):
        # OutputPump drains independently of the planner, including while Coal runs.
        if not line.startswith(('TRICAL_','FIELDWORK_EVENT ','FIELDWORK_NOTICE ')):return
        tag,payload=line.split(' ',1);value=json.loads(payload)
        with self.lock:
            if tag=='FIELDWORK_NOTICE' and value['code']=='recording_io_error':raise MeasurementFailure('k5_recording_failed')
            if tag=='TRICAL_ACK':self.acks[value['generation']]=value
            elif tag=='FIELDWORK_EVENT':
                self.last_phase=value['phase']
                if value['phase'] in ('STOPPING','FAILED') and self.error is None:self.error=MeasurementFailure(value['code'])
            elif tag=='TRICAL_FEEDBACK':
                q=np.asarray(value['q']);dq=np.asarray(value['dq']);received=value['receive_ns']
                self.latest=dict(q=q,dq=dq,receive_ns=received,status=np.zeros(14,dtype=np.uint8))
                packet=self.sensor.read(q,received)
                if packet is not None:
                    self.rows.append(dict(q_rad=q,dq_rad_s=dq,joint_sequence=len(self.rows),joint_sample_ns=-1,
                        joint_receive_ns=received,joint_sample_known=False,
                        indicator_readings_m=packet['readings'],indicator_sequence=np.full(3,packet['sequence'],dtype=np.int64),
                        indicator_sample_ns=np.full(3,packet['sample_ns'],dtype=np.int64),
                        indicator_receive_ns=np.full(3,received,dtype=np.int64),indicator_sample_known=np.ones(3,dtype=bool),
                        indicator_status=packet['status'],ball_id=self.target,tool_frame=self.frame))
                    self.row_times.append(received)
            self.live.append(dict(tag=tag,**value))

    def check(self):
        reason=self.external_cancel() if self.external_cancel else None
        if reason:self.cancel_reason=reason
        if self.cancel_reason:raise MeasurementFailure(self.cancel_reason)
        if self.error:raise MeasurementFailure(str(self.error))
        if self.result is not None:raise MeasurementFailure('executor_ended:'+self.result['reason'])
        if self.latest is not None and self.now-self.latest['receive_ns']>self.config['workflow']['max_joint_age_ns']:
            raise MeasurementFailure('joint_feedback_timeout')

    def wait(self,predicate,timeout_s):
        deadline=time.monotonic()+timeout_s
        while not predicate():
            self.check()
            if time.monotonic()>deadline:raise MeasurementFailure('sdk_ack_or_feedback_timeout')
            time.sleep(.01)
        self.check()

    def start(self,q):
        self.generation=1;write_segment(self.mailbox,1,0,q,q)
        def run():
            try:
                self.result=run_executor(self.profile,self.root/'capture',self.domain,timeout_s=None,
                    trical_command=self.mailbox,on_output=self.line,cancel_check=self.cancel)
            except BaseException as exc:self.error=exc
        self.thread=threading.Thread(target=run,name='trical-sdk-supervisor');self.thread.start()
        self.wait(lambda:self.last_phase=='STREAMING',30.)

    def cancel(self):
        reason=self.external_cancel() if self.external_cancel else None
        if reason:self.cancel_reason=reason
        return self.cancel_reason

    def move(self,start,goal,duration):
        self.check();self.generation+=1
        write_segment(self.mailbox,self.generation,1,start,goal,duration)
        self.command_times.append(self.now)
        self.wait(lambda:self.generation in self.acks,5.)
        ack=self.acks[self.generation]
        self.wait(lambda:self.now>=ack['start_receive_clock_ns']+ack['duration_ns']+10_000_000,duration+2.)
        return ack

    def arrays(self):
        with self.lock:
            if not self.rows:return None
            result={k:np.asarray([r[k] for r in self.rows]) for k in self.rows[0]}
        for key in ('joint_sequence','joint_sample_ns','joint_receive_ns'):result[key]=result[key].astype(np.int64)
        return result

    def close(self,q,success):
        if self.thread is None:return
        if success and self.error is None and self.result is None:
            self.generation+=1;write_segment(self.mailbox,self.generation,2,q,q)
        else:self.cancel_reason=self.cancel_reason or 'k5_session_failed'
        self.thread.join(timeout=20.)
        if self.thread.is_alive():raise RuntimeError('SDK supervisor did not finish')


class RealtimeController(MeasurementController):
    def execute(self,goal,profile,label,duration=None,monitor=True):
        self.device.check()
        motion,gate=segment(profile,self.commanded,goal,self.config,label,duration,gate_cache=self.route_cache)
        record=dict(label=label,reference=gate,motion=motion.metadata(),route_phase=profile.metadata['route_phase'])
        self.motions.append(record)
        if not gate['passed']:raise MeasurementFailure('collision_reference_rejected:'+label)
        self.current_profile=profile;self.event(label)
        record['ack']=self.device.move(self.commanded,goal,motion.duration)
        self.commanded=np.asarray(goal).copy()

    def hold(self,duration,label='settle'):
        # Holding never schedules a new motion or changes mode. Sender keeps 500 Hz.
        self.event(label);end=self.device.now+round(duration*1e9)
        self.device.wait(lambda:self.device.now>=end,duration+1.)

    def wait_window(self,after):
        deadline=self.device.now+round(self.config['workflow']['measurement_timeout_s']*1e9)
        while self.device.now<deadline:
            self.device.check()
            with self.device.lock:
                arrays=self.device.arrays();times=np.asarray(self.device.row_times,dtype=np.int64)
            if arrays is not None and np.any(arrays['indicator_status'][-1]>=2):raise MeasurementFailure('indicator_invalid')
            window=stable_window(arrays,times,self.device.now,self.config,after,self.device.frame,self.device.target)
            if window is not None:return window
            time.sleep(.02)
        raise MeasurementFailure('stable_window_timeout')

    def recover(self):
        # No recovery motion, mode re-request or automatic withdrawal on faults.
        self.device.cancel_reason='k5_session_failed'
        self.event('session_terminated_no_resume');return False


def run_point(profile,output,domain_id,*,case='left_A',fault=None,cancel_check=None,planner_pause_s=0.):
    field=load_profile(profile);validate_connection(field.get('connection',{}))
    if field['connection']['domain_id']!=domain_id:raise ValueError('Use frozen private Mock domain')
    config=configuration();config.pop('mock')
    # Leave feedback headroom below the unchanged 0.35 rad/s K2 limit. Only the
    # traversal time changes; route geometry, tube and contact policy do not.
    config['route_settings'].update(maximum_velocity_rad_s=.25,maximum_acceleration_rad_s2=.5,maximum_jerk_rad_s3=4.)
    visits=[v for v in config['workflow']['visits'] if v['id']==case]
    if len(visits)!=1:raise ValueError('Select one existing left/right A--D visit')
    config['workflow']['visits']=visits
    # Protocol-development timings, not new hardware sampling/geometry evidence.
    config['workflow']['max_joint_age_ns']=200_000_000
    field['max_feedback_age_ns']=200_000_000;field['max_mode_age_ns']=500_000_000
    field['maximum_velocity_rad_s']=config['workflow']['maximum_velocity_rad_s']
    field['breakaway_parameters']=dict(command_kp=list(map(float,config['drive']['kp'])),command_kd=list(map(float,config['drive']['kd'])))
    field['gravity_feedforward']='nominal'
    root=Path(output).resolve();root.mkdir(parents=True,exist_ok=False)
    engine,geometry=nominal_kinematics(config)
    write_json(root/'profile.json',config);write_json(root/'geometry.json',geometry)
    write_json(root/'protocol.json',dict(schema_version='casbot_trical_sdk_mock_v1',case=case,
        connection=field['connection'],joint_sample_time_known=False,instrument='synthetic_from_delivered_sdk_q',
        joint_sequence_semantics='local_snapshot_counter_not_device_freshness',joint_status_semantics='transport_fields_valid_no_axis_state_bits',
        sensor_bias_m=[.00012,-.00008,.00006],feedforward='nominal_gravity_only_no_friction',
        physical_contact_qualified=False,**run_marker()))
    device=RealtimeDevice(config,field,root,domain_id,fault=fault,cancel_check=cancel_check)
    controller=routed_controller(RealtimeController)(engine,config,device)
    failure=None;complete=False
    try:
        # Read-only first snapshot; no target is taken from the configured home.
        from .hardware_field import snapshot_field,_read_snapshot
        snapshot_field(profile,root/'start_snapshot',domain_id)
        q=_read_snapshot(root/'start_snapshot')
        # Certify the initial hold/connection before requesting a control mode.
        _,start_gate=segment(controller.free,q,controller.home,config,'measured_start_to_home',gate_cache=controller.route_cache)
        write_json(root/'start_preflight.json',start_gate)
        if not start_gate['passed']:raise MeasurementFailure('measured_start_reference_rejected')
        device.start(q)
        controller.commanded=q.copy()
        controller.execute(controller.home,controller.free,'measured_start_to_home')
        if planner_pause_s:controller.hold(planner_pause_s,'planner_pause_test')
        controller.visit(visits[0]);complete=True
    except (Exception,KeyboardInterrupt) as exc:
        failure=str(exc) or type(exc).__name__;controller.event('rejected',reason=failure);controller.recover()
    finally:
        device.close(controller.commanded,complete)
        arrays=device.arrays()
        if arrays is not None:
            np.savez_compressed(root/'observations.npz',**arrays)
            write_json(root/'row_receive_ns.json',device.row_times)
        write_json(root/'events.json',controller.events);write_json(root/'motions.json',controller.motions)
        write_json(root/'measurements.json',controller.measurements);write_json(root/'live.json',device.live)
    passed=complete and device.result is not None and device.result['passed']
    if not passed and failure is None:
        failure=str(device.error) if device.error else (device.result or {}).get('fault') or (device.result or {}).get('reason','executor_missing_result')
    report=dict(schema_version='casbot_trical_sdk_report_v1',passed=passed,failure=failure,
        measurements=len(controller.measurements),execution=device.result,session_resumable=False,
        synthetic_instrument=True,physical_device_tested=False,hardware_executable=False,
        formal_kinematic_acceptance=False,joint_sampling_evidence=False,
        last_received_home_error_rad=(device.latest['q']-controller.home).tolist() if device.latest is not None else None)
    write_json(root/'report.json',report);seal(root)
    return report


def main():
    parser=argparse.ArgumentParser(description='K5私有SDK Mock单测点闭环；不允许实机连接')
    parser.add_argument('--profile',type=Path,default=Path('configs/hardware_field.yaml'));parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--mock',action='store_true',help='为本次运行创建并清理独占私有SDK Mock')
    parser.add_argument('--domain-id',type=int,required=True);parser.add_argument('--case',default='left_A')
    args=parser.parse_args()
    if args.mock:
        from .sdk_mock import SDKMockSession
        args.output.mkdir(parents=True,exist_ok=False)
        with SDKMockSession('configs/sdk_mock.yaml',args.output/'mock',args.domain_id,profile=args.profile,diagnostics=False) as mock:
            result=run_point(mock.client_profile,args.output/'point',args.domain_id,case=args.case)
    else:result=run_point(args.profile,args.output,args.domain_id,case=args.case)
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
