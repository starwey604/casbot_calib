"""K2 low-speed measurement state machine; no geometry parameter identification."""
from copy import deepcopy,copy
from hashlib import sha256
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from . import _core
from .data import write_json
from .kinematics import workflow_config,nominal_kinematics,ball_world,calibration
from .trical_data import observation_metadata,save_observations
from .trical_fixture import fixture_profile
from .trical_motion import solve_position,segment,local_correction
from .trical_device import make_mock
from .collision import certify_polyline


class MeasurementFailure(RuntimeError):pass


def stable_window(arrays,times,now,config,after,frame=None,target=None):
    """Host-time static acceptance with independent per-channel freshness and sequence."""
    settings=config['workflow'];width=round(settings['stable_window_s']*1e9)
    if arrays is None:return None
    frame=arrays['tool_frame'][-1] if frame is None else frame;target=arrays['ball_id'][-1] if target is None else target
    selected=np.flatnonzero((times>=max(after,now-width)) & (times<=now) & (arrays['tool_frame']==frame) & (arrays['ball_id']==target))
    if len(selected)<6 or times[selected[-1]]-times[selected[0]]<width-25_000_000:return None
    a={key:value[selected] for key,value in arrays.items()}
    if np.any(a['indicator_status']!=0):return None
    if np.any(np.diff(a['indicator_sequence'],axis=0)<=0):return None
    known=a['indicator_sample_known']
    # Mock clock domains are declared; unknown device times require static settling
    # longer than the declared sensor delivery bound before this host-time window.
    if not known.all():
        bound=config['sensor']['delay_ns']+config['sensor']['jitter_ns']+config['sensor']['period_ns']
        if now-after<width+bound:return None
    if np.any(now-a['indicator_receive_ns'][-1]>settings['max_indicator_age_ns']):return None
    if np.any((now-a['indicator_sample_ns'][-1])[known[-1]]>settings['max_indicator_age_ns']):return None
    if now-a['joint_receive_ns'][-1]>settings['max_joint_age_ns']:return None
    if np.max(abs(a['dq_rad_s']))>settings['stable_velocity_rad_s']:return None
    if np.max(np.ptp(a['q_rad'],axis=0))>settings['stable_joint_span_rad']:return None
    if np.max(np.ptp(a['indicator_readings_m'],axis=0))>settings['stable_indicator_span_m']:return None
    return dict(indices=selected.tolist(),q_rad=np.mean(a['q_rad'],axis=0).tolist(),
        readings_m=np.mean(a['indicator_readings_m'],axis=0).tolist(),start_ns=int(times[selected[0]]),end_ns=int(times[selected[-1]]))


def observed_profile(profile, allowance):
    result=copy(profile);result.pair_clearance_m=profile.pair_clearance_m+profile.radii@np.full(14,allowance)
    return result


class MeasurementController:
    def __init__(self, engine, config, device):
        self.engine=engine;self.config=config;self.device=device;self.events=[];self.motions=[];self.measurements=[]
        self.commanded=np.array(config['workflow']['home_rad']);self.period=config['control_period_ns']
        self.free=fixture_profile(engine,config);self.contact=None;self.stage=None;self.home=self.commanded.copy()

    def event(self,state,**details):
        self.events.append(dict(time_ns=self.device.now,state=state,**details))
        if state in ('preflight','read_offset','sample_accepted','rejected','aborted_at_home'):
            print(f'K2 {state} @ {self.device.now*1e-9:.3f}s {details}',flush=True)

    def execute(self, goal, profile, label, duration=None, monitor=True):
        motion,gate=segment(profile,self.commanded,goal,self.config,label,duration)
        record=dict(label=label,start_ns=self.device.now,reference=gate,motion=motion.metadata(),
            target=dict(frame=self.device.frame,ball_id=self.device.target),
            collision=dict(geometry_sha256=profile.metadata['geometry_sha256'],exact_contact_pairs=profile.metadata['exact_contact_pairs'],
                pair_clearances_sha256=profile.metadata['pair_clearances_sha256']))
        self.motions.append(record)
        if not gate['passed']:raise MeasurementFailure('collision_reference_rejected:'+label)
        self.event(label);curve=motion.polynomial();begin=self.device.now
        duration_ns=int(np.ceil(motion.duration*1e9/self.period))*self.period;end=begin+duration_ns
        raw_begin=len(self.device.raw)
        while self.device.now<end:
            chunk=min(end,self.device.now+20_000_000)
            times=np.arange(self.device.now,chunk,self.period,dtype=np.int64);t=np.minimum((times-begin)*1e-9,motion.duration)
            q,v,a=(curve(t,nu=d) for d in (0,1,2));ff=self.engine.evaluate_batch(q,v,a)['total']
            for i,time in enumerate(times):
                values=np.column_stack([q[i],v[i],ff[i],self.config['drive']['kp'],self.config['drive']['kd']])
                self.device.submit(int(time),values)
            self.device.advance(chunk)
            latest=self.device.latest
            if monitor and latest is not None:
                if self.device.now-int(latest['receive_ns'])>self.config['workflow']['max_joint_age_ns'] or np.any(latest['status']!=0):
                    raise MeasurementFailure('joint_feedback_invalid')
                if np.max(abs(latest['dq']))>self.config['workflow']['maximum_velocity_rad_s']*1.2:
                    raise MeasurementFailure('observed_velocity_limit')
                distances=profile.check(latest['q'])['distances'][0]
                surplus=distances-profile.pair_clearance_m-profile.radii@np.full(14,self.config['workflow']['geometry_allowance_rad'])
                record['online_distance_checks']=record.get('online_distance_checks',0)+1
                record['minimum_online_surplus_m']=min(record.get('minimum_online_surplus_m',float('inf')),float(surplus.min()))
                if np.any(surplus<=0):raise MeasurementFailure('online_observed_clearance_rejected')
        self.commanded=np.asarray(goal).copy();record['end_ns']=self.device.now
        chunks=self.device.raw[max(0,raw_begin-1):]
        positions=np.concatenate([x['q'] for x in chunks]);times=np.concatenate([x['sample_ns'] for x in chunks])*1e-9
        receives=np.concatenate([x['receive_ns'] for x in chunks]);selected=(times>=begin*1e-9)&(receives<=self.device.now)
        positions=positions[selected];times=times[selected]
        result=certify_polyline(observed_profile(profile,self.config['workflow']['geometry_allowance_rad']),positions,times,
            clearance_m=self.config['workflow']['clearance_m'],max_gap_s=self.config['drive']['sample_period_ns']*1.5e-9)
        record['observed']=result
        if not result['passed']:raise MeasurementFailure('collision_observed_rejected:'+label)

    def hold(self,duration,label='settle'):
        self.execute(self.commanded,self.contact or self.free,label,duration)

    def wait_window(self,after):
        deadline=self.device.now+round(self.config['workflow']['measurement_timeout_s']*1e9)
        while self.device.now<deadline:
            for packet in self.device.indicators.values():
                if packet['status'] in (2,3,4):raise MeasurementFailure('indicator_fault_'+str(packet['status']))
            arrays=self.device.arrays();window=stable_window(arrays,np.array(self.device.row_times),self.device.now,self.config,after)
            if window is not None:return window
            self.hold(.04,'wait_stable')
            if self.device.indicators and any(self.device.now-p['receive_ns']>self.config['workflow']['max_indicator_age_ns'] for p in self.device.indicators.values()):
                raise MeasurementFailure('indicator_feedback_timeout')
        raise MeasurementFailure('stable_window_timeout')

    def visit(self, visit):
        frame,ball=visit['frame'],visit['ball_id'];goal=np.array(visit['nominal_q_rad']);target=ball_world(self.config,ball)
        transform=self.engine.frame_pose(goal,frame)
        self.stage=solve_position(self.engine,frame,target-transform[:3,:3]@np.full(3,.06),goal)
        self.contact=fixture_profile(self.engine,self.config,(frame,ball),True)
        self.device.select(frame,ball,target);self.event('preflight',visit=visit['id'])
        for start,end,profile,label in [(self.commanded,self.stage,self.free,'approach'),(self.stage,goal,self.contact,'insert'),
                (goal,self.stage,self.contact,'withdraw'),(self.stage,self.home,self.free,'return_home')]:
            _,gate=segment(profile,start,end,self.config,label)
            self.event('preflight_segment',segment=label,passed=gate['passed'])
            if not gate['passed']:raise MeasurementFailure('complete_visit_preflight_rejected:'+label)
        self.execute(self.stage,self.free,'approach')
        self.execute(goal,self.contact,'insert',self.config['workflow']['insertion_s'])
        after=self.device.now;self.hold(self.config['workflow']['settle_s'])
        for iteration in range(self.config['workflow']['maximum_corrections']+1):
            window=self.wait_window(after)
            offset=_core.trical_offset(np.array(window['readings_m']),calibration(self.config))
            self.event('read_offset',iteration=iteration,offset_sensor_m=offset.tolist())
            if np.max(abs(offset))<=self.config['workflow']['centering_tolerance_m']:
                self.measurements.append(dict(visit=visit['id'],frame=frame,ball_id=ball,window=window,offset_sensor_m=offset.tolist(),stable_after_ns=after,accepted_ns=self.device.now,
                    experiment_conditions={k:visit[k] for k in ('split','batch_id','candidate_id') if k in visit}))
                self.event('sample_accepted',visit=visit['id']);break
            if iteration==self.config['workflow']['maximum_corrections']:raise MeasurementFailure('centering_iteration_limit')
            observed=np.array(window['q_rad'])
            # Integrate the measured Cartesian correction into the LAST reference.
            # Replacing it with observed q would reintroduce the same steady servo
            # error on every iteration and prevent convergence.
            corrected=self.commanded+(local_correction(self.engine,frame,observed,offset,self.config)-observed)
            _,escape=segment(self.contact,corrected,self.stage,self.config,'correction_escape')
            if not escape['passed']:raise MeasurementFailure('correction_escape_rejected')
            self.execute(corrected,self.contact,'correct',self.config['workflow']['correction_s'])
            after=self.device.now;self.hold(self.config['workflow']['settle_s'])
        self.execute(self.stage,self.contact,'withdraw')
        self.contact=None
        if visit.get('return_home',True):self.execute(self.home,self.free,'return_home')
        self.stage=None

    def recover(self):
        self.event('abort_hold')
        try:
            if not self.device.command_times:
                self.event('aborted_before_motion');return True
            if self.device.latest is None:raise MeasurementFailure('no_joint_feedback_for_recovery')
            if self.device.now-int(self.device.latest['receive_ns'])>self.config['workflow']['max_joint_age_ns'] or np.any(self.device.latest['status']!=0):
                raise MeasurementFailure('invalid_joint_feedback_for_recovery')
            self.commanded=self.device.latest['q'].copy()
            self.hold(.2,'fault_hold')
            if self.stage is not None:self.execute(self.stage,self.contact,'fault_withdraw')
            self.contact=None;self.execute(self.home,self.free,'fault_return_home')
            self.event('aborted_at_home');return True
        except MeasurementFailure as error:
            self.event('latched_hold_manual_recovery_required',reason=str(error));return False


def run_mock(folder, config=None, *, fault=None, first_only=False):
    started=perf_counter()
    folder=Path(folder)
    if folder.exists():raise FileExistsError(folder)
    config=workflow_config() if config is None else deepcopy(config)
    hidden=config.pop('mock');engine,manifest=nominal_kinematics(config)
    folder.mkdir(parents=True,exist_ok=False);write_json(folder/'profile.json',config);write_json(folder/'geometry.json',manifest)
    device=make_mock(engine,config,hidden);device.journal=folder/'journal';controller=MeasurementController(engine,config,device)
    visits=config['workflow']['visits'][:1] if first_only else config['workflow']['visits'];failure=None;recovered=None
    if fault is not None:
        if fault not in ('disconnected','frozen','out_of_range','no_contact'):raise ValueError('Unknown instrument fault')
        device.fault(fault)
    try:
        for visit in visits:controller.visit(visit)
    except MeasurementFailure as error:
        failure=str(error);controller.event('rejected',reason=failure);recovered=controller.recover()
    device.flush_journal();device.export(folder/'raw');device.export_evaluation(folder/'evaluation')
    arrays=device.arrays()
    if arrays is not None:
        metadata=observation_metadata(manifest,config,run_id=folder.name,source='cpp_persistent_trical_mock',
            clocks=dict(joint_sample='virtual_ns',indicator_sample=['virtual_ns' if config['sensor']['expose_sample_time'] else 'unknown']*3,receive='virtual_ns'))
        save_observations(folder/'observations',arrays,metadata)
    write_json(folder/'events.json',controller.events);write_json(folder/'motions.json',controller.motions)
    write_json(folder/'measurements.json',controller.measurements)
    write_json(folder/'evaluation/mock.json',hidden)
    passed=failure is None and len(controller.measurements)==len(visits)
    report=dict(schema_version=1,stage='K2',passed=passed,synthetic_only=True,hardware_executable=False,
        calibration_performed=False,visits_requested=len(visits),measurements=len(controller.measurements),failure=failure,recovered_to_home=recovered,
        command_frequency_hz=1e9/config['control_period_ns'],indicator_frequency_hz=1e9/config['sensor']['period_ns'],
        native_sha256=sha256(Path(_core.__file__).read_bytes()).hexdigest(),geometry_sha256=manifest['geometry_sha256'],
        contact_pairs_policy='Only three named tips vs current target ball, bounded 5mm stroke; all other pairs retain clearance',
        no_contact_load_model=True,wall_time_not_realtime=True,truth_used_by_controller=False,
        virtual_duration_ns=device.now,wall_s=perf_counter()-started,
        pending=['K3 geometry identification / observability / MDH-Hayati','K4 full acceptance and video','K5 real devices'])
    write_json(folder/'report.json',report)
    sealed=[p for p in folder.rglob('*') if p.is_file() and p.parts[len(folder.parts)] not in ('evaluation','analysis') and p.name!='report.json']
    write_json(folder/'manifest.json',dict(schema_version=1,native_sha256=report['native_sha256'],
        files={str(p.relative_to(folder)):sha256(p.read_bytes()).hexdigest() for p in sorted(sealed)}))
    return report
