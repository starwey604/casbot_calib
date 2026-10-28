"""Mock-only boundary: controller receives delivered observations, never plant state."""
import numpy as np
from . import _core
from .kinematics import calibration


def make_mock(engine, config, hidden):
    if np.max(np.abs(hidden['encoder_offset_rad']))+6*config['drive']['position_noise_rad']>config['workflow']['geometry_allowance_rad']:
        raise ValueError('Mock encoder/noise exceeds declared geometric uncertainty envelope')
    plant=engine
    if 'geometry_scaled' in hidden:
        from .kinematic_parameters import KinematicParameters,inverse
        from .trical_fixture import fixture_profile
        from scipy.spatial.transform import Rotation
        parameters=KinematicParameters(engine);plant=parameters.build(hidden['geometry_scaled'])
        if hidden.get('geometry_chart_sha256',parameters.chart['sha256'])!=parameters.chart['sha256']:
            raise ValueError('Hidden geometry chart does not match the frozen nominal parameter convention')
        profile=fixture_profile(engine,config);nominal=engine.model;actual=plant.model
        changes=[inverse(a.origin.matrix())@b.origin.matrix() for a,b in zip(nominal.joints,actual.joints,strict=True)]
        maximum=0.
        for geometry in profile.geometry:
            if geometry.parent<0:continue
            radius=float(np.linalg.norm(geometry.vertices@geometry.placement.rotation.T+geometry.placement.translation,axis=1).max());error=0.
            j=geometry.parent
            while j>=0:
                delta=changes[j];angle=Rotation.from_matrix(delta[:3,:3]).magnitude()
                error+=float(np.linalg.norm(delta[:3,3])+2*radius*np.sin(angle/2))
                radius+=np.linalg.norm(nominal.joints[j].origin.translation);j=nominal.parents[j]
            maximum=max(maximum,error)
        if maximum>config['workflow'].get('geometry_error_budget_m',0.):
            raise ValueError('Mock rigid geometry exceeds the declared all-configuration displacement budget')
    drive=config['drive'];arm=_core.ArmSimulationConfig()
    for key in ('integration_ns','sample_period_ns','current_tau','command_delay_ns','feedback_delay_ns'):
        setattr(arm,key,drive[key])
    arm.torque_limit=drive['torque_limit_nm'];arm.gain=np.full(14,drive['gain_nm_a'])
    arm.position_noise=drive['position_noise_rad'];arm.current_noise=drive['current_noise_a'];arm.seed=hidden['seed']
    sensor=_core.TricalMockConfig();sensor.calibration=calibration(config)
    for key,value in config['sensor'].items():setattr(sensor,key,value)
    sensor.minimum=config['indicators']['minimum_readings_m'];sensor.maximum=config['indicators']['maximum_readings_m']
    sensor.encoder_offset=np.asarray(hidden['encoder_offset_rad']);sensor.bias=hidden['bias_m'];sensor.seed=hidden['seed']+1
    session=_core.TricalMockSession(plant,arm,sensor,np.array(config['workflow']['home_rad']),np.zeros(14),True)
    return MockDevice(session)


class MockDevice:
    def __init__(self, session):
        self._session=session;self.pending=[];self.latest=None;self.indicators={};self.packets=[]
        self.raw=[];self._evaluation=[];self.command_times=[];self.command_values=[];self.rows=[];self.row_times=[]
        self._snapshot_sequences=None
        self.journal=None;self._journal_time=0;self._journal_offsets=(0,0,0);self._journal_index=0

    @property
    def now(self):return self._session.time_ns

    def select(self, frame, target, ball):
        self._session.select_target(frame,target,ball);self.indicators={};self._snapshot_sequences=None
        self.frame,self.target=frame,target

    def fault(self, kind):self._session.set_fault(1,kind)

    def submit(self, time, values):
        self._session.submit(time,values);self.command_times.append(time);self.command_values.append(values.copy())

    def advance(self, end):
        observations,packets,evaluation=self._session.advance_to(int(end))
        if len(observations['q']):
            self.raw.append(observations)
            for i in range(len(observations['q'])):
                self.pending.append({key:array[i].copy() for key,array in observations.items()})
        if len(evaluation['q']):self._evaluation.append(evaluation)
        self.pending.sort(key=lambda row:int(row['receive_ns']))
        while self.pending and self.pending[0]['receive_ns']<=end:
            sample=self.pending.pop(0)
            if self.latest is None or sample['sequence']>self.latest['sequence']:self.latest=sample
        for packet in packets:
            self.packets.append(dict(packet))
            if packet['target_id']==self.target and packet['tool_frame']==self.frame:
                self.indicators[packet['channel']]=dict(packet)
        if self.latest is not None and len(self.indicators)==3:
            sequences=tuple(self.indicators[i]['sequence'] for i in range(3))
            if self._snapshot_sequences is None or all(a!=b for a,b in zip(sequences,self._snapshot_sequences)):
                self._snapshot_sequences=sequences;state=self.latest
                self.rows.append(dict(q_rad=state['q'].copy(),dq_rad_s=state['dq'].copy(),joint_sequence=int(state['sequence']),
                    joint_sample_ns=int(state['sample_ns']),joint_receive_ns=int(state['receive_ns']),joint_sample_known=True,
                    indicator_readings_m=np.array([self.indicators[i]['reading_m'] for i in range(3)]),
                    indicator_sequence=np.array(sequences,dtype=np.int64),
                    indicator_sample_ns=np.array([self.indicators[i]['sample_ns'] for i in range(3)],dtype=np.int64),
                    indicator_receive_ns=np.array([self.indicators[i]['receive_ns'] for i in range(3)],dtype=np.int64),
                    indicator_sample_known=np.array([self.indicators[i]['sample_known'] for i in range(3)],dtype=bool),
                    indicator_status=np.array([self.indicators[i]['status'] for i in range(3)],dtype=np.uint8),
                    ball_id=self.target,tool_frame=self.frame))
                self.row_times.append(end)
        if self.journal is not None and end-self._journal_time>=1_000_000_000:self.flush_journal()

    def flush_journal(self):
        if self.journal is None:return
        from pathlib import Path
        root=Path(self.journal);root.mkdir(parents=True,exist_ok=True);r,p,c=self._journal_offsets;data={}
        chunks=self.raw[r:]
        if chunks:data.update({'joint_'+key:np.concatenate([a[key] for a in chunks]) for key in chunks[0]})
        packets=self.packets[p:]
        if packets:data.update({'indicator_'+key:np.asarray([a[key] for a in packets]) for key in packets[0]})
        data['command_send_ns']=np.asarray(self.command_times[c:],dtype=np.int64)
        data['commands']=np.asarray(self.command_values[c:]).reshape(-1,14,5)
        if len(chunks) or len(packets) or len(data['command_send_ns']):
            path=root/f'{self._journal_index:06d}.npz'
            if path.exists():raise FileExistsError(path)
            np.savez_compressed(path,**data);self._journal_index+=1
        self._journal_time=self.now;self._journal_offsets=(len(self.raw),len(self.packets),len(self.command_times))

    def arrays(self):
        if not self.rows:return None
        result={key:np.asarray([r[key] for r in self.rows]) for key in self.rows[0]}
        for key in ('joint_sequence','joint_sample_ns','joint_receive_ns'):result[key]=result[key].astype(np.int64)
        return result

    def export(self, root):
        from pathlib import Path
        root=Path(root);root.mkdir(parents=True,exist_ok=False)
        if self.raw:np.savez_compressed(root/'joint_packets.npz',**{k:np.concatenate([a[k] for a in self.raw]) for k in self.raw[0]})
        if self.packets:np.savez_compressed(root/'indicator_packets.npz',**{k:np.asarray([a[k] for a in self.packets]) for k in self.packets[0]})
        np.savez_compressed(root/'commands.npz',send_ns=np.array(self.command_times,dtype=np.int64),commands=np.array(self.command_values).reshape(-1,14,5))
        np.savez_compressed(root/'row_decision_times.npz',time_ns=np.array(self.row_times,dtype=np.int64))

    def export_evaluation(self, root):
        from pathlib import Path
        root=Path(root);root.mkdir(parents=True,exist_ok=False)
        if self._evaluation:np.savez_compressed(root/'plant_truth.npz',**{k:np.concatenate([a[k] for a in self._evaluation]) for k in self._evaluation[0]})
