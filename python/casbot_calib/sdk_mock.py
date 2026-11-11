"""Single SDK-shaped Mock launcher over the existing native physical host."""
from contextlib import AbstractContextManager
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
import numpy as np
import yaml
from .data import write_json
from .model_runtime import model_for, collision_for, context_for, dynamics_scene, urdf_for, check_same_context
from .hardware_field import load_profile, seal
from .sdk_connection import mock_connection
from .vendor_identity import sha256


def mock_config(path):
    config = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    purpose=config.setdefault('purpose','calibration')
    if purpose not in ('calibration','manual_tuning'):raise ValueError('Unknown SDK Mock purpose')
    protection=config.setdefault('protection_gravity',True)
    if type(protection) is not bool:raise ValueError('Explicit protection_gravity boolean required')
    if purpose=='manual_tuning' and (protection or config['internal_gravity']):raise ValueError('Manual tuning does not add gravity compensation')
    if config.get('schema_version') != 'casbot_sdk_mock_v1': raise ValueError('Explicit SDK Mock schema required')
    for key in ('control_hz', 'publish_hz'):
        if type(config[key]) is not int or not 100 <= config[key] <= 2000: raise ValueError('Invalid Mock rate')
    for key in ('cache_period_ns', 'interpolation_ns', 'watchdog_ns'):
        if type(config[key]) is not int or not 1000000 <= config[key] <= 1000000000: raise ValueError('Invalid Mock timing')
    if config['watchdog_ns'] < 10000000: raise ValueError('Mock watchdog must be at least10ms')
    for key in ('internal_gravity', 'allow_effort', 'allow_custom_pd'):
        if type(config[key]) is not bool: raise ValueError('Explicit Mock capability booleans required')
    if not np.isfinite(config['effort_limit_nm']) or not 0 < config['effort_limit_nm'] <= 100: raise ValueError('Invalid Mock effort limit')
    q = np.asarray(config['initial_q'], dtype=float)
    if q.shape != (14,) or not np.isfinite(q).all(): raise ValueError('Finite14-axis initial pose required')
    engine, _ = model_for(config)
    if any(not j.lower <= x <= j.upper for x, j in zip(q, engine.model.joints)): raise ValueError('Initial pose outside nominal limits')
    context=context_for(config)
    if purpose=='calibration':
        scene = collision_for(config,scene=dynamics_scene(context) if context else None)
        if np.min(scene.check(q[None, :])['distances']) <= .01: raise ValueError('Initial SDK Mock pose collides or lacks10mm clearance')
    for key in ('servo_natural_frequency_rad_s', 'servo_damping_ratio','servo_minimum_kp','servo_minimum_kd'):
        if not np.isfinite(config[key]) or config[key] <= 0: raise ValueError('Positive synthetic servo parameter required')
    plant = config['plant']
    for key in ('control_hz', 'feedback_hz'):
        if type(plant[key]) is not int or not 100 <= plant[key] <= 2000: raise ValueError('Invalid native host rate')
    if type(plant['zero_gravity']) is not bool or plant['zero_gravity']: raise ValueError('SDK Mock uses nominal installation gravity')
    if not np.isfinite(plant['current_tau']) or plant['current_tau'] < 0: raise ValueError('Invalid drive response')
    for key in ('drive_delay_ns', 'feedback_delay_ns'):
        if type(plant[key]) is not int or not 0 <= plant[key] <= 100000000: raise ValueError('Invalid native delay')
    if 'stiction_enabled' in plant and type(plant['stiction_enabled']) is not bool: raise ValueError('Boolean synthetic stiction required')
    if plant.get('device_pd') is not True: raise ValueError('SDK five-field contract requires device-local PD')
    if 'inertia_scales' in plant:
        values=np.asarray(plant['inertia_scales'],dtype=float)
        if values.shape!=(14,) or not np.isfinite(values).all() or np.any(values<.1) or np.any(values>5.):raise ValueError('14 bounded synthetic inertia scales required')
    if 'sample_period_ns' in plant and (type(plant['sample_period_ns']) is not int or plant['sample_period_ns']<50000): raise ValueError('Invalid private raw sample period')
    if 'integration_ns' in plant and (type(plant['integration_ns']) is not int or not 25000<=plant['integration_ns']<=200000):raise ValueError('Bounded Mock integration step required')
    for key in ('friction_viscous','friction_coulomb','friction_static'):
        if key in plant:
            values=np.asarray(plant[key])
            if values.shape!=(14,) or not np.isfinite(values).all() or np.any(values<0): raise ValueError('14 nonnegative synthetic friction parameters required')
    if np.any(np.asarray(plant.get('friction_static',[0]*14))<plant.get('friction_coulomb',[0]*14)): raise ValueError('Static friction must exceed Coulomb friction')
    config['model_context']=context.identity
    return config


def stop(process):
    from .sdk_process import stop_process
    return stop_process(process)


class SDKMockSession(AbstractContextManager):
    """Owns only Mock processes. Clients use the generated ordinary H1 profile."""
    def __init__(self, config_path, output, domain_id, *, profile='configs/hardware_field.yaml', endpoint_root=None, recording_pause_file=None, cancel_check=None):
        self.config = mock_config(config_path); self.root = Path(output).resolve()
        self.recording_pause_file=recording_pause_file;self.cancel_check=cancel_check
        self.domain_id = domain_id; self.endpoint = endpoint_root or '/casbot_calib_s2_sdk_' + uuid.uuid4().hex[:10]
        mock_connection(self.endpoint, domain_id, '0'*64)  # Validate before filesystem/process actions.
        self.profile = load_profile(profile); self.processes = []; self.logs = []
        check_same_context(self.config,self.profile)
        clearance = self.profile['scene']['clearance_m']
        if self.config['purpose']=='calibration' and np.min(collision_for(self.profile).check(self.config['initial_q'])['distances']) <= clearance:
            raise ValueError('Initial SDK Mock pose violates the configured collision scene')
        if 'model_context' in self.config and not np.allclose(self.profile['gravity_world'],model_for(self.config)[0].model.gravity_world,rtol=0,atol=1e-12):
            raise ValueError('SDK Mock and client installation gravity differ')
        self.client_profile = self.root / 'client_profile.yaml'

    def __enter__(self):
        self.root.mkdir(parents=True, exist_ok=False)
        engine, source = model_for(self.config); model = engine.model
        q = np.asarray(self.config['initial_q'])
        omega = self.config['servo_natural_frequency_rad_s']; damping = self.config['servo_damping_ratio']
        from .control_gains import nominal_pd_gains
        kp,kd=nominal_pd_gains(engine,q,omega,damping,self.config['servo_minimum_kp'],self.config['servo_minimum_kd'])
        if np.max(kp)>1000 or np.max(kd)>100: raise ValueError('Derived synthetic gains exceed declared range')
        server = Path(os.environ.get('CASBOT_VENDOR_INSTALL', 'install/vendor')).resolve()/'casbot_calib_vendor/lib/casbot_calib_vendor/sdk_mock_server'
        if not server.is_file(): raise FileNotFoundError('Build optional SDK packages first: ' + str(server))
        protocol = self.root/'protocol'; protocol.mkdir(); plant_dir = self.root/'evaluation/plant'; plant_dir.mkdir(parents=True)
        server_dir = self.root/'evaluation/server'; server_dir.mkdir()
        model_path = str(urdf_for(self.config))
        context=context_for(self.config)
        model_id = context.model_id
        device_params = dict(self.config['plant'])
        device_params['plant_control_hz'] = device_params.pop('control_hz')
        device_params['plant_feedback_hz'] = device_params.pop('feedback_hz')
        device_params.update(initial_q=q.tolist(),plant_output=str(plant_dir))
        keys = ('control_hz','publish_hz','cache_period_ns','interpolation_ns','watchdog_ns','internal_gravity','protection_gravity','allow_effort','allow_custom_pd','effort_limit_nm')
        server_params = dict(experiment_id='sdk_mock',model_id=model_id,model_path=model_path,
            output=str(server_dir),endpoint_root=self.endpoint,joint_names=[j.name for j in model.joints],
            kp=kp.tolist(),kd=kd.tolist(),**{k:self.config[k] for k in keys},**device_params)
        if self.recording_pause_file:server_params['recording_pause_file']=str(self.recording_pause_file)
        write_json(protocol/'behavior.json', dict(schema_version='casbot_sdk_mock_backend_v2',config=self.config,
            initial_pose_collision_qualified=False,initial_pose_policy='operator_specified' if self.config['purpose']=='manual_tuning' else 'calibration_collision_check',
            backend='in_process_shared_SimulatedArmDevice',internal_ros_bridge=False,source_model_sha256=source['sha256'],
            server_sha256=sha256(server),server_params=server_params,
            **({'model_context':context.identity} if context else {}),
            assumptions=self.config['assumptions'],physical_device_semantics_verified=False,
            public_feedback_sample_time_known=False,stop_service_available=False,
            internal_diagnostics_not_for_identification=True))
        seal(protocol)
        self.connection = mock_connection(self.endpoint, self.domain_id, sha256(protocol/'manifest.json'))
        profile = dict(self.profile); profile.pop('scene')
        profile.update(collision_scene='scene.yaml', connection=self.connection)
        (self.root/'scene.yaml').write_text(yaml.safe_dump(self.profile['scene']), encoding='utf-8')
        self.client_profile.write_text(yaml.safe_dump(profile), encoding='utf-8')
        from .sdk_process import ros_arguments
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain_id), ROS_LOCALHOST_ONLY='1')
        try:
            for name, command in [('server',ros_arguments(server,server_params,{'__ns':self.endpoint}))]:
                stream=(self.root/(name+'.log')).open('x');self.logs.append(stream)
                self.processes.append(subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT))
            deadline=time.monotonic()+30
            while True:
                if self.cancel_check and self.cancel_check():raise RuntimeError('本地Mock启动已取消')
                if any(p.poll() is not None for p in self.processes): raise RuntimeError('SDK Mock process exited; inspect logs')
                events=server_dir/'server_events.csv'
                if events.exists() and 'settled_initial_hold' in events.read_text(): break
                if time.monotonic()>deadline: raise TimeoutError('SDK Mock did not settle its initial hold')
                time.sleep(.05)
            write_json(self.root/'ready.json',dict(connection=self.connection,client_profile=str(self.client_profile),
                public_fields='SDK messages only; no acquisition timestamps, device sequences or current',
                physical_device_tested=False))
            return self
        except BaseException:
            self.__exit__(None,None,None);raise

    def __exit__(self, exc_type, exc_value, traceback):
        for process in reversed(self.processes):stop(process)
        for stream in self.logs:stream.close()
        if self.processes:
            write_json(self.root/'shutdown.json',dict(returncodes=[p.returncode for p in self.processes],
                physical_device_tested=False, fault_recovery_performed=False))
        return False


def serve_mock(config, output, domain_id, *, profile='configs/hardware_field.yaml', duration=None):
    if duration is not None and (not np.isfinite(duration) or duration<=0): raise ValueError('Positive Mock duration required')
    with SDKMockSession(config,output,domain_id,profile=profile) as session:
        print('SDK Mock已就绪；H1–H4共用此profile：'+str(session.client_profile),flush=True)
        deadline=time.monotonic()+duration if duration is not None else float('inf')
        try:
            while time.monotonic()<deadline:
                if any(p.poll() is not None for p in session.processes): raise RuntimeError('SDK Mock process exited')
                time.sleep(.2)
        except KeyboardInterrupt: pass
    return dict(passed=all(p.returncode==0 for p in session.processes), physical_device_tested=False)
