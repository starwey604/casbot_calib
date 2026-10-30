"""Single SDK-shaped Mock launcher over the existing native physical host."""
from contextlib import AbstractContextManager
from importlib.resources import files
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import numpy as np
import yaml
from .data import write_json
from .dual_model import nominal_dual_arm
from .hardware_field import load_profile, seal
from .sdk_connection import mock_connection
from .vendor_identity import sha256


def mock_config(path):
    config = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
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
    engine, _ = nominal_dual_arm()
    if any(not j.lower <= x <= j.upper for x, j in zip(q, engine.model.joints)): raise ValueError('Initial pose outside nominal limits')
    from .collision import nominal_profile
    scene = nominal_profile()
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
    if type(plant.get('device_pd')) is not bool: raise ValueError('Explicit Mock PD backend required')
    if 'sample_period_ns' in plant and (type(plant['sample_period_ns']) is not int or plant['sample_period_ns']<50000): raise ValueError('Invalid private raw sample period')
    if 'integration_ns' in plant and (type(plant['integration_ns']) is not int or not 25000<=plant['integration_ns']<=200000):raise ValueError('Bounded Mock integration step required')
    for key in ('friction_viscous','friction_coulomb','friction_static'):
        if key in plant:
            values=np.asarray(plant[key])
            if values.shape!=(14,) or not np.isfinite(values).all() or np.any(values<0): raise ValueError('14 nonnegative synthetic friction parameters required')
    if np.any(np.asarray(plant.get('friction_static',[0]*14))<plant.get('friction_coulomb',[0]*14)): raise ValueError('Static friction must exceed Coulomb friction')
    return config


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=10)
        except subprocess.TimeoutExpired: process.kill(); process.wait()


class SDKMockSession(AbstractContextManager):
    """Owns only Mock processes. Clients use the generated ordinary H1 profile."""
    def __init__(self, config_path, output, domain_id, *, profile='configs/hardware_field.yaml', endpoint_root=None):
        self.config = mock_config(config_path); self.root = Path(output).resolve()
        self.domain_id = domain_id; self.endpoint = endpoint_root or '/casbot_calib_s2_sdk_' + uuid.uuid4().hex[:10]
        mock_connection(self.endpoint, domain_id, '0'*64)  # Validate before filesystem/process actions.
        self.profile = load_profile(profile); self.processes = []; self.logs = []
        from .collision import nominal_profile
        clearance = self.profile['scene']['clearance_m']
        if np.min(nominal_profile(scene=self.profile['scene']).check(self.config['initial_q'])['distances']) <= clearance:
            raise ValueError('Initial SDK Mock pose violates the configured collision scene')
        self.client_profile = self.root / 'client_profile.yaml'

    def __enter__(self):
        self.root.mkdir(parents=True, exist_ok=False)
        engine, source = nominal_dual_arm(); model = engine.model
        q = np.asarray(self.config['initial_q'])
        omega = self.config['servo_natural_frequency_rad_s']; damping = self.config['servo_damping_ratio']
        from .control_gains import nominal_pd_gains
        kp,kd=nominal_pd_gains(engine,q,omega,damping,self.config['servo_minimum_kp'],self.config['servo_minimum_kd'])
        if np.max(kp)>1000 or np.max(kd)>100: raise ValueError('Derived synthetic gains exceed declared range')
        native = Path('install/humble-ros/casbot_calib_transport/lib/casbot_calib_transport/mock_host').resolve()
        server = Path(os.environ.get('CASBOT_VENDOR_INSTALL', 'install/vendor')).resolve()/'casbot_calib_vendor/lib/casbot_calib_vendor/sdk_mock_server'
        for binary in (native, server):
            if not binary.is_file(): raise FileNotFoundError('Build native ROS and optional SDK packages first: ' + str(binary))
        protocol = self.root/'protocol'; protocol.mkdir(); plant_dir = self.root/'evaluation/plant'; plant_dir.mkdir(parents=True)
        server_dir = self.root/'evaluation/server'; server_dir.mkdir()
        model_path = str(files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf'))
        model_id = 'casbot_20251015_nominal:' + source['sha256'][:12]
        common = dict(experiment_id='sdk_mock', model_id=model_id, model_path=model_path)
        host_params = dict(common, output_dir=str(plant_dir), initial_q=q.tolist(), **self.config['plant'])
        keys = ('control_hz','publish_hz','cache_period_ns','interpolation_ns','watchdog_ns','internal_gravity','allow_effort','allow_custom_pd','effort_limit_nm')
        server_params = dict(common, output=str(server_dir), endpoint_root=self.endpoint,
            joint_names=[j.name for j in model.joints], kp=kp.tolist(), kd=kd.tolist(), **{k:self.config[k] for k in keys})
        write_json(protocol/'behavior.json', dict(config=self.config, source_model_sha256=source['sha256'],
            host_sha256=sha256(native), server_sha256=sha256(server), host_params=host_params, server_params=server_params,
            assumptions=self.config['assumptions'], physical_device_semantics_verified=False,
            public_feedback_sample_time_known=False, stop_service_available=False,
            internal_diagnostics_not_for_identification=True))
        seal(protocol)
        self.connection = mock_connection(self.endpoint, self.domain_id, sha256(protocol/'manifest.json'))
        profile = dict(self.profile); profile.pop('scene')
        profile.update(collision_scene='scene.yaml', connection=self.connection)
        (self.root/'scene.yaml').write_text(yaml.safe_dump(self.profile['scene']), encoding='utf-8')
        self.client_profile.write_text(yaml.safe_dump(profile), encoding='utf-8')
        def args(binary, params, namespace=None):
            command = [str(binary), '--ros-args']
            if namespace: command += ['-r', '__ns:='+namespace]
            for key, value in params.items():
                command += ['-p', key+':='+ (json.dumps(value) if isinstance(value,(list,bool)) else str(value))]
            return command
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain_id), ROS_LOCALHOST_ONLY='1')
        try:
            for name, command in [('host', args(native,host_params,self.endpoint+'/plant')), ('server',args(server,server_params,self.endpoint))]:
                stream=(self.root/(name+'.log')).open('x');self.logs.append(stream)
                self.processes.append(subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT))
            deadline=time.monotonic()+30
            while True:
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
        print('SDK Mock已就绪；H1/H2使用此profile：'+str(session.client_profile),flush=True)
        deadline=time.monotonic()+duration if duration is not None else float('inf')
        try:
            while time.monotonic()<deadline:
                if any(p.poll() is not None for p in session.processes): raise RuntimeError('SDK Mock process exited')
                time.sleep(.2)
        except KeyboardInterrupt: pass
    return dict(passed=all(p.returncode==0 for p in session.processes), physical_device_tested=False)
