"""S2 observed-start planning, immutable input guards and local SDK execution."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import yaml
from .vendor_data import load_config
from .vendor_identity import sha256


def position_config(path):
    config=yaml.safe_load(Path(path).read_text())
    return validate_position_config(config)


def validate_position_config(config):
    if config.get('schema_version')!='casbot_vendor_s2_v1' or config.get('scope')!='localhost_contract_mock' or config.get('hardware_execution_enabled') is not False:
        raise ValueError('S2 currently qualifies isolated local execution only; hardware prerequisites are unresolved')
    if config['control_hz'] not in (100,250,500): raise ValueError('S2 frequencies are 100,250,500Hz')
    required=dict(arm_subset='arms_only_14_no_waist_head_hand',waist_drive='excluded_from_commands',
                  default_pd='confirmed_in_local_bridge_only',time_ref='zero_latest_in_local_bridge_only',
                  stop='explicit_mock_stop_service_holds_current_pose',watchdog='mock_bridge_100ms_then_hold',
                  mode_exit='only_after_observed_rest_at_start')
    if config.get('mock_contract')!=required: raise ValueError('Unconfirmed subset/waist/PD/time/stop semantics')
    amplitude=np.asarray(config['amplitude_rad'],dtype=float)
    if amplitude.shape!=(14,) or not np.isfinite(amplitude).all() or not np.any(amplitude): raise ValueError('14 finite excitation amplitudes required')
    for key in ('initial_hold_s','transition_s','target_hold_s','final_hold_s','velocity_limit_rad_s',
                'acceleration_limit_rad_s2','tracking_guard_rad','collision_allowance_rad','start_tolerance_rad',
                'max_feedback_age_ns','max_mode_age_ns','transition_timeout_ns','vel_scale'):
        if not np.isfinite(config[key]) or config[key]<=0: raise ValueError('Positive finite '+key+' required')
    if config['collision_allowance_rad']<=config['tracking_guard_rad'] or config['vel_scale']>1:
        raise ValueError('Collision allowance must include tracking and stopping reserve')
    if config['start_tolerance_rad']>config['tracking_guard_rad']:raise ValueError('Start tolerance must be inside the tracking guard')
    return config


def endpoint_params(config, endpoint_root):
    if not endpoint_root.startswith('/casbot_calib_s2_') or '//' in endpoint_root:
        raise ValueError('S2 requires a private endpoint root')
    return dict(joint_topic=endpoint_root+'/joint_states',mode_topic=endpoint_root+'/status',
                command_topic=endpoint_root+'/upper_cmd',mode_service=endpoint_root+'/upper_debug',stop_service=endpoint_root+'/stop')


def executor_binary():
    prefix=Path(os.environ.get('CASBOT_VENDOR_INSTALL','install/vendor')).resolve()
    identity=json.loads((prefix/'vendor-source.json').read_text())
    for name,digest in identity['interfaces'].items():
        if sha256(prefix/'crb_ros_msg/share/crb_ros_msg'/name)!=digest: raise ValueError('Installed SDK interface identity changed')
    return prefix/'casbot_calib_vendor/lib/casbot_calib_vendor/position_executor',identity


def _run(binary, params, output, domain_id, log):
    if not 0<=domain_id<=232: raise ValueError('Explicit valid ROS domain required')
    params=dict(params,output=str(Path(output).resolve()))
    args=[str(binary),'--ros-args']
    for key,value in params.items():
        encoded=json.dumps(value) if isinstance(value,(bool,list)) else str(value)
        args+=['-p',key+':='+encoded]
    with Path(log).open('x') as stream:
        process=subprocess.Popen(args,env=dict(os.environ,ROS_DOMAIN_ID=str(domain_id),ROS_LOCALHOST_ONLY='1'),stdout=stream,stderr=subprocess.STDOUT)
        try: code=process.wait(timeout=60)
        except KeyboardInterrupt:
            process.send_signal(signal.SIGINT);code=process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGTERM)
            try: code=process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill();code=process.wait()
    return code


def _params(config,names,root):
    return dict(endpoint_params(config,root),joint_names=names,command_period_ns=1_000_000_000//config['control_hz'],
                max_feedback_age_ns=config['max_feedback_age_ns'],max_mode_age_ns=config['max_mode_age_ns'],
                transition_timeout_ns=config['transition_timeout_ns'],start_tolerance_rad=config['start_tolerance_rad'],
                tracking_limit_rad=config['tracking_guard_rad'],position_allowance_rad=config['collision_allowance_rad'],
                maximum_velocity_rad_s=config['velocity_limit_rad_s'],vel_scale=config['vel_scale'])


def certify_tracking_tube(profile,curve,duration,bounds,allowance,margin):
    from .collision import certify_reference
    # A constant tracking allowance cannot be reduced by subdividing time.
    # Fail boundedly if this existing conservative certificate cannot resolve it.
    knots=np.unique(np.r_[curve.x,(curve.x[:-1]+curve.x[1:])/2])
    lower=profile.check(curve(knots))['distances']-allowance@profile.radii.T
    if np.min(lower)<=margin:
        return dict(passed=False,reason='tracking_allowance_not_certifiable',minimum_static_lower_bound_m=float(lower.min()),
                    clearance_m=margin,position_allowance_rad=allowance.tolist(),queries=len(knots))
    return certify_reference(profile,curve,duration,bounds,clearance_m=margin,position_allowance=allowance)


def prepare_position(config_path,output,*,endpoint_root,domain_id):
    config=position_config(config_path);observation=load_config(config['observation_config'])
    binary,identity=executor_binary();params=_params(config,observation['joint_names'],endpoint_root)
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    code=_run(binary,dict(params,snapshot_only=True),output/'snapshot',domain_id,output/'snapshot.log')
    if code: raise RuntimeError('Observed-start snapshot failed; retained, no mode request issued')
    raw=np.genfromtxt(output/'snapshot/snapshot.csv',delimiter=',',names=True,ndmin=1)[0]
    q=np.array([raw['q'+str(j)] for j in range(14)]);v=np.array([raw['v'+str(j)] for j in range(14)])
    return plan_position(config,observation,q,v,output,params,identity,sha256(binary),domain_id)


def plan_position(config,observation,q,v,output,params,sdk,binary_sha,domain_id):
    from .collision import nominal_profile,certify_reference,certify_polyline
    from .dual_model import nominal_dual_arm
    from .staged_motion import Motion,derivative_bounds,gate_motion
    output=Path(output);scene=yaml.safe_load(Path(config['collision_scene']).read_text())
    engine,source=nominal_dual_arm();profile=nominal_profile(scene=scene)
    names=[j.name for j in engine.model.joints]
    if names!=observation['joint_names'] or np.asarray(q).shape!=(14,) or not np.isfinite(q).all() or np.max(abs(v))>=.02:
        raise ValueError('Observed start is not a complete, stationary nominal 14-axis pose')
    amplitude=np.asarray(config['amplitude_rad']);motion=Motion('sdk_position_preflight','position',q)
    motion.add(config['initial_hold_s'],q,label='initial_hold')
    for sign in (1,-1):
        motion.add(config['transition_s'],q+sign*amplitude,label='position_move').add(config['target_hold_s'],q+sign*amplitude,label='target_hold')
    motion.add(config['transition_s'],q,label='return_home').add(config['final_hold_s'],q,label='final_hold')
    curve=motion.polynomial();bounds=derivative_bounds(curve);acceleration=derivative_bounds(curve,2)
    envelope=gate_motion(profile,motion,scene['clearance_m'])
    allowance=np.full(14,config['collision_allowance_rad'])
    tracking=certify_tracking_tube(profile,curve,motion.duration,bounds.max(axis=0),allowance,scene['clearance_m'])
    limits=np.array([[j.lower,j.upper] for j in engine.model.joints])
    lower=curve.c.min(axis=(0,1))-allowance;upper=curve.c.max(axis=(0,1))+allowance
    period=1_000_000_000//config['control_hz'];duration=round(motion.duration*1e9)
    times=np.arange(0,duration+1,period,dtype=np.int64)
    if times[-1]!=duration: raise ValueError('Duration must be an integer multiple of all tested control periods')
    positions=curve(times*1e-9);velocities=curve(times*1e-9,nu=1)
    commands=certify_polyline(profile,positions,times*1e-9,clearance_m=scene['clearance_m'])
    checks=dict(reference=envelope['passed'],tracking_and_stop_envelope=tracking['passed'],command_interpolant=commands['passed'],
                travel=bool(np.all(lower>=limits[:,0]) and np.all(upper<=limits[:,1])),
                velocity=bool(np.max(bounds)<=config['velocity_limit_rad_s']),acceleration=bool(np.max(acceleration)<=config['acceleration_limit_rad_s2']))
    report=dict(passed=all(checks.values()),checks=checks,reference=envelope,tracking_envelope=tracking,command_interpolant=commands,
                scope='synthetic nominal geometry and declared bounded tracking/stopping reserve; no hardware certificate')
    (output/'collision_reference.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['passed']: raise ValueError('Complete observed-start reference/envelope rejected; no mode request issued')
    np.savetxt(output/'position_plan.csv',np.column_stack([times,positions,velocities]),delimiter=',',
               header='relative_ns'+''.join(','+key+str(j) for key in ('q','v') for j in range(14)),comments='',fmt=['%d']+['%.17g']*28)
    np.savez_compressed(output/'reference.npz',coefficients=curve.c,breakpoints=curve.x)
    metadata=dict(schema_version='casbot_vendor_s2_plan_v1',config=config,observation=observation,params=params,sdk_identity=sdk,
                  executor_sha256=binary_sha,model_source=source,scene=scene,geometry_sha256=profile.metadata['geometry_sha256'],
                  domain_id=domain_id,initial_q=q.tolist(),motion=motion.metadata(),hardware_executable=False)
    (output/'plan.json').write_text(json.dumps(metadata,indent=2)+'\n')
    files=['position_plan.csv','reference.npz','collision_reference.json','plan.json','snapshot/snapshot.csv','snapshot/result.json']
    (output/'manifest.json').write_text(json.dumps({name:sha256(output/name) for name in files},indent=2)+'\n')
    return metadata


def execute_position(plan,output):
    plan=Path(plan).resolve();metadata=json.loads((plan/'plan.json').read_text())
    # Re-run profile validation, not just a user-written passed flag.
    config=validate_position_config(metadata['config'])
    for name,digest in json.loads((plan/'manifest.json').read_text()).items():
        if sha256(plan/name)!=digest: raise ValueError('Frozen position plan changed')
    binary,identity=executor_binary()
    if sha256(binary)!=metadata['executor_sha256'] or identity!=metadata['sdk_identity']: raise ValueError('Executor or SDK changed since planning')
    if not json.loads((plan/'collision_reference.json').read_text())['passed']: raise ValueError('Reference gate failed')
    params=metadata['params'];endpoint_params(config,params['joint_topic'].rsplit('/',1)[0])
    from .dual_model import nominal_dual_arm
    engine,source=nominal_dual_arm()
    if source!=metadata['model_source']: raise ValueError('Model changed after planning')
    from .collision import nominal_profile,certify_reference,certify_polyline
    from .staged_motion import derivative_bounds
    from scipy.interpolate import BPoly
    scene=yaml.safe_load(Path(config['collision_scene']).read_text())
    if scene!=metadata['scene']: raise ValueError('Collision scene changed after planning')
    profile=nominal_profile(scene=scene)
    if profile.metadata['geometry_sha256']!=metadata['geometry_sha256']:raise ValueError('Collision geometry changed')
    with np.load(plan/'reference.npz') as data:curve=BPoly(data['coefficients'],data['breakpoints'])
    csv=np.genfromtxt(plan/'position_plan.csv',delimiter=',',skip_header=1)
    if not np.allclose(csv[:,1:15],curve(csv[:,0]*1e-9),atol=1e-12,rtol=0):raise ValueError('CSV reference differs from the continuous model')
    gate=certify_tracking_tube(profile,curve,curve.x[-1],derivative_bounds(curve).max(axis=0),
                              np.full(14,config['collision_allowance_rad']),scene['clearance_m'])
    command_gate=certify_polyline(profile,csv[:,1:15],csv[:,0]*1e-9,clearance_m=scene['clearance_m'])
    if not gate['passed'] or not command_gate['passed']:raise ValueError('Pre-execution collision recheck failed')
    limits=np.array([[j.lower,j.upper] for j in engine.model.joints])
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    observation=dict(metadata['observation'],joint_topic=params['joint_topic'],mode_topic=params['mode_topic'],debug_topic='',robot_state_topic='')
    (output/'observation-config.yaml').write_text(yaml.safe_dump(observation))
    with (output/'recorder.log').open('x') as log:
        recorder=subprocess.Popen([sys.executable,'-m','casbot_calib.cli','vendor-capture','--config',str(output/'observation-config.yaml'),
            '--output',str(output/'raw_state'),'--duration','60','--domain-id',str(metadata['domain_id']),'--scope','localhost_contract_mock'],stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+5
            while not (output/'raw_state/raw/recorder.json').exists():
                if recorder.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Read-only recorder did not start; no commands issued')
                time.sleep(.01)
            code=_run(binary,dict(params,snapshot_only=False,plan_file=str(plan/'position_plan.csv'),lower=limits[:,0].tolist(),upper=limits[:,1].tolist()),
                      output/'execution',metadata['domain_id'],output/'process.log')
        finally:
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
                try:recorder.wait(timeout=5)
                except subprocess.TimeoutExpired:recorder.kill();recorder.wait()
    result=json.loads((output/'execution/result.json').read_text()) if (output/'execution/result.json').exists() else dict(passed=False,reason='executor_failed_before_completion')
    result.update(returncode=code,plan=str(plan),plan_manifest_sha256=sha256(plan/'manifest.json'),hardware_executable=False)
    (output/'capture.json').write_text(json.dumps(result,indent=2)+'\n')
    paths=['capture.json','process.log','recorder.log','observation-config.yaml']+[str(p.relative_to(output)) for p in sorted((output/'execution').glob('*'))]
    paths += [str(p.relative_to(output)) for p in sorted((output/'raw_state').rglob('*')) if p.is_file()]
    (output/'manifest.json').write_text(json.dumps({name:sha256(output/name) for name in paths},indent=2)+'\n')
    return result
