"""Immutable plan consumption and batch C++ acquisition; no hardware I/O."""
import hashlib,json
from datetime import datetime,timezone
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.interpolate import BPoly
from . import _core
from .data import write_json
from .collision import certify_polyline
from .dual_model import nominal_dual_arm

NATIVE_BINARY_SHA256=hashlib.sha256(Path(_core.__file__).read_bytes()).hexdigest()


def preserve_incomplete_trial(root,name):
    """Keep interrupted writes outside the immutable accepted-record namespace."""
    root=Path(root);target=root/'runs'/name
    if not target.exists() or (target/'metadata.json').exists():return
    archive=root/'interrupted_records'/(name+'_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f'))
    archive.mkdir(parents=True,exist_ok=False);target.rename(archive/'run')
    evaluation=root/'evaluation'/name
    if evaluation.exists():evaluation.rename(archive/'evaluation')
    write_json(archive/'reason.json',dict(trial=name,reason='interrupted collection has no completion metadata; retained before explicit resume'))
    print(f'保留未完成采集 {name} -> {archive}',flush=True)


def synthetic_plant(config):
    nominal,_=nominal_dual_arm();model=nominal.model;joints=model.joints;mock=config['mock'];rng=np.random.default_rng(mock['seed'])
    for k,joint in enumerate(joints):
        body=joint.body;body.mass*=mock['mass_scales'][k];body.com=body.com+rng.uniform(-1,1,3)*mock['com_offset_m']
        body.inertia_com=body.inertia_com*mock['mass_scales'][k]*mock['inertia_shape_scale'];joint.body=body
        f=joint.friction;f.viscous=mock['viscous_nm_s_rad'];f.coulomb=mock['coulomb_nm'];f.static_friction=mock['static_friction_nm']
        f.smoothing_velocity=.001;f.stribeck_velocity=.002;joint.friction=f
    model.joints=joints;return _core.ArmDynamics(model)


def load_trial(folder,name):
    root=Path(folder);plan=json.loads((root/'plan_report.json').read_text());target=root/'plans'/name
    if (root/'plan_rejected.json').exists():raise ValueError('Plan was rejected by a later audit; collection blocked')
    if not plan['passed'] or name not in plan['selected_trials']:raise ValueError('Trial is not in an accepted complete plan')
    if plan.get('completed_round')=='R1':
        from .staged_protocol import load_protocol
        from .staged_multiconfiguration import verify_motion_file
        protocol,_,_=load_protocol(root);verify_motion_file(root,name,protocol)
    manifest=json.loads((target/'manifest.json').read_text());metadata=json.loads((target/'motion.json').read_text())
    for filename,key in [('motion.json','motion_sha256'),('reference.npz','reference_sha256')]:
        if hashlib.sha256((target/filename).read_bytes()).hexdigest()!=manifest[key]:raise ValueError('Planned trajectory digest mismatch')
    gate=json.loads((target/'collision_reference.json').read_text())
    if 'collision_sha256' in manifest and hashlib.sha256((target/'collision_reference.json').read_bytes()).hexdigest()!=manifest['collision_sha256']:
        raise ValueError('Planned collision evidence changed')
    if not gate['passed'] or not manifest['reference_gate_passed']:raise ValueError('Reference collision gate rejected')
    with np.load(target/'reference.npz',allow_pickle=False) as data:curve=BPoly(data['coefficients'],data['breakpoints'])
    if metadata['name']!=name or not metadata['initial_final_home']:raise ValueError('Invalid home boundary contract')
    return metadata,curve


def collect_trial(folder,name,plant,feedforward,config,profile,*,command_transform=None,integration_ns=None,guard=None):
    root=Path(folder);metadata,curve=load_trial(root,name);duration=metadata['duration_s'];n=14;drive=config['drive']
    session=json.loads((root/'session.json').read_text());planned=session['config']
    if planned!={k:v for k,v in config.items() if k!='mock'}:raise ValueError('Acquisition config changed after planning; create a new plan')
    p0_identity={}
    if config.get('schema_version')==2:
        from .staged_protocol import load_protocol,load_frozen_candidate,file_hash,IDENTITY
        _,_,split_manifest=load_protocol(root)
        from .staged_multiconfiguration import planned_slot
        slot=planned_slot(root,name,split_manifest)
        if slot is None or any(metadata['conditions'].get(k)!=slot[k] for k in IDENTITY):
            raise ValueError('P0 collection identity differs from frozen role manifest')
        p0_identity['protocol_sha256']=file_hash(root/'protocol.json')
        if slot['split']=='test':
            load_frozen_candidate(root)
            p0_identity['candidate_freeze_sha256']=file_hash(root/'candidate_freeze.json')
            if (root/'final_test_protocol.json').exists():
                from .staged_protocol import _unseal
                final=_unseal(root,'final_test_protocol.json')
                if final['candidate_freeze_sha256']!=p0_identity['candidate_freeze_sha256'] or file_hash(Path(final['analysis'])/'final_protocol.json')!=final['final_protocol_sha256']:
                    raise ValueError('Final collection protocol changed')
                p0_identity['final_protocol_sha256']=final['final_protocol_sha256']
    if profile.metadata['geometry_sha256']!=session['collision']['geometry_sha256'] or profile.metadata['scene']!=session['collision']['scene']:
        raise ValueError('Collision geometry/scene changed after planning')
    command_period=metadata['conditions'].get('command_period_ns',config['control_period_ns'])
    times=np.arange(0,round(duration*1e9),command_period,dtype=np.int64);t=times*1e-9
    q,v,a=(curve(t,nu=d) for d in range(3));ff=feedforward.evaluate_batch(q,v,a)['total']
    commands=np.stack([q,v,ff,np.broadcast_to(drive['kp'],q.shape),np.broadcast_to(drive['kd'],q.shape)],axis=2)
    if command_transform is not None:commands=command_transform(t,commands,metadata)
    native=_core.ArmSimulationConfig();native.gain=np.full(n,drive['gain_nm_a']);native.stiction_enabled=True
    native.integration_ns=integration_ns or metadata['conditions'].get('integration_ns',config['integration_ns']);native.sample_period_ns=metadata['conditions'].get('sample_period_ns',config['sample_period_ns'])
    for field,value in dict(current_tau=drive['current_tau'],command_delay_ns=drive['command_delay_ns'],feedback_delay_ns=drive['feedback_delay_ns'],
        torque_limit=drive['torque_limit_nm'],position_noise=drive['position_noise_rad'],current_noise=drive['current_noise_a'],seed=metadata['conditions'].get('acquisition_seed',config.get('acquisition_seed',27))).items():setattr(native,field,value)
    target=root/'runs'/name;target.mkdir(parents=True,exist_ok=False);started=perf_counter()
    write_json(target/'feedforward.json',dict(rigid_parameters=feedforward.rigid_parameters(),
        viscous=[j.friction.viscous for j in feedforward.model.joints],coulomb=[j.friction.coulomb for j in feedforward.model.joints],
        source='declared nominal or earlier fitted model, never evaluated plant parameters'))
    events=[]
    if guard is None:obs,truth=_core.ArmSimulationRunner(plant,native).run(times,commands,round(duration*1e9),curve(0),np.zeros(n))
    else:
        obs,truth,executed,events=_core.run_guarded_arm(plant,native,times,commands,round(duration*1e9),curve(0),np.zeros(n),guard)
        np.savez_compressed(target/'planned_commands.npz',send_ns=times,commands=commands);commands=executed
        write_json(target/'guard_events.json',dict(events=list(events),policy='delivered position + velocity sustained; stop ramp and restore PD hold',
            joint=guard.joint,begin_ns=guard.begin_ns,end_ns=guard.end_ns,displacement_threshold_rad=guard.displacement_threshold,
            speed_threshold_rad_s=guard.speed_threshold,sustain_ns=guard.sustain_ns,max_sample_age_ns=guard.max_sample_age_ns,
            maximum_displacement_rad=guard.maximum_displacement,maximum_speed_rad_s=guard.maximum_speed))
    elapsed=perf_counter()-started
    # Raw observations are written before processing and never overwritten.
    np.savez_compressed(target/'observations.npz',**obs);np.savez_compressed(target/'commands.npz',send_ns=times,commands=commands)
    evaluation=root/'evaluation'/name;evaluation.mkdir(parents=True,exist_ok=False);np.savez_compressed(evaluation/'truth.npz',**truth)
    actual=certify_polyline(profile,obs['q'],obs['sample_ns']*1e-9,clearance_m=profile.metadata['scene']['clearance_m'],max_gap_s=1.5*native.sample_period_ns*1e-9)
    write_json(target/'collision_actual.json',actual)
    report=dict(schema_version=3,stage=metadata['stage'],conditions=metadata['conditions'],synthetic_only=True,hardware_executable=False,
        **p0_identity,
        raw_sha256=hashlib.sha256((target/'observations.npz').read_bytes()).hexdigest(),command_sha256=hashlib.sha256((target/'commands.npz').read_bytes()).hexdigest(),
        sample_clock='synthetic acquisition integer nanoseconds; receive timestamp separately retained',joint_names=[j.name for j in plant.model.joints],
        units=dict(q='rad',dq='rad/s',current='A',time='int64 ns',effort='Nm'),torque_source='output current times declared gain; not independent torque measurement',
        integration_backend=_core.SimulationRunner.integration_backend(),simulation_wall_s=elapsed,virtual_duration_s=duration,
        control_period_ns=command_period,sample_period_ns=native.sample_period_ns,integration_ns=native.integration_ns,
        inner_drive='synthetic continuous PD + feedforward/current response; ROS endpoint is a separate timing benchmark',
        native_binary_sha256=NATIVE_BINARY_SHA256,
        feedforward_sha256=hashlib.sha256((target/'feedforward.json').read_bytes()).hexdigest(),
        reference_gate_passed=True,actual_interpolant_gate_passed=actual['passed'],saturated=bool(np.any(obs['status']&1)),
        actual_range_deg=np.degrees(np.ptp(obs['q'],axis=0)).tolist(),stiction_enabled=True)
    report['stiction_backend']=getattr(_core,'arm_stiction_backend','legacy_scalar_stop_v1')
    joints=plant.model.joints;limits=np.array([[j.lower,j.upper] for j in joints])
    report['nominal_travel_limits_passed']=bool(np.all(obs['q']>=limits[:,0]) and np.all(obs['q']<=limits[:,1]))
    report['feedback_guard_passed']=guard is None or (len(events)==1 and events[0]['reason']=='breakaway_confirmed' and not events[0]['fault'])
    report['position_allowance_passed']=True
    if 'position_allowance_rad' in metadata['conditions']:
        knots=metadata['knots_s'];active=[(knots[i],knots[i+1]) for i,label in enumerate(metadata['labels']) if label in ('static_dwell','torque_ramp')]
        t=obs['sample_ns']*1e-9;mask=np.zeros(len(t),dtype=bool)
        for a,b in active:mask|=(t>=a)&(t<=b)
        report['position_allowance_passed']=bool(np.all(abs(obs['q'][mask]-curve(t[mask]))<=np.asarray(metadata['conditions']['position_allowance_rad'])))
    report['passed']=actual['passed'] and not report['saturated'] and report['nominal_travel_limits_passed'] and report['feedback_guard_passed'] and report['position_allowance_passed'];write_json(target/'metadata.json',report)
    print(f"采集 {name}: {'PASS' if report['passed'] else 'REJECT'} (C++ {elapsed:.2f}s, collision {actual['wall_s']:.2f}s)",flush=True)
    if not report['passed']:raise RuntimeError(f'Trial {name} failed actual collision/saturation gate; data retained, next stage blocked')
    return obs,report
