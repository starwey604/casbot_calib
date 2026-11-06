"""Actual four-ball, sequential-tool routes over the shared TriCal collector."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from .data import write_json
from .kinematics import digest
from .model_context import load_context
from .trical_paths import gate_chord,gate_orientation


def configuration(path):
    path=Path(path);settings=yaml.safe_load(path.read_text(encoding='utf-8'))
    if settings.get('profile')!='dvt2_trical_geometry_v1':raise ValueError('Explicit DVT2 TriCal profile required')
    context=load_context();config=context.kinematic_config()
    from importlib.resources import files
    defaults=json.loads(files('casbot_calib').joinpath('assets/trical_capture_defaults.json').read_text())
    config.update(defaults);config['profile_id']='dvt2_trical_geometry_v1'
    reference=json.loads((path.parent/settings['routes']).read_text());rules=json.loads((path.parent/settings['route_settings']).read_text())
    if reference['bundle_sha256']!=context.identity['bundle_sha256'] or reference.get('fixture_layout') is not None:raise ValueError('TriCal route source/layout differs from selected model')
    rows=reference['results']
    if {(r['arm'],r['ball']) for r in rows}!={(s,b) for s in ('left','right') for b in 'ABCD'} or len(rows)!=8:raise ValueError('Eight actual tool/ball routes required')
    config['routes']=deepcopy(rows);config['route_settings']=rules;config['route_source_sha256']=digest(reference)
    w=config['workflow'];w.update(home_rad=context.home.tolist(),geometry_allowance_rad=rules['position_allowance_rad'],
        encoder_allowance_rad=.00005,tracking_allowance_rad=rules['position_allowance_rad']-.00005,
        clearance_m=rules['clearance_m'],geometry_error_budget_m=.0001,geometry_rotation_budget_rad=0.,
        maximum_velocity_rad_s=rules['maximum_velocity_rad_s'],maximum_acceleration_rad_s2=rules['maximum_acceleration_rad_s2'],maximum_jerk_rad_s3=rules['maximum_jerk_rad_s3'])
    w['visits']=[dict(id=r['arm']+'_'+r['ball'],frame=r['arm']+'_trical_tcp',ball_id=r['ball'],nominal_q_rad=r['contact'][0]['q'],return_home=True) for r in rows]
    config['k3']=dict(design_seed=20261106,training_per_ball=2,validation_per_ball=1,pool_per_ball=4,
        maximum_attempts_per_ball=80,training_separation_rad=.025,validation_separation_rad=.012,variation_rad=.1,ik_joint_margin_rad=rules['position_allowance_rad']+.0001,orientation_variation_rad=.003,sigma_m=2e-6,maximum_condition=200.,validation_limit_m=20e-6)
    overrides=settings.get('design',{})
    if set(overrides)-set(config['k3']):raise ValueError('Unknown DVT2 design setting')
    config['k3'].update(overrides)
    d=config['k3']
    if any(type(d[k]) is not int or d[k]<1 for k in ('training_per_ball','validation_per_ball','pool_per_ball','maximum_attempts_per_ball')) or d['training_per_ball']<2 or d['pool_per_ball']<d['training_per_ball']:raise ValueError('Invalid DVT2 design counts')
    if any(not np.isfinite(d[k]) or d[k]<=0 for k in ('training_separation_rad','validation_separation_rad','variation_rad','ik_joint_margin_rad','orientation_variation_rad','sigma_m','maximum_condition','validation_limit_m')):raise ValueError('Positive DVT2 design settings required')
    config['parameter_policy'].update(fit_enabled=True,mdh_hayati='K3_exact_chart_Pinocchio_geometry_regressor')
    from .kinematic_parameters import KinematicParameters
    parameters=KinematicParameters(context.engine);hidden=np.zeros(parameters.size)
    for i,definition in enumerate(parameters.definitions):
        hidden[i]={'left.mount_1.tx':.15,'left.mount_1.ty':-.06,'left.mount_1.tz':.10,
                   'right.mount_1.tx':-.12,'right.mount_1.ty':.08,'right.mount_1.tz':.16}.get(definition['name'],0.)
    config['mock']=dict(seed=20261106,encoder_offset_rad=(np.array([1,-.7,.5,-1,.4,-.5,.8,-1,.6,-.5,.9,-.4,.6,-.7])*1e-5).tolist(),
        bias_m=[0.,0.,0.],geometry_scaled=hidden.tolist(),geometry_chart_sha256=parameters.chart['sha256'])
    from .kinematics import validate_config
    validate_config(config);return config


def route(config,frame,ball):
    return next(r for r in config['routes'] if r['arm']==frame.split('_')[0] and r['ball']==ball)


def route_steps(config,row):
    """Outward from the contact seed; reverse order is the insertion route."""
    local=np.array([r['q'] for r in row['contact']]);spec=row.get('withdrawal')
    release=spec['release_index'] if spec else len(local)-1
    return [(local[i],local[i+1],'released' if spec and i>=release else 'contact') for i in range(len(local)-1)]


def geometric_profile(engine,config,target=None,contact=False,*,active_arm=None,phase=None):
    from .trical_assembly import released_profile
    context=load_context()
    if config['model_context']!=context.identity:raise ValueError('TriCal context changed')
    if config['fixture']!=context.kinematic_config()['fixture']:raise ValueError('TriCal metrology fixture differs from the CAD collision scene')
    if config['workflow']['clearance_m']!=.01 or config['workflow']['allowed_tip_stroke_m']!=.005:raise ValueError('Actual CAD policy is fixed at ordinary10mm / exact-tip5mm')
    if active_arm is None:active_arm=target[0].split('_')[0] if target else config['workflow']['visits'][0]['frame'].split('_')[0]
    phase=phase or ('contact' if contact else 'free')
    ball=target[1] if target else None
    if phase not in ('free','contact','released'):raise ValueError('Unknown actual assembly phase')
    p=context.collision_profile(active_tool=active_arm,ball_id=ball,contact=phase!='free',engine=engine)
    if phase=='released':
        spec=route(config,target[0],ball).get('withdrawal')
        if spec is None:raise ValueError('No release/turn phase in this route')
        p=released_profile(p,spec['minimum_released_tip_clearance_m'])
    budget=config['workflow'].get('geometry_error_budget_m',0.)
    moving=np.array([g.parent>=0 for g in p.geometry],dtype=int);pairs=np.array(p.pairs)
    p.pair_clearance_m=p.pair_clearance_m+budget*(moving[pairs[:,0]]+moving[pairs[:,1]])
    p.metadata={**p.metadata,'route_phase':phase,'rigid_geometry_displacement_budget_m':budget}
    p.metadata['pair_clearances_sha256']=digest(dict(pairs=p.pairs,thresholds=p.pair_clearance_m.tolist()))
    if phase=='contact':
        row=route(config,target[0],ball)
        p.metadata['orientation_reference']=engine.frame_pose(row['contact'][0]['q'],target[0])[:3,:3].tolist()
        p.metadata['orientation_frame']=target[0]
        p.metadata['orientation_tolerance_rad']=row.get('withdrawal',{}).get('contact_orientation_tolerance_rad',np.deg2rad(1.))-config['workflow'].get('geometry_rotation_budget_rad',0.)
    return p


def bounded_chord(profile,a,b,allowance,config):
    from .collision import certify_chord_box_pruned
    settings=config['route_settings'];a=np.asarray(a);b=np.asarray(b)
    # Recompute the already established local radius on small subchords. A
    # whole-path descendant-displacement bound was too loose at left D.
    count=max(1,int(np.ceil(np.max(abs(b-a))/.001))) if profile.metadata['route_phase']=='contact' else 4
    reports=[]
    for i in range(count):
        start=a+(b-a)*i/count;end=a+(b-a)*(i+1)/count
        proof=certify_chord_box_pruned(profile,start,end,allowance,settings['clearance_m'],settings['joint_box_query_budget'],
            subdivisions=1,max_joint_depth=settings['joint_box_max_depth'],local_motion_bounds=True)
        reports.append(proof)
        if not proof['passed']:break
    return dict(passed=len(reports)==count and all(r['passed'] for r in reports),reason='subchord_tubes_certified' if len(reports)==count and all(r['passed'] for r in reports) else reports[-1]['reason'],
        subchords=reports,queries=sum(r['queries'] for r in reports),thresholds_unchanged=True,all_pairs_covered=len(reports)==count and all(r['passed'] for r in reports))


def motion_gate(profile,start,end,config,cache=None):
    settings=config['route_settings'];a=np.asarray(start);b=np.asarray(end)
    limits=np.array([[j.lower,j.upper] for j in profile.engine.model.joints]);allow=settings['position_allowance_rad']
    if np.any(np.minimum(a,b)-allow<limits[:,0]) or np.any(np.maximum(a,b)+allow>limits[:,1]):return dict(passed=False,reason='uncertain_position_outside_joint_limits')
    if profile.metadata['route_phase']=='contact':
        orientation=gate_orientation(profile.engine,profile.metadata['orientation_frame'],a,b,np.array(profile.metadata['orientation_reference']),profile.metadata['orientation_tolerance_rad'],allow)
        if not orientation['passed']:return dict(passed=False,reason='contact_orientation_rejected',contact_orientation=orientation)
    if profile.metadata['route_phase']=='contact':
        key=('contact_subchords',profile.metadata['geometry_sha256'],profile.pair_clearance_m.tobytes(),*sorted([a.tobytes(),b.tobytes()]))
        if cache is not None and key in cache:proof=cache[key]
        else:
            proof=bounded_chord(profile,a,b,np.full(14,allow),config)
            if cache is not None:cache[key]=proof
    else:proof=gate_chord(profile,a,b,settings,cache)
    if profile.metadata['route_phase']=='contact':proof={**proof,'contact_orientation':orientation}
    return proof


def local_profile(profile,start,end,config):
    from .collision import local_motion_profile
    a=np.asarray(start);b=np.asarray(end);allow=config['workflow']['geometry_allowance_rad']
    # Covers the commanded chord, observed tracking error, and encoder uncertainty.
    return local_motion_profile(profile,(a+b)/2,abs(b-a)/2+2*allow)[0]


def profiles(engine,config,frame,ball):
    target=(frame,ball)
    result={k:geometric_profile(engine,config,target,phase=k) for k in ('free','contact')}
    if route(config,frame,ball).get('withdrawal'):result['released']=geometric_profile(engine,config,target,phase='released')
    return result


def preflight(engine,config,row,cache=None):
    p=profiles(engine,config,row['arm']+'_trical_tcp',row['ball']);transfer=np.array(row['transfer']);home=np.array(config['workflow']['home_rad'])
    if not np.array_equal(transfer[0],home) or not np.array_equal(transfer[-1],row['contact'][-1]['q']):raise ValueError('Frozen transfer endpoints differ')
    from .trical_assembly import static_platform_check
    gates=[static_platform_check(p['free'],config['workflow']['clearance_m'])]
    gates += [motion_gate(p['free'],a,b,config,cache) for a,b in zip(transfer[:-1],transfer[1:])]
    gates += [motion_gate(p[phase],a,b,config,cache) for a,b,phase in route_steps(config,row)]
    return p,gates


def routed_controller(base):
    """Reuse command delivery, stable windows and iterative correction unchanged."""
    class RoutedController(base):
        def __init__(self,*args):
            super().__init__(*args);self.active_arm=None;self.route_cache={};self.profile_cache={}

        def visit(self,visit):
            from .trical_workflow import MeasurementFailure
            from .kinematics import ball_world
            frame,ball=visit['frame'],visit['ball_id'];side=frame.split('_')[0]
            if not np.allclose(self.commanded,self.home,rtol=0,atol=1e-12):raise MeasurementFailure('tool_switch_or_visit_not_at_home')
            if self.active_arm!=side:
                if self.device.latest is not None:
                    latest=self.device.latest;w=self.config['workflow']
                    if np.max(abs(latest['dq']))>w['stable_velocity_rad_s'] or np.max(abs(latest['q']-self.home))>w['geometry_allowance_rad'] or self.device.now-int(latest['receive_ns'])>w['max_joint_age_ns'] or np.any(latest['status']):
                        raise MeasurementFailure('tool_switch_not_stationary_at_home')
                self.event('install_tool' if self.active_arm is None else 'manual_remount_at_home',active_tool=side,inactive_tool=None)
                self.active_arm=side
            row=route(self.config,frame,ball);key=(frame,ball)
            self.device.select(frame,ball,ball_world(self.config,ball));self.event('preflight',visit=visit['id'])
            if key not in self.profile_cache:
                p,gates=preflight(self.engine,self.config,row,self.route_cache)
                if not all(g['passed'] for g in gates):raise MeasurementFailure('frozen_route_preflight_rejected:'+side+'_'+ball)
                self.profile_cache[key]=p
            p=self.profile_cache[key];self.free=p['free'];self.contact=p['contact']
            original=np.array(row['contact'][0]['q']);goal=np.array(visit['nominal_q_rad'])
            # Both directions use the same chord image; the complete escape is
            # checked before entering a distinct measurement posture.
            if not motion_gate(p['contact'],original,goal,self.config,self.route_cache)['passed']:raise MeasurementFailure('measurement_posture_route_rejected')
            transfer=np.array(row['transfer'])
            for q in transfer[1:]:self.execute(q,p['free'],'approach')
            steps=route_steps(self.config,row)
            for index,(a,b,phase) in reversed(list(enumerate(steps))):
                self.execute(a,p[phase],'insert' if index==0 else 'insert_'+phase)
            if not np.array_equal(original,goal):self.execute(goal,p['contact'],'measurement_posture')
            self.center(visit,original)
            if not np.array_equal(self.commanded,original):self.execute(original,p['contact'],'withdraw_to_seed')
            for index,(a,b,phase) in enumerate(steps):self.execute(b,p[phase],'withdraw' if index==0 else 'withdraw_'+phase)
            self.contact=None
            for q in transfer[-2::-1]:self.execute(q,p['free'],'return_home')
            self.stage=None

        def recover(self):
            # A straight-line escape is not valid for this fixture. Preserve the
            # partial record and hold only if the delivered feedback allows it.
            self.event('abort_hold')
            from .trical_workflow import MeasurementFailure
            try:
                if not self.device.command_times:self.event('aborted_before_motion');return True
                if self.device.latest is None or self.device.now-int(self.device.latest['receive_ns'])>self.config['workflow']['max_joint_age_ns']:raise MeasurementFailure('no_fresh_joint_feedback')
                self.commanded=self.device.latest['q'].copy();self.execute(self.commanded,self.current_profile,'fault_hold',.2)
            except MeasurementFailure as error:self.event('fault_hold_rejected',reason=str(error))
            self.event('latched_hold_manual_recovery_required',reason='No unverified straight withdrawal or automatic tool remount')
            return False
    return RoutedController


def plan(folder,config):
    """Training-only selection; every variant connects to a verified CAD seed."""
    from .kinematics import nominal_kinematics,ball_world
    from .kinematic_parameters import KinematicParameters
    from .kinematic_identification import select_parameters,observability
    from .kinematic_design import _d_optimal
    from .trical_motion import solve_position
    from hashlib import sha256
    root=Path(folder);root.mkdir(parents=True,exist_ok=False);config=deepcopy(config);hidden=config.pop('mock')
    e,geometry=nominal_kinematics(config);parameters=KinematicParameters(e);d=config['k3'];cache={};all_candidates=[];rejected=[];proofs={}
    write_json(root/'profile.json',config);write_json(root/'geometry.json',geometry);write_json(root/'parameterization.json',parameters.description())
    (root/'evaluation').mkdir();write_json(root/'evaluation/mock.json',hidden)
    def failure(reason,**details):
        result=dict(passed=False,reason=reason,**details);write_json(root/'plan_report.json',result);write_json(root/'rejected.json',rejected);return result
    for row in config['routes']:
        frame=row['arm']+'_trical_tcp';case=row['arm']+'_'+row['ball'];p,gates=preflight(e,config,row,cache);proofs[case]=gates
        if not all(g['passed'] for g in gates):return failure('actual_assembly_route_rejected',case=case)
        original=np.array(row['contact'][0]['q']);pose=e.frame_pose(original,frame);active=np.arange(7)+(0 if row['arm']=='left' else 7)
        for split in ('train','validation'):
            rng=np.random.default_rng(d['design_seed']+len(proofs)*100+(10000 if split=='validation' else 0))
            wanted=d['pool_per_ball'] if split=='train' else d['validation_per_ball'];accepted=[]
            for attempt in range(d['maximum_attempts_per_ball']):
                if len(accepted)>=wanted:break
                seed=original.copy();seed[active]+=rng.uniform(-d['variation_rad'],d['variation_rad'],7)
                R=pose[:3,:3]@Rotation.from_rotvec(rng.uniform(-d['orientation_variation_rad'],d['orientation_variation_rad'],3)).as_matrix()
                try:
                    q=solve_position(e,frame,ball_world(config,row['ball']),seed,R,joint_margin_rad=d.get('ik_joint_margin_rad',.02))
                    separation=d.get('training_separation_rad',.025) if split=='train' else d.get('validation_separation_rad',.012)
                    if any(c['frame']==frame and np.linalg.norm(q-np.array(c['q']))<separation for c in all_candidates+accepted):continue
                    # Falsification screen only; admission still requires the full
                    # continuous joint-box proof below, including the parked arm.
                    from itertools import product
                    corners=np.tile(q,(128,1));corners[:,active]+=np.array(list(product([-1.,1.],repeat=7)))*config['workflow']['geometry_allowance_rad']
                    if np.any(p['contact'].check(corners)['distances']<=p['contact'].pair_clearance_m):
                        rejected.append(dict(case=case,split=split,attempt=attempt,reason='endpoint_corner_counterexample'));continue
                    gate=motion_gate(p['contact'],original,q,config,cache)
                    if not gate['passed']:rejected.append(dict(case=case,split=split,attempt=attempt,gate=gate));continue
                    offset=e.frame_pose(q,frame)[:3,:3].T@(ball_world(config,row['ball'])-e.frame_pose(q,frame)[:3,3])
                    _,jac=parameters.evaluate(np.zeros(parameters.size),q[None],np.array([frame]),offset[None])
                    accepted.append(dict(id=f'{split}_{case}_{attempt:03}',split=split,frame=frame,ball_id=row['ball'],q=q.tolist(),
                        stage_q=row['contact'][-1]['q'],nominal_offset_sensor_m=offset.tolist(),jacobian=jac[0].tolist(),gates=[gate]))
                except ValueError as exc:rejected.append(dict(case=case,split=split,attempt=attempt,reason=str(exc)))
            print(f'DVT2 K3 {split}/{case}: {len(accepted)}/{wanted}',flush=True)
            all_candidates+=accepted;write_json(root/'candidates.json',all_candidates);write_json(root/'route_gates.json',proofs)
            if len(accepted)<wanted:return failure('insufficient_collision_free_posture_variants',case=case,split=split,accepted=len(accepted),required=wanted)
    # Legacy targets are arm-specific; the real four balls are shared. Group by
    # tool AND ball while reusing the same information-design implementation.
    pool=[dict(c,ball_id=c['frame']+':'+c['ball_id']) for c in all_candidates if c['split']=='train']
    selected=_d_optimal(pool,d['training_per_ball'],parameters.size,d['sigma_m'])
    selected=[dict(c,ball_id=c['ball_id'].split(':')[-1]) for c in selected]
    validation=[c for c in all_candidates if c['split']=='validation']
    def schedule(rows,split):
        return [dict(id=c['id'],candidate_id=c['id'],split=split,batch_id='dvt2_'+split,frame=c['frame'],ball_id=c['ball_id'],nominal_q_rad=c['q'],return_home=True)
                for c in sorted(rows,key=lambda c:(c['frame'],c['ball_id'],c['id']))]
    _,jac=parameters.evaluate(np.zeros(parameters.size),np.array([c['q'] for c in selected]),np.array([c['frame'] for c in selected]),np.array([c['nominal_offset_sensor_m'] for c in selected]))
    chosen,zero=select_parameters(jac,parameters.definitions,d['maximum_condition'])
    for split,rows in [('train',selected),('validation',validation)]:write_json(root/(split+'_visits.json'),schedule(rows,split))
    write_json(root/'route_gates.json',proofs);write_json(root/'candidates.json',all_candidates);write_json(root/'rejected.json',rejected)
    anchors={frame:dict(targets=4,rank=int(np.linalg.matrix_rank(np.array([ball_world(config,b) for b in 'ABCD'])-np.mean([ball_world(config,b) for b in 'ABCD'],axis=0)))) for frame in ('left_trical_tcp','right_trical_tcp')}
    result=dict(passed=bool(chosen),training_samples=len(selected),validation_samples=len(validation),candidate_count=len(all_candidates),
        rejected_count=len(rejected),selected_parameter_count=len(chosen),selected_training_parameters=chosen,zero_observability=zero,
        full_candidate_observability=observability(jac.reshape(-1,parameters.size)/d['sigma_m'],d['maximum_condition']),
        anchor_geometry=anchors,geometry_sha256=geometry['geometry_sha256'],model_context=config['model_context'],
        training_only_selection=True,synthetic_only=True,hardware_executable=False,calibration_performed=False,
        tool_sequence=['left','manual_remount_at_home','right'],metrology_anchor='Source CAD registration and synthetic mastering; not surveyed hardware')
    write_json(root/'plan_report.json',result)
    paths=[p for p in root.rglob('*') if p.is_file() and 'evaluation' not in p.relative_to(root).parts]
    write_json(root/'manifest.json',dict(files={str(p.relative_to(root)):sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}))
    return result


def observed_gate(profile,positions,times,config):
    """Bound the complete delivered polyline, including every interior sample.

    A block's deviation from its endpoint chord is bounded at all sample knots;
    linear interpolation preserves that bound between knots. Reuse N2's full
    joint-box proof instead of rejecting on an unresolved radius inflation.
    """
    from time import perf_counter
    started=perf_counter();q=np.asarray(positions);t=np.asarray(times);gap=config['drive']['sample_period_ns']*1.5e-9
    if len(q)<2 or np.any(np.diff(t)<=0) or np.max(np.diff(t))>gap:return dict(passed=False,reason='missing_observed_samples')
    if profile.metadata['route_phase']=='contact':
        rotations=profile.engine.frame_batch(q,profile.metadata['orientation_frame'])['poses'][:,:3,:3]
        angles=Rotation.from_matrix(np.array(profile.metadata['orientation_reference']).T@rotations).magnitude()
        if np.max(angles)+7*config['workflow'].get('encoder_allowance_rad',config['workflow']['geometry_allowance_rad'])>profile.metadata['orientation_tolerance_rad']:
            return dict(passed=False,reason='observed_contact_orientation_rejected')
    results=[];begin=0
    while begin<len(q)-1:
        end=min(begin+2000,len(q)-1);delta=q[end]-q[begin];norm=float(delta@delta)
        u=np.clip((q[begin:end+1]-q[begin])@delta/norm,0,1) if norm>1e-20 else np.linspace(0,1,end-begin+1)
        deviation=np.max(abs(q[begin:end+1]-(q[begin]+u[:,None]*(q[end]-q[begin]))),axis=0)
        allowance=deviation+config['workflow'].get('encoder_allowance_rad',config['workflow']['geometry_allowance_rad'])
        proof=bounded_chord(profile,q[begin],q[end],allowance,config)
        results.append(proof)
        if not proof['passed']:break
        begin=end
    return dict(passed=begin==len(q)-1 and all(r['passed'] for r in results),blocks=len(results),queries=sum(r['queries'] for r in results),
        path_model='complete observed piecewise-linear feedback, bounded block deviation plus declared encoder uncertainty',wall_s=perf_counter()-started,
        maximum_gap_s=float(np.max(np.diff(t))),proofs=results)


def seed_reference_cache(controller,source):
    """Consume sealed reference proofs for identical geometry/thresholds/endpoints.

    This avoids running an offline proof tree again inside virtual-time delivery.
    Observed packets and correction/escape paths are still checked independently.
    """
    from hashlib import sha256
    source=Path(source);manifest=json.loads((source/'manifest.json').read_text())
    for name,expected in manifest['files'].items():
        path=Path(name)
        if path.is_absolute() or '..' in path.parts or 'evaluation' in path.parts:raise ValueError('Invalid reference proof source path')
        if sha256((source/path).read_bytes()).hexdigest()!=expected:raise ValueError('Frozen reference proof input changed')
    planned=json.loads((source/'profile.json').read_text());config=controller.config
    for key in ('model_context','fixture','tools','routes','route_settings'):
        if planned[key]!=config[key]:raise ValueError('Reference geometry/route differs from measurement profile: '+key)
    for key in ('geometry_allowance_rad','geometry_error_budget_m','geometry_rotation_budget_rad','clearance_m','allowed_tip_stroke_m'):
        if planned['workflow'].get(key)!=config['workflow'].get(key):raise ValueError('Reference uncertainty/clearance policy changed: '+key)
    cache=controller.route_cache;engine=controller.engine;count=0
    def put(p,a,b,proof):
        nonlocal count
        if not proof['passed']:return
        a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float);endpoints=sorted([a.tobytes(),b.tobytes()])
        if p.metadata['route_phase']=='contact':key=('contact_subchords',p.metadata['geometry_sha256'],p.pair_clearance_m.tobytes(),*endpoints)
        else:key=(p.metadata['geometry_sha256'],p.pair_clearance_m.tobytes(),*endpoints,tuple(sorted(config['route_settings'].items())))
        cache[key]=proof;count+=1
    if (source/'route_gates.json').exists():
        if not json.loads((source/'plan_report.json').read_text())['passed']:raise ValueError('Rejected design cannot seed execution')
        saved=json.loads((source/'route_gates.json').read_text());p_by_case={}
        for row in config['routes']:
            frame=row['arm']+'_trical_tcp';p=profiles(engine,config,frame,row['ball']);p_by_case[(frame,row['ball'])]=p;gates=saved[row['arm']+'_'+row['ball']];transfer=np.array(row['transfer'])
            steps=[(a,b,'free') for a,b in zip(transfer[:-1],transfer[1:])]+route_steps(config,row)
            if len(gates)!=len(steps)+1 or not all(g['passed'] for g in gates):raise ValueError('Incomplete frozen route proof')
            for (a,b,phase),proof in zip(steps,gates[1:],strict=True):put(p[phase],a,b,proof)
        for candidate in json.loads((source/'candidates.json').read_text()):
            p=p_by_case[(candidate['frame'],candidate['ball_id'])]['contact']
            a=route(config,candidate['frame'],candidate['ball_id'])['contact'][0]['q'];put(p,a,candidate['q'],candidate['gates'][0])
    else:
        if not json.loads((source/'report.json').read_text())['passed']:raise ValueError('Rejected capture cannot seed execution')
        p_cache={}
        for motion in json.loads((source/'motions.json').read_text()):
            target=motion['target'];key=(target['frame'],target['ball_id'],motion['route_phase'])
            if key not in p_cache:p_cache[key]=geometric_profile(engine,config,(target['frame'],target['ball_id']),phase=motion['route_phase'])
            p=p_cache[key]
            if p.metadata['geometry_sha256']!=motion['collision']['geometry_sha256'] or p.metadata['pair_clearances_sha256']!=motion['collision']['pair_clearances_sha256']:raise ValueError('Recorded proof geometry/clearance identity changed')
            states=np.asarray(motion['motion']['states'])
            if states.shape!=(2,4,14) or np.any(abs(states[:,1:])>1e-12):raise ValueError('Recorded proof is not a rest-to-rest C3 chord')
            put(p,states[0,0],states[-1,0],motion['reference'])
    return dict(source_manifest_sha256=sha256((source/'manifest.json').read_bytes()).hexdigest(),proofs_consumed=count,
                policy='same canonical geometry, route, reference tube and thresholds; raw observation proofs never reused')
