"""Collision-filtered training-only D-optimal posture design for fixed artifact balls."""
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import numpy as np
import yaml
from .kinematics import workflow_config,nominal_kinematics,ball_world,digest
from .kinematic_parameters import KinematicParameters
from .kinematic_identification import observability,select_parameters
from .trical_fixture import fixture_profile
from .trical_motion import solve_position,segment
from .data import write_json


def k3_config(path=None):
    from .dvt2_trical import configuration
    return configuration(path)


def _d_optimal(candidates, count_per_ball, size, sigma):
    inverse=np.eye(size);selected=[];remaining=list(range(len(candidates)))
    groups=sorted({x['ball_id'] for x in candidates})
    for _ in range(count_per_ball):
        for ball in groups:
            choices=[i for i in remaining if candidates[i]['ball_id']==ball]
            values=np.array([np.array(candidates[i]['jacobian'])/sigma for i in choices])
            scores=np.linalg.slogdet(np.eye(3)[None]+values@inverse@values.transpose(0,2,1))[1]
            best=choices[int(np.argmax(scores))];selected.append(best);remaining.remove(best);j=np.array(candidates[best]['jacobian'])/sigma
            inverse-=inverse@j.T@np.linalg.solve(np.eye(3)+j@inverse@j.T,j@inverse)
            inverse=(inverse+inverse.T)/2
    return [candidates[i] for i in selected]


def _schedule(selected, engine, config, free, split):
    ordered=sorted(selected,key=lambda row:(row['frame'],row['ball_id'],row['id']))
    visits=[];connections=[]
    for i,item in enumerate(ordered):
        visit=dict(id=item['id'],candidate_id=item['id'],split=split,batch_id='k3_'+split,frame=item['frame'],ball_id=item['ball_id'],
            nominal_q_rad=item['q'],return_home=True)
        if i+1<len(ordered) and ordered[i+1]['frame']==item['frame']:
            _,gate=segment(free,np.array(item['stage_q']),np.array(ordered[i+1]['stage_q']),config,'between_measurements')
            connections.append(dict(first=item['id'],second=ordered[i+1]['id'],gate=gate))
            if gate['passed']:visit['return_home']=False
        visits.append(visit)
    return visits,connections


def plan(folder,config=None):
    root=Path(folder)
    if root.exists():raise FileExistsError(root)
    config=k3_config() if config is None else deepcopy(config)
    if 'model_context' in config:
        from .dvt2_trical import plan as actual_plan
        return actual_plan(root,config)
    hidden=config.pop('mock')
    engine,geometry=nominal_kinematics(config);parameters=KinematicParameters(engine);settings=config['k3'];home=np.array(config['workflow']['home_rad'])
    free=fixture_profile(engine,config);candidates=[];rejected=[]
    root.mkdir(parents=True,exist_ok=False);(root/'evaluation').mkdir();write_json(root/'profile.json',config);write_json(root/'geometry.json',geometry)
    write_json(root/'parameterization.json',parameters.description());write_json(root/'evaluation/mock.json',hidden)
    anchors={}
    for frame in {v['frame'] for v in config['workflow']['visits']}:
        points=np.unique(np.array([ball_world(config,v['ball_id']) for v in config['workflow']['visits'] if v['frame']==frame]),axis=0)
        rank=int(np.linalg.matrix_rank(points-points.mean(axis=0),tol=1e-9));anchors[frame]=dict(targets=len(points),rank=rank)
        if len(points)<3 or rank<2:
            report=dict(passed=False,reason='insufficient_noncollinear_metrology_targets',anchors=anchors)
            write_json(root/'plan_report.json',report);return report
    for split in ('train','validation'):
        for ball_index,template in enumerate(config['workflow']['visits']):
            frame,ball=template['frame'],template['ball_id'];active=np.arange(7) if frame.startswith('left') else np.arange(7,14)
            rng=np.random.default_rng(settings['design_seed']+(10000 if split=='validation' else 0)+ball_index)
            target=ball_world(config,ball);contact=fixture_profile(engine,config,(frame,ball),True)
            wanted=settings['pool_per_ball'] if split=='train' else settings['validation_per_ball']+3;accepted=0
            for attempt in range(settings['maximum_attempts_per_ball']):
                if accepted>=wanted:break
                name=f'{split}_{ball}_{attempt:03}';seed=np.array(template['nominal_q_rad'])
                seed[active]+=rng.uniform(-settings['variation_rad'],settings['variation_rad'],7)
                try:
                    q=solve_position(engine,frame,target,seed);pose=engine.frame_pose(q,frame)
                    if any(x['frame']==frame and np.linalg.norm(np.array(x['q'])-q)<.025 for x in candidates):continue
                    distance=contact.check(q)['distances'][0]-contact.pair_clearance_m-contact.radii@np.full(14,config['workflow']['geometry_allowance_rad'])
                    if distance.min()<=0:
                        rejected.append(dict(id=name,reason='pose_clearance',minimum_surplus_m=float(distance.min())));continue
                    stage=solve_position(engine,frame,target-pose[:3,:3]@np.full(3,.06),q)
                    gates=[]
                    for a,b,profile,label in [(home,stage,free,'approach'),(stage,q,contact,'insert')]:
                        _,gate=segment(profile,a,b,config,label);gates.append(gate)
                        if not gate['passed']:break
                    if not all(g['passed'] for g in gates) or len(gates)!=2:
                        rejected.append(dict(id=name,reason='motion_gate',gates=gates));continue
                    # Zero v/a/jerk Hermite reversal covers exactly the same q set.
                    gates.extend([dict(passed=True,exact_time_reversal_of='insert'),dict(passed=True,exact_time_reversal_of='approach')])
                    offset=pose[:3,:3].T@(target-pose[:3,3]);_,jac=parameters.evaluate(np.zeros(parameters.size),q[None],np.array([frame]),offset[None])
                    row=dict(id=name,split=split,frame=frame,ball_id=ball,q=q.tolist(),stage_q=stage.tolist(),
                        nominal_offset_sensor_m=offset.tolist(),jacobian=jac[0].tolist(),gates=gates)
                    candidates.append(row);accepted+=1
                except ValueError as error:rejected.append(dict(id=name,reason=str(error)))
            print(f'K3候选 {split}/{ball}: {accepted}/{wanted}，拒绝记录累计{len(rejected)}',flush=True)
            if accepted<wanted:
                report=dict(passed=False,reason='insufficient_collision_free_candidates',split=split,ball=ball,accepted=accepted,required=wanted)
                write_json(root/'rejected.json',rejected);write_json(root/'plan_report.json',report);return report
    training_pool=[c for c in candidates if c['split']=='train'];selected=_d_optimal(training_pool,settings['training_per_ball'],parameters.size,settings['sigma_m'])
    validation=[]
    for template in config['workflow']['visits']:
        validation.extend([c for c in candidates if c['split']=='validation' and c['ball_id']==template['ball_id']][:settings['validation_per_ball']])
    q=np.array([c['q'] for c in selected]);frames=np.array([c['frame'] for c in selected]);offsets=np.array([c['nominal_offset_sensor_m'] for c in selected])
    _,jac=parameters.evaluate(np.zeros(parameters.size),q,frames,offsets);chosen,zero_info=select_parameters(jac,parameters.definitions,settings['maximum_condition'])
    train_visits,train_connections=_schedule(selected,engine,config,free,'train')
    validation_visits,validation_connections=_schedule(validation,engine,config,free,'validation')
    write_json(root/'train_visits.json',train_visits);write_json(root/'validation_visits.json',validation_visits)
    write_json(root/'connections.json',dict(train=train_connections,validation=validation_connections));write_json(root/'candidates.json',candidates);write_json(root/'rejected.json',rejected)
    report=dict(schema_version=1,passed=bool(chosen),training_samples=len(selected),validation_samples=len(validation),candidate_count=len(candidates),
        rejected_count=len(rejected),training_only_selection=True,zero_observability=zero_info,
        full_candidate_observability=observability(jac.reshape(-1,parameters.size)/settings['sigma_m'],settings['maximum_condition']),
        selected_parameter_count=len(chosen),selected_training_parameters=chosen,
        geometry_sha256=geometry['geometry_sha256'],synthetic_only=True,hardware_executable=False,calibration_performed=False,
        anchor_geometry=anchors,
        metrology_anchor='common metric artifact world transform and both tools are fixed synthetic surveyed inputs')
    write_json(root/'plan_report.json',report)
    paths=[p for p in root.rglob('*') if p.is_file() and 'evaluation' not in p.relative_to(root).parts]
    write_json(root/'manifest.json',dict(files={str(p.relative_to(root)):sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}))
    return report
