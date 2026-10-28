"""Build and gate a complete experiment manifest before any Mock collection."""
import hashlib
from pathlib import Path
from time import perf_counter
import numpy as np
import yaml
from .data import write_json
from .collision import nominal_profile
from .staged_motion import build_recipes,gate_motion,derivative_bounds
from .rigid_identification import parameter_scales


def plan_session(folder,config,development_reference=None,contract_only=False):
    if config.get('schema_version')==2:
        from .staged_protocol import prepare_protocol
        if contract_only and development_reference is not None:raise ValueError('Contract-only planning does not bind predecessor observations')
        if config['p0'].get('planning_round')=='R1' and not contract_only:
            from .staged_multiconfiguration import plan_references
            return prepare_protocol(folder,config,planner=lambda *args:plan_references(*args,development_reference=development_reference))
        return prepare_protocol(folder,config)
    if development_reference is not None:raise ValueError('Development reference is only supported by P0 R1')
    if 'p0' in config:raise ValueError('P0 requires schema 2; legacy planning cannot qualify P0')
    if config.get('schema_version')!=1 or config.get('mode')!='mock_only':raise ValueError('Only versioned mock plans are supported')
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    scene=yaml.safe_load(Path(config['collision_scene']).read_text(encoding='utf-8'))
    profile=nominal_profile(scene=scene);motions=build_recipes(config)
    candidates=[m for m in motions if m.stage=='inertia' and m.conditions['batch']=='train'];design=[];selected=None
    for m in candidates:
        gate=gate_motion(profile,m,scene['clearance_m']);curve=m.polynomial();period=m.conditions['period_s']
        start=m.windows('periodic')[0][0];t=start+np.arange(256)*period/256;q,v,a=(curve(t,nu=d) for d in range(3))
        y=profile.engine.regressor_batch(q,v,a)['regressor'].reshape(-1,140)*parameter_scales(profile.engine.rigid_parameters())
        singular=np.linalg.svd(y,compute_uv=False);rank=int(np.count_nonzero(singular>singular[0]*1e-7))
        peaks=[float(np.max(derivative_bounds(curve,d))) for d in (1,2,3)]
        coverage=dict(minimum_range_rad=float(np.ptp(q,axis=0).min()),minimum_speed_rms_rad_s=float(np.sqrt(np.mean(v*v,axis=0)).min()),
            minimum_acceleration_rms_rad_s2=float(np.sqrt(np.mean(a*a,axis=0)).min()))
        valid=bool(gate['passed'] and peaks[0]<=2 and peaks[1]<=8 and peaks[2]<=60 and coverage['minimum_range_rad']>.15 and coverage['minimum_speed_rms_rad_s']>.01 and coverage['minimum_acceleration_rms_rad_s2']>.02)
        row=dict(seed=m.conditions['seed'],rank=rank,log_information=float(np.log(singular[:rank]).sum()),admissible=valid,
            peaks=dict(speed_rad_s=peaks[0],acceleration_rad_s2=peaks[1],jerk_rad_s3=peaks[2]),coverage=coverage,collision=gate)
        design.append(row)
        if valid and (selected is None or (rank,row['log_information'])>(selected[0]['rank'],selected[0]['log_information'])):selected=(row,m)
    write_json(folder/'excitation_design.json',dict(candidates=design,selected_seed=None if selected is None else selected[0]['seed'],
        method='Pinocchio full rigid regressor SVD / physical coordinate scaling / conservative collision gate / synthetic derivative envelope',
        parameter_count=140,unique_parameters_claimed=False))
    if selected is None:raise ValueError('No collision-free informative inertia candidate inside the synthetic motion envelope')
    chosen=selected[1];chosen.name='inertia_train';candidate_ids={id(m) for m in candidates};ordered=[];inserted=False
    for motion in motions:
        if id(motion) in candidate_ids:
            if not inserted:ordered.append(chosen);inserted=True
        else:ordered.append(motion)
    motions=ordered
    observable_config={key:value for key,value in config.items() if key!='mock'}
    if 'mock' in config:
        (folder/'evaluation').mkdir();write_json(folder/'evaluation'/'plant_config.json',config['mock'])
    write_json(folder/'session.json',dict(config=observable_config,collision=profile.metadata,
        joint_names=[j.name for j in profile.engine.model.joints],
        sample_clock='synthetic acquisition time; not ROS arrival time',
        hardware_executable=False,synthetic_only=True))
    reports=[];selected=[];candidates=[];started=perf_counter();gravity_counts=dict(train=0,validation=0);pending_pair=[]
    for m in motions:
        if m.stage=='gravity' and gravity_counts[m.conditions['batch']]>=config['gravity_'+m.conditions['batch']+'_poses']:continue
        result=gate_motion(profile,m,scene['clearance_m'])
        target=folder/'plans'/m.name;target.mkdir(parents=True)
        write_json(target/'motion.json',m.metadata());write_json(target/'collision_reference.json',result)
        curve=m.polynomial();np.savez_compressed(target/'reference.npz',coefficients=curve.c,breakpoints=curve.x)
        write_json(target/'manifest.json',dict(reference_sha256=hashlib.sha256((target/'reference.npz').read_bytes()).hexdigest(),
            motion_sha256=hashlib.sha256((target/'motion.json').read_bytes()).hexdigest(),
            collision_sha256=hashlib.sha256((target/'collision_reference.json').read_bytes()).hexdigest(),reference_gate_passed=result['passed']))
        print(f"碰撞规划 {m.name}: {'PASS' if result['passed'] else 'REJECT'} ({result['queries']} queries)",flush=True)
        if m.stage=='gravity':
            pending_pair.append((m,result))
            if len(pending_pair)==2:
                if all(r['passed'] for _,r in pending_pair):
                    selected.extend(x for x,_ in pending_pair);reports.extend(r for _,r in pending_pair);gravity_counts[m.conditions['batch']]+=1
                else:candidates.extend(dict(motion=x.name,gate=r) for x,r in pending_pair)
                pending_pair=[]
        else:selected.append(m);reports.append(result)
    enough=all(gravity_counts[b]>=config['gravity_'+b+'_poses'] for b in gravity_counts)
    report=dict(schema_version=1,passed=all(r['passed'] for r in reports) and enough,trials=len(selected),
        total_virtual_duration_s=sum(m.duration for m in selected),wall_s=perf_counter()-started,
        control_hz=1e9/config['control_period_ns'],raw_sample_hz=1e9/config['sample_period_ns'],
        stages={s:dict(trials=sum(m.stage==s for m in selected),passed=all(r['passed'] for m,r in zip(selected,reports) if m.stage==s)) for s in dict.fromkeys(m.stage for m in selected)},
        minimum_sampled_distance_m=min(r['minimum_distance_m'] for r in reports),
        rejected=[r['motion'] for r in reports if not r['passed']],
        gravity_pose_counts=gravity_counts,requested_gravity_poses_available=enough,
        selected_trials=[m.name for m in selected],rejected_gravity_candidates=candidates,
        geometry_sha256=profile.metadata['geometry_sha256'],backend=profile.metadata['backend'],
        fixture_geometry_measured=scene['fixture_geometry_measured'],
        status='planned references only; actual Mock trajectories require a separate gate',hardware_executable=False)
    write_json(folder/'plan_report.json',report)
    (folder/'README.md').write_text('# 双臂分阶段实验规划\n\n'+
        f"{len(selected)} 条独立试验，命令频率 {report['control_hz']:g} Hz，设备采样 {report['raw_sample_hz']:g} Hz。参考碰撞门禁：{'通过' if report['passed'] else '拒绝'}。\n\n"+
        '顺序：方向/软件零位 → 通信/驱动 → 多姿态重力 → 多速度运动摩擦 → 起动摩擦 → 阶段验收 → 加速度激励/惯量 → 独立综合验收。\n\n'+
        '每条试验包括初始保持、接近、测量/运动、制动、回到共同姿态和最终保持。没有实机执行入口；夹具是未测量代理。起动力矩斜坡的实际运动尚需在采集后检查。\n\n[规划报告](plan_report.json) · [模型及环境](session.json)\n',encoding='utf-8')
    return report
