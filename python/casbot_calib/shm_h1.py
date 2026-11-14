"""SHM H1: measured-start recipes, explicit reference gravity, immutable observation-time analysis."""
import argparse
import json
from pathlib import Path
import struct
import numpy as np
import yaml
from .data import write_json
from .field_artifacts import seal,check_seal,output_outside,FieldContractError
from .artifact_io import sha256
from .field_motion import jog_motion,static_motion
from .model_runtime import model_for,collision_for,context_for,urdf_for
from .shm_data import ROOT,NAMES,DIRECTION,OFFSET
from .shm_reference import snapshot_positions,export_motion
from .staged_motion import gate_motion,derivative_bounds
from .run_marker import run_marker
from .field_defaults import start_velocity_limit

DEFAULT_PROFILE=ROOT/'configs/shm/hardware_field.yaml'
DEFAULT_EXECUTOR=ROOT/'install/shm-core/bin/shm_executor'


def read(path):return json.loads(Path(path).read_text())


def load_profile(path):
    path=Path(path).resolve();c=yaml.safe_load(path.read_text())
    if c.get('schema_version')!='casbot_shm_h1_v1' or c.get('transport')!='shm_mc':raise ValueError('需要明确的SHM H1 profile')
    if c['command_period_ns']!=1_000_000 or c['start_tolerance_rad']!=.005:raise ValueError('本版原生周期1ms、起点误差0.005rad')
    c['start_velocity_rad_s']=start_velocity_limit(c)
    for key in ('initial_hold_s','final_hold_s','anchor_transition_s','maximum_velocity_rad_s','feedforward_limit_nm'):
        if not np.isfinite(c[key]) or c[key]<=0:raise ValueError('须为正有限值：'+key)
    if c['maximum_velocity_rad_s']>1.5 or c['feedforward_limit_nm']>100:raise ValueError('超出当前现场参考包络')
    for key,maximum in (('kp',500),('kd',50)):
        x=np.broadcast_to(np.asarray(c[key],float),(14,)).copy()
        if not np.isfinite(x).all() or np.any(x<0) or np.any(x>maximum):raise ValueError('逐轴PD范围无效')
        c[key]=x.tolist()
    if c['gravity_feedforward'] not in ('nominal_reference','none'):raise ValueError('显式选择nominal_reference/none')
    for key in ('effort_gain','effort_zero_nm'):
        x=np.broadcast_to(np.asarray(c[key],float),(14,)).copy()
        if not np.isfinite(x).all() or (key=='effort_gain' and np.any(x==0)):raise ValueError('力矩尺度/零点无效')
        c[key]=x.tolist()
    g=np.asarray(c['gravity_world'],float)
    if g.shape!=(3,) or not np.isfinite(g).all() or np.linalg.norm(g)==0:raise ValueError('需要安装重力方向')
    c['scene']=yaml.safe_load((path.parent/c.pop('collision_scene')).read_text())
    c['model_context']=context_for(c).identity
    from .model_runtime import dynamics_scene
    if c['scene']!=dynamics_scene(context_for(c)):raise FieldContractError('H1场景与模型不一致')
    return c


def snapshot(profile,output,*,executor=DEFAULT_EXECUTOR,endpoint='/shm_mc',mock=False,mock_config=None):
    from . import shm_fieldwork as fw
    c=load_profile(profile) if not isinstance(profile,dict) else profile
    root=Path(output).resolve()
    args=['snapshot','--output',str(root),'--executor',str(executor),'--urdf',str(urdf_for(c))]
    if mock:args+=['--mock']
    else:args+=['--endpoint',endpoint]
    if mock:args+=['--mock-config',str(mock_config or ROOT/'configs/shm/h1_mock.json')]
    code=fw.main(args)
    if code:raise ValueError('SHM只读快照失败，见记录：'+str(root))
    write_json(root/'h1_snapshot.json',dict(schema_version='casbot_shm_h1_snapshot_v1',model_context=c['model_context'],
        scope='private_mock' if mock else 'hardware',endpoint=endpoint,codec='orin_35_compat_20261113',**run_marker()))
    seal(root);return root


def measured_start(snapshot_root,config):
    root=Path(snapshot_root).resolve();check_seal(root);metadata=read(root/'h1_snapshot.json')
    if metadata['schema_version']!='casbot_shm_h1_snapshot_v1' or metadata['model_context']!=config['model_context']:
        raise FieldContractError('快照与H1模型不一致')
    raw=(root/'native/status.bin').read_bytes();q=np.asarray(snapshot_positions(root))
    velocity=np.array([DIRECTION[j]*struct.unpack_from('<i',raw,12+(j+12)*16+6)[0]*1e-6 for j in range(14)])
    threshold=start_velocity_limit(config)
    if not np.isfinite(q).all() or np.max(abs(velocity))>=threshold:raise ValueError(f'H1需要完整、当前停稳的起点（|v|<{threshold:g}rad/s）')
    if any(struct.unpack_from('<H',raw,12+j*16+14)[0] for j in range(29)):raise ValueError('快照有驱动故障；不开始H1')
    return q,metadata


def static_recipe(preset,config,*,max_poses_per_split=None,candidate_ids=None):
    root=Path(preset).resolve();check_seal(root);report=read(root/'report.json');engine,source=model_for(config)
    if not report['passed'] or report['source_model_sha256']!=source['sha256'] or report['joint_names']!=list(NAMES):raise FieldContractError('静态预设未通过或模型不匹配')
    if read(root/'scene.json')!=config['scene']:raise FieldContractError('静态预设与H1场景不同')
    if max_poses_per_split is not None and max_poses_per_split<1:raise ValueError('姿态上限须为正整数')
    candidates=read(root/'candidates.json');selected={}
    for arm in ('left','right'):
        for split in ('train','validation'):
            rows=[r for r in candidates if r['status']=='selected' and r['arm']==arm and r['split']==split]
            if max_poses_per_split is not None:rows=rows[:max_poses_per_split]
            selected.update({r['id']:r for r in rows})
    if candidate_ids is not None:
        if not candidate_ids or len(set(candidate_ids))!=len(candidate_ids) or not set(candidate_ids)<=set(selected):raise ValueError('需要不重复的已选静态姿态ID')
        selected={key:selected[key] for key in candidate_ids}
    motions=[dict(read(root/'plans'/name/'motion.json'),trial_name=name) for name in report['planned_execution_order']]
    motions=[m for m in motions if m['conditions']['candidate_id'] in selected]
    if not motions:raise ValueError('没有选中静态参考')
    c=read(root/'config.json')
    return dict(kind='static',home_rad=c['home_rad'],planning_envelope=c['planning_envelope'],motions=motions,
                selection=dict(selected_candidate_ids=list(selected),preset_manifest_sha256=sha256(root/'manifest.json')))


def reference_effort(motion,engine,config,preparation=None):
    if config['gravity_feedforward']=='none':return [dict(t_ns=0,value=[0.]*14)]
    curve=motion.polynomial();last=round(motion.duration*1e9);period=config['command_period_ns'];rows=[];previous=None
    # Keep each actual integer-output change, not millions of repeated hold rows.
    # Store the original float so native truncation is not applied twice.
    for first in range(0,last+period,period*4096):
        times=np.arange(first,min(last+period,first+period*4096),period,dtype=np.int64);times=np.minimum(times,last)
        torque=engine.gravity_batch(curve(times*1e-9))
        if preparation is not None:
            from .shm_ready import handover_fraction
            torque*=handover_fraction(times,preparation)[:,None]
        if not np.isfinite(torque).all() or np.max(abs(torque))>config['feedforward_limit_nm']:raise ValueError('名义重力前馈超出显式力矩范围；不夹取')
        codes=np.trunc(torque*np.asarray(DIRECTION)*100).astype(np.int64)
        for ns,value,code in zip(times,torque,codes):
            if previous is None or np.any(code!=previous):rows.append(dict(t_ns=int(ns),value=value.tolist()));previous=code.copy()
    return rows


def prepare(profile,snapshot_root,output,*,joint_id=12,amplitude_deg=1.,preset=None,max_poses_per_split=None,candidate_ids=None,executor=DEFAULT_EXECUTOR,recipe=None,with_ready=False):
    c=load_profile(profile) if not isinstance(profile,dict) else profile
    start,binding=measured_start(snapshot_root,c);engine,source=model_for(c)
    if recipe is None:
        if preset:recipe=static_recipe(preset,c,max_poses_per_split=max_poses_per_split,candidate_ids=candidate_ids)
        else:
            if joint_id not in range(12,26) or not np.isfinite(amplitude_deg) or not 0<amplitude_deg<=10:raise ValueError('H1往返测试使用ID12–25、幅度(0,10]度')
            recipe=dict(kind='jog',joint=NAMES[joint_id-12],amplitude_deg=amplitude_deg,anchor_rad=None)
        if with_ready:recipe['preparation']='arm_ready_v1'
    ready=None;prefix=None;formal_start=start
    if recipe.get('preparation') is not None:
        if recipe['preparation']!='arm_ready_v1':raise ValueError('未知准备姿态配方')
        from .shm_ready import preparation,ready_position
        prefix,ready=preparation(start,engine,c['maximum_velocity_rad_s']);formal_start=ready_position()
    if recipe['kind']=='static':motion,trials=static_motion(c,formal_start,recipe)
    elif recipe['kind']=='jog':motion,_=jog_motion(c,formal_start,list(NAMES),recipe['joint'],recipe['amplitude_deg'],anchor_rad=recipe['anchor_rad']);trials=[]
    else:raise ValueError('未知H1配方')
    # Time rounding changes the reference: normalize BEFORE all certificates.
    motion.knots=[round(t*1e9)*1e-9 for t in motion.knots]
    for trial in trials:
        for key in ('begin_s','end_s','measure_begin_s','measure_end_s'):trial[key]=round(trial[key]*1e9)*1e-9
    root=output_outside(output,snapshot_root,*([preset] if preset else []));root.mkdir(parents=True,exist_ok=False)
    gate=gate_motion(collision_for(c,engine),motion,c['scene']['clearance_m'])
    if ready is not None:
        gate.update(scope='formal_calibration_reference_only',execution_offset_ns=ready['calibration_begin_ns'])
        write_json(root/'preparation.json',ready);write_json(root/'calibration_motion.json',motion.metadata())
    write_json(root/'collision_reference.json',gate)
    if not gate['passed']:seal(root);raise FieldContractError('H1连续参考碰撞/URDF检查未通过；没有发送命令')
    if np.max(derivative_bounds(motion.polynomial()))>c['maximum_velocity_rad_s']*(1+1e-9):raise FieldContractError('H1参考速度越限')
    if ready is not None:
        from .shm_ready import prepend
        motion=prepend(prefix,motion)
        for trial in trials:
            for key in ('begin_s','end_s','measure_begin_s','measure_end_s'):
                trial[key]=(ready['calibration_begin_ns']+round(trial[key]*1e9))*1e-9
    bundle=export_motion(motion,kp=c['kp'],kd=c['kd'],effort=reference_effort(motion,engine,c,ready))
    bundle.update(purpose='shm_h1_ready_then_collision_checked' if ready else 'shm_h1_collision_checked',gravity_feedforward=c['gravity_feedforward'],
                  observed_velocity_limit_rad_s=c['maximum_velocity_rad_s'])
    if ready is not None:
        from .shm_ready import pd_schedule
        bundle.update(pd_schedule=pd_schedule(ready,c['kp'],c['kd']),markers=[
            dict(t_ns=0,event='ready_begin'),dict(t_ns=ready['move_end_ns'],event='ready_field_handover_begin'),
            dict(t_ns=ready['calibration_begin_ns'],event='h1_calibration_begin')])
    if trials:
        markers=bundle.setdefault('markers',[])
        for i,trial in enumerate(trials,1):
            markers.extend([dict(t_ns=round(trial['measure_begin_s']*1e9),event=f'static_{i:02d}_measure_begin'),
                            dict(t_ns=round(trial['measure_end_s']*1e9),event=f'static_{i:02d}_measure_end')])
        markers.sort(key=lambda row:row['t_ns'])
    write_json(root/'reference.json',bundle);write_json(root/'motion.json',motion.metadata())
    binary=Path(executor).resolve()
    if not binary.is_file():raise ValueError('找不到H1执行器：'+str(binary))
    metadata=dict(schema_version='casbot_shm_h1_plan_v1',config=c,model_context=c['model_context'],joint_names=list(NAMES),
        source_model_sha256=source['sha256'],stage=motion.stage,trials=trials,recipe=recipe,duration_ns=round(motion.duration*1e9),
        scope=binding['scope'],endpoint=binding['endpoint'],snapshot_manifest_sha256=sha256(Path(snapshot_root)/'manifest.json'),
        executor_sha256=sha256(binary),reference_sha256=sha256(root/'reference.json'),
        time_quantization='knots rounded to integer nanoseconds before collision/velocity proof',
        hidden_compensation=False,hardware_calibration_accepted=False,**run_marker())
    if ready is not None:metadata['preparation']=ready
    if binding['scope']=='private_mock':
        fixture=read(Path(snapshot_root)/'mock/config.json');fixture['urdf']=str(urdf_for(c));write_json(root/'mock.json',fixture)
    write_json(root/'plan.json',metadata);seal(root);return metadata


def execution_plan(plan,executor,endpoint,mock):
    plan=Path(plan).resolve();check_seal(plan);metadata=read(plan/'plan.json')
    if metadata['schema_version']!='casbot_shm_h1_plan_v1':raise ValueError('需要SHM H1计划')
    if (metadata['scope']=='private_mock')!=mock:raise FieldContractError('快照/计划的真实或Mock范围不同；不自动切换目标')
    if not mock and endpoint!=metadata['endpoint']:raise FieldContractError('执行端点与快照不同')
    if sha256(executor)!=metadata['executor_sha256']:raise FieldContractError('执行器已变化，须重建H1计划')
    engine,source=model_for(metadata['config'])
    if source['sha256']!=metadata['source_model_sha256'] or context_for(metadata['config']).identity!=metadata['model_context']:raise FieldContractError('H1模型绑定变化')
    return metadata


def _execute_once(plan,output,*,executor=DEFAULT_EXECUTOR,endpoint='/shm_mc',mock=False,hand_background=None,do_execute=False):
    from . import shm_fieldwork as fw
    plan=Path(plan).resolve();metadata=execution_plan(plan,executor,endpoint,mock)
    args=['execute',str(plan/'reference.json'),'--executor',str(executor),'--urdf',str(urdf_for(metadata['config'])),'--output',str(output)]
    if mock:args+=['--mock','--mock-config',str(plan/'mock.json')]
    else:args+=['--endpoint',endpoint]
    if hand_background:args+=['--hand-background',str(hand_background)]
    if do_execute:args+=['--execute']
    if not do_execute:return fw.main(args)
    print('H1封存计划及模型/正式参考碰撞证明已核对，开始原生执行。',flush=True)
    if metadata.get('preparation'):
        print('先执行URDF-only准备段：双肩roll +20°/-20°，其余双臂0°，Kp20/Kd2；不中断使能接入H1。',flush=True)
    code=fw.main(args);root=Path(output)
    if root.is_dir():
        copied=root/'native/reference.json'
        if copied.exists() and sha256(copied)!=metadata['reference_sha256']:raise FieldContractError('实际执行参考与H1封存计划不一致')
        write_json(root/'h1_capture.json',dict(schema_version='casbot_shm_h1_capture_v1',plan_manifest_sha256=sha256(plan/'manifest.json'),
            executor_sha256=metadata['executor_sha256'],reference_sha256=metadata['reference_sha256'],scope=metadata['scope'],returncode=code,**run_marker()))
        seal(root)
    return code



def execute(plan,output,*,executor=DEFAULT_EXECUTOR,endpoint='/shm_mc',mock=False,hand_background=None,do_execute=False):
    options=dict(executor=executor,endpoint=endpoint,mock=mock,hand_background=hand_background)
    if not do_execute:return _execute_once(plan,output,**options)
    original=Path(plan).resolve();metadata=execution_plan(original,executor,endpoint,mock)
    root=output_outside(output,original);root.mkdir(parents=True,exist_ok=False)
    effective=original;attempts=[];index=0
    while True:
        directory=root/'attempts'/f'{index:04d}';directory.mkdir(parents=True)
        capture=directory/'capture'
        code=_execute_once(effective,capture,do_execute=True,**options)
        native=read(capture/'result.json').get('native') or {}
        reason=native.get('reason','');zero=int(native.get('commands_sent',0))==0 and int(native.get('first_shm_write_mono_ns',0))==0
        status=capture/'native/status.bin'
        raw_status=status.read_bytes() if status.is_file() else b''
        no_fault=len(raw_status)==696 and not any(struct.unpack_from('<H',raw_status,12+j*16+14)[0] for j in range(29))
        eligible=code!=0 and zero and no_fault and reason in (
            'Reference start differs from fresh measured position; no commands sent',
            'Start changed during preflight; no commands sent')
        attempts.append(dict(index=index,capture=str(capture.relative_to(root)),returncode=code,reason='start_replan' if eligible else reason))
        summary=dict(schema_version='casbot_shm_h1_collection_v1',execution_completed=code==0,returncode=code,
            original_plan_manifest_sha256=sha256(original/'manifest.json'),attempts=attempts,
            effective_plan=None if effective==original else str(effective.relative_to(root)),
            effective_capture=str(capture.relative_to(root)),resume_permitted=False)
        write_json(root/'result.json',summary)
        if not eligible:seal(root);return code
        print('起点变化，原生证明零写入且未进入使能；重新只读快照并按原配方规划。',flush=True)
        fresh=directory/'refresh'
        try:
            snapshot(metadata['config'],fresh,executor=executor,endpoint=endpoint,mock=mock,
                     mock_config=original/'mock.json' if mock else None)
            effective=directory/'replanned'
            replan(original,fresh,effective,executor=executor)
        except (ValueError,OSError) as error:
            summary.update(execution_completed=False,returncode=2,replan_error=str(error));write_json(root/'result.json',summary);seal(root);return 2
        index+=1


def replan(plan,snapshot_root,output,*,executor=DEFAULT_EXECUTOR):
    root=Path(plan);check_seal(root);m=read(root/'plan.json')
    if sha256(executor)!=m['executor_sha256']:raise FieldContractError('执行器变化，不能延续旧配方')
    _,binding=measured_start(snapshot_root,m['config'])
    if binding['scope']!=m['scope'] or binding['endpoint']!=m['endpoint']:raise FieldContractError('重规划快照来自不同连接范围')
    return prepare(m['config'],snapshot_root,output,executor=executor,recipe=m['recipe'])


def analyse(plan,capture,output,*,windows_only=False):
    from .shm_h1_analysis import analyse as implementation
    return implementation(plan,capture,output,windows_only=windows_only)


def main(argv=None):
    p=argparse.ArgumentParser(description='SHM H1：快照、规划、显式执行、静态重力分析；不依赖ROS')
    sub=p.add_subparsers(dest='action',required=True)
    for action in ('snapshot','preset-small','plan-jog','plan-static','execute','replan','analyse'):
        a=sub.add_parser(action);a.add_argument('--output',type=Path,required=True)
        if action in ('snapshot','preset-small','plan-jog','plan-static'):a.add_argument('--profile',type=Path,default=DEFAULT_PROFILE)
        if action=='preset-small':
            from .shm_static_small import DEFAULT_SETTINGS
            a.add_argument('--config',type=Path,default=DEFAULT_SETTINGS)
        if action in ('snapshot','plan-jog','plan-static','execute','replan'):a.add_argument('--executor',type=Path,default=DEFAULT_EXECUTOR)
        if action in ('plan-jog','plan-static','replan'):a.add_argument('snapshot',type=Path)
        if action in ('execute','replan','analyse'):a.add_argument('plan',type=Path)
        if action=='plan-static':
            a.add_argument('--preset',type=Path,required=True);a.add_argument('--max-poses-per-split',type=int);a.add_argument('--candidate-id',action='append')
        if action=='plan-jog':
            a.add_argument('--joint-id',type=int,choices=range(12,26),default=12);a.add_argument('--amplitude-deg',type=float,default=1.)
        if action in ('plan-jog','plan-static'):
            a.add_argument('--with-ready',action='store_true',help='先脚本复位准备姿态（仅准备段跳过碰撞；URDF/速度仍检查），同次使能接正式H1')
        if action in ('snapshot','execute'):
            a.add_argument('--mock',action='store_true');a.add_argument('--endpoint',default='/shm_mc')
        if action=='snapshot':a.add_argument('--mock-config',type=Path)
        if action=='execute':
            a.add_argument('--execute',action='store_true');a.add_argument('--hand-background',type=Path)
        if action=='analyse':a.add_argument('capture',type=Path);a.add_argument('--windows-only',action='store_true')
    a=p.parse_args(argv)
    try:
        if a.action=='snapshot':snapshot(a.profile,a.output,executor=a.executor,endpoint=a.endpoint,mock=a.mock,mock_config=a.mock_config)
        elif a.action=='preset-small':
            from .shm_static_small import make_preset
            make_preset(a.profile,a.output,settings=a.config)
        elif a.action in ('plan-jog','plan-static'):
            prepare(a.profile,a.snapshot,a.output,executor=a.executor,with_ready=a.with_ready,**(dict(joint_id=a.joint_id,amplitude_deg=a.amplitude_deg) if a.action=='plan-jog' else dict(preset=a.preset,max_poses_per_split=a.max_poses_per_split,candidate_ids=a.candidate_id)))
        elif a.action=='execute':return execute(a.plan,a.output,executor=a.executor,endpoint=a.endpoint,mock=a.mock,hand_background=a.hand_background,do_execute=a.execute)
        elif a.action=='replan':replan(a.plan,a.snapshot,a.output,executor=a.executor)
        else:analyse(a.plan,a.capture,a.output,windows_only=a.windows_only)
        print('H1结果：'+str(a.output));return 0
    except (ValueError,OSError,KeyError) as error:print('H1错误：'+str(error),file=__import__('sys').stderr);return 2


if __name__=='__main__':raise SystemExit(main())
