"""Chinese recording playback with nominal collision envelopes; no dynamics."""
import hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .arm_viewer import _values,_pose_attributes
from .collision import nominal_profile
from .dual_model import nominal_dual_arm
from .telemetry import JointMapping,TelemetryAdapter,StateFrame
from .data import write_json
from .staged_collection import load_trial
from .dual_presentation import _trace


def write_scene(profile,path):
    model=profile.engine.model;root=ET.Element('mujoco',model='staged_dual_geometry_only')
    ET.SubElement(root,'compiler',angle='radian');ET.SubElement(root,'option',gravity='0 0 0')
    default=ET.SubElement(root,'default');ET.SubElement(default,'geom',contype='0',conaffinity='0',density='0')
    asset=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='1 -1 2',dir='-1 1 -2',diffuse='.9 .9 .9')
    ET.SubElement(world,'geom',name='floor',type='plane',pos='0 0 -.8',size='2 2 .05',rgba='.07 .10 .15 1')
    bodies=[]
    for k,j in enumerate(model.joints):
        parent=world if model.parents[k]<0 else bodies[model.parents[k]]
        body=ET.SubElement(parent,'body',name=j.name+'_view',**_pose_attributes(dict(rotation=j.origin.rotation,translation=j.origin.translation)));bodies.append(body)
        ET.SubElement(body,'inertial',pos='0 0 0',mass='1',diaginertia='1 1 1')  # unused viewer compilation scaffold
        ET.SubElement(body,'joint',name=j.name,type='hinge',axis=_values(j.axis),limited='false')
        ET.SubElement(body,'geom',name=j.name+'_marker',type='sphere',size='.013',rgba='1 .75 .25 1')
        for axis,tint in zip(np.eye(3),('1 .25 .25 1','.3 1 .4 1','.3 .5 1 1')):
            ET.SubElement(body,'geom',type='capsule',size='.002',fromto=_values(np.r_[np.zeros(3),axis*.055]),rgba=tint)
    for index,g in enumerate(profile.geometry):
        name='envelope_'+str(index);ET.SubElement(asset,'mesh',name=name,vertex=_values(g.vertices.ravel()),face=' '.join(str(int(x)) for x in g.triangles.ravel()))
        parent=world if g.parent<0 else bodies[g.parent]
        color='.36 .43 .53 .7' if g.parent<0 else ('.1 .72 .64 .82' if g.parent<7 else '.25 .52 .88 .82')
        if 'fixture' in g.name:color='1 .62 .16 .9'
        ET.SubElement(parent,'geom',name=g.name+'_mesh',type='mesh',mesh=name,rgba=color,
            **_pose_attributes(dict(rotation=g.placement.rotation,translation=g.placement.translation)))
    ET.indent(root,space='  ');Path(path).write_text(ET.tostring(root,encoding='unicode')+'\n',encoding='utf-8')
    return TelemetryAdapter('casbot_20251015_dual_mock',[JointMapping(j.name,j.name) for j in model.joints])


def _story(root):
    rows=[]
    def add(key,title,run,seconds,joint=3,begin=None,end=None):
        meta,_=load_trial(root,run)
        if begin is None:begin=.1
        if end is None:end=meta['duration_s']-.1
        rows.append(dict(key=key,title=title,run=run,seconds=seconds,joint=joint,start_s=begin,end_s=end))
    for j in range(14):add('direction','逐轴方向与编码器软件零位',f'direction_{j:02}',1.4,j,begin=.6,end=4.2)
    for name in ('timing_train','timing_validation','timing_frequency_250'):add('timing','驱动响应、延迟与频率对照',name,2.,0,begin=.02,end=.2)
    for p in (0,2):add('gravity','正反接近、多姿态静态重力',f'gravity_train_{p:02}_-1',3.,3)
    for speed in (.04,.14):add('friction','三档速度、正反匀速与采样收敛',f'friction_validation_00_03_{speed:g}',3.,3)
    for run in ('breakaway_0_03_+1_0.5_0.24_0','breakaway_1_06_-1_1.5_0.12_1'):
        meta,_=load_trial(root,run);k=meta['labels'].index('torque_ramp');event=json.loads((root/'runs'/run/'guard_events.json').read_text())['events'][0]
        add('breakaway','重力补偿起动、反馈确认与恢复保持',run,4.,meta['conditions']['joint'],meta['knots_s'][k]-.1,event['decision_ns']*1e-9+.25)
    add('acceptance','前序验收通过、冻结重力与摩擦','gravity_train_00_+1',3.,3,begin=2.7,end=3.7)
    for run in ('wrist_probe_1_04_+1','inertia_train'):
        meta,_=load_trial(root,run);label='probe_measure' if 'probe' in run else 'periodic';indices=[i for i,x in enumerate(meta['labels']) if x==label]
        start=meta['knots_s'][indices[0]]+(0 if 'probe' in run else 12.)
        add('inertia','加速度激励与腕部惯量专项验证',run,5.,4 if 'probe' in run else 3,start,min(start+6.,meta['knots_s'][indices[-1]+1]))
    for run in ('inertia_validation','inertia_left_validation','inertia_right_validation'):
        meta,_=load_trial(root,run);k=meta['labels'].index('periodic');start=meta['knots_s'][k]+12.
        add('validation','冻结模型、双臂与逐臂独立验证',run,2.5,10 if 'right' in run else 3,start,start+6.)
    run='control_identified_original';meta,_=load_trial(root,run);k=meta['labels'].index('periodic');start=meta['knots_s'][k]+12.
    add('control','原模型与辨识模型、原增益与优化增益',run,7.,3,start,start+6.)
    return rows


def render_staged(folder,output=None,fps=12,video=True,ros_report=None):
    if (Path(folder)/'final_protocol.json').exists():
        if ros_report is not None:raise ValueError('R4 playback labels reused virtual preflight; ROS benchmarks remain separate evidence')
        from .staged_final_presentation import render_final
        return render_final(folder,output,fps=fps,video=video)
    from PIL import Image,ImageDraw
    from .presentation import _font
    from .viewer import JointView
    root=Path(folder);output=Path(output) if output else root/'presentation';output.mkdir(parents=True,exist_ok=False)
    workflow=json.loads((root/'workflow.json').read_text())
    if not workflow['passed']:raise ValueError('An accepted staged calibration/control workflow is required')
    read=lambda path:json.loads((root/path).read_text())
    reports={name:read(value['report']) for name,value in workflow['stages'].items()};control=read('control_comparison.json');direction=read('direction_check.json');timing=read('driver_timing.json')
    session=read('session.json');engine,_=nominal_dual_arm();profile=nominal_profile(scene=session['collision']['scene']);adapter=write_scene(profile,output/'scene.xml')
    external={};ros_case=None
    if ros_report is not None:
        ros_path=Path(ros_report).resolve();external[str(ros_path)]=hashlib.sha256(ros_path.read_bytes()).hexdigest()
        ros=json.loads(ros_path.read_text())
        if not ros['passed'] or ros.get('hardware_tested') or ros.get('physical_network_tested'):raise ValueError('Expected accepted local ROS Mock report')
        ros_case=next(c for c in ros['cases'] if c['name']=='train_500_500')
    story=_story(root);cache={};paths={root/'session.json',root/'workflow.json',root/'control_comparison.json'}
    paths.update(root/value['report'] for value in workflow['stages'].values())
    for row in story:
        run=row['run']
        if run in cache:continue
        with np.load(root/'runs'/run/'observations.npz',allow_pickle=False) as data:obs={k:data[k] for k in data.files}
        meta,curve=load_trial(root,run);cache[run]=(obs,meta,curve)
        paths.update(root/'runs'/run/file for file in ('observations.npz','commands.npz','metadata.json'))
    previous=root/'runs'/'control_nominal_original'/'observations.npz'
    with np.load(previous,allow_pickle=False) as data:old_q=data['q'].copy();old_time=data['sample_ns'].copy()
    paths.add(previous);before={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    fonts={n:_font(n) for n in (12,14,16,18,22,26)};text=(228,237,245);muted=(151,172,192);gold=(245,194,91);teal=(62,218,189)
    labels=['肩俯仰','肩滚转','肩偏航','肘俯仰','腕偏航','腕俯仰','腕滚转'];tabs=['方向零位','驱动时序','静态重力','运动摩擦','起动摩擦','阶段验收','惯量激励','独立验证','控制对照']
    keys=['direction','timing','gravity','friction','breakaway','acceptance','inertia','validation','control'];writer=None;errors=[];frames=0;snapshots=set();clips=[]
    if video:
        import imageio_ffmpeg
        writer=imageio_ffmpeg.write_frames(str(output/'staged_calibration_zh.mp4'),(1280,720),fps=fps,codec='libx264',pix_fmt_in='rgb24',pix_fmt_out='yuv420p',macro_block_size=16);writer.send(None)
    try:
        with JointView(adapter,output/'scene.xml',width=800,height=420) as view:
            view.camera.lookat[:]=[0.,0.,.18];view.camera.distance=1.35;view.camera.azimuth=190;view.camera.elevation=-12
            original=view.model.geom_rgba.copy()
            for row in story:
                obs,meta,curve=cache[row['run']];j=row['joint'];start,end=row['start_s'],row['end_s'];mask=(obs['sample_ns']*1e-9>=start)&(obs['sample_ns']*1e-9<=end);t=obs['sample_ns'][mask]*1e-9
                if len(t)<2:raise ValueError('Playback segment has no observations')
                ref=curve(t)[:,j];unit='关节角度 / 度';curves=[('目标',np.degrees(ref),gold),('采集',np.degrees(obs['q'][mask,j]),teal)]
                if row['key']=='timing':unit='采集电流 / 安';curves=[('实际采集',obs['current'][mask,j],teal)]
                elif row['key']=='breakaway':unit='电流换算力矩 / 牛米';curves=[('实际采集',obs['current'][mask,j]*session['config']['drive']['gain_nm_a'],teal)]
                elif row['key']=='control':
                    previous_q=np.interp(obs['sample_ns'][mask],old_time,old_q[:,j]);unit='位置误差 / 毫弧度'
                    curves=[('原模型、原增益',(previous_q-ref)*1000,(249,112,119)),('辨识后、原增益',(obs['q'][mask,j]-ref)*1000,teal)]
                count=max(1,round(row['seconds']*fps)) if video else 1
                for k in range(count):
                    at=start+(end-start)*(k/max(1,count-1) if video else .5);index=int(np.clip(np.searchsorted(obs['sample_ns'],round(at*1e9)),0,len(obs['sample_ns'])-1))
                    frame=StateFrame(adapter.robot_model_id,'recorded staged mock',int(obs['sequence'][index]),int(obs['sample_ns'][index]),int(obs['receive_ns'][index]),tuple(session['joint_names']),obs['q'][index],obs['dq'][index],obs['current'][index],None,clock_domain='synthetic_acquisition')
                    view.camera.azimuth=230 if j<7 else 145;view.model.geom_rgba[:]=original
                    rgb=view.render(frame);expected=engine.fk(frame.q);error=0.
                    for n,joint in enumerate(engine.model.joints):
                        body=view.mj.mj_name2id(view.model,view.mj.mjtObj.mjOBJ_BODY,joint.name+'_view')
                        error=max(error,float(np.max(abs(view.data.xpos[body]-expected[n][:3,3]))),float(np.max(abs(view.data.xmat[body].reshape(3,3)-expected[n][:3,:3]))))
                    errors.append(error)
                    if np.any(view.data.qacc):raise RuntimeError('Geometry viewer unexpectedly computed dynamics')
                    distance=float(profile.check([frame.q])['distances'].min())
                    canvas=Image.new('RGB',(1280,720),(10,17,27));draw=ImageDraw.Draw(canvas)
                    draw.text((24,12),'CASBOT / 双臂分阶段动力学标定',font=fonts[26],fill=text)
                    active=keys.index(row['key'])
                    for n,label in enumerate(tabs):
                        x=24+n*138;draw.rounded_rectangle((x,52,x+128,78),5,fill=teal if n==active else (18,29,43));draw.text((x+7,57),f'{n+1:02} {label}',font=fonts[12],fill=(10,17,27) if n==active else muted)
                    draw.text((24,90),row['title'],font=fonts[18],fill=teal);canvas.paste(Image.fromarray(rgb),(24,120));draw.rounded_rectangle((848,120,1256,658),10,fill=(18,29,43),outline=(40,58,76))
                    draw.text((868,137),('左臂 ' if j<7 else '右臂 ')+labels[j%7],font=fonts[22],fill=gold)
                    metrics=[('实际角度',f'{np.degrees(frame.q[j]):+.3f} 度'),('实际速度',f'{frame.dq[j]:+.4f} 弧度/秒'),('当前最小几何间隙',f'{distance*1000:.2f} 毫米')]
                    key=row['key']
                    if key=='direction':
                        r=direction['axes'][j];metrics += [('软件方向 / 零位',f"{r['direction']:+d} / {r['software_zero_ticks']} 计数"),('机械零位','待独立实测确认')]
                    elif key=='timing':
                        fit=timing['axes'][j]['fit'];metrics=[('默认命令 / 控制 / 发布','500 / 500 / 500 赫兹'),('电流响应时间常数',f"{fit['current_tau_s']*1000:.3f} 毫秒"),('拟合指令延迟',f"{fit['command_delay_s']*1000:.3f} 毫秒"),('当前画面','虚拟时钟驱动测试')]
                        if ros_case is not None:
                            rates=[ros_case[k]['hz'] for k in ('command_publish','command_adoption','feedback_receive')]
                            metrics += [('本机 ROS 发布 / 采用 / 接收',' / '.join(f'{value:.1f}' for value in rates)+' 赫兹'),
                                ('本机指令传输 P99',f"{ros_case['command_transport']['milliseconds']['p99']:.3f} 毫秒")]
                        metrics += [('实机以太网频率 / 延迟','待实测，不由本机回环推断')]
                    elif key in ('gravity','acceptance'):
                        metrics += [('重力可观测组合',f"{reports['gravity']['fit']['rank']} / 140"),('独立配对最大误差',f"{reports['gravity']['validation']['maximum_absolute_rmse_nm']:.6f} 牛米"),('前序参数','先验收，再冻结进入惯量阶段')]
                    elif key=='friction':
                        f=reports['friction'];metrics += [('黏滞摩擦系数',f"{f['viscous'][j]:.5f} 牛米秒/弧度"),('库仑摩擦',f"{f['coulomb'][j]:.5f} 牛米"),('原始采样收敛对照','4000 / 8000 赫兹')]
                    elif key=='breakaway':
                        r=reports['breakaway']['axes'][j];metrics += [('等效起动阈值',f"{r['equivalent_symmetric_static_nm']:.5f} 牛米"),('确认方式','位移 + 持续速度，随后恢复保持'),('起动估计','回溯起动区间，不用较晚确认力矩')]
                    else:
                        metrics += [('刚体回归可观测组合',f"{reports['inertia']['initial_fit']['rank']} / 140"),('独立力矩预测最大误差',f"{max(v['maximum_normalized_rmse'] for v in reports['inertia']['validation'].values()):.2%}")]
                        if key=='inertia' and 'probe' in row['run']:
                            probe=reports['inertia']['probe_validation'][row['run']]
                            metrics += [('独立惯量复核差异',f"{probe['relative_inertia_disagreement']:.2%}"),('输出侧等效惯量',f"{probe['model_joint_inertia_kg_m2']:.3g} 千克米²")]
                        if key=='control':metrics += [('固定增益，前馈收益',f"误差下降 {control['fixed_pd_feedforward_rmse_reduction']:.1%}"),('原模型调参收益',f"误差下降 {control['nominal_model_pd_rmse_reduction']:.1%}"),('五档增益选择',f"比例 {control['selected_kp_factor']:.2f}，独立轨迹验收")]
                    for n,(label,value) in enumerate(metrics[:8]):
                        y=183+n*56;draw.text((868,y),label,font=fonts[14],fill=muted);draw.text((868,y+22),value,font=fonts[16],fill=text)
                    _trace(draw,(24,548,824,658),t,curves,frame.sample_ns*1e-9,unit,fonts[12])
                    draw.text((24,671),f"记录选段 {(end-start)/row['seconds']:.2f} 倍速回放 / 真实角度未放大 / 采样时刻 {frame.sample_ns*1e-9:.3f} 秒",font=fonts[14],fill=muted)
                    draw.text((24,696),'合成实验 / CAD 凸包络 / 腰部夹具为未测量代理 / MuJoCo 仅显示几何 / 未连接实机',font=fonts[12],fill=gold)
                    if key not in snapshots and (not video or k>=count//2):canvas.save(output/f'{active+1:02}_{key}.png');snapshots.add(key)
                    if writer:writer.send(np.asarray(canvas))
                    frames+=1
                clips.append(row)
    finally:
        if writer:writer.close()
    if before!={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}:raise RuntimeError('Playback modified calibration inputs')
    if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=digest for p,digest in external.items()):raise RuntimeError('External ROS report changed during playback')
    result=dict(passed=bool(errors and max(errors)<1e-12),frames=frames,max_fk_error=max(errors),duration_s=frames/fps if video else 0.,
        truth_read=False,angles_amplified=False,source_sha256=before,external_source_sha256=external,clips=clips,mujoco_usage='mj_kinematics/mj_camlight and rendering only; no integration, dynamics or collision')
    write_json(output/'presentation.json',result)
    if not result['passed']:raise RuntimeError('Pinocchio/MuJoCo geometry mismatch')
    return result
