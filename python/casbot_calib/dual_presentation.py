"""Chinese dual-arm recording playback. MuJoCo is geometry-only, never a plant."""
import hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .arm_viewer import _values,_pose_attributes
from .dual_model import nominal_dual_arm
from .dual_identification import load_observations
from .data import write_json
from .telemetry import JointMapping,TelemetryAdapter,StateFrame


def write_dual_scene(engine,path):
    model=engine.model;root=ET.Element('mujoco',model='nominal_dual_arm_geometry_only')
    ET.SubElement(root,'compiler',angle='radian');ET.SubElement(root,'option',gravity='0 0 0')
    default=ET.SubElement(root,'default');ET.SubElement(default,'geom',contype='0',conaffinity='0')
    world=ET.SubElement(root,'worldbody');ET.SubElement(world,'light',pos='1 -1 2',dir='-1 1 -2',diffuse='.9 .9 .9')
    ET.SubElement(world,'geom',type='plane',pos='0 0 -.8',size='2 2 .05',rgba='.07 .1 .16 1')
    neutral=np.array(engine.fk(np.zeros(14)))[:14,:3,3];roots=[i for i,p in enumerate(model.parents) if p<0]
    shoulder=neutral[roots].mean(axis=0)
    # Schematic torso/clamp, deliberately no photo-derived measurement coordinates.
    ET.SubElement(world,'geom',name='torso_schematic',type='box',pos=_values(shoulder-[0,0,.15]),size='.065 .12 .17',rgba='.22 .29 .37 .45')
    ET.SubElement(world,'geom',name='waist_clamp_schematic',type='box',pos=_values(shoulder-[0,0,.32]),size='.09 .15 .025',rgba='1 .65 .2 1')
    bodies=[]
    for i,joint in enumerate(model.joints):
        parent=world if model.parents[i]<0 else bodies[model.parents[i]]
        pose=dict(rotation=joint.origin.rotation,translation=joint.origin.translation)
        body=ET.SubElement(parent,'body',name=joint.name+'_view',**_pose_attributes(pose));bodies.append(body)
        ET.SubElement(body,'joint',name=joint.name,type='hinge',axis=_values(joint.axis),limited='false')
        color='.1 .8 .68 1' if i<7 else '.3 .55 1 1'
        ET.SubElement(body,'geom',name=joint.name+'_ball',type='sphere',size='.033',rgba=color)
        children=[k for k,p in enumerate(model.parents) if p==i]
        end=np.array(model.joints[children[0]].origin.translation) if children else np.array([0.,0.,-.09])
        if np.linalg.norm(end)>1e-5:ET.SubElement(body,'geom',type='capsule',size='.024',fromto=_values(np.r_[np.zeros(3),end]),rgba=color)
        # Asymmetric local axes make axial wrist rotations visible without altering angles.
        for axis,tint in zip(np.eye(3),('1 .25 .25 1','.3 1 .4 1','.3 .5 1 1'),strict=True):
            ET.SubElement(body,'geom',type='capsule',size='.003',fromto=_values(np.r_[np.zeros(3),axis*.065]),rgba=tint)
        if not children:
            ET.SubElement(body,'geom',type='box',pos='.02 0 -.09',size='.035 .018 .008',rgba=color)
            ET.SubElement(body,'geom',type='box',pos='.05 .025 -.09',size='.025 .006 .008',rgba='1 .75 .25 1')
    ET.indent(root,space='  ');Path(path).write_text(ET.tostring(root,encoding='unicode')+'\n',encoding='utf-8')
    return TelemetryAdapter('casbot_20251015_dual_mock',[JointMapping(j.name,j.name) for j in model.joints])


def _legacy_storyboard(folder,session,clip_seconds):
    seconds=clip_seconds or 3.;motion=session['motion'];start=motion['period_s']*motion['discard_periods']
    cases=[('gravity_train','固定边界与原始采集'),('gravity_train','重力保持'),('friction_train','摩擦采集'),
        ('dynamic_train','多频激励'),('dynamic_train','参数辨识'),('dynamic_validation','独立验证'),
        ('left_only_validation','单臂验证'),('identified_control_validation','前馈对照')]
    return dict(stages=[dict(key='legacy',title=title,segments=[dict(run=name,start_s=start if name!='gravity_train' else 0.,end_s=(start if name!='gravity_train' else 0.)+seconds,seconds=seconds,joint=3)]) for name,title in cases])


def _trace(draw,box,time,curves,cursor,unit,font):
    x0,y0,x1,y1=box;draw.rounded_rectangle(box,8,fill=(17,28,42),outline=(40,58,76))
    if not len(time):return
    low=min(float(np.min(values)) for _,values,_ in curves);high=max(float(np.max(values)) for _,values,_ in curves)
    margin=max((high-low)*.12,1e-5);low-=margin;high+=margin
    left,top,right,bottom=x0+52,y0+27,x1-15,y1-22
    def point(t,value):return (left+(t-time[0])/max(time[-1]-time[0],1e-9)*(right-left),bottom-(value-low)/(high-low)*(bottom-top))
    draw.text((x0+9,y0+4),unit,font=font,fill=(139,160,180))
    draw.text((x0+5,top),f'{high:.2g}',font=font,fill=(139,160,180));draw.text((x0+5,bottom-12),f'{low:.2g}',font=font,fill=(139,160,180))
    stride=max(1,len(time)//700)
    for index,(name,values,color) in enumerate(curves):
        points=[point(t,value) for t,value in zip(time[::stride],values[::stride],strict=True)]
        if len(points)>1:draw.line(points,fill=color,width=2)
        draw.text((x0+160+index*185,y0+4),name,font=font,fill=color)
    x=point(np.clip(cursor,time[0],time[-1]),0)[0];draw.line((x,top,x,bottom),fill=(255,195,78),width=1)
    draw.text((left,bottom+3),f'{time[0]:.3f} s',font=font,fill=(139,160,180))
    draw.text((right-72,bottom+3),f'{time[-1]:.3f} s',font=font,fill=(139,160,180))


def render_dual_workflow(folder,output=None,fps=12,clip_seconds=None,video=True):
    from PIL import Image,ImageDraw
    from .presentation import _font
    from .viewer import JointView
    folder=Path(folder);output=Path(output) if output else folder/'presentation'
    if fps<=0 or (clip_seconds is not None and clip_seconds<=0):raise ValueError('Positive frame rate/duration required')
    output.mkdir(parents=True,exist_ok=False)
    def read(path):return json.loads((folder/path).read_text(encoding='utf-8'))
    session=read('session.json');report=read('identification/report.json');control=read('control_comparison.json')
    from .dual_preflight import verify_preflight
    verify_preflight(folder,session)
    story=read('storyboard.json') if (folder/'storyboard.json').exists() else _legacy_storyboard(folder,session,clip_seconds)
    direction=read('direction_check.json') if (folder/'direction_check.json').exists() else None
    timing=read('driver_timing.json') if (folder/'driver_timing.json').exists() else None
    engine,source=nominal_dual_arm();model=engine.model;names=[j.name for j in model.joints]
    if session['joint_names']!=names or session['source_sha256']!=source['sha256']:raise ValueError('Viewer/session model mismatch')
    adapter=write_dual_scene(engine,output/'scene.xml')
    write_json(output/'mapping.json',dict(robot_model_id=adapter.robot_model_id,joints=[m.__dict__ for m in adapter.mappings]))
    runs={clip['run'] for stage in story['stages'] for clip in stage['segments']}
    runs.add('dynamic_validation');cache={};paths={folder/name for name in ('session.json','identification/report.json','control_comparison.json')}
    for name in ('storyboard.json','preflight.json','direction_check.json','driver_timing.json','motion_coverage.json'):
        if (folder/name).exists():paths.add(folder/name)
    for name in runs:
        obs=load_observations(folder/name,names)
        with np.load(folder/name/'commands.npz',allow_pickle=False) as data:commands={k:data[k].copy() for k in data.files}
        entry=dict(obs=obs,commands=commands);paths.update(folder/name/file for file in ('observations.npz','commands.npz','metadata.json','observation_schema.json'))
        for file,key in (('raw_telemetry.npz','raw'),('current_prediction.npz','prediction')):
            if (folder/name/file).exists():
                with np.load(folder/name/file,allow_pickle=False) as data:entry[key]={k:data[k].copy() for k in data.files}
                paths.add(folder/name/file)
        cache[name]=entry
    def digests():return {str(path.relative_to(folder)):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    before=digests();writer=None;errors=[];frame_count=0;manifest=[]
    if video:
        import imageio_ffmpeg
        writer=imageio_ffmpeg.write_frames(str(output/'dual_calibration_zh.mp4'),(1280,720),fps=fps,codec='libx264',pix_fmt_in='rgb24',pix_fmt_out='yuv420p',macro_block_size=16)
        writer.send(None)
    fonts={size:_font(size) for size in (12,14,16,18,20,26)}
    color=(66,220,192);muted=(154,174,194);gold=(240,198,102);text=(226,235,243)
    labels=['肩俯仰','肩滚转','肩偏航','肘俯仰','腕偏航','腕俯仰','腕滚转']
    stage_names=['方向／零位','驱动时序','重力保持','低速摩擦','动态激励','独立验证','逐臂验证','前馈对照']
    try:
        with JointView(adapter,output/'scene.xml',width=800,height=420) as view:
            positions=[]
            for stage in story['stages']:
                for clip in stage['segments']:
                    obs=cache[clip['run']]['obs'];mask=(obs['sample_ns']>=clip['start_s']*1e9)&(obs['sample_ns']<=clip['end_s']*1e9)
                    shown=obs['q'][mask]
                    for q in shown[::max(1,len(shown)//20)]:positions.extend(np.array(engine.fk(q))[:14,:3,3])
            positions=np.array(positions);view.camera.lookat[:]=(positions.min(axis=0)+positions.max(axis=0))/2
            view.camera.distance=max(1.05,float(np.linalg.norm(np.ptp(positions,axis=0)))*1.15)
            view.camera.azimuth=195;view.camera.elevation=-12
            original=view.model.geom_rgba.copy()
            for stage_index,stage in enumerate(story['stages']):
                segments=stage['segments'] if video else [stage['segments'][len(stage['segments'])//2]]
                snapshot_saved=False
                for clip in segments:
                    name=clip['run'];entry=cache[name];obs=entry['obs'];commands=entry['commands'];j=clip['joint']
                    azimuth=(245 if j<7 else 115) if stage['key']=='direction' and j%7 in (0,3,5) else 195
                    view.camera.azimuth=azimuth
                    start,end=clip['start_s'],clip['end_s'];seconds=clip_seconds or clip['seconds']
                    if start<obs['sample_ns'][0]*1e-9 or end>obs['sample_ns'][-1]*1e-9+1e-8:raise ValueError(f'Replay interval outside recording: {name}')
                    count=max(1,round(seconds*fps)) if video else 1;rate=(end-start)/(count/fps if video else seconds)
                    window=(obs['sample_ns']*1e-9>=start)&(obs['sample_ns']*1e-9<=end);times=obs['sample_ns'][window]*1e-9
                    requested=np.clip(np.searchsorted(commands['send_ns'],obs['sample_ns'],side='right')-1,0,len(commands['send_ns'])-1)
                    reference=commands['q_ref'][requested,j]
                    if stage['key']=='timing':
                        prediction=entry['prediction'];unit='电流 / A'
                        curves=[('指令换算',prediction['sent_target_current'][window,j],gold),('采集反馈',obs['current'][window,j],color),('拟合响应',prediction['predicted_current'][window,j],(240,110,140))]
                    elif stage['key']=='control':
                        previous=cache['dynamic_validation']['obs'];prior=np.interp(obs['sample_ns'],previous['sample_ns'],previous['q'][:,j])
                        unit='跟踪误差 / mrad';curves=[('原模型',1000*(prior[window]-reference[window]),(250,106,108)),('辨识后',1000*(obs['q'][window,j]-reference[window]),color)]
                    else:unit='关节角 / deg';curves=[('目标位置',np.degrees(reference[window]),gold),('实际采集',np.degrees(obs['q'][window,j]),color)]
                    for k in range(count):
                        progress=k/max(count-1,1) if video else .5;presentation_s=start+progress*(end-start)
                        receive=presentation_s*1e9+float(np.median(obs['receive_ns']-obs['sample_ns']))
                        index=int(np.clip(np.searchsorted(obs['receive_ns'],receive,side='right')-1,0,len(obs['sample_ns'])-1))
                        frame=StateFrame(adapter.robot_model_id,'recorded dual-arm mock',int(obs['sequence'][index]),int(obs['sample_ns'][index]),int(obs['receive_ns'][index]),tuple(names),
                            obs['q'][index],obs['dq'][index],obs['current'][index],None,status=int(np.bitwise_or.reduce(obs['status'][index])),clock_domain='recorded_synthetic_acquisition')
                        view.model.geom_rgba[:]=original
                        ball=view.mj.mj_name2id(view.model,view.mj.mjtObj.mjOBJ_GEOM,names[j]+'_ball')
                        view.model.geom_rgba[ball,:3]=np.array(gold)/255
                        rgb=view.render(frame);expected=engine.fk(frame.q)[:14];error=0.
                        for joint_index,joint in enumerate(engine.model.joints):
                            body=view.mj.mj_name2id(view.model,view.mj.mjtObj.mjOBJ_BODY,joint.name+'_view')
                            error=max(error,float(np.max(np.abs(view.data.xpos[body]-expected[joint_index][:3,3]))),float(np.max(np.abs(view.data.xmat[body].reshape(3,3)-expected[joint_index][:3,:3]))))
                        errors.append(error)
                        if np.any(view.data.qacc):raise RuntimeError('Viewer unexpectedly computed dynamics')
                        canvas=Image.new('RGB',(1280,720),(10,17,27));draw=ImageDraw.Draw(canvas)
                        draw.text((24,12),'CASBOT / 双臂动力学标定',font=fonts[26],fill=text)
                        for pill,label in enumerate(stage_names):
                            x=24+pill*155;draw.rounded_rectangle((x,52,x+145,78),5,fill=color if pill==stage_index else (17,28,42))
                            draw.text((x+8,57),f'{pill+1:02} {label}',font=fonts[12],fill=(10,17,27) if pill==stage_index else muted)
                        draw.text((24,88),stage['title'],font=fonts[18],fill=color)
                        canvas.paste(Image.fromarray(rgb),(24,120))
                        draw.rounded_rectangle((848,120,1256,658),10,fill=(17,28,42),outline=(40,58,76))
                        joint_label=('左' if j<7 else '右')+'臂 '+labels[j%7]
                        draw.text((868,137),joint_label+' / 当前高亮关节',font=fonts[18],fill=gold)
                        metrics=[('实际角度',f'{np.degrees(frame.q[j]):+.2f}°'),('实际速度',f'{frame.dq[j]:+.3f} rad/s')]
                        if stage['key']=='direction' and direction:
                            row=direction['axes'][j];raw=entry['raw']['raw_position'][index,j]
                            axis=model.joints[j].axis
                            metrics+=[('原始编码器（合成）',f'{raw:.1f} tick'),('方向 / 声明零位',f"{row['declared_direction']:+d} / {row['zero_raw']:.0f}"),
                                ('URDF 局部轴 / 正转',f'[{axis[0]:g}, {axis[1]:g}, {axis[2]:g}] / 右手定则'),
                                ('本轴实际正反转跨度',f"{np.degrees(row['observed_range_rad']):.1f}°"),('软件映射检查','通过' if row['passed'] else '失败'),('实机方向 / 机械零位','待独立确认')]
                        elif stage['key']=='timing' and timing:
                            row=timing['axes'][j];fit=row['fit']
                            maximum=max(value['rmse_a'] for axis in timing['axes'] for value in axis['validation'].values())
                            current_period=float(np.median(np.diff(commands['send_ns'])))*1e-6
                            metrics=[('当前命令周期',f'{current_period:.1f} ms'),('电流响应时间常数',f"{fit['current_tau_s']*1000:.3f} ms"),('指令延迟',f"{fit['command_delay_s']*1000:.3f} ms"),
                                ('反馈传输时间差',f"{timing['feedback_transport_delay_ms']:.3f} ms"),('训练 / 独立验证周期','1 / 2、4 ms'),('反馈采样周期',f"{timing['feedback_period_ms']:.1f} ms"),
                                ('冻结参数验证最大误差',f'{maximum*1000:.3f} mA'),('14轴时序检查','通过' if timing['passed'] else '失败')]
                        elif stage['key']=='control':
                            reduction=1-control['aggregate_identified_rmse_rad']/control['aggregate_nominal_rmse_rad']
                            metrics+=[('原模型汇总 RMSE',f"{control['aggregate_nominal_rmse_rad']*1000:.3f} mrad"),('辨识后汇总 RMSE',f"{control['aggregate_identified_rmse_rad']*1000:.3f} mrad"),('本次模拟误差下降',f'{reduction:.1%}'),('控制参数 / 限幅','两组保持一致')]
                        else:
                            command_index=requested[index];worst=max(value['maximum_normalized_rmse'] for value in report['validation'].values())
                            metrics+=[('参考加速度',f"{commands['ddq_ref'][command_index,j]:+.3f} rad/s²"),('本段实际角度跨度',f"{np.degrees(np.ptp(obs['q'][window,j])):.1f}°"),
                                ('独立预测最大 NRMSE',f'{worst:.2%}'),('受限参数 / 刚体回归秩',f"42 / {report['full_rigid_regressor']['rank']}")]
                        for row,(label,value) in enumerate(metrics):
                            y=188+row*52;draw.text((868,y),label,font=fonts[14],fill=muted);draw.text((868,y+20),value,font=fonts[16],fill=text)
                        _trace(draw,(24,548,824,658),times,curves,frame.sample_ns*1e-9,unit,fonts[12])
                        draw.text((24,671),f'原始记录 {rate:.2f} 倍速回放 | 采样时刻 {frame.sample_ns*1e-9:.3f} s | 角度未放大',font=fonts[14],fill=muted)
                        draw.text((24,696),'合成实验 / 腰部夹具仅示意 / 机械零位未标定 / MuJoCo 仅显示几何，不计算动力学',font=fonts[12],fill=gold)
                        if not snapshot_saved and (not video or progress>=.5):
                            canvas.save(output/f'{stage_index+1:02}_stage.png');snapshot_saved=True
                        if writer:writer.send(np.asarray(canvas))
                        frame_count+=1
                    manifest.append({**clip,'stage':stage['key'],'rendered_seconds':count/fps if video else 0.,'playback_rate':rate,'camera_azimuth':azimuth})
    finally:
        if writer:writer.close()
    if before!=digests():raise RuntimeError('Presentation modified source data')
    result=dict(passed=max(errors)<1e-12,frames=frame_count,max_fk_error=max(errors),source_sha256=before,
        mujoco_usage='mj_kinematics and mj_camlight only',truth_read=False,fixture_geometry='schematic only',
        angles_amplified=False,clips=manifest,video_duration_s=frame_count/fps if video else 0.)
    write_json(output/'presentation.json',result)
    if not result['passed']:raise RuntimeError('Dual viewer FK mismatch')
    return output


