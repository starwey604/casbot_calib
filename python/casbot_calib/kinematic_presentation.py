"""Chinese TriCal playback of delivered joint/sensor records; MuJoCo viewer only."""
from hashlib import sha256
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .data import write_json
from .kinematics import nominal_kinematics
from .kinematic_acceptance import load_protocol,file_hash
from .trical_fixture import fixture_profile
from .staged_presentation import write_scene
from .arm_viewer import _pose_attributes
from .telemetry import StateFrame,TelemetryAdapter


def scene(engine,config,path):
    profile=fixture_profile(engine,config);adapter=write_scene(profile,path)
    tree=ET.parse(path);world=tree.getroot().find('worldbody');model=engine.model
    tree.getroot().set('model',model.model_id+'_trical_view')
    adapter=TelemetryAdapter(model.model_id,list(adapter.mappings))
    for geom in world.findall('.//geom'):
        name=geom.get('name','')
        if name.startswith('ball_'):geom.set('rgba','.85 .7 .25 1' if '_stem' not in name else '.4 .46 .54 1')
        if 'trical_tcp' in name:geom.set('rgba','.92 .38 .35 1' if '_tip_' in name else '.25 .8 .76 .95')
    for tool in config['tools']:
        frame=next(f for f in model.frames if f.name==tool['name'])
        parent=world if frame.parent<0 else world.find(".//body[@name='"+model.joints[frame.parent].name+"_view']")
        ET.SubElement(parent,'site',name=tool['name'],size='.005',rgba='1 .3 .4 1',
            **_pose_attributes(dict(rotation=frame.placement.rotation,translation=frame.placement.translation)))
    ET.indent(tree.getroot(),space='  ');tree.write(path,encoding='unicode')
    return adapter


def load_playback(folder):
    root=Path(folder);manifest=json.loads((root/'manifest.json').read_text())
    required={'raw/joint_packets.npz','raw/indicator_packets.npz','motions.json','measurements.json','events.json'}
    if not required<=set(manifest['files']):raise ValueError('Playback manifest omits required sources')
    for name,expected in manifest['files'].items():
        p=Path(name)
        if p.is_absolute() or '..' in p.parts or 'evaluation' in p.parts:raise ValueError('Invalid playback source manifest')
        if file_hash(root/p)!=expected:raise ValueError('Playback raw digest mismatch')
    with np.load(root/'raw/joint_packets.npz',allow_pickle=False) as raw:joints={k:raw[k] for k in raw.files}
    with np.load(root/'raw/indicator_packets.npz',allow_pickle=False) as raw:indicators={k:raw[k] for k in raw.files}
    order=np.argsort(joints['receive_ns'],kind='stable');joints={k:v[order] for k,v in joints.items()}
    if not len(joints['sequence']):raise ValueError('No recorded joint feedback for playback')
    sequence=joints['sequence'];adopted=np.r_[True,sequence[1:]>np.maximum.accumulate(sequence[:-1])]
    joints={k:v[adopted] for k,v in joints.items()}
    return dict(root=root,joints=joints,indicators=indicators,
        motions=json.loads((root/'motions.json').read_text()),measurements=json.loads((root/'measurements.json').read_text()),
        events=json.loads((root/'events.json').read_text()))


def delivered_state(data,time_ns):
    joints=data['joints'];i=np.searchsorted(joints['receive_ns'],time_ns,side='right')-1
    if i<0:raise ValueError('No delivered joint observation at playback time')
    return {key:value[i] for key,value in joints.items()}


def delivered_indicators(data,time_ns,frame,ball):
    packets=data['indicators'];result=[]
    for channel in range(3):
        chosen=np.flatnonzero((packets['receive_ns']<=time_ns)&(packets['channel']==channel)&(packets['tool_frame']==frame)&(packets['target_id']==ball))
        if len(chosen):
            # Native simulator emits delivery order; do not show a future sample.
            i=chosen[np.argmax(packets['receive_ns'][chosen])];result.append({key:value[i] for key,value in packets.items()})
        else:result.append(None)
    return result


def storyboard(cache):
    result=[]
    def motion(source,side,label,stage,title,seconds):
        rows=[r for r in cache[source]['motions'] if r['label']==label and r['target']['frame'].startswith(side)]
        if label=='withdraw':
            corrections=[r for r in cache[source]['motions'] if r['label']=='correct' and r['target']['frame'].startswith(side)]
            if corrections:rows=[r for r in rows if r['start_ns']>=corrections[0]['end_ns']]
        if not rows:return
        row=rows[0]
        end=row['end_ns']
        if label=='correct':
            accepted=[m['accepted_ns'] for m in cache[source]['measurements'] if m['frame']==row['target']['frame'] and m['ball_id']==row['target']['ball_id'] and m['accepted_ns']>=end]
            if not accepted:raise ValueError('Correction playback has no subsequent accepted stable measurement')
            end=min(accepted)
        result.append(dict(source=source,stage=stage,title=title,seconds=seconds,start_ns=row['start_ns'],end_ns=end,**row['target']))
    motion('reference','left','approach',0,'六球基准与左臂接近',6.)
    motion('reference','left','insert',1,'左臂入量程与三表接触',4.)
    motion('reference','left','correct',2,'读表修正与稳定采样',5.)
    motion('reference','left','withdraw',2,'退出目标球与测点切换',3.)
    motion('reference','right','approach',0,'完整双臂模型／右臂接近',6.)
    motion('reference','right','correct',2,'右臂自动对中与稳定窗口',5.)
    for stage,title,seconds in [(3,'训练选参、零位层与几何层求解',5.),(4,'独立批次验证／参数保持冻结',5.)]:
        source='validation';m=cache[source]['measurements'][0]
        result.append(dict(source=source,stage=stage,title=title,seconds=seconds,start_ns=m['accepted_ns']-140_000_000,end_ns=m['accepted_ns'],frame=m['frame'],ball_id=m['ball_id']))
    motion('repeat_1','left','correct',5,'重新采集批次一／同一工装条件',4.)
    motion('repeat_2','right','correct',5,'重新采集批次二／参数与TCP重复性',4.)
    source='validation';m=cache[source]['measurements'][-1]
    result.append(dict(source=source,stage=6,title='合成验收结果与实机待办',seconds=6.,start_ns=m['accepted_ns']-140_000_000,end_ns=m['accepted_ns'],frame=m['frame'],ball_id=m['ball_id']))
    return result


def render_kinematic(folder,analysis,output,*,fps=12,video=True):
    from PIL import Image,ImageDraw
    from .presentation import _font
    from .viewer import JointView
    if type(fps) is not int or not 1<=fps<=60:raise ValueError('Playback fps must be an integer in [1,60]')
    root,protocol,reference=load_protocol(folder);analysis=Path(analysis);output=Path(output)
    accepted=json.loads((analysis/'report.json').read_text())
    if not accepted['passed'] or accepted['source_protocol_sha256']!=file_hash(root/'protocol.json'):raise ValueError('Accepted matching K4 report required')
    if output.exists():raise FileExistsError(output)
    config=json.loads((reference/'train/profile.json').read_text());engine,_=nominal_kinematics(config)
    paths={'reference':reference/'train','validation':reference/'validation','repeat_1':root/'batches/repeat_01','repeat_2':root/'batches/repeat_02'}
    cache={key:load_playback(path) for key,path in paths.items()};story=storyboard(cache)
    sealed={str(path/'manifest.json'):file_hash(path/'manifest.json') for path in paths.values()}
    sealed[str(analysis/'report.json')]=file_hash(analysis/'report.json');sealed[str(root/'protocol.json')]=file_hash(root/'protocol.json')
    output.mkdir(parents=True,exist_ok=False);adapter=scene(engine,config,output/'scene.xml')
    fonts={n:_font(n) for n in (12,14,16,18,22,26)};bg=(10,17,27);text=(226,235,243);muted=(147,168,189);teal=(62,218,189);gold=(245,194,91)
    tabs=['工装接近','三表接触','对中采样','分层辨识','独立验证','批次重复','验收边界']
    writer=None;frames=0;errors=[];snapshots=set();stats=accepted['repeatability'];fit=accepted['fits'][0]
    if video:
        import imageio_ffmpeg
        writer=imageio_ffmpeg.write_frames(str(output/'kinematic_workflow.mp4'),(1280,720),fps=fps,codec='libx264',pix_fmt_out='yuv420p',macro_block_size=1,output_params=['-crf','22','-preset','fast']);writer.send(None)
    status_names={0:'有效',1:'无接触',2:'超量程',3:'旧反馈',4:'失联'}
    try:
        with JointView(adapter,output/'scene.xml',width=808,height=424) as view:
            view.camera.lookat[:]=[.2,0,.18];view.camera.distance=1.55;view.camera.azimuth=190;view.camera.elevation=-12
            original_colors=view.model.geom_rgba.copy()
            for clip in story:
                view.model.geom_rgba[:]=original_colors
                ball_geom=view.mj.mj_name2id(view.model,view.mj.mjtObj.mjOBJ_GEOM,clip['ball_id']+'_mesh')
                if ball_geom>=0:view.model.geom_rgba[ball_geom]=[1.,.9,.25,1.]
                data=cache[clip['source']];count=max(1,round(clip['seconds']*fps)) if video else 1
                begin=max(clip['start_ns'],int(data['joints']['receive_ns'][0]))
                if begin>clip['end_ns']:raise ValueError('Playback segment contains no delivered observations')
                clip['leading_unobserved_time_omitted_ns']=begin-clip['start_ns']
                for k in range(count):
                    fraction=k/max(count-1,1) if video else .7
                    stamp=round(begin+(clip['end_ns']-begin)*fraction)
                    sample=delivered_state(data,stamp)
                    frame=StateFrame(adapter.robot_model_id,'recorded_trical_observations',int(sample['sequence']),int(sample['sample_ns']),int(sample['receive_ns']),
                        tuple(m.name for m in adapter.mappings),sample['q'],sample['dq'],sample['current'],None,
                        status=int(np.bitwise_or.reduce(sample['status'])),clock_domain='recorded_virtual_ns')
                    rgb=view.render(frame)
                    closeup=None
                    if clip['stage'] in (1,2):
                        look=view.camera.lookat.copy();distance=view.camera.distance;azimuth=view.camera.azimuth;elevation=view.camera.elevation
                        view.camera.lookat[:]=engine.frame_pose(frame.q,clip['frame'])[:3,3];view.camera.distance=.35
                        view.camera.azimuth=230 if clip['frame'].startswith('left') else 145;view.camera.elevation=-25
                        closeup=Image.fromarray(view.render(frame)).resize((280,147),Image.Resampling.LANCZOS)
                        view.camera.lookat[:]=look;view.camera.distance=distance;view.camera.azimuth=azimuth;view.camera.elevation=elevation
                    for name in ('left_trical_tcp','right_trical_tcp'):
                        index=view.mj.mj_name2id(view.model,view.mj.mjtObj.mjOBJ_SITE,name);expected=engine.frame_pose(frame.q,name)
                        errors.append(max(float(np.max(abs(view.data.site_xpos[index]-expected[:3,3]))),float(np.max(abs(view.data.site_xmat[index].reshape(3,3)-expected[:3,:3])))))
                    if np.any(view.data.qacc):raise RuntimeError('Viewer computed accelerations')
                    canvas=Image.new('RGB',(1280,720),bg);draw=ImageDraw.Draw(canvas)
                    draw.text((24,15),'CASBOT / 双臂千分计运动学标定',font=fonts[26],fill=text)
                    draw.text((24,55),clip['title'],font=fonts[18],fill=teal)
                    draw.text((24,79),'原始采集回放 / 右侧精度与重复性指标为离线结果',font=fonts[12],fill=muted)
                    for i,title in enumerate(tabs):
                        x=24+i*176;draw.rounded_rectangle((x,94,x+164,124),radius=5,fill=(27,87,81) if i==clip['stage'] else (25,37,52))
                        draw.text((x+10,99),title,font=fonts[14],fill=teal if i==clip['stage'] else muted)
                    canvas.paste(Image.fromarray(rgb),(20,141))
                    if closeup is not None:
                        canvas.paste(closeup,(30,407));draw.text((36,411),'摄像机近景／实际几何尺度',font=fonts[12],fill=gold)
                    draw.text((858,145),'已送达的三表观测 / µm',font=fonts[18],fill=text)
                    packets=delivered_indicators(data,stamp,clip['frame'],clip['ball_id'])
                    for channel,packet in enumerate(packets):
                        y=184+channel*61;draw.text((860,y),['X通道','Y通道','Z通道'][channel],font=fonts[14],fill=muted)
                        if packet is None:value='等待目标反馈'
                        elif not np.isfinite(packet['reading_m']):value='无读数 / '+status_names[int(packet['status'])]
                        else:value=f"{packet['reading_m']*1e6:+.2f}  / {status_names[int(packet['status'])]}"
                        draw.text((860,y+22),value,font=fonts[16],fill=text)
                    metrics=[('活动工具与基准球',('左臂' if clip['frame'].startswith('left') else '右臂')+' / '+clip['ball_id']),
                        ('命令 / 仪表采样',f"{1e9/config['control_period_ns']:g} Hz / 每表 {1e9/config['sensor']['period_ns']:g} Hz"),('冻结拟合参数 / 候选',f"{len(protocol['selected_parameters'])} / 60"),
                        ('位置约束 RMSE：名义／零位／几何',f"{fit['validation']['nominal']['position_rmse_m']*1e6:.2f} / {fit['validation']['zero_layer']['position_rmse_m']*1e6:.2f} / {fit['validation']['geometry_layer']['position_rmse_m']*1e6:.2f} µm"),
                        ('三批 TCP 预测标准差最大值',f"{stats['maximum_prediction_std_m']*1e6:.2f} µm"),('外部计量 / 间隔复标','未测量 / 未测量')]
                    if clip['stage']==3:
                        metrics[1]=('零位层 / 几何层拟合参数',f"14 / {len(protocol['selected_parameters'])}")
                        metrics[4]=('训练选参后缩放条件数',f"{fit['fit_condition']:.2f} / 200")
                    if clip['stage']==5:
                        metrics[1]=('独立训练批次 / 公共验证点','3 批 / 12 点')
                        metrics[2]=('选中参数最大标准差（无量纲）',f"{stats['maximum_selected_parameter_std_scaled']:.5f} / 0.15")
                    for i,(label,value) in enumerate(metrics):
                        y=383+i*48;draw.text((858,y),label,font=fonts[12],fill=muted);draw.text((858,y+18),value,font=fonts[16],fill=gold if i==5 else text)
                    qdeg=np.degrees(frame.q);draw.text((26,581),'左臂实际观测角度  '+ '  '.join(f'{v:+.1f}°' for v in qdeg[:7]),font=fonts[14],fill=teal)
                    draw.text((26,606),'右臂实际观测角度  '+ '  '.join(f'{v:+.1f}°' for v in qdeg[7:]),font=fonts[14],fill=(115,173,245))
                    speed=(clip['end_ns']-clip['start_ns'])*1e-9/clip['seconds']
                    draw.text((26,643),f"原始记录选段 / {speed:.2f} 倍速 / 记录时刻 {stamp*1e-9:.3f}s / 真实角度未放大",font=fonts[14],fill=muted)
                    draw.text((26,673),'合成测量闭环；微米差异仅以数字展示；不是实机绝对或姿态精度验收。',font=fonts[14],fill=gold)
                    draw.text((26,699),'MuJoCo 只显示几何 / Pinocchio 核对 TCP / 不推进物理、不发送控制 / 腰部夹具为代理',font=fonts[12],fill=muted)
                    if clip['stage'] not in snapshots and (not video or k>=count//2):canvas.save(output/f"{clip['stage']+1:02}_{clip['stage']}.png");snapshots.add(clip['stage'])
                    if writer:writer.send(np.asarray(canvas))
                    frames+=1
    finally:
        if writer:writer.close()
    # Check every sealed raw input again, not just the manifest file itself.
    for path in paths.values():
        manifest=json.loads((path/'manifest.json').read_text())
        if any(file_hash(path/name)!=expected for name,expected in manifest['files'].items()):raise RuntimeError('Playback modified raw inputs')
    if any(file_hash(path)!=expected for path,expected in sealed.items()):raise RuntimeError('Playback source changed')
    result=dict(passed=bool(errors and max(errors)<1e-12),frames=frames,duration_s=frames/fps if video else 0.,
        maximum_tcp_pose_error=max(errors),truth_read=False,angles_amplified=False,geometry_error_amplified=False,
        raw_sources_unchanged=True,
        displayed_geometry='frozen nominal acquisition model; calibrated accuracy shown numerically',source_sha256=sealed,clips=story,
        mujoco_usage='mj_kinematics / mj_camlight / rendering only; no dynamics, integration or collision')
    write_json(output/'presentation.json',result)
    if not result['passed']:raise RuntimeError('Viewer TCP disagrees with Pinocchio')
    return result


def plot_acceptance(analysis,path):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt,font_manager
    from .presentation import _font
    report=json.loads((Path(analysis)/'report.json').read_text());font=font_manager.FontProperties(fname=_font(12).path)
    fig,axes=plt.subplots(1,2,figsize=(12,4));x=np.arange(len(report['fits']))
    for i,(key,label) in enumerate([('nominal','名义模型'),('zero_layer','零位层'),('geometry_layer','几何层')]):
        axes[0].bar(x+(i-1)*.24,[r['validation'][key]['position_rmse_m']*1e6 for r in report['fits']],.24,label=label)
    axes[0].set_xticks(x,['基准批次','复测批次一','复测批次二'],fontproperties=font);axes[0].set_yscale('log')
    axes[0].set_ylabel('独立球心位置约束 RMSE（µm）',fontproperties=font);axes[0].legend(prop=font)
    values=np.array(report['repeatability']['prediction_std_m'])*1e6
    axes[1].plot(np.arange(len(values))+1,values,'o-',label='三批预测样本标准差')
    axes[1].set_xlabel('共同留出观测姿态编号',fontproperties=font);axes[1].set_ylabel('TCP 预测标准差（µm）',fontproperties=font);axes[1].legend(prop=font)
    for a in axes:a.grid(axis='y',alpha=.2)
    fig.suptitle('K4 合成重复性／同一工装与物理参数；未验收实机绝对精度及间隔漂移',fontproperties=font)
    fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)
