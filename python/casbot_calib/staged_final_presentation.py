"""Chinese R4 playback of delivered samples; MuJoCo supplies geometry only."""
from pathlib import Path
import numpy as np
from .data import write_json
from .staged_protocol import _read,file_hash
from .staged_final import load_final_protocol
from .staged_collection import load_trial


def final_story(root,analysis):
    source=Path(_read(root/'development_reference.json')['source']);plan=_read(root/'motion_plan.json');rows=[]
    def add(stage,title,folder,name,seconds=3.,joint=3,label=None):
        meta,_=load_trial(folder,name);begin=.1;end=meta['duration_s']-.1
        if label:
            indices=[i for i,x in enumerate(meta['labels']) if x==label];begin=meta['knots_s'][indices[0]];end=meta['knots_s'][indices[-1]+1]
            if label=='periodic':begin+=12.;end=begin+6.
        if stage=='breakaway':
            k=meta['labels'].index('torque_ramp');event=_read(folder/'runs'/name/'guard_events.json')['events'][0]
            begin=meta['knots_s'][k]-.1;end=event['decision_ns']*1e-9+.25
        rows.append(dict(stage=stage,title=title,folder=str(folder),run=name,seconds=seconds,joint=joint,start_s=begin,end_s=end))
    for j in (3,10):add('direction','方向与软件零位（复用开发预检，机械零位待实测）',source,f'direction_{j:02}',1.5,j)
    add('timing','驱动时序与频率（复用开发预检，非实机网络）',source,'timing_frequency_250',2.,0)
    slots=plan['final_prior_trials']
    for stage,label in (('gravity','static_measure'),('friction','constant_velocity'),('breakaway','torque_ramp')):
        slot=next(s for s in slots if s['stage']==stage and (stage=='gravity' or s['joint']==3))
        add(stage,'冻结模型：新的'+{'gravity':'多姿态重力留出','friction':'匀速摩擦留出','breakaway':'重力补偿起动留出'}[stage],root,slot['trial_id'],3.,3,label)
    train=next(s for s in plan['trials'] if s['split']=='train' and s['stage']=='inertia')
    add('inertia','多构型训练记录：完整刚体模型与稳定性检查',root,train['trial_id'],4.,3,'periodic')
    tests=[s for s in plan['trials'] if s['split']=='test' and s['repeat_id']==0]
    for group in ('both','left','right'):
        slot=next(s for s in tests if s['stage']=='inertia' and s['active_arm_group']==group)
        add('validation',{'both':'双臂','left':'仅左臂','right':'仅右臂'}[group]+'：新构型、冻结候选正式测试',root,slot['trial_id'],3.,10 if group=='right' else 3,'periodic')
    slot=next(s for s in tests if s['stage']=='inertia_probe' and s['joint']==11)
    add('validation','右腕专项：单向加速度与独立力矩平衡',root,slot['trial_id'],3.,11,'probe_measure')
    both=next(s for s in tests if s['stage']=='inertia' and s['active_arm_group']=='both')
    add('attribution','A–D同带宽归因：偏置、摩擦与刚体更新分开',root,both['trial_id'],4.,3,'periodic')
    add('control','独立控制对照：原模型/辨识模型 × 原增益/优化增益',analysis/'control','control_identified_original',6.,3,'periodic')
    return rows


def render_final(folder,output=None,*,fps=12,video=True):
    from PIL import Image,ImageDraw
    from .presentation import _font
    from .viewer import JointView
    from .staged_presentation import write_scene
    from .dual_presentation import _trace
    from .telemetry import StateFrame
    from .collision import nominal_profile
    from .dual_model import nominal_dual_arm
    analysis=Path(folder).resolve();protocol=_read(analysis/'final_protocol.json');root=Path(protocol['parent'])
    load_final_protocol(root,analysis)
    accepted=_read(analysis/'acceptance/report.json');control=_read(analysis/'control/control_comparison.json')
    if not accepted['model_acceptance_passed']:raise ValueError('R4 playback requires an accepted frozen model')
    output=Path(output) if output else analysis/'presentation';output.mkdir(parents=True,exist_ok=False)
    (output/'renderer_source.py').write_bytes(Path(__file__).read_bytes())
    from .staged_final_validation import audit_final_control
    audit=audit_final_control(analysis,output/'control_audit.json')
    plot_final_diagnostics(accepted,output/'final_diagnostics.png')
    profile=nominal_profile(scene=_read(root/'session.json')['collision']['scene']);engine,_=nominal_dual_arm()
    adapter=write_scene(profile,output/'scene.xml');story=final_story(root,analysis);cache={};paths={analysis/'final_protocol.json',analysis/'acceptance/report.json',analysis/'acceptance/frequency_budget.json',analysis/'control/control_comparison.json',root/'candidate_freeze.json'}
    for row in story:
        folder=Path(row['folder']);key=(row['folder'],row['run'])
        if key in cache:continue
        meta,curve=load_trial(folder,row['run']);target=folder/'runs'/row['run'];record=_read(target/'metadata.json')
        if not record['passed'] or file_hash(target/'observations.npz')!=record['raw_sha256']:raise ValueError('Playback raw observations changed')
        with np.load(target/'observations.npz',allow_pickle=False) as data:obs={k:data[k] for k in data.files}
        cache[key]=(obs,curve)
        paths.update(target/name for name in ('metadata.json','observations.npz','commands.npz'))
        paths.add(folder/'plans'/row['run']/'motion.json')
    original=analysis/'control/runs/control_nominal_original/observations.npz';paths.add(original)
    with np.load(original,allow_pickle=False) as data:old_q=data['q'];old_t=data['sample_ns']
    source=Path(_read(root/'development_reference.json')['source']);timing=_read(source/'driver_timing.json');paths.add(source/'driver_timing.json')
    before={str(p):file_hash(p) for p in paths};fonts={s:_font(s) for s in (12,14,16,18,22,26)}
    tabs=['方向零位','驱动时序','静态重力','运动摩擦','起动摩擦','惯量稳定','正式测试','频段归因','控制对照']
    keys=['direction','timing','gravity','friction','breakaway','inertia','validation','attribution','control']
    teal=(62,218,189);gold=(245,194,91);muted=(151,172,192);text=(228,237,245);writer=None;errors=[];snapshots=set();frames=0
    if video:
        import imageio_ffmpeg
        writer=imageio_ffmpeg.write_frames(str(output/'staged_calibration_zh.mp4'),(1280,720),fps=fps,codec='libx264',pix_fmt_in='rgb24',pix_fmt_out='yuv420p',macro_block_size=16);writer.send(None)
    try:
        with JointView(adapter,output/'scene.xml',width=800,height=420) as view:
            view.camera.lookat[:]=[0.,0.,.18];view.camera.distance=1.35;view.camera.elevation=-12
            names=tuple(_read(root/'session.json')['joint_names'])
            for row in story:
                obs,curve=cache[(row['folder'],row['run'])];j=row['joint'];stage=row['stage'];active=keys.index(stage)
                start,end=row['start_s'],row['end_s'];mask=(obs['sample_ns']*1e-9>=start)&(obs['sample_ns']*1e-9<=end);time=obs['sample_ns'][mask]*1e-9
                target=curve(time)[:,j];unit='关节角度 / 度';traces=[('目标',np.degrees(target),gold),('采集',np.degrees(obs['q'][mask,j]),teal)]
                if stage=='timing':unit='采集电流 / 安';traces=[('实际采集',obs['current'][mask,j],teal)]
                if stage=='breakaway':unit='电流换算力矩 / 牛米';traces=[('实际采集',obs['current'][mask,j]*_read(root/'session.json')['config']['drive']['gain_nm_a'],teal)]
                if stage=='control':
                    unit='位置误差 / 毫弧度';previous=np.interp(obs['sample_ns'][mask],old_t,old_q[:,j])
                    traces=[('原模型、原增益',(previous-target)*1000,(249,112,119)),('辨识后、原增益',(obs['q'][mask,j]-target)*1000,teal)]
                count=max(1,round(row['seconds']*fps)) if video else 1
                for k in range(count):
                    at=start+(end-start)*(k/max(1,count-1) if video else .5);index=int(np.clip(np.searchsorted(obs['sample_ns'],round(at*1e9)),0,len(obs['sample_ns'])-1))
                    frame=StateFrame(adapter.robot_model_id,'recorded P0 mock',int(obs['sequence'][index]),int(obs['sample_ns'][index]),int(obs['receive_ns'][index]),names,obs['q'][index],obs['dq'][index],obs['current'][index],None,clock_domain='synthetic_acquisition')
                    view.camera.azimuth=230 if j<7 else 145;rgb=view.render(frame);expected=engine.fk(frame.q);error=0.
                    for n,joint in enumerate(engine.model.joints):
                        body=view.mj.mj_name2id(view.model,view.mj.mjtObj.mjOBJ_BODY,joint.name+'_view')
                        error=max(error,float(np.max(abs(view.data.xpos[body]-expected[n][:3,3]))),float(np.max(abs(view.data.xmat[body].reshape(3,3)-expected[n][:3,:3]))))
                    if np.any(view.data.qacc):raise RuntimeError('Viewer computed dynamics')
                    errors.append(error);canvas=Image.new('RGB',(1280,720),(10,17,27));draw=ImageDraw.Draw(canvas)
                    draw.text((24,12),'CASBOT / 双臂动力学 P0：冻结候选与独立验收',font=fonts[26],fill=text)
                    for n,label in enumerate(tabs):
                        x=24+n*138;draw.rounded_rectangle((x,52,x+128,78),5,fill=teal if n==active else (18,29,43));draw.text((x+7,57),f'{n+1:02} {label}',font=fonts[12],fill=(10,17,27) if n==active else muted)
                    draw.text((24,90),row['title'],font=fonts[18],fill=teal);canvas.paste(Image.fromarray(rgb),(24,120));draw.rounded_rectangle((848,120,1256,658),10,fill=(18,29,43))
                    draw.text((868,137),('左臂' if j<7 else '右臂')+f' 第{j%7+1}轴 / 原始观测',font=fonts[22],fill=gold)
                    benefit=accepted['frequency_attribution'];worst=max(r['maximum_normalized_rmse'] for r in accepted['prediction']['periodic'].values())
                    metrics=[('实际角度 / 实际速度',f'{np.degrees(frame.q[j]):+.3f} 度 / {frame.dq[j]:+.4f} 弧度/秒'),('冻结候选',f'ridge = {protocol["ridge"]:g}；未用 test 拟合')]
                    if stage=='direction':metrics += [('当前检查','复用开发方向/编码器映射'),('机械零位','尚需独立实测')]
                    elif stage=='timing':metrics += [('默认命令 / 控制 / 发布','500 / 500 / 500 赫兹'),('电流响应时间常数',f'{timing["axes"][0]["fit"]["current_tau_s"]*1000:.3f} 毫秒'),('物理网络 / 实机时序','待独立实测')]
                    elif stage=='gravity':metrics += [('新配对重力最大误差',f'{accepted["prior_holdout"]["gravity"]["paired_approaches"]["maximum_absolute_rmse_nm"]:.6f} 牛米'),('新独立观测','16组静态保持；未重新辨识')]
                    elif stage=='friction':metrics += [('新匀速摩擦最大轴误差',f'{max(accepted["prior_holdout"]["friction"]["axis_rmse_nm"]):.6f} 牛米'),('采样资格','4千赫兹；复用已有4/8千赫兹对照')]
                    elif stage=='breakaway':metrics += [('冻结起动阈值最大差',f'{accepted["prior_holdout"]["breakaway"]["maximum_frozen_prediction_error_nm"]:.6f} 牛米'),('448条完整起动专项','位移与持续速度确认，回溯起动区间')]
                    elif stage=='inertia':metrics += [('稳定性证据','复用 R2 三档路径、删组与先验检查'),('模型范围','同一140参数物理代表；非逐参数唯一')]
                    elif stage=='attribution':metrics += [('新 test 的 B/A 频段改善',f'{benefit["improvement_fraction"]["B"]:.2%}'),('新 test 的 D/A 频段改善',f'{benefit["improvement_fraction"]["D"]:.2%}'),('偏置与带外残差','单独报告，不用于选参')]
                    elif stage=='control':metrics += [('固定PD，前馈误差下降',f'{control["fixed_pd_feedforward_rmse_reduction"]:.2%}'),('五档PD比例选择',f'{control["selected_kp_factor"]:.2f}；使用独立调参轨迹'),('控制约束验收','通过' if control['passed'] else '未通过，保留结果')]
                    else:metrics += [('新 test 最大逐轴预测误差',f'{worst:.2%}；门槛5%'),('独立测试','两中心、双批次；双臂与逐臂'),('保持臂','静止力矩区间验收，不假定唯一摩擦力')]
                    for n,(label,value) in enumerate(metrics[:8]):
                        y=183+n*56;draw.text((868,y),label,font=fonts[14],fill=muted);draw.text((868,y+22),value,font=fonts[16],fill=text)
                    _trace(draw,(24,548,824,658),time,traces,frame.sample_ns*1e-9,unit,fonts[12])
                    draw.text((24,671),f'真实角度未放大 / 记录选段 {(end-start)/row["seconds"]:.2f} 倍速 / 采样时刻 {frame.sample_ns*1e-9:.3f} 秒',font=fonts[14],fill=muted)
                    draw.text((24,696),'合成验收 / nominal双臂、腰部固定 / MuJoCo仅显示几何 / 未连接实机 / 未验收全频带',font=fonts[12],fill=gold)
                    if stage not in snapshots and (not video or k>=count//2):canvas.save(output/f'{active+1:02}_{stage}.png');snapshots.add(stage)
                    if writer:writer.send(np.asarray(canvas))
                    frames+=1
    finally:
        if writer:writer.close()
    if before!={str(p):file_hash(p) for p in paths}:raise RuntimeError('R4 playback inputs changed')
    result=dict(passed=bool(audit['passed'] and errors and max(errors)<1e-12),control_audit=audit,frames=frames,duration_s=frames/fps if video else 0.,max_fk_error=max(errors),
        source_sha256=before,renderer_source_sha256=file_hash(__file__),clips=story,angles_amplified=False,truth_read=False,mujoco_usage='kinematics/camera/rendering only; no dynamics/integration/collision',
        scope='read-only delivered observations; reused preflight explicitly labelled, new tests and controller records shown')
    write_json(output/'presentation.json',result);return result


def plot_final_diagnostics(report,path):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.font_manager import FontProperties
    from .presentation import _font
    font=FontProperties(fname=_font(12).path);figure=Figure(figsize=(15,5));FigureCanvasAgg(figure);axes=figure.subplots(1,3)
    rows=report['prediction']['periodic'].values()
    maximum=[max(r['normalized_rmse'][j] for r in rows if r['normalized_rmse'][j] is not None)*100 for j in range(14)]
    axes[0].bar(np.arange(14),maximum);axes[0].axhline(5.,color='red',ls='--');axes[0].set_ylabel('最坏运动区间 NRMSE / %',fontproperties=font)
    axes[0].set_xlabel('左臂1–7 / 右臂1–7；保持轴另按静止区间验收',fontproperties=font)
    bands=report['frequency_attribution']['mean_qualified_band_rmse_nm']
    axes[1].bar(list(MODEL_LABELS),[bands[k]*1000 for k in MODEL_LABELS]);axes[1].set_ylabel('合格激励频段等权 RMSE / 毫牛米',fontproperties=font)
    axes[1].set_xlabel('A 前序 / B 刚体 / C 联合刚体 / D 完整候选',fontproperties=font)
    ramps=report['prior_holdout']['breakaway']['ramps']
    onset=[max(r['frozen_prediction_error_nm'] for r in ramps if r['joint']==j)*1000 for j in range(14)]
    axes[2].bar(np.arange(14),onset);axes[2].axhline(10.,color='red',ls='--');axes[2].set_ylabel('冻结起动阈值最坏差 / 毫牛米',fontproperties=font)
    axes[2].set_xlabel('14轴、方向/姿态/斜率/静置条件全部保留',fontproperties=font)
    for axis,title in zip(axes,('新构型、双批次正式预测','完整周期、同轨迹匹配频带归因','448条独立起动扫描')):
        axis.set_title(title,fontproperties=font);axis.grid(axis='y',alpha=.2)
    labels=[f'{side}{j}' for side in ('左','右') for j in range(1,8)]
    for axis in (axes[0],axes[2]):axis.set_xticks(np.arange(14),labels,fontproperties=font,fontsize=8)
    figure.suptitle('R4 合成正式test：冻结模型，无重新拟合、偏置修正或候选选择',fontproperties=font)
    figure.tight_layout();figure.savefig(path,dpi=140)


MODEL_LABELS=('A','B','C','D')
