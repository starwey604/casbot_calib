"""Read-only recorded workflow presentation and portable H.264 export."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from .data import write_json, load_observations
from .telemetry import ReplaySource, TelemetryAdapter
from .presentation_text import LABELS, STORIES

BG=(10,17,27);PANEL=(17,28,42);EDGE=(40,58,76);TEXT=(226,235,243)
MUTED=(139,160,180);CYAN=(50,220,190);GOLD=(255,195,78);RED=(250,106,108)


def _font(size,bold=False):
    """Use an installed CJK font; fail clearly instead of emitting missing glyphs."""
    from PIL import ImageFont
    import os
    candidates=[os.environ.get("CASBOT_CJK_FONT", ""),
        "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf",
        "/System/Library/Fonts/PingFang.ttc"]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            # Noto CJK collection index 2 is Simplified Chinese.
            index=2 if "NotoSansCJK" in candidate and candidate.endswith(".ttc") else 0
            return ImageFont.truetype(candidate,size,index=index)
    raise RuntimeError("中文渲染需要 CJK 字体；安装 fonts-noto-cjk 或设置 CASBOT_CJK_FONT")


class DemoPresentation:
    def __init__(self,folder,width=1280,height=720):
        from .viewer import JointView
        self.folder=Path(folder);self.width=width;self.height=height
        self.report=json.loads((self.folder/"demo_report.json").read_text(encoding="utf-8"))
        self.clips=json.loads((self.folder/"storyboard.json").read_text(encoding="utf-8"))["clips"]
        for clip in self.clips:
            clip["title"],clip["note"]=STORIES[clip["key"]]
        self.adapter=TelemetryAdapter.load(self.folder/"mapping.json")
        self.sources={};self.observations={}
        for clip in self.clips:
            for name in (clip["run"],clip.get("before")):
                if name is not None and name not in self.sources:
                    run=self.folder/"records"/name
                    self.sources[name]=ReplaySource.load(run,self.adapter,source="CPP MOCK / RECORDED")
                    self.observations[name]=load_observations(run)
        self.view=JointView(self.adapter,width=800,height=340)
        self.fonts={size:_font(size) for size in (11,12,13,14,16,18,20,24,28)}
        self.bold={size:_font(size,True) for size in (11,12,13,14,16,18,20,24,28)}

    def _text(self,draw,xy,text,size=14,color=TEXT,bold=False):
        draw.text(xy,str(text),font=(self.bold if bold else self.fonts)[size],fill=color)

    def _wrap(self,draw,text,width,size=14):
        lines=[];line=""
        for word in text:
            candidate=line+word
            if draw.textlength(candidate,font=self.fonts[size])>width and line:
                lines.append(line);line=word
            else:line=candidate
        return lines+[line]

    def _metric(self,draw,y,label,value,color=TEXT):
        self._text(draw,(868,y),label,13,MUTED)
        length=draw.textlength(str(value),font=self.bold[18])
        self._text(draw,(1234-length,y-3),value,18,color,True)

    def _plot(self,draw,clip,cursor_s):
        bounds=(24,474,824,652);draw.rounded_rectangle(bounds,12,fill=PANEL,outline=EDGE)
        name=clip["run"];obs=self.observations[name]
        time=obs["sample_ns"]*1e-9;curves=[];title="输出侧角度";unit="deg"
        if clip["key"]=="timing":
            title="电流阶跃响应";unit="A"
            curves=[("采集值",obs["current"],CYAN)]
            # Focus on 120 ms of the actually recorded response.
            time_limit=.12
            take=time<=time_limit;time=time[take]
            curves=[(name,data[take],color) for name,data,color in curves]
            cursor_s=min(cursor_s,time_limit)
        elif clip.get("before"):
            from .excitation import multisine
            reference=multisine(time,frequencies=(.75,1.25,2.75),amplitudes=(.07,.04,.018),phases=(0.,0.,0.))[0]
            before=self.observations[clip["before"]]
            curves=[("原模型",(before["q"]-reference)*1000,RED),
                    ("辨识模型",(obs["q"]-reference)*1000,CYAN)]
            title="独立轨迹跟踪误差";unit="mrad"
        else:
            curves=[("采集值",np.degrees(obs["q"]),CYAN)]
            if clip["key"]=="direction":
                commands=self.sources[name].commands
                target=np.interp(time,commands["send_ns"]*1e-9,commands["commands"][:,1])
                curves.append(("目标值",np.degrees(target),GOLD))
        if clip.get("before"):
            take=time>=1.
            time=time[take];curves=[(name,data[take],color) for name,data,color in curves]
        self._text(draw,(42,486),title,12,MUTED,True)
        self._text(draw,(774,486),unit,12,MUTED)
        x0,y0,x1,y1=84,514,804,625
        maximum=max(float(np.max(np.abs(data))) for _,data,_ in curves)
        maximum=max(maximum*1.13,1e-3)
        def point(t,value):
            return (x0+(x1-x0)*(t-time[0])/max(time[-1]-time[0],1e-9),(y0+y1)/2-(y1-y0)*.45*value/maximum)
        for frac in (-1.,0.,1.):
            y=point(0,frac*maximum)[1]
            draw.line((x0,y,x1,y),fill=EDGE)
            self._text(draw,(30,y-7),f"{frac*maximum:+.2f}",11,MUTED)
        stride=max(1,len(time)//900)
        for index,(name,data,color) in enumerate(curves):
            points=[point(t,v) for t,v in zip(time[::stride],data[::stride],strict=True)]
            draw.line(points,fill=tuple(int(v*.42) for v in color),width=1)
            selected=time[::stride]<=cursor_s
            past=[p for p,take in zip(points,selected,strict=True) if take]
            if len(past)>1:draw.line(past,fill=color,width=2)
            self._text(draw,(280+index*210,486),name,12,color)
        x=point(np.clip(cursor_s,time[0],time[-1]),0)[0];draw.line((x,y0,x,y1),fill=GOLD,width=1)
        self._text(draw,(x0,632),f"{time[0]:g} s",11,MUTED)
        self._text(draw,(x1-54,632),f"{time[-1]:.2f} s",11,MUTED)

    def frame(self,index,progress):
        from PIL import Image,ImageDraw
        clip=self.clips[index];source=self.sources[clip["run"]]
        first=max(int(source.times[0]),round(clip.get("play_start_s",0)*1e9))
        last=min(int(source.times[-1]),round(clip.get("play_end_s",source.times[-1]*1e-9)*1e9))
        play_ns=int(first+np.clip(progress,0,1)*(last-first))
        frame=source.read(play_ns)
        previous=self.sources[clip["before"]].read(play_ns) if clip.get("before") else None
        rgb=self.view.render(frame,previous)
        canvas=Image.new("RGB",(1280,720),BG);draw=ImageDraw.Draw(canvas)
        self._text(draw,(24,18),"CASBOT / 动力学标定流程",24,TEXT,True)
        self._text(draw,(894,23),"合成实验 / 只读回放",13,CYAN,True)
        self._text(draw,(24,53),"独立采集记录 / 输出侧等效关节 / 仅作几何显示",13,MUTED)
        labels=LABELS
        for i,label in enumerate(labels):
            x=24+i*155
            draw.rounded_rectangle((x,83,x+145,109),5,fill=CYAN if i==index else PANEL)
            self._text(draw,(x+9,89),f"{i+1:02}  {label}",11,BG if i==index else MUTED,i==index)
        canvas.paste(Image.fromarray(rgb),(24,122))
        draw.rounded_rectangle((24,122,824,462),12,outline=EDGE,width=1)
        self._text(draw,(42,138),"关节 1 / 局部 +Z 轴 / 正转为逆时针",12,TEXT,True)
        self._text(draw,(42,432),"采集值",12,CYAN,True)
        if frame.target_q is not None:self._text(draw,(146,432),"目标值",12,GOLD,True)
        if previous is not None:self._text(draw,(226,432),"原模型",12,RED,True)
        self._text(draw,(392,432),"对照杆仅错开高度，角度未放大",11,MUTED)
        draw.rounded_rectangle((848,122,1256,652),12,fill=PANEL,outline=EDGE)
        self._text(draw,(868,140),f"{index+1:02} / {labels[index]}",12,CYAN,True)
        y=170
        for line in self._wrap(draw,clip["title"],356,20):
            self._text(draw,(868,y),line,20,TEXT,True);y+=28
        y+=12
        r=self.report
        if clip["key"]=="direction":
            metrics=[("原始编码器（合成）","不可用" if frame.raw_position is None else f"{frame.raw_position[0]:.2f}"),
                ("方向 / 零位","-1 / 8192"),("归一化角度",f"{np.degrees(frame.q[0]):+.3f} deg"),
                ("软件映射","通过"),("实机物理方向","待确认")]
        elif clip["key"]=="timing":
            metrics=[("电流响应时间常数",f"{1000*r['driver_response']['current_tau_s']:.3f} ms"),
                ("指令延迟",f"{1000*r['driver_response']['command_delay_s']:.3f} ms"),
                ("电流拟合误差",f"{1000*r['driver_response']['rmse_a']:.3f} mA"),
                ("位置 / 电流时序","分通道对齐")]
        elif clip["key"]=="scale":
            metrics=[("已知附加惯量",f"{r['drive_scale']['added_inertia']:.3f} kg m2"),("输出侧力矩增益",f"{r['drive_scale']['gain']:.4f} Nm/A"),
                ("绝对尺度锚点","已知负载"),("独立力传感器","非必要")]
        elif clip["key"] in ("identify","summary"):
            p=r["identified"]["parameters"]
            metrics=[("等效惯量",f"{p['inertia']:.5f} kg m2"),("黏性阻尼",f"{p['damping']:.5f}"),
                ("夹具物理刚度",f"{p['spring']:.5f} Nm/rad" if "spring" in p else "未辨识"),("库仑摩擦",f"{p['coulomb']:.5f} Nm"),
                ("回归矩阵秩",f"{r['identified']['rank']} / {r['identified']['columns']}")]
        elif clip["key"]=="validate":
            metrics=[("独立轨迹力矩残差",f"{1000*r['heldout']['torque_rmse']:.3f} mNm"),
                ("训练 / 验证","独立整批记录"),("验证期间参数","冻结"),
                ("辨识读取真值","否")]
            if "repeatability" in r:
                metrics.append(("含噪整批验证",f"{r['repeatability']['passed_count']} / {r['repeatability']['total']} 通过"))
            if "gravity" in r:
                metrics.append(("固定姿态重力残差",f"{1000*r['gravity']['heldout']['torque_rmse']:.3f} mNm"))
        else:
            suffix="baseline" if clip["key"]=="feedforward" else "selected"
            old=r["controls"][f"original_{suffix}"];new=r["controls"][f"identified_{suffix}"]
            metrics=[("原模型轨迹误差",f"{1000*old['rmse_rad']:.3f} mrad"),
                ("辨识后轨迹误差",f"{1000*new['rmse_rad']:.3f} mrad"),
                ("所用 Kp / Kd",f"{new['kp']:.1f} / {new['kd']:.2f}"),
                ("饱和样本比例",f"{100*new['saturation_fraction']:.2f}%"),
                ("控制对照","四组")]
        for label,value in metrics:
            self._metric(draw,y,label,value,GOLD if value=="待确认" else TEXT);y+=43
        draw.line((868,y+2,1236,y+2),fill=EDGE)
        y+=18
        for line in self._wrap(draw,clip["note"],356,12):
            self._text(draw,(868,y),line,12,MUTED);y+=18
        self._text(draw,(868,610),f"q {np.degrees(frame.q[0]):+.2f} deg   i {frame.current[0]:+.3f} A",12,CYAN)
        self._text(draw,(868,630),f"序号 {frame.sequence}   来源：C++ 模拟设备",11,MUTED)
        self._plot(draw,clip,play_ns*1e-9)
        virtual_s=(last-first)*1e-9
        speed=virtual_s/clip["seconds"]
        self._text(draw,(24,674),f"采集记录回放  {speed:.2f}x   |   采样时间 {frame.sample_ns*1e-9:.3f} s",12,MUTED)
        self._text(draw,(735,674),"仅作几何显示；不计算动力学，不下发指令。",12,CYAN)
        draw.rectangle((24,702,1256,706),fill=EDGE)
        draw.rectangle((24,702,24+1232*(index+progress)/len(self.clips),706),fill=CYAN)
        if (self.width,self.height)!=(1280,720):
            canvas=canvas.resize((self.width,self.height),Image.Resampling.LANCZOS)
        return np.asarray(canvas)

    def close(self):self.view.close()


def render_demo(folder,output=None,*,fps=24,width=1280,height=720,clip_seconds=None,video=True,progress=print):
    from PIL import Image
    folder=Path(folder);output=Path(output) if output is not None else folder/"presentation"
    if fps<=0 or width<320 or height<180 or width%2 or height%2:
        raise ValueError("Use positive FPS and even dimensions >= 320 x 180")
    if clip_seconds is not None and clip_seconds<=0:raise ValueError("Clip duration must be positive")
    expected=json.loads((folder/"evidence_sha256.json").read_text())
    def verify():
        for name,digest in expected.items():
            if hashlib.sha256((folder/name).read_bytes()).hexdigest()!=digest:
                raise ValueError(f"Raw evidence changed: {name}")
    verify();output.mkdir(parents=True,exist_ok=False)
    start=perf_counter();presentation=DemoPresentation(folder,width,height)
    if clip_seconds is not None:
        for clip in presentation.clips:clip["seconds"]=clip_seconds
    writer=None;frame_count=0
    try:
        if video:
            import imageio_ffmpeg
            writer=imageio_ffmpeg.write_frames(str(output/"calibration_workflow.mp4"),(width,height),fps=fps,
                codec="libx264",pix_fmt_out="yuv420p",quality=8,macro_block_size=2,ffmpeg_log_level="error")
            writer.send(None)
        for i,clip in enumerate(presentation.clips):
            progress(f"Rendering {i+1}/{len(presentation.clips)}: {clip['title']}")
            snapshot=presentation.frame(i,.6)
            Image.fromarray(snapshot).save(output/f"{i+1:02}_{clip['key']}.png")
            if writer is not None:
                count=max(1,round(clip["seconds"]*fps))
                for n in range(count):
                    writer.send(presentation.frame(i,n/max(count-1,1)));frame_count+=1
    finally:
        if writer is not None:writer.close()
        presentation.close()
    verify()
    write_json(output/"viewer_manifest.json",{"fps":fps,"width":width,"height":height,"frame_count":frame_count,
        "video_duration_s":frame_count/fps,"wall_s":perf_counter()-start,"raw_evidence_unchanged":True,
        "mujoco_usage":"mj_kinematics and mj_camlight only; no dynamics or integration",
        "language":"zh-CN",
        "data_source":"recorded observations and command logs; no evaluation truth read",
        "clips":presentation.clips})
    return output
