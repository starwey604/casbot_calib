"""Local SHM remote tuning desk. Control and rendering never share a scheduler."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid

import numpy as np

from .shm_data import ROOT, NAMES, PARTS
from .shm_gui_client import RemoteClient
from .shm_gui_protocol import FIELDS
from .fieldwork_display import configure_graphics_environment
from .fieldwork_frames import FrameBuffer, WIDTH, HEIGHT

SETTINGS=dict(velocity_limit=1.5,effort_limit=10.,kp_max=100.,kd_max=10.,lease_ms=1000.,end_velocity=False)
PHASES={-1:'只读',0:'失能确认',1:'切换 MIT',2:'按实际起点使能',3:'控制中',4:'正在失能'}


def validate_fields(fields,hello,settings):
    result={k:np.asarray(fields[k],dtype=float) for k in FIELDS}
    for k,v in result.items():
        if v.shape!=(14,) or not np.isfinite(v).all():raise ValueError(k+'须为14个有限数值')
    bounds={'position':(hello['lower'],hello['upper']), 'velocity':(-np.minimum(hello['velocity'],settings['velocity_limit']),np.minimum(hello['velocity'],settings['velocity_limit'])),
            'effort':(-settings['effort_limit'],settings['effort_limit']),'kp':(0,settings['kp_max']),'kd':(0,settings['kd_max'])}
    for k,(lo,hi) in bounds.items():
        bad=np.flatnonzero((result[k]<lo)|(result[k]>hi))
        if len(bad):raise ValueError(f'ID{bad[0]+12} {k} 超出输入范围，目标未发送')
    return {k:v.tolist() for k,v in result.items()}


class Desk:
    active_render_fps=30
    def __init__(self,root,args,*,render=True):
        from .model_context import load_context
        self.root=Path(root);self.args=args;self.context=load_context()
        self.root.joinpath('view').mkdir(exist_ok=True)
        metadata=dict(model_context=self.context.identity,joint_names=NAMES)
        (self.root/'view/metadata.json').write_text(json.dumps(metadata))
        self.frames=None;self.renderer=None;self.render_channel=None;self.render_log=None
        if render:
            self.frames=FrameBuffer();parent,child=socket.socketpair();self.render_channel=parent;parent.setblocking(False)
            self.render_log=(self.root/'renderer.log').open('wb')
            self.renderer=subprocess.Popen([sys.executable,'-m','casbot_calib.shm_gui_render','--root',str(root),'--frame-fd',str(self.frames.fd),'--pose-fd',str(child.fileno())],pass_fds=(self.frames.fd,child.fileno()),stdout=self.render_log,stderr=self.render_log,env=dict(os.environ,MUJOCO_GL='egl'))
            child.close()
        self.client=None;self.q=None;self.feedback=None;self.commands=None;self.selected=0
        self.message='连接只读观测，点击开始控制后才发送五字段。';self.recording_error=None;self.render_error=None;self.save_message=''
        self.save_job=None;self.short=None;self.reopen_root=None;self.control_bounds={};self.frame=0;self.texture=None;self.texture_stamp=0
        self.view_visible=True;self.camera=[160.,-8.,2.3];self.last_pose=0;self.last_fields=0;self.dirty=False;self.await_start=False;self.previous_phase=-1
        self.settings=dict(SETTINGS);self.settings_draft=dict(SETTINGS);self.settings_open=False;self.settings_pending=None
        self.history=deque(maxlen=1800);self.history_seq=0;self.received_delay_ms=0;self.model_ok=False;self.last_ready=None
        if args.preview:
            self.q=np.array(self.context.home);self.message='离线模型预览，未连接机器人。'
    @property
    def pending_start(self):return self.await_start
    @property
    def phase(self):return self.feedback['phase'] if self.feedback else -1
    @property
    def connected(self):return bool(self.client and not self.client.failure and self.model_ok and self.feedback and self.feedback['receive_ns'])
    def connect(self):
        if self.args.preview:return
        if self.client:self.client.close()
        self.q=None;self.feedback=None;self.commands=None;self.model_ok=False;self.await_start=False;self.previous_phase=-1;self.history.clear();self.history_seq=0
        remote_root=self.args.remote_output or ('artifacts/shm_gui/'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
        config=dict(output=remote_root,mock=self.args.mock,endpoint=self.args.endpoint,budget_mib=self.args.cache_limit_mib)
        if not self.args.host:config.update(executor=str(self.args.executor),output=str((ROOT/remote_root).resolve()))
        if self.args.hand_background:config['hand_background']=self.args.hand_background
        if self.args.mock_config:config['mock_config']=self.args.mock_config
        self.client=RemoteClient(config,self.root,host=self.args.host,repo=self.args.remote_repo);self.client.connect();self.message='正在连接，只读等待实际反馈。'
    def start(self):
        if not self.connected or self.phase!=-1:return
        self.commands['position']=np.array(self.feedback['q']);self.commands['velocity']=np.zeros(14);self.commands['effort']=np.zeros(14)
        try:validate_fields(self.commands,self.client.hello,self.settings)
        except ValueError as e:self.message=str(e);return
        self.client.send('start',kp=self.commands['kp'].tolist(),kd=self.commands['kd'].tolist());self.await_start=True;self.dirty=False;self.message='正在按新读实际起点进入控制。'
    def stop(self):
        if self.client:self.client.send('stop',control=self.client.control)
        self.await_start=False;self.dirty=False;self.message='已请求失能，等待原生应答。'
    def pulse(self):
        if self.client and self.model_ok:self.client.tick()
    def poll(self):
        from OpenGL import GL
        if self.client:
            for item in self.client.drain():
                kind=item['type']
                if kind=='remote_ready':
                    self.model_ok=item['model_context']==self.context.identity and tuple(item['joint_names'])==tuple(NAMES)
                    if not self.model_ok:self.client.close();self.message='远端模型/关节顺序不一致，未开始控制。'
                    self.last_ready=item
                elif kind=='reply':
                    if not item['ok']:self.message='请求被拒绝：'+item['reason'];self.await_start=False
                    elif item['id']==self.settings_pending:self.settings=dict(self.settings_draft);self.settings_pending=None;self.message='限制设置已由原生确认。'
                elif kind=='status':
                    self.message=PHASES.get(item['phase'],'状态')+'：'+item['reason'];
                    if item['phase']==-1:self.await_start=False;self.dirty=False
                elif kind=='error':self.message='远端错误：'+item['reason']
            s=self.client.latest
            if self.model_ok and s and s['receive_ns']:
                self.feedback=s;self.q=np.array(s['q'])
                if self.commands is None:self.commands=dict(position=self.q.copy(),velocity=np.zeros(14),effort=np.zeros(14),kp=np.full(14,20.),kd=np.full(14,2.))
                if self.previous_phase!=3 and s['phase']==3:
                    # Initialize to native's second fresh position, never replay a stale draft.
                    self.commands={k:np.array(v) for k,v in s['command'].items()};self.await_start=False
                self.previous_phase=s['phase']
                if s['seq']!=self.history_seq:self.history.append((s['observation_ns'],s['clock_segment'],s['q'],s['dq'],s['effort']));self.history_seq=s['seq']
                if s['recording_stopped']:self.recording_error='Orin记录已停止；已写前缀保留，控制仍运行。'
                now=time.monotonic()
                if self.dirty and s['phase']==3 and now-self.last_fields>=.02:
                    try:self.client.send('fields',**validate_fields(self.commands,self.client.hello,self.settings));self.dirty=False;self.last_fields=now
                    except ValueError as e:self.message=str(e);self.dirty=False
            if self.client.failure:self.message=self.client.failure+'；失能状态未知，请查看远端记录。';self.await_start=False;self.dirty=False
        if self.renderer and self.renderer.poll() is not None:self.render_error='渲染器退出：'+str(self.root/'renderer.log')
        if self.render_channel and self.q is not None and time.monotonic()-self.last_pose>=1/30:
            payload=json.dumps(dict(q=self.q.tolist(),selected=self.selected,receive_ns=(self.feedback or {}).get('receive_ns',0),visible=self.view_visible,camera=self.camera)).encode()+b'\n'
            try:
                # Unix SOCK_STREAM partial writes are retained below; never publish a broken JSON line.
                pending=getattr(self,'pose_pending',b'')
                if pending:
                    n=self.render_channel.send(pending);self.pose_pending=pending[n:]
                else:
                    n=self.render_channel.send(payload);self.pose_pending=payload[n:]
            except BlockingIOError:pass
            except OSError:self.render_error='渲染通道关闭'
            self.last_pose=time.monotonic()
        pixels=self.frames.read(self.texture_stamp) if self.frames else None
        if pixels:
            seq,rgb=pixels
            if self.texture is None:
                self.texture=int(GL.glGenTextures(1));GL.glBindTexture(GL.GL_TEXTURE_2D,self.texture)
                GL.glTexParameteri(GL.GL_TEXTURE_2D,GL.GL_TEXTURE_MIN_FILTER,GL.GL_LINEAR);GL.glTexParameteri(GL.GL_TEXTURE_2D,GL.GL_TEXTURE_MAG_FILTER,GL.GL_LINEAR)
                GL.glTexImage2D(GL.GL_TEXTURE_2D,0,GL.GL_RGB,WIDTH,HEIGHT,0,GL.GL_RGB,GL.GL_UNSIGNED_BYTE,rgb)
            else:
                GL.glBindTexture(GL.GL_TEXTURE_2D,self.texture);GL.glTexSubImage2D(GL.GL_TEXTURE_2D,0,0,0,WIDTH,HEIGHT,GL.GL_RGB,GL.GL_UNSIGNED_BYTE,rgb)
            self.texture_stamp=seq
    def gui(self):
        from imgui_bundle import imgui
        self.frame+=1;imgui.push_id(str(self.root))
        def button(label,enabled=True):
            imgui.push_id(label.encode().hex());imgui.begin_disabled(not enabled);clicked=imgui.button(label);imgui.end_disabled();imgui.pop_id();return clicked
        if button('连接 / 重连只读',not self.args.preview and self.phase==-1):self.connect()
        imgui.same_line()
        if button('开始控制',self.connected and self.phase==-1 and not self.await_start):self.start()
        imgui.same_line()
        if button('停止并失能',self.phase>=0 or self.await_start):self.stop()
        imgui.same_line()
        if button('限制设置'):self.settings_open=not self.settings_open
        imgui.same_line();imgui.text('离线预览' if self.args.preview else PHASES.get(self.phase,'等待连接'))
        imgui.text_wrapped(self.message)
        if self.last_ready:imgui.text_disabled('Orin记录：'+self.last_ready['workspace'])
        if self.recording_error:imgui.text_colored(imgui.ImVec4(.75,.15,.12,1),self.recording_error)
        if self.render_error:imgui.text_wrapped(self.render_error)
        imgui.separator();width=imgui.get_content_region_avail().x;height=imgui.get_content_region_avail().y
        imgui.begin_child('axes',imgui.ImVec2(240,height),imgui.ChildFlags_.border)
        for i,name in enumerate(NAMES):
            if i in (0,7):imgui.separator_text('左臂' if i==0 else '右臂')
            label=f'ID{i+12} {PARTS[i%7]}'
            if self.q is not None:label+=f'  {self.q[i]:.4f}'
            imgui.push_id(i);clicked,_=imgui.selectable(label,self.selected==i);imgui.pop_id()
            if clicked:self.selected=i
        if self.feedback and imgui.collapsing_header('其它部位状态'):
            for j,s in enumerate(self.feedback['joints']):
                if not 12<=j<=25:imgui.text(f'ID{j}  0x{s[0]:04x}  mode {s[4]}  err {s[5]}')
        if self.feedback and imgui.collapsing_header('双手原始状态'):
            for h,hand in enumerate(self.feedback['hands']):
                imgui.text(f'手{h} status {hand[0]} err {hand[1]}')
                for j in range(6):imgui.text(f'{j}: q {hand[2+j*4]} v {hand[3+j*4]} I {hand[4+j*4]}')
        imgui.end_child();imgui.same_line()
        imgui.begin_child('view',imgui.ImVec2(max(300,width-600),height),imgui.ChildFlags_.border)
        imgui.text('实际反馈模型' if not self.args.preview else '模型预览（非实际反馈）');imgui.text_disabled('非双臂为模型固定姿态；左拖旋转，滚轮缩放')
        w=imgui.get_content_region_avail().x
        if self.texture:
            imgui.image(self.texture,imgui.ImVec2(w,w*.75));a=imgui.get_item_rect_min();b=imgui.get_item_rect_max();self.control_bounds['model_image']=(a.x,a.y,b.x,b.y)
            if imgui.is_item_hovered():
                io=imgui.get_io()
                if imgui.is_mouse_dragging(0):self.camera[0]-=io.mouse_delta.x*.4;self.camera[1]=min(80,max(-80,self.camera[1]+io.mouse_delta.y*.4))
                if io.mouse_wheel:self.camera[2]=min(8,max(.5,self.camera[2]*np.exp(-io.mouse_wheel*.1)))
        else:imgui.text('等待模型和实际反馈…')
        if self.history:
            imgui.text_disabled('显示抽样序列；非均匀时间，完整分析读取Orin记录')
            rows=list(self.history);segment=rows[-1][1];rows=[r for r in rows if r[1]==segment]
            for column,label in ((2,'实际位置 / rad'),(3,'实际速度 / rad/s'),(4,'估算输出 / Nm')):
                values=np.array([r[column][self.selected] for r in rows],dtype=np.float32)
                imgui.plot_lines(label,values,graph_size=imgui.ImVec2(-1,65))
        imgui.end_child();imgui.same_line()
        imgui.begin_child('fields',imgui.ImVec2(0,height),imgui.ChildFlags_.border)
        i=self.selected;imgui.text(f'ID{i+12} '+('左' if i<7 else '右')+PARTS[i%7]);imgui.text_wrapped(NAMES[i])
        imgui.text_disabled('Kp/Kd 无设备回读，初始配置 20/2；前馈初始 0')
        if self.commands is not None and self.client and self.client.hello:
            hello=self.client.hello
            specs=[('position','位置目标 / rad',hello['lower'][i],hello['upper'][i]),('velocity','速度目标 / rad/s',-self.settings['velocity_limit'],self.settings['velocity_limit']),('effort','前馈 / Nm',-self.settings['effort_limit'],self.settings['effort_limit']),('kp','Kp',0,self.settings['kp_max']),('kd','Kd',0,self.settings['kd_max'])]
            for key,label,lo,hi in specs:
                imgui.text(label);imgui.set_next_item_width(-1);imgui.begin_disabled(self.phase not in (-1,3) or not self.connected)
                changed,value=imgui.slider_float('##slider'+key,float(self.commands[key][i]),float(lo),float(hi),'%.4f')
                imgui.set_next_item_width(-1);typed,value=imgui.input_double('##exact'+key,value,0,0,'%.6f')
                imgui.end_disabled()
                if changed or typed:self.commands[key][i]=value;self.dirty=True
        if self.feedback:
            s=self.feedback;j=s['joints'][i+12];imgui.separator();imgui.text(f'实际 q {s["q"][i]:.6f} rad');imgui.text(f'实际 dq {s["dq"][i]:.5f} rad/s');imgui.text(f'估算 effort {s["effort"][i]:.2f} Nm');imgui.text(f'状态 0x{j[0]:04x}  mode {j[4]}  error {j[5]}')
            imgui.text_wrapped(f'入队版本 {s["revision"]}（不表示模组已采用）')
            if s['control']:
                for key in FIELDS:imgui.text(f'{key}: {s["command"][key][i]:.6g}')
            imgui.text_disabled('画面源：Orin 主站时间；不是驱动采样时刻')
        imgui.end_child()
        if self.settings_open:
            imgui.set_next_window_size(imgui.ImVec2(470,380),imgui.Cond_.first_use_ever)
            visible,self.settings_open=imgui.begin('限制设置',self.settings_open)
            if visible:
                imgui.text_wrapped('本轮先在只读状态应用；输入范围改变不移动目标。')
                for key,label in [('velocity_limit','速度观察/输入上限 rad/s'),('effort_limit','前馈输入绝对值 Nm'),('kp_max','Kp 输入上限'),('kd_max','Kd 输入上限'),('lease_ms','控制租约 ms')]:
                    _,self.settings_draft[key]=imgui.input_double(label,self.settings_draft[key],0,0,'%.3f')
                _,self.settings_draft['end_velocity']=imgui.checkbox('反馈速度越限结束（位置始终只告警）',self.settings_draft['end_velocity'])
                if button('应用并等待原生回执',self.connected and self.phase==-1):self.settings_pending=self.client.send('settings',**self.settings_draft)
            imgui.end()
        imgui.pop_id()
    def close(self):
        if self.client:self.client.close()
        if self.render_channel:self.render_channel.close()
        if self.renderer:
            try:self.renderer.wait(timeout=3)
            except subprocess.TimeoutExpired:self.renderer.terminate();self.renderer.wait(timeout=3)
        if self.frames:self.frames.close()
        if self.render_log:self.render_log.close()
        if self.texture:
            from OpenGL import GL
            GL.glDeleteTextures([self.texture])


def parser():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host');p.add_argument('--remote-repo',default='/home/nvidia/casbot_calib_shm')
    p.add_argument('--remote-output');p.add_argument('--output',type=Path);p.add_argument('--mock',action='store_true');p.add_argument('--mock-config')
    p.add_argument('--endpoint',default='/shm_mc');p.add_argument('--hand-background',help='远端本开机窗口手部背景 command.bin')
    p.add_argument('--executor',type=Path,default=ROOT/'install/shm-core/bin/shm_executor')
    p.add_argument('--cache-limit-mib',type=float,default=512);p.add_argument('--software-rendering',action='store_true');p.add_argument('--preview',action='store_true');p.add_argument('--frames',type=int);p.add_argument('--screenshot',type=Path)
    return p


def main(argv=None):
    args=parser().parse_args(argv)
    if args.frames is not None and args.frames<=0:raise ValueError('frames须为正数')
    if args.preview and (args.mock or args.host):raise ValueError('离线预览不连接/启动Mock')
    if args.software_rendering:os.environ['LIBGL_ALWAYS_SOFTWARE']='1'
    configure_graphics_environment()
    root=(args.output or Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state')))/'casbot-calib/shm-gui'/uuid.uuid4().hex).resolve();root.mkdir(parents=True,exist_ok=False);(root/'view').mkdir()
    print('本机GUI工作区：'+str(root),flush=True)
    from .fieldwork_window import run_window
    args.profile=None
    return run_window(args,root,None,None,desk_factory=lambda root:Desk(root,args)) or 0


if __name__=='__main__':raise SystemExit(main())
