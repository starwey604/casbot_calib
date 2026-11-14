"""Local SHM remote tuning desk. Control and rendering never share a scheduler."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import threading
import uuid

import numpy as np

from .shm_data import ROOT, NAMES, PARTS
from .shm_gui_client import RemoteClient
from .shm_gui_protocol import FIELDS
from .fieldwork_display import configure_graphics_environment
from .fieldwork_frames import FrameBuffer, WIDTH, HEIGHT

SETTINGS=dict(velocity_limit=1.5,input_velocity_limit=1.5,effort_limit=10.,kp_max=100.,kd_max=10.,lease_ms=1000.,feedback_ms=2000.,send_ms=2000.,transition_ms=3000.,end_velocity=False,axes={})
PHASES={-1:'只读',0:'失能确认',1:'切换 MIT',2:'按实际起点使能',3:'控制中',4:'正在失能'}


def axis_policy(settings,hello,j):
    result={k:v for k,v in settings.items() if k!='axes'};result.update(settings.get('axes',{}).get(str(j+12),{}))
    result.setdefault('input_lower',hello['lower'][j]);result.setdefault('input_upper',hello['upper'][j])
    result.setdefault('observed_lower',hello['lower'][j]);result.setdefault('observed_upper',hello['upper'][j])
    return result


def validate_fields(fields,hello,settings):
    result={k:np.asarray(fields[k],dtype=float) for k in FIELDS}
    for k,v in result.items():
        if v.shape!=(14,) or not np.isfinite(v).all():raise ValueError(k+'须为14个有限数值')
    for j in range(14):
        policy=axis_policy(settings,hello,j)
        bounds={'position':(max(hello['lower'][j],policy['input_lower']),min(hello['upper'][j],policy['input_upper'])),
                'velocity':(-min(hello['velocity'][j],policy['input_velocity_limit']),min(hello['velocity'][j],policy['input_velocity_limit'])),
                'effort':(-policy['effort_limit'],policy['effort_limit']),'kp':(0,policy['kp_max']),'kd':(0,policy['kd_max'])}
        for k,(lo,hi) in bounds.items():
            if not lo<=result[k][j]<=hi:raise ValueError(f'ID{j+12} {k} 超出输入范围，目标未发送')
    return {k:v.tolist() for k,v in result.items()}


class Desk:
    active_render_fps=30
    def __init__(self,root,args,*,render=True):
        from .model_context import load_context
        self.root=Path(root);self.args=args;self.context=load_context();recovered_row=None
        if args.recover:
            from .shm_gui_storage import load_snapshot,last_feedback
            metadata=load_snapshot(args.recover)
            if metadata['model_context']!=self.context.identity:raise ValueError('恢复记录模型不匹配')
            recovered_row=last_feedback(Path(args.recover)/'native/feedback.csv')
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
        self.view_visible=True;self.camera=[160.,-8.,2.3];self.last_pose=0;self.last_fields=0;self.dirty=False;self.await_start=False;self.start_waiting=False;self.previous_phase=-1
        self.settings=copy.deepcopy(SETTINGS);self.settings_draft=copy.deepcopy(SETTINGS);self.input_settings=copy.deepcopy(SETTINGS);self.settings_version=0;self.input_version=0;self.override_axis=False;self.pending_settings=None;self.settings_open=False;self.settings_pending=None
        self.job_cancel=threading.Event();self.jobs=ThreadPoolExecutor(max_workers=2,thread_name_prefix='shm-gui-job');self.zero_job=None;self.zero_request=None;self.zero_control=None;self.zero_seen=False;self.operation_generation=0
        self.save_open=False;self.save_path='';self.last_saved=None;self.video_job=None;self.archive=bool(args.recover)
        self.byte_sample=(time.monotonic(),0);self.byte_rate=0.
        self.history=deque(maxlen=1800);self.history_seq=0;self.received_delay_ms=0;self.model_ok=False;self.last_ready=None
        if args.recover:
            self.q=np.array([float(recovered_row['q'+str(j)]) for j in range(14)])
            self.message='只读快照：'+str(args.recover)+'；不会连接或发送。';self.last_saved=Path(args.recover)
        if args.preview:
            self.q=np.array(self.context.home);self.message='离线模型预览，未连接机器人。'
    @property
    def pending_start(self):return self.await_start
    @property
    def phase(self):return self.feedback['phase'] if self.feedback else -1
    @property
    def connected(self):return bool(self.client and self.client.hello and not self.client.failure and self.model_ok and self.feedback and self.feedback['receive_ns'])
    def connect(self):
        if self.args.preview or self.archive:return
        if self.client:self.disconnect()
        self.q=None;self.feedback=None;self.commands=None;self.model_ok=False;self.await_start=False;self.start_waiting=False;self.previous_phase=-1;self.history.clear();self.history_seq=0
        remote_root=(self.args.remote_output+('_'+uuid.uuid4().hex[:8] if self.last_ready else '')) if self.args.remote_output else ('artifacts/shm_gui/'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
        config=dict(output=remote_root,mock=self.args.mock,endpoint=self.args.endpoint,budget_mib=self.args.cache_limit_mib)
        if not self.args.host:config.update(executor=str(self.args.executor),output=str((ROOT/remote_root).resolve()))
        if self.args.hand_background:config['hand_background']=self.args.hand_background
        if self.args.mock_config:config['mock_config']=self.args.mock_config
        self.client=RemoteClient(config,self.root,host=self.args.host,repo=self.args.remote_repo);self.client.connect();self.message='正在连接，只读等待实际反馈。'
    def disconnect(self):
        self.operation_generation+=1;self.start_waiting=False;self.await_start=False;self.zero_request=None;self.zero_control=None;self.dirty=False
        if self.client:
            self.client.close();messages=self.client.drain()
            ack=any(x.get('type')=='closed' and x.get('disable_ack') for x in messages)
            self.message='已断开；'+('已收到失能应答。' if ack else '退出结果见远端记录。')+'末次位置为历史反馈。'
        self.client=None;self.feedback=None;self.commands=None;self.model_ok=False
    def start(self):
        if not self.connected or self.phase!=-1 or self.settings_pending is not None or self.zero_job or self.zero_control:return
        if max(abs(v) for v in self.feedback['dq'])>=.02:
            self.start_waiting=True;self.await_start=True;self.message='等待实际速度低于 0.02rad/s；尚未请求使能，可停止取消。';return
        self.start_waiting=False
        self.commands['position']=np.array(self.feedback['q']);self.commands['velocity']=np.zeros(14);self.commands['effort']=np.zeros(14)
        try:validate_fields(self.commands,self.client.hello,self.settings)
        except ValueError as e:self.message=str(e);return
        self.input_settings=copy.deepcopy(self.settings);self.input_version=self.settings_version
        self.client.send('start',kp=self.commands['kp'].tolist(),kd=self.commands['kd'].tolist());self.await_start=True;self.dirty=False;self.message='正在按新读实际起点进入控制。'
    def stop(self):
        self.operation_generation+=1;self.zero_request=None;self.start_waiting=False
        if self.client:self.client.send('stop',control=self.client.control)
        self.await_start=False;self.dirty=False;self.message='已请求失能，等待原生应答。'
    def begin_save(self):
        if not self.client or not self.client.ready:return
        from .shm_gui_archive import save_workspace
        self.save_job=self.jobs.submit(save_workspace,self.client,self.save_path,cancel=self.job_cancel.is_set);self.save_message='正在封存远端前缀并下载；控制通道不传文件。'
    def return_zero(self):
        if self.connected and self.phase==-1 and not self.zero_job:
            self.zero_request=self.client.send('zero_get');self.message='正在读取已保存零位。'
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
                    if self.model_ok:self.pending_settings=copy.deepcopy(self.settings);self.settings_pending=self.client.send('settings',**self.pending_settings)
                elif kind=='zero':
                    if item['id']==self.zero_request:
                        from .shm_gui_zero import validate_zero,plan_zero
                        try:
                            target=validate_zero(item['value'],self.context.identity,self.client.ready['mock'])
                            if not self.feedback or max(abs(x) for x in self.feedback['dq'])>=.02:raise ValueError('回零起点尚未停稳（|dq|<0.02rad/s），本次未规划')
                            self.zero_job=self.jobs.submit(plan_zero,list(self.feedback['q']),target,self.commands['kp'].tolist(),self.commands['kd'].tolist());self.zero_job.generation=self.operation_generation;self.message='正在检查回零 C3 连续参考与碰撞。'
                        except ValueError as error:self.message=str(error)
                        self.zero_request=None
                    else:self.message='已采集实际位置为零位：'+item['path']
                elif kind=='reply':
                    if not item['ok']:
                        self.message='请求被拒绝：'+item['reason'];self.await_start=False
                        if item['id']==self.settings_pending:self.settings_pending=None;self.pending_settings=None
                        if item['id']==self.zero_control:self.zero_control=None
                    elif item['id']==self.settings_pending:
                        self.settings=copy.deepcopy(self.pending_settings);self.settings_version=item['id'];self.settings_pending=None;self.pending_settings=None
                        (self.root/'settings.json').write_text(json.dumps(dict(version=self.settings_version,settings=self.settings),indent=2)+'\n');self.message='观察设置已由原生确认；输入/通信设置在下次控制生效。'
                elif kind=='status':
                    self.message=PHASES.get(item['phase'],'状态')+'：'+item['reason'];
                    if item['phase']==-1:
                        self.await_start=False;self.dirty=False
                        if item['control']==self.zero_control:self.zero_control=None
                elif kind=='error':self.message='远端错误：'+item['reason']
            s=self.client.latest
            if self.model_ok and s and s['receive_ns']:
                self.feedback=s;self.q=np.array(s['q'])
                if self.start_waiting and max(abs(v) for v in s['dq'])<.02:self.start()
                if self.zero_control==s['control'] and s['phase']>=0:self.zero_seen=True
                if self.zero_control==s['control'] and self.zero_seen and s['phase']==-1:self.zero_control=None;self.zero_seen=False
                if self.commands is None:self.commands=dict(position=self.q.copy(),velocity=np.zeros(14),effort=np.zeros(14),kp=np.full(14,20.),kd=np.full(14,2.))
                if self.previous_phase!=3 and s['phase']==3:
                    # Initialize to native's second fresh position, never replay a stale draft.
                    self.commands={k:np.array(v) for k,v in s['command'].items()};self.await_start=False
                self.previous_phase=s['phase']
                if s['seq']!=self.history_seq:self.history.append((s['observation_ns'],s['clock_segment'],s['q'],s['dq'],s['effort']));self.history_seq=s['seq']
                if s['recording_stopped']:self.recording_error='Orin记录已停止；已写前缀保留，控制仍运行。'
                now=time.monotonic()
                if self.dirty and not self.zero_control and s['phase']==3 and now-self.last_fields>=.02:
                    try:self.client.send('fields',**validate_fields(self.commands,self.client.hello,self.input_settings));self.dirty=False;self.last_fields=now
                    except ValueError as e:self.message=str(e);self.dirty=False
            if self.client.failure:self.message=self.client.failure+'；失能状态未知，请查看远端记录。';self.await_start=False;self.start_waiting=False;self.dirty=False
        if self.zero_job and self.zero_job.done():
            job=self.zero_job;self.zero_job=None
            try:
                result=job.result()
                if job.generation==self.operation_generation and self.connected and self.phase==-1:
                    self.zero_control=self.client.send('zero_return',**result);self.zero_seen=False;self.await_start=True;self.message='回零参考已提交，等待原生起点复核。'
                else:self.message='回零规划已取消，本次未发送。'
            except Exception as error:self.message='回零未执行：'+str(error)
        if self.save_job and self.save_job.done():
            job=self.save_job;self.save_job=None
            try:self.last_saved=Path(job.result()['output']);self.save_message='已保存：'+str(self.last_saved);self.save_open=False
            except Exception as error:self.save_message='保存失败：'+str(error)
        if self.video_job and self.video_job.poll() is not None:
            self.save_message=('视频已导出' if self.video_job.returncode==0 else '视频导出失败，见 video_export.log');self.video_job=None
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
        if button('连接 / 重连只读',not self.args.preview and not self.archive and self.phase==-1):self.connect()
        imgui.same_line()
        if button('开始控制',self.connected and self.phase==-1 and not self.await_start and self.settings_pending is None and not self.zero_job and not self.zero_control):self.start()
        imgui.same_line()
        if button('停止并失能',self.phase>=0 or self.await_start or self.zero_job is not None):self.stop()
        imgui.same_line()
        if button('断开',self.client is not None):self.disconnect()
        imgui.same_line()
        if button('限制设置'):self.settings_open=not self.settings_open
        imgui.same_line()
        if button('保存会话',bool(self.client and self.client.ready) and self.save_job is None):
            self.save_open=not self.save_open;self.save_path=self.save_path or str(self.root.parent/('saved_'+self.root.name))
        imgui.same_line()
        if button('采集零位',self.connected):self.client.send('zero_capture')
        imgui.same_line()
        if button('回到保存零位',self.connected and self.phase==-1 and not self.zero_job and not self.zero_control):self.return_zero()
        imgui.same_line();imgui.text('只读快照' if self.archive else '离线预览'  if self.args.preview else PHASES.get(self.phase,'等待连接'))
        if self.save_open:
            imgui.set_next_item_width(600);_,self.save_path=imgui.input_text('保存到本机新目录',self.save_path)
            imgui.same_line()
            if button('封存并下载',bool(self.save_path) and self.save_job is None):self.begin_save()
        if self.last_saved and button('导出视频',self.video_job is None):
            log=(self.root/'video_export.log').open('wb')
            self.video_job=subprocess.Popen([sys.executable,'-m','casbot_calib.shm_gui_archive','video','--source',str(self.last_saved),'--output',str(self.last_saved.parent/(self.last_saved.name+'_'+str(time.monotonic_ns())+'.mp4'))],stdout=log,stderr=log,env=dict(os.environ,MUJOCO_GL='egl'));log.close()
        if self.video_job:
            imgui.same_line()
            if button('取消视频导出'):self.video_job.terminate()
        if self.save_message:imgui.text_wrapped(self.save_message)
        imgui.text_wrapped(self.message)
        if self.last_ready:imgui.text_disabled('Orin记录：'+self.last_ready['workspace'])
        if self.feedback and self.feedback.get('recording_bytes') is not None:
            used=self.feedback['recording_bytes'];limit=self.feedback['budget_bytes'];now=time.monotonic()
            if now-self.byte_sample[0]>=1:
                self.byte_rate=max(0,(used-self.byte_sample[1])/(now-self.byte_sample[0]));self.byte_sample=(now,used)
            remaining=f'，预计剩余 {max(0,limit-used)/self.byte_rate:.0f}s' if self.byte_rate>0 and not self.feedback['recording_stopped'] else ''
            imgui.text_disabled(f'原生记录 {used/1048576:.1f} / {limit/1048576:.0f}MiB，{self.byte_rate/1048576:.2f}MiB/s'+remaining)
        if self.feedback and self.client:
            age=max(0,(self.feedback['mono_ns']-self.feedback['receive_ns'])*1e-6);gap=max(0,(time.monotonic_ns()-self.client.received_at)*1e-6)
            latency=f'，RPC往返 {self.client.rtt_ms:.1f}ms' if self.client.rtt_ms is not None else ''
            imgui.text_disabled(f'主站出队距遥测发送 {age:.1f}ms，本机距收包 {gap:.1f}ms'+latency)
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
        imgui.text('记录末次反馈' if self.archive else '历史反馈模型（未连接）' if self.q is not None and self.client is None and not self.args.preview else '实际反馈模型' if not self.args.preview else '模型预览（非实际反馈）');imgui.text_disabled('非双臂为模型固定姿态；左拖旋转，滚轮缩放')
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
            hello=self.client.hello;limits=axis_policy(self.input_settings if self.phase>=0 else self.settings,hello,i)
            specs=[('position','位置目标 / rad',limits['input_lower'],limits['input_upper']),('velocity','速度目标 / rad/s',-limits['input_velocity_limit'],limits['input_velocity_limit']),('effort','前馈 / Nm',-limits['effort_limit'],limits['effort_limit']),('kp','Kp',0,limits['kp_max']),('kd','Kd',0,limits['kd_max'])]
            for key,label,lo,hi in specs:
                imgui.text(label);imgui.set_next_item_width(-1);imgui.begin_disabled(self.phase not in (-1,3) or not self.connected or bool(self.zero_control))
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
                imgui.text_wrapped('观察范围在线生效；目标输入/通信设置在下次控制生效。')
                imgui.text(f'观察确认版本 {self.settings_version}；本次输入版本 {self.input_version}')
                _,self.override_axis=imgui.checkbox('编辑当前轴覆盖（否则编辑全轴默认）',self.override_axis)
                draft=self.settings_draft
                if self.override_axis:
                    overrides=draft.setdefault('axes',{});key=str(self.selected+12)
                    if key not in overrides:overrides[key]={}
                    draft=overrides[key]
                    imgui.text('ID'+key+' 未覆盖字段继承全轴默认')
                    if button('当前轴全部恢复继承'):overrides.pop(key,None);draft={}
                specs=[('velocity_limit','反馈速度上限 rad/s'),('input_velocity_limit','目标速度输入上限 rad/s'),('effort_limit','前馈输入绝对值 Nm'),('kp_max','Kp 输入上限'),('kd_max','Kd 输入上限')]
                if self.override_axis and self.client and self.client.hello:
                    specs+=[('input_lower','目标位置下限 rad'),('input_upper','目标位置上限 rad'),('observed_lower','观察位置下限 rad'),('observed_upper','观察位置上限 rad')]
                defaults=axis_policy(self.settings_draft,self.client.hello,self.selected) if self.client and self.client.hello else self.settings_draft
                for key,label in specs:
                    changed,value=imgui.input_double(label+'##'+key,float(draft.get(key,defaults.get(key,0.))),0,0,'%.4f')
                    if changed:draft[key]=value
                changed,value=imgui.checkbox('速度越限结束（位置始终只告警）',draft.get('end_velocity',self.settings_draft['end_velocity']))
                if changed:draft['end_velocity']=value
                if not self.override_axis:
                    for key,label in [('lease_ms','控制租约 ms'),('feedback_ms','反馈断流超时 ms'),('send_ms','持续发送失败超时 ms'),('transition_ms','模式切换超时 ms')]:
                        _,draft[key]=imgui.input_double(label,draft[key],0,0,'%.1f')
                if button('应用并等待原生回执',self.connected and self.settings_pending is None):
                    self.pending_settings=copy.deepcopy(self.settings_draft);self.settings_pending=self.client.send('settings',**self.pending_settings)
            imgui.end()
        imgui.pop_id()
    def close(self):
        if self.client and self.client.process and self.client.process.poll() is None:self.client.send('quit')
        self.operation_generation+=1
        if self.video_job and self.video_job.poll() is None:self.video_job.terminate();self.video_job.wait(timeout=3)
        self.job_cancel.set();self.jobs.shutdown(wait=True,cancel_futures=True)
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
    p=argparse.ArgumentParser(description='本机SHM调参台：本地绘图，SSH连接Orin或私有Mock；默认只读。');p.add_argument('--host');p.add_argument('--remote-repo',default='/home/nvidia/casbot_calib_shm')
    p.add_argument('--recover',type=Path,help='只读打开已保存快照，不连接/发送');p.add_argument('--remote-output');p.add_argument('--output',type=Path);p.add_argument('--mock',action='store_true');p.add_argument('--mock-config')
    p.add_argument('--endpoint',default='/shm_mc');p.add_argument('--hand-background',help='远端本开机窗口手部背景 command.bin')
    p.add_argument('--executor',type=Path,default=ROOT/'install/shm-core/bin/shm_executor')
    p.add_argument('--cache-limit-mib',type=float,default=512);p.add_argument('--software-rendering',action='store_true');p.add_argument('--preview',action='store_true');p.add_argument('--frames',type=int);p.add_argument('--screenshot',type=Path)
    return p


def main(argv=None):
    args=parser().parse_args(argv)
    if not np.isfinite(args.cache_limit_mib) or args.cache_limit_mib<=0:raise ValueError('记录额度须为正有限值')
    if args.screenshot and args.screenshot.exists():raise ValueError('截图目标已存在')
    if args.frames is not None and args.frames<=0:raise ValueError('frames须为正数')
    if args.recover and (args.preview or args.mock or args.host):raise ValueError('恢复只读，不同时连接实机/Mock')
    if args.preview and (args.mock or args.host):raise ValueError('离线预览不连接/启动Mock')
    if args.software_rendering:os.environ['LIBGL_ALWAYS_SOFTWARE']='1'
    configure_graphics_environment()
    root=(args.output or Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state')))/'casbot-calib/shm-gui'/uuid.uuid4().hex).resolve();root.mkdir(parents=True,exist_ok=False);(root/'view').mkdir()
    print('本机GUI工作区：'+str(root),flush=True)
    from .fieldwork_window import run_window
    args.profile=None
    return run_window(args,root,None,None,desk_factory=lambda root:Desk(root,args)) or 0


if __name__=='__main__':raise SystemExit(main())
