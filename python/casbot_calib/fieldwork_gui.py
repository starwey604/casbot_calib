"""Direct tuning desk with persistent observation and explicit native control status."""
import csv
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

import numpy as np
import yaml

from .fieldwork import clock_id
from .fieldwork_analysis import read_json, write_json
from .fieldwork_control import Session, Workspace, initial_commands, tuning_config, validate_commands, write_command, short_position
from .fieldwork_ipc import Child
from .fieldwork_display import FramePacer, configure_graphics_environment, latest_json, VIEW_FPS
from .fieldwork_status import JOINT_LABELS, OperatorStatus, describe
from .hardware_batch import atomic_json


def claim_frontend(root):
    import fcntl
    lease=(Path(root)/'.frontend.lock').open('a+')
    try:fcntl.flock(lease.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        lease.close();raise ValueError('同一工作区已有调参窗口') from None
    return lease


def last_csv(path):
    with Path(path).open('rb') as stream:
        header=stream.readline().decode().strip().split(',')
        stream.seek(0,2);size=stream.tell();stream.seek(max(0,size-16384))
        lines=stream.read().decode().splitlines(keepends=True)
    for line in reversed(lines):
        if not line.endswith('\n'):continue
        row=next(csv.reader([line]))
        if len(row)==len(header) and row!=header:return dict(zip(header,row))
    return None


class Desk:
    def __init__(self, root, *, preview=False, profile='configs/hardware_field.yaml'):
        from .hardware_field import load_profile, model_for
        self.root=Path(root).resolve();self.preview=preview;self.archive=False
        self.snapshot=(self.root/'saved_session.json').exists() or (self.root/'saving.json').exists()
        self.frontend_lease=None if preview else claim_frontend(root)
        if preview:
            config=load_profile(profile);engine,_=model_for(config)
            self.status=dict(joint_names=[j.name for j in engine.model.joints],model_context=config['model_context'],
                             domain_id=None,scope='offline_preview',runs=[],controls=[])
        else:
            metadata=read_json(self.root/'protocol/session.json')
            self.archive=metadata.get('schema_version')=='casbot_fieldwork_manual_v2'
            session=Session(root) if self.archive else Workspace(root)
            self.status=session.status();config=session.config;engine,_=model_for(config)
        self.read_only=self.archive or self.snapshot
        from .fieldwork_storage import recording_pause_file,cached_sessions
        self.pause_file=recording_pause_file(self.root);self.cache_monitor=None;self.owned_mock=None
        self.save_job=None;self.save_open=False;self.save_path='';self.save_message='';self.reopen_root=None
        self.recovery_rows=cached_sessions();self.connection_channels={};self.connected_at=None
        self.engine=engine;self.config=config;self.tuning=tuning_config(config);self.names=self.status['joint_names']
        self.lower=np.array([j.lower for j in engine.model.joints]);self.upper=np.array([j.upper for j in engine.model.joints])
        self.axes=[np.asarray(j.axis).tolist() for j in engine.model.joints]
        self.owner_id=uuid.uuid4().hex;self.local_clock=clock_id();self.heartbeat=self.root/'frontend_heartbeat.json'
        self.observer=None;self.task=None;self.renderer=None;self.pending=None;self.response=None
        self.observer_response=None
        self.observer_cancel=None;self.control_cancel=None;self.closing=False;self.render_error=None
        self.selected=0;self.frame=0;self.controls={};self.control_bounds={}
        self.has_feedback_pose=False
        self.texture=None;self.texture_stamp=0;self.rendered_frames=0
        self.q=self.first_q=None;self.receive_ns=None;self.commands=None;self.sources={};self.dirty=set()
        self.feedback=dict(q=None,dq=None,effort_raw=None,kp=None,kd=None)
        self.mode='未连接';self.message='连接只读观测；点击开始后才申请控制并发送五字段。'
        self.recording_error=None;self.operator=OperatorStatus();self.sending=False;self.start_requested=False
        self.needs_reinitialize=False;self.generation=0;self.active_run=None;self.control_root=self.root
        self.control_roots=[];self.last_recording=None
        self.short=None;self.short_amplitude=.25;self.short_duration=6.
        self.view_last=0;self.last_render_pose=None;self.heartbeat_archived_at=None
        view=self.root/'view';view.mkdir(exist_ok=True)
        atomic_json(view/'metadata.json',self.status,replace=True)
        nominal=yaml.safe_load(Path('configs/fieldwork_mock.yaml').read_text())['initial_q']
        latest_json(view/'pose.json',dict(q=nominal,selected=0,receive_ns=0,source='nominal_not_feedback'))
        if self.archive:self.load_archive()
        elif not preview:
            for relative in reversed(self.status.get('controls',[])):
                control=self.root/relative;state=Session(control).status()
                for run in reversed(state['runs']):
                    if (control/run/'recording/feedback.csv').exists() or (control/run/'capture/execution/feedback.csv').exists():self.last_recording=(control,run);break
                if self.last_recording:break
            if self.last_recording:
                control,relative=self.last_recording;path=control/relative/'capture/execution/feedback.csv'
                if path.exists():
                    row=last_csv(path)
                    if row:self.q=np.array([float(row['q'+str(j)]) for j in range(14)]);self.mode='历史记录 / 尚未连接'
        if not preview and not self.archive:
            from .fieldwork_storage import last_observation
            historical=last_observation(self.root,self.names)
            if historical:
                for key in self.feedback:self.feedback[key]=historical.get(key)
                if self.q is None:self.q=np.asarray(historical['q']);self.mode='历史记录 / 尚未连接'
            if self.last_recording:
                control,relative=self.last_recording;directory=control/relative
                initial=read_json(directory/'initial.json')
                if initial:
                    self.first_q=np.asarray(initial['q_rad']);self.sources=initial['sources']
                    self.commands={key:np.asarray(value) for key,value in initial['command'].items()}
                path=directory/'capture/execution/commands.csv'
                row=last_csv(path) if path.exists() else None
                if row:
                    values={key:np.array([float(row[column+str(j)]) for j in range(14)]) for key,column in
                        [('position','q'),('velocity','v'),('effort','effort'),('kp','kp'),('kd','kd')]}
                    if all(np.isfinite(value).all() for value in values.values()):self.commands=values
        self.has_feedback_pose=self.q is not None
        if self.snapshot:self.message='历史数据只读查看，可导出运动视频或另存。'
        self.pulse();self.start_renderer()

    @property
    def process(self):return self.task.process if self.task else None
    @property
    def busy(self):return self.task is not None
    @property
    def observing(self):return self.observer is not None and not self.observer.done()
    @property
    def connected(self):return self.observing and self.q is not None
    @property
    def pending_start(self):return self.start_requested and not self.sending

    def load_archive(self):
        self.message='旧控制会话只读查看；请使用新output创建工作区，原记录不迁移。'
        self.mode='旧会话只读'
        if not self.status['runs']:return
        self.active_run=self.status['runs'][-1];directory=self.root/self.active_run
        self.last_recording=(self.root,self.active_run)
        initial=read_json(directory/'initial.json')
        path=directory/'capture/execution/feedback.csv'
        if path.exists():
            row=last_csv(path)
            if row:
                self.q=np.array([float(row['q'+str(j)]) for j in range(14)]);self.receive_ns=int(row['receive_ns'])
        if initial:
            self.first_q=np.array(initial['q_rad']);self.commands={k:np.array(v) for k,v in initial['command'].items()};self.sources=initial['sources']

    def heartbeat_row(self,cancel=None):
        return dict(clock_id=self.local_clock,owner_id=self.owner_id,monotonic_ns=time.monotonic_ns(),cancel=cancel)

    def pulse(self, cancel=None):
        if cancel:self.control_cancel=cancel
        final='frontend_closed' if self.closing else None
        row=self.heartbeat_row(final)
        # Deliver stop/close BEFORE any archival I/O. Live workers use pipes.
        if self.task:self.task.pulse(self.heartbeat_row(final or self.control_cancel))
        if self.observer:self.observer.pulse(self.heartbeat_row(final or self.observer_cancel))
        if self.renderer:self.renderer.pulse(row)
        now=time.monotonic()
        if self.heartbeat_archived_at is None or now-self.heartbeat_archived_at>=1. or final or cancel:
            try:latest_json(self.heartbeat,row);self.heartbeat_archived_at=now
            except OSError as exc:self.recording_error='存活标记归档失败（停止通道仍使用管道）：'+str(exc)

    def launch(self, action, values, session_root=None):
        directory=self.root/'operations';directory.mkdir(exist_ok=True);token=uuid.uuid4().hex[:12]
        request=directory/(token+'_request.json');output=directory/(token+'_result.json')
        write_json(request,dict(action=action,values=values))
        command=[sys.executable,'-m','casbot_calib.fieldwork_worker','--session',str(session_root or self.root),
            '--request',str(request),'--heartbeat',str(self.heartbeat),'--owner-id',self.owner_id,
            '--output',str(output),'--pipe-heartbeat']
        child=Child(command,directory/(token+'.log'),pause_file=self.pause_file);child.pulse(self.heartbeat_row())
        return child,dict(action=action,values=values,output=output)

    def start_renderer(self):
        command=[sys.executable,'-m','casbot_calib.fieldwork_worker','--session',str(self.root),
                 '--heartbeat',str(self.heartbeat),'--owner-id',self.owner_id,'--render','--pipe-heartbeat']
        self.renderer=Child(command,self.root/'view'/('renderer_'+self.owner_id[:8]+'.log'),environment=dict(os.environ,MUJOCO_GL='egl'))
        self.renderer.pulse(self.heartbeat_row())

    def connect(self):
        if self.preview or self.read_only or self.observer:return
        self.observer_cancel=None;self.recording_error=None;self.q=self.first_q=self.commands=None;self.receive_ns=None
        self.dirty.clear();self.sources={};self.feedback=dict(q=None,dq=None,effort_raw=None,kp=None,kd=None)
        self.needs_reinitialize=False;self.operator=OperatorStatus();self.control_cancel=None
        self.observer_response=None;self.connected_at=time.monotonic();self.connection_channels={}
        try:self.observer,_=self.launch('observe',dict(relative='observations/'+uuid.uuid4().hex[:12]))
        except (OSError,ValueError,RuntimeError) as exc:self.message='无法建立只读连接：'+str(exc);return
        self.message='只读连接已启动，等待SDK反馈；尚未申请控制。';self.mode='等待反馈'

    def disconnect(self):
        self.stop();self.observer_cancel='operator_disconnect';self.pulse()
        self.message='正在断开只读连接；不自动回位。'

    def refresh_initial(self):
        if self.busy or not self.connected:return
        self.first_q=self.q.copy();self.dirty.clear()
        self.commands,self.sources=initial_commands(self.first_q,self.feedback['kp'],self.feedback['kd'],self.tuning)
        self.needs_reinitialize=False;self.operator=OperatorStatus()
        self.message='已从当前反馈建立新控制起点；尚未发送。'

    def save_initial(self):
        if self.active_run:
            atomic_json(self.control_root/self.active_run/'initial.json',dict(q_rad=self.first_q.tolist(),
                command={k:v.tolist() for k,v in self.commands.items()},sources=self.sources,
                mechanical_zero_known=False,position_limits_source='bundled_URDF_not_device_readback'),replace=True)

    def begin_sending(self):
        if self.preview or self.read_only or not self.observing or self.busy or self.start_requested or self.needs_reinitialize:return
        if self.commands is not None:
            try:validate_commands(self.commands,self.lower,self.upper,self.tuning,self.config['maximum_velocity_rad_s'])
            except ValueError as exc:self.message=str(exc);return
        self.start_requested=True;self.operator=OperatorStatus();self.control_cancel=None
        if self.commands is None:
            self.operator.accept(describe(dict(code='waiting_feedback',phase='WAIT_FEEDBACK',action='wait',since_ns=time.monotonic_ns(),observations=[])))
        else:
            try:self._launch_control()
            except (OSError,ValueError,RuntimeError) as exc:self.start_requested=False;self.message='无法开始控制：'+str(exc)

    def _launch_control(self):
        relative=Workspace(self.root).new_control();self.control_root=self.root/relative;self.control_roots.append(relative)
        self.active_run='runs/'+uuid.uuid4().hex[:12];directory=self.control_root/self.active_run;directory.mkdir(parents=True)
        self.generation=0;self.save_initial()
        self.task,self.pending=self.launch('stream',dict(relative=self.active_run,command_file=str(directory/'latest.csv'),
              initial_values={k:v.tolist() for k,v in self.commands.items()},
              **(dict(recording_pause_file=str(self.pause_file)) if self.pause_file else {})),self.control_root)
        self.response=None;self.message='启动请求已保留，等待后端条件满足；可取消。'

    def start(self, action, **values):
        if action=='stream':self.begin_sending();return
        if action!='video' or self.busy or self.preview or not self.last_recording:return
        control,relative=self.last_recording;self.control_cancel=None
        self.task,self.pending=self.launch('video',dict(relative=relative),control)
        self.response=None;self.message='正在从已记录反馈导出视频。'

    def send(self):
        if not self.task or self.pending['action']!='stream' or self.control_cancel:return
        path=self.control_root/self.active_run/'latest.csv'
        if not path.exists():return  # Worker creates the first frame only after checking its empty mailbox.
        arrays=validate_commands(self.commands,self.lower,self.upper,self.tuning,self.config['maximum_velocity_rad_s'])
        self.generation+=1;write_command(path,self.generation,arrays)
        if self.pause_file and self.pause_file.exists():return
        with (path.parent/'target_edits.jsonl').open('a') as stream:
            import json
            stream.write(json.dumps(dict(monotonic_ns=time.monotonic_ns(),generation=self.generation,
                **{k:v.tolist() for k,v in arrays.items()}))+'\n')

    def change(self, name, index, value):
        old=self.commands[name][index];self.commands[name][index]=value
        try:
            validate_commands(self.commands,self.lower,self.upper,self.tuning,self.config['maximum_velocity_rad_s'])
            if self.busy and self.pending['action']=='stream':self.send()
        except (ValueError,OSError) as exc:
            self.commands[name][index]=old;self.message='本次修改未采纳：'+str(exc);return
        self.dirty.add(name)

    def stop(self):
        pending=self.pending_start;self.short=None;self.start_requested=False
        if self.busy and self.pending['action']=='stream':self.control_cancel='operator_cancel'
        self.pulse();self.message='已请求取消启动。' if pending else '已请求停止发送；只读反馈继续，不自动回位。'
        if not self.busy:self.operator=OperatorStatus()

    def _feedback(self,row):
        self.mode=row['mode']
        for key in self.feedback:
            self.feedback[key]=row.get(key)
        if row.get('q') is not None:
            self.q=np.asarray(row['q']);self.receive_ns=row['receive_ns'];self.has_feedback_pose=True
        if self.q is not None and self.commands is None:
            self.first_q=self.q.copy();self.commands,self.sources=initial_commands(self.first_q,self.feedback['kp'],self.feedback['kd'],self.tuning)
        if self.commands is not None and not self.busy:
            for key in ('kp','kd'):
                if key not in self.dirty and self.sources[key].startswith('配置默认') and self.feedback[key] is not None:
                    self.commands[key]=np.asarray(self.feedback[key]);self.sources[key]='首次可用生效增益反馈'

    def _handle(self,item,observer=False):
        kind=item['type']
        if kind=='feedback':self._feedback(item)
        elif kind in ('observer_started','connection_status'):self.connection_channels=item.get('channels',{})
        elif kind=='recording_error':
            self.recording_error='当前录制不完整；实时反馈和停止通道继续。原因：'+item['message'];self.short=None
        elif kind=='status':
            value=item['event']
            if value['code']=='recording_io_error':self.recording_error=value['title']+'；'+value.get('effect','');self.short=None
            previous=self.operator.current
            if previous is None or (previous['code'],previous.get('phase'))!=(value['code'],value.get('phase')):
                print(value['title']+'；'+value['effect']+'\n'+'\n'.join(value['details']),file=sys.stderr)
            self.operator.accept(value)
            if value.get('action')=='sending':self.message='五字段发送中；拖动滑条更新目标。'
            self.sending=value.get('action')=='sending' or (self.sending and value.get('action') not in ('end_control','cancel_or_stop','observe_after_stop','control_ended'))
            if value.get('action') in ('end_control','cancel_or_stop','observe_after_stop','control_ended'):
                self.sending=False;self.start_requested=False;self.short=None
            if self.pending_start or self.sending:self.send()
        elif kind=='response':
            if observer:self.observer_response=item['response']
            else:self.response=item['response']
        elif kind=='ipc_error':self.message=item['message']

    def begin_save(self):
        from .fieldwork_storage import SaveJob
        if self.save_job and not self.save_job.done:return
        self.save_job=SaveJob(self.root,self.save_path);self.save_message='正在保存已完成写入的数据段；控制和只读连接继续。'

    def poll(self):
        if getattr(self,'save_job',None) and self.save_job.done:
            if self.save_job.error:self.save_message='保存失败：'+self.save_job.error
            else:self.save_message='会话已保存：'+self.save_job.result['destination'];self.save_open=False
            self.save_job=None
        monitor=getattr(self,'cache_monitor',None)
        if monitor and monitor.paused:
            self.recording_error='缓存容量已达到上限，已停止追加记录；反馈和手动控制继续。请保存已有记录，下一次使用新会话。';self.short=None
        if monitor and monitor.error:self.recording_error='缓存检查失败：'+monitor.error
        if getattr(self,'owned_mock',None) and self.owned_mock.done() and self.observing:
            self.message='本地Mock进程已结束；等待反馈超时处理，不自动重启。'
        if self.observer:
            for item in self.observer.drain():self._handle(item,True)
            if self.observer.done():
                self.observer.close();self.observer=None
                if self.start_requested or self.sending:
                    if self.observer_cancel:self.stop()
                    else:
                        self.start_requested=False;self.short=None;self.control_cancel='observation_exited'
                        self.operator.accept(describe(dict(code='observation_exited',action='end_control',observations=[])))
                        self.pulse()
                self.mode='只读连接已结束 / 末次反馈';self.message='只读连接已结束；末次反馈保留，可重新连接。'
                if not self.observer_cancel:
                    response=self.observer_response or {}
                    self.message+=' 原因：'+str(response.get('error') or '记录器退出码 '+str(response.get('result',{}).get('result',{}).get('returncode','未知；查看operations日志')))
        if self.task:
            for item in self.task.drain():
                try:self._handle(item)
                except (OSError,ValueError) as exc:self.recording_error=str(exc)
            if self.task.done():
                for item in self.task.drain():
                    try:self._handle(item)
                    except (OSError,ValueError) as exc:self.recording_error=str(exc)
                code=self.task.process.returncode;self.task.close();self.task=None
                self.sending=False;self.start_requested=False;self.short=None
                if self.pending['action']=='stream':
                    self.needs_reinitialize=True
                    if (self.control_root/self.active_run/'capture/execution/feedback.csv').exists():self.last_recording=(self.control_root,self.active_run)
                    if self.response and self.response['completed']:
                        result=self.response['result']
                        if result.get('recording_error'):self.recording_error=result['recording_error']
                        if self.operator.first_fault:self.message='本次控制已结束；只读反馈继续。点“重读 / 新控制”建立新起点。'
                        elif result.get('controlled_stop'):self.message='已取消启动或停止发送；只读反馈继续。'
                        else:self.message='发送器结束：'+str(result['result'].get('fault') or result['result']['reason'])+'；'+ '\n'.join(result.get('diagnostics',[]))
                    else:self.message='本次控制结束：'+(self.response or {}).get('error',f'后台进程退出码 {code}，查看operations日志')
                elif self.response and self.response['completed']:self.message='视频已保存：'+self.response['result']['video']
                else:self.message='视频导出失败：'+(self.response or {}).get('error',f'进程退出码 {code}')
        if self.start_requested and not self.busy and self.commands is not None and self.observing:
            try:self._launch_control()
            except (OSError,ValueError,RuntimeError) as exc:self.start_requested=False;self.message='无法开始控制：'+str(exc)
        if self.q is not None and time.monotonic()-self.view_last>1/VIEW_FPS:
            pose=dict(q=self.q.tolist(),selected=self.selected,receive_ns=self.receive_ns,source='SDK_feedback' if self.observing else 'recorded_SDK_feedback')
            if pose!=self.last_render_pose:
                try:latest_json(self.root/'view/pose.json',pose);self.last_render_pose=pose
                except OSError as exc:self.render_error='几何画面更新失败，数值反馈仍独立接收：'+str(exc)
            self.view_last=time.monotonic()
        if self.renderer and self.renderer.done() and not self.closing:self.render_error='几何视图进程已结束；数值反馈和停止通道仍独立运行。'
        if self.short and self.sending:
            elapsed=time.monotonic()-self.short['start'];target=short_position(self.short['anchor'],self.short['amplitude'],self.short['duration'],elapsed)
            self.change('position',self.short['joint'],target)
            if elapsed>=self.short['duration']:self.short=None

    def update_texture(self):
        from OpenGL import GL
        path=self.root/'view/frame.npy'
        if not path.exists():return
        stamp=path.stat().st_mtime_ns
        if stamp==self.texture_stamp:return
        try:pixels=np.load(path,allow_pickle=False)
        except (OSError,ValueError):return
        allocate=self.texture is None
        if allocate:self.texture=int(GL.glGenTextures(1))
        GL.glBindTexture(GL.GL_TEXTURE_2D,self.texture)
        GL.glTexParameteri(GL.GL_TEXTURE_2D,GL.GL_TEXTURE_MIN_FILTER,GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D,GL.GL_TEXTURE_MAG_FILTER,GL.GL_LINEAR)
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT,1)
        if allocate:GL.glTexImage2D(GL.GL_TEXTURE_2D,0,GL.GL_RGB,pixels.shape[1],pixels.shape[0],0,GL.GL_RGB,GL.GL_UNSIGNED_BYTE,pixels)
        else:GL.glTexSubImage2D(GL.GL_TEXTURE_2D,0,0,0,pixels.shape[1],pixels.shape[0],GL.GL_RGB,GL.GL_UNSIGNED_BYTE,pixels)
        GL.glBindTexture(GL.GL_TEXTURE_2D,0);self.texture_stamp=stamp;self.rendered_frames+=1

    def close(self):
        if self.closing and not any((self.task,self.observer,self.renderer)):return
        self.closing=True;self.pulse()
        for name in ('task','observer','renderer'):
            child=getattr(self,name)
            if child:
                try:child.process.wait(timeout=12 if name!='renderer' else 5)
                except subprocess.TimeoutExpired:child.process.terminate();child.process.wait(timeout=5)
                child.close();setattr(self,name,None)
        if self.save_job:self.save_job.thread.join()
        if self.frontend_lease:self.frontend_lease.close();self.frontend_lease=None

    def gui(self):
        from imgui_bundle import imgui
        imgui.push_id(self.owner_id)
        self.frame+=1;self.controls.clear();self.control_bounds.clear();self.poll();self.update_texture()
        def remember(key):
            a=imgui.get_item_rect_min();b=imgui.get_item_rect_max()
            self.controls[key]=((a.x+b.x)/2,(a.y+b.y)/2);self.control_bounds[key]=(a.x,a.y,b.x,b.y)
        def button(key,label,enabled=True):
            imgui.begin_disabled(not enabled);clicked=imgui.button(label);remember(key);imgui.end_disabled();return clicked
        def number(value):return '—' if value is None else f'{value:.5f}'
        imgui.push_font(self.heading_font);imgui.text('双臂手动调参');imgui.pop_font()
        imgui.same_line();imgui.text_disabled('离线预览' if self.preview else f"域 {self.status['domain_id']} / {self.status['scope']} / 原始五字段")
        if self.cache_monitor:
            imgui.same_line();imgui.text_disabled(f'缓存 {self.cache_monitor.used/1024**2:.1f}/{self.cache_monitor.limit/1024**2:.0f} MiB'+(' / 本窗口Mock' if self.owned_mock else ''))
        if button('connect','连接 / 读取',not self.preview and not self.read_only and self.observer is None):self.connect()
        imgui.same_line()
        if button('send','开始发送',self.observing and not self.busy and not self.start_requested and not self.needs_reinitialize):self.begin_sending()
        imgui.same_line()
        if button('stop','取消启动' if self.pending_start else '停止发送',self.start_requested or self.sending or (self.busy and self.pending['action']=='stream')):self.stop()
        imgui.same_line()
        if button('disconnect','断开连接',self.observing):self.disconnect()
        imgui.same_line()
        if button('refresh','重读 / 新控制',self.connected and not self.busy and not self.start_requested):self.refresh_initial()
        imgui.same_line()
        if button('video','导出运动视频',not self.busy and self.last_recording is not None):self.start('video')
        imgui.same_line()
        if button('save_session','保存会话',not self.preview and self.save_job is None):
            from datetime import datetime
            self.save_path=str(Path('artifacts/fieldwork/saved')/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]))
            self.save_open=True
        imgui.same_line()
        if button('recover_menu','缓存记录'):
            from .fieldwork_storage import cached_sessions
            self.recovery_rows=cached_sessions();imgui.open_popup('recovery_list')
        if self.save_open:
            imgui.set_next_item_width(-220);_,self.save_path=imgui.input_text('保存到',self.save_path);remember('save_path')
            imgui.same_line()
            if button('save_apply','保存',self.save_job is None and bool(self.save_path.strip())):self.begin_save()
            imgui.same_line()
            if button('save_cancel','取消',self.save_job is None):self.save_open=False
        if self.save_message:imgui.text_wrapped(self.save_message)
        if self.snapshot:imgui.text_disabled('已保存快照 / 只读；实时运行请另开新会话。')
        if imgui.begin_popup('recovery_list'):
            imgui.text('恢复查看缓存；不会自动连接或发送')
            if not self.recovery_rows:imgui.text_disabled('暂无缓存记录')
            for row in self.recovery_rows:
                if Path(row['workspace'])==self.root:continue
                imgui.text(row['id']);imgui.same_line()
                if button('recover_'+row['id'],('已打开' if row['active'] else '打开')+'##'+row['id'],not row['active'] and not self.busy and not self.save_job):self.reopen_root=Path(row['workspace']);imgui.close_current_popup()
            imgui.end_popup()
        if self.observing and self.connected_at is not None:
            joint=self.connection_channels.get('joint',{});count=joint.get('publishers')
            label='正在发现发布者' if count is None else f'发布者 {count}'
            elapsed=time.monotonic()-self.connected_at
            imgui.text_wrapped(f"目标 {joint.get('topic',self.config['joint_topic'])} / {label}")
            if self.receive_ns is None:
                imgui.text_wrapped(f'等待首次完整位置反馈 {elapsed:.1f} s；尚未收到有效数据。'+('目标尚未出现：请确认服务在运行且profile属于本次Mock实例。' if count==0 else ''))
        event=self.operator.displayed
        if event:
            followup=self.operator.current if self.operator.first_fault and self.operator.current is not event else None
            color=imgui.ImVec4(.68,.14,.12,1) if event['severity']=='error' else imgui.ImVec4(.58,.34,.08,1) if event['severity']=='warning' else imgui.ImVec4(.10,.34,.58,1)
            imgui.begin_child('control_status',imgui.ImVec2(0,75+min(3,len(event['details']))*23+(28 if followup else 0)),imgui.ChildFlags_.border)
            imgui.text_colored(color,event['title']);imgui.same_line();imgui.text_disabled(event['code'])
            if event.get('action') in ('wait','request_mode','observe_after_stop') and event.get('since_ns'):
                imgui.same_line();imgui.text_disabled(f"已等待 {(time.monotonic_ns()-event['since_ns'])*1e-9:.1f} s")
            imgui.text_wrapped(event['effect'])
            for detail in event['details']:imgui.text_wrapped(detail)
            if followup:imgui.text_wrapped('后续状态：'+followup['title'])
            imgui.end_child()
        imgui.text_wrapped(self.message)
        if self.recording_error:
            imgui.push_style_color(imgui.Col_.text,imgui.ImVec4(.68,.14,.12,1))
            imgui.text_wrapped('录制问题：'+self.recording_error);imgui.pop_style_color()
        if self.render_error:imgui.text_colored(imgui.ImVec4(.58,.34,.08,1),self.render_error)
        imgui.separator();imgui.push_style_var(imgui.StyleVar_.item_spacing,imgui.ImVec2(8,5))
        width=imgui.get_content_region_avail().x;height=imgui.get_content_region_avail().y
        imgui.begin_child('axes',imgui.ImVec2(max(590.,width*.55),height),imgui.ChildFlags_.border)
        imgui.text('位置 / rad');imgui.same_line();imgui.text_disabled('查看轴不改变目标；起点不是机械零位')
        highlighted={x['joint'] for x in (event or {}).get('observations',[]) if x.get('joint')}
        if imgui.begin_table('axes_table',5,imgui.TableFlags_.sizing_stretch_prop):
            for title,weight in [('SDK轴 / 序号',1.25),('目标',1.65),('URDF范围',1.35),('当前',.8),('本次起点',.8)]:imgui.table_setup_column(title,init_width_or_weight=weight)
            imgui.table_headers_row()
            for i,name in enumerate(self.names):
                imgui.push_id(i);imgui.table_next_row();imgui.table_next_column()
                label=('左' if i<7 else '右')+f' {i%7+1} '+JOINT_LABELS[i%7]
                if name in highlighted:imgui.push_style_color(imgui.Col_.text,imgui.ImVec4(.7,.13,.12,1))
                clicked,_=imgui.selectable(label,self.selected==i)
                if clicked:self.selected=i
                if name in highlighted:imgui.pop_style_color()
                if imgui.is_item_hovered():imgui.set_tooltip(name+'\n序号是数组顺序，不是设备硬件ID')
                imgui.table_next_column();imgui.set_next_item_width(-1)
                imgui.begin_disabled(self.commands is None or self.read_only or self.short is not None or self.needs_reinitialize)
                value=float(self.commands['position'][i]) if self.commands is not None else 0.
                changed,value=imgui.slider_float('##q',value,float(self.lower[i]),float(self.upper[i]),'%.4f',imgui.SliderFlags_.always_clamp)
                remember('position_'+str(i));imgui.end_disabled()
                if changed:self.selected=i;self.change('position',i,value)
                imgui.table_next_column();imgui.text_disabled(f'{self.lower[i]:.3f}…{self.upper[i]:.3f}')
                imgui.table_next_column();imgui.text(number(None if self.q is None else self.q[i]))
                imgui.table_next_column();imgui.text(number(None if self.first_q is None else self.first_q[i]));imgui.pop_id()
            imgui.end_table()
        imgui.separator();imgui.text_wrapped(self.names[self.selected]);imgui.text_wrapped(f'模型轴向 {self.axes[self.selected]}；min/max来自URDF。')
        imgui.text_wrapped('模式：'+self.mode)
        imgui.end_child();imgui.same_line()
        imgui.begin_child('tuning',imgui.ImVec2(0,height),imgui.ChildFlags_.border)
        imgui.text('SDK反馈模型 / 显示网格不用于碰撞')
        image_width=min(imgui.get_content_region_avail().x,380.)
        if self.texture is not None:imgui.image(self.texture,imgui.ImVec2(image_width,image_width*.75))
        else:imgui.dummy(imgui.ImVec2(image_width,image_width*.75));imgui.text('正在载入几何视图…')
        age=(time.monotonic_ns()-self.receive_ns)*1e-6 if self.receive_ns else None
        live=self.observing and age is not None and age<=self.config['max_feedback_age_ns']*1e-6
        imgui.text_disabled('只读反馈更新中' if live else '末次反馈 / 未收到新数据' if self.q is not None else '末次反馈模型 / 尚无本次数据' if self.has_feedback_pose else '名义模型 / 尚无反馈')
        if age is not None:imgui.same_line();imgui.text_disabled(f'回调间隔 {age:.0f} ms')
        i=self.selected;imgui.separator();imgui.text(('左臂 ' if i<7 else '右臂 ')+JOINT_LABELS[i%7]+' / 五字段')
        if self.commands is not None:imgui.text(f"目标 {self.commands['position'][i]:.5f} rad    当前 {self.q[i]:.5f} rad")
        specs=[('velocity','目标速度 / rad/s',-self.config['maximum_velocity_rad_s'],self.config['maximum_velocity_rad_s']),
               ('kp','Kp / Nm/rad',0.,self.tuning['kp_max']),('kd','Kd / Nm·s/rad',0.,self.tuning['kd_max']),
               ('effort','前馈力矩 / Nm',-self.tuning['effort_limit_nm'],self.tuning['effort_limit_nm'])]
        for key,label,lo,hi in specs:
            imgui.text(label);imgui.same_line(185);imgui.set_next_item_width(-1)
            imgui.begin_disabled(self.commands is None or self.read_only or self.needs_reinitialize)
            value=float(self.commands[key][i]) if self.commands is not None else 0.
            changed,value=imgui.slider_float('##'+key,value,float(lo),float(hi),'%.4f',imgui.SliderFlags_.always_clamp);remember(key);imgui.end_disabled()
            if changed:self.change(key,i,value)
            if imgui.is_item_hovered():imgui.set_tooltip(self.sources.get(key,'连接后初始化'))
        f=self.feedback
        imgui.text_wrapped('回读dq '+number(None if f['dq'] is None else f['dq'][i])+' rad/s；估算effort '+number(None if f['effort_raw'] is None else f['effort_raw'][i]))
        imgui.text_wrapped('生效Kp '+number(None if f['kp'] is None else f['kp'][i])+'；Kd '+number(None if f['kd'] is None else f['kd'][i]))
        if self.sources:imgui.text_wrapped('初始化：Kp/Kd '+('回读' if '反馈' in self.sources['kp'] else '配置默认值'))
        imgui.separator();imgui.set_next_item_width(105)
        _,self.short_amplitude=imgui.slider_float('幅度/°',self.short_amplitude,.01,1.,'%.2f',imgui.SliderFlags_.always_clamp)
        imgui.same_line();imgui.set_next_item_width(90)
        _,self.short_duration=imgui.slider_float('时长/s',self.short_duration,2.,12.,'%.1f',imgui.SliderFlags_.always_clamp)
        if button('short','所选轴短轨迹往返',self.sending and self.short is None):
            amplitude=np.deg2rad(self.short_amplitude);anchor=self.commands['position'][i]
            if self.lower[i]<=anchor-amplitude and anchor+amplitude<=self.upper[i]:self.short=dict(start=time.monotonic(),duration=self.short_duration,amplitude=amplitude,anchor=anchor,joint=i)
            else:self.message='短轨迹端点超出'+self.names[i]+'的URDF范围，本次轨迹未启动。'
        imgui.end_child();imgui.pop_style_var();self.pulse();imgui.pop_id()
def run_gui(args):
    if getattr(args,'software_rendering',False):os.environ['LIBGL_ALWAYS_SOFTWARE']='1'
    configure_graphics_environment()
    if args.screenshot and args.screenshot.exists():raise ValueError('截图输出已存在，请使用新文件')
    if args.frames is not None and args.frames<=0:raise ValueError('--frames必须为正整数')
    from .fieldwork_launch import prepare_gui
    import copy
    while True:
        with prepare_gui(args) as (root,monitor,mock):
            reopen=_run_window(args,root,monitor,mock)
        if reopen is None:return 0
        args=copy.copy(args);args.session=reopen;args.output=args.domain_id=args.scope=None
        args.mock=False;args.recover=None;args.preview=False;args.screenshot=None


def _run_window(args,root,monitor,mock):
    from imgui_bundle import imgui, hello_imgui
    desk = Desk(root, preview=args.preview, profile=args.profile)
    desk.cache_monitor=monitor;desk.owned_mock=mock
    hello_imgui.set_assets_folder(str(Path(root).resolve()))
    params = hello_imgui.RunnerParams()
    params.app_window_params.window_title = 'CASBOT Fieldwork'
    params.app_window_params.window_geometry.size = [1440, 980]
    params.app_window_params.restore_previous_geometry = False
    params.ini_filename = str(Path(root)/'window.ini')
    params.imgui_window_params.show_menu_bar = False
    params.imgui_window_params.show_status_bar = False
    # Hello ImGui expresses these margins in em, not pixels.
    params.imgui_window_params.full_screen_window_margin_top_left = imgui.ImVec2(.75, .5)
    params.imgui_window_params.full_screen_window_margin_bottom_right = imgui.ImVec2(.75, .5)
    # Refresh live numbers/heartbeat at 20 Hz while idle, respond at up to 30 Hz
    # when interacting. Neither rate is the native command/physics frequency.
    params.fps_idling.enable_idling = True
    params.fps_idling.fps_idle = 20.
    params.fps_idling.time_active_after_last_event = .5
    params.callbacks.pre_new_frame = FramePacer()
    # ImGui fonts/edges are already antialiased; avoid a multisampled full-window
    # framebuffer in the software desktop renderer. CAD rendering is separate.
    params.renderer_backend_options.open_gl_options.anti_aliasing_samples = 0
    def fonts():
        from .presentation import _font
        path = _font(18).path
        io = imgui.get_io()
        glyphs = [0x20, 0xFF, 0x2000, 0x206F, 0x3000, 0x30FF, 0x4E00, 0x9FFF, 0xFF00, 0xFFEF, 0]
        io.font_default = io.fonts.add_font_from_file_ttf(path, 18, glyph_ranges_as_int_list=glyphs)
        desk.heading_font = io.fonts.add_font_from_file_ttf(path, 25, glyph_ranges_as_int_list=glyphs)
    def style():
        imgui.style_colors_light()
        s = imgui.get_style(); s.window_padding = imgui.ImVec2(16, 12); s.item_spacing = imgui.ImVec2(10, 8)
        s.frame_padding = imgui.ImVec2(8, 5); s.frame_rounding = 3; s.child_rounding = 4
        for name, color in dict(window_bg=(.94,.96,.98,1), child_bg=(.99,.995,1,1),
            text=(.10,.18,.28,1), text_disabled=(.34,.40,.47,1), border=(.77,.82,.87,1),
            button=(.80,.88,.96,1), button_hovered=(.65,.79,.94,1), button_active=(.48,.68,.89,1),
            check_mark=(.12,.40,.69,1), header=(.75,.85,.94,1), plot_lines=(.12,.40,.69,1)).items():
            s.set_color_(getattr(imgui.Col_, name), imgui.ImVec4(*color))
    params.callbacks.load_additional_fonts = fonts
    params.callbacks.setup_imgui_style = style
    def gui():
        desk.gui()
        # Keep GUI-generated short trajectories on the active redraw budget.
        params.fps_idling.enable_idling = desk.short is None
        if desk.reopen_root or (args.frames and desk.frame >= args.frames):
            params.app_shall_exit = True
    params.callbacks.show_gui = gui
    params.callbacks.before_exit = desk.close
    try:
        hello_imgui.run(params)
        if args.screenshot:
            from PIL import Image
            Image.fromarray(hello_imgui.final_app_window_screenshot()).save(args.screenshot)
    finally:
        desk.close()
    return desk.reopen_root
