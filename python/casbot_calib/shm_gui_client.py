"""Non-blocking frontend adapter; lease renewal is driven only by GUI tick()."""
from collections import deque
import json
import os
from pathlib import Path
import queue
import shlex
import subprocess
import sys
import threading
import time

from .shm_gui_protocol import json_frame, read_frame, telemetry
from .shm_gui_remote import encode_config


class RemoteClient:
    def __init__(self, config, local_root, *, host=None, repo=None, ssh_sockets=None):
        self.config=dict(config);self.root=Path(local_root);self.root.mkdir(parents=True,exist_ok=True)
        self.host=host;self.repo=repo;self.ssh_sockets=ssh_sockets or (None,None)
        if host and (not repo or host.startswith('-')):raise ValueError('explicit_host_and_repo_required')
        if self.ssh_sockets[0] and self.ssh_sockets[0]==self.ssh_sockets[1]:raise ValueError('independent_ssh_transports_required')
        self.messages=queue.Queue(256);self.outgoing=deque();self.lock=threading.Lock();self.wake=threading.Event()
        self.process=None;self.display=None;self.latest=None;self.received_at=0;self.ready=None;self.hello=None
        self.failure=None;self.control=0;self.revision=0;self.id=0;self.closed=False;self.stopping=False
        self.last_renew=0;self.last_challenge=None;self.last_fields=0;self.bytes_received=0;self.rtt_ms=None;self.requests={}
        self.threads=[];self.logs=[]
    def command(self, action, *extra, channel=0):
        args=['python3' if self.host else sys.executable,'-u','-m','casbot_calib.shm_gui_remote',action,*extra]
        if not self.host:return args
        command='cd '+shlex.quote(str(self.repo))+' && PYTHONPATH='+shlex.quote(str(Path(self.repo)/'python'))+' exec '+shlex.join(args)
        socket=self.ssh_sockets[channel]
        return ['ssh','-T','-x','-o','Compression=no','-o','ControlMaster=no','-o','ControlPath='+str(socket or 'none'),'-o','ConnectTimeout=7',self.host,command]
    def _thread(self, target):
        thread=threading.Thread(target=target,daemon=True);self.threads.append(thread);thread.start()
    def connect(self):
        if self.process:raise ValueError('client_already_connected')
        log=(self.root/'control_ssh.log').open('wb');self.logs.append(log)
        self.process=subprocess.Popen(self.command('serve','--config',encode_config(self.config)),stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log)
        def read_control():
            try:
                while True:
                    kind,body=read_frame(self.process.stdout)
                    if kind!=1:raise ValueError('control_frame_type')
                    message=json.loads(body)
                    if message['type']=='remote_ready':
                        self.ready=message;self._display(message['socket'])
                    elif message['type']=='hello':self.hello=message
                    elif message['type']=='reply':
                        sent=self.requests.pop(message.get('id'),None)
                        if sent:self.rtt_ms=(time.monotonic_ns()-sent)*1e-6
                    self.messages.put_nowait(message)
            except (EOFError,OSError,ValueError,queue.Full) as error:
                if not self.closed:self.failure='控制连接已断开：'+str(error)
        def write_control():
            while not self.closed:
                self.wake.wait(.05);self.wake.clear()
                with self.lock:item=self.outgoing.popleft() if self.outgoing else None
                if item is None:continue
                try:self.process.stdin.write(json_frame(item));self.process.stdin.flush()
                except (OSError,ValueError):
                    if not self.closed:self.failure='控制发送失败'
                    break
                with self.lock:
                    if self.outgoing:self.wake.set()
        self._thread(read_control);self._thread(write_control)
    def _display(self, path):
        log=(self.root/'display_ssh.log').open('wb');self.logs.append(log)
        self.display=subprocess.Popen(self.command('subscribe','--socket',path,channel=1),stdout=subprocess.PIPE,stderr=log)
        def read_display():
            try:
                while True:
                    kind,body=read_frame(self.display.stdout)
                    if kind!=2:raise ValueError('display_frame_type')
                    value=telemetry(body)
                    if self.latest and value['seq']<=self.latest['seq']:raise ValueError('display_sequence')
                    self.latest=value;self.received_at=time.monotonic_ns();self.bytes_received+=len(body)+5
            except (EOFError,OSError,ValueError) as error:
                if not self.closed:self.failure='反馈连接已断开：'+str(error)
        self._thread(read_display)
    def send(self, type, **values):
        self.id+=1;item=dict(type=type,id=self.id,**values)
        if type=='start':self.control=self.id;self.revision=0;self.stopping=False
        if type=='fields':
            self.revision+=1;item.update(control=self.control,revision=self.revision)
        if type in ('stop','quit'):self.stopping=True
        with self.lock:
            if type in ('stop','quit'):self.outgoing.clear();self.outgoing.appendleft(item)
            else:
                if type in ('fields','renew'):self.outgoing=deque(x for x in self.outgoing if x['type']!=type)
                if len(self.outgoing)>=16:raise RuntimeError('控制收件箱已满')
                self.outgoing.append(item)
        if type!='renew':
            self.requests[self.id]=time.monotonic_ns()
            while len(self.requests)>128:self.requests.pop(next(iter(self.requests)))
        self.wake.set();return self.id
    def tick(self):
        """Must run on GUI main loop. No timer/network thread calls this method."""
        now=time.monotonic_ns();s=self.latest
        if self.failure or not s or not s['receive_ns'] or now-self.received_at>500_000_000:
            if self.control and not self.stopping:self.send('stop',control=self.control)
            return
        if 0<=s['phase']<4 and not self.stopping and now-self.last_renew>=100_000_000 and s['challenge']!=self.last_challenge:
            self.last_challenge=s['challenge'];self.last_renew=now
            self.send('renew',control=self.control,challenge=s['challenge'],seq=s['seq'])
    def drain(self):
        result=[]
        while True:
            try:result.append(self.messages.get_nowait())
            except queue.Empty:return result
    def close(self):
        if self.closed:return
        if self.process and self.process.poll() is None:
            try:
                self.send('quit')
                end=time.monotonic()+4
                while self.process.poll() is None and time.monotonic()<end:time.sleep(.02)
            finally:
                # Closing SSH also closes the bridge input; the native lease remains independent.
                if self.process.poll() is None:self.process.terminate()
                try:self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:pass
        self.closed=True;self.wake.set()
        if self.display and self.display.poll() is None:
            self.display.terminate()
            try:self.display.wait(timeout=2)
            except subprocess.TimeoutExpired:pass
        for thread in self.threads:thread.join(timeout=.2)
        for log in self.logs:log.close()
