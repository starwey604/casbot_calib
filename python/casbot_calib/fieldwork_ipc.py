"""Local pipe status/heartbeat. Recording files are archives, never the stop channel."""
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time

PREFIX = 'FIELDWORK_IPC '
_EMIT_LOCK=threading.Lock()


def emit(kind, **values):
    with _EMIT_LOCK:
        print(PREFIX+json.dumps(dict(type=kind, **values), ensure_ascii=False, allow_nan=False), flush=True)


class PipeHeartbeat:
    def __init__(self, fd, owner, timeout_s=2.):
        from .fieldwork import clock_id
        self.fd=fd; self.owner=owner; self.clock=clock_id(); self.timeout_s=timeout_s
        self.latest=None; self.buffer=b''; self.eof=False; self.created=time.monotonic_ns()
        os.set_blocking(fd,False)

    def __call__(self):
        while True:
            try: chunk=os.read(self.fd,65536)
            except BlockingIOError: break
            if not chunk: self.eof=True; break
            self.buffer+=chunk
            while b'\n' in self.buffer:
                line,self.buffer=self.buffer.split(b'\n',1)
                try: self.latest=json.loads(line)
                except (ValueError,UnicodeError): return 'frontend_heartbeat_invalid'
            if len(self.buffer)>65536: return 'frontend_heartbeat_invalid'
        if self.eof: return 'frontend_closed'
        if self.latest is None:
            return 'frontend_heartbeat_timeout' if self.timeout_s is not None and time.monotonic_ns()-self.created>self.timeout_s*1e9 else None
        row=self.latest
        if not isinstance(row,dict): return 'frontend_heartbeat_invalid'
        if row.get('owner_id')!=self.owner: return 'frontend_owner_changed'
        if row.get('clock_id')!=self.clock: return 'frontend_clock_mismatch'
        if row.get('cancel'): return row['cancel']
        stamp=row.get('monotonic_ns')
        if not isinstance(stamp,int) or time.monotonic_ns()<stamp or (self.timeout_s is not None and time.monotonic_ns()-stamp>self.timeout_s*1e9):
            return 'frontend_heartbeat_timeout'
        return None


class Child:
    """One producer for heartbeat, background reader drains live output even during UI stalls."""
    def __init__(self, command, log, *, environment=None, pause_file=None, pass_fds=()):
        self.pause_file=Path(pause_file) if pause_file else None
        self.log=Path(log).open('x'); self.messages=queue.SimpleQueue(); self.log_error=None
        self.latest_feedback=None;self.feedback_lock=threading.Lock()
        try:
            self.process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,text=True,bufsize=1,env=environment,pass_fds=pass_fds)
        except Exception:
            self.log.close();raise
        os.set_blocking(self.process.stdin.fileno(),False)
        self.reader=threading.Thread(target=self._read,daemon=True);self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            if line.startswith(PREFIX):
                try:
                    value=json.loads(line[len(PREFIX):])
                    if value.get('type')=='feedback':
                        with self.feedback_lock:self.latest_feedback=value
                    else:self.messages.put(value)
                except ValueError: self.messages.put(dict(type='ipc_error',message='后台状态消息无法解析'))
            if self.log_error is None and not (self.pause_file and self.pause_file.exists()):
                try: self.log.write(line); self.log.flush()
                except OSError as exc:
                    self.log_error=str(exc);self.messages.put(dict(type='recording_error',message='后台日志写入失败：'+str(exc)))
        self.process.stdout.close()

    def pulse(self, row):
        if self.process.poll() is not None:return
        data=(json.dumps(row,ensure_ascii=False)+'\n').encode()
        try: os.write(self.process.stdin.fileno(),data)
        except BlockingIOError: pass  # Never block the UI; a newer pulse follows next frame.
        except BrokenPipeError: pass

    def drain(self):
        with self.feedback_lock:
            result=[self.latest_feedback] if self.latest_feedback else []
            self.latest_feedback=None
        while True:
            try: result.append(self.messages.get_nowait())
            except queue.Empty:return result

    def done(self): return self.process.poll() is not None

    def close(self):
        if self.process.stdin and not self.process.stdin.closed:self.process.stdin.close()
        self.reader.join(timeout=2.)
        try:self.log.close()
        except OSError as exc:self.log_error=str(exc)
