"""Shared SDK process lifetime; callers retain scope, stop policy and result meaning."""
from contextlib import contextmanager
from dataclasses import dataclass
import json
from pathlib import Path
import signal
import subprocess
import time
import threading


def ros_arguments(binary,parameters,remaps=None):
    command=[str(binary),'--ros-args']
    for key,value in parameters.items():
        encoded="''" if value=='' else json.dumps(value) if isinstance(value,(bool,list)) else str(value)
        command+=['-p',key+':='+encoded]
    for source,target in (remaps or {}).items():command+=['-r',source+':='+target]
    return command


def stop_process(process,*,first_signal=signal.SIGINT,timeout_s=10):
    if process.poll() is None:
        process.send_signal(first_signal)
        try:process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:process.kill();process.wait()
    return process.returncode


@dataclass(frozen=True)
class ProcessResult:
    returncode:int
    interruption:str|None=None
    started:bool=True


class SharedRecorder:
    """Watch the exact local recorder process without owning or stopping it."""
    def __init__(self, metadata):
        self.pid=int(metadata['recorder_pid']);self.start=str(metadata['process_start_ticks'])
        self.startup_interruption='raw_recorder_exited' if self.poll() is not None else None
    def poll(self):
        try:
            stat=Path(f'/proc/{self.pid}/stat').read_text();fields=stat[stat.rfind(')')+2:].split()
            return None if fields[19]==self.start and fields[0] not in ('Z','X') else 1
        except (FileNotFoundError,ProcessLookupError):return 1


class OutputPump:
    """Drain child status independently of an optional archival log."""
    def __init__(self, source, log, callback, pause_file=None):
        self.pause_file=Path(pause_file) if pause_file else None
        self.source=source; self.log=log; self.callback=callback; self.error=None; self.log_failed=False
        self.thread=threading.Thread(target=self._read,daemon=True);self.thread.start()
    def _read(self):
        try:
            for line in self.source:
                self.callback(line)
                if not self.log_failed and not (self.pause_file and self.pause_file.exists()):
                    try:self.log.write(line);self.log.flush()
                    except OSError as exc:
                        self.log_failed=True
                        self.callback('FIELDWORK_NOTICE '+json.dumps(dict(code='recording_io_error',message='进程日志写入失败：'+str(exc)))+'\n')
        except Exception as exc:self.error=exc
        finally:self.source.close()
    def join(self):self.thread.join(timeout=2.)


def run_process(command,environment,log,*,timeout_s=60,recorder=None,timeout_signal=signal.SIGINT,stop_timeout_s=10,cancel_check=None,on_output=None, recording_pause_file=None):
    process=None;interruption=None;pump=None
    with Path(log).open('x') as stream:
        try:
            early=getattr(recorder,'startup_interruption',None) or ('raw_recorder_exited' if recorder is not None and recorder.poll() is not None else None) or (cancel_check() if cancel_check else None)
            if early:return ProcessResult(-int(signal.SIGINT),early,False)
            process=subprocess.Popen(command,env=environment,stdout=subprocess.PIPE if on_output else stream,
                stderr=subprocess.STDOUT,**(dict(text=True,bufsize=1) if on_output else {}))
            if on_output:pump=OutputPump(process.stdout,stream,on_output,recording_pause_file)
            deadline=time.monotonic()+timeout_s if timeout_s is not None else None
            try:
                while process.poll() is None:
                    if cancel_check is not None:
                        reason=cancel_check()
                        if reason:interruption=reason;break
                    if recorder is not None and recorder.poll() is not None:
                        interruption='raw_recorder_exited';break
                    if pump and pump.error:interruption='status_channel_failed';break
                    if deadline is not None and time.monotonic()>deadline:interruption='execution_timeout';break
                    time.sleep(.02)
            except KeyboardInterrupt:interruption='user_interrupt'
            if interruption and on_output and interruption not in ('operator_cancel','frontend_closed'):
                on_output('FIELDWORK_EVENT '+json.dumps(dict(code=interruption,phase='SUPERVISOR',action='end_control',
                    event_ns=time.monotonic_ns(),observations=[]))+'\n')
            if process.poll() is None:
                first=timeout_signal if interruption=='execution_timeout' else signal.SIGINT
                stop_process(process,first_signal=first,timeout_s=stop_timeout_s)
            if pump:pump.join()
            return ProcessResult(process.returncode,interruption)
        finally:
            if process is not None:stop_process(process,timeout_s=stop_timeout_s)
            if pump:pump.join()


@contextmanager
def recording_process(command,environment,log,ready_path,*,ready_timeout_s=10,stop_timeout_s=10,cancel_check=None):
    process=None
    with Path(log).open('x') as stream:
        try:
            process=subprocess.Popen(command,env=environment,stdout=stream,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+ready_timeout_s
            while not Path(ready_path).exists():
                reason=cancel_check() if cancel_check else None
                if reason:
                    process.startup_interruption=reason
                    break
                if process.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Raw recorder failed before executor started')
                time.sleep(.02)
            if process.poll() is not None:raise RuntimeError('Raw recorder exited before executor started')
            yield process
        finally:
            if process is not None:stop_process(process,timeout_s=stop_timeout_s)
