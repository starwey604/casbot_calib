"""Shared SDK process lifetime; callers retain scope, stop policy and result meaning."""
from contextlib import contextmanager
from dataclasses import dataclass
import json
from pathlib import Path
import signal
import subprocess
import time


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


def run_process(command,environment,log,*,timeout_s=60,recorder=None,timeout_signal=signal.SIGINT,stop_timeout_s=10):
    process=None;interruption=None
    with Path(log).open('x') as stream:
        try:
            process=subprocess.Popen(command,env=environment,stdout=stream,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+timeout_s
            try:
                while process.poll() is None:
                    if recorder is not None and recorder.poll() is not None:
                        interruption='raw_recorder_exited';break
                    if time.monotonic()>deadline:interruption='execution_timeout';break
                    time.sleep(.02)
            except KeyboardInterrupt:interruption='user_interrupt'
            if process.poll() is None:
                first=timeout_signal if interruption=='execution_timeout' else signal.SIGINT
                stop_process(process,first_signal=first,timeout_s=stop_timeout_s)
            return ProcessResult(process.returncode,interruption)
        finally:
            if process is not None:stop_process(process,timeout_s=stop_timeout_s)


@contextmanager
def recording_process(command,environment,log,ready_path,*,ready_timeout_s=10,stop_timeout_s=10):
    process=None
    with Path(log).open('x') as stream:
        try:
            process=subprocess.Popen(command,env=environment,stdout=stream,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+ready_timeout_s
            while not Path(ready_path).exists():
                if process.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Raw recorder failed before executor started')
                time.sleep(.02)
            if process.poll() is not None:raise RuntimeError('Raw recorder exited before executor started')
            yield process
        finally:
            if process is not None:stop_process(process,timeout_s=stop_timeout_s)
