"""SSH stdio bridge: one native owner, a private local telemetry subscription."""
import argparse
import base64
from contextlib import ExitStack
import json
import os
from pathlib import Path
import queue
import select
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

from .shm_data import URDF, NAMES
from .shm_gui_protocol import frame, json_frame, read_frame, read_exact


def encode_config(config):
    return base64.urlsafe_b64encode(json.dumps(config, allow_nan=False).encode()).decode()


def model_identity(urdf):
    metadata=json.loads(Path(urdf).with_name('model.json').read_text())
    return dict(schema_version='casbot_model_context_v1', **{k:metadata[k] for k in ('model_id','bundle_sha256','urdf_sha256')})


def serve(config):
    root=Path(config['output']).resolve();root.mkdir(parents=True,exist_ok=False)
    os.chmod(root,0o700)
    executor=Path(config.get('executor','install/shm-core/bin/shm_executor')).resolve()
    urdf=Path(config.get('urdf',URDF)).resolve()
    identity=model_identity(urdf)
    mock=None;native=None;done=threading.Event();output_done=threading.Event();outgoing=queue.Queue(128)
    latest=[None,0];condition=threading.Condition();subscribers=[];listener=None
    def emit(item):
        try:outgoing.put_nowait(json_frame(item))
        except queue.Full:done.set()
    def signal_stop(*_):done.set()
    for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP):signal.signal(sig,signal_stop)
    with ExitStack() as resources:
        log=resources.enter_context((root/'bridge.log').open('wb'))
        input_stream=resources.enter_context(os.fdopen(os.dup(sys.stdin.fileno()),'rb',buffering=0))
        try:
            if config.get('mock'):
                from .shm_mock import SHMMock
                mock=resources.enter_context(SHMMock(root/'mock',executor,config.get('mock_config','configs/shm/gui_mock.json'),urdf=urdf))
                endpoint=mock.endpoint;background=mock.background
            else:
                endpoint=config.get('endpoint','/shm_mc');background=config.get('hand_background')
            arguments=[str(executor.with_name('shm_gui_owner')),'--shm',endpoint,'--output',str(root/'native'), '--urdf',str(urdf),'--budget-mib',str(config.get('budget_mib',512))]
            if background:arguments+=['--hand-background',str(Path(background).resolve())]
            native=subprocess.Popen(arguments,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log)
            token=resources.enter_context(tempfile.TemporaryDirectory(prefix='casbot_gui_'))
            os.chmod(token,0o700);path=str(Path(token)/'telemetry.sock')
            listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);listener.bind(path);listener.listen(1);listener.settimeout(.1)
            def write_control():
                while not output_done.is_set() or not outgoing.empty():
                    try:packet=outgoing.get(timeout=.05)
                    except queue.Empty:continue
                    try:sys.stdout.buffer.write(packet);sys.stdout.buffer.flush()
                    except (OSError,ValueError):done.set();break
            def read_native():
                try:
                    while True:
                        kind,body=read_frame(native.stdout)
                        if kind==1:emit(json.loads(body))
                        else:
                            with condition:latest[:]=[frame(2,body),latest[1]+1];condition.notify_all()
                except EOFError:pass
                except Exception as error:emit(dict(type='error',reason=str(error)))
                finally:
                    if native.poll() is None:native.wait()
                    if native.returncode:emit(dict(type='error',reason='原生退出 '+str(native.returncode)+'；日志 '+str(root/'bridge.log')))
                    done.set()
            def subscribe():
                # Exactly one display connection. A stalled writer cannot block native readout.
                while not done.is_set():
                    try:connection,_=listener.accept()
                    except socket.timeout:continue
                    except OSError:break
                    subscribers.append(connection);connection.settimeout(.5);sequence=-1
                    try:
                        with connection:
                            while not done.is_set():
                                with condition:
                                    condition.wait_for(lambda:done.is_set() or latest[1]!=sequence,timeout=.1)
                                    packet,sequence=latest
                                if packet:connection.sendall(packet)
                    except OSError:pass
                    finally:subscribers.remove(connection)
            threads=[threading.Thread(target=f,daemon=True) for f in (write_control,read_native,subscribe)]
            for thread in threads:thread.start()
            emit(dict(type='remote_ready',version=1,workspace=str(root),socket=path,
                      model_context=identity,joint_names=NAMES,
                      boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                      endpoint=endpoint,mock=bool(mock),non_arm='disabled_during_control',hands='explicit_frozen_background'))
            (root/'session.json').write_text(json.dumps(dict(schema_version='casbot_shm_gui_workspace_v1',config=config,model_context=identity,joint_names=NAMES,endpoint=endpoint,boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()),indent=2)+'\n')
            # Framing can block on a partial input. The owner lease remains independent.
            while not done.is_set():
                ready,_,_=select.select([input_stream],[],[],.1)
                if not ready:continue
                try:kind,body=read_frame(input_stream)
                except EOFError:break
                if kind!=1:raise ValueError('control_requires_json')
                item=json.loads(body)
                native.stdin.write((json.dumps(item,allow_nan=False,separators=(',',':'))+'\n').encode());native.stdin.flush()
        except Exception as error:
            emit(dict(type='error',reason=str(error)))
            if not locals().get('threads'):
                sys.stdout.buffer.write(json_frame(dict(type='error',reason=str(error))));sys.stdout.buffer.flush()
        finally:
            if native:
                if native.stdin and not native.stdin.closed:native.stdin.close()
                try:native.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    native.terminate()
                    try:native.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        # Never SIGKILL a hardware sender; leave diagnostics and ownership intact.
                        log.write(b'Native sender did not exit; do not restart or steal SHM owner\n');log.flush()
                (root/'process.json').write_text(json.dumps(dict(returncode=native.poll(),ended_mono_ns=time.monotonic_ns()))+'\n')
            done.set();output_done.set()
            with condition:condition.notify_all()
            if listener:listener.close()
            for connection in list(subscribers):
                try:connection.shutdown(socket.SHUT_RDWR)
                except OSError:pass
            for thread in locals().get('threads',[]):thread.join(timeout=1)
    return 0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    sub.add_parser('serve').add_argument('--config',required=True)
    sub.add_parser('subscribe').add_argument('--socket',required=True)
    args=parser.parse_args(argv)
    if args.action=='serve':return serve(json.loads(base64.urlsafe_b64decode(args.config)))
    path=Path(args.socket)
    if path.name!='telemetry.sock' or not path.parent.name.startswith('casbot_gui_') or path.parent.parent!=Path(tempfile.gettempdir()):
        raise ValueError('private_subscription_socket_required')
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
        connection.connect(str(path))
        while True:
            packet=connection.recv(65536)
            if not packet:break
            sys.stdout.buffer.write(packet);sys.stdout.buffer.flush()
    return 0


if __name__=='__main__':raise SystemExit(main())
