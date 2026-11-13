"""Own one private SHM physical process. No ROS, privileged commands or hardware fallback."""
import json
import os
from pathlib import Path
import struct
import subprocess
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]


class SHMMock:
    def __init__(self, root, executor, config=ROOT/'configs/shm/mock.json', *, urdf=None):
        self.root=Path(root).resolve();self.executor=Path(executor).resolve()
        self.source=Path(config).resolve();self.override_urdf=urdf
        self.endpoint=f'/casbot_calib_test_physics_{os.getpid()}_{uuid.uuid4().hex[:10]}'
        self.process=None;self.log=None;self.closed=False;self.ipc_inodes={}

    def __enter__(self):
        self.root.mkdir(parents=True,exist_ok=False)
        config=json.loads(self.source.read_text())
        if config.get('schema')!='casbot_shm_mock_v1':
            raise ValueError('SHM physics config required')
        model=Path(config['urdf'])
        config['urdf']=str(Path(self.override_urdf).resolve() if self.override_urdf else (self.source.parent/model).resolve())
        (self.root/'config.json').write_text(json.dumps(config,indent=2)+'\n')
        hands=bytearray(744);struct.pack_into('<H',hands,10,2)
        self.background=self.root/'hand_background.bin';self.background.write_bytes(hands)
        self.log=(self.root/'server.log').open('wb')
        try:
            self.process=subprocess.Popen([str(self.executor.with_name('shm_mock_server')), '--shm',self.endpoint,
                '--config',str(self.root/'config.json'),'--output',str(self.root/'server')],stdout=self.log,stderr=subprocess.STDOUT)
            end=time.monotonic()+10
            while not (self.root/'server/ready.txt').exists():
                if self.process.poll() is not None:
                    raise RuntimeError('SHM physics process failed: '+(self.root/'server.log').read_text())
                if time.monotonic()>end:
                    raise TimeoutError('SHM physics startup timed out')
                time.sleep(.01)
            for suffix in ('', '_front', '_back', '_start'):
                path=Path('/dev/shm'+self.endpoint) if not suffix else Path('/dev/shm/sem.'+self.endpoint[1:]+suffix)
                st=path.stat();self.ipc_inodes[path]=(st.st_dev,st.st_ino)
            subprocess.run([str(self.executor.with_name('shm_channel_start')),'--fresh-master',self.endpoint],check=True,
                           stdout=self.log,stderr=self.log,timeout=3)
            return self
        except BaseException:
            self.close();raise

    def close(self):
        if self.closed:return
        errors=[]
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=5)
            # A killed master cannot run its destructors. Remove only the exact
            # private objects observed after our own successful O_EXCL startup.
            for path,inode in self.ipc_inodes.items():
                try:
                    st=path.stat()
                    if (st.st_dev,st.st_ino)==inode:path.unlink()
                except FileNotFoundError:pass
                except OSError as error:errors.append(str(error))
            (self.root/'exit.json').write_text(json.dumps(dict(returncode=self.process.returncode,endpoint=self.endpoint,cleanup_errors=errors))+'\n')
        if self.log is not None:
            self.log.close();self.log=None
        self.closed=True

    def __exit__(self,*_):
        self.close()
