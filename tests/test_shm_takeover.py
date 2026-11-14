"""Only fake hosts and owned subprocesses; never services, real master or SHM."""
import json
from pathlib import Path
import subprocess
import sys
import time
import pytest
from casbot_calib import shm_takeover as takeover
from casbot_calib import shm_fieldwork as fw


class FakeHost:
    def __init__(self,root,*,stuck=False,deactivating=False):
        self.vendor=root/'vendor';self.vendor.mkdir()
        for name in ('launcher.bash','run_ethercat_station.bash'):(self.vendor/name).touch()
        self.states={unit:'active' for unit in takeover.SERVICES}
        if deactivating:self.states[takeover.SERVICES[0]]='deactivating'
        self.master=dict(pid=321,name='ethercat_statio',start_ticks='123',threads=[dict(tid=321,state='Z'),dict(tid=322,state='D')])
        self.events=[];self.stuck=stuck
    def services(self):return self.states.copy()
    def processes(self):return [] if self.master is None else [self.master]
    def terminate_master(self,p):
        self.events.append('master_sigterm')
        if not self.stuck:self.master=None
    def diagnostics(self,root):
        self.events.append('diagnosis');(root/'diagnosis.json').write_text(json.dumps(dict(services=self.states,processes=self.processes())))
    def start(self,root):self.events.append('start');assert self.master is None and all(s=='inactive' for s in self.states.values())
    def run(self,cmd,path,**kw):
        path.touch()
        if cmd[-1]=='stop':
            assert self.master is None;self.events.append('vendor_stop');self.states={u:'inactive' for u in self.states}
        return 0


def test_prepare_graceful_exit_precedes_vendor_hard_kill_and_start(tmp_path):
    host=FakeHost(tmp_path);out=tmp_path/'takeover'
    assert takeover.prepare(out,host=host,runner=host.run)==0
    assert host.events==['master_sigterm','vendor_stop','start']
    assert json.loads((out/'prepare_result.json').read_text())['station_start_requested']


def test_existing_stop_job_fails_without_signalling_or_starting(tmp_path):
    host=FakeHost(tmp_path,deactivating=True);out=tmp_path/'takeover'
    assert takeover.prepare(out,host=host,runner=lambda *a,**k:pytest.fail('must not run stop'))==2
    assert host.events==['diagnosis']
    assert not json.loads((out/'prepare_result.json').read_text())['station_start_requested']


def test_zombie_leader_with_D_thread_is_not_stopped(tmp_path,monkeypatch):
    host=FakeHost(tmp_path,stuck=True);out=tmp_path/'takeover'
    original=takeover.graceful_master
    monkeypatch.setattr(takeover,'graceful_master',lambda h,r:original(h,r,timeout=.001))
    assert takeover.prepare(out,host=host,runner=host.run)==2
    assert host.events==['master_sigterm','diagnosis']
    assert not (out/'stop.log').exists()


def test_success_message_does_not_replace_service_exit(tmp_path):
    host=FakeHost(tmp_path,deactivating=True);host.master=None
    with pytest.raises(takeover.HandoffError,match='尚未退出'):takeover.wait_stopped(host,tmp_path,timeout=.001)
    assert 'start' not in host.events


def test_diagnosis_only_never_runs_stop_or_signal(tmp_path):
    host=FakeHost(tmp_path,deactivating=True)
    assert takeover.prepare(tmp_path/'read_only',host=host,diagnose=True,runner=lambda *a,**k:pytest.fail('stop forbidden'))==0
    assert host.events==['diagnosis']


def test_start_only_keeps_old_stop_log_and_checks_old_process(tmp_path,monkeypatch):
    host=FakeHost(tmp_path,stuck=True);host.states={u:'inactive' for u in host.states}
    out=tmp_path/'takeover';out.mkdir();(out/'stop.log').write_text('previous stop log')
    original=takeover.wait_stopped
    monkeypatch.setattr(takeover,'wait_stopped',lambda h,r:original(h,r,timeout=.001))
    assert takeover.prepare(out,start_only=True,host=host,runner=lambda *a,**k:pytest.fail('no new stop'))==2
    assert host.events==['diagnosis'] and (out/'stop.log').read_text()=='previous stop log'


def test_run_logged_does_not_wait_for_inherited_stdout(tmp_path):
    # Vendor-style background child keeps stdout open after its parent exits.
    code='import subprocess,sys; subprocess.Popen([sys.executable,"-c","import time; time.sleep(2)"]); print("Service stopped successfully.",flush=True)'
    begin=time.monotonic()
    assert takeover.run_logged([sys.executable,'-c',code],tmp_path/'stop.log',timeout=.8)==0
    assert time.monotonic()-begin<.8
    assert 'Service stopped successfully.' in (tmp_path/'stop.log').read_text()


def test_stop_helper_timeout_is_bounded(tmp_path):
    with pytest.raises(takeover.HandoffError,match='超时'):
        takeover.run_logged([sys.executable,'-c','import time; time.sleep(10)'],tmp_path/'stop.log',timeout=.02)


def test_prepare_interrupt_is_recorded_without_traceback(tmp_path,monkeypatch):
    class Child:
        pid=321
        def __init__(self):self.interrupted=False
        def wait(self,timeout=None):
            if not self.interrupted:raise KeyboardInterrupt
            return 130
        def poll(self):return None
        def send_signal(self,sig):self.interrupted=True;assert sig==2
    child=Child();monkeypatch.setattr(fw.subprocess,'Popen',lambda *a,**kw:child)
    out=tmp_path/'setup'
    assert fw.main(['prepare','--execute','--output',str(out)])==130
    assert json.loads((out/'process.json').read_text())['operator_interrupted']
    assert not json.loads((out/'result.json').read_text())['execution_completed']


def test_proc_scan_reads_sibling_D_even_if_leader_is_zombie(tmp_path):
    proc=tmp_path/'proc';folder=proc/'321';(folder/'task/321').mkdir(parents=True);(folder/'task/322').mkdir()
    (folder/'comm').write_text('ethercat_statio\n')
    (folder/'stat').write_text('321 (ethercat_statio) '+' '.join(['Z']+['0']*18+['123']))
    (folder/'task/321/status').write_text('State:\tZ (zombie)\n')
    (folder/'task/322/status').write_text('State:\tD (disk sleep)\n')
    processes=takeover.Host(proc=proc).processes()
    assert len(processes)==1 and processes[0]['start_ticks']=='123'
    assert takeover.alive(processes[0])
    assert {t['tid']:t['state'] for t in processes[0]['threads']}[322]=='D'
    assert not takeover.alive(dict(threads=[dict(tid=321,state='Z')]))
