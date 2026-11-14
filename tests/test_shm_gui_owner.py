"""Native GUI sessions exercise private SHM only, including independent leases."""
import csv
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time

import pytest
from casbot_calib.shm_data import URDF
from casbot_calib.shm_gui_protocol import read_frame, telemetry, SIZE
from test_shm_fieldwork import master, NATIVE, ROOT

pytestmark = pytest.mark.skipif(not (NATIVE/'shm_gui_owner').exists(), reason='build GUI owner')

class Owner:
    def __init__(self, name, root, *, background=True, budget=512):
        self.root=root;self.events=queue.Queue();self.latest=None;self.hello=None;self.id=0;self.control=0;self.error=None
        flags=['--hand-background', str(ROOT/'tests/fixtures/shm/command_slot_0.bin')] if background else []
        self.log=(root.parent/(root.name+'.log')).open('wb')
        self.p=subprocess.Popen([str(NATIVE/'shm_gui_owner'), '--shm', name,'--output',str(root), '--urdf',str(URDF),'--budget-mib',str(budget),*flags],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log)
        def read():
            try:
                while True:
                    kind, body=read_frame(self.p.stdout)
                    if kind==2:self.latest=telemetry(body)
                    else:
                        item=json.loads(body)
                        if item['type']=='hello':self.hello=item
                        self.events.put(item)
            except EOFError:pass
            except Exception as e:self.error=e
        self.reader=threading.Thread(target=read,daemon=True);self.reader.start()
    def send(self, type, **values):
        self.id+=1
        if type=='start':self.control=self.id
        data=dict(type=type,id=self.id,**values)
        self.p.stdin.write((json.dumps(data,allow_nan=False)+'\n').encode());self.p.stdin.flush()
        return self.id
    def wait(self, test, duration=3, renew=False):
        end=time.monotonic()+duration;last=0
        while time.monotonic()<end:
            assert self.error is None,self.error
            if test():return
            if self.p.poll() is not None:raise AssertionError(self.log.name+': '+Path(self.log.name).read_text())
            if renew and self.latest and self.latest['challenge']!=last:
                last=self.latest['challenge'];self.send('renew',control=self.control,challenge=last,seq=self.latest['seq'])
            time.sleep(.005)
        raise AssertionError((self.latest,list(self.events.queue)))
    def start(self):
        self.wait(lambda:self.hello and self.latest and self.latest['master_frame']>1)
        self.send('start');self.wait(lambda:self.latest['phase']==3,renew=True)
    def close(self):
        if self.p.poll() is None:
            try:self.send('quit')
            except BrokenPipeError:pass
            self.p.wait(timeout=5)
        self.reader.join(timeout=3);self.log.close()

@pytest.fixture
def owner(master,tmp_path):
    objects=[]
    def create(**kw):
        name,source=master();obj=Owner(name,tmp_path/f'gui{len(objects)}',**kw);objects.append(obj)
        return obj,source,name
    yield create
    for obj in objects:obj.close()


def test_protocol_and_readonly_zero_commands(owner):
    o,source,_=owner();o.wait(lambda:o.latest and o.latest['seq']>10)
    assert SIZE == 1696
    assert o.latest['phase']==-1 and o.latest['writes']==0
    assert source.joinpath('commands.bin').stat().st_size==0
    assert len(o.latest['joints'])==35 and len(o.latest['hands'])==2
    assert o.latest['q'][0]==pytest.approx(0.)
    o.close();assert o.root.joinpath('result.json').exists()


def test_repeated_controls_fields_quantization_and_stop(owner):
    o,_,_=owner();o.start();first=o.control
    fields={key:list(value) for key,value in o.latest['command'].items()}
    fields['position'][0]+=.001;fields['effort'][0]=.019
    o.send('fields',control=first,revision=1,**fields)
    o.wait(lambda:o.latest['revision']==1,renew=True)
    assert o.latest['command']['effort'][0]==.01
    assert o.latest['command']['kp'][0]==20
    for j in [0,11,26,28]:assert o.latest['joints'][j][0]&0x6f==0x21
    o.send('stop',control=first);o.wait(lambda:o.latest['phase']==-1)
    assert o.latest['disable_ack']
    writes=o.latest['writes'];time.sleep(.05);assert o.latest['writes']==writes
    o.start();assert o.control!=first
    o.send('fields',control=first,revision=100,**fields);time.sleep(.03)
    assert o.latest['revision']==0


def test_lease_expires_without_more_gui_ticks(owner):
    o,_,_=owner();o.start()
    o.wait(lambda:o.latest['phase']==-1,duration=2)
    assert o.latest['disable_ack']
    assert any('lease_expired' in x.get('reason','') for x in o.events.queue)
    writes=o.latest['writes'];time.sleep(.03);assert o.latest['writes']==writes


def test_eof_and_signal_disable_and_preserve_results(owner):
    o,_,_=owner();o.start();o.p.stdin.close();o.p.wait(timeout=3)
    result=json.loads((o.root/'result.json').read_text())
    assert result['disable_ack'] and result['reason']=='bridge_eof'
    rows=list(csv.DictReader((o.root/'feedback.csv').open()));assert int(rows[-1]['status12'])&0x6f==0x21


def test_invalid_target_keeps_last_revision_and_missing_background(owner):
    o,_,_=owner();o.start();fields={k:list(v) for k,v in o.latest['command'].items()}
    fields['position'][0]=999;o.send('fields',control=o.control,revision=1,**fields)
    time.sleep(.05);assert o.latest['revision']==0
    o2,_,_=owner(background=False);o2.wait(lambda:o2.latest is not None);o2.send('start');time.sleep(.05)
    assert o2.latest['writes']==0


def test_budget_stops_recording_not_control(owner):
    o,_,_=owner(budget=.02);o.start();o.wait(lambda:o.latest['recording_stopped'],renew=True)
    assert o.latest['phase']==3
    o.send('stop');o.wait(lambda:o.latest['phase']==-1);assert o.latest['disable_ack']


def test_exclusive_owner_refuses_second_reader(owner,tmp_path):
    o,_,name=owner();o.wait(lambda:o.latest is not None)
    p=subprocess.run([str(NATIVE/'shm_gui_owner'),'--shm',name,'--output',str(tmp_path/'second'),'--urdf',str(URDF)],capture_output=True,timeout=3)
    assert p.returncode==1 and b'Another fieldwork owner' in p.stderr
    assert not (tmp_path/'second').exists()


def test_old_challenge_cannot_keep_control_alive(owner):
    o,_,_=owner();o.start();old=dict(control=o.control,challenge=o.latest['challenge'],seq=o.latest['seq'])
    o.send('renew',**old);time.sleep(.03)
    end=time.monotonic()+1.4
    while time.monotonic()<end:
        o.send('renew',**old);time.sleep(.03)
    o.wait(lambda:o.latest['phase']==-1)
    assert o.latest['disable_ack']


def test_drive_error_ends_control_without_reenable(master,tmp_path):
    name,_=master(fault=15);o=Owner(name,tmp_path/'fault')
    try:
        o.wait(lambda:o.latest and o.latest['master_frame']>1);o.send('start')
        o.wait(lambda:o.latest['phase']==-1 and o.latest['writes']>10,renew=True)
        assert o.latest['disable_ack']
        writes=o.latest['writes'];o.send('start');time.sleep(.05)
        assert o.latest['writes']==writes
    finally:o.close()
