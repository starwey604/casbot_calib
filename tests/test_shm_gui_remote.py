"""The same remote protocol over stdio/Unix sockets, with private physical devices."""
import io
import json
import os
from pathlib import Path
import time

import pytest
from casbot_calib.shm_gui_client import RemoteClient
from casbot_calib.shm_gui_protocol import frame, json_frame, read_frame
from test_shm_fieldwork import NATIVE, ROOT

pytestmark=pytest.mark.skipif(not (NATIVE/'shm_gui_owner').exists(),reason='build native GUI owner')


def wait(client,test,timeout=8,tick=True):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if tick:client.tick()
        if test():return
        if client.failure:raise AssertionError(client.failure+': '+(client.root/'control_ssh.log').read_text())
        time.sleep(.01)
    raise AssertionError((client.latest,client.drain()))


@pytest.fixture
def client(tmp_path):
    c=RemoteClient(dict(output=str(tmp_path/'remote'),mock=True,executor=str(NATIVE/'shm_executor'),mock_config=str(ROOT/'configs/shm/gui_mock.json')),tmp_path/'local')
    c.connect()
    try:yield c
    finally:c.close()


def test_framing_truncation_and_bad_length():
    value={'type':'stop','id':1}
    kind,body=read_frame(io.BytesIO(json_frame(value)))
    assert kind==1 and json.loads(body)==value
    with pytest.raises(EOFError):read_frame(io.BytesIO(b'\x08\x00\x00\x00bad'))
    with pytest.raises(ValueError):read_frame(io.BytesIO(b'\xff\xff\xff\xff'))


def test_physical_remote_round_trip_and_main_loop_lease(client):
    c=client;wait(c,lambda:c.hello and c.ready and c.latest and c.latest['receive_ns'])
    assert c.ready['mock'] and c.ready['endpoint'].startswith('/casbot_calib_test_')
    assert c.latest['writes']==0
    c.send('start');wait(c,lambda:c.latest['phase']==3)
    fields={k:list(v) for k,v in c.latest['command'].items()};fields['position'][0]+=.005
    c.send('fields',**fields);wait(c,lambda:c.latest['revision']==1)
    assert c.latest['command']['position'][0]==pytest.approx(fields['position'][0],abs=1.1e-6)
    # Background readers keep running, but deliberately do not call GUI tick().
    wait(c,lambda:c.latest['phase']==-1,timeout=3,tick=False)
    assert c.latest['disable_ack']
    assert c.rtt_ms is not None
    c.close();root=Path(c.ready['workspace'])
    assert (root/'native/result.json').exists()
    assert json.loads((root/'mock/exit.json').read_text())['returncode']==0


def test_display_disconnect_causes_control_stop(client):
    c=client;wait(c,lambda:c.hello and c.latest and c.latest['receive_ns']);c.send('start');wait(c,lambda:c.latest['phase']==3)
    c.display.terminate();c.display.wait(timeout=2)
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:c.tick();time.sleep(.01)
    c.close()
    result=json.loads((Path(c.ready['workspace'])/'native/result.json').read_text())
    assert result['disable_ack']


def test_two_ssh_connections_never_share_controlmaster(tmp_path):
    c=RemoteClient({},tmp_path,host='nvidia@172.16.0.10',repo='/a repo')
    args=c.command('serve','--config','abc')
    assert '-T' in args and 'ControlPath=none' in args and 'ControlMaster=no' in args
    assert "'/a repo'" in args[-1] and '-X' not in args
    with pytest.raises(ValueError):RemoteClient({},tmp_path,host='test',repo='/repo',ssh_sockets=('same','same'))
