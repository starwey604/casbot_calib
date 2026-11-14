#!/usr/bin/env python3
"""Use only an owned private physical Mock; optionally over two SSH transports."""
import argparse
import json
from pathlib import Path
import time
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'python'))
from casbot_calib.shm_gui_client import RemoteClient


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host');p.add_argument('--repo',default='/home/nvidia/casbot_calib_shm');p.add_argument('--output',type=Path,required=True);p.add_argument('--remote-output',required=True);p.add_argument('--executor',default='install/shm-core/bin/shm_executor');p.add_argument('--control-socket');p.add_argument('--display-socket')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    c=RemoteClient(dict(output=a.remote_output,mock=True,executor=a.executor),a.output,host=a.host,repo=a.repo,ssh_sockets=(a.control_socket,a.display_socket));c.connect()
    phases=[]
    def wait(test,seconds=15,tick=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if tick:c.tick()
            if c.failure:raise RuntimeError(c.failure)
            if test():return
            time.sleep(.01)
        raise TimeoutError(str((c.latest,c.drain())))
    try:
        wait(lambda:c.hello and c.ready and c.latest and c.latest['receive_ns'])
        assert c.ready['mock'] and c.ready['endpoint'].startswith('/casbot_calib_test_')
        assert c.latest['writes']==0;phases.append('readonly_zero_writes')
        c.send('start');wait(lambda:c.latest['phase']==3)
        values={k:list(v) for k,v in c.latest['command'].items()};values['position'][0]+=.005
        c.send('fields',**values);wait(lambda:c.latest['revision']==1)
        assert abs(c.latest['command']['position'][0]-values['position'][0])<=1.1e-6
        phases.append('five_fields_enqueued');start=c.latest;time.sleep(.02)
        c.send('stop',control=c.control);wait(lambda:c.latest['phase']==-1)
        assert c.latest['disable_ack'];phases.append('normal_disable_ack')
        c.send('start');wait(lambda:c.latest['phase']==3)
        wait(lambda:c.latest['phase']==-1,seconds=3,tick=False)
        assert c.latest['disable_ack'];phases.append('GUI_stall_lease_disable_ack')
        result=dict(passed=True,scope='owned_private_mock',phases=phases,ready=c.ready,telemetry=start,after_stall=c.latest,rtt_ms=c.rtt_ms,received_bytes=c.bytes_received)
    finally:c.close()
    result['bridge_returncode']=c.process.returncode
    (a.output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ('telemetry','after_stall','ready')},ensure_ascii=False))

if __name__=='__main__':main()
