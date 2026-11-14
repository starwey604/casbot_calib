"""One startup policy across field transports, H1--H4, K and manual tools."""
import csv
import json
from pathlib import Path
import re
import struct
import numpy as np
import pytest
from casbot_calib.field_defaults import START_VELOCITY_RAD_S,start_velocity_limit
from casbot_calib.field_artifacts import seal
from casbot_calib.shm_data import ROOT




@pytest.mark.parametrize('value',[0,-1,float('nan'),float('inf'),True])
def test_invalid_startup_threshold(value):
    with pytest.raises(ValueError):start_velocity_limit(dict(start_velocity_rad_s=value))


@pytest.mark.parametrize('speed,allowed',[(.1,True),(.199999,True),(.200001,False),(.21,False)])
def test_shm_measured_start_uses_shared_threshold(tmp_path,speed,allowed):
    from casbot_calib.shm_h1 import load_profile,DEFAULT_PROFILE,measured_start
    c=load_profile(DEFAULT_PROFILE);root=tmp_path/'snapshot';(root/'native').mkdir(parents=True)
    raw=bytearray(696);struct.pack_into('<i',raw,12+12*16+6,round(speed*1e6))
    (root/'native/status.bin').write_bytes(raw)
    (root/'h1_snapshot.json').write_text(json.dumps(dict(schema_version='casbot_shm_h1_snapshot_v1',model_context=c['model_context'])))
    seal(root)
    if allowed:assert len(measured_start(root,c)[0])==14
    else:
        with pytest.raises(ValueError,match='0.2rad/s'):measured_start(root,c)




@pytest.mark.parametrize('speed,starts',[(.1,True),(.2,False)])
def test_gui_start_uses_same_default_without_graphics(tmp_path,speed,starts):
    from types import SimpleNamespace
    from casbot_calib.shm_gui import Desk,parser
    root=tmp_path/'gui';root.mkdir();desk=Desk(root,parser().parse_args([]),render=False);calls=[]
    try:
        desk.client=SimpleNamespace(hello=dict(version=2,hand_background=True,lower=[-3.]*14,upper=[3.]*14,velocity=[1.5]*14),
            failure=None,send=lambda *args,**kw:calls.append((args,kw)))
        desk.model_ok=True;desk.feedback=dict(receive_ns=1,phase=-1,q=[0.]*14,dq=[speed]*14)
        desk.commands=dict(position=np.zeros(14),velocity=np.zeros(14),effort=np.zeros(14),kp=np.full(14,20.),kd=np.full(14,2.))
        desk.start()
        assert bool(calls)==starts and desk.start_waiting==(not starts)
        if not starts:assert '0.2rad/s' in desk.message
    finally:desk.client=None;desk.close()
