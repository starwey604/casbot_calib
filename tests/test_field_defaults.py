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


def test_python_native_and_all_new_profile_defaults_agree():
    from casbot_calib.shm_h1 import load_profile as shm_profile,DEFAULT_PROFILE
    from casbot_calib.hardware_field import load_profile,params
    from casbot_calib.fieldwork_settings import GLOBAL_DEFAULTS
    from casbot_calib.shm_gui import SETTINGS
    from casbot_calib.shm_data import NAMES
    header=(ROOT/'cpp/include/casbot_calib/field_defaults.hpp').read_text()
    assert float(re.search(r'field_start_velocity_rad_s\s*=\s*([\d.]+)',header)[1])==START_VELOCITY_RAD_S==.2
    assert GLOBAL_DEFAULTS['start_velocity']==SETTINGS['start_velocity_rad_s']==.2
    assert shm_profile(DEFAULT_PROFILE)['start_velocity_rad_s']==.2
    for path in ('configs/hardware_field.yaml','configs/dvt2/hardware_field.yaml'):
        c=load_profile(ROOT/path)
        assert c['start_velocity_rad_s']==.2
        for manual in (False,True):assert params(c,list(NAMES),manual=manual)['start_velocity_rad_s']==.2
    assert start_velocity_limit({'start_velocity_rad_s':.03})==.03


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


@pytest.mark.parametrize('speed,allowed',[(.1,True),(.2,False)])
def test_sdk_snapshot_shared_by_h1_h4_and_k_uses_same_threshold(tmp_path,speed,allowed):
    from casbot_calib.hardware_field import _read_snapshot
    root=tmp_path/'snapshot';(root/'execution').mkdir(parents=True)
    (root/'result.json').write_text(json.dumps(dict(passed=True,snapshot_only=True)))
    (root/'request.json').write_text(json.dumps(dict(config=dict(start_velocity_rad_s=.2))))
    with (root/'execution/snapshot.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow([*[f'q{j}' for j in range(14)],*[f'v{j}' for j in range(14)]])
        writer.writerow([*np.zeros(14),speed,*np.zeros(13)])
    seal(root)
    if allowed:np.testing.assert_array_equal(_read_snapshot(root),0)
    else:
        with pytest.raises(ValueError,match='stationary'):_read_snapshot(root)


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
