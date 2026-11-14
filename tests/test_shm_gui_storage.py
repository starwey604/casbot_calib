"""Snapshot consistency, read-only recovery and zero planning without rendering."""
import csv
import json
from pathlib import Path
import time

import numpy as np
import pytest
from casbot_calib.shm_gui_storage import snapshot,load_snapshot,last_feedback
from casbot_calib.shm_gui_archive import save_workspace
from casbot_calib.shm_gui_zero import plan_zero,validate_zero
from casbot_calib.shm_gui import Desk,parser
from test_shm_gui_remote import client,wait
from test_shm_gui_owner import owner,Owner
from test_shm_fieldwork import master


def test_active_save_matches_csv_raw_and_recovery_is_readonly(client,tmp_path):
    c=client;wait(c,lambda:c.hello and c.latest and c.latest['receive_ns']);c.send('start');wait(c,lambda:c.latest['phase']==3)
    saved=save_workspace(c,tmp_path/'saved')
    assert c.latest['phase']==3
    manifest=load_snapshot(saved['output']);root=Path(saved['output'])
    for name,width in (('feedback',704),('commands',752)):
        rows=list(csv.DictReader((root/'native'/f'{name}.csv').open()))
        assert len(rows)==manifest['rows'][name]
        assert (root/'native'/f'{name}.bin').stat().st_size==len(rows)*width
    assert not manifest['complete_session']
    local=tmp_path/'recover';local.mkdir()
    args=parser().parse_args(['--recover',str(root)])
    desk=Desk(local,args,render=False)
    try:
        assert desk.archive and desk.client is None and np.isfinite(desk.q).all()
        desk.connect();assert desk.client is None
    finally:desk.close()
    c.send('stop');wait(c,lambda:c.latest['phase']==-1)
    metadata=[json.loads(x)['message'] for x in (root/'native/controls.jsonl').read_text().splitlines()]
    assert any(x.get('type')=='start' for x in metadata)


def test_snapshot_truncates_incomplete_tail_and_rejects_manifest_changes(client,tmp_path):
    c=client;wait(c,lambda:c.ready and c.latest and c.latest['seq']>15)
    c.close();source=Path(c.ready['workspace']);raw=source/'native/feedback.bin'
    with raw.open('ab') as stream:stream.write(b'partial')
    table=source/'native/feedback.csv'
    with table.open('ab') as stream:stream.write(b'partial')
    out=tmp_path/'prefix';snapshot(source,out);load_snapshot(out)
    last_feedback(out/'native/feedback.csv')
    (out/'extra').write_text('not inventoried')
    with pytest.raises(ValueError,match='清单'):load_snapshot(out)


def test_mock_zero_is_local_to_its_owned_workspace(client):
    c=client;wait(c,lambda:c.ready and c.latest and c.latest['receive_ns'])
    request=c.send('zero_capture');messages=[]
    def got_zero():
        messages.extend(c.drain());return any(m.get('type')=='zero' and m['id']==request for m in messages)
    wait(c,got_zero)
    value=next(m['value'] for m in messages if m.get('type')=='zero')
    target=validate_zero(value,c.ready['model_context'],True)
    assert len(target)==14 and c.latest['writes']==0
    assert (Path(c.ready['workspace'])/'zero.yaml').exists()
    with pytest.raises(ValueError):validate_zero(value,c.ready['model_context'],False)


def test_zero_continuous_collision_contract_and_native_execution(owner,tmp_path):
    o,_,_=owner();o.wait(lambda:o.hello and o.latest and o.latest['master_frame']>1)
    q=np.array(o.latest['q']);target=q.copy();target[0]+=.001
    result=plan_zero(q,target,[20.]*14,[2.]*14,duration_s=1.)
    assert result['certificate']['passed']
    assert result['bundle']['effort']==[dict(t_ns=0,value=[0.]*14)]
    reference=tmp_path/'reference.json';reference.write_text(json.dumps(result['bundle']))
    o.send('trajectory',reference=str(reference));o.wait(lambda:o.latest['phase']==3,renew=True)
    # Independent-field input is rejected while a frozen reference is running.
    fields={k:list(v) for k,v in o.latest['command'].items()};o.send('fields',control=o.control,revision=1,**fields)
    o.wait(lambda:o.latest['phase']==-1,duration=6,renew=True)
    assert o.latest['disable_ack']
    assert any(x.get('reason')=='disabled_reference_completed' for x in o.events.queue)
    assert abs(o.latest['q'][0]-target[0])<2e-6


def test_zero_rejects_urdf_invalid_target_before_sending():
    q=np.zeros(14);q[1]=np.deg2rad(20);q[8]=-np.deg2rad(20)
    target=q.copy();target[0]=100
    with pytest.raises(ValueError,match='检查未通过'):plan_zero(q,target,[20.]*14,[2.]*14)


def test_shm_video_reader_uses_native_receive_column(tmp_path):
    from casbot_calib.fieldwork_streaming import VideoFeedback
    path=tmp_path/'feedback.csv'
    path.write_text('receive_mono_ns,'+','.join('q'+str(j) for j in range(14))+'\n'+
                    '100000000000,'+','.join(['0']*14)+'\n'+
                    '100050000000,'+','.join(['1']*14)+'\n')
    rows=list(VideoFeedback(path,receive_column='receive_mono_ns').frames())
    assert rows[0][2]==[0.]*14 and rows[-1][2]==[1.]*14


def test_zero_bridge_get_and_submit_share_same_private_owner(client,tmp_path):
    c=client;wait(c,lambda:c.ready and c.hello and c.latest and c.latest['receive_ns'])
    c.send('zero_capture');messages=[]
    def captured():
        messages.extend(c.drain());return any(x.get('type')=='zero' for x in messages)
    wait(c,captured)
    assert (Path(c.ready['workspace'])/'zero.yaml').exists()
    # Capture itself is read-only; a malformed certificate must not request control.
    c.send('zero_return',bundle=dict(purpose='wrong'),certificate=dict(passed=False),model_context=c.ready['model_context'])
    time.sleep(.05);assert c.latest['writes']==0


def test_cancelled_save_keeps_source_and_retryable_prefix(client,tmp_path):
    c=client;wait(c,lambda:c.ready and c.latest and c.latest['seq']>10)
    before=Path(c.ready['workspace'])/'native/feedback.bin'
    assert before.exists()
    with pytest.raises(ValueError,match='取消'):save_workspace(c,tmp_path/'cancelled',cancel=lambda:True)
    assert before.exists() and not (tmp_path/'cancelled').exists()


def test_video_failure_does_not_publish_final_file(client,tmp_path,monkeypatch):
    c=client;wait(c,lambda:c.ready and c.latest and c.latest['seq']>20);c.close()
    saved=save_workspace(c,tmp_path/'saved_for_video')['output']
    import casbot_calib.fieldwork_view as view
    import casbot_calib.shm_gui_archive as archive
    class FakeView:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def render(self,*args):raise RuntimeError('injected_render_failure')
    class FakeEncoder:
        def send(self,*args):pass
        def close(self):pass
    import imageio_ffmpeg
    monkeypatch.setattr(view,'open_view',lambda *a,**kw:FakeView())
    monkeypatch.setattr(view,'frame',lambda *a,**kw:None)
    monkeypatch.setattr(imageio_ffmpeg,'write_frames',lambda *a,**kw:FakeEncoder())
    output=tmp_path/'failed.mp4'
    with pytest.raises(RuntimeError,match='injected'):archive.export_video(saved,output)
    assert not output.exists() and not output.with_name('failed.pending.mp4').exists()
    cancelled=tmp_path/'cancelled.mp4'
    with pytest.raises(ValueError,match='中断|取消'):archive.export_video(saved,cancelled,cancel_check=lambda:'operator_cancel')
    assert not cancelled.exists() and not cancelled.with_name('cancelled.pending.mp4').exists()


def test_bridge_zero_reference_full_protocol_with_explicit_zero_gravity_fixture(tmp_path):
    from casbot_calib.shm_gui_client import RemoteClient
    from test_shm_fieldwork import ROOT,NATIVE
    config=json.loads((ROOT/'configs/shm/gui_mock.json').read_text())
    config['urdf']=str((ROOT/'configs/shm'/config['urdf']).resolve());config['gravity_world']=[0,0,0]
    config['initial_fields']['effort']=[0.]*14;config['purpose']='zero_protocol_only_explicit_zero_gravity_fixture'
    path=tmp_path/'fixture.json';path.write_text(json.dumps(config))
    c=RemoteClient(dict(output=str(tmp_path/'remote_zero'),mock=True,executor=str(NATIVE/'shm_executor'),mock_config=str(path)),tmp_path/'zero_local');c.connect()
    try:
        wait(c,lambda:c.ready and c.hello and c.latest and c.latest['receive_ns'])
        q=list(c.latest['q']);target=list(q);target[0]+=.001
        planned=plan_zero(q,target,[20.]*14,[2.]*14,duration_s=1.)
        c.send('zero_return',**planned);wait(c,lambda:c.latest['phase']==3)
        wait(c,lambda:c.latest['phase']==-1,timeout=5)
        assert c.latest['disable_ack'] and abs(c.latest['q'][0]-target[0])<.001
        c.close();result=json.loads((Path(c.ready['workspace'])/'native/result.json').read_text())
        assert result['reason']=='reference_completed'
    finally:c.close()
