"""Disk cache lifetime, saved prefixes and additive launch contracts."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

import pytest

from casbot_calib.fieldwork_storage import new_cache,cache_for,cached_sessions,recover_workspace,CacheMonitor,save_snapshot
from casbot_calib.fieldwork_launch import prepare_gui
from casbot_calib.fieldwork_cli import parser
from casbot_calib.sdk_process import OutputPump


def workspace(cache):
    root=cache/'workspace';(root/'protocol').mkdir(parents=True)
    (root/'protocol/session.json').write_text('{"fixture":true}')
    return root


def test_auto_cache_is_unique_recoverable_and_not_deleted(tmp_path,monkeypatch):
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path))
    first=new_cache();second=new_cache();root=workspace(first)
    assert first!=second and cache_for(root)==first
    assert cached_sessions()[0]['workspace']==str(root)
    assert recover_workspace(first.name)==root
    assert recover_workspace()==root
    assert first.exists() and second.exists()


def test_cache_quota_latches_without_deleting_existing_data(tmp_path,monkeypatch):
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path));cache=new_cache()
    root=workspace(cache);data=root/'log';data.write_bytes(b'x'*300)
    monitor=CacheMonitor(cache,interval_s=.01)
    try:
        monitor.limit=1;monitor.check()
        assert monitor.paused and data.read_bytes()==b'x'*300
        assert json.loads((cache/'recording_paused.json').read_text())['recording_complete'] is False
    finally:monitor.close()


def test_live_snapshot_trims_partial_lines_and_unreferenced_cdr(tmp_path):
    root=tmp_path/'source';raw=root/'observations/one/capture/raw';raw.mkdir(parents=True)
    packet=dict(cdr_offset=0,cdr_size=3,message={})
    (raw/'packets.jsonl').write_text(json.dumps(packet)+'\n'+json.dumps(dict(cdr_offset=3,cdr_size=4))[:10])
    (raw/'cdr.bin').write_bytes(b'abcdefgTAIL')
    (root/'commands.csv').write_bytes(b'time,q\n1,.2\n2,')
    (root/'latest.csv').write_text('not archived')
    (root/'.pending').mkdir();(root/'.pending/unfinished.json').write_text('{')
    (root/'view').mkdir();(root/'view/large_scene').write_text('not archived')
    target=tmp_path/'saved';before=(raw/'cdr.bin').read_bytes()
    result=save_snapshot(root,target)
    assert result['files']==3
    assert (target/'commands.csv').read_bytes()==b'time,q\n1,.2\n'
    assert (target/'observations/one/capture/raw/cdr.bin').read_bytes()==b'abc'
    assert not (target/'latest.csv').exists() and not (target/'view').exists()
    assert (raw/'cdr.bin').read_bytes()==before
    report=json.loads((target/'saved_session.json').read_text())
    assert report['read_only'] and report['copy_complete'] and report['file_cuts']['commands.csv']['bytes']==len(b'time,q\n1,.2\n')
    with pytest.raises(FileExistsError):save_snapshot(root,target)
    with pytest.raises(ValueError):save_snapshot(root,root/'nested')


def test_bad_cdr_is_not_published_as_a_complete_archive(tmp_path):
    root=tmp_path/'source';root.mkdir();(root/'packets.jsonl').write_text('{"cdr_offset":0,"cdr_size":10}\n')
    (root/'cdr.bin').write_bytes(b'short')
    target=tmp_path/'failed'
    with pytest.raises(ValueError,match='CDR'):save_snapshot(root,target)
    assert (target/'saving.json').exists() and not (target/'saved_session.json').exists()


def test_paused_cache_snapshot_marks_capacity_and_keeps_recovery(tmp_path,monkeypatch):
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path/'cache'))
    cache=new_cache();root=workspace(cache);(cache/'recording_paused.json').write_text('{}')
    target=tmp_path/'saved';save_snapshot(root,target)
    assert json.loads((target/'saved_session.json').read_text())['recording_capacity_reached']
    assert json.loads((cache/'last_saved.json').read_text())['destination']==str(target)
    assert recover_workspace(cache.name)==root


def test_preview_no_output_uses_cache_without_ros(tmp_path,monkeypatch):
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path))
    args=parser().parse_args(['gui','--preview'])
    with prepare_gui(args) as (root,monitor,mock):
        assert root.is_dir() and monitor is not None and mock is None
    assert root.exists()


def test_explicit_output_and_recovery_keep_their_original_contract(tmp_path,monkeypatch):
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path/'cache'))
    args=parser().parse_args(['gui','--preview','--output',str(tmp_path/'explicit')])
    with prepare_gui(args) as (root,monitor,mock):assert root==args.output and monitor is None
    with pytest.raises(FileExistsError):
        with prepare_gui(args):pass
    bad=parser().parse_args(['gui','--recover','--domain-id','181'])
    with pytest.raises(ValueError,match='封存'):
        with prepare_gui(bad):pass


def test_paused_archival_log_still_delivers_live_status(tmp_path):
    pause=tmp_path/'pause';pause.touch();output=[]
    process=subprocess.Popen([sys.executable,'-c','print("live feedback",flush=True)'],stdout=subprocess.PIPE,text=True)
    with (tmp_path/'log').open('x') as log:
        pump=OutputPump(process.stdout,log,output.append,pause);process.wait(timeout=5);pump.join()
    assert output==['live feedback\n'] and (tmp_path/'log').stat().st_size==0


def test_active_cache_is_not_implicitly_recovered(tmp_path,monkeypatch):
    import fcntl
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path));cache=new_cache();root=workspace(cache)
    with (root/'.frontend.lock').open('a+') as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        assert cached_sessions()[0]['active']
        with pytest.raises(ValueError,match='另一个GUI'):recover_workspace(cache.name)
    assert recover_workspace(cache.name)==root


def test_recovery_ignores_an_incomplete_csv_tail_and_empty_header(tmp_path):
    from casbot_calib.fieldwork_gui import last_csv
    source=tmp_path/'feedback.csv';source.write_text('receive_ns,q0\n123,0.25\n124,0.')
    assert last_csv(source)==dict(receive_ns='123',q0='0.25')
    source.write_text('receive_ns,q0\n');assert last_csv(source) is None


def test_snapshot_only_includes_controls_published_in_workspace_journal(tmp_path):
    root=tmp_path/'workspace';(root/'events').mkdir(parents=True)
    (root/'events/0.json').write_text('{"event":"control_created","control":"controls/committed"}')
    for name in ('committed','still_being_created'):
        path=root/'controls'/name;path.mkdir(parents=True);(path/'config.json').write_text('{}')
    target=tmp_path/'saved';save_snapshot(root,target)
    assert (target/'controls/committed/config.json').exists()
    assert not (target/'controls/still_being_created').exists()


def test_observation_only_cache_retains_last_actual_pose(tmp_path):
    from casbot_calib.fieldwork_storage import last_observation
    names=[f'joint_{i}' for i in range(14)]
    path=tmp_path/'observations/one/capture/raw/packets.jsonl';path.parent.mkdir(parents=True)
    row=dict(channel='joint',receive_sequence=0,receive_ns=123,type='sensor_msgs/msg/JointState',
        message=dict(name=names,position=[.2]*14,velocity=[],effort=[]))
    path.write_text(json.dumps(row)+'\n'+json.dumps(dict(channel='mode',message=dict(data='READY')))+'\n')
    result=last_observation(tmp_path,names)
    assert result['q']==[.2]*14 and result['dq'] is None and result['effort_raw'] is None


def test_multiple_manual_saves_update_only_the_mutable_last_saved_marker(tmp_path,monkeypatch):
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path/'cache'))
    cache=new_cache();root=workspace(cache)
    save_snapshot(root,tmp_path/'first');save_snapshot(root,tmp_path/'second')
    assert (tmp_path/'first/saved_session.json').exists() and (tmp_path/'second/saved_session.json').exists()
    assert json.loads((cache/'last_saved.json').read_text())['destination']==str(tmp_path/'second')


def test_explicit_gui_output_gets_budget(tmp_path,monkeypatch):
    from casbot_calib import fieldwork_launch as launch
    from casbot_calib.fieldwork_storage import recording_pause_file
    def create(profile,root,*args,**kwargs):
        (root/'protocol').mkdir(parents=True);(root/'protocol/session.json').write_text('{}')
    monkeypatch.setattr(launch,'create_session',create)
    args=parser().parse_args(['gui','--output',str(tmp_path/'explicit'),'--domain-id','181','--scope','hardware_position_stream'])
    with prepare_gui(args) as (root,monitor,mock):
        assert monitor is not None and mock is None and monitor.root==root
        assert recording_pause_file(root)==root/'recording_paused.json'
        monitor.limit=1;monitor.check();assert monitor.paused


def test_external_workspace_and_owned_mock_share_quota(tmp_path,monkeypatch):
    from casbot_calib.fieldwork_storage import attach_budget,directory_bytes,recording_pause_file
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path/'cache'))
    external=tmp_path/'external';external.mkdir()
    cache=new_cache(workspace=external);attach_budget(external,cache=cache)
    (external/'large.csv').write_bytes(b'x'*1000);(cache/'mock.csv').write_bytes(b'x'*300)
    monitor=CacheMonitor(cache)
    try:
        monitor.close();monitor.check()
        assert monitor.used==directory_bytes(cache)+directory_bytes(external)
        assert cache_for(external)==cache and recording_pause_file(external)==cache/'recording_paused.json'
        monitor.limit=500;monitor.check();assert monitor.paused
    finally:monitor.close()


def test_quota_rate_remaining_and_nested_roots_not_double_counted(tmp_path,monkeypatch):
    from casbot_calib.fieldwork_storage import directory_bytes
    monkeypatch.setenv('CASBOT_FIELDWORK_CACHE',str(tmp_path))
    cache=new_cache();root=workspace(cache)
    meta=json.loads((cache/'cache.json').read_text());meta['recording_roots']=[str(cache),str(root)]
    (cache/'cache.json').write_text(json.dumps(meta))
    monitor=CacheMonitor(cache);monitor.close()
    monkeypatch.setattr('casbot_calib.fieldwork_storage.time.monotonic',lambda:100.)
    monitor._last=(directory_bytes(cache)-1000,98.);monitor.check()
    assert monitor.used==directory_bytes(cache) and monitor.write_bytes_per_s==500.
    assert monitor.remaining_s==pytest.approx((monitor.limit-monitor.used)/500.)


def test_saved_snapshot_has_no_live_quota_binding(tmp_path):
    from casbot_calib.fieldwork_storage import attach_budget
    root=tmp_path/'explicit';root.mkdir();attach_budget(root)
    save_snapshot(root,tmp_path/'saved')
    assert not (tmp_path/'saved/recording_budget.json').exists()
    assert cache_for(tmp_path/'saved') is None
