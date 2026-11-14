"""Exact streaming summaries, invariant fit windows and bounded video traversal."""
import csv
import json
from pathlib import Path

import numpy as np
import pytest

from casbot_calib.fieldwork_streaming import AnalysisScratch,StreamingCadence,scan_commands,scan_feedback,VideoFeedback,csv_rows


def write_feedback(path,ns,values=None):
    if values is None:values=np.zeros((len(ns),42))
    with path.open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['receive_ns',*[p+str(j) for p in ('q','v','effort_raw') for j in range(14)]])
        for n,v in zip(ns,values):writer.writerow([int(n),*v])






def test_window_discontinuity_is_not_hidden_by_cropping(tmp_path):
    origin=10**15;ns=origin+np.array([0,2,8,3,9])*1000000000
    write_feedback(tmp_path/'feedback.csv',ns)
    with AnalysisScratch(tmp_path) as scratch:
        _,_,contiguous=scan_feedback(tmp_path/'feedback.csv',scratch,origin,[1,5])
    assert not contiguous


@pytest.mark.parametrize('last',[25000000,75000000,100000000,130000000,1000000000])
def test_video_zoh_matches_searchsorted_with_duplicate_stamps(tmp_path,last):
    offsets=sorted([0,0,10000000,last,last]);ns=np.array(offsets,dtype=np.int64)+10**18
    values=np.repeat(np.arange(len(ns))[:,None],42,axis=1)
    path=tmp_path/'feedback.csv';write_feedback(path,ns,values)
    video=VideoFeedback(path);actual=list(video.frames())
    targets=range(0,last+25000000,50000000)
    expected=[max(0,int(np.searchsorted(ns,10**18+t,side='right')-1)) for t in targets]
    assert [row[1] for row in actual]==[ns[i] for i in expected]
    assert [row[2][0] for row in actual]==expected
    assert video.rows==len(ns) and video.duration_s==last*1e-9


def test_video_rejects_backwards_incomplete_and_nonfinite_data(tmp_path):
    path=tmp_path/'feedback.csv';write_feedback(path,[3,2])
    with pytest.raises(ValueError,match='倒退'):list(VideoFeedback(path).frames())
    write_feedback(path,[0,1]);path.write_text(path.read_text()+'2,.1\n')
    with pytest.raises(ValueError,match='不完整'):list(VideoFeedback(path).frames())
    v=np.zeros((2,42));v[1,0]=np.nan;write_feedback(path,[0,1],v)
    with pytest.raises(ValueError,match='非有限'):list(VideoFeedback(path).frames())


def test_video_cancel_checks_rows_not_only_output_frames(tmp_path):
    path=tmp_path/'feedback.csv';write_feedback(path,[0]*100)
    calls=[]
    def cancel():calls.append(1);return 'operator_cancel' if len(calls)>10 else None
    with pytest.raises(ValueError,match='operator_cancel'):list(VideoFeedback(path,cancel).frames())
    assert len(calls)==11






def test_scratch_failure_is_explicit_and_cleans_disk(tmp_path):
    import sqlite3
    with pytest.raises(RuntimeError,match='临时磁盘'):
        with AnalysisScratch(tmp_path) as scratch:
            scratch.db.execute('PRAGMA query_only=ON')
            scratch.add(scratch.series(),123);scratch.flush()
    assert not list(tmp_path.iterdir())


def test_imu_spool_does_not_deduplicate_identical_frames(tmp_path):
    with AnalysisScratch(tmp_path) as scratch:
        for seq in range(5):scratch.add_imu(0,seq,10**18,[0.]*6)
        ns,values=scratch.imu_window(dict(id=0,count=5),10**18,10**18+1)
        assert ns.tolist()==[10**18]*5 and values.shape==(5,6)


