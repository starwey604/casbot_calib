"""Exact streaming summaries, invariant fit windows and bounded video traversal."""
import csv
import json
from pathlib import Path

import numpy as np
import pytest

from casbot_calib.fieldwork_streaming import AnalysisScratch,StreamingCadence,scan_commands,scan_feedback,VideoFeedback,csv_rows
from casbot_calib.fieldwork_a_evidence import cadence,imu_summary,window_a


def write_feedback(path,ns,values=None):
    if values is None:values=np.zeros((len(ns),42))
    with path.open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['receive_ns',*[p+str(j) for p in ('q','v','effort_raw') for j in range(14)]])
        for n,v in zip(ns,values):writer.writerow([int(n),*v])


@pytest.mark.parametrize('intervals',[[],[0],[-1,0,-3],[1000,2000,1000,7000],[10**14,-1,7,7,5],list(range(1,5000))])
def test_external_quantiles_equal_original_numpy(intervals,tmp_path):
    ns=np.concatenate(([10**18],10**18+np.cumsum(intervals,dtype=np.int64)))
    with AnalysisScratch(tmp_path) as scratch:
        counter=StreamingCadence(scratch)
        for n in ns:counter.add(n)
        assert counter.result()==cadence(ns)
        series=scratch.series();values=np.array(intervals or [0],dtype=np.int64)
        for v in values:scratch.add(series,v)
        q=scratch.quantiles(series,len(values),dict(p50=.5,p99=.99,maximum=1.),1e-6)
        assert q=={k:float(np.quantile(values*1e-6,p)) for k,p in dict(p50=.5,p99=.99,maximum=1.).items()}
    assert not list(tmp_path.iterdir())


def test_csv_scan_keeps_full_counts_but_only_window_values(tmp_path):
    origin=10**18;ns=origin+np.arange(10000,dtype=np.int64)*1000000
    values=np.arange(10000*42,dtype=float).reshape((-1,42));values[0,0]=np.nan;values[-1,15]=np.inf
    write_feedback(tmp_path/'feedback.csv',ns,values)
    with (tmp_path/'commands.csv').open('w') as stream:
        out=csv.writer(stream);out.writerow(['publish_ns','planned_ns'])
        for i,n in enumerate(ns[::2]):out.writerow([int(n),int(n)-i%97])
    with AnalysisScratch(tmp_path) as scratch:
        cmd=scan_commands(tmp_path/'commands.csv',scratch)
        stats,obs,contiguous=scan_feedback(tmp_path/'feedback.csv',scratch,origin,[4,5])
        assert cmd['sender']==cadence(ns[::2]) and cmd['planned']==cadence(ns[::2]-np.arange(5000)%97)
        assert stats['receiver']==cadence(ns) and stats['missing_values']==dict(q=1,dq=1,effort_raw=0)
        assert contiguous and len(obs['receive_ns'])==1002
        np.testing.assert_array_equal(obs['receive_ns'][1:-1],ns[4000:5000])
        np.testing.assert_array_equal(obs['q'][1:-1],values[4000:5000,:14])
        assert stats['feedback_interval_s']==[0.,(ns[-1]-origin)*1e-9]


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


def test_imu_streaming_summary_and_window_match_in_memory(tmp_path):
    raw=tmp_path/'raw';raw.mkdir();origin=10**18
    (raw/'recorder.json').write_text('{"clock_id":"known"}')
    values=np.random.default_rng(5).normal(size=(1000,6));packets=[]
    for i,v in enumerate(values):
        packets.append(dict(channel='imu',receive_ns=origin+i*1000000,message=dict(header=dict(frame_id='torso'),
            angular_velocity=v[:3].tolist(),linear_acceleration=v[3:].tolist(),angular_velocity_covariance=[-1.] if i%7==0 else [])))
    (raw/'packets.jsonl').write_text(''.join(json.dumps(p)+'\n' for p in packets))
    with AnalysisScratch(tmp_path) as scratch:
        summary=imu_summary(tmp_path,scratch=scratch,spool_windows=True)[0]
        assert summary['receive']==cadence([p['receive_ns'] for p in packets])
        for key,use,cols in [('angular_velocity',np.arange(1000)%7!=0,slice(0,3)),('linear_acceleration',np.ones(1000,bool),slice(3,6))]:
            data=values[use,cols];result=summary['channels'][key]
            np.testing.assert_allclose(result['mean'],data.mean(axis=0),rtol=1e-13,atol=1e-14)
            np.testing.assert_allclose(result['rms_about_mean'],np.sqrt(np.mean((data-data.mean(axis=0))**2,axis=0)),rtol=1e-13)
        source=scratch.imu_sources[0];ns,v=scratch.imu_window(source,origin+300000000,origin+400000000)
        valid=np.flatnonzero(np.arange(1000)%7!=0);expected=np.unique(np.r_[valid[0],valid[(valid>=300)&(valid<400)],valid[-1]])
        np.testing.assert_array_equal(ns,origin+expected*1000000);np.testing.assert_array_equal(v,values[expected])


def test_window_fit_is_unchanged_after_streaming_crop(tmp_path,monkeypatch):
    from test_hardware_probe import fixture,rotor
    import casbot_calib.model_runtime as runtime
    monkeypatch.setattr(runtime,'model_for',lambda config:(rotor(),None))
    obs,t,_=fixture();origin=10**18
    ns=origin+np.round(t*1e9).astype(np.int64)
    data=np.column_stack([obs['q'],obs['dq'],obs['effort_raw']]);write_feedback(tmp_path/'feedback.csv',ns,data)
    profile=dict(gravity_world=[0.,0.,0.],effort_gain=1.,effort_zero_nm=0.)
    original=window_a(dict(obs,receive_ns=ns), (ns-origin)*1e-9,[1,3],4,profile,30.)
    with AnalysisScratch(tmp_path) as scratch:
        _,selected,contiguous=scan_feedback(tmp_path/'feedback.csv',scratch,origin,[1,3])
    streamed=window_a(selected,(selected['receive_ns']-origin)*1e-9,[1,3],4,profile,30.)
    assert contiguous and streamed==original


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


@pytest.mark.parametrize('failure',[None,'encode','partial'])
def test_video_pipeline_uses_stream_and_closes_encoder(tmp_path,monkeypatch,failure):
    from contextlib import contextmanager
    from types import SimpleNamespace
    import imageio_ffmpeg
    from PIL import ImageDraw
    from casbot_calib import fieldwork_worker as worker,presentation
    path=tmp_path/'runs/unit/capture/execution/feedback.csv';path.parent.mkdir(parents=True)
    ns=np.array([10**18,10**18+70000000,10**18+130000000],dtype=np.int64)
    values=np.repeat(np.arange(3)[:,None],42,axis=1);write_feedback(path,ns,values)
    if failure=='partial':path.write_text(path.read_text()+'1000000000200000000,0.1\n')
    meta=dict(scope='localhost_contract_mock')
    monkeypatch.setattr(worker,'Session',lambda root:SimpleNamespace(metadata=meta))
    monkeypatch.setattr(worker,'frame',lambda metadata,view,q,receive_ns:dict(q=q,ns=receive_ns))
    monkeypatch.setattr(presentation,'_font',lambda size:None)
    monkeypatch.setattr(ImageDraw,'Draw',lambda image:SimpleNamespace(rectangle=lambda *a,**k:None,text=lambda *a,**k:None))
    seen=[];closed=[];encoded=[]
    class View:
        def render(self,value):seen.append(value);return np.zeros((8,8,3),dtype=np.uint8)
    @contextmanager
    def view(*args):yield View()
    monkeypatch.setattr(worker,'open_view',view)
    def encoder(output,*a,**k):
        Path(output).write_bytes(b'encoder stub')
        try:
            while True:
                image=yield
                encoded.append(image)
                if failure=='encode':raise OSError('test encoder failure')
        finally:closed.append(True)
    monkeypatch.setattr(imageio_ffmpeg,'write_frames',encoder)
    # Any accidental reintroduction of whole-table NumPy loading fails the test.
    monkeypatch.setattr(np,'genfromtxt',lambda *a,**k:pytest.fail('whole CSV loaded'))
    output=tmp_path/'runs/unit/recording/motion.mp4'
    if failure:
        with pytest.raises((ValueError,OSError)):
            worker.render_recording(tmp_path,'runs/unit')
        assert not output.exists()
    else:
        result=worker.render_recording(tmp_path,'runs/unit')
        assert result['duration_s']==.13 and output.exists()
        assert [v['ns'] for v in seen]==[ns[0],ns[0],ns[1],ns[2]]
        assert len(encoded)==4
    assert closed==[True]
