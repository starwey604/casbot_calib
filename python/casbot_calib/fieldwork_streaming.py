"""Bounded CSV traversal and disk-backed exact statistics for fieldwork readers."""
from array import array
from collections import Counter
import csv
import math
from pathlib import Path
import sqlite3
import tempfile

import numpy as np


def csv_rows(path, cancel_check=None):
    """One row at a time; malformed tails are errors, never silently trimmed."""
    path=Path(path)
    with path.open(newline='') as stream:
        reader=csv.DictReader(stream,strict=True)
        names=reader.fieldnames
        if names is None:raise ValueError('CSV没有数据：'+str(path))
        if len(set(names))!=len(names):raise ValueError('CSV列名重复：'+str(path))
        count=0
        try:
            for row in reader:
                if cancel_check:
                    reason=cancel_check()
                    if reason:raise ValueError('视频导出已中断：'+reason)
                if None in row or any(value is None for value in row.values()):
                    raise ValueError('CSV末尾不完整，请先保存会话：'+str(path))
                count+=1;yield row
        except csv.Error as exc:raise ValueError(f'CSV格式错误：{path}：{exc}') from exc
        if not count:raise ValueError('CSV没有数据：'+str(path))


class AnalysisScratch:
    """Exact empirical quantiles and IMU windows on disk, with a 4MiB SQL cache.

    Histograms preserve every observation (including repeated values). Distinct
    nanosecond intervals live in SQLite, not an unbounded Python dictionary.
    """
    def __init__(self, directory=None):
        self.temporary=tempfile.TemporaryDirectory(prefix='fieldwork-analysis-',dir=directory)
        self.db=None
        try:
            self.db=sqlite3.connect(str(Path(self.temporary.name)/'statistics.sqlite'))
            self.db.executescript('''PRAGMA journal_mode=OFF; PRAGMA synchronous=OFF;
            PRAGMA temp_store=FILE; PRAGMA cache_size=-4096; PRAGMA mmap_size=0;
            CREATE TABLE samples(series INTEGER, value INTEGER, n INTEGER,
                PRIMARY KEY(series,value)) WITHOUT ROWID;
            CREATE TABLE imu(source INTEGER, seq INTEGER, ns INTEGER,
                v0 REAL,v1 REAL,v2 REAL,v3 REAL,v4 REAL,v5 REAL,
                PRIMARY KEY(source,seq)) WITHOUT ROWID;
            CREATE INDEX imu_time ON imu(source,ns);''')
        except sqlite3.Error as exc:
            if self.db is not None:self.db.close()
            self.temporary.cleanup()
            raise RuntimeError('分析临时磁盘不可用：'+str(exc)) from exc
        self.pending=Counter();self.pending_count=0;self.serial=0;self.imu_pending=[];self.imu_sources=[]
    def __enter__(self):return self
    def __exit__(self,exc_type,exc,tb):
        try:self.db.close()
        finally:self.temporary.cleanup()
        if isinstance(exc,sqlite3.Error):raise RuntimeError('分析临时磁盘写入或查询失败：'+str(exc)) from exc
    def series(self):
        result=self.serial;self.serial+=1;return result
    def add(self, series, value):
        self.pending[(series,int(value))]+=1;self.pending_count+=1
        if self.pending_count>=4096:self.flush()
    def flush(self):
        if self.pending:
            self.db.executemany('INSERT INTO samples VALUES (?,?,?) ON CONFLICT(series,value) DO UPDATE SET n=n+excluded.n',
                ((series,value,n) for (series,value),n in self.pending.items()))
            self.pending.clear();self.pending_count=0
        if self.imu_pending:
            self.db.executemany('INSERT INTO imu VALUES (?,?,?,?,?,?,?,?,?)',self.imu_pending);self.imu_pending.clear()
        self.db.commit()
    def quantiles(self, series, count, points, scale):
        self.flush()
        if not count:return {}
        ranks={q:q*(count-1) for q in points.values()}
        wanted=sorted({int(f(r)) for r in ranks.values() for f in (math.floor,math.ceil)})
        selected={};at=0;target=iter(wanted);rank=next(target,None)
        for value,n in self.db.execute('SELECT value,n FROM samples WHERE series=? ORDER BY value',(series,)):
            while rank is not None and rank<at+n:
                selected[rank]=float(value)*scale;rank=next(target,None)
            if rank is None:break
            at+=n
        result={}
        for key,q in points.items():
            r=ranks[q];lo=math.floor(r);hi=math.ceil(r);fraction=r-lo
            a,b=selected[lo],selected[hi];delta=b-a
            # Match NumPy's linear interpolation, including its >=.5 branch.
            result[key]=b-delta*(1-fraction) if fraction>=.5 else a+delta*fraction
        return result
    def add_imu(self, source, seq, ns, values):
        self.imu_pending.append((source,seq,ns,*values))
        if len(self.imu_pending)>=4096:self.flush()
    def imu_window(self, source, lo_ns, hi_ns):
        self.flush();count=source['count']
        rows=self.db.execute('''SELECT seq,ns,v0,v1,v2,v3,v4,v5 FROM imu
            WHERE source=? AND ns>=? AND ns<?
            UNION SELECT seq,ns,v0,v1,v2,v3,v4,v5 FROM imu WHERE source=? AND seq IN (0,?)
            ORDER BY seq''',(source['id'],lo_ns,hi_ns,source['id'],count-1))
        times=array('q');values=array('d')
        for _,ns,*v in rows:times.append(ns);values.extend(v)
        return np.asarray(times,dtype=np.int64),np.asarray(values,dtype=float).reshape((-1,6))


class StreamingCadence:
    def __init__(self, scratch):
        self.scratch=scratch;self.series=scratch.series();self.count=0;self.positive=0;self.nonincreasing=0
        self.first=self.last=None
    def add(self, ns):
        ns=int(ns)
        if self.count:
            delta=ns-self.last
            if delta>0:self.scratch.add(self.series,delta);self.positive+=1
            else:self.nonincreasing+=1
        else:self.first=ns
        self.last=ns;self.count+=1
    def result(self):
        if self.count<2:return dict(rows=self.count,hz=None,nonincreasing_intervals=0)
        return dict(rows=self.count,hz=(self.count-1)*1e9/(self.last-self.first) if self.last>self.first else None,
            nonincreasing_intervals=self.nonincreasing,
            interval_ms=self.scratch.quantiles(self.series,self.positive,
                dict(p50=.5,p95=.95,p99=.99,maximum=1.),1e-6))


def scan_commands(path, scratch):
    cadence=StreamingCadence(scratch);planned=StreamingCadence(scratch);lateness=scratch.series()
    has_planned=False
    for row in csv_rows(path):
        if 'publish_ns' not in row:raise ValueError('CSV缺少publish_ns：'+str(path))
        sent=int(row['publish_ns']);cadence.add(sent);has_planned='planned_ns' in row
        if has_planned:
            target=int(row['planned_ns']);planned.add(target);scratch.add(lateness,sent-target)
    result=dict(sender=cadence.result(),first_publish_ns=cadence.first)
    if has_planned:
        result.update(planned=planned.result(),send_lateness_ms=scratch.quantiles(lateness,cadence.count,dict(p50=.5,p99=.99,maximum=1.),1e-6))
    return result


def scan_feedback(path, scratch, origin, window=None):
    cadence=StreamingCadence(scratch);missing=dict(q=0,dq=0,effort_raw=0)
    times=array('q');values=array('d');first=last=None;first_selected=last_selected=None;contiguous=True
    for index,row in enumerate(csv_rows(path)):
        if 'receive_ns' not in row:raise ValueError('CSV缺少receive_ns：'+str(path))
        ns=int(row['receive_ns']);cadence.add(ns)
        v=[float(row.get(prefix+str(j)) or 'nan') for prefix in ('q','v','effort_raw') for j in range(14)]
        finite=np.isfinite(v).reshape((3,14))
        for k,key in enumerate(missing):missing[key]+=int(np.sum(~finite[k]))
        last=(ns,v,index)
        if first is None:first=last
        elapsed=(ns-origin)*1e-9
        if window is not None and window[0]<=elapsed<window[1]:
            if last_selected is not None and index!=last_selected+1:contiguous=False
            if first_selected is None:first_selected=index
            last_selected=index;times.append(ns);values.extend(v)
    result=dict(receiver=cadence.result(),feedback_interval_s=[(first[0]-origin)*1e-9,(last[0]-origin)*1e-9],missing_values=missing)
    obs=None
    if window is not None:
        t=np.asarray(times,dtype=np.int64);v=np.asarray(values,dtype=float).reshape((-1,42))
        # Only global endpoints are needed for the original coverage check;
        # the Fourier kernel itself consumes exactly [START,END).
        if first_selected!=0:
            t=np.concatenate(([first[0]],t));v=np.vstack((first[1],v))
        if last_selected!=last[2] and last[2]!=0:
            t=np.concatenate((t,[last[0]]));v=np.vstack((v,last[1]))
        obs=dict(receive_ns=t,q=v[:,:14],dq=v[:,14:28],effort_raw=v[:,28:42])
    return result,obs,contiguous


class VideoFeedback:
    """Receive-time ZOH at 20fps with two live feedback rows, no whole-table load."""
    def __init__(self,path,cancel_check=None,*,receive_column='receive_ns'):
        self.path=Path(path);self.cancel_check=cancel_check;self.receive_column=receive_column;self.duration_s=0.;self.rows=0
    def frames(self):
        rows=csv_rows(self.path,self.cancel_check);origin=None;previous=None
        def sample():
            nonlocal origin,previous
            try:row=next(rows)
            except StopIteration:return None
            if self.receive_column not in row:raise ValueError('视频CSV缺少'+self.receive_column)
            ns=int(row[self.receive_column])
            if previous is not None and ns<previous:raise ValueError('视频反馈接收时刻倒退，不能按时间回放')
            if any('q'+str(j) not in row for j in range(14)):raise ValueError('视频CSV缺少完整14轴位置')
            q=[float(row['q'+str(j)]) for j in range(14)]
            if not np.isfinite(q).all():raise ValueError('视频反馈位置有缺失或非有限值')
            if origin is None:origin=ns
            previous=ns;self.duration_s=(ns-origin)*1e-9;self.rows+=1
            return ns,q
        try:
            current=sample();upcoming=sample()
            if upcoming is None:raise ValueError('记录不足两帧')
            index=0
            while True:
                if self.cancel_check:
                    reason=self.cancel_check()
                    if reason:raise ValueError('视频导出已中断：'+reason)
                target=origin+index*50000000
                while upcoming is not None and upcoming[0]<=target:
                    current=upcoming;upcoming=sample()
                if upcoming is None and index*50000000>=previous-origin+25000000:break
                yield index*.05,current[0],current[1]
                index+=1
        finally:rows.close()


class RunningVector:
    def __init__(self):
        self.count=0;self.mean=np.zeros(3);self.m2=np.zeros(3);self.peak=0.
    def add(self,value):
        value=np.asarray(value,dtype=float);self.count+=1
        delta=value-self.mean;self.mean+=delta/self.count;self.m2+=delta*(value-self.mean)
        self.peak=max(self.peak,float(np.linalg.norm(value)))
    def result(self):
        return dict(valid_rows=self.count,mean=self.mean.tolist(),
            rms_about_mean=np.sqrt(np.maximum(self.m2/self.count,0.)).tolist(),peak_norm=self.peak)
