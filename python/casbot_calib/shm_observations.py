"""SHM observation time is mapped master conversion time, never receive/sample time."""
import bisect
import json
from pathlib import Path
import numpy as np
from .fieldwork_streaming import csv_rows
from .shm_fieldwork import text_result


def native_directory(root):
    root=Path(root).resolve()
    result=root/'result.json'
    if result.exists():
        value=json.loads(result.read_text())
        if value.get('schema_version')=='casbot_shm_h1_collection_v1':
            child=(root/value['effective_capture']).resolve()
            if not child.is_relative_to(root):raise ValueError('H1采集路径越界')
            root=child
    return root/'native' if (root/'native').is_dir() else root


def command_origin(root):
    root=native_directory(root);result=text_result(root/'result.txt')
    if not result:raise ValueError('缺少原生封存结果')
    origin=int(result.get('first_command_mono_ns',0))
    if origin<=0:raise ValueError('此记录没有首条执行命令；只读观测不能用于相对执行窗A分析')
    first=next((int(row['send_mono_ns']) for row in csv_rows(root/'commands.csv') if row['phase']=='3'),None)
    if first!=origin:raise ValueError('首条执行命令与原生时间原点不一致')
    return origin


class ObservationReader:
    def __init__(self,root):
        self.root=native_directory(root);self.origin=command_origin(self.root)
        self.pairs=list(csv_rows(self.root/'clock_pairs.csv'))
        self.pair_times=[int(p['mono_after_ns']) for p in self.pairs]
        if any(b<=a for a,b in zip(self.pair_times,self.pair_times[1:])):raise ValueError('时钟配对顺序无效')
        self.ambiguous=[]
        for a,b in zip(self.pairs,self.pairs[1:]):
            if int(b['jump_detected']):self.ambiguous.append((int(a['mono_after_ns']),int(b['mono_after_ns'])))

    def rows(self):
        previous=None;last_source={}
        for index,row in enumerate(csv_rows(self.root/'feedback.csv')):
            receive=int(row['receive_mono_ns']);source=int(row['source_timestamp_ms']);at=int(row['observation_time_ns']);segment=int(row['clock_segment'])
            if int(row['passive']):raise ValueError('被动快照不能作为一致SHM观测')
            if previous is not None and receive<=previous:raise ValueError('SHM接收时刻倒退或重复')
            previous=receive;pair=bisect.bisect_right(self.pair_times,receive)-1
            if pair<0:raise ValueError('反馈早于本机时钟配对')
            p=self.pairs[pair]
            if int(p['clock_segment'])!=segment or source*1_000_000-int(p['offset_ns'])!=at:
                raise ValueError('观测时间与原始毫秒stamp/本机配对不一致')
            uncertain=any(a<receive<=b for a,b in self.ambiguous)
            backward=segment in last_source and at<last_source[segment];last_source[segment]=at
            values=np.array([[float(row[prefix+str(j)]) for j in range(14)] for prefix in ('q','v','effort')])
            if not np.isfinite(values).all():raise ValueError('SHM反馈含非有限q/velocity/effort')
            yield dict(index=index,receive_ns=receive,observation_time_ns=at,source_timestamp_ms=source,
                clock_segment=segment,frame_counter=int(row['frame_counter']),phase=int(row.get('phase',-1)),elapsed_s=(at-self.origin)*1e-9,
                time_ambiguous=uncertain or backward,q=values[0],dq=values[1],effort_raw=values[2])

    def window(self,interval,segment=None):
        lo,hi=interval
        if not np.isfinite([lo,hi]).all() or lo>=hi:raise ValueError('窗口须START<END')
        selected=[];ends={};counts={};bad=[]
        for row in self.rows():
            s=row['clock_segment'];counts[s]=counts.get(s,0)+1
            if not row['time_ambiguous']:
                if s not in ends:ends[s]=[row,row]
                ends[s][1]=row
            if lo<=row['elapsed_s']<hi and (segment is None or segment==s):
                selected.append(row)
                if row['time_ambiguous']:bad.append(row['index'])
        if bad:raise ValueError('窗口涉及墙钟跳变的未定位区间或倒退时间；请选择稳定分段内的窗口')
        segments={r['clock_segment'] for r in selected}
        if len(segments)!=1:raise ValueError('窗口没有观测或跨越时钟分段；可显式指定--clock-segment')
        chosen=next(iter(segments));first,last=ends[chosen]
        if lo<first['elapsed_s'] or hi>last['elapsed_s']:raise ValueError('窗口超出该时钟段的有效反馈覆盖')
        if any(b['index']!=a['index']+1 for a,b in zip(selected,selected[1:])):raise ValueError('窗口观测不连续，不跨时钟缺口拼接')
        if selected[0]['index']!=first['index']:selected.insert(0,first)
        if selected[-1]['index']!=last['index']:selected.append(last)
        ns=np.array([r['observation_time_ns'] for r in selected],dtype=np.int64)
        obs=dict(receive_ns=np.array([r['receive_ns'] for r in selected],dtype=np.int64),observation_time_ns=ns,
                 **{key:np.array([r[key] for r in selected]) for key in ('q','dq','effort_raw')})
        return obs,(ns-np.int64(self.origin))*1e-9,dict(clock_segment=chosen,raw_rows=counts[chosen],
            source_timestamp_quantum_ns=1_000_000,time_basis='mapped master conversion time; not motor acquisition',
            raw_receive_times_preserved=True)
