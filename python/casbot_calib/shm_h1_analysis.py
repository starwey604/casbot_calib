"""Streaming H1 collision/position/static summaries on SHM observation time."""
import bisect
from pathlib import Path
import numpy as np
from scipy.interpolate import BPoly
from .data import write_json
from .field_artifacts import check_seal,seal,output_outside,FieldContractError
from .artifact_io import sha256
from .model_runtime import model_for,collision_for
from .collision import certify_polyline
from .shm_h1 import read
from .shm_observations import ObservationReader
from .shm_fieldwork import text_result


class TimeMean:
    """Trapezoid exposure per unique timestamp; duplicates share group weight."""
    def __init__(self):
        self.current=None;self.previous=None;self.total=None;self.weight=0.;self.rows=0;self.groups=0
        self.first=None;self.last=None;self.maximum_gap_ns=0;self.error=None;self.segment=None
        self.low=None;self.high=None
    def _flush(self,next_ns=None):
        if self.current is None:return
        at,values,count=self.current
        before=at-self.previous if self.previous is not None else 0
        after=next_ns-at if next_ns is not None else 0
        w=(before+after)*.5
        if w>0:
            if self.total is None:self.total=np.zeros_like(values)
            self.total+=w*values/count;self.weight+=w
        self.previous=at;self.groups+=1
    def add(self,row,values):
        ns=row['observation_time_ns'];self.rows+=1
        if row['time_ambiguous']:self.error='wall-clock jump interval or source time reversal'
        if self.segment is not None and self.segment!=row['clock_segment']:self.error='measurement crosses clock segments'
        self.segment=row['clock_segment']
        self.low=np.asarray(values).copy() if self.low is None else np.minimum(self.low,values)
        self.high=np.asarray(values).copy() if self.high is None else np.maximum(self.high,values)
        if self.first is None:self.first=ns
        self.last=ns
        if self.current is not None and ns<self.current[0]:self.error='observation time reversal';return
        if self.current is not None and ns==self.current[0]:self.current[1]+=values;self.current[2]+=1;return
        if self.current is not None:self.maximum_gap_ns=max(self.maximum_gap_ns,ns-self.current[0]);self._flush(ns)
        self.current=[ns,np.asarray(values).copy(),1]
    def result(self):
        self._flush();self.current=None
        return None if self.error or not self.weight else self.total/self.weight


def analyse(plan,capture,output,*,windows_only=False):
    plan=Path(plan).resolve();capture=Path(capture).resolve();check_seal(plan);check_seal(capture)
    collection=read(capture/'result.json')
    if collection.get('schema_version')=='casbot_shm_h1_collection_v1':
        if collection['original_plan_manifest_sha256']!=sha256(plan/'manifest.json'):raise FieldContractError('H1 collection属于其它计划')
        if not collection['execution_completed']:raise FieldContractError('H1 collection执行未完成')
        effective=capture/collection['effective_capture']
        replanned=capture/collection['effective_plan'] if collection['effective_plan'] else plan
        if not effective.resolve().is_relative_to(capture) or (collection['effective_plan'] and not replanned.resolve().is_relative_to(capture)):
            raise FieldContractError('H1 collection路径越界')
        capture=effective.resolve();plan=replanned.resolve();check_seal(plan);check_seal(capture)
    m=read(plan/'plan.json');binding=read(capture/'h1_capture.json');native=text_result(capture/'native/result.txt')
    if binding['plan_manifest_sha256']!=sha256(plan/'manifest.json') or binding['reference_sha256']!=m['reference_sha256']:
        raise FieldContractError('H1采集与封存计划不同')
    if not native or native['execution_completed']!='1' or binding['returncode']!=0:raise FieldContractError('H1执行未完成；故障记录仅供诊断，不恢复会话')
    if native['recording_stopped']!='0':raise FieldContractError('H1记录中途停录；保留前缀，不伪造完整采集')
    root=output_outside(output,plan,capture);root.mkdir(parents=True,exist_ok=False)
    c=m['config'];engine,source=model_for(c)
    if source['sha256']!=m['source_model_sha256']:raise FieldContractError('H1模型变化')
    geometry=collision_for(c,engine);reader=ObservationReader(capture)
    motion=read(plan/'motion.json');curve=BPoly.from_derivatives(motion['knots_s'],np.asarray(motion['states']))
    ready=m.get('preparation');calibration_begin=ready['calibration_begin_ns']*1e-9 if ready else 0.
    interval=motion['conditions'].get('comparison_window_s',[calibration_begin,m['duration_ns']*1e-9])
    tracking=TimeMean();windows=[TimeMean() for _ in m['trials']];maximum_speed=[0.]*len(windows)
    begins=[t['measure_begin_s'] for t in m['trials']];ends=[t['measure_end_s'] for t in m['trials']]
    block=[];proofs=[];count=0;checked=0;excluded=0;first_receive=None;last_receive=None
    def prove(rows):
        q=np.array([r['q'] for r in rows]);ns=np.array([r['receive_ns'] for r in rows],dtype=np.int64)
        proof=certify_polyline(geometry,q,(ns-ns[0])*1e-9,clearance_m=c['scene']['clearance_m'],block_samples=1024)
        if not proof['passed']:write_json(root/'collision_received_failure.json',proof);raise FieldContractError('实际接收位置折线未通过碰撞检查')
        proofs.append(dict(passed=True,minimum_distance_m=proof.get('minimum_distance_m',proof.get('minimum_sampled_distance_m')),rows=len(rows)))
    for row in reader.rows():
        count+=1
        if first_receive is None:first_receive=row['receive_ns']
        last_receive=row['receive_ns']
        elapsed=row['elapsed_s']
        if ready is None or (row['phase']==3 and elapsed>=calibration_begin):
            checked+=1;block.append(row)
            if len(block)==1024:prove(block);block=[block[-1]]
        else:
            if len(block)>1:prove(block)
            block=[];excluded+=1
        if interval[0]<=elapsed<interval[1]:
            error=row['q']-curve(elapsed)
            tracking.add(row,np.r_[error*error,row['q']])
        index=bisect.bisect_right(begins,elapsed)-1
        if 0<=index<len(ends) and elapsed<ends[index]:
            torque=(row['effort_raw']-np.asarray(c['effort_zero_nm']))*np.asarray(c['effort_gain'])
            windows[index].add(row,np.r_[row['q'],torque]);maximum_speed[index]=max(maximum_speed[index],float(np.max(abs(row['dq']))))
    if len(block)>1:prove(block)
    if count<2:raise ValueError('H1没有可分析的完整反馈')
    if checked<2:raise ValueError('H1正式参考段没有可分析的完整反馈')
    means=[]
    for trial,values,speed in zip(m['trials'],windows,maximum_speed):
        value=values.result();available=value is not None
        means.append(dict(name=trial['name'],conditions=trial['conditions'],q=value[:14] if available else None,
            torque=value[14:] if available else None,observation_rows=values.rows,observation_time_groups=values.groups,
            available=available,unavailable_reason=values.error or (None if available else 'no distinct observation times in window'),
            observed_duration_s=(values.last-values.first)*1e-9 if values.first is not None else 0.,
            maximum_received_speed_rad_s=speed,maximum_observation_gap_s=values.maximum_gap_ns*1e-9,
            position_span_rad=values.high[:14]-values.low[:14] if values.low is not None else None,
            independent_acquisition_count=None,sample_time_known=False,time_basis='mapped master conversion time relative to first execution command',
            weighting='trapezoid exposure per millisecond group; equal shares within group'))
    position=tracking.result()
    report=dict(schema_version='casbot_shm_h1_report_v1',passed=True,execution_completed=True,collection_complete=True,
        stage=m['stage'],feedback_rows=count,feedback_receive_hz=(count-1)*1e9/(last_receive-first_receive),
        observation_time_origin_ns=reader.origin,time_basis='mapped master conversion time, not acquisition',
        first_shm_write_mono_ns=int(native['first_shm_write_mono_ns']),sample_time_known=False,torque_scale_verified=False,
        hardware_calibration_accepted=False,analysis_available=position is not None if m['stage']=='position' else False,
        tracking_rmse_rad=np.sqrt(position[:14]) if position is not None else None,
        tracking_unavailable_reason=tracking.error,reference_scope='nominal continuous reference and received piecewise-linear path, not unobserved physical motion',
        source_plan_manifest_sha256=sha256(plan/'manifest.json'),source_capture_manifest_sha256=sha256(capture/'manifest.json'))
    if ready:
        report.update(preparation=ready,calibration_begin_s=calibration_begin,
            reference_scope='formal calibration continuous reference and received piecewise-linear path; preparation collision check explicitly skipped')
    write_json(root/'collision_received_interpolant.json',dict(passed=True,chunks=proofs,feedback_rows=count,
        collision_checked_rows=checked,excluded_rows=excluded,execution_begin_s=calibration_begin,
        scope='formal_phase3_only_preparation_and_entry_exit_excluded' if ready else 'full_capture'))
    write_json(root/'static_windows.json',means)
    if m['stage']!='position':
        if windows_only:report.update(analysis_available=any(r['available'] for r in means),stage='gravity_windows')
        else:
            try:
                from .field_gravity import write_gravity_candidate
                fit=write_gravity_candidate(means,c,root,m);report.update(fit);report['analysis_available']=True
                report['schema_version']='casbot_shm_h1_report_v1'
            except (ValueError,np.linalg.LinAlgError) as error:
                report.update(analysis_available=False,analysis_unavailable_reason=str(error),fit_succeeded=False)
    write_json(root/'report.json',report);seal(root);return report
