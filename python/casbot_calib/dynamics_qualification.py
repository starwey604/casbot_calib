"""S3 evidence-backed, stage-specific observation gates; never actuation permission."""
import json
import csv
from pathlib import Path
import numpy as np
from .dynamics_data import load_dynamics,read_json
from .vendor_identity import sha256

POLICY=dict(version='s3_raw_observations_v2',minimum_dynamic_raw_hz=4000.,maximum_time_error_ns=100000,
            maximum_scale_relative_error=.05,maximum_zero_error_nm=.01,minimum_scale_reference_nm=.2,
            maximum_gap_factor=1.5,static_velocity_rad_s=.002,static_position_range_rad=.0005,
            minimum_seconds=dict(gravity=.5,friction=1.,breakaway=.5,inertia=12.,inertia_probe=1.))


def scale_check(meta):
    evidence=meta['scale_validation'];conversion=meta['conversion'];reasons=[]
    if evidence is None or conversion is None:return dict(passed=False,reasons=['missing_scale_reference_or_conversion'])
    if evidence.get('reference_basis') not in ('known_load','known_added_inertia','independent_transducer','synthetic_reference'):
        reasons.append('scale_reference_is_not_independent_of_the_observed_signal')
    if evidence.get('reference_basis')=='synthetic_reference' and meta['scope']!='synthetic_observations':
        reasons.append('synthetic_reference_cannot_qualify_hardware')
    signal=meta['contract']['signal'];identity=meta['contract']['identity']
    if not signal.get('sensor_identity') or evidence.get('sensor_identity')!=signal['sensor_identity']:
        reasons.append('scale_reference_sensor_identity_mismatch')
    if evidence.get('device_identity')!={key:identity.get(key) for key in ('robot_serial','firmware_version')}:
        reasons.append('scale_reference_device_or_firmware_mismatch')
    try:
        x=np.asarray(evidence['raw_delta'],dtype=float);y=np.asarray(evidence['reference_delta_nm'],dtype=float)
        if x.ndim!=2 or x.shape[1]!=14 or x.shape!=y.shape or len(x)<4 or not np.isfinite(x).all() or not np.isfinite(y).all():
            raise ValueError('invalid reference arrays')
        gain=np.asarray(conversion['gain_nm_per_raw_unit']);zero=np.asarray(conversion['zero_raw'])
        excitation=np.maximum(np.sqrt(np.mean(y*y,axis=0)),1e-12)
        error=np.sqrt(np.mean((x*gain-y)**2,axis=0))/excitation
        zero_ref=np.asarray(evidence['reference_zero_raw'],dtype=float)
        uncertainty=np.asarray(evidence['zero_uncertainty_raw'],dtype=float)
        if zero_ref.shape!=(14,) or uncertainty.shape!=(14,) or not np.isfinite(zero_ref).all() or not np.isfinite(uncertainty).all() or np.any(uncertainty<0):
            raise ValueError('invalid zero reference')
        zero_error=(abs(zero-zero_ref)+uncertainty)*abs(gain)
        if np.any(excitation<POLICY['minimum_scale_reference_nm']) or not np.all((y.min(axis=0)<0)&(y.max(axis=0)>0)):
            reasons.append('insufficient_bidirectional_scale_excitation')
        if max(error)>POLICY['maximum_scale_relative_error']:reasons.append('declared_gain_fails_independent_reference')
        if max(zero_error)>POLICY['maximum_zero_error_nm']:reasons.append('offset_is_not_anchored')
    except (KeyError,ValueError,TypeError) as exc:return dict(passed=False,reasons=['invalid_scale_evidence:'+str(exc)])
    return dict(passed=not reasons,reasons=reasons,relative_rmse=error.tolist(),maximum_relative_rmse=float(max(error)),
                maximum_zero_uncertainty_nm=float(max(zero_error)),parameters_refitted=False,
                source_trust='reference provenance is a supplied experimental contract, not a software proof of physical semantics')


def align_time(meta,data):
    c=meta['contract'];pt=data['position_time_ns'];tt=data['signal_time_ns'];reasons=[]
    if c is None or not np.all(pt>=0) or not np.all(tt>=0):
        return pt.copy(),dict(passed=False,reasons=['acquisition_time_unknown'],maximum_alignment_error_ns=None)
    pc=c['position_time']['clock_id'];tc=c['signal_time']['clock_id'];mapped=pt.copy();uncertainty=0.
    if pc!=tc:
        m=meta['clock_mapping']
        if m is None:return mapped,dict(passed=False,reasons=['different_unmapped_acquisition_clocks'],maximum_alignment_error_ns=None)
        try:
            if m['source_clock']!=pc or m['target_clock']!=tc:raise ValueError('clock IDs mismatch')
            src0=int(m['source_origin_ns']);dst0=int(m['target_origin_ns']);ratio=float(m['rate_ratio']);uncertainty=float(m['uncertainty_ns'])
            pairs_s=np.asarray(m['validation_source_ns']);pairs_t=np.asarray(m['validation_target_ns'])
            if (pairs_s.dtype.kind not in 'iu' or pairs_t.dtype.kind not in 'iu' or pairs_s.shape!=pairs_t.shape or pairs_s.ndim!=1
                    or len(pairs_s)<3 or not np.isfinite(ratio) or ratio<=0 or not np.isfinite(uncertainty) or uncertainty<0):
                raise ValueError('invalid affine validation pairs')
            if (np.any(pairs_s>np.iinfo(np.int64).max) or np.any(pairs_t>np.iinfo(np.int64).max)
                    or np.any(pairs_s<0) or np.any(pairs_t<0) or len(np.unique(pairs_s))!=len(pairs_s)):
                raise ValueError('invalid or duplicate int64 validation pairs')
            pairs_s=pairs_s.astype(np.int64);pairs_t=pairs_t.astype(np.int64)
            bounds=m['valid_source_range_ns']
            if np.min(pt)<bounds[0] or np.max(pt)>bounds[1] or np.min(pairs_s)>np.min(pt) or np.max(pairs_s)<np.max(pt):
                raise ValueError('mapping evidence does not cover this acquisition')
            errors=(pairs_s-src0).astype(float)*ratio-(pairs_t-dst0).astype(float)
            if np.max(abs(errors))+uncertainty>POLICY['maximum_time_error_ns']:raise ValueError('mapping validation residual too large')
            mapped=dst0+np.rint((pt-src0).astype(float)*ratio).astype(np.int64)
        except (KeyError,ValueError,TypeError) as exc:
            return pt.copy(),dict(passed=False,reasons=['clock_mapping_rejected:'+str(exc)],maximum_alignment_error_ns=None)
    skew=np.abs(mapped-tt)
    bound=float(max(skew))+uncertainty
    if bound>POLICY['maximum_time_error_ns']:reasons.append('position_signal_alignment_exceeds_0.1ms')
    return mapped,dict(passed=not reasons,reasons=reasons,position_clock=pc,signal_clock=tc,
                      maximum_alignment_error_ns=bound,clock_mapping_used=pc!=tc,
                      semantics='acquisition times from declared contract; no receive/header/RTT inference')


def contiguous_segments(valid,position_ns,signal_ns,sequence):
    """Never sort or concatenate across invalid samples, gaps or clock resets."""
    positive=np.r_[np.diff(position_ns),np.diff(signal_ns)];positive=positive[positive>0]
    step=int(np.median(positive)) if len(positive) else 0
    segments=[];begin=None
    for i,ok in enumerate(valid):
        cut=not ok
        if i and ok and begin is not None:
            dp=int(position_ns[i])-int(position_ns[i-1]);dt=int(signal_ns[i])-int(signal_ns[i-1])
            seq_gap=sequence[i]>=0 and sequence[i-1]>=0 and sequence[i]-sequence[i-1]!=1
            cut=bool(dp<=0 or dt<=0 or dp>POLICY['maximum_gap_factor']*step or dt>POLICY['maximum_gap_factor']*step or seq_gap)
        if cut and begin is not None:
            if i-begin>=2:segments.append((begin,i))
            begin=None
        if ok and begin is None:begin=i
    if begin is not None and len(valid)-begin>=2:segments.append((begin,len(valid)))
    return segments,step


def qualify_dynamics(folder,output):
    root=Path(folder);meta,data=load_dynamics(root);contract=meta['contract'];reasons=[]
    scale=scale_check(meta);aligned,time=align_time(meta,data)
    known_source=bool(meta['conversion'] is not None and contract and contract.get('position_unit')=='rad'
                      and contract.get('velocity_unit')=='rad/s')
    if not known_source:reasons.append('torque_source_units_or_total_output_semantics_unknown')
    if not scale['passed']:reasons.extend(scale['reasons'])
    if not time['passed']:reasons.extend(time['reasons'])
    names=meta['joint_names'];identity=contract.get('identity',{}) if contract else {}
    identity_ok=bool(identity.get('model_sha256') and len(identity['model_sha256'])==64 and identity.get('dataset_id'))
    if meta['scope']=='hardware_observations':
        identity_ok=identity_ok and all(identity.get(k) not in (None,'','unknown') for k in ('robot_serial','firmware_version','motion_software_version','provenance_reference'))
    if not identity_ok:reasons.append('device_model_or_dataset_identity_unqualified')
    status_known=np.isfinite(data['status_raw']).all(axis=1)
    if not np.any(status_known):reasons.append('saturation_or_fault_status_not_exposed')
    valid=(np.isfinite(data['q']).all(axis=1)&np.isfinite(data['dq']).all(axis=1)&np.isfinite(data['torque_nm']).all(axis=1)
           &status_known&np.all(data['status_raw']==0,axis=1))
    if not time['passed']:valid[:]=False
    segments,step=contiguous_segments(valid,aligned,data['signal_time_ns'],data['device_sequence'])
    rows=[]
    for begin,end in segments:
        dt=(int(aligned[end-1])-int(aligned[begin]))*1e-9
        rate=(end-begin-1)/dt if dt>0 else 0.
        rows.append(dict(begin=begin,end=end,samples=end-begin,duration_s=dt,observed_acquisition_hz=rate))
    filter_kind=contract.get('signal',{}).get('filter',{}).get('kind','unknown') if contract else 'unknown'
    raw_declared=bool(contract and contract.get('signal',{}).get('raw_acquisition') is True)
    source_complete=known_source and identity_ok and scale['passed'] and time['passed']
    stable=[]
    for row in rows:
        a,b=row['begin'],row['end']
        mask=valid[a:b]&(np.max(abs(data['dq'][a:b]),axis=1)<POLICY['static_velocity_rad_s'])
        windows,_=contiguous_segments(mask,aligned[a:b],data['signal_time_ns'][a:b],data['device_sequence'][a:b])
        for lo,hi in windows:
            duration=(int(aligned[a+hi-1])-int(aligned[a+lo]))*1e-9
            if duration>=POLICY['minimum_seconds']['gravity'] and np.max(np.ptp(data['q'][a+lo:a+hi],axis=0))<=POLICY['static_position_range_rad']:
                stable.append(dict(begin=a+lo,end=a+hi,duration_s=duration))
    band=meta['profile']['requested_band_hz'];stages={}
    for stage in ('gravity','friction','breakaway','inertia','inertia_probe'):
        stage_reasons=list(reasons)
        candidates=[row for row in rows if row['duration_s']>=POLICY['minimum_seconds'][stage]]
        if not candidates:stage_reasons.append('no_long_enough_contiguous_valid_segment')
        if stage=='gravity':
            if not stable:stage_reasons.append('no_stable_static_window')
            if filter_kind!='none':stage_reasons.append('static_filter_settling_not_qualified')
        else:
            if filter_kind!='none':stage_reasons.append('unknown_or_unmatched_filter_response')
            if not raw_declared:stage_reasons.append('raw_acquisition_provenance_unknown')
            candidates=[row for row in candidates if row['observed_acquisition_hz']>=POLICY['minimum_dynamic_raw_hz']*(1-1e-9)]
            if not candidates:stage_reasons.append('raw_bandwidth_below_4khz_or_no_valid_duration')
            if step<=0 or 1e9/step<=2*band:stage_reasons.append('requested_band_not_resolved')
            experiment=contract.get('experiment',{}) if contract else {}
            if stage=='inertia' and experiment.get('kind')=='periodic':
                start=int(experiment['begin_ns'])+round(float(experiment['period_s'])*experiment.get('discard_periods',2)*1e9)
                end=int(experiment['end_ns'])
                candidates=[row for row in candidates if aligned[row['begin']]<=start and
                            aligned[row['end']-1]+max(step,1)>=end]
                if not candidates:stage_reasons.append('declared_steady_window_crosses_gap_or_invalid_data')
        stages[stage]=dict(observations_eligible=bool(source_complete and not stage_reasons),reasons=sorted(set(stage_reasons)),
                           candidate_segments=candidates,execution_enabled=False,
                           note='Data qualification only; experiment design, prior-stage acceptance and control capability remain separate gates')
    result=dict(schema_version='casbot_vendor_s3_qualification_v1',passed=any(s['observations_eligible'] for s in stages.values()),
                scope=meta['scope'],stage_data_eligibility=stages,policy=POLICY,scale=scale,time=time,
                source_capture_scope=meta['source_capture_scope'],status_known_samples=int(status_known.sum()),
                temperature_observed_samples=int(np.isfinite(data['temperature_raw']).all(axis=1).sum()),
                device_sequence_known=bool(np.all(data['device_sequence']>=0)),
                gaps_scope='Source acquisition timing/sequence discontinuities, never a network packet-loss measurement',
                signal_origin=contract.get('signal',{}).get('origin','unknown') if contract else 'unknown',
                torque_is_independent_sensor=bool(meta['conversion'] and meta['conversion']['independent_torque_sensor']),
                torque_observation_channel_count=1 if known_source else 0,
                independent_torque_sensor_count=int(bool(meta['conversion'] and meta['conversion']['independent_torque_sensor'])),
                valid_samples=int(valid.sum()),total_samples=len(valid),segments=rows,stable_static_windows=stable,
                signal_filter_kind=filter_kind,requested_band_hz=band,
                hardware_executable=False,hardware_end_to_end_tested=False,physical_semantics_verified_by_software=False,
                driver_response_identification_eligible=False,driver_gate_reason='Requires separately qualified actual write history and sampled current; S3 observations alone do not provide it',
                source_manifest_sha256=sha256(root/'manifest.json'),source=str(root.resolve()),
                limitations=['Evidence declarations must be reviewed against device/firmware; digest checks are not physical verification',
                             'No implicit period-average alternative to raw 4kHz; it needs an independent measurement/model validation',
                             'No independent force sensor is required; current-derived effort remains one observation'])
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'processing.npz',valid=valid,aligned_position_time_ns=aligned)
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    with (output/'stages.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.writer(f);writer.writerow(['stage','observations_eligible','execution_enabled','reasons'])
        for name,status in stages.items():writer.writerow([name,status['observations_eligible'],False,';'.join(status['reasons'])])
    (output/'manifest.json').write_text(json.dumps({n:sha256(output/n) for n in ('report.json','processing.npz','stages.csv')},indent=2)+'\n')
    return result


def qualified_observations(qualification,stage):
    root=Path(qualification)
    for name,digest in read_json(root/'manifest.json').items():
        if sha256(root/name)!=digest:raise ValueError('Qualification report changed')
    report=read_json(root/'report.json');source=Path(report['source'])
    if sha256(source/'manifest.json')!=report['source_manifest_sha256']:raise ValueError('Qualification source binding changed')
    if not report['stage_data_eligibility'][stage]['observations_eligible']:
        raise ValueError('S3 observation gate blocks '+stage+': '+','.join(report['stage_data_eligibility'][stage]['reasons']))
    metadata,data=load_dynamics(source)
    with np.load(root/'processing.npz',allow_pickle=False) as processing:
        data['aligned_position_time_ns']=processing['aligned_position_time_ns'].copy()
    return metadata,data,report
