"""S3 observable-only units/source/clock boundary. No device I/O or fitting."""
import json
from pathlib import Path
import numpy as np
import yaml
from .vendor_identity import sha256


def read_json(path):return json.loads(Path(path).read_text())


def evidence(reference,base,schema):
    if reference is None:return None,None
    if not isinstance(reference,dict) or set(reference)!={'path','sha256'}:
        raise ValueError('Evidence requires an explicit file path and SHA256')
    path=(Path(base)/reference['path']).resolve()
    if sha256(path)!=reference['sha256']:raise ValueError('Evidence file changed')
    content=read_json(path)
    if content.get('schema_version')!=schema:raise ValueError('Unsupported evidence schema')
    return content,path


def load_profile(path):
    profile=yaml.safe_load(Path(path).read_text())
    if profile.get('schema_version')!='casbot_vendor_s3_profile_v1' or profile.get('scope') not in ('synthetic_observations','hardware_observations'):
        raise ValueError('Explicit S3 offline observation scope required')
    if not np.isfinite(profile.get('requested_band_hz',0)) or profile['requested_band_hz']<=0:raise ValueError('Positive requested band required')
    return profile


def _host_arrays(path):
    if path.suffix=='.npz':
        with np.load(path,allow_pickle=False) as data:return {key:data[key].copy() for key in data.files}
    if path.suffix=='.csv':
        values=np.genfromtxt(path,delimiter=',',names=True,dtype=None,encoding='utf-8',ndmin=1)
        return {name:values[name] for name in values.dtype.names}
    raise ValueError('Host logs require NPZ or named CSV; no implicit format guessing')


def field(raw,spec,count,*,vector=False):
    if spec is None:return np.full((count,14) if vector else (count,),np.nan)
    if isinstance(spec,list):
        if len(spec)!=14:raise ValueError('A vector field needs 14 explicitly ordered columns')
        result=np.column_stack([raw[name] for name in spec])
    else:result=np.asarray(raw[spec])
    if result.shape!=((count,14) if vector else (count,)):raise ValueError('Host log field shape mismatch')
    return result.copy()


def time_values(raw,spec,count):
    if not spec or spec.get('role')!='acquisition':return np.full(count,-1,dtype=np.int64)
    values=field(raw,spec.get('field'),count)
    if values.dtype.kind not in 'iu' or np.any(values<0) or np.any(values>np.iinfo(np.int64).max):raise ValueError('Acquisition times must be nonnegative int64 nanoseconds')
    if not spec.get('clock_id'):raise ValueError('Acquisition clock domain required')
    return values.astype(np.int64)


def import_dynamics(source,profile_path,output,*,source_type):
    source=Path(source).resolve();profile_path=Path(profile_path).resolve();profile=load_profile(profile_path)
    base=profile_path.parent;contract,contract_path=evidence(profile.get('contract'),base,'casbot_observation_contract_v1')
    scale,scale_path=evidence(profile.get('scale_validation'),base,'casbot_scale_validation_v1')
    mapping,mapping_path=evidence(profile.get('clock_mapping'),base,'casbot_clock_mapping_v1')
    inputs={str(profile_path):sha256(profile_path)}
    if source_type=='vendor':
        # S1 is kept immutable, including its conservative capability flags.
        for name,digest in read_json(source/'manifest.json').items():
            if sha256(source/name)!=digest:raise ValueError('S1 normalized data changed')
            inputs[str(source/name)]=digest
        inputs[str(source/'manifest.json')]=sha256(source/'manifest.json')
        raw=_host_arrays(source/'observations.npz');session=read_json(source/'session.json')
        names=session['config']['joint_names'];model_id=session['config']['model_id'];count=len(raw['receive_ns'])
        source_digest=sha256(source/'observations.npz')
        defaults=dict(position='q',velocity='dq',signal='effort_raw',receive='receive_ns')
        capture_scope=session['connection_scope']
    elif source_type=='host':
        if contract is None:raise ValueError('Host logs need an explicit digest-bound field/identity contract')
        raw=_host_arrays(source);inputs[str(source)]=sha256(source);source_digest=sha256(source)
        names=contract['joint_names'];model_id=contract['model_id'];defaults=contract['fields']
        count=len(raw[defaults['position'][0] if isinstance(defaults['position'],list) else defaults['position']])
        capture_scope='external_host_log_contract'
    else:raise ValueError('Source type must be vendor or host')
    if count<2:raise ValueError('At least two original observations required')
    if len(names)!=14 or len(set(names))!=14:raise ValueError('14 unique output-side joint names required')
    if contract is not None:
        if (contract.get('source_sha256')!=source_digest or contract.get('joint_names')!=names or
                contract.get('model_id')!=model_id or contract.get('scope')!=profile['scope']):
            raise ValueError('Observation contract identity/scope/input binding mismatch')
        if source_type=='vendor' and profile['scope']=='hardware_observations' and capture_scope!='hardware_read_only':
            raise ValueError('A localhost Mock capture cannot be upgraded to a hardware observation contract')
        defaults=contract.get('fields',defaults)
        inputs[str(contract_path)]=sha256(contract_path)
    q=field(raw,defaults['position'],count,vector=True).astype(float)
    dq=field(raw,defaults.get('velocity'),count,vector=True).astype(float)
    sensor=field(raw,defaults.get('signal'),count,vector=True).astype(float)
    receive=field(raw,defaults.get('receive'),count)
    if defaults.get('receive') is not None and (receive.dtype.kind not in 'iu' or np.any(receive<0) or np.any(receive>np.iinfo(np.int64).max)):
        raise ValueError('Receive timestamps must be int64 nanoseconds if exposed')
    if defaults.get('receive') is None:receive=np.full(count,-1,dtype=np.int64)
    pt=time_values(raw,contract.get('position_time') if contract else None,count)
    tt=time_values(raw,contract.get('signal_time') if contract else None,count)
    # Mapping is validated later; raw clocks/times always remain separately saved.
    sequence=field(raw,defaults.get('device_sequence'),count)
    if defaults.get('device_sequence') is not None and (sequence.dtype.kind not in 'iu' or np.any(sequence<0) or np.any(sequence>np.iinfo(np.int64).max)):
        raise ValueError('Device sequence must be an integer source field, never callback order')
    sequence=sequence.astype(np.int64) if defaults.get('device_sequence') else np.full(count,-1,dtype=np.int64)
    status=field(raw,defaults.get('status'),count,vector=True).astype(float)
    if defaults.get('status') and (not np.isfinite(status).all() or np.any(status<0) or np.any(status!=np.floor(status))):
        raise ValueError('Exposed status must be finite nonnegative integer flags')
    temperature=field(raw,defaults.get('temperature'),count,vector=True).astype(float)
    torque=np.full_like(sensor,np.nan);conversion=None
    if contract is not None:
        signal=contract.get('signal',{})
        if any(k in signal for k in ('gear_ratio','reduction_ratio','reducer_ratio')):
            raise ValueError('Output-equivalent gain already includes gearing; a second reducer factor is not accepted')
        recognized=signal.get('origin') in ('motor_current','output_torque_estimate','independent_output_torque')
        units=signal.get('unit')==('A' if signal.get('origin')=='motor_current' else 'Nm')
        if recognized and units and signal.get('side')=='output_equivalent' and signal.get('includes')=='total_actuator_output':
            gain=np.asarray(signal.get('gain_nm_per_raw_unit'),dtype=float);zero=np.asarray(signal.get('zero_raw'),dtype=float)
            if gain.shape!=(14,) or zero.shape!=(14,) or not np.isfinite(gain).all() or not np.isfinite(zero).all() or np.any(gain==0):
                raise ValueError('14 finite output-side gains and offsets required; no second gear ratio factor')
            torque=(sensor-zero)*gain
            conversion=dict(gain_nm_per_raw_unit=gain.tolist(),zero_raw=zero.tolist(),reducer_applied=False,
                            independent_torque_sensor=signal['origin']=='independent_output_torque')
    for content,path in ((scale,scale_path),(mapping,mapping_path)):
        if path:inputs[str(path)]=sha256(path)
        if content and (content.get('scope')!=profile['scope'] or content.get('joint_names')!=names or content.get('model_id')!=model_id):
            raise ValueError('Scale/clock evidence belongs to a different scope or identity')
    data=dict(q=q,dq=dq,signal_raw=sensor,torque_nm=torque,position_time_ns=pt,signal_time_ns=tt,
              receive_ns=receive.astype(np.int64),device_sequence=sequence,status_raw=status,temperature_raw=temperature)
    metadata=dict(schema_version='casbot_dynamics_observations_v1',source_type=source_type,source=str(source),scope=profile['scope'],
                  profile=profile,joint_names=names,model_id=model_id,contract=contract,scale_validation=scale,clock_mapping=mapping,
                  conversion=conversion,source_sha256=inputs,units=dict(q='declared rad',dq='declared rad/s',torque='Nm only if conversion established',time='int64 ns'),
                  source_capture_scope=capture_scope,
                  timestamp_policy='raw acquisition fields kept; receive/header not promoted without evidence',
                  hardware_executable=False,physical_semantics_verified_by_software=False)
    if any(sha256(p)!=digest for p,digest in inputs.items()):raise ValueError('Observation input changed while importing')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(output/'observations.npz',**data)
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    (output/'manifest.json').write_text(json.dumps({name:sha256(output/name) for name in ('observations.npz','metadata.json')},indent=2)+'\n')
    return metadata


def load_dynamics(folder):
    root=Path(folder)
    for name,digest in read_json(root/'manifest.json').items():
        if sha256(root/name)!=digest:raise ValueError('Canonical observations changed')
    metadata=read_json(root/'metadata.json')
    if metadata.get('schema_version')!='casbot_dynamics_observations_v1':raise ValueError('Unsupported dynamics observation schema')
    with np.load(root/'observations.npz',allow_pickle=False) as data:arrays={key:data[key].copy() for key in data.files}
    return metadata,arrays
