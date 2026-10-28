"""Independent static position-meter input; no instrument or robot SDK here."""
from hashlib import sha256
import json
from pathlib import Path
import numpy as np
from .data import write_json
from .kinematics import geometry_manifest
from .kinematic_identification import residual_metrics


UNITS={'q_rad':'rad','dq_rad_s':'rad/s','position_meter_m':'m','time':'ns'}


def validate_meter(arrays,metadata):
    if metadata.get('schema')!='kinematic_external_position_v1' or metadata.get('units')!=UNITS:raise ValueError('External metrology schema/SI units mismatch')
    if metadata.get('registration_frozen_before_calibration') is not True or not metadata.get('registration_source') or not metadata.get('registration_id'):
        raise ValueError('Independent frozen meter-to-robot registration required')
    if metadata.get('registration_estimated_from_validation') is not False:raise ValueError('Do not register using validation outcomes')
    if metadata.get('timing')!='common_clock_static_hold' or not metadata.get('clock_domain'):raise ValueError('Known common static measurement clock required')
    if not metadata.get('source_batch_id') or type(metadata.get('synthetic_only')) is not bool:raise ValueError('Explicit independent batch and source type required')
    transform=np.array(metadata['robot_from_meter'],dtype=float)
    if transform.shape!=(4,4) or not np.isfinite(transform).all() or not np.allclose(transform[3],[0,0,0,1],atol=1e-12,rtol=0):raise ValueError('Invalid external registration')
    if not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-10,rtol=0) or abs(np.linalg.det(transform[:3,:3])-1)>1e-10:raise ValueError('External registration is not a rigid rotation')
    sigma=metadata.get('measurement_std_m')
    if isinstance(sigma,bool) or not isinstance(sigma,(int,float)) or not np.isfinite(sigma) or sigma<=0:raise ValueError('Declared meter uncertainty required')
    n=len(arrays['q_rad'])
    if n<18:raise ValueError('At least three postures per arm with three measurements each required')
    for key,width in [('q_rad',14),('dq_rad_s',14),('position_meter_m',3)]:
        a=np.asarray(arrays[key])
        if a.shape!=(n,width) or not np.isfinite(a).all():raise ValueError('Invalid meter array: '+key)
    if np.max(abs(arrays['dq_rad_s']))>1e-3:raise ValueError('External meter record is not a static hold')
    for key in ('joint_sample_ns','joint_receive_ns','meter_sample_ns','meter_receive_ns','sequence'):
        a=np.asarray(arrays[key])
        if a.shape!=(n,) or a.dtype!=np.dtype('int64') or np.any(a<0):raise ValueError('External timestamps/sequence must be nonnegative int64')
    if np.any(np.diff(arrays['sequence'])<=0):raise ValueError('Duplicated external measurement packets')
    for source in ('joint','meter'):
        if np.any(arrays[source+'_receive_ns']<arrays[source+'_sample_ns']):raise ValueError('External receive precedes known sampling time')
    if np.any(abs(arrays['joint_sample_ns']-arrays['meter_sample_ns'])>1_000_000):raise ValueError('External static samples are not aligned within declared 1ms scope')
    for key in ('tool_frame','point_id'):
        a=np.asarray(arrays[key])
        if a.shape!=(n,) or a.dtype.kind!='U' or any(not x for x in a):raise ValueError('Invalid external measurement identity')
    if set(arrays['tool_frame'])!={'left_trical_tcp','right_trical_tcp'}:raise ValueError('External validation must cover both tools')
    if arrays['status'].shape!=(n,) or np.any(arrays['status']!=0):raise ValueError('External records include invalid observations')
    for frame in np.unique(arrays['tool_frame']):
        selected=arrays['tool_frame']==frame;points=np.unique(arrays['point_id'][selected])
        if len(points)<3:raise ValueError('Too few independent external postures')
        means=[]
        for point in points:
            rows=selected&(arrays['point_id']==point)
            if np.count_nonzero(rows)<3:raise ValueError('Too few repeated external measurements')
            if np.max(np.ptp(arrays['q_rad'][rows],axis=0))>.002:raise ValueError('External repeat point mixes different postures')
            means.append(arrays['q_rad'][rows].mean(axis=0))
        for i,q in enumerate(means):
            if any(np.linalg.norm(q-other)<.01 for other in means[:i]):raise ValueError('Duplicated external postures')
    return transform


def save_meter(folder,arrays,metadata):
    validate_meter(arrays,metadata);folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(folder/'observations.npz',**arrays);write_json(folder/'metadata.json',metadata)
    write_json(folder/'manifest.json',dict(files={name:sha256((folder/name).read_bytes()).hexdigest() for name in ('observations.npz','metadata.json')}))


def load_meter(folder):
    root=Path(folder);manifest=json.loads((root/'manifest.json').read_text())
    if set(manifest['files'])!={'observations.npz','metadata.json'}:raise ValueError('External meter manifest incomplete')
    for name,expected in manifest['files'].items():
        if sha256((root/name).read_bytes()).hexdigest()!=expected:raise ValueError('External meter source digest mismatch')
    metadata=json.loads((root/'metadata.json').read_text())
    with np.load(root/'observations.npz',allow_pickle=False) as raw:arrays={key:raw[key] for key in raw.files}
    transform=validate_meter(arrays,metadata);return arrays,metadata,transform


def evaluate_meter(folder,parameters,scaled,*,excluded_batch_ids=(),position_limit_m=.0004,repeatability_limit_m=.0001):
    arrays,metadata,t=load_meter(folder)
    if metadata['source_batch_id'] in excluded_batch_ids:raise ValueError('External metrology reuses a calibration batch')
    if metadata['joint_names']!=[j.name for j in parameters.nominal.model.joints]:raise ValueError('External joint order mismatch')
    if metadata['source_geometry_sha256']!=geometry_manifest(parameters.nominal,'nominal')['geometry_sha256']:raise ValueError('External source geometry mismatch')
    points,_=parameters.evaluate(scaled,arrays['q_rad'],arrays['tool_frame'],np.zeros((len(arrays['q_rad']),3)))
    measured=arrays['position_meter_m']@t[:3,:3].T+t[:3,3];residual=points-measured;repeats=[]
    for frame in np.unique(arrays['tool_frame']):
        for point in np.unique(arrays['point_id'][arrays['tool_frame']==frame]):
            selected=(arrays['tool_frame']==frame)&(arrays['point_id']==point)
            repeats.append(dict(tool_frame=str(frame),point_id=str(point),samples=int(selected.sum()),
                residual_std_m=float(np.sqrt(np.var(residual[selected],axis=0,ddof=1).sum()))))
    metrics=residual_metrics(residual);maximum=max(r['residual_std_m'] for r in repeats)
    checks=dict(position_rmse=metrics['position_rmse_m']<=position_limit_m,residual_repeatability=maximum<=repeatability_limit_m)
    return dict(status='measured_synthetic' if metadata['synthetic_only'] else 'measured_external',passed=all(checks.values()),checks=checks,
        position=metrics,repeated_points=repeats,measurement_std_m=metadata['measurement_std_m'],
        position_limit_m=position_limit_m,repeatability_limit_m=repeatability_limit_m,
        source_manifest_sha256=sha256((Path(folder)/'manifest.json').read_bytes()).hexdigest(),
        registration_id=metadata['registration_id'],truth_read=False,
        scope='Independent registered position observations and repeat residual standard deviation; not ISO AP/RP or orientation acceptance')
