"""Frozen septic-Hermite and independent-five-field bundles; no device access."""
import json
import math
from pathlib import Path
import struct
from .shm_data import NAMES, DIRECTION, OFFSET


def snapshot_positions(path):
    path=Path(path)
    if path.is_dir():
        path=path/'native/status.bin' if (path/'native/status.bin').exists() else path/'status.bin'
    data=path.read_bytes()
    if len(data)!=696:
        raise ValueError('Expected a 696-byte status snapshot')
    return [DIRECTION[j]*struct.unpack_from('<i',data,12+(j+12)*16+2)[0]*1e-6+OFFSET[j] for j in range(14)]


def export_motion(motion, *, kp, kd, effort=None, origin='absolute'):
    """Preserve Motion's q/dq/ddq/jerk knots, instead of inventing CSV interpolation."""
    meta=motion.metadata()
    times=[round(t*1e9) for t in meta['knots_s']]
    if any(abs(t*1e9-ns)>.001 for t,ns in zip(meta['knots_s'],times)):
        raise ValueError('Motion knots must lie on integer nanoseconds')
    return dict(schema='casbot_shm_reference_v1',origin=origin,joint_names=list(NAMES),knots_ns=times,
                states=meta['states'],kp=list(kp),kd=list(kd),
                effort=effort or [dict(t_ns=0,value=[0.]*14)],purpose='shm_fieldwork_urdf_only')


def jog_bundle(q, joint, amplitude, segment_s, kp=20, kd=2, *, relative=False):
    if len(q)!=14 or joint not in range(12,26) or not 0<amplitude<=.5 or segment_s<1:
        raise ValueError('Explicit 14-axis start, arm ID, amplitude(0,.5], segment>=1 required')
    if not all(math.isfinite(v) for v in [*q,amplitude,segment_s,kp,kd]) or not 0<=kp<=500 or not 0<=kd<=50:
        raise ValueError('Invalid reference values/gains')
    states=[]
    for offset in (0.,-amplitude,amplitude,0.):
        position=list(q);position[joint-12]+=offset
        states.append([position,[0.]*14,[0.]*14,[0.]*14])
    segment_ns=round(segment_s*1e9)
    return dict(schema='casbot_shm_reference_v1',origin='measured_start' if relative else 'absolute',
                joint_names=list(NAMES),knots_ns=[i*segment_ns for i in range(4)],states=states,
                kp=[kp]*14,kd=[kd]*14,effort=[dict(t_ns=0,value=[0.]*14)],purpose='shm_fieldwork_urdf_only')


def fields_bundle(q, joint, *, position=None, velocity=0, effort=0, kp=20,kd=2,duration=3):
    if len(q)!=14 or joint not in range(12,26) or not 0<duration<=3600:
        raise ValueError('Invalid direct-field start/ID/duration')
    position_values=list(q)
    if position is not None:position_values[joint-12]=position
    speeds=[0.]*14;torque=[0.]*14;speeds[joint-12]=velocity;torque[joint-12]=effort
    values=[*position_values,velocity,effort,kp,kd,duration]
    if not all(math.isfinite(v) for v in values) or not 0<=kp<=500 or not 0<=kd<=50:
        raise ValueError('Invalid direct fields')
    return dict(schema='casbot_shm_fields_v1',joint_names=list(NAMES),duration_ns=round(duration*1e9),
                position=position_values,velocity=speeds,effort=torque,kp=[kp]*14,kd=[kd]*14,
                purpose='shm_fieldwork_urdf_only',transition='direct_fields_no_hidden_smoothing')


def save_bundle(root, bundle):
    root=Path(root);root.mkdir(parents=True,exist_ok=False)
    (root/'reference.json').write_text(json.dumps(bundle,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return root/'reference.json'
