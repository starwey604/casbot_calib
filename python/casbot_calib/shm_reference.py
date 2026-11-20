"""Frozen septic-Hermite and independent-five-field bundles; no device access."""
import json
import math
from pathlib import Path
import struct
from .shm_data import NAMES, DIRECTION, OFFSET

try:
    import numpy as _np
except ImportError:
    _np = None

try:
    import orjson as _orjson
except ImportError:
    _orjson = None


def snapshot_positions(path):
    path=Path(path)
    if path.is_dir():
        path=path/'native/status.bin' if (path/'native/status.bin').exists() else path/'status.bin'
    data=path.read_bytes()
    if len(data)!=696:
        raise ValueError('Expected a 696-byte status snapshot')
    return [DIRECTION[j]*struct.unpack_from('<i',data,12+(j+12)*16+2)[0]*1e-6+OFFSET[j] for j in range(14)]


def export_motion(motion, *, kp, kd, effort=None, origin='absolute', non_arm_hold=None,
                  observed_velocity_limit_rad_s=None):
    """Preserve Motion's q/dq/ddq/jerk knots, instead of inventing CSV interpolation."""
    meta=motion.metadata()
    times=[round(t*1e9) for t in meta['knots_s']]
    if any(abs(t*1e9-ns)>.001 for t,ns in zip(meta['knots_s'],times)):
        raise ValueError('Motion knots must lie on integer nanoseconds')
    bundle = dict(schema='casbot_shm_reference_v1',origin=origin,joint_names=list(NAMES),knots_ns=times,
                states=meta['states'],kp=list(kp),kd=list(kd),
                effort=effort or [dict(t_ns=0,value=[0.]*14)],purpose='shm_fieldwork_urdf_only')

    if non_arm_hold and non_arm_hold['enabled']:
        if observed_velocity_limit_rad_s is None or not 0 < observed_velocity_limit_rad_s <= 1.5:
            raise ValueError('身体保持参考需要有效的观测速度上限')
        bundle.update(schema='casbot_shm_reference_v2',
                      observed_velocity_limit_rad_s=observed_velocity_limit_rad_s,
                      non_arm_hold=dict(ramp_s=non_arm_hold['ramp_s'],
                                        cst_comm_timeout_ms=non_arm_hold['cst_comm_timeout_ms'],
                                        slots=[dict(slot=int(slot), **fields)
                                               for slot, fields in non_arm_hold['slots'].items()]))
    return bundle


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


def _reject_nonfinite(value):
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError('Reference bundle contains non-finite float (NaN or Inf)')
    elif _np is not None and isinstance(value, _np.ndarray):
        if value.dtype.kind == 'f' and not _np.isfinite(value).all():
            raise ValueError('Reference bundle contains non-finite array values (NaN or Inf)')
    elif _np is not None and isinstance(value, _np.floating):
        if not _np.isfinite(value):
            raise ValueError('Reference bundle contains non-finite float (NaN or Inf)')
    elif isinstance(value, dict):
        for v in value.values():
            _reject_nonfinite(v)
    elif isinstance(value, (list, tuple)):
        for v in value:
            _reject_nonfinite(v)


def _orjson_default(obj):
    if _np is not None:
        if isinstance(obj, _np.ndarray):
            if obj.dtype.kind in 'biuf':
                return _np.ascontiguousarray(obj)
            return obj.tolist()
        if isinstance(obj, _np.generic):
            return obj.item()
    raise TypeError(f'Object of type {type(obj).__name__} is not JSON serializable')


def save_bundle(root, bundle):
    root=Path(root);root.mkdir(parents=True,exist_ok=False)
    target=root/'reference.json'
    tmp=target.with_name(target.name+'.tmp')
    if _orjson is not None:
        raw=_orjson.dumps(bundle,option=_orjson.OPT_SERIALIZE_NUMPY,default=_orjson_default)
        if b'null' in raw:
            _reject_nonfinite(bundle)
        tmp.write_bytes(raw+b'\n')
    else:
        _reject_nonfinite(bundle)
        tmp.write_text(json.dumps(bundle,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n',encoding='utf-8')
    tmp.replace(target)
    return target
