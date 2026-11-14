"""Immutable raw observations, separately stored truth and explicit SI metadata."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np

try:
    import orjson as _orjson
except ImportError:
    _orjson = None

UNITS = {"q": "rad", "dq": "rad/s", "ddq": "rad/s^2", "current": "A",
         "estimated_torque": "Nm (derived from current; not independent)",
         "time": "int64 ns", "inertia": "kg m^2", "damping": "Nm s/rad",
         "gravity_sin": "Nm", "gravity_cos": "Nm", "spring": "Nm/rad", "gain": "Nm/A output side", "temperature": "deg C"}


def jsonable(value):
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return jsonable(value.tolist())
    if isinstance(value, np.generic):
        return jsonable(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def _orjson_default(obj):
    if isinstance(obj, np.ndarray):
        if obj.dtype.kind in "biuf":
            return np.ascontiguousarray(obj)
        return obj.tolist()
    if isinstance(obj, np.generic):
        return obj.item()
    raise TypeError(f"Object of type {type(obj).__name__} is not JSON serializable")


def write_json(path, data, *, compact=False):
    target = Path(path)
    tmp = target.with_name(target.name + ".tmp")
    if _orjson is not None:
        option = _orjson.OPT_SERIALIZE_NUMPY | _orjson.OPT_NON_STR_KEYS
        if not compact:
            option |= _orjson.OPT_INDENT_2
        raw = _orjson.dumps(data, option=option, default=_orjson_default)
        tmp.write_bytes(raw + b"\n")
    else:
        indent = None if compact else 2
        separators = (",", ":") if compact else None
        text = json.dumps(jsonable(data), ensure_ascii=False, indent=indent, separators=separators, allow_nan=False) + "\n"
        tmp.write_text(text, encoding="utf-8")
    tmp.replace(target)


def save_run(folder, observations, truth, metadata, times=None, commands=None):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(folder/"observations.npz", **observations)
    write_json(folder/"observation_schema.json",{"schema_version":1,"units":UNITS,
        "clock":"virtual monotonic ns; device and host share an origin in this simulator",
        "torque_source":"current_times_output_gain; not an independent sensor"})
    if truth is not None:
        np.savez_compressed(folder/"evaluation_truth.npz", **truth)
    if times is not None:
        np.savez_compressed(folder/"commands.npz", send_ns=times, commands=commands)
    digest = hashlib.sha256((folder/"observations.npz").read_bytes()).hexdigest()
    write_json(folder/"metadata.json", {"schema_version": 1, "model_version": "output_joint_v1",
        "robot_model": "synthetic_single_joint_not_evt6_or_CASBOT20251015",
        "units": UNITS, "raw_sha256": digest, **metadata})


def load_observations(folder):
    folder=Path(folder)
    schema=folder/"observation_schema.json"
    if schema.exists():
        declared=json.loads(schema.read_text(encoding="utf-8"))
        if declared.get("schema_version")!=1:
            raise ValueError("Unsupported observation schema")
        for name in ("q","dq","current","time"):
            if declared.get("units",{}).get(name)!=UNITS[name]:
                raise ValueError(f"Expected SI units for {name}: {UNITS[name]}")
    with np.load(folder/"observations.npz", allow_pickle=False) as arrays:
        return {k: arrays[k] for k in arrays.files}
