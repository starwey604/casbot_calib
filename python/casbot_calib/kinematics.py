"""Shared Pinocchio geometry and explicit K1 TriCal conventions; no optimiser/IO."""
from copy import deepcopy
from hashlib import sha256
from importlib.resources import files
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from . import _core
from .dual_model import nominal_dual_arm


def digest(value):
    return sha256(json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()).hexdigest()


def vector(value, size=3):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'Expected finite vector[{size}]')
    return result


def pose(translation, rotation_vector):
    result = _core.ArmPose()
    result.translation = vector(translation)
    result.rotation = Rotation.from_rotvec(vector(rotation_vector)).as_matrix()
    return result


def default_config(path=None):
    config = json.loads(files('casbot_calib').joinpath('assets/trical_nominal_v1.json').read_text(encoding='utf-8'))
    if path is not None:
        settings = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        if not isinstance(settings, dict) or set(settings) - {'profile', 'control_period_ns'} or settings.get('profile') != 'trical_nominal_v1':
            raise ValueError('K1 YAML requires profile=trical_nominal_v1; unknown overrides rejected')
        config['control_period_ns'] = settings.get('control_period_ns', config['control_period_ns'])
    validate_config(config)
    return config


def calibration(config):
    indicators = config['indicators']; result = _core.TricalCalibration()
    result.axes = np.asarray(indicators['axes_sensor'], dtype=float)
    result.zero = vector(indicators['centered_readings_m'])
    _core.trical_offset(result.zero, result)  # Native validation, including axis rank.
    return result


def validate_config(config):
    if config['schema_version'] != 1 or config['synthetic_only'] is not True or config['hardware_executable'] is not False:
        raise ValueError('K1 supports synthetic, non-executable profiles only')
    period = config['control_period_ns']
    if type(period) is not int or period <= 0:
        raise ValueError('control_period_ns must be a positive integer')
    if config['parameter_policy']['fit_enabled'] is not False:
        raise ValueError('K1 does not implement identification')
    tools = config['tools']
    if len(tools) != 2 or {t['name'] for t in tools} != {'left_trical_tcp', 'right_trical_tcp'}:
        raise ValueError('Expected two named dual-arm measurement tools')
    for tool in tools:
        pose(tool['translation_m'], tool['rotation_vector_rad'])
    fixture = config['fixture']; pose(fixture['world_translation_m'], fixture['world_rotation_vector_rad'])
    balls = fixture['balls']; ids = [b['id'] for b in balls]
    centers = np.array([vector(b['center_m']) for b in balls])
    if len(balls) < 4 or len(set(ids)) != len(ids) or not all(isinstance(x, str) and x for x in ids):
        raise ValueError('Expected at least four uniquely named datum balls')
    if np.linalg.matrix_rank(centers - centers.mean(axis=0), tol=1e-9) != 3:
        raise ValueError('K1 artifact must span three spatial directions')
    if not np.isfinite(fixture['ball_radius_m']) or fixture['ball_radius_m'] <= 0:
        raise ValueError('Invalid datum ball radius')
    indicators = config['indicators']; calibration(config)
    low = vector(indicators['minimum_readings_m']); high = vector(indicators['maximum_readings_m'])
    zero = vector(indicators['centered_readings_m'])
    if np.any(low >= high) or np.any(zero <= low) or np.any(zero >= high):
        raise ValueError('Mastered readings must lie inside indicator ranges')
    if indicators['channel_names'] != ['x', 'y', 'z']:
        raise ValueError('Expected ordered x,y,z indicator channels')


def ball_world(config, ball_id):
    fixture = config['fixture']; transform = pose(fixture['world_translation_m'], fixture['world_rotation_vector_rad'])
    matches = [b for b in fixture['balls'] if b['id'] == ball_id]
    if len(matches) != 1:
        raise ValueError(f'Unknown datum ball: {ball_id}')
    return transform.rotation @ vector(matches[0]['center_m']) + transform.translation


def add_tools(engine, tools):
    """Attach massless tools to imported frames, retaining fixed-link transforms."""
    model = engine.model; frames = list(model.frames); by_name = {f.name: f for f in frames}
    for tool in tools:
        name, parent = tool['name'], tool['parent_frame']
        if name in by_name or parent not in by_name:
            raise ValueError('Duplicate tool or unknown parent frame')
        relative = pose(tool['translation_m'], tool['rotation_vector_rad']); source = by_name[parent]
        frame = _core.ArmFrame(); frame.name = name; frame.parent = source.parent
        placement = _core.ArmPose()
        placement.rotation = source.placement.rotation @ relative.rotation
        placement.translation = source.placement.translation + source.placement.rotation @ relative.translation
        frame.placement = placement; frames.append(frame); by_name[name] = frame
    model.frames = frames
    return _core.ArmDynamics(model)


def geometry_manifest(engine, geometry_version):
    """Geometry identity excludes inertial/friction values; model format unchanged."""
    model = engine.model
    geometry = dict(base=model.base.matrix().tolist(), legacy_tool=model.tool.matrix().tolist(), parents=list(model.parents),
        joints=[dict(name=j.name, axis=j.axis.tolist(), origin=j.origin.matrix().tolist()) for j in model.joints],
        frames=[dict(name=f.name, parent=f.parent, placement=f.placement.matrix().tolist()) for f in model.frames])
    return dict(schema_version=1, robot_model_id=model.model_id, model_format=model.model_version,
        geometry_version=geometry_version, geometry_sha256=digest(geometry),
        joint_names=[j.name for j in model.joints], frame_names=engine.frame_names(),
        coordinate_convention='T_parent_child maps child to parent; q_model=q_observed, SI; zero corrections baked once into origin',
        geometry=geometry)


def nominal_kinematics(config=None):
    config = default_config() if config is None else deepcopy(config); validate_config(config)
    engine, source = nominal_dual_arm()
    if config['urdf_sha256'] != source['sha256'] or config['robot_model_id'] != engine.model.model_id:
        raise ValueError('Kinematics profile does not match nominal dual-arm source')
    engine = add_tools(engine, config['tools'])
    if config['root_frame'] not in engine.frame_names():
        raise ValueError('Unknown canonical robot root')
    manifest = geometry_manifest(engine, config['geometry_version'])
    manifest.update(urdf_sha256=source['sha256'], profile_sha256=digest(config), synthetic_only=True, hardware_executable=False)
    return engine, manifest


def apply_joint_corrections(engine, corrections):
    """Absolute corrections relative to the supplied nominal model, not encoder IO.

    T_origin' = [R_origin Exp(tilt_local) Exp(axis*zero), t_origin + delta_parent].
    Tilt excludes axial rotation, which is represented only by the zero parameter.
    This candidate parameterisation is not a proof of identifiability or MDH mapping.
    """
    model = engine.model; joints = list(model.joints); names = [j.name for j in joints]
    if set(corrections) - set(names):
        raise ValueError('Unknown joint correction')
    for name, fields in corrections.items():
        if set(fields) - {'zero_offset_rad', 'translation_parent_m', 'tilt_local_rad'}:
            raise ValueError('Unknown correction field')
        index = names.index(name); joint = joints[index]; zero = float(fields.get('zero_offset_rad', 0.))
        tilt = vector(fields.get('tilt_local_rad', [0, 0, 0])); translation = vector(fields.get('translation_parent_m', [0, 0, 0]))
        if not np.isfinite(zero) or abs(tilt @ joint.axis) > 1e-12:
            raise ValueError('Finite zero and tilt perpendicular to nominal joint axis required')
        origin = joint.origin
        origin.rotation = origin.rotation @ Rotation.from_rotvec(tilt).as_matrix() @ Rotation.from_rotvec(joint.axis * zero).as_matrix()
        origin.translation = origin.translation + translation
        joint.origin = origin; joints[index] = joint
    model.joints = joints
    return _core.ArmDynamics(model)
