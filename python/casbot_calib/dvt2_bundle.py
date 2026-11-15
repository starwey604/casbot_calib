"""Frozen N2 bundles, loaded through the existing native dynamics/collision kernel.

Python Pinocchio is only needed to build/verify a bundle, not to consume one.
"""
from copy import copy, deepcopy
import json
from hashlib import sha256
from pathlib import Path
import numpy as np
from . import _core
from .collision import CollisionProfile, box_geometry
from .kinematics import geometry_manifest, digest
from .model_sources import _resource

_BUNDLE_CACHE = {}
_PROFILE_CACHE = {}


def _bundle_fingerprint(root: Path):
    try:
        meta_path = root / 'model.json'
        st_meta = meta_path.stat()
        rels = ['model.json', 'model.urdf', 'collision.json', 'collision.npz', 'trical_scene.json']
        stats = []
        for rel in rels:
            p = root / rel
            st = p.stat()
            stats.append((rel, st.st_mtime_ns, st.st_size))
        return (str(root), st_meta.st_mtime_ns, tuple(stats))
    except OSError:
        return None
    except Exception:
        return None


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def bundle_identity(root, metadata):
    """Bind the profile, collision exclusions and scene into one batch identity."""
    return digest(dict(
        descriptor={k: v for k, v in metadata.items() if k != 'bundle_sha256'},
        collision=read(Path(root) / 'collision.json'),
        trical=read(Path(root) / 'trical_scene.json'),
    ))



def _bind_frozen_frames(engine, frozen):
    """Restore frozen fixed frames after verified URDF parsing on another CPU.

    Moving-axis geometry remains exact. Only fixed-frame roundoff within eight
    float64 epsilons is accepted, then replaced, not retained as a new identity.
    """
    actual = geometry_manifest(engine, 'dvt2_locked_v1')
    if actual['geometry_sha256'] == frozen['geometry_sha256']:
        return engine
    expected, current = frozen['geometry'], actual['geometry']
    if ({k:v for k,v in expected.items() if k != 'frames'} !=
            {k:v for k,v in current.items() if k != 'frames'}):
        raise ValueError('Derived geometry differs from frozen bundle: moving geometry')
    if len(expected['frames']) != len(current['frames']):
        raise ValueError('Derived geometry differs from frozen bundle: frame count')
    model = engine.model
    frames = model.frames
    for frame, ref, observed in zip(frames, expected['frames'], current['frames']):
        if ref['name'] != observed['name'] or ref['parent'] != observed['parent']:
            raise ValueError('Derived geometry differs from frozen bundle: frame topology')
        target, value = np.asarray(ref['placement']), np.asarray(observed['placement'])
        if not np.allclose(target, value, rtol=0, atol=8*np.finfo(np.float64).eps):
            raise ValueError('Derived geometry differs from frozen bundle: '+frame.name)
        placement = _core.ArmPose()
        placement.rotation, placement.translation = target[:3,:3], target[:3,3]
        frame.placement = placement
    model.frames = frames
    engine = _core.ArmDynamics(model)
    if geometry_manifest(engine, 'dvt2_locked_v1')['geometry_sha256'] != frozen['geometry_sha256']:
        raise ValueError('Derived geometry differs from frozen bundle after frame restoration')
    return engine


def load_bundle(folder):
    root = Path(folder).resolve()
    fp = _bundle_fingerprint(root)
    if fp is not None and fp in _BUNDLE_CACHE:
        cached_model, cached_meta = _BUNDLE_CACHE[fp]
        return _core.ArmDynamics(cached_model), deepcopy(cached_meta)
    metadata = read(root / 'model.json')
    if metadata['schema_version'] != 'casbot_dvt2_bundle_v1':
        raise ValueError('Unsupported DVT2 bundle')
    if metadata.get('bundle_sha256') != bundle_identity(root, metadata):
        raise ValueError('Bundle profile, exclusions or scene identity changed')
    if sha256(_resource(root / 'model.urdf').read_bytes()).hexdigest() != metadata['urdf_sha256']:
        raise ValueError('Derived URDF identity changed')
    engine = _core.ArmDynamics.from_urdf(str(root / 'model.urdf'))
    model = engine.model
    model.gravity_world = np.array(metadata['profile']['gravity_world_m_s2'])
    engine = _core.ArmDynamics(model)
    if model.model_id != metadata['model_id'] or metadata['profile']['model_id'] != metadata['model_id']:
        raise ValueError('Derived model identities differ')
    if [j.name for j in model.joints] != metadata['joint_names']:
        raise ValueError('Derived joint order changed')
    engine = _bind_frozen_frames(engine, metadata['geometry'])
    if fp is not None:
        _BUNDLE_CACHE[fp] = (engine.model, deepcopy(metadata))
    return engine, metadata


def robot_profile(folder, *, environment=True):
    root = Path(folder).resolve()
    fp = _bundle_fingerprint(root)
    cache_key = (fp, bool(environment)) if fp is not None else None
    if cache_key is not None and cache_key in _PROFILE_CACHE:
        cached = _PROFILE_CACHE[cache_key]
        cloned = copy(cached)
        cloned.metadata = deepcopy(cached.metadata)
        if hasattr(cached, 'pair_clearance_m'):
            cloned.pair_clearance_m = cached.pair_clearance_m.copy()
        return cloned
    engine, metadata = load_bundle(root)
    cache = read(root / 'collision.json')
    if cache['urdf_sha256'] != metadata['urdf_sha256']:
        raise ValueError('Collision/URDF identity mismatch')
    array_path = _resource(root / 'collision.npz')
    if sha256(array_path.read_bytes()).hexdigest() != cache['array_sha256']:
        raise ValueError('Collision arrays changed')
    geometry = []
    with np.load(array_path, allow_pickle=False) as arrays:
        for row in cache['geometries']:
            g = _core.CollisionGeometry()
            g.name = row['name']
            g.parent = row['parent_joint_index']
            g.vertices = arrays[row['array_key'] + '_vertices']
            g.triangles = arrays[row['array_key'] + '_triangles']
            geometry.append(g)
    indices = {g.name: i for i, g in enumerate(geometry)}
    excluded = [(indices[row['first']], indices[row['second']]) for row in cache['excluded_pairs']]
    if environment:
        for row in metadata['scene']['obstacles']:
            g = box_geometry(row['name'], [0, 0, 0], row['half_extents'], False)
            pose = _core.ArmPose()
            pose.translation = np.array(row['center'])
            pose.rotation = np.asarray(row.get('rotation', np.eye(3)))
            g.placement = pose
            geometry.append(g)
    built = CollisionProfile(engine, geometry, excluded, {
        **cache,
        'geometry_sha256': metadata['geometry']['geometry_sha256'],
        'scene': metadata['scene'],
        'actual_geometry_count': len(geometry),
        'model_id': engine.model.model_id,
    })
    if cache_key is not None:
        _PROFILE_CACHE[cache_key] = built
        cloned = copy(built)
        cloned.metadata = deepcopy(built.metadata)
        return cloned
    return built


def trical_scene(folder):
    load_bundle(folder)
    return read(Path(folder) / 'trical_scene.json')


def check_viewer_assets(folder):
    root = Path(folder)
    metadata = read(root / "model.json")
    if 'viewer_sha256' in metadata:
        if sha256(_resource(root / 'scene.xml').read_bytes()).hexdigest() != metadata['viewer_sha256']:
            raise ValueError('Viewer asset identity changed')
    for filename,expected in metadata.get('viewer_files_sha256',{}).items():
        if sha256(_resource(root/filename).read_bytes()).hexdigest()!=expected:
            raise ValueError('Viewer asset identity changed: '+filename)


def viewer_adapter(folder):
    """Use the same logical state mapping without regenerating the CAD scene."""
    from .telemetry import TelemetryAdapter, JointMapping
    check_viewer_assets(folder)
    engine, _ = load_bundle(folder)
    return TelemetryAdapter(engine.model.model_id,
                            [JointMapping(j.name, j.name) for j in engine.model.joints])
