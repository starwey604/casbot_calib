"""Build the NMPC geometry contract from the selected collision profile."""
import json

import numpy as np
import yaml

from ..model_context import load_context
from ..project_paths import ROOT
from ..shm_ready import ready_position
from .model import arm_chains, fixture_profile, forward
from .qp_options import profile_options

CONFIG = ROOT / 'configs/nmpc/dvt2_teleop.yaml'
OUTPUT = ROOT / 'build/nmpc/dvt2_bare_flange.json'


def fit_capsule(vertices):
    """Winora PCA fit: minimize volume over radii that enclose the convex hull."""
    center = vertices.mean(axis=0)
    relative = vertices - center
    _, basis = np.linalg.eigh(np.cov(relative.T))
    axis = basis[:, -1]
    projection = relative @ axis
    radial2 = np.maximum(0, np.sum(relative**2, axis=1) - projection**2)
    min_radius = np.sqrt(radial2.max()) + .0001
    best_volume = np.inf
    for radius in np.linspace(min_radius, 1.7 * min_radius, 40):
        span = np.sqrt(np.maximum(0, radius**2 - radial2))
        lo, hi = (projection + span).min(), (projection - span).max()
        if lo > hi:
            lo = hi = (lo + hi) / 2
        volume = np.pi * radius**2 * (hi - lo + 4 * radius / 3)
        if volume < best_volume:
            best_volume = volume
            endpoints = center + np.outer([lo, hi], axis)
            best_radius = radius
    return endpoints, best_radius


def point_segment(point, start, end):
    delta = end - start
    t = np.clip(((point - start) @ delta) / max(delta @ delta, 1e-30), 0, 1)
    return np.linalg.norm(point - start - t[..., None] * delta, axis=-1)


def capsule_gap(first, second, radii_sum):
    """Closest point on the interior or one of the four parameter-box edges."""
    a, b = first
    c, d = second
    gap = min(point_segment(a, c, d), point_segment(b, c, d),
              point_segment(c, a, b), point_segment(d, a, b))
    directions = np.column_stack((b - a, c - d))
    params = np.linalg.lstsq(directions, c - a, rcond=None)[0]
    if np.all((params >= 0) & (params <= 1)):
        gap = min(gap, np.linalg.norm(a - c + directions @ params))
    return float(gap - radii_sum)


def pair_thresholds(config, profile):
    overrides = {}
    for row in config['near_pairs']:
        for side in ('left', 'right'):
            pair = tuple(sorted(profile.names.index('robot_' + link.format(side=side) + '_0')
                                for link in row['links']))
            overrides[pair] = row['clearance_m']
    if not overrides.keys() <= set(profile.pairs):
        raise ValueError('Configured clearance refers to an inactive geometry pair')
    return {mode: np.array([min(margin, overrides.get(pair, margin)) for pair in profile.pairs])
            for mode, margin in config['clearance_m'].items()}


def import_model(context, config):
    profile = fixture_profile(context)
    engine = profile.engine
    model = arm_chains(context)
    ready = ready_position()
    starts = np.stack([ready, context.home])
    exact = profile.check(starts)['distances']
    thresholds = pair_thresholds(config, profile)
    geometries = []
    capsule_poses = []
    for g in profile.geometry:
        endpoints, radius = fit_capsule(g.vertices)
        residual = float(point_segment(g.vertices, *endpoints).max() - radius)
        if residual > 1e-10:
            raise ValueError(f'Capsule fails to enclose {g.name}: {residual} m')
        placement = g.placement.matrix()
        if g.parent == -1 and g.attached_to_base:
            placement = engine.model.base.matrix() @ placement
        local = endpoints @ placement[:3, :3].T + placement[:3, 3]
        capsule_poses.append(local)
        geometries.append(dict(name=g.name, parent=g.parent, endpoints=local.tolist(),
                               radius_m=float(radius), max_vertex_residual_m=residual))
    approx = np.empty_like(exact)
    for k, q in enumerate(starts):
        joints, _ = forward(model, q)
        world = []
        for g, points in zip(profile.geometry, capsule_poses):
            if g.parent >= 0:
                transform = joints[g.parent]
                points = points @ transform[:3, :3].T + transform[:3, 3]
            world.append(points)
        for i, (a, b) in enumerate(profile.pairs):
            approx[k, i] = capsule_gap(world[a], world[b],
                                       geometries[a]['radius_m'] + geometries[b]['radius_m'])
    audit = []
    for i, pair in enumerate(profile.pairs):
        modes = {}
        for mode, margins in thresholds.items():
            margin = float(margins[i])
            if not 0 < margin < exact[:, i].min():
                raise ValueError(f'Invalid {mode} start clearance for {pair}: {exact[:, i]} vs {margin}')
            modes[mode] = dict(pair_clearance_m=margin,
                               geometry='capsule' if np.all(approx[:, i] > margin) else 'convex')
        audit.append(dict(indices=list(pair), convex_gap_m=exact[:, i].tolist(),
                          capsule_gap_m=approx[:, i].tolist(), modes=modes))
    robot = yaml.safe_load((ROOT / 'configs/robot/dvt2.yaml').read_text())
    velocity = [min(j.velocity_limit, robot['maximum_velocity_rad_s']) for j in engine.model.joints]
    return dict(**model, model_context=context.identity, q0=ready.tolist(), home=context.home.tolist(),
                velocity=velocity, config=config, qp_options=profile_options(config['qp_profile']),
                scene_geometry_sha256=profile.metadata['geometry_sha256'],
                geometries=geometries, audit_pose_order=['ready', 'home'], pairs=audit)


def write_model(output=OUTPUT):
    spec = import_model(load_context(), yaml.safe_load(CONFIG.read_text()))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(spec, indent=2, allow_nan=False) + '\n')
    return spec
