#!/usr/bin/env python3
"""Compare the packaged 14-axis engine with Python Pinocchio/Coal in a fresh process."""
import json
import os
from pathlib import Path
import sys


def main():
    output, order = Path(sys.argv[1]), sys.argv[2]
    if order == 'core-first':
        from casbot_calib import _core
    import numpy as np
    import scipy.linalg
    import pinocchio as pin
    import coal
    if order == 'math-first':
        from casbot_calib import _core
    from casbot_calib.dvt2_bundle import load_bundle, robot_profile
    from casbot_calib.model_sources import model_directory
    from casbot_calib.model_context import BUNDLE_RELATIVE

    root = model_directory() / BUNDLE_RELATIVE
    engine, metadata = load_bundle(root)
    profile = robot_profile(root)
    rng = np.random.default_rng(20261115)
    q = np.array(metadata['profile']['home_rad']) + rng.uniform(-.2, .2, (32, 14))
    velocity = rng.uniform(-.3, .3, q.shape)
    acceleration = rng.uniform(-.8, .8, q.shape)
    model = pin.buildModelFromUrdf(str(root / 'model.urdf'))
    model.gravity.linear = engine.model.gravity_world
    data = model.createData()
    assert list(model.names)[1:] == [joint.name for joint in engine.model.joints]
    expected_rnea = np.stack([pin.rnea(model, data, x, v, a).copy()
                              for x, v, a in zip(q, velocity, acceleration)])
    before = {item.name for item in Path('/proc/self/task').iterdir()}
    # Each kernel is first in one import-order case, exposing its worker creation.
    if order == 'math-first':
        collision = profile.check(q)
        after = {item.name for item in Path('/proc/self/task').iterdir()}
        torque = engine.evaluate_batch(q, velocity, acceleration)['total']
    else:
        torque = engine.evaluate_batch(q, velocity, acceleration)['total']
        after = {item.name for item in Path('/proc/self/task').iterdir()}
        collision = profile.check(q)
    threads = int(os.environ['OMP_NUM_THREADS'])
    assert len(after - before) == threads - 1, (threads, before, after)
    serial = np.concatenate([profile.check(row[None, :])['distances'] for row in q])

    geometries = profile.geometry
    shapes = []
    for geometry in geometries:
        vertices, triangles = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
        for vertex in geometry.vertices:
            vertices.append(vertex)
        for face in geometry.triangles:
            triangles.append(coal.Triangle(*map(int, face)))
        shapes.append(coal.Convex(vertices, triangles))
    expected_distance = np.empty_like(collision['distances'])
    request = coal.DistanceRequest(True, True)
    base = engine.model.base.matrix()
    placements = [geometry.placement.matrix() for geometry in geometries]
    fixed = {}
    for index, geometry in enumerate(geometries):
        if geometry.parent < 0:
            world = (base if geometry.attached_to_base else np.eye(4)) @ placements[index]
            fixed[index] = coal.Transform3s(world[:3, :3], world[:3, 3])
    for row_index, row in enumerate(q):
        pin.forwardKinematics(model, data, row)
        transforms = []
        for index, geometry in enumerate(geometries):
            if geometry.parent < 0:
                transforms.append(fixed[index])
            else:
                world = data.oMi[geometry.parent + 1].homogeneous @ placements[index]
                transforms.append(coal.Transform3s(world[:3, :3], world[:3, 3]))
        for pair_index, (first, second) in enumerate(profile.pairs):
            result = coal.DistanceResult()
            expected_distance[row_index, pair_index] = coal.distance(
                shapes[first], transforms[first], shapes[second], transforms[second], request, result)
    errors = {
        'rnea_vs_pinocchio': float(np.max(np.abs(torque - expected_rnea))),
        'collision_vs_serial': float(np.max(np.abs(collision['distances'] - serial))),
    }
    errors['collision_vs_coal'] = float(np.max(np.abs(collision['distances'] - expected_distance)))
    np.testing.assert_array_equal(collision['worst_pair'], np.argmin(expected_distance, axis=1))
    assert all(value < 1e-12 for value in errors.values()), errors
    np.testing.assert_allclose(scipy.linalg.solve(np.diag([2., 3.]), [4., 9.]), [2., 3.], rtol=0, atol=1e-14)
    import runpy
    regression = runpy.run_path(str(Path(__file__).resolve().parents[2] / 'tests/test_collision_solver.py'))
    regression['test_h3_separated_base_and_shoulder_distance_stays_positive']()
    runtime = Path(_core.__file__).parent / '.libs/libomp.so.5'
    libraries = sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                        if any(name in line for name in ('libomp.so', 'libgomp.so', 'openblas'))})
    assert [path for path in libraries if 'libomp.so' in path] == [str(runtime)], libraries
    np.savez(output.with_suffix('.npz'), torque=torque, distances=collision['distances'],
             worst_pair=collision['worst_pair'], reference_distances=expected_distance)
    report = dict(threads=threads, order=order, first_kernel='collision' if order == 'math-first' else 'rnea',
                  new_worker_tids=sorted(after - before), rows=len(q), joints=q.shape[1],
                  collision_pairs=len(profile.pairs), coal_reference_rows=len(q), collision_counterexample='passed', errors=errors, libraries=libraries)
    output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
