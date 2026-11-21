"""MuJoCo state display and two draggable flange targets in a separate process."""
import json
import multiprocessing as mp
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from ..fieldwork_display import configure_graphics_environment, lightweight_scene
from ..model_context import load_context
from ..trical.assembly import asset_directory, asset_manifest
from .model import SQUARE_TUBE
from .interaction import ReleaseSnap, tether

TRACK, PAUSE, RETURN_READY = range(3)

def values(array, shape):
    with array.get_lock():
        return np.asarray(array[:]).reshape(shape)


def put(array, value):
    with array.get_lock():
        array[:] = np.asarray(value).ravel()


def make_scene(ctx, spec, folder):
    canonical = folder / 'scene.xml'
    ctx.write_view_scene(canonical)
    scene = lightweight_scene(ctx, canonical)
    tree = ET.parse(scene)
    root = tree.getroot()
    world = root.find('worldbody')
    fixture_assets = asset_directory(ctx.bundle)
    tube = next(p for p in asset_manifest(fixture_assets)['parts'] if p['name'] == SQUARE_TUBE)
    ET.SubElement(root.find('asset'), 'mesh', name='square_tube_mesh',
                  file=str((fixture_assets / tube['mesh']).resolve()))
    ET.SubElement(world, 'geom', name='fixture_square_tube', type='mesh',
                  mesh='square_tube_mesh', rgba='.50 .57 .66 1')
    for side, color in [('left', '.1 .75 1 .7'), ('right', '1 .55 .12 .7')]:
        body = ET.SubElement(world, 'body', name=f'{side}_goal', mocap='true')
        ET.SubElement(body, 'geom', type='sphere', size='.035', rgba=color, group='0',
                      contype='0', conaffinity='0')
        for end, rgb in zip(('0.1 0 0', '0 0.1 0', '0 0 0.1'),
                            ('1 .15 .1 1', '.1 1 .1 1', '.2 .4 1 1')):
            ET.SubElement(body, 'geom', type='capsule', size='.004', fromto='0 0 0 '+end,
                          rgba=rgb, contype='0', conaffinity='0')
    for g in spec['geometries']:
        if g['name'] == 'dvt2_floor_proxy':
            continue
        parent = g['parent']
        body = world if parent < 0 else world.find(f".//body[@name='{spec['joint_names'][parent]}_view']")
        ends = np.asarray(g['endpoints'])
        attrs = dict(size=str(g['radius_m']), rgba='.25 .85 .65 .24', group='4')
        if np.linalg.norm(ends[1]-ends[0]) < 1e-8:
            attrs.update(type='sphere', pos=' '.join(map(str, ends[0])))
        else:
            attrs.update(type='capsule', fromto=' '.join(map(str, ends.ravel())))
        ET.SubElement(body, 'geom', **attrs)
    output = folder / 'teleop.xml'
    tree.write(output, encoding='unicode')
    return output


def render(state, targets, flags, closed, ready, model_file):
    configure_graphics_environment()
    import mujoco
    import mujoco.viewer

    spec = json.loads(model_file.read_text())
    try:
        with tempfile.TemporaryDirectory(prefix='casbot-nmpc-') as temp:
            scene = make_scene(load_context(), spec, Path(temp))
            model = mujoco.MjModel.from_xml_path(str(scene))
            model.vis.quality.offsamples = 0
            data = mujoco.MjData(model)
            qpos = [model.joint(name).qposadr[0] for name in spec['joint_names']]
            ids = [model.body(side+'_goal').mocapid[0] for side in ('left', 'right')]

            spheres = [model.body(side+'_goal').geomadr[0] for side in ('left', 'right')]
            normal_colors = model.geom_rgba[spheres].copy()
            snaps = [ReleaseSnap(), ReleaseSnap()]
            aligned = np.zeros(2, dtype=int)

            def on_key(key):
                if key == 256:  # GLFW_KEY_ESCAPE
                    closed.set()
                with flags.get_lock():
                    if key == ord('R'):
                        flags[1] += 1
                    elif key == ord('O'):
                        flags[3] ^= 1
                    elif key == ord('C'):
                        flags[2] ^= 1
                    elif key == ord('H'):
                        flags[0] = RETURN_READY
                        flags[1] += 1
                    elif key == 32:
                        flags[0] = TRACK if flags[0] == PAUSE else PAUSE
                        flags[1] += 1

            def align(poses):
                data.mocap_pos[ids] = poses[:, :3, 3]
                data.mocap_quat[ids] = Rotation.from_matrix(poses[:, :3, :3]).as_quat(scalar_first=True)

            initial = values(state, -1)
            data.qpos[qpos] = initial[:14]
            align(initial[14:46].reshape(2, 4, 4))
            mujoco.mj_forward(model, data)
            with mujoco.viewer.launch_passive(model, data, key_callback=on_key,
                                               show_left_ui=False, show_right_ui=False) as viewer:
                with viewer.lock():
                    viewer.cam.lookat[:] = [0, 0, 1.07]
                    viewer.cam.distance = 1.8
                    viewer.cam.azimuth = 150
                    viewer.cam.elevation = -9
                    for flag in (mujoco.mjtRndFlag.mjRND_SHADOW, mujoco.mjtRndFlag.mjRND_REFLECTION):
                        viewer.user_scn.flags[flag] = False
                last_reset = last_capsules = last_orientation = -1
                last_indicators = None
                ignore_drag = False
                last_q = initial[:14].copy()
                while viewer.is_running() and not closed.is_set():
                    actual = values(state, -1)
                    paused, reset, capsules, orientation = values(flags, -1)
                    poses = actual[14:46].reshape(2, 4, 4)
                    indicators = tuple(actual[46:].astype(int))
                    changed = indicators != last_indicators or orientation != last_orientation
                    with viewer.lock():
                        moved = not np.array_equal(actual[:14], last_q)
                        if moved:
                            last_q = actual[:14].copy()
                            data.qpos[qpos] = last_q
                        if paused or reset != last_reset:
                            align(poses)
                        if orientation != last_orientation or paused or reset != last_reset:
                            snaps = [ReleaseSnap(), ReleaseSnap()]
                            ignore_drag = bool(viewer.perturb.active)
                        quats = Rotation.from_matrix(poses[:, :3, :3]).as_quat(scalar_first=True)
                        if not orientation or orientation != last_orientation:
                            data.mocap_quat[ids] = quats
                        if changed:
                            labels = ['6D pose' if orientation else '3D position']
                            for arm, side in enumerate(('L', 'R')):
                                joint, pair = indicators[2*arm:2*arm+2]
                                color = normal_colors[arm]
                                if joint >= 0:
                                    color = [1., .85, .12, 1.]
                                    labels.append(f"{side}: {spec['joint_names'][joint]} near limit")
                                if pair >= 0:
                                    color = [1., .18, .12, 1.]
                                    a, b = spec['pairs'][pair]['indices']
                                    names = [spec['geometries'][i]['name'].removeprefix('robot_').removesuffix('_0')
                                             for i in (a, b)]
                                    labels.append(f"{side}: {' / '.join(names)} near clearance")
                                model.geom_rgba[spheres[arm]] = color
                        viewer.opt.geomgroup[4] = capsules
                        if moved or paused or reset != last_reset:
                            mujoco.mj_kinematics(model, data)
                    # Full sync carries option changes. Routine frames only transfer dynamic state.
                    if changed:
                        viewer.set_texts((None, None, '\n'.join(labels), ''))
                    viewer.sync(state_only=capsules == last_capsules and not changed)
                    last_indicators = indicators
                    last_capsules = capsules
                    with viewer.lock():
                        perturb = viewer.perturb
                        dragged = -1
                        if not perturb.active:
                            ignore_drag = False
                        if (not paused and not ignore_drag and reset == last_reset and perturb.active
                                and perturb.select > 0 and model.body_mocapid[perturb.select] >= 0):
                            dragged = ids.index(model.body_mocapid[perturb.select])
                            mujoco.mjv_applyPerturbPose(model, data, perturb, 0)
                        now = time.monotonic()
                        for i, mocap in enumerate(ids):
                            data.mocap_pos[mocap] = tether(data.mocap_pos[mocap], poses[i, :3, 3])
                            if snaps[i].update(i == dragged, poses[i], now):
                                data.mocap_pos[mocap] = poses[i, :3, 3]
                                data.mocap_quat[mocap] = quats[i]
                                aligned[i] += 1
                            if not orientation or orientation != last_orientation:
                                data.mocap_quat[mocap] = quats[i]
                        goals = np.tile(np.eye(4), (2, 1, 1))
                        goals[:, :3, 3] = data.mocap_pos[ids]
                        goals[:, :3, :3] = Rotation.from_quat(data.mocap_quat[ids],
                                                              scalar_first=True).as_matrix()
                    put(targets, np.r_[orientation, aligned, goals.ravel()])
                    last_orientation = orientation
                    last_reset = reset
                    ready.set()
                    closed.wait(1/30)
    finally:
        closed.set()


class MocapViewer:
    def __init__(self, model_file, q, poses):
        ctx = mp.get_context('spawn')
        self.state = ctx.Array('d', 50)
        self.targets = ctx.Array('d', 35)
        self.flags = ctx.Array('i', [0, 0, 0, 0])
        self.closed = ctx.Event()
        self.ready = ctx.Event()
        self.update(q, poses)
        self.aligned = np.zeros(2)
        put(self.targets, np.r_[0, self.aligned, poses.ravel()])
        self.process = ctx.Process(target=render, args=(self.state, self.targets, self.flags,
                                                        self.closed, self.ready, model_file))
        self.process.start()

    def update(self, q, poses, indicators=(-1, -1, -1, -1)):
        put(self.state, np.r_[q, poses.ravel(), indicators])

    def close(self):
        self.closed.set()
        self.process.join(5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join()
