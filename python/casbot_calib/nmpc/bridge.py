"""MuJoCo mocap interaction for local kinematics and the SHM physics loop."""
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .mocap_viewer import MocapViewer, values
from .runtime import BUILD, Controller


def filtered_target(previous, target, dt, cfg):
    if np.max(np.abs(target - previous)) < 1e-12:
        return previous
    result = previous.copy()
    delta = target[:, :3, 3] - previous[:, :3, 3]
    step = cfg['mocap_linear_m_s'] * dt
    fraction = np.minimum(1., step/np.maximum(np.linalg.norm(delta, axis=1), 1e-15))
    result[:, :3, 3] += fraction[:, None]*delta
    start = Rotation.from_matrix(previous[:, :3, :3])
    turn = (start.inv() * Rotation.from_matrix(target[:, :3, :3])).as_rotvec()
    angle = cfg['mocap_angular_rad_s'] * dt
    fraction = np.minimum(1., angle/np.maximum(np.linalg.norm(turn, axis=1), 1e-15))
    result[:, :3, :3] = (start * Rotation.from_rotvec(fraction[:, None]*turn)).as_matrix()
    return result


def simulate():
    controller = Controller()
    viewer = MocapViewer(BUILD/'model.json', controller.x[:14], controller.poses)
    try:
        while not viewer.ready.wait(.05):
            if not viewer.process.is_alive():
                raise RuntimeError('MuJoCo window failed to initialize')
        target = controller.poses.copy()
        ready_q = controller.x[:14].copy()
        last_reset = 0
        last_status = 0.
        print('双击蓝色左手柄或橙色右手柄，Ctrl+右键平移，Ctrl+左键旋转。'
              'Space 暂停，R 对齐，C 显示胶囊，Esc 退出。', flush=True)
        while not viewer.closed.is_set() and viewer.process.is_alive():
            start = time.monotonic()
            paused, reset, _ = values(viewer.flags, -1)
            if reset != last_reset or paused:
                target = controller.poses.copy()
            else:
                target = filtered_target(target, values(viewer.targets, (2, 4, 4)),
                                         controller.dt, controller.spec['config'])
            last_reset = reset
            at_rest = (np.max(np.abs(controller.x[14:])) < 1e-10
                       and np.max(np.abs(controller.backup)) < 1e-10)
            at_ready_goal = (np.max(np.abs(controller.x[:14] - ready_q)) < 1e-10
                             and np.max(np.abs(target - controller.poses)) < 1e-10)
            idle = at_rest and (paused or at_ready_goal)
            if idle:
                controller.mode = 'paused' if paused else 'ready'
                controller.solve_ms = controller.check_ms = 0.
            else:
                controller.propose(target, stop=bool(paused))
                controller.commit()
                viewer.update(controller.x[:14], controller.poses)
            if start - last_status > .5:
                mode = 'paused' if paused and np.max(np.abs(controller.x[14:])) < 1e-7 else controller.mode
                print(f'\r{mode:9} 间距 {controller.gaps.min()*1000:5.1f} mm  '
                      f'求解 {controller.solve_ms:4.1f} ms  认证 {controller.check_ms:4.1f} ms  '
                      f'周期计算 {(time.monotonic()-start)*1000:4.1f} ms', end='', flush=True)
                last_status = start
            viewer.closed.wait(max(0., controller.dt - (time.monotonic()-start)))
        if viewer.process.exitcode not in (0, None):
            raise RuntimeError(f'MuJoCo exited with {viewer.process.exitcode}')
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()
        controller.close()
        print()


def physical_simulation(*, host=None, repo='/home/nvidia/casbot_calib_shm', delivery=None, feedforward=True):
    import signal
    from datetime import datetime
    from ..project_paths import ROOT
    from .shm_loop import SHMLoop, make_client

    root = ROOT / 'artifacts/nmpc' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    client = make_client(root, host, repo)
    viewer = None

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signals = (signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT)
    previous = {sig: signal.signal(sig, interrupt) for sig in signals}
    try:
        with SHMLoop(client, delivery=delivery, feedforward=feedforward) as loop:
            ctl = loop.controller
            viewer = MocapViewer(BUILD/'model.json', ctl.x[:14], ctl.poses)
            while not viewer.ready.wait(.05):
                if viewer.closed.is_set() or not viewer.process.is_alive():
                    return
                loop.poll()
            if viewer.closed.is_set():
                return
            loop.start()
            print(f"端点 {client.ready['endpoint']}  前馈 {loop.ff.source if feedforward else '关闭'}"
                  f"  记录 {client.ready['workspace']}", flush=True)
            target = ctl.poses.copy()
            reset_seen = 0
            last_status = 0.
            while not viewer.closed.is_set() and viewer.process.is_alive():
                start = time.monotonic()
                paused, reset, _ = values(viewer.flags, -1)
                if reset != reset_seen or paused:
                    target = loop.measured_poses.copy()
                else:
                    target = filtered_target(target, values(viewer.targets, (2, 4, 4)), ctl.dt,
                                             ctl.spec['config'])
                reset_seen = reset
                at_rest = np.max(np.abs(ctl.backup)) < 1e-10 and np.max(np.abs(ctl.x[14:])) < 1e-8
                if at_rest and (paused or np.max(np.abs(target-ctl.poses)) < 1e-10):
                    loop.feedback(loop.poll(), ctl.x[:14])
                    loop.check_connection(ctl.x[:14])
                    ctl.mode = 'paused' if paused else 'ready'
                else:
                    loop.step(target, stop=bool(paused))
                viewer.update(loop.measured_q, loop.measured_poses)
                if start-last_status >= .5:
                    error = np.max(np.abs(loop.measured_q-ctl.x[:14]))
                    slack = np.min(loop.measured_gaps-ctl.margin)
                    print(f'\r{ctl.mode:9} 实测间距裕量 {1000*slack:5.1f} mm  '
                          f'跟踪误差 {error:.3f} rad  入队确认 {loop.ack_ms:.1f} ms',
                          end='', flush=True)
                    last_status = start
                viewer.closed.wait(max(0., ctl.dt - (time.monotonic()-start)))
    except KeyboardInterrupt:
        pass
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if viewer is not None:
            viewer.close()
        print()
