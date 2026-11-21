"""MuJoCo mocap interaction for local kinematics and the SHM physics loop."""
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .mocap_viewer import MocapViewer, PAUSE, RETURN_READY, values
from .runtime import BUILD, Controller
from .interaction import LimitIndicators, tether


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


def interactive_target(viewer, ctl, previous, actual, align):
    packet = values(viewer.targets, -1)
    orientation = bool(packet[0])
    goals = packet[3:].reshape(2, 4, 4)
    if ctl.track_orientation != orientation:
        ctl.set_orientation_tracking(orientation)
        print('\n6D 位姿跟踪' if orientation else '\n3D 位置优先', flush=True)
        previous = previous.copy()
        previous[:, :3, :3] = actual[:, :3, :3]
    snapped = packet[1:3] != viewer.aligned
    viewer.aligned = packet[1:3]
    if align:
        return actual.copy()
    result = filtered_target(previous, goals, ctl.dt, ctl.spec['config']).copy()
    result[snapped] = actual[snapped]
    for arm in range(2):
        result[arm, :3, 3] = tether(result[arm, :3, 3], actual[arm, :3, 3])
    return result


def finish_ready_return(viewer, mode):
    if mode in ('home_done', 'home_blocked'):
        with viewer.flags.get_lock():
            if viewer.flags[0] == RETURN_READY:
                viewer.flags[0] = PAUSE
                viewer.flags[1] += 1
        print('\n准备位参考已到达。' if mode == 'home_done' else '\n回程未通过连续认证，正在停车。',
              '按空格恢复拖动。', flush=True)


def simulate():
    controller = Controller()
    viewer = MocapViewer(BUILD/'model.json', controller.x[:14], controller.poses)
    try:
        while not viewer.ready.wait(.05):
            if not viewer.process.is_alive():
                raise RuntimeError('MuJoCo window failed to initialize')
        indicators = LimitIndicators(controller)
        target = controller.poses.copy()
        ready_q = controller.x[:14].copy()
        last_reset = 0
        last_status = 0.
        print('双击蓝色左手柄或橙色右手柄，Ctrl+右键平移，Ctrl+左键旋转。'
              'Space 暂停，O 切换 3D/6D，H 回准备位，R 对齐，C 显示胶囊，Esc 退出。', flush=True)
        while not viewer.closed.is_set() and viewer.process.is_alive():
            start = time.monotonic()
            paused, reset, _, _ = values(viewer.flags, -1)
            target = interactive_target(viewer, controller, target, controller.poses, paused or reset != last_reset)
            last_reset = reset
            at_rest = (np.max(np.abs(controller.x[14:])) < 1e-10
                       and np.max(np.abs(controller.backup)) < 1e-10)
            at_ready_goal = (np.max(np.abs(controller.x[:14] - ready_q)) < 1e-10
                             and np.max(np.abs(target - controller.poses)) < 1e-10)
            idle = at_rest and paused != RETURN_READY and (paused or at_ready_goal)
            if idle:
                controller.mode = 'paused' if paused else 'ready'
                controller.solve_ms = controller.check_ms = 0.
            else:
                controller.propose(target, stop=bool(paused), return_ready=paused == RETURN_READY)
                controller.commit()
                finish_ready_return(viewer, controller.mode)
                viewer.update(controller.x[:14], controller.poses,
                              indicators.read(controller, controller.x[:14], controller.gaps))
            if start - last_status > .5:
                mode = 'paused' if paused == PAUSE and np.max(np.abs(controller.x[14:])) < 1e-7 else controller.mode
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
            initial = ctl.native.query(loop.initial_q, flange=True)[2]
            viewer = MocapViewer(BUILD/'model.json', loop.initial_q, initial)
            while not viewer.ready.wait(.05):
                if viewer.closed.is_set() or not viewer.process.is_alive():
                    return
                loop.poll()
            if viewer.closed.is_set():
                return
            def show_preparation(q):
                if viewer.closed.is_set() or not viewer.process.is_alive():
                    raise KeyboardInterrupt
                viewer.update(q, ctl.native.query(q, flange=True)[2])

            if loop.needs_ready:
                with viewer.flags.get_lock():
                    viewer.flags[0] = PAUSE
            loop.start(on_preparation=show_preparation)
            ctl = loop.controller
            viewer.update(loop.measured_q, loop.measured_poses)
            if loop.needs_ready:
                with viewer.flags.get_lock():
                    viewer.flags[0] = PAUSE
                    viewer.flags[1] += 1
                print('已到准备位，按空格开始跟踪。', flush=True)
            print(f"端点 {client.ready['endpoint']}  前馈 {loop.ff.source if feedforward else '关闭'}"
                  f"  记录 {client.ready['workspace']}  O 切换 3D/6D，H 回准备位，Space 暂停/恢复，Esc 退出。", flush=True)
            indicators = LimitIndicators(ctl)
            target = ctl.poses.copy()
            reset_seen = 0
            last_status = 0.
            while not viewer.closed.is_set() and viewer.process.is_alive():
                start = time.monotonic()
                paused, reset, _, _ = values(viewer.flags, -1)
                target = interactive_target(viewer, ctl, target, loop.measured_poses, paused or reset != reset_seen)
                reset_seen = reset
                at_rest = np.max(np.abs(ctl.backup)) < 1e-10 and np.max(np.abs(ctl.x[14:])) < 1e-8
                if at_rest and paused != RETURN_READY and (paused or np.max(np.abs(target-ctl.poses)) < 1e-10):
                    loop.feedback(loop.poll(), ctl.x[:14])
                    ctl.mode = 'paused' if paused else 'ready'
                else:
                    loop.step(target, stop=bool(paused), return_ready=paused == RETURN_READY)
                    finish_ready_return(viewer, ctl.mode)
                viewer.update(loop.measured_q, loop.measured_poses,
                              indicators.read(ctl, loop.measured_q, loop.measured_gaps))
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
