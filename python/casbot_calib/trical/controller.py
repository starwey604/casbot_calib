"""Interactive native executor supervision and measured-window acquisition."""
from collections import deque
import json
from pathlib import Path
import queue
import subprocess
import threading
import time
import numpy as np

from .. import _core
from ..shm_ready import ready_position
from .assembly import ball_center, indicator_calibration
from .indicators import MockQasBridge, UsbBridgeIndicatorDriver
from .window import SettledWindowCollector


class Executor:
    def __init__(self, binary, endpoint, context, robot, recipe, side, output):
        self.blocks = deque(maxlen=8192)
        self.events = queue.Queue()
        self.replies = {}
        self.identifier = 0
        self.q = self.hold = None
        self.done = False
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        self.errors = open(output.parent/'executor.log', 'w')
        csv = lambda values: ','.join(map(str, values))
        command = [str(binary), '--shm', endpoint, '--urdf', str(context.urdf), '--output', str(output),
                   '--bare-flange', '--tool-side', side, '--kp', csv(robot['kp']), '--kd', csv(robot['kd']),
                   '--gravity', csv(robot['gravity_world']), '--tool-mass-kg', str(recipe['tool_mass_kg']),
                   '--tool-com-m', csv(recipe['tool_com_m']), '--max-windup-rad', str(recipe['max_windup_rad'])]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.errors, text=True, bufsize=1)
        def read():
            try:
                for line in self.process.stdout:
                    self.events.put(json.loads(line))
            except Exception as error:
                self.events.put(error)
            finally:
                self.events.put(None)
        self.reader = threading.Thread(target=read)
        self.reader.start()
        try:
            ready = self.wait('ready', timeout=10)
            self.hold = self.q = np.array(ready['q0'])
        except BaseException:
            self.close()
            raise

    def pump(self):
        while not self.events.empty():
            event = self.events.get_nowait()
            if event is None:
                if not self.done: raise RuntimeError('TriCal 原生执行器退出；查看 executor.log')
                continue
            if isinstance(event, Exception): raise event
            kind = event['type']
            if kind == 'state':
                self.blocks.append(event)
                self.q = np.array(event['q_mean'])
            elif kind == 'finished':
                self.done = True
                self.replies[(kind, 0)] = event
                if not event['ok']: raise RuntimeError('TriCal 执行中止：'+event['reason'])
            else:
                self.replies[(kind, event.get('id', 0))] = event

    def wait(self, kind, identifier=0, timeout=10):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.pump()
            if (kind, identifier) in self.replies:
                return self.replies.pop((kind, identifier))
            time.sleep(.002)
        raise TimeoutError(f'TriCal {kind} 超时')

    def request(self, kind, **values):
        self.identifier += 1
        self.process.stdin.write(json.dumps(dict(id=self.identifier, type=kind, **values))+'\n')
        self.process.stdin.flush()
        reply = self.wait('reply', self.identifier)
        if not reply['ok']: raise RuntimeError('原生段拒绝：'+reply['reason'])
        return self.identifier

    def move(self, target, *, windup=False, velocity=.35):
        target = np.asarray(target)
        distance = np.max(abs(target-self.hold))
        seconds = max(.25, 2.1875*distance/velocity, np.sqrt(7.514*distance), np.cbrt(52.5*distance/8))
        identifier = self.request('segment', q_from=self.hold.tolist(), q_to=target.tolist(),
                                  duration_ns=int(np.ceil(seconds*1000))*1_000_000, windup=windup)
        self.wait('segment_done', identifier, seconds+5)
        self.hold = target.copy()
        self.blocks.clear()

    def finish(self):
        self.request('finish')
        self.wait('finished', timeout=5)
        if self.process.wait(timeout=5) != 0:
            raise RuntimeError('TriCal 执行器结束失败')

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            self.process.wait(timeout=5)
        self.reader.join(timeout=5)
        self.process.stdout.close()
        if not self.process.stdin.closed: self.process.stdin.close()
        self.errors.close()


def feedback_means(path, windows):
    records = _core.read_feedback_csv_zst(path)
    if records['truncated_tail']:
        raise ValueError('SHM反馈记录尾部不完整')
    # A queued status may be returned repeatedly; retain the first receipt of each run.
    counters = records['frame_counter']
    unique = np.ones(len(counters), dtype=bool)
    unique[1:] = counters[1:] != counters[:-1]
    positions = records['q'][unique]
    times = records['receive_mono_ns'][unique]
    means = []
    for row in windows:
        selected = (times > row['t_start_ns']) & (times <= row['t_end_ns'])
        if not selected.any(): raise ValueError('测量窗口没有 feedback.csv.zst 帧')
        means.append(positions[selected].mean(axis=0).tolist())
    return means


def correction(engine, q, side, offset, maximum_step, max_windup, hold):
    if not np.isfinite(maximum_step) or maximum_step <= 0:
        raise ValueError('修正步长须为正有限值')
    frame = side+'_trical_tcp'
    pose = engine.frame_pose(q, frame)
    step = pose[:3, :3] @ np.asarray(offset)
    step *= min(1., maximum_step / max(np.linalg.norm(step), 1e-12))
    ix = slice(0, 7) if side == 'left' else slice(7, 14)
    j = engine.frame_jacobian(q, frame)[:3, ix]
    # Prefer shoulder/elbow motion when the low-gain wrist remains in stiction.
    weights = np.array([1., 1., 1., 1., .1, .1, .1])
    weighted = j * weights
    dq = weighted.T @ np.linalg.solve(weighted@j.T + 1e-6*np.eye(3), step)
    dq *= min(1., max_windup / max(np.max(abs(dq)), 1e-12))
    target = np.array(hold)
    if np.max(abs(target-q)) > max_windup:
        raise RuntimeError('保持指令与观测差已超过 windup 限值')
    # Increment the held command to overcome static friction, within the observed bound.
    target[ix] = np.clip(target[ix]+dq, q[ix]-max_windup, q[ix]+max_windup)
    origin = engine.frame_pose(hold, frame)[:3,3]
    while True:
        distance = np.linalg.norm(engine.frame_pose(target, frame)[:3,3]-origin)
        if distance <= maximum_step:
            return target
        target = np.asarray(hold) + (target-hold)*(.999*maximum_step/distance)


def mock_contact(engine, executor, side, ball, cal, hidden_offsets):
    q = executor.q + hidden_offsets
    pose = engine.frame_pose(q, side+'_trical_tcp')
    d = pose[:3, :3].T @ (ball-pose[:3, 3])
    readings = cal.axes @ d + cal.zero
    # Finite planar faces and 14 mm plunger travel.
    tangential = np.sqrt(np.maximum(0., d@d - (cal.axes@d)**2))
    valid = (tangential <= .00496) & (abs(readings-cal.zero) <= .007)
    return readings, ['VALID' if v else 'OUT_OF_RANGE' for v in valid]


def validate_entry(before, middle, pose_before, pose_middle, seed_pose, ball, calibration):
    predicted = calibration.axes @ (
        pose_middle[:3,:3].T @ (ball-pose_middle[:3,3])
        - pose_before[:3,:3].T @ (ball-pose_before[:3,3]))
    observed = np.array(middle['readings_m']) - before['readings_m']
    if (np.any(abs(observed-predicted) > .00075)
            or np.any((abs(predicted) > .0002) & (abs(observed) < abs(predicted)*.25))):
        raise RuntimeError('首点两步入球读数变化不符，检查测头接触与槽位映射')
    offset = _core.trical_offset(middle['readings_m'], calibration)
    future = offset + pose_middle[:3,:3].T @ (pose_middle[:3,3]-seed_pose[:3,3])
    tangential = np.sqrt(np.maximum(0., future@future-(calibration.axes@future)**2))
    if np.max(abs(future)) > .004 or np.max(tangential) > .00496:
        raise RuntimeError('首点外推偏移超出捕获范围')


def acquire(binary, endpoint, routes, robot, recipe, points, capture, *, mock=False):
    capture = Path(capture); capture.mkdir(parents=True, exist_ok=True)
    side = routes.side
    client = Executor(binary, endpoint, routes.context, robot, recipe, side, capture/'native')
    cal = indicator_calibration(side, recipe['centered_readings_m'])
    selected_ball = [ball_center(points[0]['ball'])]
    driver = None
    windows = []
    try:
        if mock:
            hidden = np.array(recipe['mock']['joint_offsets_rad'])
            driver = MockQasBridge(lambda: mock_contact(routes.engine, client, side, selected_ball[0], cal, hidden),
                                   capture/'indicators_raw.jsonl', hz=recipe['mock']['indicator_hz'],
                                   reply_delay_s=recipe['mock']['reply_delay_s'])
        else:
            driver = UsbBridgeIndicatorDriver(recipe['port'], [recipe['channels'][axis] for axis in ('x','y','z')],
                                              capture/'indicators_raw.jsonl', reply_timeout_s=recipe['reply_timeout_s'])
        collector = SettledWindowCollector(driver, client, **{k: recipe[k] for k in (
            'settle_window_s', 'settle_span_rad', 'settle_timeout_s', 'discard_first',
            'samples_per_slot', 'stable_indicator_span_m')})
        def move(q, ball=None, *, local=False, windup=False):
            if local: routes.check_local(ball, client.hold, q)
            else: routes.check(client.hold, q, ball)
            client.move(q, windup=windup)
        # Only the fixed +/-20 degree preparation prefix uses URDF/velocity checks.
        client.move(ready_position())
        home = routes.context.home
        move(home)
        first = True
        for ball in dict.fromkeys(p['ball'] for p in points):
            selected_ball[0] = ball_center(ball)
            transfer, contact = routes.path(ball)
            for q in transfer[1:]: move(q)
            for q in contact[-2:1:-1]: move(q, ball, local=True)
            if first:
                before = collector.collect()
                pose_before = routes.engine.frame_pose(client.q, side+'_trical_tcp')
                move(contact[1], ball, local=True)
                middle = collector.collect()
                pose_middle = routes.engine.frame_pose(client.q, side+'_trical_tcp')
                seed_pose = routes.engine.frame_pose(contact[0], side+'_trical_tcp')
                validate_entry(before, middle, pose_before, pose_middle, seed_pose, selected_ball[0], cal)
                first = False
            else:
                move(contact[1], ball, local=True)
            move(contact[0], ball, local=True)
            for point in [p for p in points if p['ball'] == ball]:
                for q in routes.check_contact(ball, client.hold, point['q_rad'])[1:]:
                    client.move(q)
                for attempt in range(recipe['max_corrections']+1):
                    window = collector.collect()
                    offset = _core.trical_offset(window['readings_m'], cal)
                    if np.max(abs(offset)) <= recipe['range_tolerance_m']:
                        break
                    if attempt == recipe['max_corrections']:
                        raise RuntimeError('测点达到修正次数仍未入量程')
                    target = correction(routes.engine, client.q, side, offset, recipe['max_correction_step_m'],
                                        recipe['max_windup_rad'], client.hold)
                    for q in routes.check_contact(ball, client.hold, target)[1:]:
                        client.move(q, windup=True)
                point_meta = {key: value for key, value in point.items() if key != 'q_rad'}
                windows.append(dict(point_meta, target_q_rad=point['q_rad'], **window,
                                    offset_m=np.asarray(offset).tolist(),
                                    ball_center_m=selected_ball[0].tolist(), corrections=attempt))
                (capture/'windows.json').write_text(json.dumps(windows, indent=2)+'\n')
                print(f'[{side}] {len(windows)}/{len(points)}：{ball} 球，偏移 {np.max(abs(offset))*1000:.3f} mm，修正 {attempt} 次', flush=True)
                for q in routes.check_contact(ball, client.hold, contact[0])[1:]:
                    client.move(q)
            for q in contact[1:]: move(q, ball, local=True)
            for q in transfer[-2::-1]: move(q)
        client.finish()
    finally:
        if driver is not None: driver.close()
        client.close()
    return windows


def analyse_capture(capture):
    capture = Path(capture)
    windows = json.loads((capture/'windows.json').read_text())
    q = feedback_means(capture/'native/feedback.csv.zst', windows)
    return [dict(row, q_rad=mean) for row, mean in zip(windows, q)]
