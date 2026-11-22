"""One confirmed five-field reference at a time over the existing SHM owner."""
from pathlib import Path
import sys
import time

import numpy as np

from ..field_defaults import start_velocity_limit
from ..model_context import load_context
from ..project_paths import ROOT
from ..shm_gui_client import RemoteClient
from ..shm_ready import ready_position
from .preparation import ReadyMotion
from .dynamics_ff import Feedforward
from .runtime import Controller


def make_client(root, host=None, repo='/home/nvidia/casbot_calib_shm'):
    root = Path(root)
    if host is None:
        config = dict(mock=True, output=str(root / 'owner'),
                      executor=str(ROOT / 'build/nmpc/shm-host/bin/shm_executor'))
    else:
        if not host.strip():
            raise ValueError('需要指定 Orin 主机')
        config = dict(mock=False, output=str(Path(repo) / 'artifacts/nmpc' / root.name))
    return RemoteClient(config, root / 'client', host=host, repo=repo)


class SHMLoop:
    def __init__(self, client, *, delivery=None, feedforward=True):
        self.client = client
        self.delivery = delivery
        self.feedforward = feedforward
        self.controller = None
        self.active = False
        self.start_rejected = False
        self.disable_ack = False
        self.last_reply = 0
        self.last_applied = 0
        self.ack_ms = 0.
        self.next_tracking_warning = 0.
        self._next_warnings = {}

    def __enter__(self):
        try:
            self.client.connect()
            self.wait(lambda s: self.client.hello and self.client.ready and s
                      and s['master_frame'] > 1 and s['receive_ns'], 10, '等待首次反馈')
            ready = self.client.ready
            ctx = load_context()
            mock = self.client.config['mock']
            if (ready['mock'] != mock or (mock and not ready['endpoint'].startswith('/casbot_calib_test_'))
                    or ready['model_context'] != ctx.identity or tuple(ready['joint_names']) != ctx.joint_names):
                raise RuntimeError('远端模型、轴序或端点不匹配')
            self.initial_q = np.array(self.client.latest['q'])
            self.mode = 'simulation' if mock else 'hardware'
            self.needs_ready = np.max(np.abs(self.initial_q-ready_position())) > .005
            self.controller = Controller(ready_position() if self.needs_ready else self.initial_q,
                                         mode=self.mode, physical=True)
            self.cfg = self.controller.spec['config']['shm']
            self.ff = Feedforward(self.cfg, self.delivery, enabled=self.feedforward)
            return self
        except BaseException:
            self.close()
            raise

    def poll(self):
        client = self.client
        for msg in client.drain():
            if msg.get('type') == 'reply':
                if not msg['ok']:
                    if msg['id'] == client.control:
                        self.start_rejected = True
                    raise RuntimeError(msg['reason'])
                self.last_reply = msg['id']
            if (client.control and msg.get('type') == 'status' and msg['control'] == client.control
                    and msg['phase'] in (4, -1)):
                raise RuntimeError(msg['reason'])
        if client.failure:
            raise RuntimeError(client.failure)
        s = client.latest
        if s and self.active:
            if s['phase'] != 3 or s['control'] != client.control:
                raise RuntimeError('SHM 控制会话已中断')
            if time.monotonic_ns()-client.received_at > self.timeout*1e9:
                raise TimeoutError('SHM 反馈超时')
        client.tick()
        return s

    def wait(self, predicate, timeout, reason):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            s = self.poll()
            if predicate(s) and time.monotonic() < end:
                return s
            time.sleep(.001)
        raise TimeoutError(reason)

    @property
    def timeout(self):
        return self.cfg['ack_timeout_ms'] / 1000

    def start(self, on_preparation=None):
        ctl, client = self.controller, self.client
        s = self.poll()
        if (s['phase'] != -1 or time.monotonic_ns()-client.received_at > self.timeout*1e9
                or s['mono_ns']-s['receive_ns'] > self.timeout*1e9
                or np.max(np.abs(s['dq'])) >= start_velocity_limit()
                or any(j[-1] for j in s['joints'][:29])):
            raise RuntimeError('起点未停稳、反馈无效或驱动故障')
        if np.max(np.abs(s['q']-self.initial_q)) > .03:
            raise RuntimeError('使能前起点发生变化')
        prefix = ReadyMotion(load_context(), np.asarray(s['q']), ctl) if self.needs_ready else None
        if prefix:
            from ..data import write_json
            write_json(client.root/'preparation.json', dict(motion=prefix.motion.metadata(),
                       limits=prefix.certificate, fixture=prefix.proof))
        else:
            self.feedback(s, ctl.x[:14])
        self.ff.effort(ctl.x)
        axes = {str(12+i): dict(input_lower=lo, input_upper=hi, input_velocity_limit=v,
                                velocity_limit=ctl.spec['velocity'][i], kp_max=max(kp,20), kd_max=max(kd,2))
                for i, (lo, hi, v, kp, kd) in enumerate(zip(ctl.lower, ctl.upper, ctl.velocity,
                                                           self.cfg['kp'], self.cfg['kd']))}
        settings = client.send('settings', axes=axes, effort_limit=self.ff.limit, end_velocity=False,
                    feedback_ms=self.cfg['ack_timeout_ms'], send_ms=self.cfg['ack_timeout_ms'])
        self.wait(lambda _: self.last_reply == settings, self.timeout, '等待 SHM 限位设置确认')
        s = self.poll()
        if (np.max(np.abs(s['q']-self.initial_q)) > .03 or max(map(abs,s['dq'])) >= start_velocity_limit()
                or time.monotonic_ns()-client.received_at > self.timeout*1e9
                or s['mono_ns']-s['receive_ns'] > self.timeout*1e9):
            raise RuntimeError('准备轨迹认证后起点变化或反馈过期')
        client.send('start', kp=[20.]*14 if prefix else self.cfg['kp'],
                    kd=[2.]*14 if prefix else self.cfg['kd'])
        s = self.wait(lambda s: s and s['phase'] == 3 and s['control'] == client.control
                      and s['applied_ns'] > 0, 4, '等待 SHM 使能')
        self.active = True
        self.last_applied = s['applied_ns']
        if prefix:
            self.run_preparation(prefix, on_preparation)
            settle_end = time.monotonic() + 1.0
            s = self.poll()
            while max(map(abs, s['dq'])) >= start_velocity_limit() and time.monotonic() < settle_end:
                time.sleep(.01)
                s = self.poll()
            if max(map(abs, s['dq'])) >= start_velocity_limit():
                self._warn('prep_settle', f"准备段结束时双臂仍有余速 {max(map(abs, s['dq'])):.3f} rad/s（仅报警）", interval=0.)
            self.warn_tracking(np.asarray(s['q']), ready_position(), '准备段结束', threshold=.05)
        self.feedback(s, ctl.x[:14])
        self.send_confirmed(ctl.x)

    def _warn(self, key, message, *, interval=1.0):
        now = time.monotonic()
        if now >= self._next_warnings.get(key, 0.):
            self._next_warnings[key] = now + interval
            print(f'\n{message}', file=sys.stderr, flush=True)

    def _warn_measured_limits(self, q, stage):
        ctl = self.controller
        below = ctl.lower - q
        above = q - ctl.upper
        violation = np.maximum(below, above)
        axis = int(np.argmax(violation))
        if violation[axis] > 0.:
            bound = ctl.lower[axis] if below[axis] >= above[axis] else ctl.upper[axis]
            self._warn(f'limit:{stage}',
                       f'{stage}实测越软限位 ID{12+axis} 实测 {q[axis]:.3f} rad（限位 {bound:.3f}，仅报警）')

    def run_preparation(self, prefix, on_preparation):
        ctl = self.controller
        duration = prefix.motion.duration
        for t in np.linspace(0., duration, int(np.ceil(duration/ctl.dt))+1):
            begin = time.monotonic()
            s = self.poll()
            q = np.asarray(s['q'])
            state, effort, kp, kd = prefix.fields(t, self.ff, self.cfg)
            if not np.isfinite(q).all():
                raise RuntimeError('准备段实测关节位置含非有限值')
            self._warn_measured_limits(q, '准备段')
            self.warn_tracking(q, state[:14], '准备段')
            slack = prefix.measured_fixture_slack(q)
            if slack <= 0.:
                self._warn('prep_fixture', f'准备段实测工装间距低于准入裕度（裕量 {1000*slack:.1f} mm，仅报警）')
            if on_preparation:
                on_preparation(q)
            self.send_fields(state, effort, kp, kd)
            time.sleep(max(0., ctl.dt-(time.monotonic()-begin)))

    def warn_tracking(self, q, reference, stage, threshold=None):
        limit = self.cfg['tracking_error_rad'] if threshold is None else threshold
        error = np.abs(q-reference)
        axis = int(np.argmax(error))
        now = time.monotonic()
        if error[axis] > limit and (now >= self.next_tracking_warning or threshold is not None):
            self.next_tracking_warning = now + 1.
            print(f'\n{stage}跟踪报警 ID{12+axis} 误差 {error[axis]:.3f} rad'
                  f'（阈值 {limit:.3f}，参考 {reference[axis]:.3f}，实测 {q[axis]:.3f}）',
                  file=sys.stderr, flush=True)

    def feedback(self, s, reference):
        ctl = self.controller
        q = np.asarray(s['q'])
        if not np.isfinite(q).all():
            raise RuntimeError('实测关节位置含非有限值')
        self._warn_measured_limits(q, 'NMPC')
        self.warn_tracking(q, reference, 'NMPC')
        gaps, _, poses = ctl.native.query(q, flange=True)
        slack = gaps - ctl.margin
        worst = int(np.argmin(slack))
        if slack[worst] <= 0.:
            severity = '接近物理接触' if gaps[worst] <= 0. else '低于准入裕度'
            self._warn('clearance',
                       f'NMPC实测碰撞间距{severity}（距离 {1000*gaps[worst]:.1f} mm，裕量 {1000*slack[worst]:.1f} mm，仅报警）')
        self.measured_q, self.measured_poses, self.measured_gaps = q, poses, gaps

    def send_confirmed(self, state):
        return self.send_fields(state, self.ff.effort(state), self.cfg['kp'], self.cfg['kd'])

    def send_fields(self, state, effort, kp, kd):
        client = self.client
        started = time.monotonic()
        client.send('fields', position=state[:14].tolist(), velocity=state[14:28].tolist(),
                    effort=effort.tolist(), kp=kp, kd=kd)
        revision = client.revision

        def applied(s):
            if s['revision'] > revision:
                raise RuntimeError('SHM 参考版本发生跳变')
            return s['revision'] == revision and s['applied_ns'] > self.last_applied

        remaining = self.timeout - (time.monotonic()-started)
        s = self.wait(applied, remaining, 'SHM 命令入队确认超时')
        self.ack_ms = (time.monotonic()-started)*1000
        self.last_applied = s['applied_ns']
        return s

    def step(self, goal, *, stop=False, return_ready=False):
        ctl = self.controller
        before = self.poll()
        self.feedback(before, ctl.x[:14])
        state = ctl.propose(goal, stop=stop, return_ready=return_ready)
        latest = self.poll()
        if latest['seq'] != before['seq']:
            self.feedback(latest, ctl.x[:14])
        s = self.send_confirmed(state)
        self.feedback(s, state[:14])
        ctl.commit()
        return self.measured_q

    def close(self):
        client = self.client
        try:
            if client.control and not self.start_rejected and not client.closed:
                client.send('stop', control=client.control)
                end = time.monotonic()+3.5
                while time.monotonic() < end:
                    s = client.latest
                    messages = client.drain()
                    terminal = next((m for m in messages if m.get('type') == 'status'
                                     and m['control'] == client.control and m['phase'] == -1), None)
                    if not terminal and s and s['phase'] == -1 and s['control'] == client.control:
                        terminal = s
                    if terminal:
                        self.disable_ack = bool(terminal['disable_ack'])
                        break
                    if client.process.poll() is not None:
                        break
                    time.sleep(.005)
        finally:
            client.close()
            # On loss of the control transport the owner disables on EOF. A still
            # connected display may carry its final acknowledgement during close.
            s = client.latest
            if s and s['control'] == client.control and s['phase'] == -1 and s['disable_ack']:
                self.disable_ack = True
            if self.controller:
                self.controller.close()
            self.active = False

    def __exit__(self, exc_type, exc, tb):
        self.close()
        if self.client.control and not self.start_rejected and not self.disable_ack:
            raise RuntimeError('未收到 SHM 失能应答') from exc
