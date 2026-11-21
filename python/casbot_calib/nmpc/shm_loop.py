"""One confirmed five-field reference at a time over the existing SHM owner."""
from pathlib import Path
import time

import numpy as np

from ..field_defaults import start_velocity_limit
from ..model_context import load_context
from ..project_paths import ROOT
from ..shm_gui_client import RemoteClient
from .dynamics_ff import Feedforward
from .runtime import Controller, pointer


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
            self.controller = Controller(self.client.latest['q'], mode='simulation' if mock else 'hardware', physical=True)
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
            if (time.monotonic_ns()-client.received_at > self.timeout*1e9
                    or s['mono_ns']-s['receive_ns'] > self.timeout*1e9):
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

    def start(self):
        ctl, client = self.controller, self.client
        s = self.poll()
        if (s['phase'] != -1 or time.monotonic_ns()-client.received_at > self.timeout*1e9
                or s['mono_ns']-s['receive_ns'] > self.timeout*1e9
                or np.max(np.abs(s['dq'])) >= start_velocity_limit()
                or any(j[-1] for j in s['joints'][:29])):
            raise RuntimeError('起点未停稳、反馈无效或驱动故障')
        if np.max(np.abs(s['q']-ctl.x[:14])) > .005:
            raise RuntimeError('使能前起点发生变化')
        self.feedback(s, ctl.x[:14])
        self.ff.effort(ctl.x)
        axes = {str(12+i): dict(input_lower=lo, input_upper=hi, input_velocity_limit=v,
                                velocity_limit=ctl.spec['velocity'][i], kp_max=kp, kd_max=kd)
                for i, (lo, hi, v, kp, kd) in enumerate(zip(ctl.lower, ctl.upper, ctl.velocity,
                                                           self.cfg['kp'], self.cfg['kd']))}
        settings = client.send('settings', axes=axes, effort_limit=self.ff.limit, end_velocity=True,
                    feedback_ms=self.cfg['ack_timeout_ms'], send_ms=self.cfg['ack_timeout_ms'])
        self.wait(lambda _: self.last_reply == settings, self.timeout, '等待 SHM 限位设置确认')
        client.send('start', kp=self.cfg['kp'], kd=self.cfg['kd'])
        s = self.wait(lambda s: s and s['phase'] == 3 and s['control'] == client.control
                      and s['applied_ns'] > 0, 4, '等待 SHM 使能')
        self.active = True
        self.last_applied = s['applied_ns']
        self.feedback(s, ctl.x[:14])
        self.send_confirmed(ctl.x)

    def feedback(self, s, reference):
        ctl = self.controller
        q = np.asarray(s['q'])
        if (not np.isfinite(q).all() or np.any(q < ctl.lower) or np.any(q > ctl.upper)
                or np.max(np.abs(q-reference)) > self.cfg['tracking_error_rad']):
            raise RuntimeError('实测关节位置或跟踪误差超限')
        gaps, _, poses = ctl.native.query(q, flange=True)
        if np.any(gaps <= ctl.margin):
            raise RuntimeError('实测碰撞间距低于准入裕度')
        self.measured_q, self.measured_poses, self.measured_gaps = q, poses, gaps

    def check_connection(self, qref):
        # Cover the line between measured pose and reference, including a backup entry.
        ctl = self.controller
        delta = qref-self.measured_q
        if np.all(self.measured_gaps-ctl.margin > ctl.radii @ np.abs(delta)):
            return
        v = delta/ctl.dt
        knots = np.array([np.r_[q, v, np.zeros(14)] for q in (self.measured_q, qref)])
        ok = ctl.native.lib.nmpc_certify(ctl.native.handle, pointer(knots), pointer(np.zeros(14)),
                                         1, ctl.dt, pointer(self.measured_gaps), pointer(ctl.radii), pointer(ctl.margin))
        if ok != 1:
            raise RuntimeError('反馈至参考的路径未通过碰撞认证')

    def send_confirmed(self, state):
        client = self.client
        effort = self.ff.effort(state)
        self.check_connection(state[:14])
        started = time.monotonic()
        client.send('fields', position=state[:14].tolist(), velocity=state[14:28].tolist(),
                    effort=effort.tolist(), kp=self.cfg['kp'], kd=self.cfg['kd'])
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

    def step(self, goal, *, stop=False):
        ctl = self.controller
        before = self.poll()
        self.feedback(before, ctl.x[:14])
        state = ctl.propose(goal, stop=stop)
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
