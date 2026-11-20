"""One outstanding MEAS per slot, using the bridge's recorded JSONL protocol."""
from dataclasses import dataclass
import importlib.util
import json
from pathlib import Path
import threading
import time

import numpy as np

from ..project_paths import ROOT


class IndicatorFault(RuntimeError):
    pass


@dataclass(frozen=True)
class IndicatorEvent:
    slot: int
    epoch: int
    mono_ns: int
    value_m: float
    status: str = 'VALID'
    kind: int = 1


def bridge_module():
    spec = importlib.util.spec_from_file_location('trical_usb', ROOT / 'firmware/tools/trical_usb.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class UsbBridgeIndicatorDriver:
    def __init__(self, port, channels, log_path, *, reply_timeout_s=1.5, factory=None):
        self.channels = list(channels)
        peers = [f"{dict(public='P', random='R')[c['address_type']]}:{c['address'].upper()}"
                 for c in self.channels]
        if len(peers) != 3 or len(set(peers)) != 3:
            raise ValueError('请为 X/Y/Z 配置三个不同的仪表地址')
        self.timeout_ns = int(reply_timeout_s * 1e9)
        self.pending = {}
        self.cursor = 0
        self.inbox = []
        self.module = bridge_module()
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        self.log = open(log_path, 'w', encoding='utf-8')
        try:
            self.link = self.module.Link(port, self.log, factory=factory)
        except BaseException:
            self.log.close()
            raise
        try:
            status = self.link.status()
            for address in peers:
                if address not in status['addresses']:
                    raise IndicatorFault(f'仪表 {address} 尚未就绪，请唤醒后重试')
            self.slots = [status['addresses'].index(address) for address in peers]
            for slot in self.slots:
                self.link.bind(status, slot)
                self.link.set_param(slot, 'wcmd', 'MOD', 0)
            self.cursor = self.link.received
        except BaseException:
            self.close()
            raise

    def _events(self):
        events = []
        for raw in self.link.history:
            if raw['host_event_index'] <= self.cursor:
                continue
            self.cursor = raw['host_event_index']
            slot = raw.get('slot')
            if slot not in self.slots:
                continue
            if raw['type'] in ('disconnected', 'error'):
                raise IndicatorFault(f'仪表槽 {slot} 断连或报错')
            if raw.get('epoch', self.link.epochs[slot]) != self.link.epochs[slot]:
                raise IndicatorFault(f'仪表槽 {slot} epoch 改变')
            if raw['type'] != 'notify':
                continue
            decoded = self.module.decode_notification(bytes.fromhex(raw['hex']))
            if decoded['kind'] != 1:
                continue
            if slot not in self.pending:
                raise IndicatorFault(f'仪表槽 {slot} 无在途 MEAS 却收到测量')
            requested = self.pending.pop(slot)
            if raw['host_receive_ns'] - requested > self.timeout_ns:
                raise IndicatorFault(f'仪表槽 {slot} MEAS 应答超时')
            channel = self.channels[self.slots.index(slot)]
            valid = (decoded['numeric'] and not raw.get('truncated', False)
                     and raw.get('status', 'VALID') == 'VALID'
                     and decoded.get('unit_tag') == channel['unit_tag'])
            if not valid:
                raise IndicatorFault(f'仪表槽 {slot} 测量状态或单位无效')
            value = decoded['mantissa'] * 10. ** -decoded['decimals']
            events.append(IndicatorEvent(slot, raw['epoch'], raw['host_receive_ns'],
                                         value * channel['unit_scale_m'] * channel['sign']))
        return events

    def request(self, slot):
        if slot in self.pending:
            raise IndicatorFault(f'仪表槽 {slot} 尚有在途 MEAS')
        # Process buffered traffic before attributing a notification to a new request.
        self.inbox.extend(self._events())
        while self.link.read() is not None:
            self.inbox.extend(self._events())
        self.pending[slot] = time.monotonic_ns()
        try:
            self.link.send_qas(slot, 'wcmd', 'MEAS')
        except self.module.BridgeError as error:
            raise IndicatorFault(str(error)) from error

    def poll(self):
        try:
            self.link.read()
            events = self.inbox + self._events()
            self.inbox = []
        except (self.module.BridgeError, OSError) as error:
            raise IndicatorFault(str(error)) from error
        now = time.monotonic_ns()
        if any(now - start > self.timeout_ns for start in self.pending.values()):
            raise IndicatorFault('仪表 MEAS 应答超时')
        return events

    def close(self):
        self.link.close()
        self.log.close()


class MockQasBridge:
    """Autonomous sensor cache; requests snapshot the cache, never evaluate contact."""
    slots = (0, 1, 2)

    def __init__(self, contact, log_path, *, hz=3., reply_delay_s=.34,
                 reply_timeout_s=1.5, faults=None):
        self.contact = contact
        self.period = 1. / hz
        self.delay_ns = int(reply_delay_s * 1e9)
        self.timeout_ns = int(reply_timeout_s * 1e9)
        self.faults = faults if faults is not None else {}
        self.pending = {}
        self.epoch = 1
        self.cache = np.zeros(3)
        self.status = ['INVALID'] * 3
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.started = time.monotonic_ns()
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        self.log = open(log_path, 'w', encoding='utf-8')
        self._sample()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _record(self, event):
        self.log.write(json.dumps(event) + '\n')

    def _sample(self):
        values, status = self.contact()
        with self.lock:
            if not self.faults.get('freeze_cache'):
                for slot in self.slots:
                    if slot not in self.faults.get('stuck_slots', ()):
                        self.cache[slot] = values[slot]
                    self.status[slot] = status[slot]

    def _run(self):
        while not self.stop.wait(self.period):
            try:
                self._sample()
            except Exception:
                with self.lock:
                    self.status = ['INVALID'] * 3

    def request(self, slot):
        if slot in self.pending:
            raise IndicatorFault(f'仪表槽 {slot} 尚有在途 MEAS')
        now = time.monotonic_ns()
        with self.lock:
            self.pending[slot] = (now, float(self.cache[slot]), self.status[slot])
        self._record(dict(type='host_command', slot=slot, command='MEAS', host_send_ns=now))

    def poll(self):
        now = time.monotonic_ns()
        if self.faults.get('disconnect'):
            raise IndicatorFault('Mock 仪表断连')
        if self.faults.get('epoch_change'):
            raise IndicatorFault('Mock 仪表 epoch 改变')
        if self.faults.get('unsolicited'):
            raise IndicatorFault('Mock 无在途 MEAS 却收到测量')
        events = []
        for slot, (start, value, status) in list(self.pending.items()):
            if now - start > self.timeout_ns:
                raise IndicatorFault('Mock MEAS 应答超时')
            if now - start < self.delay_ns or slot in self.faults.get('drop_slots', ()):
                continue
            del self.pending[slot]
            event = IndicatorEvent(slot, self.epoch, now, value, status)
            self._record(dict(type='notify', **event.__dict__))
            if status != 'VALID':
                raise IndicatorFault(f'Mock 槽 {slot} 状态 {status}')
            events.append(event)
        return events

    def close(self):
        self.stop.set()
        self.thread.join()
        self.log.close()
