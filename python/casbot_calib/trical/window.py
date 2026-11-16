"""Settling and measurement envelopes from non-overlapping native state blocks."""
import time
import numpy as np

from .indicators import IndicatorFault


class SettledWindowCollector:
    def __init__(self, driver, feedback, *, settle_window_s=.2, settle_span_rad=.00025,
                 settle_timeout_s=3., discard_first=1, samples_per_slot=3,
                 stable_indicator_span_m=5e-6):
        self.driver = driver
        self.feedback = feedback
        self.window_ns = int(settle_window_s * 1e9)
        self.span = settle_span_rad
        self.timeout = settle_timeout_s
        self.discard = discard_first
        self.samples = samples_per_slot
        self.indicator_span = stable_indicator_span_m

    def _pump(self):
        self.feedback.pump()
        return self.driver.poll()

    @staticmethod
    def envelope(blocks):
        return np.max(np.array([b['q_max'] for b in blocks]), axis=0) - np.min(
            np.array([b['q_min'] for b in blocks]), axis=0)

    def settle(self):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if self._pump():
                raise IndicatorFault('停稳前收到未消费的测量通知')
            blocks = self.feedback.blocks
            if blocks and blocks[-1]['holding']:
                end = blocks[-1]['end_ns']
                window = [b for b in blocks if b['end_ns'] >= end - self.window_ns]
                if (window[0]['start_ns'] <= end - self.window_ns
                        and all(b['holding'] for b in window)
                        and np.max(self.envelope(window)) <= self.span):
                    return blocks[-1]
            time.sleep(.002)
        raise RuntimeError('关节未在停稳时限内进入包络')

    def collect(self):
        while True:
            settled = self.settle()
            start = settled['end_ns']
            counts = dict.fromkeys(self.driver.slots, 0)
            readings = {slot: [] for slot in self.driver.slots}
            pending = set()
            last = start
            while any(len(v) < self.samples for v in readings.values()):
                for slot in self.driver.slots:
                    if slot not in pending and len(readings[slot]) < self.samples:
                        self.driver.request(slot)
                        pending.add(slot)
                for event in self._pump():
                    if event.kind != 1:
                        continue
                    if event.slot not in pending or event.status != 'VALID':
                        raise IndicatorFault('测量通知无请求或状态无效')
                    pending.remove(event.slot)
                    counts[event.slot] += 1
                    if counts[event.slot] > self.discard:
                        readings[event.slot].append(event.value_m)
                    last = max(last, event.mono_ns)
                time.sleep(.002)
            # Include the whole native block containing the final notification.
            while not self.feedback.blocks or self.feedback.blocks[-1]['end_ns'] < last:
                if self._pump():
                    raise IndicatorFault('读表窗口结束后收到额外测量')
                time.sleep(.002)
            end = self.feedback.blocks[-1]['end_ns']
            blocks = [b for b in self.feedback.blocks if b['end_ns'] > start and b['start_ns'] <= end]
            z = np.array([readings[slot] for slot in self.driver.slots])
            if (np.max(np.ptp(z, axis=1)) <= self.indicator_span
                    and all(b['holding'] for b in blocks)
                    and np.max(self.envelope(blocks)) <= self.span):
                return dict(t_start_ns=start, t_end_ns=end, readings_m=z.mean(axis=1).tolist(),
                            reading_span_m=np.ptp(z, axis=1).tolist(), samples=z.tolist())
