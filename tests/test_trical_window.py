import json
import time
import numpy as np
import pytest

from casbot_calib.trical.indicators import IndicatorFault, MockQasBridge, UsbBridgeIndicatorDriver
from casbot_calib.trical.window import SettledWindowCollector


class Feedback:
    def __init__(self, drift=False):
        self.blocks = []
        self.start = time.monotonic_ns()
        self.last = self.start
        self.drift = drift

    def pump(self):
        now = time.monotonic_ns()
        if now - self.last >= 2_000_000:
            # A transient after settling: the first complete window must be retried.
            q = .001 if self.drift and .02 < (now-self.start)*1e-9 < .05 else 0.
            self.blocks.append(dict(start_ns=self.last+1, end_ns=now, count=2,
                                    holding=True, q_mean=[q]*14, q_min=[q]*14, q_max=[q]*14))
            self.last = now


def test_cache_before_settling_is_discarded_and_equal_values_count(tmp_path):
    value = np.zeros(3)
    driver = MockQasBridge(lambda: (value.copy(), ['VALID']*3), tmp_path/'raw.jsonl',
                           hz=100, reply_delay_s=.012)
    try:
        # request snapshots old cache; acquisition clock updates independently.
        for s in driver.slots: driver.request(s)
        value[:] = .002
        time.sleep(.016)
        assert [e.value_m for e in driver.poll()] == [0., 0., 0.]
        collector = SettledWindowCollector(driver, Feedback(), settle_window_s=.01)
        result = collector.collect()
        np.testing.assert_allclose(result['readings_m'], [.002]*3)
        assert result['reading_span_m'] == [0.]*3
        assert all(len(samples) == 3 for samples in result['samples'])
    finally:
        driver.close()
    rows = [json.loads(line) for line in (tmp_path/'raw.jsonl').read_text().splitlines()]
    assert sum(r['type'] == 'notify' for r in rows) == 15


def test_entire_window_drift_retries(tmp_path):
    driver = MockQasBridge(lambda: ([0.]*3, ['VALID']*3), tmp_path/'raw', hz=100, reply_delay_s=.012)
    feedback = Feedback(drift=True)
    try:
        result = SettledWindowCollector(driver, feedback, settle_window_s=.01).collect()
        assert result['t_start_ns'] > feedback.start + 50_000_000
    finally:
        driver.close()


@pytest.mark.parametrize('fault', ['disconnect', 'epoch_change', 'unsolicited'])
def test_faults(tmp_path, fault):
    driver = MockQasBridge(lambda: ([0.]*3, ['VALID']*3), tmp_path/'raw', faults={fault: True})
    try:
        with pytest.raises(IndicatorFault): driver.poll()
    finally:
        driver.close()


def test_stopped_response_times_out_and_duplicate_request_fails(tmp_path):
    driver = MockQasBridge(lambda: ([0.]*3, ['VALID']*3), tmp_path/'raw',
                           reply_timeout_s=.01, faults={'drop_slots': [0]})
    try:
        driver.request(0)
        with pytest.raises(IndicatorFault, match='在途'): driver.request(0)
        time.sleep(.012)
        with pytest.raises(IndicatorFault, match='超时'): driver.poll()
    finally:
        driver.close()


class Serial:
    def __init__(self, *args, **kwargs):
        self.lines = []
        self.closed = False

    def add(self, **event):
        self.lines.append((json.dumps(dict(schema=1, **event))+'\n').encode())

    def write(self, wire):
        identifier, command, *args = wire.decode().strip().split()
        identifier = int(identifier)
        self.add(type='result', id=identifier, result=0)
        if command == 'status':
            self.add(type='status', id=identifier, epochs=[7]*3,
                     addresses=[f'R:00:00:00:00:00:0{s}' for s in (1, 2, 0)])
        elif command == 'cmd':
            slot = int(args[0])
            assert args[1] == 'wcmd' and args[3] == '7'
            self.add(type='write_done', slot=slot, epoch=7, id=identifier, result=0)
            text = 'DAT0:m1.250' if args[2] == 'MEAS' else 'MOD:0'
            self.add(type='notify', slot=slot, epoch=7, hex=text.encode().hex())
        return len(wire)

    def read(self, n):
        return self.lines.pop(0) if self.lines else b''

    def close(self): self.closed = True


def test_usb_interleaved_slots_and_parameter_replies(tmp_path):
    channels = [dict(address=f'00:00:00:00:00:0{s}', address_type='random',
                     sign=1, unit_tag=ord('m'), unit_scale_m=.001) for s in range(3)]
    driver = UsbBridgeIndicatorDriver('fake', channels, tmp_path/'raw', factory=Serial)
    try:
        assert driver.slots == [2, 0, 1]
        for s in driver.slots: driver.request(s)
        events = []
        for _ in range(10): events.extend(driver.poll())
        assert sorted(e.slot for e in events) == [0, 1, 2]
        assert all(e.value_m == .00125 for e in events)
        driver.link.serial.add(type='notify', slot=1, epoch=7, hex=b'DAT0:m1.250'.hex())
        with pytest.raises(IndicatorFault, match='无在途'): driver.poll()
    finally:
        driver.close()


@pytest.mark.parametrize('raw_change,match', [
    ({'epoch': 8}, 'epoch'),
    ({'hex': b'DAT0:mERR'.hex()}, '无效'),
    ({'hex': b'DAT0:i1.250'.hex()}, '单位'),
])
def test_usb_epoch_status_and_unit_faults(tmp_path, raw_change, match):
    channels = [dict(address=f'00:00:00:00:00:0{s}', address_type='random',
                     sign=1, unit_tag=ord('m'), unit_scale_m=.001) for s in range(3)]
    driver = UsbBridgeIndicatorDriver('fake', channels, tmp_path/'raw', factory=Serial)
    try:
        driver.request(0)
        # The fake serial has queued the first measurement after write_done.
        raw = json.loads(driver.link.serial.lines[0])
        raw.update(raw_change)
        driver.link.serial.lines[0] = (json.dumps(raw)+'\n').encode()
        with pytest.raises(IndicatorFault, match=match): driver.poll()
    finally:
        driver.close()


def test_two_step_detects_frozen_cache_and_extrapolated_capture_limit():
    from casbot_calib import _core
    from casbot_calib.trical.controller import validate_entry
    cal = _core.TricalCalibration()
    before_pose = np.eye(4); before_pose[:3,3] = -np.ones(3)*.005/np.sqrt(3)
    middle_pose = np.eye(4); middle_pose[:3,3] = before_pose[:3,3]/2
    before = dict(readings_m=(-before_pose[:3,3]).tolist())
    middle = dict(readings_m=(-middle_pose[:3,3]).tolist())
    validate_entry(before,middle,before_pose,middle_pose,np.eye(4),np.zeros(3),cal)
    with pytest.raises(RuntimeError,match='读数变化'):
        validate_entry(before,before,before_pose,middle_pose,np.eye(4),np.zeros(3),cal)
    displacement = np.array([.004,.004,0.])
    before['readings_m'] = (-before_pose[:3,3]+displacement).tolist()
    middle['readings_m'] = (-middle_pose[:3,3]+displacement).tolist()
    with pytest.raises(RuntimeError,match='捕获范围'):
        validate_entry(before,middle,before_pose,middle_pose,np.eye(4),np.zeros(3),cal)
