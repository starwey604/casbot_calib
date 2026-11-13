import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest

FIRMWARE = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("trical_usb", FIRMWARE / "tools/trical_usb.py")
usb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usb)


def event(kind, **kwargs):
    if kind == "notify" and "hex" in kwargs:
        kwargs = {**usb.decode_notification(bytes.fromhex(kwargs["hex"])), **kwargs}
    return dict(schema=1, type=kind, **kwargs)


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.buffer = bytearray()
        self.writes = []
        self.closed = False
        self.dtr = False
        self.error = 0

    def write(self, wire):
        self.writes.append(wire)
        identifier, command = wire.decode().strip().split(" ", 1)
        identifier = int(identifier)
        events = []
        if command == "status":
            events.append(event("status", id=identifier, app="qas_rate"))
        elif command.startswith("cmd"):
            # Notifications can precede BOTH local completion and the USB result.
            events += [event("notify", slot=0, hex=b"MOD:0\r\n".hex()),
                       event("write_done", id=identifier, result=self.error)]
        events.append(event("result", id=identifier, result=0))
        self.buffer.extend(b"".join(json.dumps(e).encode() + b"\n" for e in events))
        return len(wire)

    def read(self, size):
        n = min(size, len(self.buffer), 17)
        data = self.buffer[:n]
        del self.buffer[:n]
        return data

    def close(self):
        self.closed = True


class HostTransport(unittest.TestCase):
    def test_fragmented_lines_and_early_param_answer(self):
        log = io.StringIO()
        link = usb.Link("fake", log, factory=FakeSerial)
        reply = link.request("status")
        self.assertEqual(reply["result"], 0)
        self.assertEqual(link.get_param(0, "wcmd", "MOD"), "0")
        self.assertTrue(link.serial.dtr)
        link.close()
        self.assertTrue(link.serial.closed)
        records = [json.loads(s) for s in log.getvalue().splitlines()]
        self.assertTrue(any(r["type"] == "host_command" for r in records))
        self.assertTrue(all("host_receive_ns" in r for r in records if r["type"] != "host_command"))

    def test_multiline_command_rejected(self):
        link = usb.Link("fake", io.StringIO(), factory=FakeSerial)
        with self.assertRaises(ValueError):
            link.request("status\ncmd 0 wcmd MOD:1")
        self.assertEqual(link.serial.writes, [])

    def test_bad_json_keeps_bytes_and_fails(self):
        log = io.StringIO()
        link = usb.Link("fake", log, factory=FakeSerial)
        link.serial.buffer.extend(b"not-json\n")
        with self.assertRaises(usb.BridgeError):
            while link.read() is None:
                pass
        self.assertEqual(json.loads(log.getvalue())["hex"], b"not-json".hex())

    def test_att_error_is_not_success(self):
        link = usb.Link("fake", io.StringIO(), factory=FakeSerial)
        link.serial.error = 3
        with self.assertRaisesRegex(usb.BridgeError, "GATT"):
            link.send_qas(0, "wreq", "MOD")

    def test_connection_failure_aborts_without_reconnect(self):
        link = usb.Link("fake", io.StringIO(), factory=FakeSerial)
        link.serial.buffer.extend(json.dumps(event("disconnected", slot=0, result=8)).encode()+b"\n")
        with self.assertRaisesRegex(usb.BridgeError, "连接故障"):
            link.wait(lambda e: False, fail_slot=0)
        self.assertEqual(link.serial.writes, [])


class Analysis(unittest.TestCase):
    def fixture(self):
        start = dict(run=0x80000001, slot=0, epoch=1, start_us=1_000_000, stop_us=2_000_000, end_us=2_500_000)
        records = [event("rate_started", **start)]
        common = dict(slot=0, epoch=1)
        records += [event("write", **common, time_us=1_000_000, id=start["run"]+1, result=0),
                    event("write_done", **common, time_us=1_010_000, id=start["run"]+1, result=0)]
        for t, key in [(999999, "DAT0"), (1_200_000, "DAT0"), (1_400_000, "DAT0"),
                       (1_300_000, "DAT1"), (2_000_000, "DAT0"), (2_500_000, "DAT0")]:
            records.append(event("notify", **common, time_us=t, key=key, numeric=True, hex=f"{key}:1.000".encode().hex()))
        records.append(event("notify", **common, time_us=1_500_000, key="MEAS", numeric=False, hex=b"MEAS:OK\r\n".hex()))
        report = event("rate_report", **start, window_complete=True, raw_complete=True,
                       dat0=2, dat1=1, submitted=1, done=1, done_errors=0, meas_ack=1, tail_data=1, malformed=0)
        records.append(report)
        return records

    def analysis(self, records):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"events.jsonl"
            path.write_text("".join(json.dumps(r)+"\n" for r in records))
            return usb.analyze(path)

    def test_independent_streams_boundaries_and_repeated_value(self):
        r = self.analysis(self.fixture())[0]
        self.assertTrue(r["usable"])
        self.assertEqual(r["streams"][0]["notifications"], 2)
        self.assertEqual(r["streams"][1]["notifications"], 1)
        self.assertEqual(r["streams"][0]["delivery_hz"], 2)
        self.assertEqual(r["streams"][0]["interarrival_hz"], 5)
        self.assertEqual(r["streams"][0]["gap_us"]["p95"], 200000)
        self.assertEqual(r["stack_completion_us"]["p50"], 10000)
        self.assertFalse(r["measurement_sample_time_known"])
        self.assertFalse(r["measurement_response_pairing"])

    def test_missing_raw_frame_invalidates_report(self):
        records = self.fixture()
        records.pop(4)
        self.assertFalse(self.analysis(records)[0]["usable"])

    def test_device_overflow_invalidates_report(self):
        records = self.fixture()
        records[-1]["raw_complete"] = False
        self.assertFalse(self.analysis(records)[0]["usable"])

    def test_aborted_window_not_a_full_rate_measurement(self):
        records = self.fixture()
        records[-1]["window_complete"] = False
        self.assertFalse(self.analysis(records)[0]["usable"])

    def test_late_frame_does_not_leak_to_next_phase(self):
        records = self.fixture()
        start = dict(run=0x80000010, slot=0, epoch=1, start_us=3_000_000, stop_us=4_000_000, end_us=4_500_000)
        records += [event("rate_started", **start), event("rate_report", **start,
                    window_complete=True, raw_complete=True, dat0=0, dat1=0, submitted=0,
                    done=0, done_errors=0, meas_ack=0, tail_data=0, malformed=0)]
        runs = self.analysis(records)
        self.assertEqual(len(runs), 2)
        self.assertEqual(runs[1]["streams"][0]["notifications"], 0)
        self.assertTrue(runs[1]["usable"])

    def test_time_reversal_is_not_silently_sorted(self):
        records = self.fixture()
        records[4], records[5] = records[5], records[4]
        with self.assertRaisesRegex(usb.BridgeError, "倒退"):
            self.analysis(records)

    def test_independent_decoder_detects_wrong_numeric_value(self):
        records = self.fixture()
        records[4]["mantissa"] += 1
        result = self.analysis(records)[0]
        self.assertTrue(result["raw_counts_match"])
        self.assertFalse(result["decoded_fields_match"])
        self.assertFalse(result["usable"])

    def test_independent_parser_known_and_invalid_values(self):
        self.assertEqual(usb.decode_notification(b"DAT1: m-0.001\r\n")["mantissa"], -1)
        for raw in (b"DAT0:1e3", b"DAT1:NaN", b"DAT0:1.", b"DAT1:9223372036854775808"):
            self.assertFalse(usb.decode_notification(raw)["numeric"])

    def test_linear_quantiles(self):
        self.assertEqual(usb.quantile([20, 10], .5), 15)
        self.assertIsNone(usb.quantile([], .5))

    def test_actual_c_json_fixture_is_parseable(self):
        path = FIRMWARE/"build/native-tests/usb-fixture.jsonl"
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        largest = max((r for r in rows if r["type"] == "notify"), key=lambda r: r["length"])
        self.assertEqual(len(bytes.fromhex(largest["hex"])), 244)
        self.assertFalse(largest["truncated"])
        self.assertTrue(any(r["type"] == "rate_report" for r in rows))
        for row in rows:
            if row["type"] == "notify":
                for key, value in usb.decode_notification(bytes.fromhex(row["hex"])).items():
                    self.assertEqual(row[key], value)



class ExperimentCleanup(unittest.TestCase):
    def test_failed_experiment_restores_original_mod(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        actions = []

        class Runner:
            def __init__(self, port, log):
                self.log = log
                self.received = 0

            def request(self, command, **kwargs):
                actions.append(command)
                return dict(id=1, result=0)

            def wait(self, predicate, **kwargs):
                if "rate 0 250 1000 500 wcmd" in actions:
                    # The phase starts, then collection fails.
                    if not hasattr(self, "started"):
                        self.started = True
                        return dict(run=42)
                    raise usb.BridgeError("injected collection failure")
                return dict(app="qas_rate")

            def connect(self, *args):
                actions.append("connect")

            def get_param(self, slot, mode, key):
                return "0" if key == "HIDMOD" else "1"

            def set_param(self, slot, mode, key, value):
                actions.append((key, value))

            def record(self, value):
                self.log.write(json.dumps(value)+"\n")

            def close(self):
                actions.append("close")

        with tempfile.TemporaryDirectory() as folder:
            args = SimpleNamespace(output=str(Path(folder)/"run"), port="fake", action="rate",
                                   address="78:CD:55:45:67:AD", address_type="public", slot=0,
                                   write_mode="wcmd", intervals=[250], repeats=1, seed=1,
                                   duration_ms=1000, drain_ms=500)
            with patch.object(usb, "Link", Runner):
                with self.assertRaisesRegex(usb.BridgeError, "injected"):
                    usb.run(args)
            self.assertEqual([a for a in actions if isinstance(a, tuple)], [("MOD", "0"), ("MOD", "1")])
            self.assertEqual(actions.count("connect"), 1)
            self.assertEqual(actions[-1], "close")
            result = json.loads((Path(args.output)/"analysis.json").read_text())
            self.assertIn("injected", result["failure"])


if __name__ == "__main__":
    unittest.main()
