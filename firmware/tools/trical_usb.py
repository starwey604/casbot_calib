#!/usr/bin/env python3
"""Explicit USB QAS access and a recorded, MCU-timed delivery-rate experiment."""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
import random
import re
import sys
import time


class BridgeError(RuntimeError):
    pass


class Link:
    def __init__(self, port, log, factory=None):
        if factory is None:
            from serial import Serial
            factory = Serial
        self.serial = factory(port, baudrate=115200, timeout=0.05, write_timeout=2)
        self.serial.dtr = True
        self.log = log
        self.buffer = bytearray()
        self.history = deque(maxlen=4096)
        self.next_id = 0
        self.received = 0

    def record(self, event):
        self.log.write(json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n")

    def read(self):
        while b"\n" not in self.buffer:
            data = self.serial.read(4096)
            self.buffer.extend(data)
            if len(self.buffer) > 8192 and b"\n" not in self.buffer:
                raise BridgeError("USB 数据行超过 8192 字节，停止记录")
            if not data:
                return None
        line, _, rest = self.buffer.partition(b"\n")
        self.buffer = bytearray(rest)
        try:
            event = json.loads(line)
            if not isinstance(event, dict) or event.get("schema") != 1:
                raise ValueError("unexpected schema")
        except (ValueError, UnicodeError) as exc:
            self.record(dict(type="invalid_usb_line", hex=line.hex(), host_receive_ns=time.monotonic_ns()))
            raise BridgeError("USB JSONL 无效；原始字节已记录") from exc
        event["host_receive_ns"] = time.monotonic_ns()
        self.received += 1
        event["host_event_index"] = self.received
        self.history.append(event)
        self.record(event)
        return event

    def wait(self, predicate, timeout=5, after=0, fail_slot=None):
        deadline = time.monotonic() + timeout
        checked = after
        while True:
            for event in self.history:
                index = event["host_event_index"]
                if index <= checked:
                    continue
                checked = index
                if fail_slot is not None and event.get("slot") == fail_slot and (
                    event["type"] in ("disconnected", "error") or
                    event["type"] == "subscribed" and event.get("result") != 0
                ):
                    raise BridgeError(f"仪表连接故障：{event}")
                if predicate(event):
                    return event
            if time.monotonic() >= deadline:
                raise BridgeError("等待开发板应答超时")
            self.read()

    def request(self, command, *, allow_error=False):
        if not command.isascii() or any(ord(c) < 32 or ord(c) > 126 for c in command):
            raise ValueError("命令必须是单行 ASCII")
        self.next_id += 1
        start = self.received
        wire = f"{self.next_id} {command}\n".encode("ascii")
        if len(wire) >= 128:
            raise ValueError("命令过长")
        self.record(dict(type="host_command", id=self.next_id, command=command, host_send_ns=time.monotonic_ns()))
        if self.serial.write(wire) != len(wire):
            raise BridgeError("USB 命令未完整写入")
        result = self.wait(lambda e: e["type"] == "result" and e.get("id") == self.next_id, after=start)
        if result["result"] != 0 and not allow_error:
            raise BridgeError(f"命令被拒绝（{result['result']}）：{command}")
        return result

    def send_qas(self, slot, mode, command):
        deadline = time.monotonic() + 5
        while True:
            start = self.received
            result = self.request(f"cmd {slot} {mode} {command}", allow_error=True)
            if result["result"] != -16:
                break
            if time.monotonic() > deadline:
                raise BridgeError("前一写入未完成")
            self.read()
        if result["result"] != 0:
            raise BridgeError(f"QAS 写入被拒绝：{result}")
        done = self.wait(lambda e: e["type"] == "write_done" and e.get("id") == result["id"],
                         after=start, fail_slot=slot)
        if done["result"]:
            raise BridgeError(f"GATT 写入失败：{done}")
        return start

    def get_param(self, slot, mode, key):
        start = self.send_qas(slot, mode, key)

        def answer(event):
            if event["type"] != "notify" or event.get("slot") != slot:
                return False
            text = bytes.fromhex(event["hex"]).decode("ascii", errors="replace").strip("\x00\r\n \t")
            name, separator, value = text.partition(":")
            return separator and name == key and value.strip() != "OK"

        event = self.wait(answer, after=start, fail_slot=slot)
        return bytes.fromhex(event["hex"]).decode("ascii").strip("\x00\r\n \t").partition(":")[2].strip()

    def set_param(self, slot, mode, key, value):
        self.send_qas(slot, mode, f"{key}:{value}")
        actual = self.get_param(slot, mode, key)
        if actual != str(value):
            raise BridgeError(f"{key} 回读不符：要求 {value}，实际 {actual}")

    def connect(self, slot, address, address_type):
        start = self.received
        self.request(f"connect {slot} {address} {address_type}")
        return self.wait(lambda e: e["type"] == "subscribed" and e.get("slot") == slot and e.get("state") == "ready",
                         timeout=25, after=start, fail_slot=slot)

    def close(self):
        self.serial.close()


def quantile(values, q):
    if not values:
        return None
    ordered = sorted(values)
    at = (len(ordered) - 1) * q
    low = int(at)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (at - low)


def decode_notification(raw):
    """Independent byte-level decoder for auditing the firmware's parsed fields."""
    parsed = dict(kind=0, key="", numeric=False)
    if len(raw) == 0 or len(raw) > 244:
        return parsed
    text = raw.strip(b"\x00\r\n \t")
    match = re.fullmatch(rb"([A-Z0-9]{1,15}):(.*)", text, flags=re.DOTALL)
    if match is None:
        return parsed
    key = match[1].decode("ascii")
    parsed.update(kind=1 if key in ("DAT0", "DAT1") else 2, key=key)
    if parsed["kind"] != 1:
        return parsed
    number = re.fullmatch(rb"([mi]?)([+-]?)([0-9]+(?:\.[0-9]{1,9})?|\.[0-9]{1,9})", match[2].lstrip(b" \t"))
    if number is None:
        return parsed
    digits = number[3]
    mantissa = int(digits.replace(b".", b""))
    if mantissa > 2**63 - 1:
        return parsed
    if number[2] == b"-":
        mantissa = -mantissa
    parsed.update(numeric=True, mantissa=mantissa,
                  decimals=len(digits.partition(b".")[2]), unit_tag=number[1][0] if number[1] else 0)
    return parsed


def analyze(path):
    """Recompute counts/intervals from raw USB events, not printed summaries."""
    runs = []
    reports = {}
    with Path(path).open(encoding="utf-8") as source:
        for line in source:
            event = json.loads(line)
            kind = event["type"]
            if kind == "rate_started":
                runs.append(dict(start=event, dat=[[], []], acks=0, bad=0, tail=0, writes={}, done={}, decoded_match=True, raw_problem=False))
            elif kind == "rate_report":
                reports[event["run"]] = event
            elif kind in ("notify", "write", "write_done"):
                for run in reversed(runs):
                    start = run["start"]
                    if event.get("slot") != start["slot"] or event.get("epoch") != start["epoch"]:
                        continue
                    t = event["time_us"]
                    if not start["start_us"] <= t < start["end_us"]:
                        continue
                    if kind == "notify":
                        raw = bytes.fromhex(event["hex"])
                        decoded = decode_notification(raw)
                        run["decoded_match"] &= all(event.get(k) == v for k, v in decoded.items())
                        run["raw_problem"] |= event.get("truncated", False) or event.get("length", len(raw)) != len(raw)
                        if decoded["key"] in ("DAT0", "DAT1"):
                            if not decoded["numeric"]:
                                run["bad"] += 1
                            elif t < start["stop_us"]:
                                run["dat"][int(decoded["key"][-1])].append(t)
                            else:
                                run["tail"] += 1
                        elif t < start["stop_us"] and bytes.fromhex(event["hex"]).strip(b"\x00\r\n \t") == b"MEAS:OK":
                            run["acks"] += 1
                    elif kind == "write" and event["id"] >= start["run"] and t < start["stop_us"]:
                        run["writes"][event["id"]] = event
                    elif kind == "write_done" and event["id"] >= start["run"]:
                        run["done"][event["id"]] = event
                    break
    results = []
    for run in runs:
        start = run["start"]
        report = reports.get(start["run"])
        duration = (start["stop_us"] - start["start_us"]) / 1e6
        streams = []
        for times in run["dat"]:
            gaps = [b - a for a, b in zip(times, times[1:])]
            if any(g < 0 for g in gaps):
                raise BridgeError("开发板通知时间倒退，不排序修复")
            streams.append(dict(notifications=len(times), delivery_hz=len(times) / duration,
                                interarrival_hz=(len(times) - 1) * 1e6 / (times[-1] - times[0])
                                if len(times) > 1 and times[-1] > times[0] else None,
                                gap_us=dict(min=min(gaps) if gaps else None,
                                            p50=quantile(gaps, .5), p95=quantile(gaps, .95),
                                            max=max(gaps) if gaps else None)))
        accepted = {i: e for i, e in run["writes"].items() if e["result"] == 0}
        done = {i: e for i, e in run["done"].items() if i in accepted}
        stack_waits = [e["time_us"] - accepted[i]["time_us"] for i, e in done.items() if e["result"] == 0]
        if any(t < 0 for t in stack_waits):
            raise BridgeError("写入完成时刻早于提交时刻")
        agreement = report is not None and [x["notifications"] for x in streams] == [report["dat0"], report["dat1"]]
        if report:
            agreement &= (len(accepted) == report["submitted"] and len(done) == report["done"] + report["done_errors"]
                          and run["acks"] == report["meas_ack"] and run["tail"] == report["tail_data"]
                          and run["bad"] == report["malformed"])
        write_times = [e["time_us"] for e in accepted.values()]
        write_gaps = [b - a for a, b in zip(write_times, write_times[1:])]
        if any(g < 0 for g in write_gaps):
            raise BridgeError("开发板写入时刻倒退")
        results.append(dict(run=start["run"], duration_s=duration, streams=streams,
                            accepted_write_gap_us=dict(p50=quantile(write_gaps, .5), p95=quantile(write_gaps, .95),
                                                       max=max(write_gaps) if write_gaps else None),
                            accepted_writes=len(accepted), accepted_write_hz=len(accepted) / duration,
                            stack_completion_us=dict(p50=quantile(stack_waits, .5), p95=quantile(stack_waits, .95)),
                            raw_counts_match=bool(agreement), decoded_fields_match=run["decoded_match"], report=report,
                            usable=bool(report and report["window_complete"] and report["raw_complete"] and agreement
                                        and run["decoded_match"] and not run["raw_problem"]),
                            measurement_sample_time_known=False, measurement_response_pairing=False))
    return results


def run(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    metadata = dict(schema="trical_qas_usb_run_v1", created_utc=datetime.now(timezone.utc).isoformat(),
                    arguments=vars(args), hardware_runtime_test=True)
    (output / "request.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    failure = None
    with (output / "events.jsonl").open("x", encoding="utf-8") as log:
        link = None
        original_mod = None
        mod_changed = False
        try:
            link = Link(args.port, log)
            result = link.request("status")
            status = link.wait(lambda e: e["type"] == "status" and e.get("id") == result["id"])
            if args.action == "scan":
                link.request(f"scan {args.scan_ms}")
                link.wait(lambda e: e["type"] == "scan_stopped", timeout=args.scan_ms / 1000 + 5)
                seen = set()
                for item in link.history:
                    if item["type"] == "advertisement" and item["address"] not in seen:
                        seen.add(item["address"])
                        print(f"{item['address']}  RSSI={item['rssi']}dBm  AD={item['hex']}")
            elif args.action == "command":
                if args.address:
                    link.connect(args.slot, args.address, args.address_type)
                for command in args.commands:
                    parts = command.split()
                    if len(parts) == 4 and parts[0] == "cmd":
                        link.send_qas(int(parts[1]), parts[2], parts[3])
                    else:
                        link.request(command)
                deadline = time.monotonic() + args.listen
                printed = 0
                while time.monotonic() < deadline:
                    for item in link.history:
                        if item["host_event_index"] > printed:
                            printed = item["host_event_index"]
                            if item["type"] == "notify":
                                value = bytes.fromhex(item["hex"]).decode("ascii", errors="backslashreplace")
                                print(f"slot={item['slot']} t={item['time_us']}us {value!r}")
                    link.read()
            else:
                if status["app"] != "qas_rate":
                    raise BridgeError("频率实验需要 firmware/tests/qas_rate 镜像")
                if not args.address:
                    raise BridgeError("rate 必须显式提供 --address 和 --address-type")
                link.connect(args.slot, args.address, args.address_type)
                if link.get_param(args.slot, args.write_mode, "HIDMOD") != "0":
                    raise BridgeError("请先显式将仪表设为软件模式 HIDMOD=0；本测试不自动修改")
                original_mod = link.get_param(args.slot, args.write_mode, "MOD")
                if original_mod not in ("0", "1"):
                    raise BridgeError("无法确认 MOD 原值")
                # Mark before writing, so a lost acknowledgment also takes the restore path.
                mod_changed = original_mod != "0"
                if mod_changed:
                    link.set_param(args.slot, args.write_mode, "MOD", "0")
                schedule = args.intervals * args.repeats
                random.Random(args.seed).shuffle(schedule)
                link.record(dict(type="experiment_schedule", intervals_ms=schedule, original_mod=original_mod))
                for interval in schedule:
                    deadline = time.monotonic() + 15
                    while True:
                        before = link.received
                        reply = link.request(f"rate {args.slot} {interval} {args.duration_ms} {args.drain_ms} {args.write_mode}", allow_error=True)
                        if reply["result"] == 0:
                            break
                        if reply["result"] not in (-11, -16) or time.monotonic() > deadline:
                            raise BridgeError(f"频率实验启动失败：{reply}")
                        # Continue draining USB while waiting for the firmware's quiet interval.
                        until = time.monotonic() + .2
                        while time.monotonic() < until:
                            link.read()
                    started = link.wait(lambda e: e["type"] == "rate_started", after=before)
                    report = link.wait(lambda e: e["type"] == "rate_report" and e.get("run") == started["run"],
                                       timeout=(args.duration_ms + args.drain_ms) / 1000 + 10, after=before,
                                       fail_slot=args.slot)
                    log.flush()
                    print(f"{interval}ms: DAT0={report['dat0']} DAT1={report['dat1']} "
                          f"写入={report['submitted']} 跳过={report['skipped']} 原始完整={report['raw_complete']}")
                    if not report["window_complete"] or not report["raw_complete"]:
                        raise BridgeError("本轮未完整记录，停止后续实验")
        except (Exception, KeyboardInterrupt) as exc:
            failure = str(exc) or type(exc).__name__
        finally:
            if link is not None:
                try:
                    link.request("stop")
                    if mod_changed:
                        # Same connection only. Never reconnect silently to restore a device.
                        link.set_param(args.slot, args.write_mode, "MOD", original_mod)
                        link.record(dict(type="restored", key="MOD", value=original_mod))
                    if args.address:
                        link.request(f"disconnect {args.slot}", allow_error=True)
                except (Exception, KeyboardInterrupt) as exc:
                    cleanup = f"清理/恢复失败：{exc}；必要时人工核对 MOD={original_mod}"
                    failure = f"{failure}; {cleanup}" if failure else cleanup
                finally:
                    link.close()
            if failure:
                log.write(json.dumps(dict(type="run_failed", reason=failure), ensure_ascii=False) + "\n")
    summaries = analyze(output / "events.jsonl")
    result = dict(schema="trical_qas_rate_analysis_v1", failure=failure, runs=summaries,
                  all_recorded_runs_usable=bool(summaries) and all(r["usable"] for r in summaries))
    (output / "analysis.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    if failure:
        raise BridgeError(failure)
    if args.action == "rate" and not result["all_recorded_runs_usable"]:
        raise BridgeError("原始记录与板端统计不一致，见 analysis.json")


def main(argv=None):
    parser = argparse.ArgumentParser(description="TriCal USB QAS 读写与频率实验；显式连接实物")
    parser.add_argument("action", choices=("scan", "command", "rate", "analyze"))
    parser.add_argument("--port")
    parser.add_argument("--output")
    parser.add_argument("--input", help="analyze: events.jsonl")
    parser.add_argument("--address")
    parser.add_argument("--address-type", choices=("public", "random"))
    parser.add_argument("--slot", type=int, choices=range(3), default=0)
    parser.add_argument("--write-mode", choices=("wcmd", "wreq"), default="wcmd")
    parser.add_argument("--commands", nargs="+", default=["status"])
    parser.add_argument("--listen", type=float, default=5)
    parser.add_argument("--scan-ms", type=int, default=5000)
    parser.add_argument("--intervals", nargs="+", type=int, default=[1000, 500, 250, 125, 100, 50, 20])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--seed", type=int, default=20261113)
    parser.add_argument("--duration-ms", type=int, default=30000)
    parser.add_argument("--drain-ms", type=int, default=2000)
    args = parser.parse_args(argv)
    if args.action == "analyze":
        if not args.input:
            parser.error("analyze 需要 --input")
        print(json.dumps(analyze(args.input), ensure_ascii=False, indent=2))
        return 0
    if not args.port or not args.output:
        parser.error("需要 --port 和一个新 --output 目录")
    if args.address and not args.address_type:
        parser.error("连接必须显式指定 --address-type")
    if args.repeats < 1 or not 1000 <= args.duration_ms <= 300000 or not 500 <= args.drain_ms <= 30000:
        parser.error("repeats≥1，duration-ms=1000..300000，drain-ms=500..30000")
    if any(not 20 <= n <= 10000 for n in args.intervals):
        parser.error("intervals=20..10000ms")
    try:
        run(args)
    except (BridgeError, OSError) as exc:
        print(f"失败：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
