#!/usr/bin/env python3
"""Check the usb_cdc example with text/binary echoes and a port reopen."""

import argparse
import sys
import time

import serial
from serial.tools import list_ports


def example_port():
    ports = [
        port.device
        for port in list_ports.comports()
        if port.vid == 0x2FE3 and port.product == "CASBOT USB CDC Example"
    ]
    if len(ports) != 1:
        raise RuntimeError(
            f"找到 {len(ports)} 个 USB CDC Example 端口；"
            "请确认已烧录 usb_cdc，或用 --port 指定端口。"
        )
    return ports[0]


def check_echo(port):
    payloads = (b"CASBOT USB CDC echo\r\n", bytes(range(256)))
    for session in range(1, 3):
        with serial.Serial(port, 115200, timeout=3, write_timeout=3) as device:
            device.dtr = True
            time.sleep(0.2)
            device.reset_input_buffer()
            for payload in payloads:
                written = device.write(payload)
                received = device.read(len(payload))
                if written != len(payload) or received != payload:
                    raise RuntimeError(
                        f"第 {session} 次连接回显不匹配："
                        f"发送 {written}/{len(payload)} B，收到 {len(received)} B；"
                        f"received={received!r}"
                    )
            print(f"PASS 第 {session} 次连接：文本与 256 B 二进制逐字节一致")
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="例如 /dev/ttyACM1；省略时按例程产品名查找")
    args = parser.parse_args()
    try:
        port = args.port or example_port()
        print(f"检查 {port}")
        check_echo(port)
    except (OSError, serial.SerialException, RuntimeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
