"""Append a recorded raw JSONL stream at its recorded receive cadence. No device I/O."""
import argparse
import json
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording",type=Path);parser.add_argument("output",type=Path)
    parser.add_argument("--speed",type=float,default=1.)
    args=parser.parse_args()
    if args.speed<=0:parser.error("speed must be positive")
    packets=[json.loads(line) for line in args.recording.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not packets:parser.error("recording is empty")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x",encoding="utf-8") as output:
        print(f"Telemetry file ready: {args.output}. Start the viewer with --follow.",flush=True)
        # Give the user time to open a second terminal's viewer.
        time.sleep(3.)
        start=time.monotonic();origin=packets[0]["receive_ns"]
        for packet in packets:
            due=(packet["receive_ns"]-origin)*1e-9/args.speed
            time.sleep(max(0.,start+due-time.monotonic()))
            output.write(json.dumps(packet)+"\n");output.flush()
    print("Replay ended; viewer will mark the last state stale.")


if __name__=="__main__":main()
