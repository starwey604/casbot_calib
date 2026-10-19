"""Optional JSONL bridge for external SDK telemetry. Never opens a motor device."""
from __future__ import annotations
import json
from pathlib import Path
from threading import Event,Thread
from .telemetry import LatestStateSource


class JsonlTelemetrySource(LatestStateSource):
    def __init__(self,path,adapter,*,follow=False):
        super().__init__(adapter)
        self.path=Path(path);self.follow=follow;self.error=None;self._stop=Event()
        if not self.path.is_file():raise ValueError(f"Telemetry file does not exist: {self.path}")
        self._thread=Thread(target=self._consume,daemon=True,name="viewer-telemetry-read")
        self._thread.start()

    def _consume(self):
        try:
            with self.path.open(encoding="utf-8") as f:
                pending=""
                while not self._stop.is_set():
                    line=f.readline()
                    if not line:
                        if not self.follow:
                            if pending.strip():self.publish(json.loads(pending))
                            break
                        self._stop.wait(.02);continue
                    pending+=line
                    if pending.endswith("\n"):
                        if pending.strip():self.publish(json.loads(pending))
                        pending=""
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.error=str(exc)

    def close(self):
        self._stop.set();self._thread.join(timeout=1.)
