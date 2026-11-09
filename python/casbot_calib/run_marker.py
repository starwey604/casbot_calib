"""Per-run UTC timestamp marker for result artifacts.

A run marker identifies one execution of a workflow; it is not a device
identity, a firmware binding, or evidence about the robot that produced data.
"""
from datetime import datetime, timezone


def run_marker():
    """Return a fresh timestamp marker to merge into one run's result dict."""
    now = datetime.now(timezone.utc)
    return dict(run_id=now.strftime('%Y%m%dT%H%M%S%fZ'),
                created_utc=now.isoformat())
