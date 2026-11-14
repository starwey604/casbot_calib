#!/usr/bin/env bash
# Explicit physical-control handoff. No vendor file/unit replacement.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
export PYTHONPATH="$repo/python${PYTHONPATH:+:$PYTHONPATH}"
if (($# == 0)); then set -- --help; fi
exec python3 -m casbot_calib.shm_takeover "$@"
