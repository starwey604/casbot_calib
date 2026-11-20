#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo"
source "$repo/scripts/shm/python_path.sh" "$repo/.venv-shm"
exec "$python" -m casbot_calib.pipeline.verify_dynamics "$@"
