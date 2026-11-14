#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo"
if [[ -n ${CASBOT_GUI_PYTHON:-} ]]; then
  export PYTHONPATH="$repo/python${PYTHONPATH:+:$PYTHONPATH}"
  exec "$CASBOT_GUI_PYTHON" -m casbot_calib.shm_gui "$@"
fi
args=(env "PYTHONPATH=$repo/python" "DISPLAY=${DISPLAY:-}" "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}" "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-}" "XAUTHORITY=${XAUTHORITY:-}" "SSH_AUTH_SOCK=${SSH_AUTH_SOCK:-}")
exec bash scripts/humble/container.sh "${args[@]}" .venv-humble/bin/python -m casbot_calib.shm_gui "$@"
