#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo"
if [[ -n ${CASBOT_GUI_PYTHON:-} ]]; then
  export PYTHONPATH="$repo/python${PYTHONPATH:+:$PYTHONPATH}"
  exec "$CASBOT_GUI_PYTHON" -m casbot_calib.shm_gui "$@"
fi
args=(env "PYTHONPATH=$repo/python")
for name in DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR XAUTHORITY SSH_AUTH_SOCK CASBOT_CJK_FONT XDG_STATE_HOME LIBGL_ALWAYS_SOFTWARE LP_NUM_THREADS; do
  if [[ -v "$name" ]]; then args+=("$name=${!name}"); fi
done
exec bash scripts/humble/container.sh "${args[@]}" .venv-humble/bin/python -m casbot_calib.shm_gui "$@"
