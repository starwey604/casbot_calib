#!/usr/bin/env bash
# F1 stays read-only; GUI control and zero-return are explicit motion actions.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo_dir"
runner=(bash scripts/humble/container.sh)
if [[ "${1:-}" == "gui" ]]; then
  # Pass the desktop's existing display/auth configuration, never relax X access.
  display_env=()
  for name in DISPLAY XAUTHORITY WAYLAND_DISPLAY XDG_RUNTIME_DIR CASBOT_CJK_FONT CASBOT_FIELDWORK_CACHE XDG_STATE_HOME LIBGL_ALWAYS_SOFTWARE LP_NUM_THREADS; do
    if [[ -v "$name" ]]; then display_env+=("$name=${!name}"); fi
  done
  runner+=(env "${display_env[@]}")
  for arg in "$@"; do
    if [[ "$arg" == "--preview" ]]; then
      exec "${runner[@]}" python -m casbot_calib.fieldwork_cli "$@"
    fi
  done
fi
if [[ "${1:-}" == "export-readback" || "${1:-}" == "report" || "${1:-}" == "a-evidence" || "${1:-}" == "sensitivity" || "${1:-}" == "--help" || "${1:-}" == "-h" || $# == 0 ]]; then
  exec "${runner[@]}" python -m casbot_calib.fieldwork_cli "$@"
fi
exec "${runner[@]}" bash scripts/humble/vendor.sh \
  python -m casbot_calib.fieldwork_cli "$@"
