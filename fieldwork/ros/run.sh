#!/usr/bin/env bash
# F1 stays read-only; GUI control and zero-return are explicit motion actions.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo_dir"
runner=(bash scripts/humble/container.sh)
if [[ "${1:-}" == "gui" ]]; then
  exec bash "$repo_dir/fieldwork/shm/run.sh" "$@"
fi
if [[ "${1:-}" == "export-readback" || "${1:-}" == "report" || "${1:-}" == "a-evidence" || "${1:-}" == "sensitivity" || "${1:-}" == "--help" || "${1:-}" == "-h" || $# == 0 ]]; then
  exec "${runner[@]}" python -m casbot_calib.fieldwork_cli "$@"
fi
exec "${runner[@]}" bash scripts/humble/vendor.sh \
  python -m casbot_calib.fieldwork_cli "$@"
