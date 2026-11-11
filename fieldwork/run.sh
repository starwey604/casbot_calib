#!/usr/bin/env bash
# Read-only entry point. No motion command or SDK mode client is reachable here.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_dir"
if [[ "${1:-}" == "report" || "${1:-}" == "--help" || "${1:-}" == "-h" || $# == 0 ]]; then
  exec bash scripts/humble/container.sh python -m casbot_calib.fieldwork_cli "$@"
fi
exec bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python -m casbot_calib.fieldwork_cli "$@"
