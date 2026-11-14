#!/usr/bin/env bash
# GUI now uses local SHM remote frontend; remaining SDK CLI stays under ros/.
set -euo pipefail
if [[ ${1:-} == gui ]]; then
  exec bash "$(dirname -- "${BASH_SOURCE[0]}")/shm/run.sh" "$@"
fi
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/ros/run.sh" "$@"
