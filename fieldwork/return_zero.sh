#!/usr/bin/env bash
# Compatibility entry point; SHM is explicit under fieldwork/shm/.
set -euo pipefail
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/ros/return_zero.sh" "$@"
