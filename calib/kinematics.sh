#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export OMP_WAIT_POLICY=PASSIVE
exec bash "$repo/calib/dynamics.sh" --stages k1,k2,k3 "$@"
