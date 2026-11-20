#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo"
source "$repo/scripts/shm/python_path.sh" "$repo/.venv-shm"
if [[ ${1:-} == dynamics-verify-mock ]]; then
  shift
  export OMP_WAIT_POLICY=PASSIVE
  exec "$python" -m casbot_calib.pipeline.verify_dynamics --mock "$@"
fi
if [[ ${1:-} == kinematics-mock ]]; then
  shift
  export OMP_WAIT_POLICY=PASSIVE
  exec "$python" -m casbot_calib.pipeline.runner --stages k1,k2,k3 --mock "$@"
fi
if [[ ${1:-} == trical-probe ]]; then
  shift
  exec "$python" "$repo/firmware/tools/trical_usb.py" "$@"
fi
if [[ ${1:-} == diagnose ]]; then
  shift
  out=${1:-"$repo/artifacts/dev/diagnose_$(date +%Y%m%d_%H%M%S)"}
  exec "$python" -m casbot_calib.shm_takeover --diagnose "$out"
fi
exec "$python" -m casbot_calib.shm_fieldwork "$@"
