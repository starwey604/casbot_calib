#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo"
if [[ -n ${CASBOT_SHM_PYTHON:-} ]]; then
  python="$CASBOT_SHM_PYTHON"
elif [[ -x "$repo/.venv-shm/bin/python" ]]; then
  python="$repo/.venv-shm/bin/python"
elif [[ -n ${VIRTUAL_ENV:-} && -x "$VIRTUAL_ENV/bin/python" ]]; then
  python="$VIRTUAL_ENV/bin/python"
else
  python=python3
fi
source "$repo/scripts/shm/python_path.sh"
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
