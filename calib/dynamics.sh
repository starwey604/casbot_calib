#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo"
export PYTHONPATH="$repo/python${PYTHONPATH:+:$PYTHONPATH}"
if [[ -n ${CASBOT_SHM_PYTHON:-} ]]; then
  python="$CASBOT_SHM_PYTHON"
elif [[ -x "$repo/.venv-shm/bin/python" ]]; then
  python="$repo/.venv-shm/bin/python"
elif [[ -n ${VIRTUAL_ENV:-} && -x "$VIRTUAL_ENV/bin/python" ]]; then
  python="$VIRTUAL_ENV/bin/python"
else
  python=python3
fi
exec "$python" -m casbot_calib.pipeline.runner "$@"
