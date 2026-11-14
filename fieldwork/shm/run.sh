#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo_dir"
if [[ ${1:-} == gui && ! -v CASBOT_SHM_PYTHON ]]; then
  shift
  exec bash "$repo_dir/scripts/shm/gui.sh" "$@"
fi
export PYTHONPATH="$repo_dir/python${PYTHONPATH:+:$PYTHONPATH}"
python=${CASBOT_SHM_PYTHON:-python3}
if [[ ! -v CASBOT_SHM_PYTHON && ( ${1:-} == h1 || ${1:-} == a-evidence ) && -x "$repo_dir/.venv-shm/bin/python" ]]; then
  python="$repo_dir/.venv-shm/bin/python"
fi
exec "$python" -m casbot_calib.shm_fieldwork "$@"
