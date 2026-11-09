#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ -n "${CONDA_PREFIX:-}" ]]; then echo 'Do not mix conda and Humble.' >&2; exit 1; fi
export PYTHONNOUSERSITE=1
set +u
source /opt/ros/humble/setup.bash
set -u
venv_activate="$repo_dir/.venv-humble/bin/activate"
if [[ ! -f "$venv_activate" ]]; then
  printf '[run.sh] Humble venv is missing: %s\n' "$venv_activate" >&2
  printf '[run.sh] Bootstrap it from the host repository root:\n' >&2
  printf '[run.sh]   bash scripts/humble/setup.sh\n' >&2
  printf '[run.sh] (or the manual bootstrap steps in docs/humble_environment.md)\n' >&2
  exit 1
fi
source "$venv_activate"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CMAKE_BUILD_PARALLEL_LEVEL=1
export MPLBACKEND=Agg MUJOCO_GL=egl
export MPLCONFIGDIR="$repo_dir/artifacts/humble_migration/matplotlib"
cd "$repo_dir"
exec "$@"
