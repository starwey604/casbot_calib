#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ -n "${CONDA_PREFIX:-}" ]]; then echo 'Do not mix conda and Humble.' >&2; exit 1; fi
export PYTHONNOUSERSITE=1
set +u
source /opt/ros/humble/setup.bash
set -u
source "$repo_dir/.venv-humble/bin/activate"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CMAKE_BUILD_PARALLEL_LEVEL=1
export MPLBACKEND=Agg MUJOCO_GL=egl
export MPLCONFIGDIR="$repo_dir/artifacts/humble_migration/matplotlib"
cd "$repo_dir"
exec "$@"
