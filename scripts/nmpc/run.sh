#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
prefix=${NMPC_CONDA_PREFIX:-"$HOME/.conda/envs/casbot_calib"}
export PATH="$prefix/bin:$PATH" ACADOS_SOURCE_DIR="$PWD/third_party/acados"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$prefix/bin/python" -m casbot_calib.nmpc.cli "$@"
