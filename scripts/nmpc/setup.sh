#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
prefix=${NMPC_CONDA_PREFIX:-"$HOME/.conda/envs/casbot_calib"}
export PATH="$prefix/bin:$PATH" CMAKE_PREFIX_PATH="$prefix"
export CC="$prefix/bin/x86_64-conda-linux-gnu-cc" CXX="$prefix/bin/x86_64-conda-linux-gnu-c++"
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-1}
build="$PWD/build/nmpc"
src="$PWD/third_party/acados"
revision=503364817c872d474ab5bed219c26760ac267769

# Pinocchio 3.9 configures 30 MPL elements. Generate matching headers for SHM Boost.PropertyTree.
# The conda glibc sysroot exports shm_open from librt.
python -m pip install --no-build-isolation --no-deps -e . \
  -Cbuild-dir="$build/host-build" -Cbuild.targets=_core -Ccmake.define.CALIB_BUILD_PYTHON=ON \
  -Ccmake.define.CMAKE_CXX_FLAGS=-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS= \
  -Ccmake.define.CMAKE_CXX_STANDARD_LIBRARIES=-lrt -Ccmake.define.BUILD_TESTING=OFF
bash scripts/shm/build.sh --with-mock --dependency-prefix "$prefix" \
  --build-dir "$build/host-build" --install-prefix "$build/shm-host" --jobs "$CMAKE_BUILD_PARALLEL_LEVEL"

if [[ ! -d "$src/.git" ]]; then
  git clone --branch v0.6.0 --depth 1 --filter=blob:none --sparse https://github.com/acados/acados.git "$src"
  git -C "$src" sparse-checkout set acados cmake external interfaces
fi
[[ $(git -C "$src" rev-parse HEAD) == "$revision" ]] || { echo "acados checkout must be $revision" >&2; exit 1; }
git -C "$src" submodule update --init --depth 1 external/blasfeo external/hpipm
cmake -S "$src" -B "$build/acados-build" -DCMAKE_INSTALL_PREFIX="$src" \
  -DCMAKE_BUILD_TYPE=Release -DACADOS_EXAMPLES=OFF -DACADOS_UNIT_TESTS=OFF \
  '-DCMAKE_INSTALL_RPATH=$ORIGIN' -DCMAKE_SHARED_LINKER_FLAGS=-Wl,-z,noexecstack -DBLASFEO_TARGET=X64_INTEL_HASWELL -DHPIPM_TARGET=AVX
cmake --build "$build/acados-build" --target install --parallel 4
python -m pip install -e "$src/interfaces/acados_template"
ACADOS_SOURCE_DIR="$src" LD_LIBRARY_PATH="$src/lib:$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" python - <<'PY'
import ctypes
import os
from pathlib import Path
from acados_template.utils import get_tera

ctypes.CDLL(str(Path(os.environ['ACADOS_SOURCE_DIR']) / 'lib/libacados.so'))
print(get_tera())
PY
