#!/usr/bin/env bash
# Project-local numerical runtime and native SHM binaries. No ROS source, system pip install or control commands.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
venv="$repo/.venv-shm"
wheelhouse=''
dependency_prefix=/opt/ros/humble
while (($#)); do
  case "$1" in
    --wheelhouse|--venv|--dependency-prefix)
      (($#>=2)) || exit 2
      case "$1" in --wheelhouse) wheelhouse=$2;; --venv) venv=$2;; --dependency-prefix) dependency_prefix=$2;; esac
      shift 2;;
    -h|--help) echo 'setup_h1.sh [--wheelhouse DIR] [--venv DIR] [--dependency-prefix DIR]';exit 0;;
    *) echo "未知参数：$1" >&2;exit 2;;
  esac
done
[[ -z ${CONDA_PREFIX:-} ]] || { echo '不要混用conda环境' >&2;exit 1; }
if [[ ! -e "$venv" ]]; then
  # Offline wheelhouse also supports Debian hosts without ensurepip.
  if [[ -n $wheelhouse ]]; then /usr/bin/python3 -m venv --without-pip "$venv"
  else /usr/bin/python3 -m venv "$venv";fi
fi
[[ -f "$venv/pyvenv.cfg" && -x "$venv/bin/python" ]] || { echo '不是可用项目venv，未覆盖' >&2;exit 1; }
"$venv/bin/python" -c 'import sys; assert sys.version_info[:2]==(3,10); assert sys.prefix!=sys.base_prefix'
if ! "$venv/bin/python" -m pip --version >/dev/null 2>&1; then
  [[ -n $wheelhouse && -f "$wheelhouse/pip-25.2-py3-none-any.whl" ]] || { echo 'venv缺pip；提供包含pip25.2的wheelhouse，或安装python3.10-venv后重试' >&2;exit 1; }
  # Use the official pip wheel to install pip into this venv only.
  PYTHONPATH="$wheelhouse/pip-25.2-py3-none-any.whl" "$venv/bin/python" -m pip install --ignore-installed --no-index --find-links "$wheelhouse" pip==25.2
fi
args=()
[[ -z $wheelhouse ]] || args+=(--no-index --find-links "$wheelhouse")
"$venv/bin/python" -m pip install "${args[@]}" -r "$repo/scripts/shm/requirements-h1.txt"
cd "$repo"
# Reuse exactly the CMake dependency prefix for independent URDF export/reload.
# Only these three math modules are linked; no ROS environment/site directory.
env -u PYTHONPATH -u LD_LIBRARY_PATH "$venv/bin/python" scripts/shm/link_math_bindings.py --dependency-prefix "$dependency_prefix"
# No vendor Python modules or private LD_LIBRARY_PATH enter the extension build.
env -u PYTHONPATH -u LD_LIBRARY_PATH CMAKE_BUILD_PARALLEL_LEVEL=1 "$venv/bin/python" -m pip install -e . --no-deps --no-build-isolation \
  --config-settings=build-dir=build/shm-python \
  --config-settings=cmake.define.CMAKE_PREFIX_PATH="$dependency_prefix" \
  --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON \
  --config-settings=cmake.define.pybind11_DIR="$("$venv/bin/python" -m pybind11 --cmakedir)" \
  --config-settings=cmake.define.CALIB_BUILD_SHM=ON \
  --config-settings=cmake.define.CALIB_BUILD_SHM_MOCK=ON \
  --config-settings=cmake.define.CMAKE_INSTALL_PREFIX="$repo/install/shm-core"
ctest --test-dir build/shm-python --output-on-failure
cmake --install build/shm-python --component Core
cmake --install build/shm-python --component Shm
"$venv/bin/python" -m pip check
env -u PYTHONPATH -u LD_LIBRARY_PATH "$venv/bin/python" -c 'from casbot_calib import _core; import numpy,scipy,clarabel; print("SHM numerical runtime:",numpy.__version__,scipy.__version__,clarabel.__version__)'
printf 'H1/A入口：CASBOT_SHM_PYTHON=%q bash fieldwork/shm/run.sh h1 --help\n' "$venv/bin/python"
