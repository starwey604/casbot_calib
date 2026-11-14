#!/usr/bin/env bash
# One-command Orin environment bootstrap, native C++/Python build (install/shm-core), and preflight check.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
venv="$repo/.venv-shm"
wheelhouse=''
dependency_prefix=/opt/ros/humble
check_only=0
mem_kb=$(awk '/^MemTotal:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)
jobs=$(( mem_kb >= 8000000 ? 2 : 1 ))

usage() {
  cat <<'EOF'
用法：bash calib/setup.sh [选项]
  --check                     仅运行环境、模型 LFS 资产与执行器预检，不重编安装
  --wheelhouse DIR            离线 Python wheel 目录（未指定时若离线会自动探测 artifacts/*wheels*）
  --venv DIR                  项目 Python 虚拟环境路径；默认 .venv-shm
  --dependency-prefix DIR     Pinocchio/Coal CMake 前缀；默认 /opt/ros/humble
  --jobs N                    C++ 编译并行数；默认按内存自动选择（1 或 2）
EOF
}

while (($#)); do
  case "$1" in
    --check) check_only=1; shift ;;
    --wheelhouse|--venv|--dependency-prefix|--jobs)
      (($# >= 2)) || { usage >&2; exit 2; }
      case "$1" in
        --wheelhouse) wheelhouse=$2 ;;
        --venv) venv=$2 ;;
        --dependency-prefix) dependency_prefix=$2 ;;
        --jobs) jobs=$2 ;;
      esac
      shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ $jobs =~ ^[1-9][0-9]*$ ]] || { echo 'jobs 必须是正整数' >&2; exit 2; }

if ((check_only)); then
  export PYTHONPATH="$repo/python${PYTHONPATH:+:$PYTHONPATH}"
  if [[ -n ${CASBOT_SHM_PYTHON:-} ]]; then
    py="$CASBOT_SHM_PYTHON"
  elif [[ -x "$venv/bin/python" ]]; then
    py="$venv/bin/python"
  elif [[ -n ${VIRTUAL_ENV:-} && -x "$VIRTUAL_ENV/bin/python" ]]; then
    py="$VIRTUAL_ENV/bin/python"
  else
    py=python3
  fi
  exec "$py" -m casbot_calib.pipeline.preflight
fi

[[ -z ${CONDA_PREFIX:-} ]] || { echo '请先退出 conda 环境再运行 calib/setup.sh' >&2; exit 1; }

if [[ -z $wheelhouse ]]; then
  for cand in "$repo"/artifacts/shm_h1_wheels* "$repo"/artifacts/wheelhouse*; do
    if [[ -d $cand ]]; then
      wheelhouse=$cand
      break
    fi
  done
fi

if [[ ! -e "$venv" ]]; then
  if [[ -n $wheelhouse ]]; then
    /usr/bin/python3 -m venv --without-pip "$venv"
  else
    /usr/bin/python3 -m venv "$venv"
  fi
fi

[[ -f "$venv/pyvenv.cfg" && -x "$venv/bin/python" ]] || { echo "无效的虚拟环境目录：$venv" >&2; exit 1; }
"$venv/bin/python" -c 'import sys; assert sys.version_info >= (3, 10); assert sys.prefix != sys.base_prefix'

req_file="$repo/scripts/shm/requirements-shm.txt"
deps_satisfied=0
if "$venv/bin/python" - "$req_file" <<'PY' >/dev/null 2>&1
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
import sys
for raw in Path(sys.argv[1]).read_text(encoding='utf-8').splitlines():
    line = raw.split('#', 1)[0].strip()
    if not line:
        continue
    name, expected = line.split('==', 1)
    try:
        if version(name.strip()) != expected.strip():
            sys.exit(1)
    except PackageNotFoundError:
        sys.exit(1)
PY
then
  deps_satisfied=1
fi

if ((!deps_satisfied)); then
  if ! "$venv/bin/python" -m pip --version >/dev/null 2>&1; then
    [[ -n $wheelhouse && -f "$wheelhouse/pip-25.2-py3-none-any.whl" ]] || {
      echo 'venv 缺少 pip；请安装 python3-venv 或使用 --wheelhouse 指定离线 wheel 目录' >&2
      exit 1
    }
    PYTHONPATH="$wheelhouse/pip-25.2-py3-none-any.whl" "$venv/bin/python" -m pip install \
      --ignore-installed --no-index --find-links "$wheelhouse" pip==25.2
  fi
  pip_args=()
  [[ -z $wheelhouse ]] || pip_args+=(--no-index --find-links "$wheelhouse")
  "$venv/bin/python" -m pip install "${pip_args[@]}" -r "$req_file"
fi

cd "$repo"
env -u PYTHONPATH -u LD_LIBRARY_PATH "$venv/bin/python" scripts/shm/link_math_bindings.py --dependency-prefix "$dependency_prefix"

env -u PYTHONPATH -u LD_LIBRARY_PATH CMAKE_BUILD_PARALLEL_LEVEL="$jobs" "$venv/bin/python" -m pip install \
  -e . --no-deps --no-build-isolation \
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
env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONPATH="$repo/python" "$venv/bin/python" -m casbot_calib.pipeline.preflight --offline
echo '环境构建与预检完成。可直接运行：bash calib/dynamics_h1.sh'
