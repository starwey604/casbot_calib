#!/usr/bin/env bash
# Native build/test only. No ROS setup or package installs. Tests use private SHM, never shm_mc.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
build_dir="$repo/build/shm-native"
prefix="$repo/install/shm-native"
dependency_prefix=''
core=OFF
mock=OFF
jobs=1
usage() {
  cat <<'EOF'
用法：bash scripts/shm/build.sh [选项]
  --with-core                 同时编译动力学核心和现场executor
  --with-mock                 额外编译真实双臂物理SHM Mock（隐含--with-core）
  --build-dir PATH            构建目录；默认 build/shm-native
  --install-prefix PATH       安装目录；默认 install/shm-native
  --dependency-prefix PATH    显式算法库CMake前缀，可用分号分隔
  --jobs N                    编译并行数；默认1
本入口配置、编译、运行测试并安装Core/Shm组件；不安装大型Models资产。
EOF
}
while (($#)); do
  case "$1" in
    --with-core) core=ON; shift ;;
    --with-mock) core=ON; mock=ON; shift ;;
    --build-dir|--install-prefix|--dependency-prefix|--jobs)
      (($# >= 2)) || { usage >&2; exit 2; }
      case "$1" in
        --build-dir) build_dir=$2 ;; --install-prefix) prefix=$2 ;;
        --dependency-prefix) dependency_prefix=$2 ;; --jobs) jobs=$2 ;;
      esac
      shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ $jobs =~ ^[1-9][0-9]*$ ]] || { echo 'jobs必须是正整数' >&2; exit 2; }
args=(-S "$repo" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
  -DCALIB_BUILD_CORE="$core" -DCALIB_BUILD_PYTHON=OFF
  -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK="$mock" -DBUILD_TESTING=ON -DCMAKE_INSTALL_PREFIX="$prefix")
[[ -z $dependency_prefix ]] || args+=(-DCMAKE_PREFIX_PATH="$dependency_prefix")
cmake "${args[@]}"
cmake --build "$build_dir" --parallel "$jobs"
ctest --test-dir "$build_dir" --output-on-failure
if [[ $core == ON ]]; then cmake --install "$build_dir" --component Core; fi
cmake --install "$build_dir" --component Shm
