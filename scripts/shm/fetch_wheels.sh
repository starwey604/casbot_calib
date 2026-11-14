#!/usr/bin/env bash
# Prefetch offline Python 3.10 wheels for Orin (aarch64) and optional host (x86_64)
# into artifacts/wheelhouse_aarch64/ with zero source builds.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
out_dir="$repo/artifacts/wheelhouse_aarch64"
req_file="$repo/scripts/shm/requirements-shm.txt"
include_x86_64=1

usage() {
  cat <<'EOF'
用法：bash scripts/shm/fetch_wheels.sh [选项]
  --output DIR       离线 wheelhouse 输出目录；默认 artifacts/wheelhouse_aarch64
  --aarch64-only     仅下载 Orin aarch64 轮子，不附带本机 x86_64 cp310 轮子
  --requirements F   依赖清单路径；默认 scripts/shm/requirements-shm.txt
EOF
}

while (($#)); do
  case "$1" in
    --aarch64-only) include_x86_64=0; shift ;;
    --output|--requirements)
      (($# >= 2)) || { usage >&2; exit 2; }
      case "$1" in
        --output) out_dir=$2 ;;
        --requirements) req_file=$2 ;;
      esac
      shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

mkdir -p "$out_dir"
out_dir=$(cd -- "$out_dir" && pwd)
req_file=$(realpath "$req_file")

run_pip_download() {
  if python3 -c 'import sys, pip; assert sys.version_info >= (3, 8)' >/dev/null 2>&1; then
    python3 -m pip download "$@"
  else
    bash "$repo/scripts/humble/container.sh" python -m pip download "$@" < /dev/null
  fi
}

echo "正在拉取 Orin (aarch64, Python 3.10) 预编译离线轮子至 $out_dir ..."
run_pip_download \
  --only-binary=:all: \
  --python-version 310 \
  --platform manylinux2014_aarch64 \
  --platform manylinux_2_17_aarch64 \
  --platform manylinux_2_24_aarch64 \
  --platform manylinux_2_27_aarch64 \
  --platform manylinux_2_28_aarch64 \
  --platform any \
  -d "$out_dir" \
  -r "$req_file"

if ((include_x86_64)); then
  echo "正在拉取本机 (x86_64, Python 3.10) 预编译离线轮子至 $out_dir ..."
  run_pip_download \
    --only-binary=:all: \
    --python-version 310 \
    --platform manylinux2014_x86_64 \
    --platform manylinux_2_17_x86_64 \
    --platform manylinux_2_24_x86_64 \
    --platform manylinux_2_27_x86_64 \
    --platform manylinux_2_28_x86_64 \
    --platform any \
    -d "$out_dir" \
    -r "$req_file"
fi

count=$(find "$out_dir" -maxdepth 1 -name '*.whl' | wc -l)
echo "离线 wheelhouse 准备完成：共 $count 个 .whl（目录：$out_dir）"
