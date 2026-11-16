#!/usr/bin/env bash
# Exercise the installed ARM64 OpenMP probe using the runtime baseline image.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
case "${1:-}" in
  -h|--help) echo '用法：bash scripts/cross/verify_openmp.sh'; exit 0 ;;
esac
(($# == 0)) || { echo "未知参数：$*" >&2; exit 2; }
exec docker run --rm --platform linux/arm64 --user "$(id -u):$(id -g)" \
  --volume "$repo:$repo:ro" --workdir "$repo" \
  casbot-calib/orin-arm64-runtime:humble \
  env -u LD_LIBRARY_PATH -u PYTHONPATH bash -c '
    set -euo pipefail
    prefix="$PWD/build/orin-cross-dev-install"
    for program in shm_executor shm_trical_executor shm_mock_server shm_gui_owner shm_reference_check shm_abi_check shm_channel_start; do
      readelf -h "$prefix/bin/$program" | grep -q "Machine:.*AArch64"
      dynamic=$(readelf -d "$prefix/bin/$program")
      if grep -E "(RPATH|RUNPATH).*(/sysroot|/opt/orin-openmp|/build/)" <<< "$dynamic"; then
        echo "$program contains a build-only library path" >&2
        exit 1
      fi
      libraries=$(ldd "$prefix/bin/$program")
      if grep -F "not found" <<< "$libraries"; then
        echo "$program has unresolved libraries" >&2
        exit 1
      fi
      echo "$program: AArch64, libraries resolved"
    done
    prefix="$PWD/build/orin-cross-tests"
    readelf -d "$prefix/bin/openmp_probe" | grep -F "libomp.so.5"
    readelf -d "$prefix/bin/openmp_probe" | grep -F '\''$ORIGIN/../lib'\''
    ldd "$prefix/bin/openmp_probe" | grep -F "$prefix/bin/../lib/libomp.so.5"
    for threads in 1 4; do
      OMP_NUM_THREADS=$threads OMP_DYNAMIC=FALSE "$prefix/bin/openmp_probe" "$threads"
    done
    "$PWD/build/orin-cross-dev-install/bin/shm_abi_check"
  '
