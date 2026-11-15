#!/usr/bin/env bash
# Step 1: C++/SHM cross build, component installation and QEMU OpenMP check.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
case "${1:-}" in
  -h|--help) echo '用法：bash scripts/cross/build.sh'; exit 0 ;;
esac
(($# == 0)) || { echo "未知参数：$*" >&2; exit 2; }
bash "$repo/scripts/cross/run.sh" bash -c '
  set -e
  cmake --preset orin-cross
  cmake --build --preset orin-cross
  python3 -c "from scripts.cross.package_orin import ROOT, install_components; install_components(ROOT / \"build/orin-cross\", ROOT / \"build/orin-cross-dev-install\", ROOT / \"build/orin-cross-tests\")"
'
bash "$repo/scripts/cross/verify_openmp.sh"
