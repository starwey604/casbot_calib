#!/usr/bin/env bash
# Run an amd64 cross-build command with the repository at its host path.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if (($# == 0)) || [[ ${1:-} == --help || ${1:-} == -h ]]; then
  echo '用法：bash scripts/cross/run.sh COMMAND [ARGS...]'
  exit 0
fi
bash "$repo/scripts/cross/ensure_images.sh"
mkdir -p "$repo/build/ccache-orin-cross"
exec docker run --rm --platform linux/amd64 --user "$(id -u):$(id -g)" \
  --volume "$repo:$repo" --workdir "$repo" \
  --env HOME=/tmp --env CCACHE_DIR="$repo/build/ccache-orin-cross" \
  casbot-calib/orin-x86-cross-builder:humble "$@"
