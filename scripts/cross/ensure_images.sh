#!/usr/bin/env bash
# Build only missing images; --rebuild reapplies Dockerfile changes with layer caching.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
rebuild=0
case "${1:-}" in
  --rebuild) rebuild=1; shift ;;
  -h|--help) echo '用法：bash scripts/cross/ensure_images.sh [--rebuild]'; exit 0 ;;
esac
(($# == 0)) || { echo "未知参数：$*" >&2; exit 2; }
for target in orin-arm64-runtime orin-x86-cross-builder; do
  image="casbot-calib/$target:humble"
  if ((rebuild)) || ! docker image inspect "$image" > /dev/null 2>&1; then
    platform=linux/amd64
    [[ $target != orin-arm64-runtime ]] || platform=linux/arm64
    docker buildx build --load --platform "$platform" --target "$target" \
      --file "$here/Dockerfile.orin" --tag "$image" "$here"
  fi
done
