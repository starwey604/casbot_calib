#!/usr/bin/env bash
# Host entry point for the existing Docker-backed distrobox. Its shared home must
# contain this repository at the same absolute path. Host conda is not inherited.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
container_name=${CALIB_HUMBLE_CONTAINER:-ros2-humble}
if [[ "$(docker inspect --format '{{.State.Running}}' "$container_name")" != true ]]; then
  docker start "$container_name" >/dev/null
fi
exec docker exec -i --user "$(id -u):$(id -g)" --workdir "$repo_dir" "$container_name" \
  bash --noprofile --norc "$repo_dir/scripts/humble/run.sh" "$@"
