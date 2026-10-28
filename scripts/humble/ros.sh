#!/usr/bin/env bash
# Run inside the Humble environment via container.sh; adds this repository's ROS overlay.
set -eo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ ! -f "$repo_dir/install/humble-ros/setup.bash" ]]; then echo 'Build the ROS packages first (see docs/ros_transport.md).' >&2; exit 1; fi
source "$repo_dir/install/humble-ros/setup.bash"
exec "$@"
