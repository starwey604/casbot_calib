#!/usr/bin/env bash
# Explicit optional overlay; leaves ROS domain/network settings to the caller.
set -eo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ ! -f "$repo_dir/install/vendor/local_setup.bash" ]]; then echo 'Run build_vendor.sh first.' >&2; exit 1; fi
source "$repo_dir/install/vendor/local_setup.bash"
export CASBOT_VENDOR_INSTALL="$repo_dir/install/vendor"
exec "$@"
