#!/usr/bin/env bash
# Optional SDK overlay; reuses the native ament core without replacing Python bindings.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
sdk_dir=${CASBOT_SDK_ROOT:-$(dirname "$repo_dir")/casbot2-ros2-sdk}
if [[ ! -f "$sdk_dir/crb_ros_msg/package.xml" ]]; then echo "SDK not found: $sdk_dir" >&2; exit 1; fi
cd "$repo_dir"
# Verify and save SDK identity before building its messages.
python -m casbot_calib.vendor_identity "$sdk_dir" --output build/vendor-source.json
env -u VIRTUAL_ENV PATH=/opt/ros/humble/bin:/usr/bin:/bin colcon --log-base log/vendor build \
  --base-paths "$sdk_dir/crb_ros_msg" "$repo_dir/ros/casbot_calib_core" "$repo_dir/ros/casbot_calib_interfaces" "$repo_dir/ros/casbot_calib_vendor" \
  --build-base build/vendor --install-base install/vendor --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
cp build/vendor-source.json install/vendor/vendor-source.json
