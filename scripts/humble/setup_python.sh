#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ -n "${CONDA_PREFIX:-}" ]]; then echo 'Do not mix conda and Humble.' >&2; exit 1; fi
export PYTHONNOUSERSITE=1
set +u  # ROS underlay setup scripts are not nounset-safe.
source /opt/ros/humble/setup.bash
set -u
/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3,10)'
# ROS underlay exposes rclpy/Pinocchio/Coal through its explicit PYTHONPATH.
# Do not expose all apt Python packages: legacy namespace .pth files (e.g.
# matplotlib) can override newer venv packages even with the intended path order.
/usr/bin/python3 -m venv "$repo_dir/.venv-humble"
source "$repo_dir/.venv-humble/bin/activate"
requirements="$repo_dir/scripts/humble/requirements.in"
if [[ -f "$repo_dir/scripts/humble/requirements.lock" ]]; then requirements="$repo_dir/scripts/humble/requirements.lock"; fi
python -m pip install --no-cache-dir -r "$requirements"
python -m pip check
python -c 'import rclpy, pinocchio, coal, numpy, scipy; print("Humble native imports:",pinocchio.__version__,numpy.__version__,scipy.__version__)'
