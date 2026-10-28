#!/usr/bin/env bash
set -euo pipefail
# Humble uses both modern Python3 and legacy PYTHON variables in generators.
# Keep all ROS build tools on the apt interpreter, not the analysis venv.
exec env -u VIRTUAL_ENV PATH="/opt/ros/humble/bin:/usr/bin:/bin" colcon --log-base log/humble build \
  --base-paths ros --build-base build/humble-ros --install-base install/humble-ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
