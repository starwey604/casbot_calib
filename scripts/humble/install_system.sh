#!/usr/bin/env bash
set -euo pipefail
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo 'Use a clean Humble container shell, without conda activation.' >&2; exit 1
fi
source /etc/os-release
export DEBIAN_FRONTEND=noninteractive
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo 'Ubuntu 22.04 required' >&2; exit 1; }
privilege=()
if [[ $EUID -ne 0 ]]; then privilege=(sudo); fi
"${privilege[@]}" apt-get update
"${privilege[@]}" apt-get install -y --no-install-recommends \
  build-essential cmake ninja-build python3-venv python3-pip \
  python3-colcon-common-extensions ros-humble-rclcpp ros-humble-rosidl-default-generators \
  ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins \
  ros-humble-pinocchio ros-humble-coal ros-humble-ros2-control ros-humble-ros2-controllers \
  libegl1 libgl1-mesa-dri fonts-noto-cjk
