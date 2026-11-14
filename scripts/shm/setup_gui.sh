#!/usr/bin/env bash
# Reuse the established Humble environment; do not install graphics on Orin.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo"
bash scripts/humble/container.sh .venv-humble/bin/python -c 'import imgui_bundle, mujoco, OpenGL, numpy; from casbot_calib.model_context import load_context; print(load_context().model_id)'
bash scripts/humble/container.sh cmake --preset humble -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=ON
bash scripts/humble/container.sh cmake --build build/humble --target shm_gui_bundle --parallel 1
bash scripts/humble/container.sh cmake --install build/humble --component Shm --prefix "$repo/install/shm-core"
echo '已准备本机GUI依赖和原生程序。启动：bash fieldwork/shm/run.sh gui --mock'
