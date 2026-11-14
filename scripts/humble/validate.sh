#!/usr/bin/env bash
# Run via scripts/humble/run.sh (inside container) or container.sh (host).
set -euo pipefail
output_dir=${1:-artifacts/validation_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$output_dir"
output_dir=$(cd -- "$output_dir" && pwd)
cmake --preset humble -DCALIB_BUILD_PYTHON=ON -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=ON
python -m pip install -e . --no-deps --no-build-isolation --config-settings=build-dir=build/humble \
  --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON
cmake --preset humble -DCALIB_BUILD_PYTHON=ON -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=ON
cmake --build --preset humble
ctest --preset humble --output-junit "$output_dir/ctest.xml"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CASBOT_TEST_RENDER=1 python -m pytest -q --junitxml="$output_dir/pytest.xml"
python scripts/humble/collect_environment.py --output "$output_dir/environment.json"
cmake --install build/humble --component Core
cmake -S tests/installation -B build/humble-consumer -G Ninja \
  -DCMAKE_PREFIX_PATH="$PWD/install/humble-native;/opt/ros/humble"
cmake --build build/humble-consumer
build/humble-consumer/calib_consumer > "$output_dir/native-consumer.txt"
python -m pip wheel . --no-deps --no-build-isolation --wheel-dir "$output_dir/packages" \
  --config-settings=build-dir=build/humble --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON
python scripts/humble/verify_wheel.py --wheel "$output_dir"/packages/*.whl --output "$output_dir/wheel"
cmake --preset humble -DCALIB_BUILD_PYTHON=ON -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=ON
