#!/usr/bin/env bash
# Run via scripts/humble/run.sh (inside container) or container.sh (host).
set -euo pipefail
output_dir=${1:?Usage: validate.sh OUTPUT_DIRECTORY [FROZEN_BASELINE_DIRECTORY]}
baseline_dir=${2:-}
[[ ! -e "$output_dir" ]] || { echo "Output already exists: $output_dir" >&2; exit 1; }
mkdir -p "$output_dir"
output_dir=$(cd -- "$output_dir" && pwd)
cmake --preset humble -DCALIB_BUILD_PYTHON=ON
python -m pip install -e . --no-deps --no-build-isolation --config-settings=build-dir=build/humble \
  --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON
cmake --preset humble -DCALIB_BUILD_PYTHON=ON
cmake --build --preset humble
ctest --preset humble --output-junit "$output_dir/ctest.xml"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CASBOT_TEST_RENDER=1 python -m pytest -q --junitxml="$output_dir/pytest.xml"
python scripts/humble/collect_environment.py --output "$output_dir/environment.json"
cmake --install build/humble --component Core
cmake -S tests/installation -B build/humble-consumer -G Ninja \
  -DCMAKE_PREFIX_PATH="$PWD/install/humble-native;/opt/ros/humble"
cmake --build build/humble-consumer
build/humble-consumer/calib_consumer > "$output_dir/native-consumer.txt"
env PATH="/opt/ros/humble/bin:/usr/bin:/bin" colcon --log-base log/humble build --base-paths ros --build-base build/humble-ros \
  --install-base install/humble-ros --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake -S tests/installation -B build/humble-ament-consumer -G Ninja \
  -DCALIB_CONSUME_AMENT=ON -DCMAKE_PREFIX_PATH="$PWD/install/humble-ros/casbot_calib_core;/opt/ros/humble"
cmake --build build/humble-ament-consumer
build/humble-ament-consumer/calib_consumer > "$output_dir/ament-consumer.txt"
python -m pip wheel . --no-deps --no-build-isolation --wheel-dir "$output_dir/packages" \
  --config-settings=build-dir=build/humble --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON
python scripts/humble/verify_wheel.py --wheel "$output_dir"/packages/*.whl --output "$output_dir/wheel"
cmake --preset humble -DCALIB_BUILD_PYTHON=ON
casbot-calib suite --configs configs --output "$output_dir/experiments"
casbot-calib arm-check --output "$output_dir/experiments/R1" --render
casbot-calib dual-calibrate --output "$output_dir/experiments/D14" --render
if [[ -n "$baseline_dir" ]]; then
  python scripts/compare_reports.py "$baseline_dir" "$output_dir/experiments" | tee "$output_dir/baseline-comparison.txt"
fi
python scripts/humble/verify_results.py --output "$output_dir"
