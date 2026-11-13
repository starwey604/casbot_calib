#!/usr/bin/env bash
# Explicit physical-control handoff. Never called by build/deploy/test scripts.
set -euo pipefail
if [[ ${1:-} == --help || $# != 1 ]]; then
  echo '用法：bash scripts/shm/prepare_field.sh NEW_LOG_DIRECTORY'
  echo '会停止厂家控制业务并只启动EtherCAT主站；先安排固定支撑和现场继电器。'
  exit 0
fi
logs=$1
launcher=/workspace/embed_fusion/bin/launcher.bash
station=/workspace/embed_fusion/bin/run_ethercat_station.bash
[[ -f $launcher && -f $station ]] || { echo '厂家入口不存在，未停止业务' >&2; exit 1; }
sudo -n true
mkdir -p "$(dirname "$logs")"
mkdir "$logs"
logs=$(cd "$logs" && pwd)
echo '停止独立服务与厂家业务；当前支撑由现场负责。'
sudo -n systemctl stop zero_calibration.service arm_debug_tool.service
sudo -n bash "$launcher" stop 2>&1 | tee "$logs/stop.log"
if pgrep -x hlorin >/dev/null || pgrep -x ethercat_station >/dev/null; then
  echo '原运控/旧主站仍存在，未启动新主站' >&2
  exit 1
fi
# Keep the SSH command line free of vendor launcher names: their broad process
# matching must not accidentally terminate the orchestration shell itself.
nohup sudo -n bash "$station" > "$logs/station.log" 2>&1 < /dev/null &
echo "$!" > "$logs/station_launcher.pid"
echo "主站启动日志：$logs/station.log"
echo '此脚本不发关节指令、不自动恢复旧业务；测试窗口持续到下次Orin重启。'
