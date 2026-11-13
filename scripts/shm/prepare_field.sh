#!/usr/bin/env bash
# Explicit physical-control handoff. Never called by build/deploy/test scripts.
set -euo pipefail
if [[ ${1:-} == --help || $# == 0 || $# -gt 2 ]]; then
  echo '用法：bash scripts/shm/prepare_field.sh NEW_LOG_DIRECTORY'
  echo '或：bash scripts/shm/prepare_field.sh --start-only EXISTING_LOG_DIRECTORY'
  echo '会停止厂家控制业务并只启动EtherCAT主站；先安排固定支撑和现场继电器。'
  exit 0
fi
start_only=0
if [[ $1 == --start-only ]]; then
  [[ $# == 2 ]] || exit 2
  start_only=1;logs=$2
else
  [[ $# == 1 ]] || exit 2
  logs=$1
fi
launcher=/workspace/embed_fusion/bin/launcher.bash
station=/workspace/embed_fusion/bin/run_ethercat_station.bash
[[ -f $launcher && -f $station ]] || { echo '厂家入口不存在，未停止业务' >&2; exit 1; }
sudo -n true
if ((start_only)); then
  [[ -d $logs && -f $logs/stop.log ]] || { echo '需要本次已有的停业务日志目录' >&2; exit 1; }
else
  mkdir -p "$(dirname "$logs")";mkdir "$logs"
fi
logs=$(cd "$logs" && pwd)
if ((!start_only)); then
  echo '停止独立服务与厂家业务；当前支撑由现场负责。'
  sudo -n systemctl stop zero_calibration.service arm_debug_tool.service
  set +e
  sudo -n bash "$launcher" stop 2>&1 | tee "$logs/stop.log"
  stop_rc=${PIPESTATUS[0]}
  set -e
  echo "$stop_rc" > "$logs/stop.returncode"
  if ((stop_rc)); then echo "厂家stop返回$stop_rc，按实际服务/进程状态决定是否已停完。"; fi
fi
for service in hl_orin_app.service zero_calibration.service arm_debug_tool.service; do
  state=$(systemctl is-active "$service" || true)
  printf '%s %s\n' "$service" "$state" | tee -a "$logs/stopped_services.txt"
  case "$state" in inactive|failed) ;; *) echo '服务尚未停完，未启动主站' >&2;exit 1 ;; esac
done
if pgrep -x hlorin >/dev/null || pgrep -f '^/workspace/embed_fusion/+bin/+ethercat_station($| )' >/dev/null; then
  echo '原运控/旧主站仍存在，未启动新主站' >&2
  exit 1
fi
# Keep the SSH command line free of vendor launcher names: their broad process
# matching must not accidentally terminate the orchestration shell itself.
nohup sudo -n bash "$station" > "$logs/station.log" 2>&1 < /dev/null &
echo "$!" > "$logs/station_launcher.pid"
echo "主站启动日志：$logs/station.log"
echo '此脚本不发关节指令、不自动恢复旧业务；测试窗口持续到下次Orin重启。'
