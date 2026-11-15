#!/usr/bin/env bash
# One-click Orin deployment from host: ensure wired link, sync clock, push committed Git/assets/wheels, and build/preflight on Orin.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
host=nvidia@172.16.0.10
destination=/home/nvidia/casbot_calib_shm
branch=main
interface=''
run_setup=1
check_only=0
jobs=''
deploy_git_args=()

usage() {
  cat <<'EOF'
用法：bash scripts/shm/deploy_orin.sh [选项]
  --host USER@HOST     Orin SSH 目标；默认 nvidia@172.16.0.10
  --interface IFACE    本机直连网卡名（默认自动探测 USB 有线网卡）
  --destination PATH   Orin 目标仓库路径；默认 /home/nvidia/casbot_calib_shm
  --branch NAME        部署分支；默认 main
  --committed-only     本地有未提交改动时，仍仅部署指定分支已提交内容
  --no-assets          不自动同步默认 DVT2 模型资产
  --no-wheels          不同步离线 wheelhouse
  --no-setup           仅同步源码/资产/轮子，不在 Orin 上运行 calib/setup.sh
  --check-only         同步后仅运行 calib/setup.sh --check 预检，不重编
  --jobs N             传给远端 calib/setup.sh 的编译并行数
EOF
}

while (($#)); do
  case "$1" in
    --committed-only|--no-assets|--no-wheels)
      deploy_git_args+=("$1"); shift ;;
    --no-setup) run_setup=0; shift ;;
    --check-only) check_only=1; shift ;;
    --host|--interface|--destination|--branch|--jobs)
      (($# >= 2)) || { usage >&2; exit 2; }
      case "$1" in
        --host) host=$2 ;;
        --interface) interface=$2 ;;
        --destination) destination=$2 ;;
        --branch) branch=$2 ;;
        --jobs) jobs=$2 ;;
      esac
      shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

target_addr=${host#*@}
if [[ -n $interface ]]; then
  bash "$repo/scripts/jetson_network.sh" up "$interface"
elif [[ $target_addr == 172.16.0.10 ]] && ! ping -n -c 1 -W 1 "$target_addr" >/dev/null 2>&1; then
  bash "$repo/scripts/jetson_network.sh" up
fi

control_path=${CALIB_SSH_CONTROL_PATH:-/tmp/casbot-shm-%C}
export CALIB_SSH_CONTROL_PATH="$control_path"
ssh_options=(-o ControlMaster=auto -o ControlPersist=120 -o "ControlPath=$control_path" -o ConnectTimeout=10)

# Establish reusable SSH session once, then align Orin clock if stuck near epoch (1970).
ssh "${ssh_options[@]}" "$host" true
host_epoch=$(date -u +%s)
printf -v sync_clock_cmd 'if (( $(date -u +%%Y) < 2025 )); then sudo -n date -u -s @%q >/dev/null 2>&1 || true; fi' "$host_epoch"
ssh "${ssh_options[@]}" "$host" "$sync_clock_cmd"

bash "$repo/scripts/shm/deploy_git.sh" \
  --host "$host" \
  --destination "$destination" \
  --branch "$branch" \
  "${deploy_git_args[@]}"

if ((run_setup)); then
  setup_args=()
  ((check_only)) && setup_args+=(--check)
  [[ -z $jobs ]] || setup_args+=(--jobs "$jobs")
  printf -v remote_setup 'cd %q && bash calib/setup.sh' "$destination"
  for arg in "${setup_args[@]}"; do
    printf -v q_arg '%q' "$arg"
    remote_setup+=" $q_arg"
  done
  ssh "${ssh_options[@]}" "$host" "$remote_setup"
fi
