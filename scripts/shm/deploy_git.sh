#!/usr/bin/env bash
# Clone/fetch committed source through an SSH tunnel to a temporary, read-only
# loopback Git server. No public listener, repository push, or robot operations.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
host=''
destination=/home/nvidia/casbot_calib_shm
branch=main
port=19418
committed_only=0
control_path=${CALIB_SSH_CONTROL_PATH:-/tmp/casbot-shm-%C}
usage() {
  cat <<'EOF'
用法：bash scripts/shm/deploy_git.sh --host nvidia@172.16.0.10 [选项]
  --destination PATH   Orin上新克隆的绝对路径；默认 /home/nvidia/casbot_calib_shm
  --branch NAME        本地已提交分支；默认main
  --port N             本机与远端的loopback临时隧道端口；默认19418
  --committed-only     本地另有未提交工作时，仍只部署指定分支的已提交内容
仅克隆或快进更新本脚本的源码仓库；默认要求本地干净，拒绝目标改动和分叉历史。
LFS大资产保持指针，本轮ABI/核心URDF测试无需网格资产；不会连接真实SHM。
SSH认证走正常SSH交互/agent/ControlMaster，不保存密码。
EOF
}
while (($#)); do
  case "$1" in
    --committed-only) committed_only=1; shift ;;
    --host|--destination|--branch|--port)
      (($# >= 2)) || { usage >&2; exit 2; }
      case "$1" in
        --host) host=$2 ;; --destination) destination=$2 ;;
        --branch) branch=$2 ;; --port) port=$2 ;;
      esac
      shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ -n $host && $host != -* ]] || { usage >&2; exit 2; }
[[ $destination == /* ]] || { echo '目标必须是绝对路径' >&2; exit 2; }
[[ $port =~ ^[0-9]+$ && ${#port} -le 5 ]] && ((10#$port > 1024 && 10#$port < 65536)) || exit 2
git -C "$repo" check-ref-format --branch "$branch" >/dev/null
git -C "$repo" show-ref --verify --quiet "refs/heads/$branch"
if [[ -n $(git -C "$repo" status --porcelain) ]]; then
  ((committed_only)) || { echo '请先提交本地工作区，或显式使用--committed-only' >&2; exit 1; }
  echo '只部署指定分支已提交内容；未提交工作不传输。' >&2
fi
git -C "$repo" log -1 --oneline "$branch"
ssh_options=(-o ControlMaster=auto -o ControlPersist=60 -o "ControlPath=$control_path"
  -o ConnectTimeout=10 -o ExitOnForwardFailure=yes)
forward="127.0.0.1:$port:127.0.0.1:$port"
daemon_pid=''
forwarded=0
cleanup() {
  if ((forwarded)); then ssh "${ssh_options[@]}" -O cancel -R "$forward" "$host" >/dev/null 2>&1 || true; fi
  if [[ -n $daemon_pid ]]; then kill "$daemon_pid" 2>/dev/null || true; wait "$daemon_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
git daemon --reuseaddr --export-all --strict-paths --listen=127.0.0.1 --port="$port" \
  --base-path="$(dirname "$repo")" --disable=receive-pack --disable=upload-archive "$repo/.git" &
daemon_pid=$!
# Authenticate/create a reusable connection before requesting a scoped forward.
ssh "${ssh_options[@]}" "$host" true
kill -0 "$daemon_pid"
ssh "${ssh_options[@]}" -O forward -R "$forward" "$host"
forwarded=1
printf -v remote_command 'bash -s -- %q %q %q' "$destination" "$branch" "git://127.0.0.1:$port/$(basename "$repo")/.git"
ssh "${ssh_options[@]}" "$host" "$remote_command" <<'REMOTE'
set -euo pipefail
destination=$1
branch=$2
url=$3
export GIT_LFS_SKIP_SMUDGE=1
# User-local Git LFS must also be visible to its standard Git hooks.
export PATH="$HOME/.local/bin:$PATH"
if [[ -e $destination ]]; then
  [[ -d $destination/.git ]] || { echo '目标已存在且不是独立Git工作区，未修改' >&2; exit 1; }
  [[ -z $(git -C "$destination" status --porcelain) ]] || { echo '目标有改动，未更新' >&2; exit 1; }
  [[ $(git -C "$destination" remote get-url origin) == "$url" ]] || { echo '目标origin不属于本次本地源码隧道，未修改' >&2; exit 1; }
  [[ $(git -C "$destination" branch --show-current) == "$branch" ]] || { echo '目标当前分支不同，未切换' >&2; exit 1; }
  git -C "$destination" fetch origin "$branch"
  git -C "$destination" merge --ff-only FETCH_HEAD
else
  git clone --single-branch --branch "$branch" "$url" "$destination"
fi
git -C "$destination" log -1 --oneline
REMOTE
