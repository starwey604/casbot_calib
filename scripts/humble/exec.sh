#!/usr/bin/env bash
# Unified host -> container command runner for the Humble environment.
#
# This is the only place that talks to a container runtime. Defaults are
# deliberately venv-free: bootstrap stages must not depend on run.sh, which
# sources .venv-humble. container.sh is the venv-enabled wrapper.
#
# Runtime selection:
#   CALIB_CONTAINER_RUNTIME=auto|docker|podman  (default auto)
#   auto prefers podman when it owns the container, otherwise falls back to
#   docker. Preferring podman avoids the "Emulate Docker CLI using podman"
#   notice emitted on every call through podman-docker.
#
# Container name: CALIB_HUMBLE_CONTAINER (default ros2-humble).
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
container_name=${CALIB_HUMBLE_CONTAINER:-ros2-humble}
runtime_mode=${CALIB_CONTAINER_RUNTIME:-auto}
user_spec="$(id -u):$(id -g)"
use_venv=0
workdir="$repo_dir"
dry_run=${CALIB_CONTAINER_DRY_RUN:-0}
print_runtime=0

usage() {
  cat <<'EOF'
Usage: exec.sh [OPTIONS] [--] COMMAND [ARGS...]

Options:
  --user SPEC       Container user (default: current uid:gid; "root" for apt).
  --venv            Run COMMAND through run.sh (sources ROS underlay + venv).
  --no-venv         Run COMMAND directly, with no venv dependency (default).
  --workdir DIR     Container working directory (default: repository root).
  --runtime MODE    Force docker or podman instead of auto detection.
  --name NAME       Container name (default: $CALIB_HUMBLE_CONTAINER or ros2-humble).
  --print-runtime   Print the detected runtime and exit.
  --dry-run         Print the command without executing or starting the container.
  -h, --help        Show this help.

Environment: CALIB_CONTAINER_DRY_RUN=1 behaves like --dry-run, which is
useful for checking container.sh call chains without a running container.

Examples:
  exec.sh --user root --no-venv -- bash scripts/humble/install_system.sh
  exec.sh --no-venv -- bash scripts/humble/setup_python.sh
  exec.sh --venv -- python -m casbot_calib.shm_fieldwork list
EOF
}

die() { printf '[exec.sh] error: %s\n' "$*" >&2; exit 1; }

detect_runtime() {
  case "$runtime_mode" in
    docker|podman) printf '%s' "$runtime_mode"; return 0 ;;
    auto) ;;
    *) die "unsupported CALIB_CONTAINER_RUNTIME=$runtime_mode (use auto, docker or podman)" ;;
  esac
  if command -v podman >/dev/null 2>&1 && podman inspect "$container_name" >/dev/null 2>&1; then
    printf 'podman'; return 0
  fi
  if command -v docker >/dev/null 2>&1 && docker inspect "$container_name" >/dev/null 2>&1; then
    printf 'docker'; return 0
  fi
  return 1
}

ensure_running() {
  local rt=$1 state
  state=$("$rt" inspect --format '{{.State.Running}}' "$container_name" 2>/dev/null || printf 'false')
  if [[ "$state" != true ]]; then
    printf '[exec.sh] starting container %s\n' "$container_name" >&2
    "$rt" start "$container_name" >/dev/null
  fi
}

[[ "$dry_run" == 0 || "$dry_run" == 1 ]] || die "CALIB_CONTAINER_DRY_RUN must be 0 or 1"

while (($#)); do
  case "$1" in
    --user)         [[ $# -ge 2 ]] || die "--user needs a value"; user_spec=$2; shift 2 ;;
    --user=*)       user_spec=${1#*=}; shift ;;
    --venv)         use_venv=1; shift ;;
    --no-venv)      use_venv=0; shift ;;
    --workdir)      [[ $# -ge 2 ]] || die "--workdir needs a value"; workdir=$2; shift 2 ;;
    --workdir=*)    workdir=${1#*=}; shift ;;
    --runtime)      [[ $# -ge 2 ]] || die "--runtime needs a value"; runtime_mode=$2; shift 2 ;;
    --runtime=*)    runtime_mode=${1#*=}; shift ;;
    --name)         [[ $# -ge 2 ]] || die "--name needs a value"; container_name=$2; shift 2 ;;
    --name=*)       container_name=${1#*=}; shift ;;
    --print-runtime) print_runtime=1; shift ;;
    --dry-run)      dry_run=1; shift ;;
    -h|--help)      usage; exit 0 ;;
    --)             shift; break ;;
    -*)             die "unknown option: $1" ;;
    *)              break ;;
  esac
done

case "$runtime_mode" in
  auto|docker|podman) ;;
  *) die "unsupported CALIB_CONTAINER_RUNTIME=$runtime_mode (use auto, docker or podman)" ;;
esac

rt=$(detect_runtime) || die "container '$container_name' not found in podman or docker.
Create it first (this script never creates containers), e.g. on this host:
  distrobox create --name $container_name --image docker.io/library/ros:humble --yes
Then re-run. Set CALIB_CONTAINER_RUNTIME to select a runtime explicitly."

if ((print_runtime)); then
  printf '%s\n' "$rt"
  exit 0
fi

(($#)) || die "no command given (use -- to separate options from COMMAND)"

if (( ! dry_run )); then ensure_running "$rt"; fi

cmd=("$rt" exec -i)
[[ -n "$user_spec" ]] && cmd+=(--user "$user_spec")
cmd+=(--workdir "$workdir" "$container_name")
if ((use_venv)); then
  cmd+=(bash --noprofile --norc "$repo_dir/scripts/humble/run.sh")
else
  cmd+=(bash --noprofile --norc -c 'exec "$@"' _)
fi
cmd+=("$@")

printf '[exec.sh] ' >&2
printf '%q ' "${cmd[@]}" >&2
printf '\n' >&2

if ((dry_run)); then exit 0; fi
exec "${cmd[@]}"
