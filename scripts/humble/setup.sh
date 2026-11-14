#!/usr/bin/env bash
# Single host-side entry point for the Humble environment (Phase 0).
#
# Stages, in dependency order:
#   preflight  runtime / container / OS / ROS underlay / Python / path / uid
#   system     install_system.sh as root (no venv)
#   python     setup_python.sh as the current user (creates .venv-humble)
#   build      editable install + cmake configure/build + ctest (via run.sh)
#   verify     CLI and import smoke check, then write the state report
#
# Bootstrap is loop-free: only "build" and "verify" go through run.sh, which
# sources the venv. The two install stages use exec.sh --no-venv instead.
#
# State lives under build/humble-setup/ (build/ is gitignored):
#   stamps/<stage>.ok   completion markers (source of truth)
#   state.json          human/CI readable summary, regenerated each run
#
# An existing .venv-humble is detected and the system/python stages are then
# inferred as satisfied, so running this on an already working checkout never
# rebuilds the venv. Use --force system or --no-skip to override.
#
# This script never creates containers (Phase 2) and never touches hardware.
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
scripts_dir="$repo_dir/scripts/humble"
state_dir="$repo_dir/build/humble-setup"
stamps_dir="$state_dir/stamps"
state_file="$state_dir/state.json"
container_name=${CALIB_HUMBLE_CONTAINER:-ros2-humble}

stages=(preflight system python build verify)
declare -A stage_status=() stage_ts=() stage_detail=() force_stage=()
target_stage=verify
only_stage=""
dry_run=0
no_skip=0
doctor_mode=0
detected_runtime="unknown"

usage() {
  cat <<'EOF'
Usage: bash scripts/humble/setup.sh [OPTIONS]

  (no options)    Run all pending stages in dependency order.
  --stage NAME    Run up to NAME (inclusive).
  --only NAME     Run just NAME, ignoring skip markers and dependency state.
  --force NAME    Re-run NAME and every later stage (repeatable).
  --no-skip       Re-run every stage up to the target.
  --doctor        Check preconditions and print a JSON report; do not install.
  --dry-run       Print the commands without touching the container or state.

  NAME is one of: preflight, system, python, build, verify.

Environment:
  CALIB_HUMBLE_CONTAINER   container name (default ros2-humble)
  CALIB_CONTAINER_RUNTIME  auto | docker | podman (default auto)

Examples:
  bash scripts/humble/setup.sh --dry-run
  bash scripts/humble/setup.sh
  bash scripts/humble/setup.sh --stage python
  bash scripts/humble/setup.sh --only build
  bash scripts/humble/setup.sh --force system
  bash scripts/humble/setup.sh --doctor

Container creation is out of scope here (Phase 2). If the container is
missing, preflight prints the one-time creation hint and stops.
EOF
}

die() { printf '[setup] error: %s\n' "$*" >&2; exit 1; }

need_value() {
  if [[ $# -lt 2 || -z "$2" || "$2" == --* ]]; then die "$1 needs a value"; fi
}

valid_stage() {
  local s
  for s in "${stages[@]}"; do
    if [[ "$s" == "$1" ]]; then return 0; fi
  done
  return 1
}

stage_index() {
  local i
  for i in "${!stages[@]}"; do
    if [[ "${stages[$i]}" == "$1" ]]; then printf '%s' "$i"; return 0; fi
  done
  return 1
}

json_escape() {
  local s=${1//\\/\\\\}
  s=${s//\"/\\\"}
  s=${s//$'\n'/ }
  s=${s//$'\t'/ }
  printf '%s' "$s"
}

write_stamp() {
  local s=$1 ts=$2 detail=$3
  if ((dry_run)); then return 0; fi
  mkdir -p "$stamps_dir"
  printf '%s\n%s\n' "$ts" "$detail" > "$stamps_dir/$s.ok"
}

load_state() {
  local s stamp seen_gap=0
  local -a lines=()
  for s in "${stages[@]}"; do
    stage_status[$s]=pending
    stage_ts[$s]=""
    stage_detail[$s]=""
    stamp="$stamps_dir/$s.ok"
    if [[ -f "$stamp" ]]; then
      lines=()
      mapfile -t lines < "$stamp"
      stage_status[$s]=ok
      stage_ts[$s]="${lines[0]:-}"
      stage_detail[$s]="${lines[1]:-}"
    fi
  done
  # Once any stage is not done, every later stage must be considered stale.
  for s in "${stages[@]}"; do
    if ((seen_gap)) || ((no_skip)) || [[ -n "${force_stage[$s]:-}" ]]; then
      stage_status[$s]=pending
    fi
    if [[ "${stage_status[$s]}" != ok ]]; then seen_gap=1; fi
  done
}

infer_existing_environment() {
  if ((no_skip)); then return 0; fi
  if [[ ! -f "$repo_dir/.venv-humble/bin/activate" ]]; then return 0; fi
  local s changed=0 blocked=0
  # Only infer stages before the earliest --force: --force X means redo from
  # X onwards, so X and everything after it must stay pending.
  for s in system python; do
    if [[ -n "${force_stage[$s]:-}" ]]; then blocked=1; fi
    if ((blocked)); then continue; fi
    if [[ "${stage_status[$s]}" != ok ]]; then
      stage_status[$s]=ok
      stage_ts[$s]="inferred"
      stage_detail[$s]="inferred from existing .venv-humble (not persisted); use --force $s to re-run"
      changed=1
    fi
  done
  if ((changed)); then
    printf '[setup] existing .venv-humble detected; system/python inferred as done.\n' >&2
  fi
  return 0
}

render_state() {
  if ((dry_run)); then return 0; fi
  mkdir -p "$state_dir"
  local i s comma
  {
    printf '{\n  "schema": 1,\n'
    printf '  "generated_at": "%s",\n' "$(date -u +%FT%TZ)"
    printf '  "repository": "%s",\n' "$(json_escape "$repo_dir")"
    printf '  "container": "%s",\n' "$(json_escape "$container_name")"
    printf '  "runtime": "%s",\n' "$(json_escape "$detected_runtime")"
    printf '  "stages": {\n'
    for i in "${!stages[@]}"; do
      s=${stages[$i]}
      comma=","
      if (( i == ${#stages[@]} - 1 )); then comma=""; fi
      printf '    "%s": {"status": "%s", "ts": "%s", "detail": "%s"}%s\n' \
        "$s" "${stage_status[$s]}" "$(json_escape "${stage_ts[$s]:-}")" \
        "$(json_escape "${stage_detail[$s]:-}")" "$comma"
    done
    printf '  }\n}\n'
  } > "$state_file"
}

run_cmd() {
  printf '[setup] $ ' >&2
  printf '%q ' "$@" >&2
  printf '\n' >&2
  if ((dry_run)); then return 0; fi
  "$@"
}

run_preflight_checks() {
  if ((dry_run)); then
    detected_runtime="dry-run"
    printf '[setup] (dry-run) container probing skipped\n' >&2
    return 0
  fi
  local rt probe
  rt=$("$scripts_dir/exec.sh" --print-runtime) || return 1
  detected_runtime=$rt
  printf '[setup] runtime=%s container=%s\n' "$rt" "$container_name" >&2
  probe='set -eu; . /etc/os-release; if [ "$ID" != ubuntu ] || [ "$VERSION_ID" != 22.04 ]; then echo "OS mismatch: $ID $VERSION_ID (need Ubuntu 22.04)" >&2; exit 1; fi; if [ ! -f /opt/ros/humble/setup.bash ]; then echo "ROS underlay missing: /opt/ros/humble/setup.bash" >&2; exit 1; fi; /usr/bin/python3 -c "import sys; assert sys.version_info[:2] == (3, 10), sys.version"; if [ ! -d "$1" ]; then echo "repository not visible in container: $1" >&2; exit 1; fi; if [ ! -w "$1" ]; then echo "repository not writable by container user: $1" >&2; exit 1; fi; if [ -n "${CONDA_PREFIX:-}" ]; then echo "conda is active inside the container" >&2; exit 1; fi; echo "preflight ok: Ubuntu 22.04, Python 3.10, ROS underlay, repo writable"'
  "$scripts_dir/exec.sh" --no-venv -- bash -c "$probe" _ "$repo_dir"
}

stage_preflight() {
  printf '[setup] === stage preflight ===\n' >&2
  if ! run_preflight_checks; then
    printf '[setup] preflight failed. Check the container and ROS underlay.\n' >&2
    return 1
  fi
  return 0
}

stage_system() {
  printf '[setup] === stage system (apt, as root) ===\n' >&2
  run_cmd "$scripts_dir/exec.sh" --user root --no-venv -- \
    bash "$scripts_dir/install_system.sh"
}

stage_python() {
  printf '[setup] === stage python (venv, as current user) ===\n' >&2
  run_cmd "$scripts_dir/exec.sh" --no-venv -- \
    bash "$scripts_dir/setup_python.sh"
}

stage_build() {
  printf '[setup] === stage build ===\n' >&2
  run_cmd "$scripts_dir/container.sh" python -m pip install -e . --no-deps --no-build-isolation \
    --config-settings=build-dir=build/humble \
    --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON
  run_cmd "$scripts_dir/container.sh" cmake --preset humble -DCALIB_BUILD_PYTHON=ON
  run_cmd "$scripts_dir/container.sh" cmake --build --preset humble
  run_cmd "$scripts_dir/container.sh" ctest --preset humble
}

stage_verify() {
  printf '[setup] === stage verify ===\n' >&2
  run_cmd "$scripts_dir/container.sh" python -m casbot_calib.shm_fieldwork list
  run_cmd "$scripts_dir/container.sh" python -c \
    'import casbot_calib, pinocchio, coal, numpy, scipy; print("Humble imports ok")'
}

run_stage() {
  case "$1" in
    preflight) stage_preflight ;;
    system)    stage_system ;;
    python)    stage_python ;;
    build)     stage_build ;;
    verify)    stage_verify ;;
    *)         die "unknown stage: $1" ;;
  esac
}

diagnose_failure() {
  local s=$1
  printf '\n[setup] stage %s FAILED.\n' "$s" >&2
  printf '[setup]   retry this stage : bash scripts/humble/setup.sh --only %s\n' "$s" >&2
  printf '[setup]   redo from here   : bash scripts/humble/setup.sh --force %s\n' "$s" >&2
  printf '[setup]   inspect state    : bash scripts/humble/setup.sh --doctor\n' >&2
  printf '[setup]   state file       : %s\n' "$state_file" >&2
}

do_doctor() {
  local ok=0 venv=0 i s comma
  if ! run_preflight_checks; then ok=1; fi
  load_state
  infer_existing_environment
  if [[ -f "$repo_dir/.venv-humble/bin/activate" ]]; then venv=1; fi
  printf '{\n'
  printf '  "schema": 1,\n  "kind": "setup-doctor",\n'
  printf '  "generated_at": "%s",\n' "$(date -u +%FT%TZ)"
  printf '  "repository": "%s",\n' "$(json_escape "$repo_dir")"
  printf '  "container": "%s",\n' "$(json_escape "$container_name")"
  printf '  "runtime": "%s",\n' "$(json_escape "$detected_runtime")"
  if ((ok == 0)); then printf '  "preflight_ok": true,\n'; else printf '  "preflight_ok": false,\n'; fi
  if ((venv == 1)); then printf '  "venv_present": true,\n'; else printf '  "venv_present": false,\n'; fi
  printf '  "stages": {\n'
  for i in "${!stages[@]}"; do
    s=${stages[$i]}
    comma=","
    if (( i == ${#stages[@]} - 1 )); then comma=""; fi
    printf '    "%s": {"status": "%s", "ts": "%s"}%s\n' \
      "$s" "${stage_status[$s]}" "$(json_escape "${stage_ts[$s]:-}")" "$comma"
  done
  printf '  }\n}\n'
  return "$ok"
}

main() {
  while (($#)); do
    case "$1" in
      --stage|--only|--force)
        need_value "$@"
        valid_stage "$2" || die "unknown stage: $2 (expected ${stages[*]})"
        case "$1" in
          --stage) target_stage=$2 ;;
          --only)  only_stage=$2 ;;
          --force) force_stage[$2]=1 ;;
        esac
        shift 2 ;;
      --no-skip) no_skip=1; shift ;;
      --doctor)  doctor_mode=1; shift ;;
      --dry-run) dry_run=1; shift ;;
      -h|--help) usage; exit 0 ;;
      *) die "unknown option: $1 (try --help)" ;;
    esac
  done

  if ((doctor_mode)); then
    if do_doctor; then exit 0; else exit 1; fi
  fi

  load_state
  infer_existing_environment

  local last_idx i s only_idx
  last_idx=$(stage_index "$target_stage")
  if [[ -n "$only_stage" ]]; then
    only_idx=$(stage_index "$only_stage")
    if (( only_idx > last_idx )); then last_idx=$only_idx; fi
  fi
  for ((i = 0; i <= last_idx; i++)); do
    s=${stages[$i]}
    if [[ -n "$only_stage" && "$s" != "$only_stage" ]]; then continue; fi
    if [[ -n "$only_stage" ]] || [[ "${stage_status[$s]}" != ok ]]; then
      if run_stage "$s"; then
        stage_status[$s]=ok
        stage_ts[$s]=$(date -u +%FT%TZ)
        stage_detail[$s]="setup.sh phase 0"
        write_stamp "$s" "${stage_ts[$s]}" "${stage_detail[$s]}"
      else
        stage_status[$s]=failed
        stage_ts[$s]=$(date -u +%FT%TZ)
        stage_detail[$s]="stage failed; see the log above"
        render_state
        diagnose_failure "$s"
        exit 1
      fi
    else
      printf '[setup] stage %s: skipped (already done)\n' "$s" >&2
    fi
  done

  render_state
  printf '\n' >&2
  if ((dry_run)); then
    printf '[setup] dry-run stage summary (no stage was actually executed): ' >&2
  else
    printf '[setup] summary: ' >&2
  fi
  for i in "${!stages[@]}"; do
    s=${stages[$i]}
    if (( i > last_idx )); then
      printf '%s=not-in-scope ' "$s" >&2
    else
      printf '%s=%s ' "$s" "${stage_status[$s]}" >&2
    fi
  done
  printf '\n' >&2
  if ((dry_run)); then
    printf '[setup] dry-run complete: nothing was executed or written.\n' >&2
  else
    printf '[setup] state: %s\n' "$state_file" >&2
    printf '[setup] next: bash scripts/humble/container.sh python -m casbot_calib.shm_fieldwork list\n' >&2
  fi
}

main "$@"
