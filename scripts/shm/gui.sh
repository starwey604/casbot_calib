#!/usr/bin/env bash
# Authenticate on the host terminal, then share only owned SSH sockets with the graphics container.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo"
host='';values=("$@")
for ((i=0;i<${#values[@]};++i)); do
  case ${values[i]} in
    --host) ((i+1<${#values[@]})) && host=${values[i+1]} ;;
    --host=*) host=${values[i]#--host=} ;;
  esac
done
auth_dir='';sockets=()
cleanup() {
  for socket in "${sockets[@]}"; do ssh -S "$socket" -O exit "$host" >/dev/null 2>&1 || true; done
  [[ -z $auth_dir ]] || rmdir "$auth_dir" 2>/dev/null || true
}
trap cleanup EXIT
if [[ -n $host && ! -v CASBOT_GUI_SSH_CONTROL_SOCKET ]]; then
  [[ $host != -* ]] || { echo '无效SSH目标' >&2; exit 2; }
  parent="$HOME/.cache/casbot-calib/shm-ssh"
  mkdir -p "$parent";auth_dir=$(mktemp -d "$parent/session.XXXXXXXX");chmod 700 "$auth_dir"
  # Each socket has its own TCP transport; file traffic cannot occupy the control stream.
  for channel in control display files; do
    socket="$auth_dir/$channel.sock";sockets+=("$socket")
    echo "SSH认证：$channel（仅运行true，尚未连接SHM）" >&2
    ssh -M -o ControlPersist=yes -o "ControlPath=$socket" -o Compression=no -o ConnectTimeout=7 "$host" true
  done
  export CASBOT_GUI_SSH_CONTROL_SOCKET="${sockets[0]}" CASBOT_GUI_SSH_DISPLAY_SOCKET="${sockets[1]}" CASBOT_GUI_SSH_FILES_SOCKET="${sockets[2]}"
fi
if [[ -n ${CASBOT_GUI_PYTHON:-} ]]; then
  export PYTHONPATH="$repo/python${PYTHONPATH:+:$PYTHONPATH}"
  "$CASBOT_GUI_PYTHON" -m casbot_calib.shm_gui "$@"
else
  args=(env "PYTHONPATH=$repo/python")
  for name in DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR XAUTHORITY SSH_AUTH_SOCK CASBOT_CJK_FONT XDG_STATE_HOME LIBGL_ALWAYS_SOFTWARE LP_NUM_THREADS CASBOT_GUI_SSH_CONTROL_SOCKET CASBOT_GUI_SSH_DISPLAY_SOCKET CASBOT_GUI_SSH_FILES_SOCKET; do
    if [[ -v "$name" ]]; then args+=("$name=${!name}"); fi
  done
  bash scripts/humble/container.sh "${args[@]}" .venv-humble/bin/python -m casbot_calib.shm_gui "$@"
fi
