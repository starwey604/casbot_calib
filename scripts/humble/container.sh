#!/usr/bin/env bash
# Host entry point for the existing Docker/podman-backed distrobox. Its shared
# home must contain this repository at the same absolute path. Host conda is
# not inherited. Thin wrapper over exec.sh; the venv-enabled path goes through
# run.sh. Existing callers (container.sh COMMAND ARGS...) keep working.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec "$here/exec.sh" --venv -- "$@"
