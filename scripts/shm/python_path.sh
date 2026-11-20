# Sourced with the project virtualenv directory by SHM entry points.
if [[ -n ${CASBOT_SHM_PYTHON:-} ]]; then
  python="$CASBOT_SHM_PYTHON"
elif [[ -x "$1/bin/python" ]]; then
  python="$1/bin/python"
elif [[ -n ${VIRTUAL_ENV:-} && -x "$VIRTUAL_ENV/bin/python" ]]; then
  python="$VIRTUAL_ENV/bin/python"
else
  python=python3
fi

if [[ -f "$(dirname -- "$python")/../casbot-prebuilt" ]]; then
  unset PYTHONPATH LD_LIBRARY_PATH
  export CASBOT_PROJECT_ROOT="$repo"
else
  export PYTHONPATH="$repo/python${PYTHONPATH:+:$PYTHONPATH}"
fi
