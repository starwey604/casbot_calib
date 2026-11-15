# Sourced after selecting $python. Native development keeps its source-first path.
if [[ -f "$(dirname -- "$python")/../casbot-prebuilt" ]]; then
  unset PYTHONPATH LD_LIBRARY_PATH
  export CASBOT_PROJECT_ROOT="$repo"
else
  export PYTHONPATH="$repo/python${PYTHONPATH:+:$PYTHONPATH}"
fi
