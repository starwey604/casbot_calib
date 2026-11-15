#!/usr/bin/env bash
# Run Orin-side native CTest, private SHM physics acceptance, and pytest without polluting .venv-shm.
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
py="$repo/.venv-shm/bin/python"
[[ -x $py ]] || { echo '未找到 .venv-shm；请先运行 bash calib/setup.sh' >&2; exit 1; }

ctest --test-dir "$repo/build/shm-python" --output-on-failure
"$py" "$repo/scripts/shm/verify_physics.py" --bin-dir "$repo/install/shm-core/bin"

cd "$repo"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$repo/python${PYTHONPATH:+:$PYTHONPATH}"
exec "$py" - "$@" <<'PY'
import importlib.util
from pathlib import Path
import sys

if importlib.util.find_spec('pytest') is None:
    sys_dist = '/usr/lib/python3/dist-packages'
    if Path(sys_dist).is_dir() and sys_dist not in sys.path:
        # Append after venv site-packages so system scipy never shadows .venv-shm.
        sys.path.append(sys_dist)

import pytest

models = Path('python/casbot_calib/assets/models')
def _is_lfs(rel: str) -> bool:
    p = models / rel
    if not p.is_file():
        return True
    with p.open('rb') as f:
        return f.read(48).startswith(b'version https://git-lfs.github.com/spec/v1')

args = ['-q', '--tb=short']
lfs_gates = [
    ('casbot_hl12p1_dvt2/source/assets/visual/CASBOTHL12/base_link.obj', ['tests/test_model_sources.py', 'tests/test_fieldwork_display.py']),
    ('trical_1105_03_gauge_cover_v2/collision.npz', ['tests/test_collision_local_bounds.py']),
    ('casbot_hl12p1_dvt2/derived/a2ff_stock_hands_candidate_v1/collision.npz', ['tests/test_dvt2_bundle.py']),
]
for asset, suites in lfs_gates:
    if _is_lfs(asset):
        for suite in suites:
            args.append(f'--ignore={suite}')

args.extend(sys.argv[1:])
raise SystemExit(pytest.main(args))
PY
