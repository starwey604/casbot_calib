"""Check installed bindings and materialized runtime assets after an update."""
from pathlib import Path
import sys
import pinocchio
import coal
import casbot_calib
from casbot_calib import _core
from casbot_calib.shm_data import URDF
from runtime_assets import FILES

root = Path(__file__).resolve().parents[2]
for module in (casbot_calib, _core):
    assert Path(module.__file__).is_relative_to(sys.prefix), module.__file__
for relative in FILES:
    with (root / relative).open('rb') as stream:
        assert not stream.read(80).startswith(b'version https://git-lfs.github.com/spec/v1'), relative
assert URDF.is_file(), URDF
print(f'Installed package: {casbot_calib.__file__}\nInstalled core: {_core.__file__}')
