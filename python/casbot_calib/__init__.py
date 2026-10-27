"""C++ virtual devices and Pinocchio arm models; Python calibration experiments."""
import os as _os
from pathlib import Path as _Path
import sys as _sys

__version__ = "0.1.0"

# Python 3.8+ Windows extension loading does not search PATH for dependency DLLs.
# Keep the handle alive without importing Pinocchio's Python bindings.
_native_dll_handles = []
if _sys.platform == "win32":
    _native_directory = _Path(_sys.prefix) / "Library" / "bin"
    if _native_directory.is_dir():
        _native_dll_handles.append(_os.add_dll_directory(str(_native_directory)))
