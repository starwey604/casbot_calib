#!/usr/bin/env python3
"""Keep absolute symlink targets within the copied target filesystem."""
import os
from pathlib import Path
import sys


def normalize(root):
    root = Path(root).resolve()
    changed = 0
    for directory, dirs, files in os.walk(root, followlinks=False):
        for name in dirs + files:
            path = Path(directory) / name
            if path.is_symlink():
                target = os.readlink(path)
                if target.startswith('/'):
                    relative = os.path.relpath(root / target.lstrip('/'), path.parent)
                    path.unlink()
                    path.symlink_to(relative)
                    changed += 1
    return changed


if __name__ == '__main__':
    print(f'Normalized {normalize(sys.argv[1])} sysroot symlinks')
