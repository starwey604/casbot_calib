#!/usr/bin/env python3
"""Expose only same-prefix robotics math bindings inside the active project venv."""
import argparse
from pathlib import Path
import sys
import sysconfig


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dependency-prefix',type=Path,default=Path('/opt/ros/humble'))
    args=parser.parse_args()
    if sys.prefix==sys.base_prefix:raise SystemExit('Run with the project venv Python, not system Python')
    source=args.dependency_prefix.resolve()/f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages'
    target=Path(sysconfig.get_path('purelib'))
    pairs=[(source/name,target/name) for name in ('pinocchio','eigenpy','coal')]
    for src,dst in pairs:
        if not src.is_dir():raise SystemExit('Missing same-prefix math binding: '+str(src))
        if (dst.exists() or dst.is_symlink()) and dst.resolve()!=src.resolve():
            raise SystemExit('Existing venv module differs; not overwritten: '+str(dst))
    for src,dst in pairs:
        if not dst.is_symlink():dst.symlink_to(src,target_is_directory=True)
    import numpy
    import pinocchio as pin
    print(f'URDF export/reload math: Pinocchio {pin.__version__}, NumPy {numpy.__version__}, prefix {source}')


if __name__=='__main__':main()
