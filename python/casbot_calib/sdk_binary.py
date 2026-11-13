"""Installed SDK executable resolver for the remaining ROS consumers."""
import json
import os
from pathlib import Path
from .artifact_io import sha256


def executor_binary():
    prefix=Path(os.environ.get('CASBOT_VENDOR_INSTALL','install/vendor')).resolve()
    identity=json.loads((prefix/'vendor-source.json').read_text())
    for name,digest in identity['interfaces'].items():
        if sha256(prefix/'crb_ros_msg/share/crb_ros_msg'/name)!=digest: raise ValueError('Installed SDK interface identity changed')
    return prefix/'casbot_calib_vendor/lib/casbot_calib_vendor/position_executor',identity


