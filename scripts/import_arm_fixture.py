"""Reproduce packaged right-arm geometry and reference FK (no mesh required)."""
import argparse
from pathlib import Path
from casbot_calib.arm_import import export_right_arm

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf",type=Path)
    parser.add_argument("--output",type=Path,default=Path("python/casbot_calib/assets/casbot_right_arm.json"))
    parser.add_argument("--reference-output",type=Path,default=Path("python/casbot_calib/assets/casbot_right_arm_fk_reference.json"))
    args=parser.parse_args()
    export_right_arm(args.urdf,args.output,args.reference_output)
    print(f"Exported synthetic seven-joint fixture -> {args.output}")
