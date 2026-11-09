#!/usr/bin/env python3
"""Verify imported N1 resources with existing native, Pinocchio and Coal loaders."""
import argparse
import json
from pathlib import Path
import pinocchio as pin
from casbot_calib import _core
from casbot_calib.data import write_json
from casbot_calib.model_sources import source_catalog,dvt2_source,inspect_source


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    catalog=source_catalog();results={}
    for variant in catalog['variants']:
        source=dvt2_source(variant);report=inspect_source(variant)
        native=_core.ArmDynamics.from_urdf(str(source.urdf));model=pin.buildModelFromUrdf(str(source.urdf))
        collision=pin.buildGeomFromUrdf(model,str(source.urdf),pin.GeometryType.COLLISION,package_dirs=[str(source.asset_root)])
        visual=pin.buildGeomFromUrdf(model,str(source.urdf),pin.GeometryType.VISUAL,package_dirs=[str(source.asset_root)])
        report.update(native_joints=len(native.model.joints),pinocchio_nq=model.nq,
            collision_geometries=len(collision.geometryObjects),visual_geometries=len(visual.geometryObjects),
            geometry_backend=_core.CollisionChecker.backend_version())
        report['passed']=report['passed'] and model.nq==len(native.model.joints)==len(report['active_joint_names'])
        report['passed']=report['passed'] and report['collision_geometries']>0 and report['visual_geometries']>0
        results[variant]=report;print(variant,report['native_joints'],report['collision_geometries'],report['passed'],flush=True)
    source=dvt2_source('a2_ff');assets=[p for p in source.asset_root.rglob('*') if p.is_file()]
    inventory=dict(files=len(assets),bytes=sum(p.stat().st_size for p in assets))
    from casbot_calib.dual_model import nominal_dual_arm
    default,selected=nominal_dual_arm()
    checks=dict(all_five_variants=all(r['passed'] for r in results.values()),
        complete_asset_inventory=all(inventory[k]==catalog['asset_inventory'][k] for k in inventory),
        dvt2_is_default=len(default.model.joints)==14 and 'bare_flange_1105_03_gauge_v2' in selected['source'])
    write_json(root/'verification.json',dict(passed=all(checks.values()),checks=checks,variants=results,asset_inventory=inventory,
        source_only=True,default_model_changed=True,hardware_tested=False,
        scope='Source and geometry parsing only; no reduction, scene collision qualification or robot execution'))
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
