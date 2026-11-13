#!/usr/bin/env python3
"""Export the existing bare-flange collision profile exactly for native tests."""
import argparse
from pathlib import Path
import numpy as np
from casbot_calib.model_context import load_context


def export(path):
    context=load_context()
    profile=context.collision_profile()
    names=list(context.joint_names)
    # C++ uses the identical reduced URDF, not a separately reconstructed robot.
    from casbot_calib import _core
    engine=_core.ArmDynamics.from_urdf(str(context.urdf))
    probe=np.zeros((2,14));probe[1,0]=.1
    rebuilt=context.collision_profile(engine=engine)
    assert np.allclose(profile.check(probe)['distances'],rebuilt.check(probe)['distances'],rtol=0,atol=1e-12)
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as out:
        out.write('CASBOT_SHM_COLLISION_V1\n14\n'+' '.join(names)+'\n')
        out.write('URDF_HEX '+context.urdf.read_bytes().hex()+'\n')
        out.write(str(len(profile.geometry))+'\n')
        for g in profile.geometry:
            vertices=np.asarray(g.vertices);triangles=np.asarray(g.triangles)
            out.write(f'{g.name} {g.parent} {int(g.attached_to_base)} {len(vertices)} {len(triangles)}\n')
            pose=np.r_[np.asarray(g.placement.rotation).ravel(),np.asarray(g.placement.translation)]
            out.write(' '.join(format(float(x),'.17g') for x in pose)+'\n')
            np.savetxt(out,vertices,fmt='%.17g')
            np.savetxt(out,triangles,fmt='%d')
        thresholds=getattr(profile,'pair_clearance_m',np.full(len(profile.pairs),.01))
        out.write(str(len(profile.pairs))+'\n')
        for (a,b),clearance in zip(profile.pairs,thresholds):
            out.write(f'{a} {b} {float(clearance):.17g}\n')
    print(f'{path}: {len(profile.geometry)} geometries, {len(profile.pairs)} pairs; no thresholds changed')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    export(parser.parse_args().output)
