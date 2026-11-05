#!/usr/bin/env python3
"""Partition two concave tool couplings with FreeCAD booleans for convex covers.

The solid is split into disjoint boxes. Each cell is conservatively convexified
by the existing builder; the volume sum checks that no source solid was dropped.
"""
import argparse,json,sys,zipfile
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'python'))
from casbot_calib.mesh_io import write_stl
sys.path.append('/usr/lib/freecad/lib')
import FreeCAD as A,Part,Import


def run(reference,assets,checkpoint=None):
    reference=Path(reference);assets=Path(assets)
    reg=json.loads((reference/'registration.json').read_text());selection={p['name']:p for p in json.loads((reference/'selection.json').read_text())}
    manifest=json.loads((assets/'manifest.json').read_text());R=np.array(reg['registration']['rotation']);t=np.array(reg['registration']['translation_m'])
    archive=zipfile.ZipFile(checkpoint) if checkpoint else None;doc=None
    if archive is None:
        doc=A.newDocument('TricalCover');Import.insert(str(reference/'1105_03.stp'),doc.Name);doc.recompute()
    try:
        for side,name in [('left','Part__Feature5150'),('right','Part__Feature5130')]:
            row=selection[name];tool=next(x for x in reg['tools'] if x['arm']==side)
            if archive is None:sh=doc.getObject(name).Shape.copy()
            else:sh=Part.Shape();sh.importBrepFromString(archive.read(row['shape']).decode(),False)
            sh.Placement=A.Placement(A.Matrix(*row['global']))
            W=tool['world_tcp'];U=np.array(W['rotation']);H=np.eye(4);H[:3,:3]=U.T@R;H[:3,3]=1000*U.T@(t-np.array(W['translation_m']))
            sh.Placement=A.Placement(A.Matrix(*H.ravel()))*sh.Placement
            box=sh.BoundBox;lo=np.array([box.XMin,box.YMin,box.ZMin]);hi=np.array([box.XMax,box.YMax,box.ZMax]);mid=(lo+hi)/2
            mid[1:]=np.clip([0.,0.],lo[1:]+1e-5,hi[1:]-1e-5)
            cells=[];volumes=[];solids=[]
            for i in range(8):
                low=np.array([mid[j] if i>>j&1 else lo[j]-.001 for j in range(3)])
                high=np.array([hi[j]+.001 if i>>j&1 else mid[j] for j in range(3)])
                cut=sh.common(Part.makeBox(*(high-low),A.Vector(*low)))
                if cut.Volume<1e-6:continue
                v,f=cut.tessellate(.2);path=f'meshes/{side}_coupling_cell_{i}.stl'
                write_stl(assets/path,np.array([[x.x,x.y,x.z] for x in v])/1000,np.array(f));cells.append(path);volumes.append(cut.Volume);solids.append(cut)
            error=abs(sum(volumes)-sh.Volume)/sh.Volume
            union=solids[0].multiFuse(solids[1:]);missing=sh.cut(union).Volume;excess=union.cut(sh).Volume
            print('coverage missing/excess mm3',missing,excess,flush=True)
            print('partition volumes',sh.Volume,volumes,'bounds',lo,hi,'mid',mid,flush=True)
            if missing>1e-3:raise ValueError('Partition lost CAD solid: '+name)
            part=next(p for p in manifest['parts'] if p['name']==side+':'+name)
            part.update(collision_meshes=cells,collision_cover=dict(method='FreeCAD BREP box partition, per-cell outer hull',
                source_volume_mm3=sh.Volume,partition_volume_mm3=sum(volumes),relative_volume_difference=error,missing_solid_mm3=missing,extra_solid_mm3=excess,
                cells=len(cells),cell_deflection_m=.0002,padding_m=.0002))
            print(side,'coupling cells',len(cells),'volume difference',error,flush=True)
        (assets/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    finally:
        if archive:archive.close()
        if doc:A.closeDocument(doc.Name)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference',required=True);p.add_argument('--assets',required=True);p.add_argument('--native-checkpoint')
    a=p.parse_args();run(a.reference,a.assets,a.native_checkpoint)
