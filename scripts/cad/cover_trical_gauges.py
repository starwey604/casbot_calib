#!/usr/bin/env python3
"""Partition six gauge bodies along their actual stem axes, preserving CAD solid."""
import argparse,json,sys,zipfile
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'python'));sys.path.append('/usr/lib/freecad/lib')
import FreeCAD as A,Part,Import
from import_trical_1105 import working_gauge_shape
from casbot_calib.mesh_io import write_stl


def run(reference,base,output,checkpoint=None):
    reference=Path(reference);base=Path(base);out=Path(output);out.mkdir(parents=True,exist_ok=False);(out/'meshes').mkdir()
    read=lambda p:json.loads(Path(p).read_text());reg=read(reference/'registration.json');selection={r['name']:r for r in read(reference/'selection.json')}
    manifest=read(base/'manifest.json')
    # One copy of original display/CAD assets; this version only adds new covers.
    for row in manifest['parts']:
        if row.get('mesh'):row['mesh']='../'+base.name+'/'+row['mesh']
        if 'collision_meshes' in row:row['collision_meshes']=['../'+base.name+'/'+p for p in row['collision_meshes']]
    for name in ['table_holes.json','pillar_holes.json','seeds.json','orientation_seeds.json']:(out/name).write_text((base/name).read_text())
    archive=zipfile.ZipFile(checkpoint) if checkpoint else None;doc=None;evidence=[]
    if archive is None:doc=A.newDocument('TricalGaugeCover');Import.insert(str(reference/'1105_03.stp'),doc.Name);doc.recompute()
    try:
        for tool in reg['tools']:
            for name in tool['body_parts']:
                row=selection[name]
                if archive is None:sh=doc.getObject(name).Shape.copy()
                else:sh=Part.Shape();sh.importBrepFromString(archive.read(row['shape']).decode(),False)
                sh.Placement=A.Placement(A.Matrix(*row['global']));working,placement=working_gauge_shape(sh,tool)
                working.Placement=placement.inverse()*working.Placement
                levels=sorted(set(round(v.Point.z,6) for v in working.Vertexes));box=working.BoundBox
                cells=[];solids=[];source_volume=working.Volume
                for i,(low,high) in enumerate(zip(levels[:-1],levels[1:])):
                    if high-low<1e-5:continue
                    cell=working.common(Part.makeBox(box.XLength+2,box.YLength+2,high-low,A.Vector(box.XMin-1,box.YMin-1,low)))
                    if cell.Volume<1e-6:continue
                    solids.append(cell)
                    v,f=cell.tessellate(.1);points=np.array([[p.x,p.y,p.z] for p in v]);H=np.array(placement.toMatrix().A).reshape(4,4)
                    cad=points@H[:3,:3].T+H[:3,3];R=np.array(reg['registration']['rotation']);t=np.array(reg['registration']['translation_m']);W=tool['world_tcp']
                    tcp=(cad/1000@R.T+t-np.array(W['translation_m']))@np.array(W['rotation'])
                    path=f'meshes/{name}_slice_{i}.stl';write_stl(out/path,tcp,np.array(f));cells.append(path)
                union=solids[0].multiFuse(solids[1:]);missing=working.cut(union).Volume;excess=union.cut(working).Volume
                if missing>1e-3 or excess>1e-3:raise ValueError('Gauge cover changed source solid: '+name)
                proof=dict(method='FreeCAD working-contact BREP partition at stem-axis face boundaries; per-cell outer hull',source_volume_mm3=source_volume,
                    partition_volume_mm3=sum(c.Volume for c in solids),missing_solid_mm3=missing,extra_solid_mm3=excess,cells=len(cells),cell_deflection_m=.0001,padding_m=.0002,axis_boundaries_mm=levels)
                part=next(r for r in manifest['parts'] if r['name']==tool['arm']+':'+name);part.update(collision_meshes=cells,collision_cover=proof)
                evidence.append(dict(name=part['name'],**proof));print(part['name'],len(cells),'missing/excess',missing,excess,flush=True)
        manifest['gauge_cover_version']='stem_slices_v2';(out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
        (out/'gauge_cover_evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
    finally:
        if archive:archive.close()
        if doc:A.closeDocument(doc.Name)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference',required=True);p.add_argument('--base',required=True);p.add_argument('--output',required=True);p.add_argument('--native-checkpoint');a=p.parse_args();run(a.reference,a.base,a.output,a.native_checkpoint)
