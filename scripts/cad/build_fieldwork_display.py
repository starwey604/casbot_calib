"""Blender-only offline builder: display LOD, no dynamics or collision replacement.

blender -b -t 2 --factory-startup --python scripts/cad/build_fieldwork_display.py -- \
  --output python/casbot_calib/assets/fieldwork_display/dvt2_v1
Input is the asset mesh inventory of the frozen model context's scene.xml.
"""
import argparse
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import bpy
import bmesh
from mathutils.bvhtree import BVHTree


def triangles(mesh):
    mesh.calc_loop_triangles()
    return [tuple(t.vertices) for t in mesh.loop_triangles]


def surface_samples(vertices,faces):
    yield from vertices
    for a,b,c in faces:yield (vertices[a]+vertices[b]+vertices[c])/3


def deviation(vertices,faces,other_vertices,other_faces):
    tree=BVHTree.FromPolygons(other_vertices,other_faces,all_triangles=True)
    return max(tree.find_nearest(p)[3] for p in surface_samples(vertices,faces))


def write_stl(path,vertices,faces):
    with path.open('xb') as stream:
        stream.write(b'casbot fieldwork display only; same CAD coordinates'.ljust(80,b'\0'))
        stream.write(struct.pack('<I',len(faces)))
        for a,b,c in faces:
            p,q,r=vertices[a],vertices[b],vertices[c];normal=(q-p).cross(r-p).normalized()
            stream.write(struct.pack('<12fH',*normal,*p,*q,*r,0))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,help='optional archived mesh inventory');parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(sys.argv[sys.argv.index('--')+1:]);args.output.mkdir(parents=True,exist_ok=False)
    bundle=Path('python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2')
    if args.input:rows=json.loads(args.input.read_text())
    else:
        tree=ET.parse(bundle/'scene_left.xml')
        used={g.get('mesh') for g in tree.iter('geom') if not g.get('name','').startswith(('fixture_','datum_'))}
        rows=[dict(name=n.get('name'),file=str((bundle/n.get('file')).resolve()))
            for n in tree.findall('./asset/mesh') if n.get('name') in used]
    entries=[];reuse={} 
    for row in rows:
        path=Path(row['file'])
        if str(path) in reuse:
            entries.append(dict(reuse[str(path)],name=row['name']));continue
        bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
        bpy.ops.wm.obj_import(filepath=str(path),forward_axis='Y',up_axis='Z')
        objects=list(bpy.context.selected_objects)
        if len(objects)!=1:raise ValueError('Expected one CAD mesh per resource: '+str(path))
        obj=objects[0];bpy.context.view_layer.objects.active=obj
        # Preserve coordinates exactly as stored in OBJ, not Blender scene axes.
        vertices=[v.co.copy() for v in obj.data.vertices];faces=triangles(obj.data)
        # CAD OBJ seams often duplicate coincident vertices. Weld at 1 micrometre
        # before decimation, retaining the unmodified source surface for checks.
        bm=bmesh.new();bm.from_mesh(obj.data)
        bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-6)
        bm.to_mesh(obj.data);bm.free();obj.data.update()
        for ratio in (.04,.08,.16,.32,.64,1.):
            modifier=obj.modifiers.new('display_only','DECIMATE');modifier.ratio=ratio;modifier.use_collapse_triangulate=True
            evaluated=obj.evaluated_get(bpy.context.evaluated_depsgraph_get());mesh=evaluated.to_mesh()
            reduced_vertices=[v.co.copy() for v in mesh.vertices];reduced_faces=triangles(mesh)
            forward=deviation(vertices,faces,reduced_vertices,reduced_faces)
            reverse=deviation(reduced_vertices,reduced_faces,vertices,faces)
            evaluated.to_mesh_clear();obj.modifiers.remove(modifier)
            bounds_drift=max(abs(op(v[i] for v in vertices)-op(v[i] for v in reduced_vertices))
                for i in range(3) for op in (min,max))
            if max(forward,reverse)<=.002 and bounds_drift<=.0005:break
        if max(forward,reverse)>.002 or bounds_drift>.0005:raise ValueError('Display geometry tolerance failed: '+str(path))
        filename=row['name']+'.stl';write_stl(args.output/filename,reduced_vertices,reduced_faces)
        entry=dict(name=row['name'],source=path.name,file=filename,source_triangles=len(faces),triangles=len(reduced_faces),
            ratio=ratio,max_sampled_deviation_m=max(forward,reverse),
            bounds_source=[[min(v[i] for v in vertices) for i in range(3)],[max(v[i] for v in vertices) for i in range(3)]],
            bounds_display=[[min(v[i] for v in reduced_vertices) for i in range(3)],[max(v[i] for v in reduced_vertices) for i in range(3)]])
        entries.append(entry);reuse[str(path)]=entry
        print(json.dumps(entry),flush=True)
    config=json.loads((bundle/'model.json').read_text())
    result=dict(schema_version='casbot_fieldwork_display_v1',model_id=config['model_id'],
        bundle_sha256=config['bundle_sha256'],urdf_sha256=config['urdf_sha256'],meshes=entries,
        scope='display only; original CAD coordinates and scene transforms; not collision or dynamics geometry',
        method='Blender 1um seam weld + quadric edge-collapse; first accepted ratio; bidirectional all vertices and triangle centroids <=2mm; bounding extrema drift <=0.5mm',
        blender_version=bpy.app.version_string)
    (args.output/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
