"""Build conservative joint-local convex envelopes from the nominal CAD collision meshes.

Uses Pinocchio's URDF geometry transforms and SciPy/Qhull, not a hand-written hull.
Runtime consumes only the bundled arrays; task meshes and Python Pinocchio/Coal
are build-time inputs. Same-joint fixed fingers are enveloped together.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
import pinocchio as pin


def export(urdf,mesh_directory,output):
    model=pin.buildModelFromUrdf(str(urdf))
    if model.nv!=14 or list(model.parents)[1:]!=[0,1,2,3,4,5,6,0,8,9,10,11,12,13]:
        raise ValueError('Expected the fixed-waist nominal 7+7 tree')
    geometry=pin.buildGeomFromUrdf(model,str(urdf),pin.GeometryType.COLLISION,package_dirs=[str(mesh_directory)])
    groups={};mesh_sources={}
    for item in geometry.geometryObjects:
        path=Path(item.meshPath)
        mesh_sources[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
        vertices=np.asarray(item.geometry.vertices(),dtype=float)
        placed=vertices@item.placement.rotation.T+item.placement.translation
        # Do not bridge the separate fixed pelvis/torso shells with one hull.
        # That can fill large external concavities around the shoulder workspace.
        key=(0,item.name) if item.parentJoint==0 else (int(item.parentJoint),'')
        groups.setdefault(key,[]).append((path.name,placed))
    arrays={};records=[];robot_names=[]
    for index,((parent,tag),meshes) in enumerate(sorted(groups.items())):
        if not meshes:raise ValueError('Missing collision geometry for joint '+str(parent))
        vertices=np.unique(np.vstack([points for _,points in meshes]),axis=0)
        hull=ConvexHull(vertices)
        lookup=np.full(len(vertices),-1,dtype=np.int64);lookup[hull.vertices]=np.arange(len(hull.vertices))
        points=vertices[hull.vertices];faces=lookup[hull.simplices].astype(np.int32)
        normals=np.cross(points[faces[:,1]]-points[faces[:,0]],points[faces[:,2]]-points[faces[:,0]])
        inward=np.sum(normals*hull.equations[:,:3],axis=1)<0
        faces[inward]=faces[inward][:,[0,2,1]]
        violation=-np.inf
        # Bounded scratch memory, even for the static torso's original triangle soup.
        for offset in range(0,len(vertices),256):
            violation=max(violation,float(np.max(vertices[offset:offset+256]@hull.equations[:,:3].T+hull.equations[:,3])))
        if violation>1e-9:raise ValueError('Envelope does not contain original vertices')
        key='g'+str(index);arrays[key+'_vertices']=points;arrays[key+'_triangles']=faces
        name='fixed_'+tag if parent==0 else str(model.names[parent])
        robot_names.append(name)
        records.append(dict(name=name,parent_joint_index=parent-1,array_key=key,kind='robot',
            frame='parent joint local; Pinocchio fixed-link geometry transforms already applied',
            source_meshes=[name for name,_ in meshes],source_vertices=len(vertices),hull_vertices=len(points),triangles=len(faces),
            maximum_vertex_plane_violation_m=violation,maximum_local_radius_m=float(np.linalg.norm(points,axis=1).max())))
    exclusions=[]
    def attached(joint):return [item['name'] for item in records if item['parent_joint_index']==joint-1]
    for j in range(1,model.njoints):
        for first in attached(j):
            for second in attached(model.parents[j]):
                exclusions.append(dict(first=first,second=second,
                    reason='URDF adjacent mechanical assembly; enclosing shells meet at the joint interface; travel limits must be checked separately'))
    for a,b in [(5,7),(12,14)]:
        exclusions.append(dict(first=attached(a)[0],second=attached(b)[0],reason='Known nominal wrist-yaw/wrist-roll housing overlap; source benchmark MJCF explicit exclusion'))
    output.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output.with_suffix('.npz'),**arrays)
    metadata=dict(schema_version=1,model_id='casbot_20251015_nominal',
        source_urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),mesh_sha256=mesh_sources,
        array_sha256=hashlib.sha256(output.with_suffix('.npz').read_bytes()).hexdigest(),
        joint_names=list(model.names)[1:],builder='Pinocchio URDF geometry + SciPy spatial.ConvexHull (Qhull)',
        envelopes='One convex outer envelope per movable joint; fixed root shells kept separate; fixed fingers merged; geometric over-approximation',
        geometries=records,excluded_pairs=exclusions,
        exclusions_policy='Only same-body fixed aggregation, explicitly listed adjacent assemblies and two audited wrist overlaps; no SRDF Never blanket suppression',
        coverage='Geometry present in the nominal upper-body URDF only; fixture/environment proxies are configured independently',
        source_geometry_count=len(geometry.geometryObjects))
    output.with_suffix('.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(f'Exported {len(records)} convex envelopes from {len(geometry.geometryObjects)} collision meshes; {output.with_suffix(".npz").stat().st_size} bytes')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,default=Path('python/casbot_calib/assets/casbot_dual_arm_benchmark_nominal.urdf'))
    parser.add_argument('--mesh-directory',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=Path('python/casbot_calib/assets/nominal_collision'))
    args=parser.parse_args();export(args.urdf,args.mesh_directory,args.output)
