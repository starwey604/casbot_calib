"""Synthetic, explicitly auditable tool/artifact collision geometry for K2."""
from copy import deepcopy
from hashlib import sha256
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from . import _core
from .collision import CollisionProfile, nominal_profile, box_geometry
from .kinematics import pose, ball_world, digest


def sphere_envelope(name, center, radius):
    # Circumscribed convex polytope: all hull support planes lie outside the sphere.
    k=np.arange(96);z=1-2*(k+.5)/len(k);angle=k*np.pi*(3-np.sqrt(5))
    vertices=np.column_stack([np.sqrt(1-z*z)*np.cos(angle),np.sqrt(1-z*z)*np.sin(angle),z])
    hull=ConvexHull(vertices);vertices*=radius/(-hull.equations[:,3]).min()
    hull=ConvexHull(vertices);faces=hull.simplices.copy()
    normals=np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]])
    inward=np.sum(normals*hull.equations[:,:3],axis=1)<0;faces[inward]=faces[inward][:,[0,2,1]]
    result=_core.CollisionGeometry();result.name=name;result.parent=-1;result.attached_to_base=False
    p=_core.ArmPose();p.translation=center;result.placement=p;result.vertices=vertices;result.triangles=faces.astype(np.int32)
    return result


def fixture_profile(engine, config, target=None, contact=False):
    """Only three exact tip/selected-ball pairs may use the bounded-stroke margin.

    All balls, supports, housings and stems remain present, including the other arm.
    Nominal body cache may be reused only after its geometry validation succeeds.
    """
    scene=deepcopy(config['workflow']['collision_scene']);base=nominal_profile(engine,scene)
    geometry=list(base.geometry);excluded=deepcopy(base.metadata['excluded_pairs']);names=list(base.names)
    exclusions=[(names.index(x['first']),names.index(x['second'])) for x in excluded]
    radius=config['fixture']['ball_radius_m'];frames={f.name:f for f in engine.model.frames}
    for tool in config['tools']:
        frame=frames[tool['name']];prefix=tool['name']
        pieces=[('housing',[0,0,-.045],[.03,.03,.008]),('connector',[0,0,-.085],[.006,.006,.055])]
        for axis in range(3):
            center=np.zeros(3);center[axis]=-radius-.0005;half=np.full(3,config['sensor']['face_radius']);half[axis]=.0005
            pieces.append((f'tip_{axis}',center,half))
        for suffix,center,half in pieces:
            g=box_geometry(prefix+'_'+suffix,[0,0,0],half)
            g.parent=frame.parent;placement=_core.ArmPose()
            placement.rotation=frame.placement.rotation
            placement.translation=frame.placement.translation+frame.placement.rotation@np.asarray(center)
            g.placement=placement;geometry.append(g)
    for ball in config['fixture']['balls']:
        center=ball_world(config,ball['id']);geometry.append(sphere_envelope(ball['id'],center,radius))
        # Support in the open (+sensor Z) octant of the declared nominal visit.
        direction=np.asarray(ball.get('stem_direction_world',[0,0,-1]),dtype=float);direction/=np.linalg.norm(direction)
        stem=box_geometry(ball['id']+'_stem',center+direction*.047,[.001,.001,.035],False)
        placement=stem.placement;placement.rotation=Rotation.align_vectors(direction[None,:],np.array([[0,0,1.]]))[0].as_matrix()
        stem.placement=placement;geometry.append(stem)
    frozen_geometry=[dict(name=g.name,parent=g.parent,attached_to_base=g.attached_to_base,placement=g.placement.matrix().tolist(),
        vertices_sha256=sha256(np.asarray(g.vertices,dtype='<f8').tobytes()).hexdigest(),
        triangles_sha256=sha256(np.asarray(g.triangles,dtype='<i4').tobytes()).hexdigest()) for g in geometry]
    metadata={**base.metadata,'profile_id':'trical_k2_fixture_v1','synthetic_only':True,'hardware_executable':False,
        'contact_model':'three finite flat tip proxies, allowed geometric stroke; contact loads neglected',
        'geometry_sha256':digest(dict(base=base.metadata['geometry_sha256'],profile=config,geometry=frozen_geometry)),
        'fixture_geometry_measured':False,'exact_contact_pairs':[]}
    result=CollisionProfile(engine,geometry,exclusions,metadata)
    margin=config['workflow']['clearance_m'];thresholds=np.full(len(result.pairs),margin)
    if contact:
        if target is None:raise ValueError('Contact clearance requires an explicit selected target')
        frame,ball=target
        expected={(frame+'_tip_'+str(i),ball) for i in range(3)};found=set()
        for i,(a,b) in enumerate(result.pairs):
            pair=(result.names[a],result.names[b])
            if pair in expected or pair[::-1] in expected:
                thresholds[i]=-config['workflow']['allowed_tip_stroke_m'];found.add(pair if pair in expected else pair[::-1])
        if found!=expected:raise ValueError('Selected contact pairs are not present')
        result.metadata['exact_contact_pairs']=[dict(first=a,second=b,minimum_distance_m=-config['workflow']['allowed_tip_stroke_m']) for a,b in sorted(found)]
    # Optional K3 rigid-placement family budget: signed distance changes by at
    # most the sum of the two bodies' displacement bounds. Zero in K2.
    geometry_budget=config['workflow'].get('geometry_error_budget_m',0.)
    if not np.isfinite(geometry_budget) or geometry_budget<0:raise ValueError('Invalid geometric family displacement budget')
    for i,(a,b) in enumerate(result.pairs):
        thresholds[i]+=geometry_budget*((geometry[a].parent>=0)+(geometry[b].parent>=0))
    result.metadata['rigid_geometry_displacement_budget_m']=geometry_budget
    result.pair_clearance_m=thresholds
    result.metadata['pair_clearances_sha256']=digest(dict(pairs=[[result.names[a],result.names[b]] for a,b in result.pairs],thresholds=thresholds.tolist()))
    return result
