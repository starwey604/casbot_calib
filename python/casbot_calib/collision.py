"""Conservative nominal collision envelopes and adaptive trajectory gates.

Coal computes distances; Pinocchio places bodies. The interval certificate is
conditional on a supplied joint variation bound. Recorded-path checks certify
the declared joint-space linear interpolant, not unobserved physical motion.
"""
from importlib.resources import files
from io import BytesIO
import hashlib
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.spatial import ConvexHull
from . import _core
from .dual_model import nominal_dual_arm


def box_geometry(name,center,half_extents,attached_to_base=True):
    center=np.asarray(center,dtype=float);half=np.asarray(half_extents,dtype=float)
    if center.shape!=(3,) or half.shape!=(3,) or not np.isfinite(center).all() or not np.isfinite(half).all() or np.any(half<=0):
        raise ValueError('Finite center and positive box half extents required')
    vertices=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]],dtype=float)*half
    hull=ConvexHull(vertices);faces=hull.simplices.copy()
    normals=np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]])
    inward=np.sum(normals*hull.equations[:,:3],axis=1)<0;faces[inward]=faces[inward][:,[0,2,1]]
    geometry=_core.CollisionGeometry();geometry.name=name;geometry.parent=-1;geometry.attached_to_base=attached_to_base
    pose=_core.ArmPose();pose.translation=center;geometry.placement=pose;geometry.vertices=vertices;geometry.triangles=faces.astype(np.int32)
    return geometry


class CollisionProfile:
    def __init__(self,engine,geometry,excluded_pairs,metadata):
        self.engine=engine;self.geometry=geometry;self.metadata=metadata
        self.names=[g.name for g in geometry];self.native=_core.CollisionChecker(engine,geometry,excluded_pairs)
        self.pairs=list(self.native.pairs);self.radii=np.asarray(self.native.motion_radii())
        self.joints=len(engine.model.joints)

    def check(self,q):
        return self.native.check_batch(np.atleast_2d(np.asarray(q,dtype=float)))


def nominal_profile(engine=None,scene=None):
    reference,source=nominal_dual_arm();engine=engine or reference
    root=files('casbot_calib').joinpath('assets');metadata=json.loads(root.joinpath('nominal_collision.json').read_text())
    payload=root.joinpath('nominal_collision.npz').read_bytes()
    if hashlib.sha256(payload).hexdigest()!=metadata['array_sha256'] or source['sha256']!=metadata['source_urdf_sha256']:
        raise ValueError('Collision asset/source digest mismatch')
    actual=engine.model;expected=reference.model
    if actual.parents!=expected.parents or [j.name for j in actual.joints]!=metadata['joint_names']:
        raise ValueError('Collision profile joint order/tree mismatch')
    for first,second in zip(actual.joints,expected.joints,strict=True):
        if not np.allclose(first.origin.matrix(),second.origin.matrix(),atol=1e-12,rtol=0) or not np.allclose(first.axis,second.axis,atol=1e-12,rtol=0):
            raise ValueError('Kinematics changed; rebuild and verify the collision geometry profile')
    geometry=[]
    with np.load(BytesIO(payload),allow_pickle=False) as arrays:
        for item in metadata['geometries']:
            g=_core.CollisionGeometry();g.name=item['name'];g.parent=item['parent_joint_index'];g.vertices=arrays[item['array_key']+'_vertices'];g.triangles=arrays[item['array_key']+'_triangles'];geometry.append(g)
    names=[g.name for g in geometry];excluded=[(names.index(item['first']),names.index(item['second'])) for item in metadata['excluded_pairs']]
    if scene:
        for obstacle in scene.get('obstacles',[]):geometry.append(box_geometry(obstacle['name'],obstacle['center'],obstacle['half_extents'],obstacle.get('attached_to_base',True)))
    metadata={**metadata,'scene':scene or {},'geometry_sha256':hashlib.sha256(payload).hexdigest(),
        'backend':_core.CollisionChecker.backend_version(),'actual_geometry_count':len(geometry)}
    return CollisionProfile(engine,geometry,excluded,metadata)


def _certify(profile,evaluate,variation,intervals,margin,max_depth,time_map=None):
    if not np.isfinite(margin) or margin<0 or max_depth<0:raise ValueError('Invalid clearance/refinement depth')
    started=perf_counter();pending=[(float(a),float(b),0) for a,b in intervals];certified=queries=0
    thresholds=np.asarray(getattr(profile,'pair_clearance_m',np.full(len(profile.pairs),margin)),dtype=float)
    if thresholds.shape!=(len(profile.pairs),) or not np.isfinite(thresholds).all():raise ValueError('Invalid audited pair clearances')
    minimum=np.inf;lower_min=np.inf;witness=None
    while pending:
        next_round=[]
        for offset in range(0,len(pending),256):
            rows=pending[offset:offset+256];lo=np.array([a for a,_,_ in rows]);hi=np.array([b for _,b,_ in rows]);mid=(lo+hi)/2
            q=np.asarray(evaluate(mid),dtype=float);bounds=np.asarray(variation(lo,hi),dtype=float)
            if q.shape!=(len(rows),profile.joints) or bounds.shape!=q.shape or not np.isfinite(bounds).all() or np.any(bounds<0):
                raise ValueError('Invalid joint variation bounds')
            observed=profile.check(q);distances=observed['distances'];queries+=len(rows)
            lower=distances-bounds@profile.radii.T
            for i,(a,b,depth) in enumerate(rows):
                nearest=int(np.argmin(distances[i]-thresholds));distance=float(distances[i,nearest]);bound=float(lower[i].min())
                if distance<minimum:
                    minimum=distance;pair=profile.pairs[nearest]
                    witness=dict(parameter=float(mid[i]),time=float(time_map(mid[i])) if time_map else float(mid[i]),q=q[i].tolist(),
                        first=profile.names[pair[0]],second=profile.names[pair[1]],distance_m=distance,
                        first_point=observed['first_point'][i].tolist() if nearest==observed['worst_pair'][i] else None,
                        second_point=observed['second_point'][i].tolist() if nearest==observed['worst_pair'][i] else None)
                if np.any(distances[i]<=thresholds):
                    return dict(passed=False,reason='collision_or_clearance',minimum_sampled_distance_m=minimum,witness=witness,queries=queries,wall_s=perf_counter()-started)
                if np.all(lower[i]>thresholds):
                    certified+=1;lower_min=min(lower_min,bound)
                elif depth>=max_depth:
                    return dict(passed=False,reason='interval_not_certified',minimum_sampled_distance_m=minimum,witness=witness,queries=queries,wall_s=perf_counter()-started)
                else:next_round.extend([(a,float(mid[i]),depth+1),(float(mid[i]),b,depth+1)])
        pending=next_round
    return dict(passed=True,reason='bounded_intervals_certified',minimum_sampled_distance_m=minimum,certified_lower_bound_m=lower_min,
        clearance_m=margin,witness=witness,queries=queries,certified_intervals=certified,wall_s=perf_counter()-started)


def certify_reference(profile,q_at,duration_s,velocity_bound,clearance_m=.01,initial_step_s=.05,max_depth=16,position_allowance=None):
    velocity=np.broadcast_to(np.asarray(velocity_bound,dtype=float),(profile.joints,))
    if duration_s<=0 or initial_step_s<=0 or not np.isfinite(velocity).all() or np.any(velocity<0):raise ValueError('Invalid reference derivative bound')
    allowance=np.zeros(profile.joints) if position_allowance is None else np.asarray(position_allowance,dtype=float)
    if allowance.shape!=(profile.joints,) or not np.isfinite(allowance).all() or np.any(allowance<0):raise ValueError('Invalid reference position allowance')
    times=np.linspace(0,duration_s,max(2,int(np.ceil(duration_s/initial_step_s))+1))
    report=_certify(profile,q_at,lambda lo,hi:(hi-lo)[:,None]*velocity[None,:]/2+allowance,
        zip(times[:-1],times[1:]),clearance_m,max_depth)
    report.update(path_model='analytic reference conditional on supplied global derivative bounds and position allowance',velocity_bound_rad_s=velocity.tolist(),position_allowance_rad=allowance.tolist())
    return report


def certify_polyline(profile,positions,times_s=None,clearance_m=.01,max_depth=12,block_samples=50,max_gap_s=None):
    q=np.asarray(positions,dtype=float)
    if q.ndim!=2 or q.shape[1]!=profile.joints or len(q)<2 or not np.isfinite(q).all():raise ValueError('Expected at least two finite joint samples')
    index=np.arange(len(q),dtype=float)
    def at(values):return np.column_stack([np.interp(values,index,q[:,j]) for j in range(profile.joints)])
    if times_s is not None:
        times_s=np.asarray(times_s,dtype=float)
        if times_s.shape!=(len(q),) or not np.isfinite(times_s).all() or np.any(np.diff(times_s)<=0):raise ValueError('Expected strictly increasing sample times')
        if max_gap_s is not None and np.max(np.diff(times_s))>max_gap_s:raise ValueError('Sample gap exceeds the declared trajectory observation limit')
    if block_samples<1:raise ValueError('Positive observation block size required')
    def variation(lo,hi):
        middle=at((lo+hi)/2);left=at(lo);right=at(hi);bounds=[]
        for i,(a,b) in enumerate(zip(lo,hi)):
            inside=q[int(np.ceil(a)):int(np.floor(b))+1]
            values=np.vstack([left[i],right[i],inside])
            bounds.append(np.maximum(abs(values.min(axis=0)-middle[i]),abs(values.max(axis=0)-middle[i])))
        return np.array(bounds)
    # A block includes the extrema of EVERY observed waypoint, not only its
    # endpoints. Adaptive splits preserve the complete declared interpolant.
    knots=np.unique(np.r_[index[::block_samples],index[-1]])
    report=_certify(profile,at,variation,
        zip(knots[:-1],knots[1:]),clearance_m,max_depth,lambda t:np.interp(t,index,times_s) if times_s is not None else t)
    report['path_model']='joint-space piecewise linear interpolant of observations; unobserved hardware motion is not certified'
    return report
