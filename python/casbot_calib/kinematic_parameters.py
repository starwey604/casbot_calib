"""Exact Craig MDH + Hayati chart, mapped to the existing native Pinocchio tree.

Only constant coordinate changes are assembled here; robot FK and geometric
regressors are evaluated by Pinocchio, not by multiplying a Python FK chain.
"""
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation
from . import _core
from .kinematics import digest,geometry_manifest


def inverse(transform):
    out=np.eye(4);out[:3,:3]=transform[:3,:3].T;out[:3,3]=-out[:3,:3]@transform[:3,3];return out


def transform(translation=None, rotation=None):
    out=np.eye(4)
    if translation is not None:out[:3,3]=translation
    if rotation is not None:out[:3,:3]=rotation
    return out


def elementary(kind, axis, value):
    generator=np.zeros((4,4));direction=np.eye(3)[axis]
    if kind=='translation':
        result=transform(direction*value);generator[:3,3]=direction
    else:
        result=transform(rotation=Rotation.from_rotvec(direction*value).as_matrix())
        generator[:3,:3]=np.array([[0,-direction[2],direction[1]],[direction[2],0,-direction[0]],[-direction[1],direction[0],0]])
    return result,generator@result


def product_chart(specification):
    """Small constant-transform chart and exact derivatives, not runtime robot FK."""
    factors=[elementary(kind,axis,value) for _,kind,axis,value in specification]
    prefix=[np.eye(4)]
    for value,_ in factors:prefix.append(prefix[-1]@value)
    suffix=np.eye(4);derivatives={}
    for i in range(len(factors)-1,-1,-1):
        derivatives[specification[i][0]]=prefix[i]@factors[i][1]@suffix;suffix=factors[i][0]@suffix
    return prefix[-1],derivatives


def mdh_row(values):
    """TriCal Eq.1: Rx(alpha) Tx(a) Ry(beta) Rz(theta) Tz(d)."""
    return product_chart([(key,kind,axis,float(values[key])) for key,kind,axis in (
        ('alpha','rotation',0),('a','translation',0),('beta','rotation',1),('theta','rotation',2),('d','translation',2))])


def mount_chart(values):
    return product_chart([(key,kind,axis,float(values.get(key,0.))) for key,kind,axis in (
        ('tx','translation',0),('ty','translation',1),('tz','translation',2),
        ('rx','rotation',0),('ry','rotation',1),('rz','rotation',2))])


def _axis_frame(point, axis, hint):
    x=hint-axis*np.dot(axis,hint)
    if np.linalg.norm(x)<1e-10:x=np.eye(3)[np.argmin(abs(axis))]-axis*axis[np.argmin(abs(axis))]
    x/=np.linalg.norm(x);y=np.cross(axis,x)
    return transform(point,np.column_stack([x,y,axis]))


def _decompose(relative, hayati):
    r=relative[:3,:3];beta=float(np.arcsin(np.clip(r[0,2],-1,1))) if hayati else 0.
    alpha=float(np.arctan2(-r[1,2],r[2,2]));pre=Rotation.from_rotvec([alpha,0,0]).as_matrix()@Rotation.from_rotvec([0,beta,0]).as_matrix()
    rz=pre.T@r;theta=float(np.arctan2(rz[1,0],rz[0,0]));direction=pre[:,2];t=relative[:3,3]
    denom=direction[1:]@direction[1:]
    if denom<1e-10:raise ValueError('MDH chart singular; choose another nominal frame convention')
    d=float(t[1:]@direction[1:]/denom);a=float(t[0]-direction[0]*d)
    values=dict(alpha=alpha,a=a,beta=beta,theta=theta,d=d)
    if np.max(abs(mdh_row(values)[0]-relative))>1e-9:raise ValueError('Nominal transform is not representable in the declared MDH/Hayati chart')
    return values


def extract_chart(engine, parallel_tolerance=1e-5):
    """Build axis frames from nominal Pinocchio placements, with stable parallel handling."""
    model=engine.model;n=len(model.joints);zero=engine.fk(np.zeros(n));sides={};conversions=[None]*n
    for side in ('left','right'):
        indices=[i for i,j in enumerate(model.joints) if j.name.startswith(side+'_')]
        if len(indices)!=7:raise ValueError('Expected fixed-waist 7+7 model')
        points=[zero[i][:3,3] for i in indices];axes=[zero[i][:3,:3]@model.joints[i].axis for i in indices]
        frames=[];parallel=[]
        for k,i in enumerate(indices):
            point=points[k];axis=axes[k];hint=zero[i][:3,0]
            if k+1<len(indices):
                other=axes[k+1];delta=points[k+1]-point;near=abs(axis@other)>1-parallel_tolerance;parallel.append(near)
                if near:
                    # Project to the next axis without the ill-conditioned common-normal division.
                    shift=(delta@axis)/(other@axis);normal=delta-other*shift
                else:
                    along=np.linalg.lstsq(np.column_stack([axis,-other]),delta,rcond=None)[0]
                    point=point+along[0]*axis;normal=points[k+1]+along[1]*other-point
                    if np.linalg.norm(normal)<1e-9:normal=np.cross(axis,other)
                hint=normal if np.linalg.norm(normal)>1e-9 else hint
            frames.append(_axis_frame(point,axis,hint))
            conversions[i]=inverse(zero[i])@frames[-1]
        base=inverse(model.base.matrix())@frames[0];rows=[]
        for k,i in enumerate(indices):
            relative=np.eye(4) if k==0 else inverse(frames[k-1])@frames[k]
            rows.append(dict(index=i,joint=model.joints[i].name,convention='root_anchor' if k==0 else 'hayati' if parallel[k-1] else 'mdh',
                values=_decompose(relative,k>0 and parallel[k-1])))
        sides[side]=dict(indices=indices,base=base.tolist(),rows=rows)
    chart=dict(schema_version=1,convention='Rx(alpha) Tx(a) Ry(beta) Rz(theta+q) Tz(d)',
        robot_model_id=model.model_id,joint_names=[j.name for j in model.joints],sides=sides,
        source_geometry_sha256=geometry_manifest(engine,'nominal')['geometry_sha256'],
        frame_changes=[c.tolist() for c in conversions],parallel_tolerance=parallel_tolerance,
        gauge='root row fixed; first effective zero represented by mount rz; tools and metric artifact fixed by default')
    chart['sha256']=digest(chart);return chart


class KinematicParameters:
    def __init__(self, nominal, chart=None):
        self.nominal=nominal;self.chart=extract_chart(nominal) if chart is None else deepcopy(chart)
        check=deepcopy(self.chart);expected=check.pop('sha256')
        if digest(check)!=expected or self.chart['source_geometry_sha256']!=geometry_manifest(nominal,'nominal')['geometry_sha256']:
            raise ValueError('MDH chart identity does not match nominal geometry')
        self.definitions=[]
        # Zero stage first, preserving an interpretable calibration subset.
        for side,data in self.chart['sides'].items():
            for k,row in enumerate(data['rows']):
                self._add(side,k,'rz' if k==0 else 'theta','mount' if k==0 else 'row',True)
        for side,data in self.chart['sides'].items():
            for key in ('tx','ty','tz','rx','ry'):self._add(side,0,key,'mount',False)
            frozen_d={k-1 for k,row in enumerate(data['rows']) if row['convention']=='hayati'}
            for k,row in enumerate(data['rows'][1:],1):
                for key in ('alpha','a','d'):
                    if key=='d' and k in frozen_d:continue
                    self._add(side,k,key,'row',False)
                if row['convention']=='hayati':self._add(side,k,'beta','row',False)
        self.scales=np.array([d['scale'] for d in self.definitions]);self.size=len(self.definitions)

    def _add(self,side,row,key,location,zero):
        length=key in ('a','d','tx','ty','tz')
        self.definitions.append(dict(name=f'{side}.{location}_{row+1}.{key}',side=side,row=row,key=key,location=location,
            unit='m' if length else 'rad',scale=.0002 if length else .001,lower_scaled=-3.,upper_scaled=3.,zero_stage=zero))

    def build(self, scaled, derivatives=False):
        x=np.asarray(scaled,dtype=float)
        if x.shape!=(self.size,) or not np.isfinite(x).all():raise ValueError('Invalid scaled geometric parameter vector')
        chart=deepcopy(self.chart);mounts={side:{} for side in chart['sides']}
        for value,definition in zip(x*self.scales,self.definitions,strict=True):
            side,row,key=definition['side'],definition['row'],definition['key']
            if definition['location']=='mount':mounts[side][key]=value
            else:chart['sides'][side]['rows'][row]['values'][key]+=value
        model=self.nominal.model;joints=list(model.joints);p=self.size;n=len(joints);basis=np.zeros((6*n,p))
        for side,data in chart['sides'].items():
            mount,dm=mount_chart(mounts[side]);base=np.array(data['base'])
            for k,row in enumerate(data['rows']):
                i=row['index'];c=np.array(chart['frame_changes'][i]);suffix=inverse(c);a,da=mdh_row(row['values'])
                parent=base@mount if k==0 else np.array(chart['frame_changes'][data['indices'][k-1]])
                origin=parent@a@suffix;native=_core.ArmPose();native.rotation=origin[:3,:3];native.translation=origin[:3,3]
                joint=joints[i];joint.origin=native;joints[i]=joint
                if derivatives:
                    for j,definition in enumerate(self.definitions):
                        if definition['side']!=side:continue
                        if definition['location']=='mount' and k==0:dorigin=base@dm[definition['key']]@a@suffix
                        elif definition['location']=='row' and definition['row']==k:dorigin=parent@da[definition['key']]@suffix
                        else:continue
                        local=inverse(origin)@dorigin;omega=local[:3,:3]
                        basis[6*i:6*i+6,j]=np.r_[local[:3,3],omega[2,1],omega[0,2],omega[1,0]]*self.scales[j]
        model.joints=joints;engine=_core.ArmDynamics(model)
        return (engine,basis) if derivatives else engine

    def evaluate(self, scaled, q, frames, offsets):
        q=np.asarray(q);frames=np.asarray(frames);offsets=np.asarray(offsets)
        if q.ndim!=2 or q.shape[1]!=14 or frames.shape!=(len(q),) or offsets.shape!=(len(q),3):raise ValueError('Invalid geometric observation shapes')
        engine,basis=self.build(scaled,True);points=np.empty((len(q),3));jac=np.empty((len(q),3,self.size))
        for frame in np.unique(frames):
            selected=np.flatnonzero(frames==frame);result=engine.point_geometry_batch(q[selected],str(frame),offsets[selected])
            points[selected]=result['positions'];jac[selected]=result['placement_jacobians']@basis
        return points,jac

    def description(self):
        return dict(schema_version=1,chart=self.chart,parameters=self.definitions,
            units='SI; optimizer variables dimensionless; physical delta=scale*x',
            anchors=['canonical robot base fixed','mastered sensor axes/zero fixed','tool frame fixed','metric artifact fixed'],
            note='Remaining null directions fixed by training-only rank revelation, not by assuming unique physical parameters')

    def equivalent_mdh_engine(self,scaled):
        """An equivalent Pinocchio model in MDH joint frames, including inertias."""
        native=self.build(scaled);model=native.model;joints=list(model.joints);changes=[np.array(c) for c in self.chart['frame_changes']]
        for i,joint in enumerate(joints):
            parent=model.parents[i];cp=np.eye(4) if parent<0 else changes[parent];c=changes[i]
            origin=inverse(cp)@joint.origin.matrix()@c;p=_core.ArmPose();p.rotation=origin[:3,:3];p.translation=origin[:3,3]
            joint.origin=p;joint.axis=np.array([0.,0.,1.]);body=joint.body
            body.com=c[:3,:3].T@(body.com-c[:3,3]);body.inertia_com=c[:3,:3].T@body.inertia_com@c[:3,:3]
            joint.body=body;joints[i]=joint
        model.joints=joints;frames=list(model.frames)
        for frame in frames:
            if frame.parent<0:continue
            relative=inverse(changes[frame.parent])@frame.placement.matrix();p=_core.ArmPose();p.rotation=relative[:3,:3];p.translation=relative[:3,3]
            frame.placement=p
        model.frames=frames;return _core.ArmDynamics(model)
