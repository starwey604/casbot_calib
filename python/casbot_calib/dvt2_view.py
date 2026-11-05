"""Derived CAD scene for the existing geometry-only MuJoCo viewer."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
from .arm_viewer import _pose_attributes,_values
from .telemetry import TelemetryAdapter,JointMapping
from .dvt2_bundle import load_bundle,robot_profile,trical_scene
from .trical_fixture import fixture_profile


def robot_scene(engine,urdf,floor_surface_z_m=0.):
    """Shared CAD-to-viewer tree; no assembly policy or mechanical stepping."""
    folder=Path(urdf).parent;model=engine.model
    root=ET.Element('mujoco',model=model.model_id+'_geometry_only');ET.SubElement(root,'compiler',angle='radian')
    ET.SubElement(root,'option',gravity='0 0 0');default=ET.SubElement(root,'default')
    ET.SubElement(default,'geom',contype='0',conaffinity='0',density='0')
    ET.SubElement(root,'statistic',center='0 0 1.0',extent='1.1')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='1280',offheight='720')
    asset=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody');ET.SubElement(world,'light',pos='1 -1 3',dir='-1 1 -2',diffuse='.9 .9 .9')
    ET.SubElement(world,'geom',name='visual_floor',type='plane',pos=_values([0,0,floor_surface_z_m]),size='2 2 .05',rgba='.08 .11 .15 1')
    bodies=[]
    for j,joint in enumerate(model.joints):
        parent=world if model.parents[j]<0 else bodies[model.parents[j]]
        body=ET.SubElement(parent,'body',name=joint.name+'_view',**_pose_attributes(dict(rotation=joint.origin.rotation,translation=joint.origin.translation)))
        bodies.append(body);ET.SubElement(body,'joint',name=joint.name,type='hinge',axis=_values(joint.axis),limited='false')
        ET.SubElement(body,'inertial',pos='0 0 0',mass='1',diaginertia='1 1 1') # viewer compilation scaffold only
    reduced=pin.buildModelFromUrdf(str(folder/'model.urdf'));geometry=pin.buildGeomFromUrdf(reduced,str(folder/'model.urdf'),pin.GeometryType.VISUAL,package_dirs=[str(folder)])
    for i,g in enumerate(geometry.geometryObjects):
        parent=world if g.parentJoint==0 else bodies[g.parentJoint-1]
        color='.65 .70 .76 1' if g.parentJoint==0 else ('.16 .66 .77 1' if g.parentJoint<=7 else '.25 .48 .85 1')
        attrs=dict(name='cad_'+g.name,rgba=color,**_pose_attributes(dict(rotation=g.placement.rotation,translation=g.placement.translation)))
        if hasattr(g.geometry,'vertices'):
            name='cad_asset_'+str(i);ET.SubElement(asset,'mesh',name=name,file=Path(os.path.relpath(g.meshPath,folder)).as_posix(),scale=_values(g.meshScale));attrs.update(type='mesh',mesh=name)
        elif type(g.geometry).__name__=='Sphere':attrs.update(type='sphere',size=str(g.geometry.radius))
        else:raise ValueError('Unsupported source visual shape: '+type(g.geometry).__name__)
        ET.SubElement(parent,'geom',**attrs)
    return root,asset,world,bodies


def write_view(folder):
    folder=Path(folder);engine,metadata=load_bundle(folder);model=engine.model
    base=robot_profile(folder);config=trical_scene(folder);profile=fixture_profile(engine,config,base_profile=base)
    root,asset,world,bodies=robot_scene(engine,folder/'model.urdf',metadata['scene']['floor_surface_z_m'])
    # Only add proxies/balls from the audited scene; robot visuals above use source CAD.
    for i,g in enumerate(profile.geometry[len(base.geometry):]+base.geometry[-len(metadata['scene']['obstacles']):]):
        name='proxy_'+str(i);ET.SubElement(asset,'mesh',name=name,vertex=_values(g.vertices.ravel()),face=' '.join(str(int(x)) for x in g.triangles.ravel()))
        parent=world if g.parent<0 else bodies[g.parent]
        color='.95 .40 .32 1' if '_tip_' in g.name else '.25 .8 .67 1' if 'trical' in g.name else '.95 .73 .23 1'
        if 'floor_proxy' in g.name:continue
        ET.SubElement(parent,'geom',name=g.name+'_proxy',type='mesh',mesh=name,rgba=color,**_pose_attributes(dict(rotation=g.placement.rotation,translation=g.placement.translation)))
    for tool in metadata['profile']['tools']:
        frame=next(f for f in model.frames if f.name==tool['name']);parent=world if frame.parent<0 else bodies[frame.parent]
        ET.SubElement(parent,'site',name=tool['name'],size='.003',rgba='1 .3 .3 1',**_pose_attributes(dict(rotation=frame.placement.rotation,translation=frame.placement.translation)))
    ET.indent(root,space='  ');ET.ElementTree(root).write(folder/'scene.xml',encoding='unicode')
    return TelemetryAdapter(model.model_id,[JointMapping(j.name,j.name) for j in model.joints])
