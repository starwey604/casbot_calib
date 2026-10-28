"""Export calibrated native geometry without changing link/body coordinate gauges."""
import csv
from copy import deepcopy
from hashlib import sha256
from importlib.resources import files
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from . import _core
from .kinematic_parameters import inverse
from .data import write_json


def export_table(parameters,scaled,output):
    chart=deepcopy(parameters.chart)
    for side in chart['sides']:chart['sides'][side]['mount_corrections']={key:0. for key in ('tx','ty','tz','rx','ry','rz')}
    for value,definition in zip(scaled*parameters.scales,parameters.definitions,strict=True):
        data=chart['sides'][definition['side']]
        if definition['location']=='mount':data['mount_corrections'][definition['key']]=float(value)
        else:data['rows'][definition['row']]['values'][definition['key']]+=float(value)
    chart['nominal_chart_sha256']=chart.pop('sha256');write_json(Path(output)/'mdh_table.json',chart)
    with (Path(output)/'mdh_table.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['side','joint','convention','alpha_rad','a_m','beta_rad','theta_rad','d_m']);writer.writeheader()
        for side,data in chart['sides'].items():
            for row in data['rows']:
                values=row['values'];writer.writerow(dict(side=side,joint=row['joint'],convention=row['convention'],
                    alpha_rad=values['alpha'],a_m=values['a'],beta_rad=values['beta'],theta_rad=values['theta'],d_m=values['d']))
    with (Path(output)/'mount_table.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        fields=['side','tx_m','ty_m','tz_m','rx_rad','ry_rad','rz_rad'];writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for side,data in chart['sides'].items():
            values=data['mount_corrections'];writer.writerow(dict(side=side,**{key+('_m' if key.startswith('t') else '_rad'):value for key,value in values.items()}))


def export_urdf(nominal,candidate,config,path):
    raw=files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf').read_bytes();root=ET.fromstring(raw)
    source=nominal.model;model=candidate.model;zero=np.zeros(14);poses=nominal.fk(zero)
    by_name={joint.get('name'):joint for joint in root.findall('joint')}
    for i,joint in enumerate(model.joints):
        element=by_name[joint.name];parent_link=element.find('parent').get('link');parent=source.parents[i]
        parent_pose=source.base.matrix() if parent<0 else poses[parent]
        fixed=inverse(parent_pose)@nominal.frame_pose(zero,parent_link)
        local=inverse(fixed)@joint.origin.matrix();origin=element.find('origin')
        if origin is None:origin=ET.SubElement(element,'origin')
        origin.set('xyz',' '.join(f'{x:.17g}' for x in local[:3,3]))
        origin.set('rpy',' '.join(f'{x:.17g}' for x in Rotation.from_matrix(local[:3,:3]).as_euler('xyz')))
    for tool in config['tools']:
        ET.SubElement(root,'link',name=tool['name']);element=ET.SubElement(root,'joint',name=tool['name']+'_mount',type='fixed')
        ET.SubElement(element,'parent',link=tool['parent_frame']);ET.SubElement(element,'child',link=tool['name'])
        ET.SubElement(element,'origin',xyz=' '.join(f'{x:.17g}' for x in tool['translation_m']),
            rpy=' '.join(f'{x:.17g}' for x in Rotation.from_rotvec(tool['rotation_vector_rad']).as_euler('xyz')))
    ET.indent(root,space='  ');path=Path(path);path.write_text(ET.tostring(root,encoding='unicode')+'\n',encoding='utf-8')
    imported=_core.ArmDynamics.from_urdf(str(path));rng=np.random.default_rng(20261030);worst=0.;mass_error=0.
    for q in rng.uniform(-.5,.5,(8,14)):
        for tool in config['tools']:worst=max(worst,float(np.max(abs(imported.frame_pose(q,tool['name'])-candidate.frame_pose(q,tool['name'])))))
        mass_error=max(mass_error,float(np.max(abs(imported.mass_matrix(q)-candidate.mass_matrix(q)))))
    if worst>1e-11 or mass_error>1e-11:raise ValueError('Exported URDF does not reproduce the calibrated native model')
    return dict(passed=True,urdf_sha256=sha256(path.read_bytes()).hexdigest(),max_pose_error=worst,max_mass_matrix_error=mass_error,
        source_inertial_properties_unchanged=True,tool_load_added=False,mesh_assets_bundled=False,
        hardware_executable=False,dynamics_revalidation_required=True,collision_revalidation_required=True)
