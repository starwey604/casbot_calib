"""Self-contained seven-joint geometry scene and optional R1 snapshot.

MuJoCo receives poses only. Pinocchio supplies the independent FK reference.
No actuator, integration, forward/inverse dynamics or control callback exists here.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .data import write_json
from .telemetry import JointMapping,TelemetryAdapter,StateFrame


def _values(values):return " ".join(f"{float(v):.17g}" for v in values)


def _pose_attributes(pose):
    q=Rotation.from_matrix(np.asarray(pose['rotation'])).as_quat()
    return dict(pos=_values(pose['translation']),quat=_values(q[[3,0,1,2]]))


def write_arm_scene(config,path):
    """Export only declared geometry, never use inertia values from the viewer."""
    root=ET.Element('mujoco',model=config['robot_model_id'])
    ET.SubElement(root,'compiler',angle='radian')
    ET.SubElement(root,'option',gravity='0 0 0')
    default=ET.SubElement(root,'default')
    ET.SubElement(default,'geom',contype='0',conaffinity='0')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='1280',offheight='720')
    ET.SubElement(visual,'rgba',haze='.04 .06 .09 1')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='1 -1 2',diffuse='.8 .8 .8',dir='-1 1 -2')
    ET.SubElement(world,'geom',type='plane',pos='0 0 -.45',size='1.5 1.5 .05',rgba='.09 .13 .19 1')
    parent=ET.SubElement(world,'body',name='arm_fixed_base',**_pose_attributes(config['base']))
    ET.SubElement(parent,'geom',type='box',size='.07 .07 .02',rgba='.28 .36 .44 1')
    ET.SubElement(parent,'geom',type='capsule',size='.025',fromto=_values(np.r_[np.zeros(3),config['joints'][0]['origin']['translation']]),rgba='.28 .36 .44 1')
    entries=config['joints']
    for i,entry in enumerate(entries):
        parent=ET.SubElement(parent,'body',name=entry['link_name'],**_pose_attributes(entry['origin']))
        ET.SubElement(parent,'joint',name=entry['name'],type='hinge',axis=_values(entry['axis']),limited='false')
        end=np.array(entries[i+1]['origin']['translation']) if i+1<len(entries) else np.array(config['tool']['translation'])
        ET.SubElement(parent,'geom',type='sphere',size='.031',rgba='.9 .68 .29 1')
        if np.linalg.norm(end)>1e-5:
            ET.SubElement(parent,'geom',type='capsule',size='.022',fromto=_values(np.r_[np.zeros(3),end]),rgba='.15 .75 .67 1')
        else:
            ET.SubElement(parent,'geom',type='sphere',size='.025',pos='0 0 -.015',rgba='.15 .75 .67 1')
    tool=ET.SubElement(parent,'body',name='arm_tool',**_pose_attributes(config['tool']))
    ET.SubElement(tool,'site',name='arm_tcp',size='.009',rgba='1 .3 .4 1')
    if config['payload']['enabled']:
        ET.SubElement(tool,'geom',type='box',pos=_values(config['payload']['body']['com']),size='.025 .03 .045',rgba='.7 .35 .65 .9')
    ET.indent(root,space='  ');Path(path).write_text(ET.tostring(root,encoding='unicode')+'\n',encoding='utf-8')
    return TelemetryAdapter(config['robot_model_id'],[JointMapping(x['name'],x['name']) for x in entries])


def render_arm_snapshot(config,engine,folder):
    from PIL import Image,ImageDraw
    from .viewer import JointView
    from .presentation import _font
    folder=Path(folder);q=np.array(config['reference_pose']);poses=np.array(engine.fk(q))
    adapter=write_arm_scene(config,folder/'arm_scene.xml')
    write_json(folder/'arm_mapping.json',dict(robot_model_id=adapter.robot_model_id,
        joints=[m.__dict__ for m in adapter.mappings],note='Synthetic SI geometry probe, not verified SDK mapping'))
    frame=StateFrame(adapter.robot_model_id,'R1 model probe, not acquired telemetry',0,0,0,
        tuple(config['joint_names']),q,np.zeros(len(q)),np.zeros(len(q)),None,
        clock_domain='synthetic_probe_no_physical_clock')
    errors=[]
    with JointView(adapter,folder/'arm_scene.xml',width=900,height=580) as view:
        view.camera.lookat[:]=(poses[:,:3,3].max(axis=0)+poses[:,:3,3].min(axis=0))/2
        view.camera.distance=max(.8,float(np.linalg.norm(np.ptp(poses[:,:3,3],axis=0)))*1.9)
        view.camera.azimuth=135;view.camera.elevation=-15
        rgb=view.render(frame)
        for name,expected in zip([x['link_name'] for x in config['joints']]+['arm_tool'],poses,strict=True):
            index=view.mj.mj_name2id(view.model,view.mj.mjtObj.mjOBJ_BODY,name)
            errors.append(dict(body=name,position_m=float(np.max(np.abs(view.data.xpos[index]-expected[:3,3]))),
                rotation=float(np.max(np.abs(view.data.xmat[index].reshape(3,3)-expected[:3,:3])))))
        if np.any(view.data.qacc):raise RuntimeError('Viewer unexpectedly contains computed accelerations')
    canvas=Image.new('RGB',(1280,720),(10,17,27));draw=ImageDraw.Draw(canvas)
    draw.text((24,18),'CASBOT / 七轴整臂统一模型',font=_font(26),fill=(226,235,243))
    draw.text((24,58),'R1：Pinocchio 动力学内核与几何坐标核对',font=_font(16),fill=(50,220,190))
    canvas.paste(Image.fromarray(rgb),(20,98))
    labels=['肩俯仰','肩滚转','肩偏航','肘俯仰','腕偏航','腕俯仰','腕滚转']
    draw.text((945,115),'固定基座 / 右臂七轴',font=_font(18),fill=(226,235,243))
    for i,value in enumerate(q):
        label=labels[i] if i<len(labels) else config['joint_names'][i]
        draw.text((945,170+i*48),f'{i+1}  {label}   {np.degrees(value):+.1f}°',font=_font(16),fill=(226,235,243))
    draw.text((945,550),'合成惯性 / 合成摩擦',font=_font(16),fill=(255,195,78))
    draw.text((945,585),'尚未执行实机标定',font=_font(16),fill=(139,160,180))
    draw.text((24,690),'MuJoCo 仅更新几何；不推进物理，不发送电机指令。',font=_font(14),fill=(50,220,190))
    canvas.save(folder/'arm_geometry.png')
    result=dict(passed=all(x['position_m']<1e-12 and x['rotation']<1e-12 for x in errors),
        body_errors=errors,mujoco_usage='geometry only: mj_kinematics / mj_camlight; no dynamics',
        frame_source='synthetic model probe; current placeholders are not acquired measurements')
    write_json(folder/'geometry_check.json',result)
    return result
