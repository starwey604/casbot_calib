"""Build same-source CAD scenes and replay references through the shared viewer."""
from pathlib import Path
import os
import xml.etree.ElementTree as ET
import numpy as np
from .dvt2_bundle import load_bundle,read,viewer_adapter
from .trical_assembly import asset_directory,asset_manifest
from .arm_viewer import _pose_attributes,_values


def write_views(folder):
    from .dvt2_view import robot_scene
    from .mesh_io import read_stl,write_stl
    folder=Path(folder);engine,metadata=load_bundle(folder);assets=asset_directory(folder);manifest=asset_manifest(assets)
    visuals={}
    for i,row in enumerate(manifest['parts']):
        if row['role']=='ball':continue
        source=assets/row['mesh']
        # MuJoCo's binary STL loader permits at most 200,000 triangles/file.
        # Split only the display asset, preserving every source triangle and
        # leaving the Coal source and its conservative covers unchanged.
        import struct
        with source.open('rb') as stream:stream.seek(80);count=struct.unpack('<I',stream.read(4))[0]
        if count<=200000:visuals[i]=[source];continue
        v,f=read_stl(source);paths=[];(folder/'visual_meshes').mkdir(exist_ok=True)
        for j,begin in enumerate(range(0,len(f),180000)):
            path=folder/'visual_meshes'/f'part_{i:03}_{j}.stl';write_stl(path,v,f[begin:begin+180000]);paths.append(path)
        visuals[i]=paths
    for side in ('left','right'):
        root,asset,world,bodies=robot_scene(engine,folder/'model.urdf')
        frame=next(f for f in engine.model.frames if f.name==side+'_trical_tcp')
        for i,row in enumerate(manifest['parts']):
            if row.get('side',side)!=side:continue
            if row['role']=='ball':
                ball=next(b for b in manifest['setup']['balls'] if b['id']==row['ball'])
                pos=np.array(ball['center_relative_waist_mm'])/1000+np.array(manifest['setup']['cad_to_robot']['world_waist_m'])
                ET.SubElement(world,'geom',name='datum_'+row['ball'],type='sphere',size='.0127',pos=_values(pos),rgba='.95 .72 .2 1');continue
            moving='side' in row
            parent=bodies[frame.parent] if moving else world
            pose=_pose_attributes(dict(rotation=frame.placement.rotation,translation=frame.placement.translation)) if moving else {}
            color='.95 .42 .25 1' if row['role']=='weight' else '.2 .72 .65 1' if moving else '.50 .57 .66 1'
            for j,path in enumerate(visuals[i]):
                name=f'fixture_asset_{i}_{j}'
                ET.SubElement(asset,'mesh',name=name,file=Path(os.path.relpath(path,folder)).as_posix())
                ET.SubElement(parent,'geom',name=f'fixture_{i}_{j}',type='mesh',mesh=name,rgba=color,**pose)
        parent=bodies[frame.parent];ET.SubElement(parent,'site',name=side+'_trical_tcp',size='.003',rgba='1 .2 .2 1',
            **_pose_attributes(dict(rotation=frame.placement.rotation,translation=frame.placement.translation)))
        ET.indent(root,space='  ');ET.ElementTree(root).write(folder/('scene_'+side+'.xml'),encoding='unicode')


def render(plan_folder,bundle,output):
    from PIL import Image,ImageDraw
    import imageio_ffmpeg,mujoco
    from .viewer import JointView
    from .presentation import _font
    from .telemetry import StateFrame
    plan_folder=Path(plan_folder);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    engine,metadata=load_bundle(bundle);report=read(plan_folder/'report.json');cases=read(plan_folder/'cases.json')
    protocol=read(plan_folder/'protocol.json');layout=protocol.get('fixture_layout')
    if protocol['bundle_sha256']!=metadata['bundle_sha256']:raise ValueError('Viewer and reference bundle identities differ')
    scenes=relocated_views(bundle,output/'scenes',layout) if layout else Path(bundle)
    names=tuple(metadata['joint_names']);adapter=viewer_adapter(bundle)
    rejected=['左'+r['ball'] if r['arm']=='left' else '右'+r['ball'] for r in cases if not r['passed']]
    checks=[];count=0
    def compose(rgb,text):
        canvas=Image.new('RGB',(1280,720),(12,21,33));draw=ImageDraw.Draw(canvas)
        draw.text((24,16),'1105_03 / 八组进退与脱离后转向',font=_font(26),fill=(235,241,247))
        draw.text((24,57),text,font=_font(18),fill=(60,220,193));canvas.paste(Image.fromarray(rgb),(10,95))
        for y,line in [(130,'同一表架 / 左右依次换装'),(175,'四球台架保持不动'),(245,'左安装 +90° / 右原装'),(290,'C++ Pinocchio / Coal'),(360,f"已放行 {len(report['admitted'])} / 8 条"),
                       (405,'预检拒绝：'+('、'.join(rejected) or '无')),(485,'参考轨迹 / 尚未采集'),(530,'500 Hz 计划 · 加速回放'),(575,'工具载荷尚待确认'),(620,'自由测杆行程待量测')]:
            draw.text((945,y),line,font=_font(16),fill=(220,229,240) if y!=405 else (255,181,114))
        draw.text((24,689),'MuJoCo 仅渲染。拒绝路径不生成执行参考；换臂须停放后人工换装。',font=_font(16),fill=(60,220,193))
        return np.asarray(canvas)
    writer=imageio_ffmpeg.write_frames(str(output/'trical_1105_workflow_zh.mp4'),(1280,720),fps=12,
        codec='libx264',pix_fmt_out='yuv420p',macro_block_size=16,output_params=['-crf','23','-preset','fast']);writer.send(None)
    forbidden=('mj_step','mj_forward','mj_inverse','mj_collision','mj_rne');original={x:getattr(mujoco,x) for x in forbidden}
    def no_physics(*a,**k):raise RuntimeError('Viewer cannot perform mechanics')
    try:
        for name in forbidden:setattr(mujoco,name,no_physics)
        for side in ('left','right'):
            with JointView(adapter,scenes/('scene_'+side+'.xml'),width=930,height=580) as view:
                view.camera.lookat[:]=[.15,0,1.03];view.camera.distance=1.8;view.camera.azimuth=135;view.camera.elevation=-18
                selected=[r for r in cases if r['arm']==side and r['passed']]
                for row in selected:
                    name=side+'_'+row['ball']
                    motion=read(plan_folder/name/'motion.json');knots=np.array(motion['knots_s']);labels=motion['labels']
                    phase_text={'initial_hold':'停放保持','approach':'自由空间接近','insert':'接近球面','contact_hold':'模型对中保持',
                        'insert_released':'正间隙接近／允许转向','insert_contact':'接触区／朝向受限','withdraw_contact':'初始退出／朝向受限',
                        'withdraw_released':'已脱离模型球面／允许转向','withdraw':'退出球面','return':'自由空间回位','final_hold':'停放保持'}
                    with np.load(plan_folder/name/'reference.npz',allow_pickle=False) as a:
                        q=a['q'];ns=a['time_ns'];indices=list(np.linspace(0,len(q)-1,48).astype(int));special={}
                        for k,label in enumerate(labels):
                            if label in {'contact_hold','withdraw_contact','withdraw_released'}:
                                index=int(np.argmin(abs(ns/1e9-(knots[k]+knots[k+1])/2)));indices.append(index);special[index]=label
                        indices=sorted(set(indices));maximum=0.;shown=set()
                        for i in indices:
                            frame=StateFrame(engine.model.model_id,'planned reference, not acquired data',count,int(ns[i]),int(ns[i]),names,q[i],None,None,None,clock_domain='virtual_reference')
                            rgb=view.render(frame);count+=1
                            site=mujoco.mj_name2id(view.model,mujoco.mjtObj.mjOBJ_SITE,side+'_trical_tcp')
                            expected=engine.frame_pose(q[i],side+'_trical_tcp')
                            maximum=max(maximum,float(np.max(abs(view.data.site_xpos[site]-expected[:3,3]))))
                            label=labels[min(len(labels)-1,max(0,int(np.searchsorted(knots,ns[i]/1e9,side='right')-1)))];shown.add(label)
                            text=('左臂' if side=='left' else '右臂')+' → '+row['ball']+'球 / '+phase_text[label]
                            composed=compose(rgb,text);writer.send(composed)
                            if i in special:
                                Image.fromarray(composed).save(output/(name+'_'+special[i]+'.png'))
                                if special[i]=='contact_hold':
                                    for _ in range(11):writer.send(composed);count+=1
                        checks.append(dict(case=name,maximum_TCP_error_m=maximum,qacc_zero=not np.any(view.data.qacc),shown_motion_labels=sorted(shown)))
                if not selected:
                    q=np.array(metadata['profile']['home_rad']);frame=StateFrame(engine.model.model_id,'parked model',count,0,0,names,q,None,None,None)
                    composed=compose(view.render(frame),('左臂' if side=='left' else '右臂')+' / 门禁拒绝，保持停放，不播放运动')
                    Image.fromarray(composed).save(output/(side+'_parked.png'))
                    for _ in range(24):writer.send(composed);count+=1
    finally:
        writer.close()
        for name,value in original.items():setattr(mujoco,name,value)
    result=dict(passed=bool(checks) and all(c['maximum_TCP_error_m']<1e-12 and c['qacc_zero'] for c in checks),
        checks=checks,frames=count,rejected=rejected,mechanics_calls_forbidden=True,source='500Hz reference, not acquisition')
    from .data import write_json
    write_json(output/'verification.json',result);return result


def relocated_views(bundle,output,layout):
    from .trical_assembly import is_platform_part,part_shift,ball_center
    bundle=Path(bundle);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    manifest=asset_manifest(asset_directory(bundle));load_bundle(bundle)
    for side in ('left','right'):
        tree=ET.parse(bundle/('scene_'+side+'.xml'));root=tree.getroot()
        for mesh in root.findall('./asset/mesh'):
            if mesh.get('file'):
                mesh.set('file',Path(os.path.relpath((bundle/mesh.get('file')).resolve(),output)).as_posix())
        for i,row in enumerate(manifest['parts']):
            if row['role']=='ball':
                node=root.find(f"./worldbody/geom[@name='datum_{row['ball']}']")
                node.set('pos',_values(ball_center(row['ball'],layout)))
            elif 'side' not in row and is_platform_part(row):
                for node in root.findall('./worldbody/geom'):
                    if node.get('name')==f'fixture_{i}' or node.get('name','').startswith(f'fixture_{i}_'):
                        node.set('pos',_values(part_shift(row,layout)))
        tree.write(output/('scene_'+side+'.xml'),encoding='unicode')
    return output


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description='只读渲染已通过门禁的三表参考；不连接SDK，不运行MuJoCo机械计算')
    for name in ['plan','bundle','output']:p.add_argument('--'+name,required=True)
    a=p.parse_args();raise SystemExit(0 if render(a.plan,a.bundle,a.output)['passed'] else 1)
