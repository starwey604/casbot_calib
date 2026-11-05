#!/usr/bin/env python3
"""Geometry-only close-ups of the audited body envelopes at original poses."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image,ImageDraw
from casbot_calib.dvt2_bundle import read,load_bundle,viewer_adapter
from casbot_calib.trical_assembly import profile,asset_directory,asset_manifest
from casbot_calib.trical_assembly_view import relocated_views
from casbot_calib.arm_viewer import _values,_pose_attributes
from casbot_calib.telemetry import StateFrame
from casbot_calib.viewer import JointView
from casbot_calib.presentation import _font
from casbot_calib.data import write_json
from casbot_calib.mesh_io import read_stl


def render(old,new,audit,output):
    import mujoco
    out=Path(output);out.mkdir(parents=True,exist_ok=False);canvas=Image.new('RGB',(1800,800),(12,21,33));draw=ImageDraw.Draw(canvas);checks=[]
    draw.text((25,17),'1105_03 / 跨侧访问的两个不同瓶颈',font=_font(30),fill=(235,241,247))
    rows=read(asset_directory(old)/'seeds.json')['results']
    original={k:getattr(mujoco,k) for k in ['mj_step','mj_forward','mj_inverse','mj_collision','mj_rne']}
    def reject(*a,**k):raise RuntimeError('No mechanics in viewer')
    try:
        for k in original:setattr(mujoco,k,reject)
        for i,(bundle,side,bid) in enumerate([(old,'left','D'),(new,'left','D'),(new,'right','C')]):
            engine,meta=load_bundle(bundle);row=next(r for r in rows if (r['arm'],r['ball'])==(side,bid));q=np.array(row['contact'][0]['q']);p=profile(bundle,side,bid,True)
            scenes=relocated_views(bundle,out/f'scene_{i}',None);path=scenes/('scene_'+side+'.xml');tree=ET.parse(path);root=tree.getroot()
            manifest=asset_manifest(asset_directory(bundle));part=next(j for j,r in enumerate(manifest['parts']) if r['name']=='left:Part__Feature5160') if i<2 else None
            for geom in root.findall('.//geom'):
                name=geom.get('name','');show=(i<2 and (name.startswith(f'fixture_{part}_') or name.startswith('fixture_2_') or name.startswith('datum_')))
                if i==2:show=name in {'cad_waist_yaw_link_0','cad_right_shoulder_roll_link_0'}
                if not show:geom.set('rgba','0 0 0 0')
                elif i<2 and name.startswith(f'fixture_{part}_'):geom.set('rgba','.25 .72 .92 1')
            if i<2:
                selected=[g for g in p.geometry if g.name.startswith('left:Part__Feature5160')]
                for k,g in enumerate(selected):
                    name=f'audit_hull_{k}';ET.SubElement(root.find('asset'),'mesh',name=name,vertex=_values(np.asarray(g.vertices).ravel()),face=' '.join(map(str,np.asarray(g.triangles).ravel())))
                    parent=root.find(f".//body[@name='{meta['joint_names'][g.parent]}_view']")
                    ET.SubElement(parent,'geom',type='mesh',name=name,mesh=name,rgba='.95 .30 .12 .40' if i==0 else '.25 .95 .4 .35',**_pose_attributes(dict(rotation=g.placement.rotation,translation=g.placement.translation)))
                g=selected[0];pose=engine.fk(q)[g.parent]@g.placement.matrix();vertices,_=read_stl(asset_directory(bundle)/manifest['parts'][part]['mesh']);points=vertices@pose[:3,:3].T+pose[:3,3]
                look=points.mean(0);look[2]-=.025
            else:
                ref=next(r for r in read(Path(audit)/'audit.json')['reference'] if r['side']==side and r['label']=='C_contact')
                points=np.array(ref['source_nearest_points_world_m']);look=points.mean(0)
                for k,point in enumerate(points):ET.SubElement(root.find('worldbody'),'geom',name=f'witness_{k}',type='sphere',size='.0015',pos=_values(point),rgba='1 .4 .15 1')
            tree.write(path,encoding='unicode')
            frame=StateFrame(engine.model.model_id,'original geometric reference',i,0,0,tuple(meta['joint_names']),q,None,None,None)
            with JointView(viewer_adapter(bundle),path,width=590,height=560) as view:
                view.camera.lookat[:]=look;view.camera.distance=.47 if i<2 else .42;view.camera.azimuth=135 if i<2 else -135;view.camera.elevation=-18
                image=view.render(frame);canvas.paste(Image.fromarray(image),(i*600,125))
                site=mujoco.mj_name2id(view.model,mujoco.mjtObj.mjOBJ_SITE,side+'_trical_tcp');err=float(np.max(abs(view.data.site_xpos[site]-engine.frame_pose(q,side+'_trical_tcp')[:3,3])))
                checks.append(dict(panel=i,TCP_error_m=err,qacc_zero=not np.any(view.data.qacc)))
            titles=['左D / 原单凸包','左D / 六段保守外包','右C / 肩部—腰部']
            draw.text((20+i*600,83),titles[i],font=_font(24),fill=(235,241,247))
            lines=[['表体—柱：单凸包 5.00 mm','源网格表面：12.04 mm'],['表体—柱：分段后约11.84 mm','修正空隙，不放宽10 mm规则'],['源网格与凸包均为4.85 mm','原姿态间隙不足10 mm，需换姿态']][i]
            for j,line in enumerate(lines):draw.text((20+i*600,690+j*34),line,font=_font(20),fill=(255,192,105) if j==0 else (190,210,230))
        draw.text((25,768),'原review姿态的几何对照；完整轨迹和±0.00105 rad误差包络另行验收。MuJoCo不参与机械计算。',font=_font(18),fill=(63,202,211))
        canvas.save(out/'collision_envelopes_zh.png')
    finally:
        for k,value in original.items():setattr(mujoco,k,value)
    result=dict(passed=all(r['TCP_error_m']<1e-12 and r['qacc_zero'] for r in checks),checks=checks,mechanics_calls_forbidden=True);write_json(out/'verification.json',result);return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['old','new','audit','output']:p.add_argument('--'+name,required=True)
    a=p.parse_args();print(render(a.old,a.new,a.audit,a.output))
