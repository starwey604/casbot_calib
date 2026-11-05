"""Independent full-source comparisons and new-scene path checks for N2 bundles."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
from .data import write_json
from .dvt2_build import transform,source_assembly_model,assembly_xml
from .dvt2_bundle import load_bundle,robot_profile,trical_scene
from .model_sources import dvt2_source
from .trical_fixture import fixture_profile
from .trical_motion import solve_position,segment


def verify_reduction(folder,*,samples=12):
    root=Path(folder);engine,metadata=load_bundle(root);c=metadata['profile'];source=dvt2_source(c['source_variant'])
    if source.urdf_sha256!=metadata['source_urdf_sha256']:raise ValueError('Source changed since reduction')
    full=source_assembly_model(source,c['assembly']);q0=pin.neutral(full)
    for name,value in c['lock_positions_rad'].items():q0[full.joints[full.getJointId(name)].idx_q]=value
    qi=[full.joints[full.getJointId(n)].idx_q for n in metadata['joint_names']];vi=[full.joints[full.getJointId(n)].idx_v for n in metadata['joint_names']]
    world=transform(c['world_from_source']);full.gravity.linear=world.rotation.T@np.array(c['gravity_world_m_s2'])
    rng=np.random.default_rng(20261102);home=np.array(c['home_rad']);limits=np.array([[j.lower,j.upper] for j in engine.model.joints])
    poses=np.clip(home+rng.uniform(-.25,.25,(samples,14)),limits[:,0]+.01,limits[:,1]-.01);poses[0]=home
    errors=dict(frame_pose=0.,tool_pose=0.,mass_matrix=0.,gravity=0.,inverse_dynamics=0.);workspace=engine.workspace();data=full.createData()
    frames=[frame.name for frame in full.frames[1:] if frame.name in engine.frame_names()]
    for q in poses:
        fq=q0.copy();fq[qi]=q;v=rng.normal(0,.15,14);a=rng.normal(0,.3,14);fv=np.zeros(full.nv);fa=np.zeros(full.nv);fv[vi]=v;fa[vi]=a
        pin.framesForwardKinematics(full,data,fq)
        for name in frames:
            expected=(world*data.oMf[full.getFrameId(name)]).homogeneous
            errors['frame_pose']=max(errors['frame_pose'],float(np.max(abs(expected-workspace.frame_pose(q,name)))))
        for tool in c['tools']:
            expected=(world*data.oMf[full.getFrameId(tool['parent_frame'])]*transform(tool)).homogeneous
            errors['tool_pose']=max(errors['tool_pose'],float(np.max(abs(expected-workspace.frame_pose(q,tool['name'])))))
        matrix=np.array(pin.crba(full,data,fq));matrix=np.triu(matrix)+np.triu(matrix,1).T
        errors['mass_matrix']=max(errors['mass_matrix'],float(np.max(abs(matrix[np.ix_(vi,vi)]-workspace.mass_matrix(q)))))
        g=np.array(pin.computeGeneralizedGravity(full,data,fq))[vi]
        errors['gravity']=max(errors['gravity'],float(np.max(abs(g-workspace.gravity(q)))))
        effort=np.array(pin.rnea(full,data,fq,fv,fa))[vi]
        errors['inverse_dynamics']=max(errors['inverse_dynamics'],float(np.max(abs(effort-workspace.inverse(q,v,a)))))
    def mass(path):return sum(float(x.get('value')) for x in ET.parse(path).getroot().findall('./link/inertial/mass'))
    source_mass=sum(float(x.get('value')) for x in assembly_xml(source,c['assembly']).findall('./link/inertial/mass'));retained_mass=mass(root/'model.urdf')
    joints=engine.model.joints
    limits_match=(np.array_equal(full.lowerPositionLimit[qi],[j.lower for j in joints])
        and np.array_equal(full.upperPositionLimit[qi],[j.upper for j in joints])
        and np.array_equal(full.velocityLimit[vi],[j.velocity_limit for j in joints])
        and np.array_equal(full.effortLimit[vi],[j.effort_limit for j in joints]))
    return dict(passed=bool(limits_match and all(v<1e-10 for v in errors.values()) and abs(source_mass-retained_mass)<1e-10),joint_limits_match=bool(limits_match),max_absolute_errors=errors,
        configuration_count=len(poses),common_source_frames=len(frames),source_total_link_mass_kg=source_mass,retained_total_link_mass_kg=retained_mass,
        comparison='Original full source URDF at locked coordinates vs derived native model; no candidate-to-itself-only test',hardware_tested=False)


def verify_scene(folder):
    root=Path(folder);engine,metadata=load_bundle(root);config=trical_scene(root);base=robot_profile(root);free=fixture_profile(engine,config,base_profile=base)
    home=np.array(metadata['profile']['home_rad']);rows=[]
    for visit in config['workflow']['visits']:
        frame=visit['frame'];goal=np.array(visit['nominal_q_rad']);pose=engine.frame_pose(goal,frame)
        contact=fixture_profile(engine,config,(frame,visit['ball_id']),True,base_profile=base)
        try:
            stage=solve_position(engine,frame,pose[:3,3]-pose[:3,:3]@np.full(3,.06),goal)
            gates=[]
            for start,end,profile,label in [(home,stage,free,'approach'),(stage,goal,contact,'insert'),(goal,stage,contact,'withdraw'),(stage,home,free,'return')]:
                _,gate=segment(profile,start,end,config,label);gates.append(dict(label=label,**gate))
            rows.append(dict(visit=visit['id'],passed=all(x['passed'] for x in gates),staging_q_rad=stage.tolist(),gates=gates,
                exact_contact_pairs=contact.metadata['exact_contact_pairs']))
        except ValueError as e:rows.append(dict(visit=visit['id'],passed=False,reason=str(e)))
    ds=free.check(home)['distances'][0];i=int(np.argmin(ds));pair=free.pairs[i]
    return dict(passed=all(x['passed'] for x in rows) and float(ds.min())>=config['workflow']['clearance_m'],visits=rows,
        home_minimum_clearance_m=float(ds.min()),home_closest_pair=[free.names[k] for k in pair],geometry_count=len(free.geometry),active_pairs=len(free.pairs),
        proxy_dimensions_measured=False,path_scope='Continuous planned paths and stated uncertainty only; not real motion or metrology')


def render_bundle(folder, output, scene_report=None):
    """Read-only geometry probe; optional replay uses already-certified references."""
    from PIL import Image, ImageDraw
    import mujoco
    from .viewer import JointView
    from .telemetry import StateFrame
    from .presentation import _font
    from .dvt2_bundle import viewer_adapter
    from .staged_motion import Motion

    root=Path(folder);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    engine,metadata=load_bundle(root);config=trical_scene(root);adapter=viewer_adapter(root)
    home=np.array(metadata['profile']['home_rad']);names=tuple(metadata['joint_names'])
    probes=[('home',home),('left_wrist',np.array(config['workflow']['visits'][2]['nominal_q_rad'])),
            ('right_wrist',np.array(config['workflow']['visits'][5]['nominal_q_rad']))]
    errors=[]
    def frame(q,index):
        return StateFrame(engine.model.model_id,'N2 synthetic geometry reference',index,index*100_000_000,
                          index*100_000_000,names,q,None,None,None,clock_domain='synthetic_reference')
    def canvas(rgb,label):
        image=Image.new('RGB',(1280,720),(10,17,27));draw=ImageDraw.Draw(image)
        draw.text((26,18),'DVT2 / 固定腰部双臂候选模型',font=_font(27),fill=(232,240,246))
        draw.text((26,60),label,font=_font(18),fill=(61,222,194))
        image.paste(Image.fromarray(rgb),(12,96))
        for y,text in [(130,'左臂 7 轴 / 青色'),(175,'右臂 7 轴 / 蓝色'),(245,'金色：合成夹具与球架'),
                       (290,'红色：三表测头代理'),(390,'普通双手候选'),(435,'末端装配尚待确认'),
                       (520,'安装与工装尺寸为模拟值'),(565,'未接入实机')]:
            draw.text((955,y),text,font=_font(17),fill=(220,230,241))
        draw.text((26,689),'Pinocchio 计算位姿；MuJoCo 仅显示几何，不参与机械计算。',font=_font(16),fill=(61,222,194))
        return np.asarray(image)
    # Fail immediately if any viewer path starts doing physics or collision work.
    forbidden=('mj_step','mj_forward','mj_inverse','mj_collision','mj_rne')
    originals={name:getattr(mujoco,name) for name in forbidden}
    def forbidden_call(*args,**kwargs):raise RuntimeError('MuJoCo physics is forbidden in the N2 viewer')
    try:
        for name in forbidden:setattr(mujoco,name,forbidden_call)
        with JointView(adapter,root/'scene.xml',width=930,height=580) as view:
            waist=engine.frame_pose(home,'waist_yaw_link')[:3,3]
            view.camera.lookat[:]=waist;view.camera.distance=2.7;view.camera.azimuth=205;view.camera.elevation=-12
            for index,(label,q) in enumerate(probes):
                view.camera.distance=2.7 if label=='home' else 2.3
                view.camera.lookat[:]=waist+np.array([0.,0.,0. if label=='home' else .25])
                rgb=view.render(frame(q,index));maximum=0.
                for joint,expected in zip(engine.model.joints,engine.fk(q),strict=True):
                    body=mujoco.mj_name2id(view.model,mujoco.mjtObj.mjOBJ_BODY,joint.name+'_view')
                    maximum=max(maximum,float(np.max(abs(view.data.xpos[body]-expected[:3,3]))),
                                float(np.max(abs(view.data.xmat[body].reshape(3,3)-expected[:3,:3]))))
                for tool in metadata['profile']['tools']:
                    sid=mujoco.mj_name2id(view.model,mujoco.mjtObj.mjOBJ_SITE,tool['name']);expected=engine.frame_pose(q,tool['name'])
                    maximum=max(maximum,float(np.max(abs(view.data.site_xpos[sid]-expected[:3,3]))),
                                float(np.max(abs(view.data.site_xmat[sid].reshape(3,3)-expected[:3,:3]))))
                errors.append(dict(probe=label,maximum_pose_error=maximum,accelerations_zero=not np.any(view.data.qacc)))
                Image.fromarray(canvas(rgb,{'home':'起始姿态与重建场景','left_wrist':'左臂工具与腕部姿态','right_wrist':'右臂工具与腕部姿态'}[label])).save(output/(label+'.png'))
            video=False
            view.camera.distance=2.3;view.camera.lookat[:]=waist+np.array([0.,0.,.25])
            if scene_report is not None and scene_report['passed']:
                import imageio_ffmpeg
                writer=imageio_ffmpeg.write_frames(str(output/'dvt2_candidate_zh.mp4'),(1280,720),fps=12,
                    codec='libx264',pix_fmt_out='yuv420p',macro_block_size=16,output_params=['-crf','22','-preset','fast']);writer.send(None)
                try:
                    i=0
                    for visit in scene_report['visits']:
                        if not visit['visit'].endswith('_3'):continue
                        target=next(v for v in config['workflow']['visits'] if v['id']==visit['visit'])
                        stage=np.array(visit['staging_q_rad']);goal=np.array(target['nominal_q_rad'])
                        for start,end,label in [(home,stage,'接近球架'),(stage,goal,'对准三表接触位'),(goal,stage,'退出接触位'),(stage,home,'返回起始姿态')]:
                            curve=Motion('N2 replay','trical',start).add(5.,end).polynomial()
                            for t in np.linspace(0,5.,12):
                                rgb=view.render(frame(curve(t),i));i+=1
                                side='左臂' if visit['visit'].startswith('left') else '右臂'
                                writer.send(canvas(rgb,side+' / '+label+'（参考路径加速回放）'))
                    video=True
                finally:writer.close()
    finally:
        for name,value in originals.items():setattr(mujoco,name,value)
    return dict(passed=all(e['maximum_pose_error']<1e-12 and e['accelerations_zero'] for e in errors),probes=errors,
        video_written=video,mujoco_usage='mj_kinematics/mj_camlight only; physics calls forbidden',hardware_tested=False)


def main():
    import argparse
    parser=argparse.ArgumentParser(description='N2完整源模型对照、场景门禁及只读渲染')
    parser.add_argument('--bundle',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--scene',action='store_true');parser.add_argument('--render',action='store_true')
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    _,metadata=load_bundle(args.bundle)
    result=dict(model_id=metadata['model_id'],bundle_sha256=metadata['bundle_sha256'],
                reduction=verify_reduction(args.bundle),hardware_tested=False,default_model_changed=False)
    if args.scene:
        result['scene']=verify_scene(args.bundle)
        write_json(out/'scene.json',result['scene'])
    if args.render:result['render']=render_bundle(args.bundle,out,result.get('scene'))
    result['passed']=all(value['passed'] for value in result.values() if isinstance(value,dict) and 'passed' in value)
    write_json(out/'verification.json',result)
    print('N2 verification:',result['passed'],str(out))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
