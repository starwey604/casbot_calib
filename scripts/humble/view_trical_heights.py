#!/usr/bin/env python3
"""Compare two locked-height reference holds in the shared read-only viewer."""
import argparse
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from casbot_calib.dvt2_bundle import read,load_bundle,viewer_adapter
from casbot_calib.trical_assembly import ball_center
from casbot_calib.trical_assembly_view import relocated_views
from casbot_calib.telemetry import StateFrame
from casbot_calib.viewer import JointView
from casbot_calib.presentation import _font
from casbot_calib.data import write_json


def render(bundle,plans,output):
    import mujoco
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    engine,meta=load_bundle(bundle);adapter=viewer_adapter(bundle);names=tuple(meta['joint_names'])
    canvas=Image.new('RGB',(1600,850),(12,21,33));draw=ImageDraw.Draw(canvas);checks=[]
    draw.text((28,20),'1105_03 / 平台锁定高度对比',font=_font(30),fill=(232,239,247))
    draw.text((28,68),'同一个左臂 A 球测量参考 · 机器人与底座不移动 · 每批运行前调整并锁紧平台',font=_font(19),fill=(63,202,211))
    forbidden=('mj_step','mj_forward','mj_inverse','mj_collision','mj_rne');original={x:getattr(mujoco,x) for x in forbidden}
    def reject(*a,**k):raise RuntimeError('Viewer mechanics call forbidden')
    try:
        for name in forbidden:setattr(mujoco,name,reject)
        for i,plan in enumerate(map(Path,plans)):
            report=read(plan/'report.json')
            if 'left_A' not in report['admitted']:raise ValueError('Comparison requires certified left_A reference')
            layout=read(plan/'protocol.json').get('fixture_layout');scene=relocated_views(bundle,out/f'scenes_{i}',layout)
            motion=read(plan/'left_A/motion.json');index=motion['labels'].index('contact_hold')+1
            q=np.array(motion['states'][index][0])
            frame=StateFrame(engine.model.model_id,'planned reference, not acquisition',i,0,0,names,q,None,None,None,clock_domain='virtual_reference')
            with JointView(adapter,scene/'scene_left.xml',width=790,height=620) as view:
                view.camera.lookat[:]=[.14,0,1.07];view.camera.distance=1.7;view.camera.azimuth=130;view.camera.elevation=-15
                rgb=view.render(frame);canvas.paste(Image.fromarray(rgb),(i*800,160))
                site=mujoco.mj_name2id(view.model,mujoco.mjtObj.mjOBJ_SITE,'left_trical_tcp');expected=engine.frame_pose(q,'left_trical_tcp')
                error=float(np.max(abs(view.data.site_xpos[site]-expected[:3,3])))
                rotation_error=float(np.max(abs(view.data.site_xmat[site].reshape(3,3)-expected[:3,:3])))
                ball_errors=[]
                for bid in 'ABCD':
                    g=mujoco.mj_name2id(view.model,mujoco.mjtObj.mjOBJ_GEOM,'datum_'+bid)
                    ball_errors.append(float(np.max(abs(view.data.geom_xpos[g]-ball_center(bid,layout)))))
                checks.append(dict(plan=str(plan),TCP_error_m=error,rotation_error=rotation_error,ball_position_error_m=max(ball_errors),qacc_zero=not np.any(view.data.qacc)))
            lift=0 if layout is None else layout.get('platform_lift_m',0)*1000
            draw.text((28+800*i,119),f'相对 CAD 高度 {lift:+.0f} mm / 连续门禁通过 {len(report["admitted"])}/8',font=_font(22),fill=(232,239,247))
        draw.text((28,800),'MuJoCo 仅渲染；图中为规划姿态。平台实际升降行程、锁紧和调整过程尚未验收。',font=_font(19),fill=(255,192,105))
        canvas.save(out/'platform_height_comparison_zh.png')
    finally:
        for name,value in original.items():setattr(mujoco,name,value)
    result=dict(passed=all(r['TCP_error_m']<1e-12 and r['rotation_error']<1e-12 and r['ball_position_error_m']<1e-12 and r['qacc_zero'] for r in checks),checks=checks,mechanics_calls_forbidden=True)
    write_json(out/'verification.json',result);return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',required=True);p.add_argument('--plans',nargs=2,required=True);p.add_argument('--output',required=True);a=p.parse_args()
    print(render(a.bundle,a.plans,a.output))
