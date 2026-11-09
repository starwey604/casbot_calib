#!/usr/bin/env python3
"""N3 source integration probe; synthetic serialization, not identification."""
import argparse
from pathlib import Path
import json
import os
import numpy as np
from casbot_calib import _core
from casbot_calib.model_context import load_context
from casbot_calib.dual_model import physical_model
from casbot_calib.kinematics import nominal_kinematics
from casbot_calib.kinematic_parameters import KinematicParameters
from casbot_calib.kinematic_export import export_urdf
from casbot_calib.dynamic_export import write_inertial_urdf,check_reload
from casbot_calib.vendor_viewer import scene_for_vendor
from casbot_calib.telemetry import StateFrame


def verify(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    context=load_context()
    from casbot_calib.dual_model import nominal_dual_arm
    engine,default_source=nominal_dual_arm();home=context.home
    if default_source['model_context']!=context.identity:raise ValueError('Default model is not the shared DVT2 context')
    config=_core.ArmSimulationConfig()
    config.gain=np.full(14,2.);config.torque_limit=100.
    config.current_tau=0.;config.command_delay_ns=0
    config.position_noise=0.;config.current_noise=0.
    commands=np.zeros((1,14,5));commands[0,:,0]=home;commands[0,:,2]=engine.gravity(home)
    observations,_=_core.ArmSimulationRunner(engine,config).run(np.array([0],dtype=np.int64),commands,10_000_000,home,np.zeros(14))
    hold_error=float(np.max(abs(observations['q']-home)))
    np.savez(output/'mock_observations.npz',**observations)
    nominal,_=nominal_kinematics();chart=KinematicParameters(nominal)
    candidate=chart.build(np.linspace(-.15,.15,chart.size))
    kinematics=export_urdf(nominal,candidate,context.kinematic_config(),output/'kinematic_candidate.urdf')
    loaded=_core.ArmDynamics.from_urdf(str(output/'kinematic_candidate.urdf'))
    all_frame_error=max(float(np.max(abs(loaded.frame_pose(home,n)-candidate.frame_pose(home,n)))) for n in candidate.frame_names())
    dynamics=physical_model(engine,np.linspace(.9,1.1,14),.02,.03)
    write_inertial_urdf(context.urdf,dynamics,output/'dynamic_candidate.urdf')
    reload=check_reload(output/'dynamic_candidate.urdf',dynamics,context.urdf)
    # The same model identity/order enters the transport-facing viewer adapter.
    scene=output/'scene.xml'
    adapter,_=scene_for_vendor(dict(model_id=context.model_id,joint_names=list(context.joint_names)),scene)
    os.environ.setdefault('MUJOCO_GL','egl')
    import mujoco
    from PIL import Image
    from casbot_calib.viewer import JointView
    forbidden=('mj_step','mj_forward','mj_inverse','mj_collision','mj_rne')
    original={name:getattr(mujoco,name) for name in forbidden}
    def blocked(*args,**kwargs):raise AssertionError('MuJoCo physics is forbidden')
    try:
        for name in forbidden:setattr(mujoco,name,blocked)
        row=-1;frame=StateFrame(context.model_id,'N3原生Mock',0,int(observations['sample_ns'][row]),
            int(observations['receive_ns'][row]),context.joint_names,observations['q'][row],observations['dq'][row],
            observations['current'][row],observations['q'][row])
        with JointView(adapter,scene,width=960,height=720) as view:
            view.camera.lookat[:]=[0,0,.9];view.camera.distance=3.0
            view.camera.azimuth=138;view.camera.elevation=-14
            pixels=view.render(frame);Image.fromarray(pixels).save(output/'native_mock_view.png')
            frame_error=0.
            expected_poses=engine.fk(frame.q)
            for index,joint in enumerate(engine.model.joints):
                joint_id=mujoco.mj_name2id(view.model,mujoco.mjtObj.mjOBJ_JOINT,joint.name)
                if joint_id<0:raise ValueError('Missing viewer joint: '+joint.name)
                body=int(view.model.jnt_bodyid[joint_id])
                expected=expected_poses[index]
                frame_error=max(frame_error,float(np.max(abs(view.data.xpos[body]-expected[:3,3]))),
                                float(np.max(abs(view.data.xmat[body].reshape(3,3)-expected[:3,:3]))))
            acceleration=float(np.max(abs(view.data.qacc)))
    finally:
        for name,value in original.items():setattr(mujoco,name,value)
    checks=dict(gravity_hold=hold_error<1e-12,kinematic_export=kinematics['passed'],
        all_native_frames=all_frame_error<1e-11,dynamic_reload=reload['passed'],
        viewer_native_fk=frame_error<1e-12,no_viewer_dynamics=acceleration==0.)
    report=dict(passed=all(checks.values()),checks=checks,model_context=context.identity,
        transport_parameters=context.transport_parameters(),frame_count=len(engine.frame_names()),chart_parameters=chart.size,
        mock_hold_duration_ns=10_000_000,mock_hold_error_rad=hold_error,kinematic_frame_error=all_frame_error,
        viewer_frame_error=frame_error,viewer_acceleration=acceleration,dynamic_reload=reload,kinematic_export=kinematics,
        scope='synthetic source/serialization/native Mock/viewer integration; not a new identification experiment',
        default_model_changed=True,hardware_executable=False)
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if not report['passed']:raise ValueError('N3 source integration rejected; see report.json')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    args=parser.parse_args();report=verify(args.output)
    print(json.dumps(dict(passed=report['passed'],checks=report['checks']),ensure_ascii=False))
