#!/usr/bin/env python3
"""Existing C++ native Mock -> real ROS -> common geometry viewer, local only."""
import argparse
from importlib.resources import files
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import numpy as np
from casbot_calib.vendor_data import load_config
from casbot_calib.ros_state import RosStateSource, MESSAGE_TYPES
from casbot_calib.vendor_viewer import present_pose_source
from casbot_calib.dual_model import nominal_dual_arm


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True); args=parser.parse_args()
    root=args.output.resolve(); root.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_LOCALHOST_ONLY']='1'; os.environ['ROS_DOMAIN_ID']=str(150+int(uuid.uuid4().hex[:2],16)%25)
    namespace='/casbot_calib_bench/view_'+uuid.uuid4().hex[:10]
    _, identity=nominal_dual_arm(); model_id='casbot_20251015_nominal:'+identity['sha256'][:12]
    path=Path(str(files('casbot_calib').joinpath('assets/casbot_dual_arm_benchmark_nominal.urdf')))
    prefix=Path('install/humble-ros/casbot_calib_transport/lib/casbot_calib_transport').resolve()
    t=np.arange(0,2_000_000_000,2_000_000,dtype=np.int64)
    home=np.tile([0.,.4,0.,-.8,0.,0.,0.],2); home[8]=-.4
    phase=t[:,None]*1e-9*np.pi
    q=home+.035*np.sin(phase)*np.ones((1,14)); dq=.035*np.pi*np.cos(phase)*np.ones((1,14))
    zeros=np.zeros_like(q)
    plan=np.column_stack([t,np.ones(len(t)),q,dq,zeros,np.ones_like(q),np.full_like(q,.05)])
    np.savetxt(root/'plan.csv',plan,delimiter=',',header='relative_ns,mode'+''.join(',x'+str(i) for i in range(70)),
               comments='',fmt=['%d','%d']+['%.17g']*70)
    common=['--ros-args','-r','__ns:='+namespace,'-p','experiment_id:=viewer_native_mock','-p','model_id:='+model_id,
            '-p','model_path:='+str(path),'-p','output_dir:='+str(root)]
    host_args=[str(prefix/'mock_host'),*common,'-p','control_hz:=500','-p','feedback_hz:=500',
               '-p','initial_q:='+json.dumps(home.tolist())]
    client_args=[str(prefix/'stream_client'),*common,'-p','plan_file:='+str(root/'plan.csv'),'-p','command_hz:=500']
    processes=[]; logs=[]
    source=RosStateSource(load_config('configs/vendor_s1.yaml'),namespace+'/observations',MESSAGE_TYPES[2],
                          source_model_id=model_id,scope='localhost_ros_mock')
    try:
        for name,command in [('host',host_args),('client',client_args)]:
            log=(root/(name+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT))
        result=present_pose_source(source,root/'presentation',live=True,duration_s=3.,fps=12)
        code=processes[1].wait(timeout=10)
        if code or processes[0].poll() is not None: raise RuntimeError('Native Mock endpoint failed; inspect logs')
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
        for log in logs:log.close()
    try:
        time.sleep(.3); stale=source.is_stale()
        samples=np.genfromtxt(root/'host_samples.csv',delimiter=',',names=True)
        frames=json.loads((root/'presentation/displayed_frames.json').read_text())
        index={int(row['sample_ns']):row for row in samples}
        errors=[]
        for frame in frames:
            row=index[frame['sample_ns']]
            errors.append(float(np.max(abs(np.asarray(frame['q'])-np.array([row['q'+str(j)] for j in range(14)])))))
        result=dict(passed=bool(result['passed'] and errors and max(errors)<1e-14 and stale and source.received>100),
                    scope='existing native Mock/Pinocchio/Boost -> ROS JointObservation -> independent geometry viewer',
                    source_model_id=model_id, namespace=namespace, ros_domain_id=int(os.environ['ROS_DOMAIN_ID']),
                    received=source.received,rejected=source.rejected,stale_after_mock_exit=stale,
                    maximum_wire_pose_error=max(errors) if errors else None,
                    maximum_fk_error=result['max_fk_error'],hardware_tested=False,physical_network_tested=False,
                    no_new_physics=True,commands_scope='private native Mock only; viewer never publishes commands')
        (root/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2)); return 0 if result['passed'] else 1
    finally: source.close()


if __name__=='__main__':raise SystemExit(main())
