from __future__ import annotations
import argparse
from datetime import datetime, timezone
from pathlib import Path
import platform
import sys
import numpy as np
import scipy
import yaml
from .data import write_json, load_observations
from .identification import Prior, identify
from .experiments import run_stage

def main(argv=None):
    parser=argparse.ArgumentParser(description="CASBOT calibration: synthetic experiments and explicit read-only vendor capture")
    sub=parser.add_subparsers(dest="command",required=True)
    batch_plan=sub.add_parser('hardware-plan-batch',help='Freeze H2 direction/frequency/static jobs without device I/O')
    batch_plan.add_argument('--config',type=Path,default=Path('configs/hardware_batch.yaml'))
    batch_plan.add_argument('--profile',type=Path,default=Path('configs/hardware_field.yaml'));batch_plan.add_argument('--preset',type=Path)
    batch_plan.add_argument('--domain-id',type=int,required=True);batch_plan.add_argument('--output',type=Path,required=True)
    batch_run=sub.add_parser('hardware-run-batch',help='ACTUAL SDK batch execution; terminal faults never resume')
    batch_run.add_argument('folder',type=Path);batch_run.add_argument('--resume',action='store_true');batch_run.add_argument('--max-jobs',type=int)
    direction_template=sub.add_parser('hardware-direction-template',help='Export human physical-direction review template bound to observations')
    direction_template.add_argument('folder',type=Path);direction_template.add_argument('--output',type=Path,required=True)
    direction_review=sub.add_parser('hardware-direction-review',help='Import human direction observations; no device writes')
    direction_review.add_argument('folder',type=Path);direction_review.add_argument('--file',type=Path,required=True)
    batch_analyse=sub.add_parser('hardware-analyse-batch',help='Read-only H2 reports and accepted static-pair gravity fitting')
    batch_analyse.add_argument('folder',type=Path);batch_analyse.add_argument('--output',type=Path,required=True)
    batch_render=sub.add_parser('hardware-render-batch',help='Read-only geometry replay of sealed batch attempts')
    batch_render.add_argument('folder',type=Path);batch_render.add_argument('--output',type=Path,required=True)
    batch_render.add_argument('--fps',type=int,default=12);batch_render.add_argument('--max-seconds-per-attempt',type=float,default=4.)
    field_snapshot=sub.add_parser('hardware-snapshot',help='Read actual SDK start; no command publisher or mode request')
    field_snapshot.add_argument('--config',type=Path,default=Path('configs/hardware_field.yaml'))
    field_snapshot.add_argument('--output',type=Path,required=True);field_snapshot.add_argument('--domain-id',type=int,required=True)
    field_jog=sub.add_parser('hardware-plan-jog',help='Plan a small single-joint direction jog from measured start; no commands')
    field_jog.add_argument('--config',type=Path,default=Path('configs/hardware_field.yaml'))
    field_jog.add_argument('--snapshot',type=Path,required=True);field_jog.add_argument('--joint',required=True)
    field_jog.add_argument('--amplitude-deg',type=float,default=1.);field_jog.add_argument('--output',type=Path,required=True)
    field_plan=sub.add_parser('hardware-plan-static',help='Build static sequence from actual snapshot and selected preset; no commands')
    field_plan.add_argument('preset',type=Path);field_plan.add_argument('--snapshot',type=Path,required=True)
    field_plan.add_argument('--config',type=Path,default=Path('configs/hardware_field.yaml'))
    field_plan.add_argument('--output',type=Path,required=True);field_plan.add_argument('--max-poses-per-split',type=int)
    field_execute=sub.add_parser('hardware-execute',aliases=['hardware-execute-static'],help='ACTUAL SDK position commands; recorded assumptions and device disconnect protection')
    field_execute.add_argument('plan',type=Path);field_execute.add_argument('--output',type=Path,required=True)
    field_execute.add_argument('--domain-id',type=int,required=True)
    field_analysis=sub.add_parser('hardware-analyse',aliases=['hardware-analyse-static'],help='Position diagnostics or exploratory gravity fit; publication timestamps remain unqualified')
    field_analysis.add_argument('plan',type=Path);field_analysis.add_argument('--capture',type=Path,required=True)
    field_analysis.add_argument('--output',type=Path,required=True)
    static_plan=sub.add_parser('plan-static-field',help='Offline static field preset: signed scans, combination poses, collision and gravity information')
    static_plan.add_argument('--config',type=Path,default=Path('configs/static_field_nominal.yaml'))
    static_plan.add_argument('--output',type=Path,required=True)
    static_verify=sub.add_parser('verify-static-field',help='Recheck frozen static field references without device I/O or measurements')
    static_verify.add_argument('folder',type=Path);static_verify.add_argument('--output',type=Path,required=True)
    dynamics_export=sub.add_parser('export-dynamics-urdf',help='Export an accepted S4 gravity/inertia candidate and verify a Pinocchio URDF reload')
    dynamics_export.add_argument('analysis',type=Path);dynamics_export.add_argument('--stage',choices=('gravity','inertia'),default='inertia')
    dynamics_export.add_argument('--output',type=Path,required=True)
    dynamics_verify=sub.add_parser('verify-dynamics-urdf',help='Recheck an exported inertial URDF and runtime sidecar without fitting or truth')
    dynamics_verify.add_argument('folder',type=Path);dynamics_verify.add_argument('--output',type=Path,required=True)
    vendor_capture=sub.add_parser('vendor-capture',help='S1 subscription-only SDK state recording; no motion or mode changes')
    vendor_capture.add_argument('--config',type=Path,default=Path('configs/vendor_s1.yaml'))
    vendor_capture.add_argument('--output',type=Path,required=True);vendor_capture.add_argument('--duration',type=float,default=10.)
    vendor_capture.add_argument('--domain-id',type=int,required=True)
    vendor_capture.add_argument('--scope',choices=('localhost_contract_mock','hardware_read_only'),required=True)
    vendor_normalize=sub.add_parser('vendor-normalize',help='S1 offline normalization preserving unavailable fields and unknown clocks')
    vendor_normalize.add_argument('folder',type=Path);vendor_normalize.add_argument('--output',type=Path,required=True)
    vendor_view=sub.add_parser('vendor-view',help='S1 read-only recorded or live file-tail poses; no device I/O')
    vendor_view.add_argument('folder',type=Path);vendor_view.add_argument('--output',type=Path,required=True)
    vendor_view.add_argument('--live',action='store_true');vendor_view.add_argument('--window',action='store_true')
    vendor_view.add_argument('--fps',type=int,default=12)
    ros_view=sub.add_parser('ros-view',help='Unified subscription-only viewer for native ROS Mock or vendor feedback')
    ros_view.add_argument('--config',type=Path,default=Path('configs/vendor_s1.yaml'))
    ros_view.add_argument('--output',type=Path,required=True);ros_view.add_argument('--topic',required=True)
    ros_view.add_argument('--message-type',choices=('sensor_msgs/msg/JointState','crb_ros_msg/msg/JointStateData','casbot_calib_interfaces/msg/JointObservation'),required=True)
    ros_view.add_argument('--source-model-id');ros_view.add_argument('--domain-id',type=int,required=True)
    ros_view.add_argument('--scope',choices=('localhost_ros_mock','hardware_read_only'),required=True)
    ros_view.add_argument('--window',action='store_true');ros_view.add_argument('--duration',type=float,default=5.)
    ros_view.add_argument('--fps',type=int,default=12)
    pos_plan=sub.add_parser('vendor-plan-position',help='S2 observed-start snapshot and collision-gated local SDK position plan')
    pos_plan.add_argument('--config',type=Path,default=Path('configs/vendor_s2.yaml'));pos_plan.add_argument('--output',type=Path,required=True)
    pos_plan.add_argument('--endpoint-root',required=True);pos_plan.add_argument('--domain-id',type=int,required=True)
    pos_run=sub.add_parser('vendor-execute-position',help='S2 isolated local SDK position execution; hardware remains blocked')
    pos_run.add_argument('plan',type=Path);pos_run.add_argument('--output',type=Path,required=True)
    pos_analyse=sub.add_parser('vendor-analyse-position',help='S2 read-only position frequency and overall response analysis')
    pos_analyse.add_argument('folder',type=Path);pos_analyse.add_argument('--output',type=Path,required=True)
    dynamics_import=sub.add_parser('vendor-import-dynamics',help='S3 immutable ROS/host-log import with explicit torque/clock provenance')
    dynamics_import.add_argument('source',type=Path);dynamics_import.add_argument('--source-type',choices=('vendor','host'),required=True)
    dynamics_import.add_argument('--profile',type=Path,default=Path('configs/vendor_s3_unknown.yaml'));dynamics_import.add_argument('--output',type=Path,required=True)
    dynamics_qualify=sub.add_parser('vendor-qualify-dynamics',help='S3 stage-specific source/scale/time/raw-bandwidth observation gates')
    dynamics_qualify.add_argument('folder',type=Path);dynamics_qualify.add_argument('--output',type=Path,required=True)
    dynamics_problem=sub.add_parser('vendor-dynamics-problem',help='S3 qualified periodic output-torque to Pinocchio rigid-regression boundary; no fitting')
    dynamics_problem.add_argument('qualification',type=Path);dynamics_problem.add_argument('--output',type=Path,required=True)
    staged_vendor_plan=sub.add_parser('vendor-plan-staged',help='S4 freeze qualified observation roles; offline, no motion')
    staged_vendor_plan.add_argument('--spec',type=Path,default=Path('configs/vendor_s4_unknown.yaml'));staged_vendor_plan.add_argument('--output',type=Path,required=True)
    staged_vendor_inspect=sub.add_parser('vendor-inspect-staged',help='S4 inspect frozen data readiness and explicit execution blockers')
    staged_vendor_inspect.add_argument('folder',type=Path);staged_vendor_inspect.add_argument('--output',type=Path,required=True)
    staged_vendor_identify=sub.add_parser('vendor-identify-staged',help='S4 sequential qualified torque fits and validation; no device I/O')
    staged_vendor_identify.add_argument('folder',type=Path);staged_vendor_identify.add_argument('--output',type=Path,required=True)
    staged_vendor_identify.add_argument('--through',choices=('gravity','friction','breakaway','inertia'),default='inertia')
    kinematics=sub.add_parser('kinematics-check',help='K1 named dual-arm TCP and TriCal schema checks; no motion or calibration')
    kinematics.add_argument('--config',type=Path)
    kinematics.add_argument('--output',required=True,type=Path)
    trical=sub.add_parser('trical-mock',help='K2 dual-arm indicator Mock, collision-gated autocentering and static measurements')
    trical.add_argument('--config',type=Path);trical.add_argument('--output',required=True,type=Path)
    trical.add_argument('--first-only',action='store_true')
    trical.add_argument('--fault',choices=('disconnected','frozen','out_of_range','no_contact'))
    trical_suite=sub.add_parser('trical-suite',help='K2 dual-arm baseline, instrument failures, collision rejection and truth isolation')
    trical_suite.add_argument('--config',type=Path);trical_suite.add_argument('--output',required=True,type=Path)
    trical_analysis=sub.add_parser('trical-analyse',help='Recheck K2 raw measurements and collision gates without evaluation truth')
    trical_analysis.add_argument('folder',type=Path);trical_analysis.add_argument('--output',required=True,type=Path)
    kin_plan=sub.add_parser('plan-kinematics',help='K3 collision-gated posture pools and training-only observability design')
    kin_plan.add_argument('--config',type=Path);kin_plan.add_argument('--output',required=True,type=Path)
    kin_collect=sub.add_parser('kinematic-calibrate',help='Collect independent K3 batches from a frozen plan and identify geometry')
    kin_collect.add_argument('--plan',required=True,type=Path);kin_collect.add_argument('--output',required=True,type=Path)
    kin_identify=sub.add_parser('kinematic-identify',help='Identify MDH/Hayati geometry from raw measured sessions without truth')
    kin_identify.add_argument('--train',required=True,type=Path);kin_identify.add_argument('--validation',required=True,type=Path)
    kin_identify.add_argument('--output',required=True,type=Path)
    kin_suite=sub.add_parser('kinematic-suite',help='K3 planning, Mock acquisition, truth-isolated identification and report')
    kin_suite.add_argument('--config',type=Path);kin_suite.add_argument('--output',required=True,type=Path)
    kin_accept=sub.add_parser('kinematic-acceptance',help='K4 three independent calibrations, repeatability and Chinese viewer playback')
    kin_accept.add_argument('--reference',type=Path,required=True);kin_accept.add_argument('--output',type=Path,required=True)
    kin_accept.add_argument('--fps',type=int,default=12);kin_accept.add_argument('--snapshots-only',action='store_true')
    kin_accept.add_argument('--external-meter',type=Path)
    kin_repeat=sub.add_parser('kinematic-repeat',help='Collect one preregistered K4 repeat batch')
    kin_repeat.add_argument('folder',type=Path);kin_repeat.add_argument('--index',type=int,choices=(1,2),required=True)
    kin_gates=sub.add_parser('kinematic-reference-gates',help='Recheck reference raw stability/motion under K4')
    kin_gates.add_argument('folder',type=Path)
    kin_verify=sub.add_parser('kinematic-verify',help='Observation-only K4 repeated calibration acceptance')
    kin_verify.add_argument('folder',type=Path);kin_verify.add_argument('--output',type=Path,required=True);kin_verify.add_argument('--external-meter',type=Path)
    kin_render=sub.add_parser('render-kinematics',help='Chinese read-only TriCal acquisition and calibration playback')
    kin_render.add_argument('folder',type=Path);kin_render.add_argument('--analysis',type=Path);kin_render.add_argument('--output',type=Path,required=True)
    kin_render.add_argument('--fps',type=int,default=12);kin_render.add_argument('--snapshots-only',action='store_true')
    run=sub.add_parser("run",help="Run one independent A-F protocol")
    run.add_argument("config",type=Path);run.add_argument("--output",required=True,type=Path)
    suite=sub.add_parser("suite",help="Run all protocols and acceptance checks")
    suite.add_argument("--configs",type=Path,default=Path("configs"));suite.add_argument("--output",required=True,type=Path)
    staged=sub.add_parser('plan-staged',help='Legacy collision plan, P0 R1 split references, or --contract-only R0 identities')
    staged.add_argument('--config',type=Path,default=Path('configs/staged_dual_arm.yaml'));staged.add_argument('--output',type=Path,required=True)
    staged.add_argument('--development-reference',type=Path,help='P0 R1: pin an existing legacy session as development predecessor observations')
    staged.add_argument('--contract-only',action='store_true',help='P0: only freeze the R0 identity contract, without R1 references')
    staged_run=sub.add_parser('staged-calibrate',help='Acquire staged Mock calibration and independently validate four control groups')
    staged_run.add_argument('--config',type=Path,default=Path('configs/staged_dual_arm.yaml'));staged_run.add_argument('--output',type=Path,required=True)
    staged_run.add_argument('--development-reference',type=Path,help='P0 R1: explicit legacy predecessor development session')
    staged_run.add_argument('--final-test',type=Path,help='R4: parent P0 session containing an R3 candidate freeze')
    staged_run.add_argument('--attribution-analysis',type=Path,help='R4: immutable R3 analysis matching the frozen selection proof')
    staged_run.add_argument('--fps',type=int,default=12,help='R4 Chinese recording playback frame rate')
    staged_run.add_argument('--through',choices=('gravity','friction','breakaway','inertia','all'),default='all');staged_run.add_argument('--resume',action='store_true')
    staged_analysis=sub.add_parser('staged-identify',help='Reanalyse staged raw observations without plant configuration or evaluation truth')
    staged_analysis.add_argument('folder',type=Path);staged_analysis.add_argument('--output',type=Path,required=True)
    staged_analysis.add_argument('--through',choices=('gravity','friction','breakaway','inertia','stability','attribution','final'),default='inertia')
    staged_analysis.add_argument('--final-protocol',type=Path,help='R4: preregistered final analysis directory; no fits or device access')
    staged_analysis.add_argument('--stability-verification',type=Path,help='R3: independently verified R2 directory')
    staged_analysis.add_argument('--resume',action='store_true',help='Continue only an unchanged R2 stability analysis')
    staged_verify=sub.add_parser('staged-verify',help='Recheck all collision gates, numerical convergence and truth-isolated inference')
    staged_verify.add_argument('folder',type=Path);staged_verify.add_argument('--output',type=Path,required=True)
    staged_verify.add_argument('--stability-analysis',type=Path,help='R2: verify sealed sensitivity fits and compare the regularization path')
    staged_verify.add_argument('--attribution-analysis',type=Path,help='R3: verify matched-band component/selection evidence')
    staged_verify.add_argument('--final-analysis',type=Path,help='R4: accepted frozen test analysis for comprehensive verification')
    staged_verify.add_argument('--freeze-candidate',action='store_true',help='R3: freeze the qualified selection winner after verification')
    staged_render=sub.add_parser('render-staged',help='Chinese geometry-only playback of accepted staged records')
    staged_render.add_argument('folder',type=Path);staged_render.add_argument('--output',type=Path)
    staged_render.add_argument('--fps',type=int,default=12);staged_render.add_argument('--snapshots-only',action='store_true')
    staged_render.add_argument('--ros-report',type=Path,help='Optional separately captured local ROS analysis JSON')
    staged_suite=sub.add_parser('staged-suite',help='Calibration, comprehensive verification and Chinese viewer delivery')
    staged_suite.add_argument('--config',type=Path,default=Path('configs/staged_dual_arm.yaml'))
    staged_suite.add_argument('--output',type=Path,required=True);staged_suite.add_argument('--resume',action='store_true')
    staged_suite.add_argument('--fps',type=int,default=12)
    staged_suite.add_argument('--ros-report',type=Path)
    transport=sub.add_parser('ros-benchmark',help='Local real ROS streaming / timing / frequency Mock (no hardware)')
    transport.add_argument('--config',type=Path,default=Path('configs/ros_transport.yaml'));transport.add_argument('--output',type=Path,required=True)
    transport_analysis=sub.add_parser('ros-analyse',help='Re-analyse ROS stream records without running nodes or reading Mock truth')
    transport_analysis.add_argument('folder',type=Path);transport_analysis.add_argument('--output',type=Path,required=True)
    estimate=sub.add_parser("identify",help="Re-estimate from observations only; no evaluation truth required")
    estimate.add_argument("run_folder",type=Path);estimate.add_argument("--config",type=Path,required=True)
    estimate.add_argument("--output",type=Path,required=True)
    demo=sub.add_parser("demo",help="Collect the calibration workflow, then export the read-only viewer video")
    demo.add_argument("--output",required=True,type=Path)
    demo.add_argument("--collect-only",action="store_true",help="Run the workflow without importing MuJoCo")
    demo.add_argument("--config",type=Path,help="Optional workflow YAML: plant, identification, repeat_noise, acceptance")
    demo.add_argument("--fps",type=int,default=24)
    render=sub.add_parser("render-demo",help="Render an existing workflow without rerunning physics or identification")
    render.add_argument("folder",type=Path);render.add_argument("--output",type=Path)
    render.add_argument("--fps",type=int,default=24)
    render.add_argument("--width",type=int,default=1280);render.add_argument("--height",type=int,default=720)
    render.add_argument("--clip-seconds",type=float,help="Override each clip duration for a short preview")
    render.add_argument("--snapshots-only",action="store_true")
    arm=sub.add_parser("arm-check",help="R1 Pinocchio rigid-arm correctness report (no acquisition)")
    arm.add_argument("--output",required=True,type=Path)
    arm.add_argument("--render",action="store_true",help="Optional geometry-only Chinese snapshot")
    dual=sub.add_parser("dual-calibrate",help="Nominal fixed-waist dual-arm mock workflow; no hardware")
    dual.add_argument("--config",type=Path,default=Path("configs/dual_arm_dynamics.yaml"))
    dual.add_argument("--output",required=True,type=Path)
    dual.add_argument("--render",action="store_true",help="Optional Chinese geometry-only recording playback")
    dual_estimate=sub.add_parser("dual-identify",help="Re-identify saved dual-arm observations without evaluation truth")
    dual_estimate.add_argument("folder",type=Path);dual_estimate.add_argument("--output",required=True,type=Path)
    dual_render=sub.add_parser("render-dual",help="Render an existing dual-arm workflow; no simulation")
    dual_render.add_argument("folder",type=Path);dual_render.add_argument("--output",type=Path)
    dual_render.add_argument("--fps",type=int,default=12);dual_render.add_argument("--snapshots-only",action="store_true")
    view=sub.add_parser("view",help="Read-only interactive joint viewer (recording or external JSONL telemetry)")
    source=view.add_mutually_exclusive_group(required=True)
    source.add_argument("--run-folder",type=Path);source.add_argument("--telemetry-jsonl",type=Path)
    view.add_argument("--mapping",type=Path);view.add_argument("--scene",type=Path)
    view.add_argument("--follow",action="store_true",help="Follow an externally appended JSONL file")
    args=parser.parse_args(argv)
    if args.command in ('hardware-plan-batch','hardware-run-batch','hardware-direction-template','hardware-direction-review','hardware-analyse-batch','hardware-render-batch'):
        from .hardware_batch import plan_batch,run_batch,write_direction_template,import_direction_review,analyse_batch
        if args.command=='hardware-plan-batch':result=plan_batch(args.config,args.profile,args.output,preset=args.preset,domain_id=args.domain_id)
        elif args.command=='hardware-run-batch':result=run_batch(args.folder,resume=args.resume,max_jobs=args.max_jobs)
        elif args.command=='hardware-direction-template':result=write_direction_template(args.folder,args.output)
        elif args.command=='hardware-direction-review':result=import_direction_review(args.folder,args.file)
        elif args.command=='hardware-render-batch':
            from .hardware_batch_viewer import render_batch
            result=render_batch(args.folder,args.output,fps=args.fps,max_seconds_per_attempt=args.max_seconds_per_attempt)
        else:result=analyse_batch(args.folder,args.output)
        print(f"现场批次状态：{result.get('state','完成')}；正式标定资格不由采集状态替代")
        return 0 if result.get('passed') or result.get('state') in ('planned','checkpoint','awaiting_direction_review','direction_review_recorded','interrupted_at_checkpoint','interrupted') else 1
    if args.command.startswith('hardware-'):
        from .hardware_field import snapshot_field,prepare_jog,prepare_field,execute_field,analyse_field
        if args.command=='hardware-snapshot':result=snapshot_field(args.config,args.output,args.domain_id)
        elif args.command=='hardware-plan-jog':result=prepare_jog(args.config,args.snapshot,args.output,args.joint,args.amplitude_deg)
        elif args.command=='hardware-plan-static':result=prepare_field(args.config,args.preset,args.snapshot,args.output,max_poses_per_split=args.max_poses_per_split)
        elif args.command in ('hardware-execute','hardware-execute-static'):result=execute_field(args.plan,args.output,args.domain_id)
        else:result=analyse_field(args.plan,args.capture,args.output)
        print(f"现场路径{'完成' if result['passed'] else '失败'}；查看假设与实测状态 -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command in ('plan-static-field','verify-static-field'):
        from .static_field import plan_static_field,verify_static_field
        result=plan_static_field(args.config,args.output) if args.command=='plan-static-field' else verify_static_field(args.folder,args.output)
        print(f"静态参考计划{'通过' if result['passed'] else '未通过'}；实机执行未开放 -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command in ('export-dynamics-urdf','verify-dynamics-urdf'):
        from .dynamic_export import export_dynamics_urdf,verify_dynamics_export
        result=export_dynamics_urdf(args.analysis,args.output,args.stage) if args.command=='export-dynamics-urdf' else verify_dynamics_export(args.folder,args.output)
        print(f"动力学URDF重加载{'通过' if result['passed'] else '失败'}；仅验证模型序列化一致性，实机执行未开放 -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command in ('vendor-plan-staged','vendor-inspect-staged','vendor-identify-staged'):
        from .vendor_staged import freeze_staged,inspect_staged,analyse_staged
        if args.command=='vendor-plan-staged':result=freeze_staged(args.spec,args.output)
        elif args.command=='vendor-inspect-staged':result=inspect_staged(args.folder,args.output)
        else:result=analyse_staged(args.folder,args.output,args.through)
        print(f"S4 {'通过' if result['passed'] else '阻断'}；离线数据流程，实机执行未开放 -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=='vendor-import-dynamics':
        from .dynamics_data import import_dynamics
        import_dynamics(args.source,args.profile,args.output,source_type=args.source_type)
        print(f'动力学观测已导入，尚需逐阶段资格检查 -> {args.output}');return 0
    if args.command=='vendor-qualify-dynamics':
        from .dynamics_qualification import qualify_dynamics
        result=qualify_dynamics(args.folder,args.output)
        eligible=[k for k,v in result['stage_data_eligibility'].items() if v['observations_eligible']]
        print(f"观测资格：{','.join(eligible) or '全部阻断'}；硬件执行仍关闭 -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=='vendor-dynamics-problem':
        from .dynamics_problem import build_periodic_problem
        result=build_periodic_problem(args.qualification,args.output)
        print(f"Pinocchio观测方程：秩{result['rank']}/140；尚未拟合或验收参数 -> {args.output}");return 0
    if args.command=='vendor-plan-position':
        from .vendor_position import prepare_position
        prepare_position(args.config,args.output,endpoint_root=args.endpoint_root,domain_id=args.domain_id)
        print(f'SDK位置计划：已读取起始姿态并通过参考门禁 -> {args.output}');return 0
    if args.command=='vendor-execute-position':
        from .vendor_position import execute_position
        result=execute_position(args.plan,args.output)
        print(f"SDK位置执行：{result['reason']} -> {args.output}");return 0 if result['passed'] else 1
    if args.command=='vendor-analyse-position':
        from .vendor_position_analysis import analyse_position
        result=analyse_position(args.folder,args.output)
        print(f"SDK位置分析：{'PASS' if result['passed'] else 'REJECT'} -> {args.output}");return 0 if result['passed'] else 1
    if args.command=='ros-view':
        from .ros_state import view_ros
        result=view_ros(args.config,args.output,topic=args.topic,message_type=args.message_type,domain_id=args.domain_id,
                        scope=args.scope,source_model_id=args.source_model_id,window=args.window,duration_s=args.duration,fps=args.fps)
        return 0 if result.get('passed',True) else 1
    if args.command=='vendor-capture':
        from .vendor_capture import capture
        result=capture(args.config,args.output,duration_s=args.duration,domain_id=args.domain_id,scope=args.scope)
        print(f"SDK只读采集：{'完成' if result['completed'] else '中断或失败，原始记录保留'} -> {args.output}")
        return 0 if result['completed'] else 1
    if args.command=='vendor-normalize':
        from .vendor_data import normalize_session
        result=normalize_session(args.folder,args.output)
        print(f"SDK只读规范化：{result['valid_position_packets']}/{result['packets']}条有效位置；动力学资格未开放 -> {args.output}")
        return 0 if result['position_observations_available'] else 1
    if args.command=='vendor-view':
        from .vendor_viewer import view_vendor
        result=view_vendor(args.folder,args.output,live=args.live,window=args.window,fps=args.fps)
        return 0 if result.get('passed',True) else 1
    if args.command=='kinematic-acceptance':
        from .kinematic_acceptance import run_acceptance
        result=run_acceptance(args.reference,args.output,video=not args.snapshots_only,fps=args.fps,external_meter=args.external_meter)
        print(f"K4合成验收：{'通过' if result['passed'] else '拒绝'} -> {args.output}");return 0 if result['passed'] else 1
    if args.command=='kinematic-repeat':
        from .kinematic_acceptance import capture_repeat
        result=capture_repeat(args.folder,args.index);return 0 if result['passed'] else 1
    if args.command=='kinematic-reference-gates':
        from .kinematic_acceptance import check_reference_gates
        result=check_reference_gates(args.folder);return 0 if all(r['passed'] for r in result.values()) else 1
    if args.command=='kinematic-verify':
        from .kinematic_acceptance import analyse_acceptance
        result=analyse_acceptance(args.folder,args.output,args.external_meter)
        print(f"K4观测复核：{'通过' if result['passed'] else '拒绝'} -> {args.output}");return 0 if result['passed'] else 1
    if args.command=='render-kinematics':
        from .kinematic_presentation import render_kinematic
        result=render_kinematic(args.folder,args.analysis or args.folder/'analysis',args.output,fps=args.fps,video=not args.snapshots_only)
        return 0 if result['passed'] else 1
    if args.command=='plan-kinematics':
        from .kinematic_design import plan,k3_config
        result=plan(args.output,k3_config(args.config));print(f"K3姿态规划：{'通过' if result['passed'] else '拒绝'} -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=='kinematic-calibrate':
        from .kinematic_workflow import collect_and_identify
        result=collect_and_identify(args.plan,args.output);print(f"K3几何标定：{'通过' if result['passed'] else '拒绝'} -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=='kinematic-identify':
        from .kinematic_identification import identify as identify_geometry
        result=identify_geometry(args.train,args.validation,args.output);print(f"K3观测辨识：{'通过' if result['passed'] else '拒绝'} -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=='kinematic-suite':
        from .kinematic_design import k3_config
        from .kinematic_workflow import run_suite
        result=run_suite(args.output,k3_config(args.config));print(f"K3综合实验：{'通过' if result['passed'] else '拒绝'} -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=='trical-suite':
        from .kinematics import workflow_config
        from .trical_suite import run_suite
        result=run_suite(args.output,workflow_config(args.config))
        print(f"K2综合验证：{'通过' if result['passed'] else '失败'} -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=='trical-mock':
        from .kinematics import workflow_config
        from .trical_workflow import run_mock
        result=run_mock(args.output,workflow_config(args.config),fault=args.fault,first_only=args.first_only)
        print(f"K2测量闭环：{'通过' if result['passed'] else '拒绝'} -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=='trical-analyse':
        from .trical_analysis import analyse
        result=analyse(args.folder,args.output)
        print(f"K2无真值复核：{'通过' if result['passed'] else '拒绝'} -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=='kinematics-check':
        from .kinematics import default_config
        from .kinematics_checks import run_checks
        result=run_checks(args.output,default_config(args.config))
        print(f"K1运动学基础检查：{'通过' if result['passed'] else '失败'} -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=='render-staged':
        from .staged_presentation import render_staged
        result=render_staged(args.folder,args.output,fps=args.fps,video=not args.snapshots_only,ros_report=args.ros_report)
        return 0 if result['passed'] else 1
    if args.command=='staged-verify':
        from .staged_validation import verify_staged
        result=verify_staged(args.folder,args.output,args.stability_analysis,args.attribution_analysis,args.freeze_candidate,args.final_analysis)
        print(f"Comprehensive staged verification: {'PASS' if result['passed'] else 'FAIL'} -> {args.output}",flush=True)
        return 0 if result['passed'] else 1
    if args.command=='staged-suite':
        from .staged_delivery import run_suite
        result=run_suite(args.output,yaml.safe_load(args.config.read_text(encoding='utf-8')),args.resume,args.fps,args.ros_report)
        print(f"Staged delivery: {'PASS' if result['passed'] else 'FAIL'} -> {args.output}",flush=True)
        return 0 if result['passed'] else 1
    if args.command=='staged-identify':
        from .staged_analysis import identify_staged
        result=identify_staged(args.folder,args.output,args.through,args.resume,args.stability_verification,args.final_protocol)
        print(f"Staged analysis: {'PASS' if result['passed'] else 'FAIL'} -> {args.output}",flush=True)
        return 0 if result['passed'] else 1
    if args.command=='staged-calibrate':
        if args.final_test is not None:
            if args.attribution_analysis is None or args.development_reference is not None or args.through!='all':raise ValueError('R4 requires --attribution-analysis, --through all and an existing frozen parent')
            from .staged_final import run_final
            result=run_final(args.final_test,args.attribution_analysis,args.output,yaml.safe_load(args.config.read_text(encoding='utf-8')),resume=args.resume,fps=args.fps)
            print(f"R4 final delivery: {'PASS' if result['passed'] else 'FAIL'} -> {args.output}",flush=True)
            return 0 if result['passed'] else 1
        if args.attribution_analysis is not None:raise ValueError('--attribution-analysis requires --final-test')
        from .staged_workflow import run_staged
        result=run_staged(args.output,yaml.safe_load(args.config.read_text(encoding='utf-8')),args.through,args.resume,args.development_reference)
        print(f"Staged checkpoint: {'PASS' if result['checkpoint_passed'] else 'FAIL'} through {result['completed_through']} -> {args.output}",flush=True)
        return 0 if result['checkpoint_passed'] else 1
    if args.command=='plan-staged':
        from .staged_plan import plan_session
        result=plan_session(args.output,yaml.safe_load(args.config.read_text(encoding='utf-8')),args.development_reference,args.contract_only)
        if result.get('completed_round')=='R0':
            print(f"P0 R0 数据契约：{'通过' if result['contract_passed'] else '失败'}；未规划碰撞轨迹、未采集、未完成P0验收 -> {args.output}",flush=True)
            return 0 if result['contract_passed'] else 1
        print(f"Staged plan: {'PASS' if result['passed'] else 'REJECT'} -> {args.output}",flush=True)
        return 0 if result['passed'] else 1
    if args.command=="dual-calibrate":
        from .dual_workflow import run_dual_workflow
        config=yaml.safe_load(args.config.read_text(encoding="utf-8"))
        result=run_dual_workflow(args.output,config)
        if args.render and result['passed']:
            from .dual_presentation import render_dual_workflow
            render_dual_workflow(args.output)
        print(f"Dual-arm mock: {'PASS' if result['passed'] else 'FAIL'} -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command=="render-dual":
        from .dual_presentation import render_dual_workflow
        path=render_dual_workflow(args.folder,args.output,fps=args.fps,video=not args.snapshots_only)
        print(f"Dual-arm presentation -> {path}")
        return 0
    if args.command=="dual-identify":
        from .dual_workflow import identify_dual_session
        result,_=identify_dual_session(args.folder,args.output)
        print(f"Dual-arm observations: {'PASS' if result['passed'] else 'FAIL'} -> {args.output}")
        return 0 if result['passed'] else 1
    if args.command in ('ros-benchmark','ros-analyse'):
        from .ros_benchmark import run_suite,analyse_suite
        result=run_suite(args.output,yaml.safe_load(args.config.read_text(encoding='utf-8'))) if args.command=='ros-benchmark' else analyse_suite(args.folder,args.output)
        print(f"ROS stream: {'PASS' if result['passed'] else 'FAIL'} -> {args.output}",flush=True)
        return 0 if result['passed'] else 1
    if args.command=="arm-check":
        from .arm_report import run_arm_checks
        report=run_arm_checks(args.output,render=args.render)
        print(f"R1: {'PASS' if report['passed'] else 'FAIL'} -> {args.output}")
        return 0 if report["passed"] else 1
    if args.command=="demo":
        from .demo import run_demo
        config=yaml.safe_load(args.config.read_text(encoding="utf-8")) if args.config else None
        run_demo(args.output,config=config)
        if not args.collect_only:
            from .presentation import render_demo
            render_demo(args.output,fps=args.fps)
        print(f"Workflow complete -> {args.output}")
        return 0
    if args.command=="render-demo":
        from .presentation import render_demo
        result=render_demo(args.folder,args.output,fps=args.fps,width=args.width,height=args.height,
            clip_seconds=args.clip_seconds,video=not args.snapshots_only)
        print(f"Presentation -> {result}")
        return 0
    if args.command=="view":
        from .viewer import default_adapter,show
        from .telemetry import TelemetryAdapter,ReplaySource
        adapter=TelemetryAdapter.load(args.mapping) if args.mapping else default_adapter()
        if args.run_folder:
            replay=ReplaySource.load(args.run_folder,adapter)
            show(replay,adapter,args.scene,duration_s=replay.times[-1]*1e-9)
        else:
            from .live import JsonlTelemetrySource
            stream=JsonlTelemetrySource(args.telemetry_jsonl,adapter,follow=args.follow)
            try:show(stream,adapter,args.scene)
            finally:stream.close()
        return 0
    if args.command=="identify":
        config=yaml.safe_load(args.config.read_text(encoding="utf-8"))
        result=identify(load_observations(args.run_folder),Prior(**config.get("identification",{})))
        write_json(args.output,result)
        print(f"identifiable={result['identifiable']} -> {args.output}")
        return 0 if result["identifiable"] else 1
    if args.command=="run":
        config=yaml.safe_load(args.config.read_text(encoding="utf-8"))
        report=run_stage(config,args.output)
        print(f"{config['stage']}: {'PASS' if report['passed'] else 'FAIL'} -> {args.output}",flush=True)
        return 0 if report["passed"] else 1
    args.output.mkdir(parents=True,exist_ok=False)
    reports=[]
    for config_path in sorted(args.configs.glob("[A-F]_*.yaml")):
        print(f"Running {config_path.name}",flush=True)
        config=yaml.safe_load(config_path.read_text(encoding="utf-8"))
        report=run_stage(config,args.output/config["stage"])
        reports.append({"stage":report["stage"],"passed":report["passed"],"checks":report["checks"]})
        print(f"  {'PASS' if report['passed'] else 'FAIL'}",flush=True)
    if len(reports)!=6: raise ValueError("Suite requires exactly six A-F configs")
    summary={"created_utc":datetime.now(timezone.utc).isoformat(),"synthetic_only":True,
             "platform":platform.platform(),"python":sys.version,"numpy":np.__version__,"scipy":scipy.__version__,
             "passed":all(r["passed"] for r in reports),"stages":reports}
    write_json(args.output/"suite.json",summary)
    (args.output/"README.md").write_text("# Synthetic calibration validation\n\n"+
        "\n".join(f"- {r['stage']}: {'PASS' if r['passed'] else 'FAIL'} — [{r['stage']}/report.md]({r['stage']}/report.md)" for r in reports)+
        "\n\nNumerical results are synthetic, not ENCOS specifications or hardware accuracy.\n",encoding="utf-8")
    return 0 if summary["passed"] else 1

if __name__=="__main__":
    raise SystemExit(main())
