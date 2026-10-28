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
    parser=argparse.ArgumentParser(description="Synthetic joint calibration experiments (no hardware)")
    sub=parser.add_subparsers(dest="command",required=True)
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
    staged=sub.add_parser('plan-staged',help='Plan and collision-gate all staged dual-arm trials (no collection)')
    staged.add_argument('--config',type=Path,default=Path('configs/staged_dual_arm.yaml'));staged.add_argument('--output',type=Path,required=True)
    staged_run=sub.add_parser('staged-calibrate',help='Acquire staged Mock calibration and independently validate four control groups')
    staged_run.add_argument('--config',type=Path,default=Path('configs/staged_dual_arm.yaml'));staged_run.add_argument('--output',type=Path,required=True)
    staged_run.add_argument('--through',choices=('gravity','friction','breakaway','inertia','all'),default='all');staged_run.add_argument('--resume',action='store_true')
    staged_analysis=sub.add_parser('staged-identify',help='Reanalyse staged raw observations without plant configuration or evaluation truth')
    staged_analysis.add_argument('folder',type=Path);staged_analysis.add_argument('--output',type=Path,required=True)
    staged_analysis.add_argument('--through',choices=('gravity','friction','breakaway','inertia'),default='inertia')
    staged_verify=sub.add_parser('staged-verify',help='Recheck all collision gates, numerical convergence and truth-isolated inference')
    staged_verify.add_argument('folder',type=Path);staged_verify.add_argument('--output',type=Path,required=True)
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
        result=verify_staged(args.folder,args.output)
        print(f"Comprehensive staged verification: {'PASS' if result['passed'] else 'FAIL'} -> {args.output}",flush=True)
        return 0 if result['passed'] else 1
    if args.command=='staged-suite':
        from .staged_delivery import run_suite
        result=run_suite(args.output,yaml.safe_load(args.config.read_text(encoding='utf-8')),args.resume,args.fps,args.ros_report)
        print(f"Staged delivery: {'PASS' if result['passed'] else 'FAIL'} -> {args.output}",flush=True)
        return 0 if result['passed'] else 1
    if args.command=='staged-identify':
        from .staged_analysis import identify_staged
        result=identify_staged(args.folder,args.output,args.through)
        print(f"Staged analysis: {'PASS' if result['passed'] else 'FAIL'} -> {args.output}",flush=True)
        return 0 if result['passed'] else 1
    if args.command=='staged-calibrate':
        from .staged_workflow import run_staged
        result=run_staged(args.output,yaml.safe_load(args.config.read_text(encoding='utf-8')),args.through,args.resume)
        print(f"Staged checkpoint: {'PASS' if result['checkpoint_passed'] else 'FAIL'} through {result['completed_through']} -> {args.output}",flush=True)
        return 0 if result['checkpoint_passed'] else 1
    if args.command=='plan-staged':
        from .staged_plan import plan_session
        result=plan_session(args.output,yaml.safe_load(args.config.read_text(encoding='utf-8')))
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
