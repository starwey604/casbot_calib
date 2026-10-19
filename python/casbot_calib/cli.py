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
    run=sub.add_parser("run",help="Run one independent A-F protocol")
    run.add_argument("config",type=Path);run.add_argument("--output",required=True,type=Path)
    suite=sub.add_parser("suite",help="Run all protocols and acceptance checks")
    suite.add_argument("--configs",type=Path,default=Path("configs"));suite.add_argument("--output",required=True,type=Path)
    estimate=sub.add_parser("identify",help="Re-estimate from observations only; no evaluation truth required")
    estimate.add_argument("run_folder",type=Path);estimate.add_argument("--config",type=Path,required=True)
    estimate.add_argument("--output",type=Path,required=True)
    demo=sub.add_parser("demo",help="Collect the calibration workflow, then export the read-only viewer video")
    demo.add_argument("--output",required=True,type=Path)
    demo.add_argument("--collect-only",action="store_true",help="Run the workflow without importing MuJoCo")
    demo.add_argument("--fps",type=int,default=24)
    render=sub.add_parser("render-demo",help="Render an existing workflow without rerunning physics or identification")
    render.add_argument("folder",type=Path);render.add_argument("--output",type=Path)
    render.add_argument("--fps",type=int,default=24)
    render.add_argument("--width",type=int,default=1280);render.add_argument("--height",type=int,default=720)
    render.add_argument("--clip-seconds",type=float,help="Override each clip duration for a short preview")
    render.add_argument("--snapshots-only",action="store_true")
    view=sub.add_parser("view",help="Read-only interactive joint viewer (recording or external JSONL telemetry)")
    source=view.add_mutually_exclusive_group(required=True)
    source.add_argument("--run-folder",type=Path);source.add_argument("--telemetry-jsonl",type=Path)
    view.add_argument("--mapping",type=Path);view.add_argument("--scene",type=Path)
    view.add_argument("--follow",action="store_true",help="Follow an externally appended JSONL file")
    args=parser.parse_args(argv)
    if args.command=="demo":
        from .demo import run_demo
        run_demo(args.output)
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
