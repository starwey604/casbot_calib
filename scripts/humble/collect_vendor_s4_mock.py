#!/usr/bin/env python3
"""Fresh 4kHz onset and later control runs, reusing native C++ physics/guards."""
import argparse
import copy
import json
from pathlib import Path
import shutil
from casbot_calib.data import write_json
from casbot_calib.vendor_identity import sha256


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--candidate-analysis',type=Path)
    args=parser.parse_args();source=args.source.resolve();root=args.output.resolve()
    from casbot_calib.staged_collection import synthetic_plant
    from casbot_calib.collision import nominal_profile
    from casbot_calib.dual_model import nominal_dual_arm
    from casbot_calib.rigid_identification import model_from_parameters
    from casbot_calib.staged_breakaway import collect_breakaway
    from casbot_calib.staged_control import run_control
    if args.candidate_analysis:
        session=json.loads((root/'session.json').read_text());config=session['config']
        analysis=json.loads((args.candidate_analysis/'analysis.json').read_text())
        if not analysis['passed'] or analysis['completed_through']!='inertia':raise ValueError('Accepted S4 candidate required')
        mock=json.loads((root/'evaluation/plant_config.json').read_text());plant=synthetic_plant(dict(config,mock=mock))
        result=run_control(root,plant,config,nominal_profile(scene=session['collision']['scene']),args.candidate_analysis/'inertia')
        write_json(root/'s4_control_binding.json',dict(passed=result['passed'],analysis_sha256=sha256(args.candidate_analysis/'manifest.json'),
            candidate_sha256=sha256(args.candidate_analysis/'inertia/candidate.json'),hardware_executable=False,
            scope='fresh native virtual-time control, not DDS/SDK or physical execution'))
        return 0 if result['passed'] else 1
    session=json.loads((source/'session.json').read_text());config=session['config']
    if config['schema_version']!=1 or config['control_period_ns']!=2000000:raise ValueError('Explicit legacy 500Hz synthetic source required')
    selected=[];plan=json.loads((source/'plan_report.json').read_text())
    for name in plan['selected_trials']:
        meta=json.loads((source/'plans'/name/'motion.json').read_text());c=meta['conditions']
        if meta['stage'] in ('control_tuning','control_validation'):selected.append(name)
        if meta['stage']=='breakaway' and ((c['pose']==0 and c['repeat']==0 and c['dwell_s']==.5 and c['slope_nm_s']==.12) or
                                         (c['pose']==1 and c['repeat']==1 and c['dwell_s']==1.5 and c['slope_nm_s']==.24)):
            selected.append(name)
    if sum(name.startswith('breakaway_') for name in selected)!=56:raise ValueError('Require two directions, all14 axes, separate train/validation conditions')
    root.mkdir(parents=True,exist_ok=False)
    write_json(root/'session.json',session)
    for name in selected:
        shutil.copytree(source/'plans'/name,root/'plans'/name)
        if name.startswith('breakaway_'):
            motion=json.loads((root/'plans'/name/'motion.json').read_text());motion['conditions']['sample_period_ns']=250000
            write_json(root/'plans'/name/'motion.json',motion)
            manifest=json.loads((root/'plans'/name/'manifest.json').read_text());manifest['motion_sha256']=sha256(root/'plans'/name/'motion.json')
            write_json(root/'plans'/name/'manifest.json',manifest)
    write_json(root/'plan_report.json',dict(passed=plan['passed'],selected_trials=selected,trials=len(selected),
        scope='unchanged digest-checked gated references; fresh onset raw4k override and later nine control runs; no whole legacy-plan completion claim',
        source_plan_sha256=sha256(source/'plan_report.json'),hardware_executable=False))
    # Collection may read a declared synthetic plant. Fitting/replay never do.
    mock=json.loads((source/'evaluation/plant_config.json').read_text());(root/'evaluation').mkdir()
    write_json(root/'evaluation/plant_config.json',mock)
    write_json(root/'lineage.json',dict(source=str(source),source_session_sha256=sha256(source/'session.json'),
        source_plan_sha256=sha256(source/'plan_report.json'),new_raw_sample_period_ns=250000,
        limitation='Two poses/dwells/slopes differ by split; not a full factorial dwell/posture dependency experiment'))
    nominal,_=nominal_dual_arm();plant=synthetic_plant(dict(config,mock=mock));profile=nominal_profile(scene=session['collision']['scene'])
    earlier=json.loads((source/'identification_gravity/report.json').read_text())
    gravity=model_from_parameters(nominal,earlier['fit']['parameters'])
    for name in selected:
        if name.startswith('breakaway_'):collect_breakaway(root,name,plant,gravity,config,profile)
    return 0


if __name__=='__main__':raise SystemExit(main())
