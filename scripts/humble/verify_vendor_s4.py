#!/usr/bin/env python3
"""S4 read-only evidence audit and new-process analysis with truth access denied."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from casbot_calib.vendor_staged import StagedObservations
from casbot_calib.vendor_identity import sha256
from casbot_calib.data import write_json


def read(path):return json.loads(Path(path).read_text())


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--analysis',type=Path,required=True);parser.add_argument('--native',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False);checks={}
    analysis=args.analysis.resolve();protocol=args.protocol.resolve();native=args.native.resolve()
    for name,digest in read(analysis/'manifest.json').items():
        if sha256(analysis/name)!=digest:raise ValueError('S4 analysis file changed')
    report=read(analysis/'analysis.json');reader=StagedObservations(protocol)
    checks['all_stages_accepted']=report['passed'] and report['completed_through']=='inertia' and len(report['stages'])==4
    checks['no_hardware_or_final_test_claim']=not report['hardware_executable'] and not report['independent_final_test_passed']
    counts={};source_bindings={}
    for name,row in reader.entries.items():
        _,motion,meta,data,qualification=reader.load(name)
        counts[row['stage']]=counts.get(row['stage'],0)+1
        source_bindings[name]=dict(qualification_sha256=row['qualification']['sha256'],raw_source_sha256=meta['contract']['source_sha256'])
    checks['complete_declared_development_inputs']=counts==dict(gravity=64,friction=210,breakaway=56,inertia=4,inertia_probe=12)
    checks['fresh_4k_onset_all_axes_and_directions']=all(
        read(native/'runs'/row['name']/'metadata.json')['sample_period_ns']==250000 for row in reader.entries.values() if row['stage']=='breakaway')
    from casbot_calib.staged_control import load_candidate,_metrics
    candidate=load_candidate(analysis/'inertia/candidate.json')  # native constructor independently validates inertias
    inertia=read(analysis/'inertia/report.json')
    checks['physical_candidate_and_prior_stages']=inertia['passed'] and inertia['prior_stage_acceptance']['passed']
    binding=read(native/'s4_control_binding.json');control=read(native/'control_comparison.json');session=read(native/'session.json')
    checks['fresh_control_bound_to_this_candidate']=binding['analysis_sha256']==sha256(analysis/'manifest.json') and binding['candidate_sha256']==sha256(analysis/'inertia/candidate.json')
    actual={name:_metrics(native,name,session['config']) for name in control['groups']}
    checks['four_control_records_independently_rescored']=len(actual)==4 and all(
        np.isclose(actual[name]['aggregate_rmse_rad'],control['groups'][name]['aggregate_rmse_rad'],rtol=0,atol=1e-14) and actual[name]['admissible'] for name in actual)
    reduction=1-actual['control_identified_original']['aggregate_rmse_rad']/actual['control_nominal_original']['aggregate_rmse_rad']
    checks['synthetic_fixed_pd_feedforward_gain']=control['passed'] and reduction>=.2
    # The replay uses only observable inputs. Audit denial is actively exercised.
    code='''import sys,os
from pathlib import Path
def guard(event,args):
    if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)) and 'evaluation' in Path(os.fsdecode(args[0])).parts:
        raise RuntimeError('evaluation access forbidden')
sys.addaudithook(guard)
try:open('/__casbot_guard_probe__/evaluation/truth.npz','rb')
except RuntimeError:pass
else:raise RuntimeError('truth guard failed')
from casbot_calib.vendor_staged import analyse_staged
result=analyse_staged(sys.argv[1],sys.argv[2])
raise SystemExit(0 if result['passed'] else 1)
'''
    replay=subprocess.run([sys.executable,'-c',code,str(protocol),str(root/'without_truth')],capture_output=True,text=True)
    (root/'truth_isolation.log').write_text(replay.stdout+replay.stderr)
    checks['fresh_process_truth_access_denied']=replay.returncode==0
    if replay.returncode==0:
        original=read(analysis/'inertia/candidate.json');again=read(root/'without_truth/inertia/candidate.json')
        changes={key:float(np.max(abs(np.asarray(original[key])-np.asarray(again[key])))) for key in ('parameters','viscous','coulomb','static_friction')}
        checks['replayed_candidate_identical_within_1e_8']=max(changes.values())<=1e-8
    else:changes={}
    result=dict(passed=all(checks.values()),checks=checks,records=counts,hardware_executable=False,physical_semantics_verified_by_software=False,
        scope='S4 synthetic offline staged and fresh native virtual-time control; reused development validation, not P0 or hardware final test',
        fixed_pd_feedforward_rmse_reduction=reduction,truth_isolated_parameter_max_changes=changes,source_bindings=source_bindings,
        analysis_manifest_sha256=sha256(analysis/'manifest.json'),protocol_sha256=sha256(protocol/'protocol.json'))
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.font_manager import FontProperties
    from casbot_calib.presentation import _font
    font=FontProperties(fname=_font(12).path);figure=Figure(figsize=(14,5));FigureCanvasAgg(figure);left,right=figure.subplots(1,2)
    stages=[('gravity','重力'),('friction','运动摩擦'),('breakaway','起动摩擦'),('inertia','惯量及前序复核')]
    left.axis('off');table=left.table(cellText=[[label,'通过' if read(analysis/stage/'report.json')['passed'] else '失败',str(counts[stage])] for stage,label in stages],
        colLabels=['阶段','离线验收','记录数'],loc='center')
    table.auto_set_font_size(False);table.set_fontsize(12);table.scale(1,2.5)
    for cell in table.get_celld().values():cell.get_text().set_fontproperties(font)
    left.set_title('分阶段辨识：346条开发观测',fontproperties=font)
    labels=['原模型\n原PD','原模型\n优化PD','辨识模型\n原PD','辨识模型\n优化PD']
    keys=['control_nominal_original','control_nominal_tuned','control_identified_original','control_identified_tuned']
    right.bar(range(4),[actual[k]['aggregate_rmse_rad']*1000 for k in keys],color=['#718096','#a0aec0','#3182ce','#38a169'])
    right.set_xticks(range(4),labels,fontproperties=font);right.set_ylabel('独立轨迹 RMSE / mrad',fontproperties=font)
    right.set_title(f'固定PD前馈改善 {100*reduction:.2f}%',fontproperties=font);right.grid(axis='y',alpha=.2)
    figure.suptitle('S4 本机合成闭环；复用开发留出，实机执行仍关闭',fontproperties=font)
    figure.tight_layout();figure.savefig(root/'summary_zh.png',dpi=140)
    write_json(root/'verification.json',result);print(json.dumps({k:v for k,v in result.items() if k!='source_bindings'},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
