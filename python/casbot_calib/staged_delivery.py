"""End-to-end staged delivery, with explicitly separate synthetic evaluation."""
import hashlib,json
from pathlib import Path
import numpy as np
from .data import write_json


def evaluate_synthetic(folder):
    """Optional evaluator; these numbers never enter fitting or controller choice."""
    from .staged_collection import synthetic_plant,load_trial
    from .staged_control import load_candidate
    root=Path(folder);session=json.loads((root/'session.json').read_text());workflow=json.loads((root/'workflow.json').read_text())
    plant_path=root/'evaluation'/'plant_config.json'
    if not plant_path.exists():return dict(available=False,used_by_identification=False)
    plant=synthetic_plant({**session['config'],'mock':json.loads(plant_path.read_text())})
    candidate=load_candidate(root/workflow['inertia_folder']/'candidate.json')
    _,curve=load_trial(root,'inertia_validation');q=curve(np.linspace(3.,25.,128))
    true_m=np.array([plant.mass_matrix(p) for p in q]);estimated=np.array([candidate.mass_matrix(p) for p in q])
    diag_error=np.max(abs(np.diagonal(estimated,axis1=1,axis2=2)/np.diagonal(true_m,axis1=1,axis2=2)-1),axis=0)
    friction={}
    for name,field in [('viscous','viscous'),('coulomb','coulomb'),('static_friction','static_friction')]:
        truth=np.array([getattr(j.friction,field) for j in plant.model.joints]);fit=np.array([getattr(j.friction,field) for j in candidate.model.joints])
        friction[name]=dict(truth=truth.tolist(),estimate=fit.tolist(),maximum_relative_error=float(np.max(abs(fit/truth-1))))
    result=dict(available=True,used_by_identification=False,matrix_diagonal_maximum_relative_errors=diag_error.tolist(),
        maximum_matrix_relative_frobenius_error=float(np.max(np.linalg.norm(estimated-true_m,axis=(1,2))/np.linalg.norm(true_m,axis=(1,2)))),
        friction=friction,scope='synthetic evaluator only; body inertias are a physical representative of observable combinations, not unique recovered CAD quantities')
    result['passed']=bool(np.max(diag_error)<=.05 and all(v['maximum_relative_error']<=.05 for v in friction.values()))
    result['thresholds']=dict(maximum_mass_matrix_diagonal_relative_error=.05,maximum_nonzero_friction_relative_error=.05)
    from .staged_validation import numerical_replays
    numerical=numerical_replays(root,plant,session['config'],['direction_00','timing_train','friction_validation_00_03_0.14',
        'breakaway_0_03_+1_0.5_0.24_0','wrist_probe_1_04_+1','inertia_validation'],model_scope='declared synthetic plant (evaluation only)')
    result['plant_numerical_replays']=numerical;result['passed']=result['passed'] and all(row['passed'] for row in numerical)
    write_json(root/'evaluation'/'parameter_comparison.json',result)
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    figure=Figure(figsize=(10,5));FigureCanvasAgg(figure);axis=figure.subplots()
    axis.bar(np.arange(14),diag_error*100);axis.set(xlabel='L1..L7, R1..R7',ylabel='Maximum synthetic M(q) diagonal error / %');axis.grid(axis='y',alpha=.2)
    figure.tight_layout();figure.savefig(root/'evaluation'/'inertia_comparison.png',dpi=140)
    return result


def finalize(folder,verification,presentation):
    root=Path(folder);verification=Path(verification);presentation=Path(presentation)
    workflow=json.loads((root/'workflow.json').read_text());checked=json.loads((verification/'verification.json').read_text());view=json.loads((presentation/'presentation.json').read_text())
    video=presentation/'staged_calibration_zh.mp4'
    passed=bool(workflow['passed'] and checked['passed'] and view['passed'] and video.is_file() and video.stat().st_size>0)
    for path,digest in checked['input_sha256'].items():
        if hashlib.sha256((root/path).read_bytes()).hexdigest()!=digest:raise ValueError('Comprehensive verification predates current inputs: '+path)
    # Playback fingerprints must still match; a later recalibration invalidates it.
    for path,digest in view['source_sha256'].items():
        if hashlib.sha256((root/path).read_bytes()).hexdigest()!=digest:raise ValueError('Presentation predates current calibration: '+path)
    for path,digest in view.get('external_source_sha256',{}).items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:raise ValueError('External ROS presentation source changed: '+path)
    evaluation=evaluate_synthetic(root)
    passed=passed and (not evaluation['available'] or evaluation['passed'])
    control=json.loads((root/'control_comparison.json').read_text())
    stages={name:json.loads((root/row['report']).read_text()) for name,row in workflow['stages'].items()}
    result=dict(passed=passed,full_plan_complete=passed,synthetic_only=True,hardware_executable=False,
        scope='M3-M6 staged synthetic workflow; M1 regression and M2 local ROS evidence are separately versioned',
        verification=str(verification.relative_to(root)),presentation=str(presentation.relative_to(root)),
        video_sha256=hashlib.sha256(video.read_bytes()).hexdigest(),evaluation=evaluation,
        metrics=dict(trials=len(checked['collision']),gravity_rank=stages['gravity']['fit']['rank'],
            gravity_maximum_paired_rmse_nm=stages['gravity']['validation']['maximum_absolute_rmse_nm'],
            maximum_inertia_validation_nrmse=max(v['maximum_normalized_rmse'] for v in stages['inertia']['validation'].values()),
            maximum_independent_wrist_inertia_disagreement=max(p['relative_inertia_disagreement'] for p in stages['inertia']['probe_validation'].values()),
            fixed_pd_feedforward_rmse_reduction=control['fixed_pd_feedforward_rmse_reduction']),
        limitations=['Local ROS timers do not qualify a physical Ethernet or ros2_control deployment',
            'Fixture dimensions/transform and actual device limits remain unmeasured',
            '4 kHz raw current acquisition and 8 kHz convergence comparison require a confirmed hardware interface',
            'Current-derived effort is not an independent torque measurement; output gain is declared synthetic',
            'Fixed waist; no torso motion/compliance or kinematic calibration'])
    write_json(root/'delivery.json',result)
    write_json(root/'checkpoint.json',dict(checkpoint_passed=passed,completed_through='comprehensive_and_presentation',
        full_plan_complete=passed,synthetic_only=True,hardware_executable=False,delivery='delivery.json',
        pending=[] if passed else ['resolve failed comprehensive or evaluation checks']))
    (root/'delivery.md').write_text('# 双臂分阶段动力学标定交付\n\n'+('通过' if passed else '未通过')+'：合成实验，未接实机。\n\n'+
        '\n'.join(f'- {key}: {value}' for key,value in result['metrics'].items())+'\n\n'+
        f'[完整验证]({result["verification"]}/verification.json) · [中文演示]({result["presentation"]}/staged_calibration_zh.mp4) · [控制对照](control_comparison.png)\n\n'+
        '\n'.join('- '+line for line in result['limitations'])+'\n',encoding='utf-8')
    return result


def _fresh(root,name):
    path=root/name;index=1
    while path.exists():path=root/(name+f'_{index}');index+=1
    return path


def run_suite(folder,config,resume=False,fps=12,ros_report=None):
    from .staged_workflow import run_staged
    from .staged_validation import verify_staged
    from .staged_presentation import render_staged
    root=Path(folder);run_staged(root,config,'all',resume)
    verification=_fresh(root,'comprehensive');checked=verify_staged(root,verification)
    if not checked['passed']:raise RuntimeError('Comprehensive validation failed; presentation delivery blocked')
    presentation=_fresh(root,'presentation');render_staged(root,presentation,fps=fps,ros_report=ros_report)
    return finalize(root,verification,presentation)
