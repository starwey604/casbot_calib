"""K3 public planning, K2 collection, truth-isolated geometry fitting and delivery."""
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from . import _core
from .data import write_json
from .kinematics import nominal_kinematics
from .kinematic_parameters import KinematicParameters
from .kinematic_design import plan,k3_config
from .kinematic_identification import identify
from .trical_workflow import run_mock
from .trical_analysis import analyse


def _verified_plan(folder):
    root=Path(folder);manifest=json.loads((root/'manifest.json').read_text())
    for name,expected in manifest['files'].items():
        path=Path(name)
        if path.is_absolute() or '..' in path.parts or 'evaluation' in path.parts:raise ValueError('Invalid planning manifest')
        if sha256((root/path).read_bytes()).hexdigest()!=expected:raise ValueError('Frozen K3 plan digest mismatch')
    if not json.loads((root/'plan_report.json').read_text())['passed']:raise ValueError('Rejected K3 plan')
    profile=json.loads((root/'profile.json').read_text());engine,_=nominal_kinematics(profile)
    parameterization=json.loads((root/'parameterization.json').read_text())
    if KinematicParameters(engine).description()!=parameterization:raise ValueError('K3 parameter convention changed after planning')
    return profile,sha256((root/'manifest.json').read_bytes()).hexdigest()


def collect_and_identify(plan_folder,output):
    started=perf_counter();root=Path(output)
    if root.exists():raise FileExistsError(root)
    config,plan_sha=_verified_plan(plan_folder);root.mkdir(parents=True,exist_ok=False)
    # Only the Mock factory receives these hidden physical parameters.
    hidden=json.loads((Path(plan_folder)/'evaluation/mock.json').read_text());captures={}
    native=sha256(Path(_core.__file__).read_bytes()).hexdigest()
    for index,split in enumerate(('train','validation')):
        profile=deepcopy(config);profile['workflow']['visits']=json.loads((Path(plan_folder)/(split+'_visits.json')).read_text())
        profile['k3']['batch_id']='k3_'+split+'_'+plan_sha[:12];profile['mock']=deepcopy(hidden);profile['mock']['seed']+=index*1000
        captures[split]=run_mock(root/split,profile)
        if not captures[split]['passed']:
            report=dict(passed=False,stage='K3',reason='measurement_batch_rejected',split=split,captures=captures)
            write_json(root/'report.json',report);return report
        if captures[split]['native_sha256']!=native:raise ValueError('Native kernel changed during acquisition')
        checked=analyse(root/split,root/split/'measurement_analysis')
        if not checked['passed']:raise ValueError('K2 observation-only reanalysis rejected K3 measurements')
    quarantine=root/'withheld_evaluation';quarantine.mkdir();moved=[]
    try:
        for label,source in [('plan',Path(plan_folder)/'evaluation'),('train',root/'train/evaluation'),('validation',root/'validation/evaluation')]:
            target=quarantine/label;source.rename(target);moved.append((source,target))
        result=identify(root/'train',root/'validation',root/'identification')
    finally:
        for source,target in reversed(moved):target.rename(source)
        quarantine.rmdir()
    for split in ('train','validation'):
        source=root/split;manifest=json.loads((source/'manifest.json').read_text())
        for name,expected in manifest['files'].items():
            if sha256((source/name).read_bytes()).hexdigest()!=expected:raise ValueError('Identification changed immutable measurements')
    report=dict(schema_version=1,stage='K3',passed=result['passed'],truth_isolated=True,raw_unchanged=True,
        source_plan_sha256=plan_sha,native_sha256=native,train_samples=result['train_samples'],validation_samples=result['validation_samples'],
        selected_parameter_count=len(result['selected_parameters']),standard_parameter_count=result['standard_parameter_count'],
        calibration_performed=bool(result['identifiable']),hardware_executable=False,synthetic_only=True,
        identification=result,wall_s=perf_counter()-started,
        pending=['K4 broad independent metrology/repeatability acceptance and MuJoCo video','K5 real robot/instrument adapters'])
    write_json(root/'report.json',report);return report


def run_suite(output,config=None):
    root=Path(output);root.mkdir(parents=True,exist_ok=False);config=k3_config() if config is None else config
    designed=plan(root/'plan',config)
    if not designed['passed']:
        result=dict(passed=False,stage='K3',reason='posture_design_rejected',planning=designed)
        write_json(root/'suite.json',result);return result
    result=collect_and_identify(root/'plan',root/'calibration')
    if result['passed']:
        from .kinematic_evaluation import evaluate,plot_result
        evaluation=evaluate(root/'calibration',root/'synthetic_evaluation');plot_result(root/'calibration',root/'constraints_zh.png')
        result['synthetic_evaluation']=evaluation
    write_json(root/'suite.json',result)
    (root/'README.md').write_text('# K3双臂几何辨识\n\n结果：'+('通过' if result['passed'] else '拒绝')+
        '。候选与参数选择仅使用训练设计/数据；验证为独立设备会话。\n\n'
        '[汇总](suite.json) · [姿态设计](plan/plan_report.json) · [辨识](calibration/identification/report.json) · '
        '[参数及MDH表](calibration/identification/mdh_table.csv) · [候选URDF](calibration/identification/candidate.urdf)\n\n'
        '仅对有尺度基准球的位置约束验收；没有实机绝对精度或末端姿态精度结论。K4再做完整验收和视频。\n',encoding='utf-8')
    return result
