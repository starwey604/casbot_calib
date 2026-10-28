"""Join separately executed M1, M2 and M3-M6 evidence; never run hardware."""
import argparse,hashlib,json
from pathlib import Path
from casbot_calib import _core
from casbot_calib.data import write_json


def verify(regression,ros,staged,output):
    read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))
    m1=read(regression/'verification.json');m2=read(ros/'verification.json');delivery=read(staged/'delivery.json')
    native=hashlib.sha256(Path(_core.__file__).read_bytes()).hexdigest()
    plan=read(staged/'plan_report.json');session=read(staged/'session.json')
    records=[read(staged/'runs'/name/'metadata.json') for name in plan['selected_trials']]
    checks=dict(m1_regression=m1['passed'] and m1['baseline_metrics_compared']==114,
        m1_consumers_wheel=all(m1[k] for k in ('native_consumer','ament_consumer','independent_wheel')),
        m2_local_ros=m2['passed'] and m2['cases']==13 and m2['truth_isolated_reanalysis_identical'],
        staged_delivery=delivery['passed'] and delivery['full_plan_complete'],
        default_500_hz=session['config']['control_period_ns']==2000000 and m2['default_command_control_feedback_hz']==500,
        recorded_native_matches_current=all(r['native_binary_sha256']==native for r in records),
        recorded_backend=all(r.get('stiction_backend')=='coupled_active_set_mass_projection_v2' for r in records),
        no_hardware_claim=not delivery['hardware_executable'] and not m2['physical_network_tested'])
    sources=[p for directory in ('cpp','python','configs','ros','scripts/humble','tests','docs') for p in Path(directory).rglob('*')
        if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.so','.pyc','.pyd')]
    sources += [Path(p) for p in ('CMakeLists.txt','CMakePresets.json','pyproject.toml','README.md','.github/workflows/validation.yml')]
    result=dict(passed=all(checks.values()),checks=checks,native_binary_sha256=native,
        regression=str(regression),ros=str(ros),staged=str(staged),tests=m1['tests'],
        metrics=delivery['metrics'],physical_network_tested=False,hardware_executable=False,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    write_json(output,result)
    if not result['passed']:raise RuntimeError('Release evidence mismatch: '+str(checks))
    print('M1-M6 synthetic release evidence: PASS',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('regression','ros','staged','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();verify(args.regression,args.ros,args.staged,args.output)
