"""Gate and summarize M1 evidence; does not claim network or new calibration stages."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def summarize(output):
    def read(name):
        return json.loads((output / name).read_text(encoding='utf-8'))
    def tests(name):
        suites = ET.parse(output / name).getroot()
        rows = [suites] if suites.tag == 'testsuite' else list(suites.findall('testsuite'))
        assert rows and all(int(row.get('failures', 0)) == int(row.get('errors', 0)) == int(row.get('skipped', 0)) == 0 for row in rows)
        return sum(int(row.get('tests', 0)) for row in rows)
    counts = dict(pytest=tests('pytest.xml'), ctest=tests('ctest.xml'))
    assert counts['pytest'] >= 87 and counts['ctest'] == 3
    environment = read('environment.json'); wheel = read('wheel/verification.json')
    assert environment['passed'] and environment['conda_paths_absent'] and wheel['passed']
    native = (output / 'native-consumer.txt').read_text()
    assert native == (output / 'ament-consumer.txt').read_text()
    assert environment['backend'] in native and environment['integration_backend'] in native
    assert read('experiments/suite.json')['passed'] and read('experiments/R1/report.json')['passed']
    workflow = read('experiments/D14/workflow.json'); report = read('experiments/D14/identification/report.json')
    render = read('experiments/D14/presentation/presentation.json')
    assert workflow['passed'] and report['passed'] and render['passed'] and not render['truth_read']
    assert not render['angles_amplified'] and render['max_fk_error'] < 1e-12
    comparison = output / 'baseline-comparison.txt'
    compared = 0
    if comparison.exists():
        match = re.fullmatch(r'PASS: (\d+) numerical metrics agree within explicit tolerances\s*', comparison.read_text())
        assert match, 'Frozen baseline comparison did not pass'
        compared = int(match.group(1))
    sources = [p for directory in ('cpp', 'python', 'configs', 'tests', 'cmake', 'ros', 'scripts/humble')
               for p in Path(directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts
               and p.suffix not in ('.pyc', '.so', '.pyd')]
    sources += [Path(name) for name in ('CMakeLists.txt', 'CMakePresets.json', 'pyproject.toml', '.github/workflows/validation.yml')]
    result = dict(passed=True, milestone='M1', scope='native Humble build and offline regression; no hardware or cross-host timing validation',
        tests=counts, backend=environment['backend'], integration_backend=environment['integration_backend'],
        baseline_metrics_compared=compared, native_consumer=True, ament_consumer=True, independent_wheel=True,
        dual_validation_nrmse={name: value['maximum_normalized_rmse'] for name, value in report['validation'].items()},
        control=workflow['control'], render_frames=render['frames'], maximum_fk_error=render['max_fk_error'],
        source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (output / 'verification.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    (output / 'README.md').write_text('# M1：Humble迁移验收\n\n'
        f"结果：通过。pytest {counts['pytest']}项、CTest {counts['ctest']}项。"
        f" 后端：{result['backend']} / {result['integration_backend']}。\n\n"
        '[环境及动态链接清单](environment.json) · [机器验收记录](verification.json) · '
        '[wheel隔离验证](wheel/verification.json) · [A–F实验](experiments/README.md) · '
        '[R1检查](experiments/R1/report.md) · [双臂流程](experiments/D14/README.md) · '
        '[中文视频](experiments/D14/presentation/dual_calibration_zh.mp4)\n\n'
        f"冻结基线比较指标数：{compared}（0表示本次未提供基线，需由CI另行比较）。"
        '独立CMake与ament消费者通过，wheel不依赖源码editable跳转或Python Pinocchio/Coal/MuJoCo。\n\n'
        '本轮只验收环境与原有离线实验，不代表跨机通信、碰撞门禁、起动摩擦或新分阶段标定已经实现。\n', encoding='utf-8')
    print(f"M1 evidence: PASS ({counts['pytest']} Python tests, {counts['ctest']} C++ tests, {compared} baseline metrics)")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    summarize(args.output)
