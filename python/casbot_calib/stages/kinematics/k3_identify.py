"""Offline K3: nominal diagnostics for quick modes, two-layer fitting for full."""
import json
from pathlib import Path
import numpy as np
import yaml
from ...data import write_json
from ...model_context import load_context
from ...pipeline.stage import Stage
from ...trical.solver import JointFixtureFit, export_geometry, FIXTURE_SCALES
from .common import load_recipe, DEFAULT_CONFIG


class K3IdentifyStage(Stage):
    name = 'k3'
    title = '联合辨识'
    family = 'kinematics'
    execution_kind = 'offline'
    requires_hand_background = False
    default_config = DEFAULT_CONFIG

    def load_recipe(self, config_path=None, **overrides):
        return load_recipe(config_path, **overrides)

    def analyse(self, plan_dir, capture_dir, analysis_dir, *, session=None):
        root = session.root
        rows = []
        for stage in ('k1','k2'):
            rows.extend(json.loads((root/stage/'measurements.json').read_text()))
        recipe = json.loads((root/'k1/plan/plan.json').read_text())['recipe']
        context = load_context()
        fit = JointFixtureFit(context.engine, rows)
        residual, _ = fit.evaluate(np.zeros(fit.size))
        lines = ['# K3 联合辨识', '', f'模式 `{recipe["mode"]}`，测点数 {len(rows)}。', '',
                 '## 名义模型残差', '', '| 手臂 / 球 | 点数 | 球心 RMSE (mm) |', '|---|---:|---:|']
        for side in ('left','right'):
            for ball in 'ABCD':
                mask = [r['arm']==side and r['ball']==ball for r in rows]
                if any(mask):
                    rmse = np.sqrt(np.mean(np.sum(residual[mask]**2, axis=1)))
                    lines.append(f'| {side} / {ball} | {sum(mask)} | {rmse*1000:.4f} |')
        report = dict(mode=recipe['mode'], measurement_count=len(rows), nominal_residual_m=residual.tolist())
        destination = root/'k3'; destination.mkdir(exist_ok=True)
        if recipe['mode']=='full':
            result = fit.fit(recipe['condition_limit']); report.update(result)
            x = np.array(result['parameters_scaled'])
            offsets = x[:14]*fit.parameters.scales[:14]
            geometry = np.r_[np.zeros(14), x[19:]]
            delivery = root/'delivery'; delivery.mkdir(exist_ok=True)
            check = export_geometry(context.engine, fit.parameters.build(geometry), context.urdf, delivery/'calibrated.urdf')
            (delivery/'joint_offsets.yaml').write_text(yaml.safe_dump(dict(
                units='rad', convention='q_true = q_encoder + offset',
                offsets={12+i: float(v) for i,v in enumerate(offsets)}), sort_keys=False))
            bundle = dict(result, fixture_correction=(x[14:19]*FIXTURE_SCALES).tolist(),
                          joint_offsets_rad=offsets.tolist(), geometry_parameters_scaled=geometry.tolist(),
                          parameter_definitions=fit.parameters.definitions, export_check=check,
                          sources=['k1/measurements.json','k2/measurements.json'])
            write_json(delivery/'calibration_bundle.json', bundle)
            lines += ['', '## 留出验证', '', '| 层级 | 左臂 RMSE (mm) | 右臂 RMSE (mm) |', '|---|---:|---:|']
            for layer, score in result['holdout_rmse_m'].items():
                lines.append(f'| {layer} | {score["left"]*1000:.4f} | {score["right"]*1000:.4f} |')
            score = result['holdout_rmse_m'][result['selected_layer']]
            lines += ['', f'交付层级：`{result["selected_layer"]}`。',
                      f'留出 RMSE：内部目标 {"通过" if max(score.values())<=.0004 else "未通过"}；交付阈值 {"通过" if max(score.values())<=.0007 else "未通过"}。',
                      f'URDF 重加载复核：{"通过" if check["passed"] else "发现差异，见 calibration_bundle.json"}。',
                      '', '台架修正 [tx, ty, tz, rx, rz]（m / rad）：`'+str(bundle['fixture_correction'])+'`。',
                      '', '最大读表极差：'+f'{max(max(r["reading_span_m"]) for r in rows)*1e6:.3f} µm。',
                      '', '## 关节零偏', '', '| SHM ID | offset (rad) |', '|---|---:|']
            lines += [f'| {i+12} | {v:.9g} |' for i,v in enumerate(offsets)]
            (delivery/'summary.md').write_text('\n'.join(lines)+'\n')
            report['delivery'] = dict(summary_md='delivery/summary.md')
            report['accepted'] = max(score.values())<=.0007
        (destination/'report.md').write_text('\n'.join(lines)+'\n')
        write_json(destination/'report.json', report)
        return report
