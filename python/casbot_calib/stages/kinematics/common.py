"""K1/K2 acquisition stages with shared recipe and capture analysis."""
import json
from pathlib import Path
import yaml
from ...data import write_json
from ...project_paths import ROOT
from ...pipeline.stage import Stage
from ...trical.routes import Routes, design_points
from ...trical.controller import acquire, analyse_capture

DEFAULT_CONFIG = ROOT/'configs/kinematics/k.yaml'


def load_recipe(path=None, **overrides):
    recipe = yaml.safe_load(Path(path or DEFAULT_CONFIG).read_text())
    if overrides.pop('smoke_enabled', False):
        raise ValueError('运动学冒烟请使用 --mode smoke')
    recipe.update(overrides)
    if recipe['mode'] not in ('smoke', 'small', 'full'):
        raise ValueError('运动学模式须为 smoke/small/full')
    return recipe


class AcquisitionStage(Stage):
    family = 'kinematics'
    execution_kind = 'interactive'
    requires_hand_background = False
    default_config = DEFAULT_CONFIG

    def load_recipe(self, config_path=None, **overrides):
        return load_recipe(config_path, **overrides)

    def plan(self, q_start, robot_config, recipe, plan_dir, *, session=None):
        self.routes = Routes(self.side)
        points = [p for p in design_points(recipe['mode'], self.routes.engine, self.routes) if p['arm']==self.side]
        self.routes.validate(points)
        meta = dict(mode=recipe['mode'], arm=self.side, points=points, recipe=recipe)
        Path(plan_dir).mkdir(parents=True, exist_ok=True)
        write_json(Path(plan_dir)/'plan.json', meta)
        return meta

    def execute(self, binary, endpoint, robot_config, plan, capture, *, mock=False):
        return acquire(binary, endpoint, self.routes, robot_config, plan['recipe'], plan['points'], capture, mock=mock)

    def analyse(self, plan_dir, capture_dir, analysis_dir, *, session=None):
        rows = analyse_capture(capture_dir)
        root = Path(analysis_dir); root.mkdir(parents=True, exist_ok=True)
        write_json(root.parent/'measurements.json', rows)
        return dict(measurement_count=len(rows), fit_succeeded=False, accepted=True)

    def write_summary(self, stage_dir, plan_meta, capture_summary, report):
        report_path = Path(stage_dir)/'report.md'
        status = '采集完成' if report else '路线检查完成'
        text = f'# {self.name.upper()} {self.title}\n\n{status}；模式 `{plan_meta["mode"]}`，{len(plan_meta["points"])} 个测点。\n'
        if self.stop_after:
            text += '\n将工装从左腕换至右腕，保持三表清零值，运行 `bash calib/kinematics.sh --resume`。\n'
        report_path.write_text(text)
        return report_path
