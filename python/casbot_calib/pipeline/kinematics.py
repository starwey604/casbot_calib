"""K stage orchestration: remount checkpoint, interactive acquisition, offline fit."""
from contextlib import nullcontext
from pathlib import Path
import json
from .robot import load_robot_config, find_executor
from .preflight import check_python_runtime, check_model_bundle, check_hardware_environment
from .session import Session
from .takeover import ensure_ready_channel
from ..project_paths import ROOT
from ..shm_mock import SHMMock
from ..model_context import load_context
from ..stages.kinematics import K1LeftStage, K2RightStage, K3IdentifyStage


def run_kinematics(*, stages, resume, resume_target, reanalyse, reanalyse_target, plan_only,
                   robot_id, stage_config, stage_overrides, robot_config, output_root,
                   endpoint, executor, mock, mock_config):
    check_python_runtime(require_solver=False, reanalyse_only=True)
    check_model_bundle()
    robot = load_robot_config(robot_config)
    context = load_context()
    if resume or reanalyse:
        session = Session.resume(output_root=output_root, target=reanalyse_target if reanalyse else resume_target,
                                 model_context=robot['model_context'], family='kinematics')
        if robot_id is not None and robot_id != session.robot_id:
            raise ValueError('机器编号与会话不一致')
    else:
        session = Session.create(output_root=output_root, robot_id=robot_id,
                                 model_context=robot['model_context'], family='kinematics',
                                 stages_requested=[s.name for s in stages], plan_only=plan_only)
    if reanalyse:
        for stage in (K1LeftStage(), K2RightStage()):
            root = session.root/stage.name
            stage.analyse(root/'plan', root/'capture', root/'analysis', session=session)
        stages = [K3IdentifyStage()]
    pending = [s for s in stages if reanalyse or not session.stage_completed(s.name)]
    recipes = {}
    for stage in pending:
        if stage.execution_kind != 'interactive':
            continue
        overrides = dict(stage_overrides or {})
        if resume and stage_config is None and (session.root/'k1/plan/plan.json').is_file():
            saved = json.loads((session.root/'k1/plan/plan.json').read_text())['recipe']
            overrides = {**saved, **overrides}
        recipes[stage.name] = stage.load_recipe(stage_config, **overrides)
    binary = None
    channel = None
    if not plan_only and any(s.execution_kind=='interactive' for s in pending):
        native = find_executor(executor)
        binary = native.with_name('shm_trical_executor')
        if not binary.is_file(): raise FileNotFoundError(f'请重建 SHM 工具：{binary}')
        if not mock:
            check_hardware_environment(endpoint=endpoint)
            channel = ensure_ready_channel(session.state_root, native, endpoint=endpoint,
                                           require_hand_background=any(s.requires_hand_background for s in pending
                                                                       if s.execution_kind=='interactive'))
    stopped = False
    for stage in pending:
        if stage.execution_kind=='offline':
            if plan_only:
                session.mark_stage(stage.name, 'planned')
                continue
            report = stage.analyse(None, None, session.root/stage.name, session=session)
            session.mark_stage(stage.name, 'completed', report_md=f'{stage.name}/report.md')
            if report.get('delivery'): session.metadata['delivery'] = report['delivery']
            session.save()
            print(f'[K3] 报告：{session.root/stage.name/"report.md"}', flush=True)
            continue
        root = session.prepare_stage_dir(stage.name)
        recipe = recipes[stage.name]
        print(f'[{stage.name.upper()}] 规划测点并检查连续路线……', flush=True)
        plan = stage.plan(context.home, robot, recipe, root/'plan', session=session)
        if plan_only:
            stage.write_summary(root, plan, None, None)
            session.mark_stage(stage.name, 'planned', report_md=f'{stage.name}/report.md')
            continue
        test_context = SHMMock(root/'mock', binary, mock_config or ROOT/'configs/shm/k_mock.json') if mock else nullcontext(None)
        try:
            with test_context as peer:
                active = peer.endpoint if peer else channel['endpoint']
                stage.execute(binary, active, robot, plan, root/'capture', mock=mock)
            report = stage.analyse(root/'plan', root/'capture', root/'analysis', session=session)
            stage.write_summary(root, plan, {}, report)
            session.mark_stage(stage.name, 'completed', report_md=f'{stage.name}/report.md',
                               capture_dir=f'{stage.name}/capture', plan_dir=f'{stage.name}/plan')
        except BaseException as error:
            session.mark_stage(stage.name, 'failed', reason=str(error))
            raise
        print(f'[{stage.name.upper()}] 采集完成：{root/"report.md"}', flush=True)
        if stage.stop_after and not mock:
            print('工装换至右腕，保持三表清零值；运行 bash calib/kinematics.sh --resume。', flush=True)
            stopped = True
            break
    return dict(session=session, completed=not stopped, plan_only=plan_only, stopped_for_remount=stopped)
