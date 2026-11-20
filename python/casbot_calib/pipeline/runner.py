"""Unified calibration pipeline runner: precheck -> takeover -> snapshot -> plan -> execute -> analyse."""
import argparse
import json
from pathlib import Path
import sys
import time
from ..shm_ready import ready_position
from ..shm_takeover import HandoffError
from ..stages.dynamics.common import resolve_effective_plan_and_capture
from .delivery import build_session_delivery
from .native import execute_with_start_replan, maybe_mock_context, native_snapshot
from .preflight import check_hardware_environment, check_model_bundle, check_native_binaries, check_python_runtime
from .robot import DEFAULT_ROBOT_CONFIG, load_robot_config
from .session import DEFAULT_OUTPUT_ROOT, Session
from .stage import resolve_stages
from .takeover import ensure_ready_channel


def _stage_checkpoint(session, stage_name, eff_plan_dir, eff_capture_dir, analysis_dir, report_path, report):
    fit_ok = bool(report.get('fit_succeeded', False))
    checkpoint = dict(
        dir=stage_name,
        plan_dir=str(eff_plan_dir.relative_to(session.root)),
        capture_dir=str(eff_capture_dir.relative_to(session.root)),
        analysis_dir=str(analysis_dir.relative_to(session.root)),
        report_md=str(report_path.relative_to(session.root)),
        fit_succeeded=fit_ok,
        accepted=bool(report.get('accepted', fit_ok)),
    )
    if report.get('candidate_urdf'):
        checkpoint['candidate_urdf'] = str((analysis_dir / report['candidate_urdf']).relative_to(session.root))
        checkpoint['calibration_json'] = str((analysis_dir / 'calibration.json').relative_to(session.root))
    return checkpoint


def reanalyse_session(
    *,
    stages='h1,h2,h3',
    target=None,
    robot_id=None,
    robot_config=DEFAULT_ROBOT_CONFIG,
    output_root=DEFAULT_OUTPUT_ROOT,
):
    """Re-run offline identification and delivery packaging on existing SHM captures without touching hardware."""
    stage_objs = resolve_stages(stages)
    check_python_runtime(require_solver=True, reanalyse_only=True)
    check_model_bundle()
    robot_cfg = load_robot_config(robot_config)

    session = Session.resume(output_root=output_root, target=target, model_context=robot_cfg['model_context'])
    if robot_id is not None and robot_id != session.metadata.get('robot_id'):
        raise ValueError(f'--robot={robot_id} 与目标会话机器编号 {session.metadata.get("robot_id")} 不一致')

    explicit_single = len(stage_objs) == 1
    targets = []
    for stage in stage_objs:
        rec = session.stage_record(stage.name) or {}
        stage_dir = session.root / stage.name
        has_capture = bool(rec.get('capture_dir')) or (stage_dir / 'capture').exists()
        if has_capture:
            targets.append(stage)
        elif explicit_single:
            raise ValueError(f'会话 {session.root} 中缺少阶段 {stage.name.upper()} 的采集记录，无法重分析')

    if not targets:
        raise ValueError(f'会话 {session.root} 中没有可重分析的已采集阶段')

    for stage in targets:
        stage_dir = session.root / stage.name
        rec = session.stage_record(stage.name) or {}
        raw_plan = session.root / rec['plan_dir'] if rec.get('plan_dir') else (stage_dir / 'plan')
        raw_capture = session.root / rec['capture_dir'] if rec.get('capture_dir') else (stage_dir / 'capture')
        eff_plan_dir, eff_capture_dir = resolve_effective_plan_and_capture(raw_plan, raw_capture)
        eff_plan_meta = json.loads((eff_plan_dir / 'plan.json').read_text(encoding='utf-8'))

        wrapper_path = (stage_dir / 'capture' / 'result.json')
        capture_summary = (
            json.loads(wrapper_path.read_text(encoding='utf-8'))
            if wrapper_path.is_file()
            else {'execution_completed': True}
        )

        analysis_dir = stage_dir / 'analysis'
        if analysis_dir.exists():
            archived = stage_dir / f'analysis.previous_{time.monotonic_ns()}'
            analysis_dir.rename(archived)

        report = stage.analyse(eff_plan_dir, eff_capture_dir, analysis_dir, session=session)
        report_path = stage.write_summary(stage_dir, eff_plan_meta, capture_summary, report)
        checkpoint = _stage_checkpoint(session, stage.name, eff_plan_dir, eff_capture_dir, analysis_dir, report_path, report)
        session.mark_stage(stage.name, 'completed', **checkpoint)
        print(f'[{stage.name.upper()}] 免动重分析完成，报告：{report_path}', flush=True)

    delivery = build_session_delivery(session, robot_cfg)
    session.metadata['delivery'] = delivery
    session.save()
    print(f'[DELIVERY] 交付包已更新：{session.root / delivery["summary_md"]}', flush=True)
    return dict(session=session, completed=True, plan_only=False, reanalysed=True, delivery=delivery)


def run_pipeline(
    *,
    stages='h1,h2,h3',
    resume=False,
    resume_target=None,
    reanalyse=False,
    reanalyse_target=None,
    plan_only=False,
    robot_id=None,
    stage_config=None,
    stage_overrides=None,
    robot_config=DEFAULT_ROBOT_CONFIG,
    hold_body=False,
    output_root=DEFAULT_OUTPUT_ROOT,
    endpoint='/shm_mc',
    executor=None,
    hand_background=None,
    mock=False,
    mock_config=None,
):
    stage_objs = resolve_stages(stages)
    if stage_objs[0].family == 'kinematics':
        if hold_body:
            raise ValueError('--hold-body 仅用于动力学阶段')
        from .kinematics import run_kinematics
        return run_kinematics(stages=stage_objs, resume=resume, resume_target=resume_target,
            reanalyse=reanalyse, reanalyse_target=reanalyse_target, plan_only=plan_only,
            robot_id=robot_id, stage_config=stage_config, stage_overrides=stage_overrides,
            robot_config=robot_config, output_root=output_root, endpoint=endpoint,
            executor=executor, mock=mock, mock_config=mock_config)
    if reanalyse:
        if plan_only or resume or mock:
            raise ValueError('--reanalyse 仅对已有实测/采集会话离线重算，不与 --plan-only / --resume / --mock 同时使用')
        return reanalyse_session(
            stages=stages,
            target=reanalyse_target,
            robot_id=robot_id,
            robot_config=robot_config,
            output_root=output_root,
        )

    stage_objs = resolve_stages(stages)
    is_stage_map = isinstance(stage_config, dict) and bool(stage_config) and set(stage_config) <= {'h1', 'h2', 'h3'}
    if stage_config is not None and not is_stage_map and len(stage_objs) != 1:
        raise ValueError('--config 仅在单阶段运行时使用')
    if resume and plan_only:
        raise ValueError('--resume 与 --plan-only 不能同时使用')
    if mock and endpoint != '/shm_mc':
        raise ValueError('--mock 自动创建私有测试端点，不同时指定 --endpoint')

    unimplemented = [s.name.upper() for s in stage_objs if not s.implemented]
    if unimplemented:
        raise NotImplementedError(
            f'阶段 {", ".join(unimplemented)} 尚未实现；当前可运行 bash calib/dynamics_h1.sh（或 --stages h1）'
        )

    check_python_runtime(require_solver=True)
    check_model_bundle()
    robot_cfg = load_robot_config(robot_config, hold_body=hold_body)
    overrides = dict(stage_overrides or {})
    recipes = {
        s.name: s.load_recipe(
            stage_config.get(s.name, s.default_config) if is_stage_map
            else (stage_config if stage_config is not None else s.default_config),
            **overrides,
        )
        for s in stage_objs
    }

    binary = None
    if not plan_only and not resume:
        binary = check_native_binaries(executor=executor, mock=mock, endpoint=endpoint)
        if not mock:
            check_hardware_environment(endpoint=endpoint)

    if resume:
        session = Session.resume(output_root=output_root, target=resume_target, model_context=robot_cfg['model_context'])
        if robot_id is not None and robot_id != session.metadata.get('robot_id'):
            raise ValueError(f'--robot={robot_id} 与待续跑会话机器编号 {session.metadata.get("robot_id")} 不一致')
    else:
        session = Session.create(
            output_root=output_root,
            robot_id=robot_id,
            model_context=robot_cfg['model_context'],
            stages_requested=[s.name for s in stage_objs],
            plan_only=plan_only,
        )

    pending = [s for s in stage_objs if not (resume and session.stage_completed(s.name))]
    if not pending:
        delivery = build_session_delivery(session, robot_cfg)
        session.metadata['delivery'] = delivery
        session.save()
        print(f'所选阶段均已完成：{session.root}', flush=True)
        return dict(session=session, completed=True, skipped_all=True, delivery=delivery)

    if plan_only:
        nominal_start = ready_position()
        for stage in pending:
            stage_dir = session.prepare_stage_dir(stage.name)
            plan_dir = stage_dir / 'plan'
            plan_meta = stage.plan(nominal_start, robot_cfg, recipes[stage.name], plan_dir, session=session)
            report_path = stage.write_summary(stage_dir, plan_meta, None, None)
            session.mark_stage(
                stage.name,
                'planned',
                dir=stage.name,
                duration_s=plan_meta['duration_ns'] * 1e-9,
                report_md=str(report_path.relative_to(session.root)),
            )
            print(f'[{stage.name.upper()}] 离线规划与连续碰撞检查通过：{report_path}', flush=True)
        return dict(session=session, completed=True, plan_only=True)

    if binary is None:
        binary = check_native_binaries(executor=executor, mock=mock, endpoint=endpoint)
        if not mock:
            check_hardware_environment(endpoint=endpoint)

    channel_info = None
    if not mock:
        channel_info = ensure_ready_channel(
            session.state_root,
            binary,
            endpoint=endpoint,
            explicit_hand_background=hand_background,
        )

    for stage in pending:
        stage_dir = session.prepare_stage_dir(stage.name)
        recipe = recipes[stage.name]
        print(f'[{stage.name.upper()}] 开始 {stage.title}（会话目录：{stage_dir}）', flush=True)
        with maybe_mock_context(mock, stage_dir, binary, robot_cfg, mock_config=mock_config) as mock_ctx:
            active_endpoint = mock_ctx.endpoint if mock_ctx is not None else channel_info['endpoint']
            active_bg = (
                Path(hand_background).resolve()
                if hand_background is not None
                else (mock_ctx.background if mock_ctx is not None else channel_info['hand_background'])
            )
            peer = mock_ctx.process if mock_ctx is not None else None

            q_start = native_snapshot(binary, active_endpoint, robot_cfg, stage_dir / 'snapshot', peer=peer)
            plan_dir = stage_dir / 'plan'
            plan_meta = stage.plan(q_start, robot_cfg, recipe, plan_dir, session=session)

            def replan_from_start(fresh_q, target_plan_dir, _stage=stage, _recipe=recipe):
                return _stage.plan(fresh_q, robot_cfg, _recipe, target_plan_dir, session=session)

            code, capture_summary, eff_plan_dir, eff_plan_meta, eff_capture_dir = execute_with_start_replan(
                initial_plan_dir=plan_dir,
                initial_plan_meta=plan_meta,
                replan_fn=replan_from_start,
                capture_root=stage_dir / 'capture',
                executor=binary,
                endpoint=active_endpoint,
                robot_config=robot_cfg,
                hand_background=active_bg,
                peer=peer,
            )

        if code != 0 or not capture_summary.get('execution_completed'):
            reason = capture_summary['attempts'][-1].get('reason') or f'returncode={code}'
            session.mark_stage(stage.name, 'failed', dir=stage.name, returncode=code, reason=reason)
            raise RuntimeError(f'{stage.title} 执行中止（{reason}），已请求失能并结束会话：{stage_dir}')

        analysis_dir = stage_dir / 'analysis'
        report = stage.analyse(eff_plan_dir, eff_capture_dir, analysis_dir, session=session)
        report_path = stage.write_summary(stage_dir, eff_plan_meta, capture_summary, report)
        checkpoint = _stage_checkpoint(session, stage.name, eff_plan_dir, eff_capture_dir, analysis_dir, report_path, report)
        session.mark_stage(stage.name, 'completed', **checkpoint)
        print(f'[{stage.name.upper()}] 完成，报告：{report_path}', flush=True)

    delivery = build_session_delivery(session, robot_cfg)
    session.metadata['delivery'] = delivery
    session.save()
    print(f'[DELIVERY] 交付包已生成：{session.root / delivery["summary_md"]}', flush=True)
    return dict(session=session, completed=True, plan_only=False, delivery=delivery)


def build_parser():
    p = argparse.ArgumentParser(description='CASBOT 双臂标定流水线（SHM 原生 1kHz）')
    p.add_argument('--stages', default='h1,h2,h3', help='运行阶段列表，逗号分隔（h1,h2,h3 或 k1,k2,k3）')
    p.add_argument('--resume', nargs='?', const='latest', default=None, metavar='SESSION_DIR',
                   help='从最近一次（或指定目录）未完成的会话继续执行')
    p.add_argument('--reanalyse', nargs='?', const='latest', default=None, metavar='SESSION_DIR',
                   help='免接管、免动机器人，对最近一次（或指定目录）采集会话重新执行辨识与交付打包')
    p.add_argument('--plan-only', action='store_true', help='只生成轨迹并完成连续碰撞/限位/速度检查，不连接设备')
    p.add_argument('--robot', default=None, metavar='ID', help='机器序列号/标识（首次指定后自动记住）')
    p.add_argument('--config', type=Path, default=None, help='自定义配方 YAML（动力学仅单阶段，运动学共用）')
    p.add_argument('--mode', choices=('small', 'full', 'smoke'), default=None, help='标定模式覆盖（small/full；smoke 适用于 H1 和运动学）')
    p.add_argument('--hold-body', action='store_true', help='采集期间原地保持腿、头与腰')
    p.add_argument('--smoke', action='store_true', help='在 H1 静态采集前加入一次准备位往返冒烟（或单独跑冒烟）')

    # Internal dev/test hooks hidden from operator --help.
    p.add_argument('--mock', action='store_true', help=argparse.SUPPRESS)
    p.add_argument('--mock-config', type=Path, default=None, help=argparse.SUPPRESS)
    p.add_argument('--endpoint', default='/shm_mc', help=argparse.SUPPRESS)
    p.add_argument('--executor', type=Path, default=None, help=argparse.SUPPRESS)
    p.add_argument('--hand-background', type=Path, default=None, help=argparse.SUPPRESS)
    p.add_argument('--output-root', type=Path, default=DEFAULT_OUTPUT_ROOT, help=argparse.SUPPRESS)
    p.add_argument('--robot-config', type=Path, default=DEFAULT_ROBOT_CONFIG, help=argparse.SUPPRESS)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    overrides = {}
    if args.mode is not None:
        overrides['mode'] = args.mode
    if args.smoke:
        overrides['smoke_enabled'] = True
    try:
        result = run_pipeline(
            stages=args.stages,
            resume=args.resume is not None,
            resume_target=args.resume,
            reanalyse=args.reanalyse is not None,
            reanalyse_target=args.reanalyse,
            plan_only=args.plan_only,
            robot_id=args.robot,
            stage_config=args.config,
            stage_overrides=overrides,
            robot_config=args.robot_config,
            hold_body=args.hold_body,
            output_root=args.output_root,
            endpoint=args.endpoint,
            executor=args.executor,
            hand_background=args.hand_background,
            mock=args.mock,
            mock_config=args.mock_config,
        )
        print(f'会话目录：{result["session"].root}', flush=True)
        return 0
    except (ValueError, OSError, KeyError, NotImplementedError, HandoffError, RuntimeError) as error:
        print(f'标定中止：{error}', file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
