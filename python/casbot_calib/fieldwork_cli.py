"""Human-facing read-only field CLI; analysis/orchestration are GUI-independent."""
import argparse
import json
from pathlib import Path
import sys


class Terminal:
    def __init__(self):
        try:
            from rich.console import Console
        except ImportError as exc:
            raise RuntimeError('请安装现场终端依赖：python -m pip install rich==14.1.0') from exc
        self.console = Console(markup=False, highlight=False)

    def event(self, event):
        labels = dict(first_valid_feedback='已收到完整14轴位置反馈', valid_feedback_timeout='有效位置反馈超时',
            initial_feedback_timeout='尚未收到完整14轴位置反馈，继续只读观测',
            valid_feedback_resumed='已恢复接收有效位置反馈', publisher_set_observed='目标topic发布者快照已更新',
            operator_disconnect='已登记断开操作', operator_reconnect='已登记恢复操作',
            operator_note='已登记备注', operator_finish='正在结束采集并保存报告')
        self.console.print(labels[event['event']], style='yellow' if event['event'] == 'valid_feedback_timeout' else 'cyan')

    def report(self, report, output):
        from rich.table import Table
        from rich.text import Text
        from .fieldwork_analysis import ITEM_LABELS, STATUS_LABELS
        table = Table(title='现场只读检查 · ' + STATUS_LABELS[report['status']])
        for name in ('项目', '状态', '结论'):
            table.add_column(name)
        for check in report['checks']:
            table.add_row(Text(ITEM_LABELS[check['item_id']]), Text(STATUS_LABELS[check['status']]), Text(check['conclusion']))
        self.console.print(table)
        for check in report['checks']:
            if check['status'] != 'passed':
                self.console.print(ITEM_LABELS[check['item_id']] + '：' + check['next_action'])
        primary = report['channels'].get('joint')
        if primary:
            hz = primary['receive']['hz']
            hz_text = f'{hz:.1f} Hz' if hz is not None else '未知'
            self.console.print(f'主反馈：{primary["receive"]["count"]}条；接收频率：{hz_text}（非电机采样率）')
            axes = Table(title='全轴位置概览 · rad')
            for name in ('SDK名称', '目标臂轴', '首值', '末值', '跨度'):
                axes.add_column(name)
            for axis in primary['axes']:
                axes.add_row(Text(axis['name']), '是' if axis['target_arm'] else '否',
                    *(f'{axis[key]:.6f}' for key in ('first_rad', 'last_rad', 'span_rad')))
            self.console.print(axes)
        self.console.print('实物映射/方向待F2确认；运动未开放。')
        self.console.print('报告：' + str(Path(output).resolve()/'report.md'))


def parser():
    result = argparse.ArgumentParser(prog='casbot-fieldwork', description='现场联调：只读预检/采集/报告，以及显式GUI逐轴控制。')
    commands = result.add_subparsers(dest='command', required=True)
    for name, label, duration in [('preflight', '环境、发现与反馈预检', 5.),
                                  ('observe', '全轴原始采集与接收节奏报告', 30.),
                                  ('reconnect', '只读观测反馈中断及恢复，不恢复运动', 120.)]:
        item = commands.add_parser(name, help=label)
        item.add_argument('--config', type=Path, default=Path('configs/vendor_s1.yaml'), help='S1只读配置')
        item.add_argument('--domain-id', type=int, required=True, help='现场确认的域号；无默认或扫描')
        item.add_argument('--scope', choices=('hardware_read_only', 'localhost_contract_mock'), required=True)
        item.add_argument('--output', type=Path, required=True, help='必须不存在的会话目录')
        item.add_argument('--duration', type=float, default=duration, help='采集时长/秒')
        item.add_argument('--timeout', type=float, help='有效位置反馈超时/秒；默认沿用S1配置')
        item.add_argument('--target-label', default='', help='操作者对现场目标的备注，不作为设备身份')
        item.add_argument('--interactive', action='store_true', help='允许输入d/r/n备注/q并回车登记操作或结束')
        item.add_argument('--json', action='store_true', help='stdout仅输出JSON；不可与interactive并用')
    report = commands.add_parser('report', help='从已有会话只读重分析，不启动ROS')
    report.add_argument('folder', type=Path)
    report.add_argument('--output', type=Path, required=True, help='源会话之外的新目录')
    report.add_argument('--json', action='store_true')
    gui = commands.add_parser('gui', help='GUI逐轴确认与短轨迹；执行按钮会实际发送SDK指令')
    gui.add_argument('--profile', type=Path, default=Path('configs/hardware_field.yaml'))
    gui.add_argument('--output', type=Path, help='新GUI会话目录')
    gui.add_argument('--session', type=Path, help='已有GUI会话；故障会话仍禁止运动')
    gui.add_argument('--domain-id', type=int)
    gui.add_argument('--scope', choices=('hardware_position_stream', 'localhost_contract_mock'))
    gui.add_argument('--conditions', type=Path, help='新实机会话的现场前提证据登记')
    gui.add_argument('--preview', action='store_true', help='纯离线界面/模型预览，所有连接和控制按钮禁用')
    gui.add_argument('--frames', type=int, help='指定帧数后关闭，用于UI检查')
    gui.add_argument('--screenshot', type=Path, help='关闭时保存实际窗口截图')
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    if args.command == 'gui':
        try:
            from .fieldwork_gui import run_gui
            return run_gui(args)
        except (OSError, ValueError, RuntimeError, ImportError) as exc:
            print(f'现场GUI：{exc}', file=sys.stderr)
            return 1
    if args.command != 'report' and args.json and args.interactive:
        parser().error('--json不能与--interactive并用')
    try:
        terminal = None if args.json else Terminal()
        if args.command == 'report':
            from .fieldwork_analysis import save_report
            report = save_report(args.folder, args.output)
        else:
            from .fieldwork import collect
            if terminal:
                terminal.console.print(f'只读会话 · 域号 {args.domain_id} · {args.scope} · {args.duration:g}秒')
                if args.interactive:
                    terminal.console.print('输入并回车：d 已断开；r 已恢复连接；n 备注；q 提前结束。仅登记操作，不控制网络或机器人。')
            report = collect(args.config, args.output, command=args.command, domain_id=args.domain_id,
                scope=args.scope, duration_s=args.duration, timeout_s=args.timeout,
                target_label=args.target_label, interactive=args.interactive,
                notify=terminal.event if terminal else lambda event: None)
        if terminal:
            terminal.report(report, args.output)
        else:
            print(json.dumps(report, ensure_ascii=False, allow_nan=False))
        return dict(passed=0, failed=1, blocked=2)[report['status']]
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'现场只读CLI：{exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
