"""Read-only field checks and explicit zero motion; GUI-independent services."""
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
        self.console.print('本报告仅为只读观测；实物映射/方向由现场核对。')
        self.console.print('报告：' + str(Path(output).resolve()/'report.md'))


def parser():
    result = argparse.ArgumentParser(prog='casbot-fieldwork', description='现场联调：只读检查、CLI/GUI保存零位与显式回零、GUI五字段控制。')
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
        item.add_argument('--imu-topic', help='可选IMU只读话题，例如 /motion/imu；Mock须私有命名空间')
        item.add_argument('--target-label', default='', help='操作者对现场目标的备注，不作为设备身份')
        item.add_argument('--interactive', action='store_true', help='允许输入d/r/n备注/q并回车登记操作或结束')
        item.add_argument('--json', action='store_true', help='stdout仅输出JSON；不可与interactive并用')
    report = commands.add_parser('report', help='从已有会话只读重分析，不启动ROS')
    report.add_argument('folder', type=Path)
    report.add_argument('--output', type=Path, required=True, help='源会话之外的新目录')
    report.add_argument('--json', action='store_true')
    evidence = commands.add_parser('a-evidence', help='从fieldwork保存记录整理A路线判断证据')
    evidence.add_argument('--source', type=Path, required=True)
    evidence.add_argument('--output', type=Path, required=True)
    evidence.add_argument('--joint', type=int, default=13, choices=range(14))
    evidence.add_argument('--window', nargs=2, type=float, metavar=('START', 'END'), help='相对首条实际发送指令的秒数；指定后计算局部A及错位敏感性')
    evidence.add_argument('--band', type=float, default=30., help='共同处理带宽Hz，默认30')
    export = commands.add_parser('export-readback', help='按需导出指定运行的回读CSV，不复制命令和反馈')
    export.add_argument('--run', type=Path, required=True, help='controls/*/runs/*运行目录')
    export.add_argument('--output', type=Path, required=True, help='源运行之外的新CSV文件')
    export.add_argument('--start', type=float, default=0., help='相对首条joint/debug接收时刻的秒数')
    export.add_argument('--end', type=float, help='结束秒数，不包含终点；默认导出到末尾')
    export.add_argument('--joint', type=int, action='append', choices=range(14), help='轴索引，可重复指定；默认全部14轴')
    sensitivity = commands.add_parser('sensitivity', help='只读比较已有SDK探针的A/B标定敏感性，不连接机器人')
    sensitivity.add_argument('--source', type=Path, action='append', required=True, help='封存探针会话，可重复指定')
    sensitivity.add_argument('--output', type=Path, required=True, help='全部源会话之外的新目录')
    sensitivity.add_argument('--reference-assessment', type=Path, help='复用已有同配方评估，比较独立会话')
    for name, label in [('zero-capture', '将当前实际14轴位置保存为工作区零位（只读）'),
                        ('zero-return', '显式发送单程轨迹到已保存零位')]:
        item = commands.add_parser(name, help=label)
        item.add_argument('--profile', type=Path, default=Path('configs/hardware_field.yaml'))
        item.add_argument('--domain-id', type=int, required=True)
        item.add_argument('--scope', choices=('hardware_position_stream', 'localhost_contract_mock'), required=True)
        if name == 'zero-return':
            item.add_argument('--duration', type=float, default=5., help='回零过渡至少用时/秒，按参考速度包络可自动延长')
            item.add_argument('--kp', type=float, default=20.)
            item.add_argument('--kd', type=float, default=2.)
    gui = commands.add_parser('gui', help='GUI五字段滑条调参；开始发送后拖动会实际发送SDK指令')
    gui.add_argument('--profile', type=Path, default=Path('configs/hardware_field.yaml'))
    gui.add_argument('--output', type=Path, help='可选：显式运行工作区；省略时自动缓存，GUI内手动保存')
    gui.add_argument('--session', type=Path, help='已有运行工作区或只读快照；不会自动连接或发送')
    gui.add_argument('--mock-diagnostics',action='store_true',help='自建Mock额外记录模型内部高频诊断；原始SDK记录始终开启')
    gui.add_argument('--mock',action='store_true',help='显式启动GUI独占的本地SDK Mock，自动配对profile')
    gui.add_argument('--recover',nargs='?',const='latest',help='打开自动缓存记录，默认最近一次；不会自动连接或发送')
    gui.add_argument('--cache-limit-mib',type=float,default=512.,help='GUI记录容量/MiB，默认512；含显式output与自有Mock，满后停录不停发')
    gui.add_argument('--software-rendering',action='store_true',help='显式使用Mesa软件渲染，适用于容器显卡驱动不兼容')
    gui.add_argument('--domain-id', type=int)
    gui.add_argument('--scope', choices=('hardware_position_stream', 'localhost_contract_mock'))
    gui.add_argument('--preview', action='store_true', help='纯离线界面/模型预览，所有连接和控制按钮禁用')
    gui.add_argument('--frames', type=int, help='指定帧数后关闭，用于UI检查')
    gui.add_argument('--screenshot', type=Path, help='关闭时保存实际窗口截图')
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    if args.command == 'export-readback':
        from .fieldwork_control import export_readback
        try:
            result=export_readback(args.run,args.output,start_s=args.start,end_s=args.end,joints=args.joint)
            print(f"已导出{result['rows']}条回读：{result['output']}")
            return 0
        except (OSError,ValueError) as exc:
            print('回读导出：'+str(exc),file=sys.stderr);return 1
    if args.command == 'a-evidence':
        from .fieldwork_a_evidence import run
        try:
            report = run(args.source, args.output, joint=args.joint, window=args.window, band=args.band)
            print(report['summary']+'；报告：'+str(args.output.resolve()/'report.md'))
            return 0
        except (OSError, ValueError, RuntimeError) as exc:
            print('A路线证据：'+str(exc), file=sys.stderr); return 1
    if args.command == 'sensitivity':
        from .sdk_feedback_sensitivity import main as sensitivity_main
        extra = ['--reference-assessment', str(args.reference_assessment)] if args.reference_assessment else []
        return sensitivity_main([item for source in args.source for item in ('--source', str(source))]
                                + ['--output', str(args.output)] + extra)
    if args.command in ('zero-capture', 'zero-return'):
        try:
            from .fieldwork_zero import capture_zero, return_zero, ZERO_FILE
            from .hardware_field import load_profile
            config = load_profile(args.profile, manual=args.command=='zero-capture')
            print('零位文件：'+str(ZERO_FILE), flush=True)
            if args.command == 'zero-capture':
                print('正在只读采集实际位置；不会写入编码器零位或申请运动控制。', flush=True)
                result = capture_zero(config, args.domain_id, args.scope)
                print('已更新保存零位。记录：'+result['recording'])
            else:
                result = return_zero(config, args.domain_id, args.scope, duration_s=args.duration, kp=args.kp, kd=args.kd)
                reason = result['result'].get('fault') or result['result'].get('reason')
                print(('回零参考已发送并退出模式' if result['passed'] else '回零未完成：'+str(reason))+'；记录：'+result['recording'])
            return 0 if result['passed'] else 1
        except (OSError, ValueError, RuntimeError) as exc:
            print(f'现场零位：{exc}', file=sys.stderr); return 1
        except KeyboardInterrupt:
            print('零位操作已中断。', file=sys.stderr); return 130
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
                notify=terminal.event if terminal else lambda event: None, imu_topic=args.imu_topic)
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
