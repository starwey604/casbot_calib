"""Local NMPC commands."""
import argparse


def main():
    parser = argparse.ArgumentParser(description='DVT2 双臂 NMPC')
    commands = parser.add_subparsers(dest='action', required=True)
    commands.add_parser('import', help='生成双臂模型并审计每个碰撞对')
    commands.add_parser("build", help="生成并编译本机 NMPC 求解器")
    commands.add_parser("sim", help="本机双臂运动学 MuJoCo 仿真")
    mock = commands.add_parser('mock', help='私有 SHM 动力学 Mock 与 MuJoCo 交互')
    teleop = commands.add_parser('teleop', help='经指定 Orin 的 SHM owner 遥操作')
    teleop.add_argument('--host', required=True)
    teleop.add_argument('--remote-repo', default='/home/nvidia/casbot_calib_shm')
    for command in (mock, teleop):
        command.add_argument('--delivery', help='动力学标定交付目录')
        command.add_argument('--no-ff', action='store_true', help='关闭逆动力学前馈')
    args = parser.parse_args()
    if args.action in ('mock', 'teleop'):
        from .bridge import physical_simulation
        connection = dict(host=args.host, repo=args.remote_repo) if args.action == 'teleop' else {}
        physical_simulation(**connection, delivery=args.delivery, feedforward=not args.no_ff)
        return
    if args.action == 'sim':
        from .bridge import simulate
        simulate()
        return
    if args.action == "build":
        from .build import build
        print(build())
        return
    from .import_dvt2 import OUTPUT, write_model
    model = write_model()
    print(f"{OUTPUT}  ·  {len(model['pairs'])} 对")
    for mode in model['config']['clearance_m']:
        count = sum(p['modes'][mode]['geometry'] == 'convex' for p in model['pairs'])
        print(f'{mode}: {count} 对使用原始凸包')


if __name__ == '__main__':
    main()
