# CASBOT Calibration

CASBOT 双臂（14轴）动力学/运动学标定。硬件路径为 Orin 本机 EtherCAT 中间件共享内存（SHM，`/shm_mc`）：C++ `shm_executor` 负责 1kHz 执行与完整记录，Python 负责轨迹规划、碰撞检查、参数辨识与流程编排。

## 快速开始

在 Orin（`/home/nvidia/casbot_calib_shm`）上执行：

```bash
# Orin 首次部署或代码更新后初始化/重编环境（安装至 install/shm-core 并完成预检）
bash calib/setup.sh

# H1 静态重力标定（自动预检 -> 接管主站 -> 准备位 -> 静态采集 -> PSD 拟合）
bash calib/dynamics_h1.sh

# 全阶段动力学标定（H1 静态重力 -> H2 摩擦 -> H3 惯量）
bash calib/dynamics.sh

# 运动学：K1 左臂；换装工装后续跑 K2 与 K3
bash calib/kinematics.sh
bash calib/kinematics.sh --resume
```

动力学结果在 `artifacts/calib/latest_dynamics/`（查看 `h1/report.md` 与 `h1/analysis/candidate.urdf`）；运动学结果在 `artifacts/calib/latest_kinematics/`（查看 `k3/report.md`，full 模式另有 `delivery/summary.md`）。

## 文档

- [运动学标定操作指南](docs/kinematics.md)：三表配置、K1/K2 换装续跑、K3 辨识与交付。
- [动力学标定操作指南](docs/dynamics.md)：运动前检查、运行命令、配方配置、结果判读。
- [调参 GUI](docs/shm_gui.md)：本机渲染，经 SSH 连接 Orin（`bash calib/gui.sh --host nvidia@172.16.0.10`）。
- [构建与部署](docs/shm_build.md)：本机 Humble 容器与 Orin 原生编译说明。
- [开发与底层现场工具](fieldwork/shm/README.md)：`bash calib/dev.sh` 诊断与 C00–C06 底层用例。
- `docs/records/`：历次实机验证与现场观测记录。

## 构建与测试

```bash
# 本机（Humble 容器提供 Pinocchio/Coal 工具链）
bash scripts/humble/container.sh bash -c 'cmake --build --preset humble && ctest --preset humble' < /dev/null
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q < /dev/null

# Orin 原生构建（完整 Python 数值环境 + C++ 执行器）
bash calib/setup.sh

# 仅构建 C++ SHM 执行器与物理 Mock（默认输出至 install/shm-core）
bash scripts/shm/build.sh --with-mock
```

## 目录结构

- `calib/`：交付入口脚本（`setup.sh`、`dynamics.sh`、`dynamics_h1.sh`..`h3.sh`、`kinematics.sh`、`gui.sh`、`dev.sh`）。
- `configs/`：机器人参数（`robot/dvt2.yaml`）与各阶段配方（`dynamics/h1.yaml`..`h3.yaml`、`kinematics/k.yaml`）。
- `cpp/`：动力学/碰撞核心、SHM 协议与 1kHz 原生执行器。
- `python/casbot_calib/`：
  - `pipeline/`：会话管理、自动接管、原生执行器封装、统一运行器。
  - `stages/dynamics/`：H1 静态重力、H2 摩擦、H3 惯量阶段实现。
  - `stages/kinematics/`、`trical/`：K1/K2 采集、K3 辨识及三表、路线与求解核心。
- `scripts/`：构建、同步与环境脚本。

旧 ROS/SDK 实现及离线研究代码已归档至 `ros` 分支。`artifacts/`、`build/`、`install/` 不入 Git。
