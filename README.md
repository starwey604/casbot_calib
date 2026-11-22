# CASBOT Calibration

CASBOT 双臂（14 轴）动力学与运动学标定工具。Orin 上的 C++ 执行器通过 EtherCAT 共享内存（`/shm_mc`）以 1 kHz 下发并记录指令与反馈，Python 负责轨迹规划、连续碰撞检查、参数辨识与流程编排；本机调参 GUI 通过 SSH 连接 Orin。

## 快速开始

在 Orin 仓库根目录准备环境，并离线校验模型与参考轨迹：

```bash
bash calib/setup.sh
bash calib/dynamics.sh --plan-only
```

上机前完成对应手册中的现场确认，再按需要运行标定或验收：

| 任务 | 文档与入口 | 主要产出 |
| --- | --- | --- |
| [动力学标定](docs/dynamics.md) | `bash calib/dynamics.sh --mode full --robot DVT2_01` | `artifacts/calib/latest_dynamics/delivery/summary.md` |
| [动力学闭环验收](docs/dynamics.md#闭环验收) | `bash calib/dynamics_verify.sh --robot DVT2_01` | 会话目录下 `verify/report.md` |
| [三表运动学标定](docs/kinematics.md) | `bash calib/kinematics.sh`，换装后加 `--resume` | `artifacts/calib/latest_kinematics/delivery/summary.md` |
| [GUI 与开发诊断](docs/gui_and_dev.md) | 本机运行 `bash calib/gui.sh --mock` | GUI 会话记录、接管与开发 Mock 验证 |
| [构建、测试与部署](docs/shm_build.md) | 本机 `bash scripts/humble/setup.sh` / 部署 Orin | Humble 容器环境、原生程序与 ARM64 预编译包 |
| [标定原理与算法参考](docs/methodology.md) | 数学模型、约束公式与文献对应 | H1–H3 动力学、TriCal 几何与闭环验收原理 |

## 本机构建与测试

```bash
bash scripts/humble/container.sh bash scripts/humble/validate.sh < /dev/null
```

完整验证会依次运行 CMake 构建、CTest、pytest（使用同次构建的原生执行器）、C++ 安装消费和 Python wheel 校验，结果输出到 `artifacts/validation_*/`。日常快速测试和 Orin 交叉编译见[构建与部署手册](docs/shm_build.md)。

## 仓库结构

- `calib/`：交付入口（`setup.sh`、`dynamics.sh`、`dynamics_h1.sh`..`h3.sh`、`dynamics_verify.sh`、`kinematics.sh`、`gui.sh`、`dev.sh`）。
- `configs/`：机器人参数（`robot/dvt2.yaml`）、阶段配方（`dynamics/`、`kinematics/`）、NMPC 配置（`nmpc/`）以及 [DVT2_01 交付样例](configs/examples/dvt2_01_delivery/README.md)。
- `cpp/`：动力学与碰撞核心、SHM 协议和 1 kHz 原生执行器。
- `python/casbot_calib/`：会话编排、轨迹规划、辨识、GUI 与报告（含独立双臂 NMPC 子包 `nmpc/`，参见 [NMPC 实现说明](docs/dual_arm_nmpc_plan.md)）。
- `firmware/`：[TriCal 三表 USB 桥接固件](firmware/README.md)。
- `scripts/`：Humble 环境、交叉构建、部署与验证脚本。
