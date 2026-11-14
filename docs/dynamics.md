# 双臂动力学标定操作指南

所有标定脚本在 Orin 本机（`/home/nvidia/casbot_calib_shm`）直接运行。程序内部自动完成主站接管、握手、手部背景帧缓存、起点静止/故障检查、连续碰撞/限位/速度检查、1kHz 执行与拟合导出。

## 1. 运动前人工确认（必查）

每次启动标定前，操作员在现场人工确认以下三项（代码内不弹确认框）：

1. **固定支撑到位**：双腿、腰、头部在标定期间处于失能状态，机架支撑稳固。
2. **双臂活动空间净空**：双臂展开及目标姿态范围内无人员、线缆或障碍物，采集期间不手托或触碰手臂。
3. **急停/断电继电器在手**：异常时可立即物理断电；终端 `Ctrl-C` 也会自动请求双臂失能并等待驱动应答。
4. **未同时开启 GUI**：若本机开着调参 GUI，先点“停止并失能”并“断开”，释放 SHM 通道锁。

## 2. 怎么跑

```bash
cd /home/nvidia/casbot_calib_shm

# 首次部署或更新 C++/Python 代码后初始化环境（平时不改代码无需重复执行；可传 --check 仅做预检）
bash calib/setup.sh

# 单跑 H1 静态重力（准备位 -> 正反接近静态采样 -> PSD 重力拟合 -> 候选 URDF）
bash calib/dynamics_h1.sh

# 指定机器序列号（首次指定后自动记住，后续无需再传）
bash calib/dynamics_h1.sh --robot DVT2_01

# 仅跑链路往返冒烟（准备位 -> ±10° 往返 -> 准备位 -> 失能）
bash calib/dynamics_h1.sh --mode smoke

# 静态采集前同周期先做一次往返冒烟
bash calib/dynamics_h1.sh --smoke

# 仅离线生成轨迹并做连续碰撞/限位/速度检查，不连接机器人
bash calib/dynamics_h1.sh --plan-only

# 从上次中断的会话续跑未完成阶段
bash calib/dynamics.sh --resume
```

全流程脚本：
- `calib/setup.sh`：Orin 环境初始化、C++/Python 原生编译（统一安装到 `install/shm-core`）及模型 LFS 预检（`--check` 仅检查不重编）。
- `calib/dynamics.sh`：按顺序执行 `H1 静态重力 -> H2 摩擦 -> H3 惯量`（H2/H3 待 P6 实现）。
- `calib/dynamics_h1.sh` / `calib/dynamics_h2.sh` / `calib/dynamics_h3.sh`：单阶段入口。
- `calib/gui.sh`：本机调参 GUI。
- `calib/dev.sh`：开发与底层诊断入口（`inspect`、`observe`、`diagnose`、`a-evidence`、`C01–C06`、私有 `--mock`）。

## 3. 配方参数

只改真正需要调的配置文件：
- 机器人公共参数：[configs/robot/dvt2.yaml](../configs/robot/dvt2.yaml)（`kp`、`kd`、`gravity_world`、力矩上限等）。
- H1 静态重力配方：[configs/dynamics/h1.yaml](../configs/dynamics/h1.yaml)：
  - `mode`: `small`（准备位附近快速静态，默认双肩 pitch `±15°` 目标、`5°` 正反接近）或 `full`（双臂全姿态 D 最优采样）或 `smoke`（单轴冒烟）。
  - `ready_pose`: 默认开启（双肩 roll `+20°/-20°`，其余 `0°`，平滑切入标定 PD 与名义重力前馈）。

临时使用自定义配方文件可传 `--config path/to/h1.yaml`。

## 4. 结果怎么看

输出自动写入 `artifacts/calib/<机器编号>/<日期时间>/`，并自动更新软链接 `artifacts/calib/latest`：

- `artifacts/calib/latest/session.json`：会话状态与各阶段产物索引。
- `artifacts/calib/latest/h1/report.md`：**直接看这份汇总报告**，包含：
  - 轨迹时长、最小碰撞间距、有效反馈帧数与频率；
  - 留出验证集（validation）上**名义模型 vs 拟合候选**的整体与逐轴力矩 RMSE（Nm），以及正反接近力矩半差；
- `artifacts/calib/latest/h1/analysis/candidate.urdf`：拟合导出的候选惯性参数 URDF（已通过独立重加载校验）。
- `artifacts/calib/latest/h1/analysis/static_comparison.csv`：每个静态窗口实测平均关节角、估算力矩与名义重力明细。
- `artifacts/calib/latest/h1/capture/`：完整 1kHz 二进制/CSV 原始命令与反馈记录。
