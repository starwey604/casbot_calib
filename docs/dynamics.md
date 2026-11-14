# 双臂动力学标定操作指南

所有标定脚本在 Orin 本机（`/home/nvidia/casbot_calib_shm`）直接运行。程序内部自动完成主站接管、握手、手部背景帧缓存、起点静止/故障检查、连续碰撞/限位/速度检查、1kHz 执行、阶段间参数自动衔接与拟合导出。

## 1. 运动前人工确认（必查）

每次启动标定前，操作员在现场人工确认以下四项（代码内不弹确认框）：

1. **固定支撑到位**：双腿、腰、头部在标定期间处于失能状态，机架支撑稳固。
2. **双臂活动空间净空**：双臂展开及目标姿态范围内无人员、线缆或障碍物，采集期间不手托或触碰手臂。
3. **急停/断电继电器在手**：异常时可立即物理断电；终端 `Ctrl-C` 也会自动请求双臂失能并等待驱动应答。
4. **未同时开启 GUI**：若本机开着调参 GUI，先点“停止并失能”并“断开”，释放 SHM 通道锁。

## 2. 怎么跑

```bash
cd /home/nvidia/casbot_calib_shm

# 首次部署或更新 C++/Python 代码后初始化环境（可传 --check 仅做预检）
bash calib/setup.sh

# 一键全流程（H1 静态重力 -> H2 摩擦 -> H3 惯量；默认 small 模式，指定 --mode full 跑 14 轴全工作空间）
bash calib/dynamics.sh --robot DVT2_01
bash calib/dynamics.sh --mode full

# 单阶段独立运行（自动衔接同会话或最近一次完成的同机器上游标定结果）
bash calib/dynamics_h1.sh
bash calib/dynamics_h2.sh
bash calib/dynamics_h3.sh

# H1 链路往返冒烟（单独跑或静态采集前串跑一次 ±10° 往返）
bash calib/dynamics_h1.sh --mode smoke
bash calib/dynamics_h1.sh --smoke

# 仅离线生成轨迹并做连续碰撞/限位/速度检查，不连接机器人
bash calib/dynamics.sh --plan-only

# 从上次中断的会话续跑未完成阶段
bash calib/dynamics.sh --resume
```

入口说明：
- `calib/setup.sh`：Orin 环境初始化、C++/Python 原生编译（统一安装到 `install/shm-core`）及模型 LFS 预检。
- `calib/dynamics.sh`：按顺序执行 `H1 静态重力 -> H2 关节摩擦 -> H3 惯量激励`，各阶段结果自动串联。
- `calib/dynamics_h1.sh` / `calib/dynamics_h2.sh` / `calib/dynamics_h3.sh`：单阶段入口。
- `calib/gui.sh`：本机调参 GUI。
- `calib/dev.sh`：开发与底层诊断入口（`inspect`、`observe`、`diagnose`、`a-evidence`、`C01–C06`、私有 `--mock`）。

## 3. 配方参数

只改真正需要调的配置文件（单阶段临时换配置可传 `--config path/to/xxx.yaml`）：
- 机器人公共参数：[dvt2.yaml](../configs/robot/dvt2.yaml)（`kp`、`kd`、`gravity_world`、力矩上限等）。
- H1 静态重力：[h1.yaml](../configs/dynamics/h1.yaml)（`small` 准备位附近快速验证 / `full` 双臂 D 最优多姿态正反接近 / `smoke` 单轴冒烟）。
- H2 关节摩擦：[h2.yaml](../configs/dynamics/h2.yaml)（左右解耦对称配对多速正反向匀速平台 + PD 起动观测，自动扣除 H1 重力）。
- H3 惯量激励：[h3.yaml](../configs/dynamics/h3.yaml)（有界 C3 包络多正弦周期激励 + 傅里叶谐波投影 + 冻结 H1 重力子空间的 140 参数 PSD 锥拟合，自动扣除 H2 摩擦与转子惯量）。

## 4. 结果怎么看

输出自动写入 `artifacts/calib/<机器编号>/<日期时间>/`，并自动更新软链接 `artifacts/calib/latest`：

- `artifacts/calib/latest/session.json`：会话状态与各阶段产物索引。
- 各阶段直接看对应的汇总报告：
  - `artifacts/calib/latest/h1/report.md`：静态重力留出验证集 RMSE（名义 vs 拟合候选）与反向接近力矩半差。
  - `artifacts/calib/latest/h2/report.md`：逐轴粘性 `b`、库仑 `fc`、静摩擦 `fs` 及留出集摩擦残差 RMSE（零摩擦基线 vs 拟合候选）。
  - `artifacts/calib/latest/h3/report.md`：逐轴质心主惯量（`Ixx/Iyy/Izz`）、重力冻结偏差及留出集动态力矩 RMSE（上游先验 vs 拟合候选）。
- 最终标定模型文件（以最后完成的阶段为准，全流程位于 `h3/analysis/`）：
  - `candidate.urdf`：写回刚体质量、质心与惯量张量的 URDF（已通过独立 Pinocchio/原生重加载校验）。
  - `runtime_dynamics.json`：配套逐轴摩擦参数与转子反射惯量 sidecar。
  - `calibration.json`：完整 140 维刚体参数、摩擦向量与留出验证指标。
- `<阶段>/capture/`：完整 1kHz 二进制/CSV 原始命令与反馈记录。
