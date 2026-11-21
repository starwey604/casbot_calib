# 双臂动力学标定操作指南

所有标定脚本在 Orin 本机（`/home/nvidia/casbot_calib_shm`）直接运行。程序内部自动完成主站接管、握手、手部背景帧缓存、起点静止/故障检查、连续碰撞/限位/速度检查、1kHz 执行、阶段间参数自动衔接、防退化门禁校验与交付包生成。

## 1. 运动前人工确认（必查）

每次启动标定前，操作员在现场人工确认以下四项（代码内不弹确认框）：

1. **固定支撑到位**：常规模式下机架承重，腿、腰和头部失能。辅助站立模式下，从接管前到最后一次失能后，吊装或外部支撑须能独立承重并防倒。双脚平贴防滑地面，双腿近直立，腰和头归正，躯干竖直。
2. **双臂活动空间净空**：双臂展开及目标姿态范围内无人员、线缆或障碍物，采集期间不手托或触碰手臂。
3. **急停/断电继电器在手**。动力学采集和闭环验收期间，按一次 `Ctrl-C` 或向标定进程发 `SIGUSR1`，双臂平稳刹车，经准备位（如适用）回到本次实测起点后失能。复位期间辅助站立保持继续工作。再按一次 `Ctrl-C`、任意时刻按 `Ctrl-\`、发送 `SIGTERM` 或 SSH 断线触发 `SIGHUP`，程序会立即请求失能。轨迹和复位过程中都要保持活动空间净空。异常退出且 `disable_ack=0` 或缺失时，立即用独立继电器切断驱动电源。
4. **未同时开启 GUI**：若本机开着调参 GUI，先点“停止并失能”并“断开”，释放 SHM 通道锁。

## 2. 怎么跑

DVT2_01 的辅助站立已完成半增益和全增益实机冒烟，现场选用配置中的全增益做动力学标定。加 `--hold-body` 启用腿、踝、腰和头的保持。单次采集结束后驱动失能，阶段间由外部支撑承重。见[实机记录](records/body_hold_20261120.md)。

```bash
bash calib/dynamics.sh --mode full --robot DVT2_01 --hold-body
```

需要复测站立时，在外部承重下先跑半增益，再跑全增益。双臂按准备段向两侧展开（肩 roll 左 +20°、右 −20°），随后保持指定时长。观察脚踝和机身是否平稳，结束时核对失能应答。

```bash
bash calib/dev.sh hold-body --scale 0.5 --duration 10 --execute
bash calib/dev.sh hold-body --scale 1 --duration 10 --execute
```

单阶段、续跑和闭环验收需要辅助站立时，在当次命令中同样加 `--hold-body`。

```bash
cd /home/nvidia/casbot_calib_shm

# 首次部署或更新 C++/Python 代码后初始化环境（可传 --check 仅做预检）
bash calib/setup.sh

# 一键全流程（H1 静态重力 -> H2 摩擦 -> H3 惯量；默认 small 模式，指定 --mode full 跑 14 轴全工作空间）
bash calib/dynamics.sh --robot DVT2_01
bash calib/dynamics.sh --mode full

# 单阶段独立运行（自动衔接同会话或最近一次已通过验收的同机器上游标定结果）
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

# 免接管、免动机器人，对最近一次（或指定目录）已有 1kHz 采集记录秒级重跑辨识与交付打包
bash calib/dynamics.sh --reanalyse
bash calib/dynamics.sh --reanalyse artifacts/calib/DVT2_01/20261115_103000
```

入口说明：
- `calib/setup.sh`：Orin 环境初始化、离线 wheelhouse 安装、C++/Python 原生编译（安装到 `install/shm-core`）及模型 LFS 预检。
- `calib/dynamics.sh`：按顺序执行 `H1 静态重力 -> H2 关节摩擦 -> H3 惯量激励`，各阶段结果自动串联并输出会话级 `delivery/` 交付包。
- `calib/dynamics_h1.sh` / `calib/dynamics_h2.sh` / `calib/dynamics_h3.sh`：单阶段入口（同样支持 `--plan-only`、`--resume`、`--reanalyse`）。
- `calib/gui.sh`：本机调参 GUI。
- `calib/dev.sh`：开发与底层诊断入口（`inspect`、`observe`、`diagnose`、`a-evidence`、`C01–C06`、私有 `--mock`）。

## 3. 配方参数

只改真正需要调的配置文件（单阶段临时换配置可传 `--config path/to/xxx.yaml`）：
- 机器人公共参数：[dvt2.yaml](../configs/robot/dvt2.yaml)（`kp`、`kd`、`gravity_world`、力矩上限等）。
- H1 静态重力：[h1.yaml](../configs/dynamics/h1.yaml)（`small` 准备位附近快速验证 / `full` 双臂同步 D 最优 2-opt 直连，每个采集臂七轴均做正反接近 + 实测角配对滞回分离 / `smoke` 单轴冒烟）。
- H2 关节摩擦：[h2.yaml](../configs/dynamics/h2.yaml)（双臂镜像分组对数速度连续扫掠 + C++ 同构 Stribeck 拟合与零偏诊断，自动扣除 H1 重力）。
- H3 惯量激励：[h3.yaml](../configs/dynamics/h3.yaml)（近远端带宽分层多正弦 + CMA-ES 相位寻优 + 平滑重力摩擦前馈 + 冻结 H1 重力子空间的 140 维刚体与 14 轴转子惯量联合锥拟合）。

H1/H2/H3 共用机器人配置中的逐轴 `kp`、`kd`，数组顺序为左臂 ID12–18、右臂 ID19–25；无需逐阶段设置。准备位展开段保持 20/2，到位后平滑切换到配置增益。修改配置用于新规划的采集；`--reanalyse` 仍按原采集数据分析，不会改变历史增益。

## 4. 结果怎么看

输出自动写入 `artifacts/calib/<机器编号>/<日期时间>/`，并自动更新软链接 `artifacts/calib/latest`：

- **首选统一交付目录 `artifacts/calib/latest/delivery/`**：
  - `summary.md`：中文一页纸验收总表（汇总各阶段留出 RMSE 改善、防退化门禁判定及最终 14 轴动力学参数）。
  - `calibrated.urdf`：最终采纳的刚体质量、质心与惯量张量 URDF（已通过独立 Pinocchio/原生重加载校验）。
  - `runtime_dynamics.json`：配套逐轴 Stribeck 摩擦参数（`viscous`、`coulomb`、`static_friction`、`stribeck_velocity`、`smoothing_velocity`）与 14 轴转子反射惯量 `rotor_inertia`。
  - `calibration_bundle.json`：记录重力、摩擦、连杆惯量与转子惯量各自采纳的阶段来源（若某阶段留出验证未改善或主激励轴显著退化，自动回退至未退化的上游先验参数）。
- **分阶段详细报告与原始数据**：
  - `h1/report.md`、`h2/report.md`、`h3/report.md`：各阶段独立验收报告（重分析时旧 `analysis/` 自动归档为 `analysis.previous_<ns>`）。
  - H1 留出 RMSE 按逐轴有效正反配对计算；未逼近、卡住或方向异常的分量自动排除。逐轴表显示有效配对数，无有效配对显示 `—`，原始窗口仍完整保留。
  - `<阶段>/capture/`：完整 1kHz 二进制（`feedback.bin`、`commands.bin`）与事件记录。

## 5. 闭环验收

使用通过 H1、H2、H3 的完整交付会话。实机验收前拆下双手和工具，按第 1 节落实对应模式的支撑条件。移走双臂活动范围内的琴架、鼓架、谱架和线缆，按第 1 节确认现场条件。

```bash
bash calib/dynamics_verify.sh --robot DVT2_01
bash calib/dynamics_verify.sh --plan-only
bash calib/dynamics_verify.sh --reanalyse
```

默认选择同机器最新符合条件的交付会话，也可直接传入标定会话目录。查看会话内的 `verify/report.md`，其中列出电子琴和架子鼓轨迹的五组法兰跟踪、关节误差、相位和抖动结果。PASS 时在 `verify/analysis/recommended_band_gains.yaml` 取得柔和增益及本次采用的前馈参数。验收退出码为 0 表示通过，1 表示指标未通过或窗口无效，2 表示执行或分析中止。

开发时用 `bash calib/dev.sh dynamics-verify-mock <标定会话目录>` 在私有物理 SHM 上演练，使用本地测试会话。
