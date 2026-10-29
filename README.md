# CASBOT Calibration (`casbot_calib`)

`casbot_calib` 是一个专为 **CASBOT 机器人（单臂与双臂系统）** 设计的动力学参数辨识、分阶段标定（Staged Calibration）、碰撞检测及仿真验证工具链。

本项目采用 **C++ 高性能底层计算 + Python 灵活工作流编排 + ROS 2 Humble 实时通信** 的混合架构，支持从离线激励轨迹生成、安全边界与自碰撞预检（Preflight Check）、真机/仿真数据采集，到摩擦、重力与惯量参数分步辨识及可视化报告生成的完整闭环。

**实机接入前必须逐项弄清[SDK待确认能力与前置条件](docs/sdk_hardware_prerequisites.md)**，尤其是反馈/指令effort语义、输出侧尺度、PD与内部补偿、采样/时钟/500Hz采用能力、停止行为，以及真实模型和腰部固定条件。当前S1–S4通过的是只读接口或本机/离线流程，未开放实机执行。未确认或不支持的能力必须阻断依赖它的阶段；只读采集保留未知信息，不能作为运动或标定资格。

---

## 🎯 核心目标与功能特性

### 1. 机器人动力学与物理参数辨识
* **分阶段标定工作流（Staged Calibration）**：支持静摩擦/启动力矩（Breakaway）、库仑与粘性摩擦（Friction Sampling）、重力补偿（Gravity Batch）以及惯性张量（Inertia Probe）的解耦与分步辨识。
* **物理一致性拟合（Physical Consistency）**：结合 Pinocchio 等刚体动力学库，确保辨识出的单臂/双臂动力学参数满足物理可实现性约束。
* **激励轨迹优化**：内置针对单臂与双臂协同的激励轨迹生成与优化器（`excitation.py` / `dual_excitation.py`）。

### 2. 安全防护与碰撞检测（Safety & Collision Guard）
* **C++ 碰撞与安全监控**：底层集成高效的几何碰撞检测（`collision.cpp`）与关节运动安全守卫（`arm_guard.cpp`）。
* **起飞前预检（Preflight Check）**：在轨迹下发至真机或仿真器前，自动评估关节限位、速度/力矩边界以及双臂自碰撞/环境碰撞风险。

### 3. 混合仿真与 ROS 2 Humble 实时传输
* **多后端仿真支持**：内置轻量级 C++ 动力学积分器与仿真器，并提供标准化的 URDF/XML 模型资产（支持与 MuJoCo / Pinocchio 对接）。
* **ROS 2 通信桥接**：提供 `casbot_calib_interfaces` 和 `casbot_calib_transport` 功能包，支持高频关节状态观测（`JointObservation`）、控制指令下发（`JointCommand`）及事件同步（`CommandEvent`）。

### 4. 可视化与自动化实验报告
* **3D 姿态与轨迹查看器（Viewer）**：支持标定轨迹、碰撞几何体及遥测数据（Telemetry）的交互式回放与预览。
* **自动化基准测试与报告**：自动对比标定前后的力矩预测残差，输出结构化评估报告与可视化图表。

---

## 📂 仓库目录结构

```text
casbot_calib/
├── configs/                # 标定工作流、双臂动力学、碰撞场景及 ROS 传输的 YAML/JSON 配置文件
├── cpp/                    # C++ 核心库（动力学计算、碰撞检测、安全守卫、仿真器及 pybind11 绑定）
│   ├── include/casbot_calib/
│   ├── src/
│   └── tests/              # C++ 单元测试与集成测试
├── python/casbot_calib/    # Python 核心包（CLI、分阶段标定逻辑、轨迹激励、参数辨识、可视化）
│   └── assets/             # 机器人 URDF、XML 模型、碰撞网格与 FK 参考基准数据
├── ros/                    # ROS 2 (Humble) 工作空间源码
│   ├── casbot_calib_core/       # 核心 C++ 库的 ROS 2 封装
│   ├── casbot_calib_interfaces/ # 自定义 ROS 2 消息接口 (JointCommand, JointObservation 等)
│   └── casbot_calib_transport/  # 实时流客户端 (stream_client) 与模拟主机 (mock_host)
├── scripts/                # 辅助脚本与 ROS 2 Humble 自动化环境部署/验证脚本集
│   └── humble/             # 容器化、系统依赖安装、ROS 编译与里程碑验证脚本
├── docs/                   # 架构设计、标定计划、数据表与验证文档
├── tests/                  # Python 端完整单元测试集 (pytest)
├── CMakeLists.txt          # 顶层 CMake 构建配置
├── pyproject.toml          # Python 包构建与依赖声明
├── environment.yml         # 核心开发与标定计算的 Conda 环境配置
└── environment-viewer.yml  # 独立的可视化工具 (Viewer) Conda 环境配置
```

---

## 🛠️ 环境配置与安装指南

本项目支持两种主要的环境配置方式：**基于 Conda 的本地开发/仿真环境**（推荐用于算法开发、离线标定与可视化）和 **基于 Ubuntu 22.04 + ROS 2 Humble 的系统/容器环境**（推荐用于实时通信与真机部署）。

### 方式一：使用 Conda 配置环境（推荐本地开发）

仓库提供了标准环境配置文件以及锁文件（`conda-lock.yml`），以确保跨平台依赖的一致性。

#### 1. 创建并激活主标定环境
```bash
# 使用 environment.yml 创建主环境
conda env create -f environment.yml

# 或者，如果您安装了 conda-lock，可使用锁文件进行精确复现：
# conda-lock install -n casbot_calib conda-lock.yml

conda activate casbot_calib
```

#### 2. 编译 C++ 扩展并安装 Python 包
本项目使用 `CMake` 与 `pyproject.toml` 管理 C++/Python 混合编译：
```bash
# 以可编辑模式安装 Python 包（会自动触发底层 C++ 绑定编译）
pip install -e . --no-build-isolation
```
*(注：如果需要单独使用 CMake 编译 C++ 核心库与测试，可使用预设：`cmake --preset default && cmake --build build`)*

#### 3. 配置可视化环境（可选）
为避免 3D 渲染库与核心数值计算库产生依赖冲突，项目提供了独立的 Viewer 环境配置：
```bash
conda env create -f environment-viewer.yml
# 或使用锁文件: conda-lock install -n casbot_calib_viewer conda-viewer-lock.yml
```

---

### 方式二：ROS 2 Humble 与容器化环境配置

针对需要运行 ROS 2 实时传输节点（`casbot_calib_transport`）或进行完整里程碑验证的场景，仓库在 `scripts/humble/` 目录下提供了一键式配置脚本。

#### 1. 原生 Ubuntu 22.04 (ROS 2 Humble) 配置
在已安装基础 Ubuntu 22.04 的系统上，按顺序执行以下脚本：
```bash
# 1. 安装系统级依赖与 ROS 2 Humble 相关组件
bash scripts/humble/install_system.sh

# 2. 配置 Python 虚拟环境并安装锁定的依赖 (requirements.lock)
bash scripts/humble/setup_python.sh

# 3. 编译 ROS 2 工作空间 (casbot_calib_core, interfaces, transport)
bash scripts/humble/build_ros.sh
```

#### 2. 使用容器环境（Docker / Podman）
如果希望在隔离容器中运行构建与测试，可直接调用容器脚本：
```bash
bash scripts/humble/container.sh
```

---

## 🚀 快速开始与常用工作流

安装完成后，您可以通过 `casbot_calib` 命令行工具（CLI）或脚本运行标定与验证任务。

### 1. 运行命令行工具 (CLI)
```bash
# 查看 CLI 支持的子命令与参数说明
python -m casbot_calib --help
```

### 2. 运行标定与工作流演示
使用 `configs/` 目录下的配置文件启动标准标定或分阶段双臂标定流程：
```bash
# 运行基础工作流演示
python -m casbot_calib --config configs/workflow_demo.yaml

# 运行双臂分阶段标定工作流
python -m casbot_calib --config configs/staged_dual_arm.yaml
```

### 3. 启动三维可视化与遥测回放
```bash
# 回放已采集的遥测数据
python scripts/replay_telemetry.py

# 启动工作空间基准测试
python scripts/benchmark_arm_workspace.py
```

### 4. ROS 2 传输层测试
在编译好 ROS 2 工作空间后，可以通过脚本加载环境并测试模拟主机与流客户端：
```bash
source scripts/humble/ros.sh
# 启动验证或运行传输基准测试
bash scripts/humble/run.sh
```

---

## ✅ 测试与验证

本项目包含完善的 C++、Python 及 ROS 2 集成测试套件，确保动力学计算、碰撞检测与参数辨识的准确性。

### 运行 Python 单元测试
```bash
pytest tests/
```

### 运行 C++ 核心测试
```bash
ctest --test-dir build --output-on-failure
```

### 运行全流程自动化验证 (Humble 里程碑验证)
```bash
# 执行完整的环境检查、编译验证与结果比对
bash scripts/humble/validate.sh
```

---

## 📚 更多文档

关于算法细节、硬件参数与各阶段验证报告，请参阅 `docs/` 目录下的详细文档：
* **[SDK实机接入前必须确认的能力](docs/sdk_hardware_prerequisites.md)**：甲方待确认清单、所需证据、阶段开放条件和结论记录要求；实机接入前必读。
* [双臂千分计运动学K1–K5计划](docs/kinematics_trical_plan.md)、[K2测量闭环](docs/kinematics_trical_k2.md)及[K3几何辨识](docs/kinematics_trical_k3.md)：三表Mock、碰撞门禁、自动对中/稳定采样、六球选点、MDH/Hayati分层求解与候选URDF导出；实机和完整计量验收仍待后续。
* [K4重复性与中文回放](docs/kinematics_trical_k4.md)：同工装三批独立采集/辨识、冻结选参、公共留出位置约束、独立外测数据契约和MuJoCo只读演示；实物外测/间隔漂移/产品AP-RP仍待验收。
* [机器人建模与参数说明](docs/model_and_parameters.md)
* [单臂 (Arm R1) 几何与标定文档](docs/arm_r1.md)
* [整臂标定计划](docs/whole_arm_calibration_plan.md) & [双臂动力学标定计划](docs/dual_arm_dynamics_plan.md)
* [碰撞检测与分阶段工作流](docs/collision_staged_workflow.md)
* [动力学P0数据契约](docs/staged_dynamics_p0_r0.md)、[R1多构型开发](docs/staged_dynamics_p0_r1.md)、[R2算子稳定性](docs/staged_dynamics_p0_r2.md)、[R3频段归因/选参](docs/staged_dynamics_p0_r3.md)及[R4正式验收](docs/staged_dynamics_p0_r4.md)：已通过本机合成测试、独立控制对照和中文回放；不代表实机或全频带性能。
* [ROS 2 传输层设计文档](docs/ros_transport.md) & [Humble 环境说明](docs/humble_environment.md)
* [CASBOT2 SDK源码接入评估](docs/casbot02_sdk_source_intake.md)与[S1只读接入](docs/vendor_s1.md)：可选SDK构建、原始CDR记录、字段规范化，以及厂商反馈/原生ROS Mock共用的只读显示；实机控制、力矩/采样能力和物理网络仍待验证。
* [S2位置流程与频率／响应](docs/vendor_s2.md)：整双臂实测起点、Coal参考门禁、C++ SDK位置流与故障停止；本机100/250/500Hz记录和316项Python测试通过，早期反馈空窗及原始失败也保留，未确认稳定500Hz或实机执行资格。
* [S3动力学观测资格](docs/vendor_s3.md)：S1/主机CSV/NPZ只读导入，来源与输出力矩尺度、采样时钟、状态和原始带宽逐阶段检查；合格周期记录生成Pinocchio回归问题。当前SDK未知反馈仍阻断，500Hz发布不代替4kHz原始采样，实机执行及S4现场闭环未开放。
* [S4离线分阶段闭环](docs/vendor_s4.md)：冻结观测角色，复用原求解器按重力→运动摩擦→起动摩擦→惯量拟合并复核前序；新增4kHz双臂起动观测和已冻结候选的Mock控制对照。SDK实机执行仍关闭；旧开发验证不冒充新的正式留出。
* [可视化工具演示指南](docs/viewer_demo.md)
* [开发者指南](docs/development.md)
