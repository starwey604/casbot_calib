# 🚀 动力学标定启动指南：从环境安装到结果验收

本文面向第一次使用 `casbot_calib` 的用户，带您完成 **环境准备 → Mock 标定 → 独立验证 → 中文回放 → 静态现场规划 → SDK 只读采集**，并说明怎样继续生成动力学候选 URDF。

默认对象为 **完整装配、腰部刚性固定的 14 轴双臂机器人**。动力学由 Pinocchio 计算，碰撞由 Coal 检查，模拟积分使用 Boost.Odeint；MuJoCo 只负责显示。先跑通本地模拟，再根据设备能力接入真实观测，可以逐步确认安装、数据和求解流程。

**当前可以完整运行的是 Mock 标定与离线分析，尚未开放实机运动。** SDK 只读采集不发送运动命令；现场执行仍需满足 [SDK 接入前置条件](sdk_hardware_prerequisites.md)。首次练习不依赖本机历史 `artifacts/`，也不复用已经消耗的正式测试数据。

---

## 🧭 一、先了解本次要标定什么

动力学标定按顺序进行，前一阶段验收通过后才进入下一阶段：

| 顺序 | 工作内容 | 主要结果 |
| --- | --- | --- |
| 1 | 方向与软件零位检查 | 关节名称、顺序、正方向及软件映射检查；实机机械零位另需确认 |
| 2 | 驱动时序与通信预检 | Mock 驱动响应；ROS 分档实验另测发送/接收频率、抖动与整体响应 |
| 3 | 多姿态静态保持 | 重力模型，约束质量与一阶矩的可观测组合 |
| 4 | 多姿态、多速度正反匀速 | 库仑摩擦与黏滞摩擦 |
| 5 | 重力补偿下的起动实验 | 等效起动摩擦阈值及其区间 |
| 6 | 前序验收后的加速度激励 | 惯性参数、质量矩阵与速度耦合项的预测能力 |
| 7 | 独立轨迹与控制对照 | 区分模型前馈收益和 PD 调参收益 |

静态数据不能唯一恢复每个连杆的质量/质心，也不能辨识转动惯量。报告会区分可观测组合与物理先验选择的参数代表；`M(q)`、重力、科氏/离心项由同一个刚体模型生成。

本文第一次运行使用 `configs/staged_dual_arm.yaml` 的 schema 1 教学/回归流程。更严格的多构型、稳定性、归因及正式留出流程见 [P0 R0–R4 计划](staged_dynamics_p0_plan.md)；schema 1 通过不等于新的 P0 或实机验收通过。

---

## 🛠️ 二、准备运行环境

### 1. 确认仓库与基础系统

本指南统一使用 **Ubuntu 22.04 + ROS 2 Humble + Python 3.10**。系统原生库与 `.venv-humble` 通过脚本组合，不需要激活 Conda，也不要向该环境重复安装另一套 Pinocchio/Coal。

将仓库放到工作目录，以下命令均从宿主仓库根目录执行：

```bash
cd ~/codings/casbot_calib
```

本机使用已有的 Docker 后端 distrobox 容器 `ros2-humble`，容器与宿主共享相同仓库绝对路径。`container.sh` 会启动并调用已有容器，**不会创建容器、安装基础 ROS，也不是交互式终端入口**。

```bash
# 确认已有容器；未启动也可以，后续脚本会启动它
docker inspect ros2-humble
```

全新机器应先准备已配置 ROS 软件源、安装 ROS 2 Humble 的 Ubuntu 22.04 环境，可从 [ROS 官方安装入口](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 开始，再使用下列项目安装步骤。不同容器名可设置 `CALIB_HUMBLE_CONTAINER`；容器仍须满足共享路径和用户访问条件。本仓库的包装脚本直接使用 Docker，不自动兼容 Podman。

### 2. 首次安装项目依赖

这一步仅首次安装或依赖更新时需要执行。先在容器中安装系统依赖，再以当前用户创建 Python 环境：

```bash
docker start ros2-humble
docker exec --user root --workdir "$PWD" ros2-humble \
  bash scripts/humble/install_system.sh

docker exec --user "$(id -u):$(id -g)" --workdir "$PWD" ros2-humble \
  bash scripts/humble/setup_python.sh
```

上例容器名为 `ros2-humble`；如果使用其他名称，请同时替换直接调用 `docker` 的容器参数。系统安装脚本要求已有 ROS 软件源；它不负责从空白 Ubuntu 配置基础 ROS。不要在项目仿真或采集中替换正在使用的环境。

### 3. 编译 C++ 绑定并检查 CLI

```bash
bash scripts/humble/container.sh python -m pip install -e . --no-deps --no-build-isolation \
  --config-settings=build-dir=build/humble \
  --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON

bash scripts/humble/container.sh cmake --preset humble -DCALIB_BUILD_PYTHON=ON
bash scripts/humble/container.sh cmake --build --preset humble
bash scripts/humble/container.sh ctest --preset humble
bash scripts/humble/container.sh casbot-calib --help
```

CLI 帮助应包含 `staged-calibrate`、`staged-suite`、`plan-static-field` 等子命令。`python -m casbot_calib` 与 `casbot-calib` 是等价入口，但都必须指定子命令。

如果已经安装好本项目，可以直接从 CLI 检查开始。后续修改 C++ 后需重新执行绑定安装；单独运行 `cmake --build` 不会刷新 Python 已安装的扩展。详细环境约定见 [Humble 环境说明](humble_environment.md)。

在原生 Ubuntu 22.04/Humble 环境中，系统依赖和 Python 环境可直接用相同安装脚本建立；后续将 `bash scripts/humble/container.sh 命令…` 替换为 `bash scripts/humble/run.sh 命令…`。原生 shell 同样不能混入 Conda。

---

## ▶️ 三、第一次跑通 Mock 标定

### 1. 选择新的实验目录

下面的变量与后续命令在**同一个终端**执行；如果换了终端，请重新设置变量。

```bash
calib_session=artifacts/first_dynamics_v1
```

该目录必须尚不存在。重新实验请换成 `first_dynamics_v2` 等新目录；不要删除失败结果后用相同路径覆盖，也不要将 `--resume` 用于条件不同的新实验。

### 2. 一次完成采集、辨识、验证和回放

```bash
bash scripts/humble/container.sh casbot-calib staged-suite \
  --config configs/staged_dual_arm.yaml \
  --output "$calib_session" --fps 12
```

程序会先生成并检查完整参考，再用 C++ Mock 采集，依次完成重力、运动摩擦、起动摩擦、惯量及四组控制对照，随后进行综合复核并生成中文视频。批量采集使用虚拟时间，不等待等长的墙上时间；综合复核与渲染仍需计算时间和磁盘空间。

**第一次成功运行的标志是 `delivery.json` 中 `passed=true`。** 单个阶段通过或出现视频文件都不足以代替完整交付；`synthetic_only=true`、`hardware_executable=false` 表示结果只适用于本次合成实验。

### 3. 到哪里看结果

首次完整会话通常包含以下内容；继续运行时，独立复核/展示目录可能带数字后缀，以 `delivery.json` 引用的路径为准。

| 路径 | 建议查看的内容 |
| --- | --- |
| `plan_report.json`、`plans/` | 参考轨迹、碰撞门禁与拒绝候选 |
| `direction_check.json`、`driver_timing.json` | Mock 映射和驱动时序检查 |
| `runs/` | 原始观测、执行命令、保护及实际路径检查 |
| `identification_gravity*/report.json` | 重力回归秩、独立姿态误差和正反接近半差 |
| `identification_friction*/report.json` | 黏滞/库仑参数及独立匀速预测 |
| `identification_breakaway*/report.json` | 起动阈值、区间和重复差异 |
| `identification_inertia*/report.json` | 独立轨迹预测、腕部专项及前序复核 |
| `control_comparison.json`、`.png` | 原模型/辨识模型 × 原 PD/优化 PD 四组对照 |
| `comprehensive*/verification.json` | 数值收敛、完整路径及无真值重分析复核 |
| `presentation*/staged_calibration_zh.mp4` | 根据实际送达观测生成的中文回放 |
| `delivery.json`、`delivery.md` | 本次完整交付结论与适用范围 |

`evaluation/` 保存合成系统真值，供独立评价使用。辨识和回放不依赖它；不要把它的参数当作实机先验，也不要手动复制进观测或辨识输入。

---

## 🔬 四、希望逐阶段推进时怎样运行

这一节是上一节“一次运行”的替代方式。**请选择一个尚不存在的会话目录，从规划开始；不要对已经完成的目录重新执行下面的规划命令。**

```bash
calib_session=artifacts/stepwise_dynamics_v1

# 只规划：不会采集或辨识
bash scripts/humble/container.sh casbot-calib plan-staged \
  --config configs/staged_dual_arm.yaml --output "$calib_session"

# 自动完成方向/时序预检，然后停在重力验收
bash scripts/humble/container.sh casbot-calib staged-calibrate \
  --config configs/staged_dual_arm.yaml --output "$calib_session" --resume --through gravity

# 查看上一阶段报告后，继续运动摩擦
bash scripts/humble/container.sh casbot-calib staged-calibrate \
  --config configs/staged_dual_arm.yaml --output "$calib_session" --resume --through friction

# 起动摩擦及前序验收
bash scripts/humble/container.sh casbot-calib staged-calibrate \
  --config configs/staged_dual_arm.yaml --output "$calib_session" --resume --through breakaway

# 加速度激励与惯量验收
bash scripts/humble/container.sh casbot-calib staged-calibrate \
  --config configs/staged_dual_arm.yaml --output "$calib_session" --resume --through inertia

# 补齐控制对照、综合复核和中文视频，形成完整交付
bash scripts/humble/container.sh casbot-calib staged-suite \
  --config configs/staged_dual_arm.yaml --output "$calib_session" --resume --fps 12
```

每个 `staged-calibrate` 检查点看 `checkpoint.json` 的 `checkpoint_passed` 和 `completed_through`。前序失败会阻断后续；应检查对应报告和原始记录，而不是调整 `passed` 字段或放宽阈值。`--through inertia` 尚不包含完整控制、综合复核和展示。

`--resume` 只用于同一实现、配置、模型与输入条件下继续已核验的会话；原始记录保持不变，已有分析可能另存带后缀的重分析。改变配置或升级核心方法时，建立新实验目录。

### 独立重分析与重新渲染

完成会话后，可在新目录中重分析和回放，原始观测不覆盖：

```bash
bash scripts/humble/container.sh casbot-calib staged-identify \
  "$calib_session" --through inertia --output "${calib_session}_reanalysis"

bash scripts/humble/container.sh casbot-calib staged-verify \
  "$calib_session" --output "${calib_session}_review"

bash scripts/humble/container.sh casbot-calib render-staged \
  "$calib_session" --output "${calib_session}_video" --fps 12
```

`staged-identify` 输出 `analysis.json`，只声明辨识与独立预测结果，不替代控制和完整交付。MuJoCo 显示的是记录中的实际角度，不参与力学、碰撞或参数求解。

---

## 📍 五、为静态现场实验准备姿态扫描

静态现场预设可独立运行，不要求先完成整个 Mock 会话：

```bash
calib_static_plan=artifacts/field_static_v1

bash scripts/humble/container.sh casbot-calib plan-static-field \
  --config configs/static_field_nominal.yaml --output "$calib_static_plan"

bash scripts/humble/container.sh casbot-calib verify-static-field \
  "$calib_static_plan" --output "${calib_static_plan}_review"
```

默认覆盖每臂 7 轴单独扫描及五轴三档组合，从 570 个去重候选中选择每臂 **24 个训练姿态 + 8 个验证姿态**，每个姿态分别正反接近，等待 2 秒后安排 1 秒测量窗口。nominal 默认计划共 128 条参考，约 21 分 9 秒，不包含现场准备、人工检查和重试。

主要查看 `poses.csv`、`candidates.json`、`report.json` 及 `plans/`。训练与验证分别检查静态可观测方向覆盖，超限、端点碰撞或连续路径不合格的候选会保留淘汰原因。

**这是独立的参考规划包，不能直接传给 `staged-calibrate` 或当作 S4 观测协议。** 配置基准角是离线设计值；实际到基准的路径、真实夹具、设备限制、跟踪包络和采样资格均需另验。后续通过现场适配逐条绑定采集与保护记录，再进入统一观测流程。详细规则见 [静态现场扫描预设](static_field_preset.md)。

---

## 🌐 六、验证 ROS 频率与整体响应

本节可选，运行真实 ROS 双端 Mock，仍只使用本机隔离端点。先构建 ROS 包，再加载 overlay：

```bash
bash scripts/humble/container.sh bash scripts/humble/build_ros.sh

bash scripts/humble/container.sh bash scripts/humble/ros.sh \
  casbot-calib ros-benchmark --config configs/ros_transport.yaml \
  --output artifacts/first_ros_benchmark_v1
```

默认客户端/主机控制/反馈发布为 500 Hz，矩阵包含 100/250 Hz 分档、显式 1000 Hz 对照，以及负载、QoS 和故障场景。发送、采用、采样、发布和接收按各自事件统计；本机回环结果不证明物理以太网性能。

厂家已说明主机仅对 position 插值到 1000 Hz，debug 反馈可能重复最新缓存，stamp 为发布时间。这条实机链路不同于 Mock 的 ZOH/电流响应实验，不能直接套用后者拆分驱动或单向网络延迟。实际接入时优先测可观测的发送/接收频率与整体响应。详见 [ROS 时序说明](ros_transport.md) 和 [最新厂家答复](sdk_hardware_prerequisites.md#厂家答复记录2026-10-30同步)。

---

## 🤖 七、连接 SDK 时先做只读采集

### 1. 准备可选 SDK overlay

本节要求现场连接、ROS domain、网络、反馈单位及轴名称已经核对。SDK 默认放在相邻 `~/codings/casbot2-ros2-sdk`；其他位置通过 `CASBOT_SDK_ROOT` 指定。

```bash
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
cp configs/vendor_s1.yaml configs/vendor_s1_local.yaml
```

在本地配置中核对 `joint_topic`、`joint_type`、具名14轴、实际软件/固件版本和 QoS。默认 `/joint_states`；若已经有对应 debug 模式反馈，可按实际端点选择 `/motion/joint_state` 或 `/motion/debug/joint_state`，消息类型分别为 `sensor_msgs/msg/JointState` 和 `crb_ros_msg/msg/JointStateData`。S1 不会为了取得反馈主动切换模式，上身模式能否提供这些端点仍需确认。

S1 的 `effort_source`、`effort_unit`、`sample_timestamp_semantics` 保留 `unknown`，`model_mapping_verified` 保留 `false`：这些保守值表示只读层尚未授予资格，不是忽略厂家答复。厂家声明记录在能力文档，物理尺度/采样资格由独立 S3 契约处理，不能靠改 S1 配置绕过。

### 2. 采集并规范化反馈

```bash
# 填写现场实际确认的域号，不使用猜测的默认值
read -r -p "现场 ROS_DOMAIN_ID: " calib_domain_id

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib vendor-capture --config configs/vendor_s1_local.yaml \
  --output artifacts/first_sdk_readonly_v1 --duration 10 \
  --domain-id "$calib_domain_id" --scope hardware_read_only

bash scripts/humble/container.sh casbot-calib vendor-normalize \
  artifacts/first_sdk_readonly_v1 --output artifacts/first_sdk_normalized_v1
```

采集保留原始 CDR、header 和本地接收时刻；规范化另存结果，未知采样时刻、设备序号、电流和状态不补造。反馈 effort 已由厂家说明是模组驱动估算 Nm、无独立扭矩传感器；输出侧/尺度/偏置、滤波和补偿含义仍待核验。

目前 **1000 Hz 发布不等于 1000 Hz 新采样，发布 stamp 不等于采样时间**。不要按它直接差分求加速度、把相同数值当旧帧删除，或把重复缓存当独立样本。当前仅含发布时刻的记录连静态 S3 资格也不会自动通过；静态缓存替代观测尚需另行实现和验证。

### 3. 何时可以进入实机动力学辨识

先按 [SDK 前置条件](sdk_hardware_prerequisites.md) 闭合设备版本、实际模型/夹具、方向/零位、力矩含义/尺度、停止、采样/时钟和相应控制能力，再建立新的硬件 S3/S4 协议。既有流程为：

**不可变观测导入 → 逐阶段资格检查 → 角色冻结 → 重力 → 运动摩擦 → 起动摩擦 → 惯量 → 独立验收。**

实机动态观测当前不满足既有 4 kHz 等方法门禁；该门槛是项目当前政策，不是厂家通用指标。不能直接改成 1000 Hz、将未知滤波标为无滤波，或将 default unknown 配置改成“通过”。详细入口见 [S3](vendor_s3.md) 与 [S4](vendor_s4.md)；本指南不提供尚未开放的实机运动命令，S2 localhost 限制保留。

---

## 📦 八、进阶：从新 Mock 会话生成候选 URDF

URDF 导出目前接受的是 **已通过并封存的 S4 分析目录**，不能将 schema 1 会话或 `staged-identify` 的输出直接传给它。本节可选；在第三节或第四节已有完整的、默认 500 Hz schema 1 Mock 会话后，继续用同一终端的 `$calib_session` 生成 S4 合成协议：

```bash
# 新采集 56 条 4 kHz 起动记录，复用已有 C++ Mock
bash scripts/humble/container.sh python scripts/humble/collect_vendor_s4_mock.py \
  --source "$calib_session" --output "${calib_session}_s4_native"

# 将该会话的开发观测与新起动记录绑定为 S3/S4 合成输入
bash scripts/humble/container.sh python scripts/humble/prepare_vendor_s4.py \
  --source "$calib_session" --native "${calib_session}_s4_native" \
  --output "${calib_session}_s4_inputs"

# 通过离线分阶段拟合后形成封存分析
bash scripts/humble/container.sh casbot-calib vendor-identify-staged \
  "${calib_session}_s4_inputs/protocol" --through inertia \
  --output "${calib_session}_s4_analysis"

bash scripts/humble/container.sh casbot-calib export-dynamics-urdf \
  "${calib_session}_s4_analysis" --stage inertia \
  --output "${calib_session}_urdf"

bash scripts/humble/container.sh casbot-calib verify-dynamics-urdf \
  "${calib_session}_urdf" --output "${calib_session}_urdf_review"
```

这些转换脚本明确只适用于合成开发数据，不生成实机尺度证据，也不构成新的正式最终留出验收。S4 的 `analysis.json`、所需阶段及 URDF `report.json` 都需通过；这里未额外运行 S4 候选的新控制对照，不把分析的 `control_validated=false` 改成 true。

导出包主要包括 `candidate.urdf`、`calibration.json`、`bodies.csv`、`report.json` 和 `manifest.json`。刚体参数写入 URDF，固定手部惯性只聚合一次；摩擦、转子惯量及世界重力保留在配套 JSON。若只导出重力阶段代表，使用新的输出目录并设置 `--stage gravity`，此时不能宣称转动惯量已经标定。

重加载通过证明序列化和候选模型一致，不证明实机质量正确，也不会替换机器人主机配置。部署前仍需实际模型/碰撞、补偿叠加和独立实机验证。详见 [动力学 URDF 导出说明](dynamic_urdf_export.md)。

---

## ✅ 九、常见问题与排查入口

| 现象 | 建议检查 |
| --- | --- |
| `docker inspect` 找不到容器 | 先准备已安装 Humble、共享路径正确的 Docker 容器；包装脚本不创建容器 |
| 找不到 `_core` 或 `casbot-calib` | 使用 Humble 包装入口；确认完成 `.venv-humble` 安装及可编辑绑定构建 |
| 提示不能混用 Conda | 使用干净的 Humble 进程环境；不要共用 Conda/Humble 编译缓存或重复装原生库 |
| 输出目录已存在 | 新实验换新路径；仅核对同一条件后使用支持 `--resume` 的入口继续 |
| 碰撞/限位规划拒绝 | 查参考报告及见证；确认角度映射和场景，保留失败，不盲目缩小动作或关闭碰撞对 |
| EGL/视频生成失败 | 先用不渲染的 `staged-calibrate` 验证数值流程，再检查 EGL/驱动与中文字体；快照模式也需要渲染环境 |
| 有阶段报告但没有完整交付 | 检查 `checkpoint.json`；完成 `staged-suite`，以 `delivery.json` 为准 |
| 现场有反馈但 S3 拒绝 | 检查来源/尺度、采样事件、同步、状态、滤波和带宽；接收成功不等于观测合格 |
| URDF 导出拒绝 | 检查输入是否为通过且封存的 S4 分析、所请求阶段及模型摘要是否匹配 |

入门时不需要重新运行维护者的全部历史实验。修改核心方法后的测试/回归要求见 [AGENTS.md](../AGENTS.md)；维护者使用 `validate.sh` 时必须提供新输出目录，若要宣称历史数值对照，还需真实存在的冻结基线，不能用当前结果冒充。

后续需要正式多构型验收时，按 [R1](staged_dynamics_p0_r1.md)、[R2](staged_dynamics_p0_r2.md)、[R3](staged_dynamics_p0_r3.md)、[R4](staged_dynamics_p0_r4.md) 创建新的协议和独立测试批次。`staged-suite` 不接受 P0 配置；不要复用本机已消耗的 R4 正式数据重新选参或调整门槛。
