# 🤖 实机 CLI 联调与分阶段标定路径 (`hardware-*`)

如果你更倾向于用**命令行（CLI）**按固定步骤执行快照读取、单轴 1° 往返测试，或者在完成初步摸底后进入**整批自动化标定数据采集（H1 → H4）**，可以按本文档操作。

> 💡 **提示**：如果刚到现场，想先用可视化界面直观地拖动各轴、调 `Kp/Kd`、测正负方向或一键回零，建议先用 **[实机接入与现场联调手册 (`fieldwork/README.md`)](../fieldwork/README.md)**。

到场只读留证、未指定轴行为与厂家辅助验证的操作和回填位置，另见[现场测试与回填手册](sdk_field_validation.md)；本文保留 CLI 命令路径。

---

## 🗺️ 实机联调与标定阶段总览

| 阶段 | 目标与内容 | 对应指南 |
| :--- | :--- | :--- |
| **F1 / GUI** | 只读预检、保存安全零位、GUI 五字段手动调参与单轴短轨迹测试 | [现场联调手册 (`fieldwork`)](../fieldwork/README.md) |
| **H1（本文）** | CLI 只读起姿快照、单轴 1° 往返测试、小批量静态姿态采集跑通链路 | 本文第 1～3 节 |
| **H2** | **整批基础采集**：14 轴方向往返 + 100/250/500Hz 频率对比 + 64 姿态静态重力采集 | [H2 整批采集指南](hardware_batch_h2.md) |
| **H3** | **摩擦标定采集**：多姿态正反匀速运动摩擦 + 零 PD 力矩斜坡起动摩擦 | [H3 摩擦采集指南](hardware_friction_h3.md) |
| **H4** | **惯量与控制验证**：多姿态周期加速度激励、物理一致性惯量拟合、五档 PD 与前馈控制对照 | [H4 惯量与控制指南](hardware_inertia_h4.md) |

---

## ⚙️ 当前实机控制与保护约定

默认配置文件为 [configs/hardware_field.yaml](../configs/hardware_field.yaml)，核心运行机制如下：

1. **控制通道与范围**：向 `/motion/upper_body_debug` 申请上身调试模式，以默认 **500Hz** 向 `/upper_body_debug/joint_cmd` 发送双臂 **14 轴**指令（不填充腰、头、手）。
   * 现有执行路径假设未指定的腰、头、手保持原目标/输出；现场仍需区分保持、释放、缺省值0，允许只发14轴不等于这些轴的实际行为已验证。
2. **MIT 控制与前馈**：CASBOT 主机将 `position, velocity, kp, kd, effort` 五项直接转发给因克斯模组本地执行 PD，并对 `position` 做 1000Hz 线性插值。H1/H2 轨迹默认在 `effort` 中附带理论模型重力前馈（`gravity_feedforward: nominal`）。
3. **运行时的安全阈值**：
   * **启动静止要求**：开始执行前，14 轴实测速度均需 `< 0.02 rad/s`，且机器人处于 `READY` 或 `IDLE_MODE / STAND` 状态。
   * **起点自动重规划**：发指令前若发现机器人当前角度与规划起点偏差超过 `0.005 rad`，程序会自动重读快照、重新规划并做碰撞检查，无需手动中断重来。
   * **运行中越限处理**：实测速度超过 **`1.5 rad/s`** 会立即停止下发指令；实测位置超出 URDF 范围**仅告警并记录到 `observed_limits.csv`，不中断运行**；已移除跟踪误差（tracking error）中断门禁，避免因 `Kp/Kd` 偏小导致误停。
   * **通信超时**：关节反馈超过 **2 秒** 或模式状态超过 **3 秒** 未更新，自动停止下发。
   * **轨迹结束行为**：发完最后一帧指令后立即请求退出上身调试模式，不再卡死等待回位误差或停稳耗时。
4. **采集不被数据质量打断**：现场批次（H1–H4）只负责把运动跑完并录下原始数据，不会因为残差大、速度波动等数据质量指标中途报错锁死整批。只有发生真正的 SDK 通信、模式丢失或超速故障时，才会终止批次。

---

## 🚀 H1：命令行单步联调步骤

在仓库根目录下准备环境并输入现场 `ROS_DOMAIN_ID`（注意每次输出的目录不能提前存在）：

```bash
cd ~/codings/casbot_calib
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
read -r -p "现场 ROS_DOMAIN_ID: " calib_domain_id
```

### 1. 只读读取当前起姿快照

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-snapshot --config configs/hardware_field.yaml \
  --domain-id "$calib_domain_id" --output artifacts/site_snapshot_v1
```

* 该命令只读订阅 `/joint_states`，不申请控制模式，要求收齐 14 轴且当前静止。
* 完成后可查看 `artifacts/site_snapshot_v1/snapshot.csv`，核对各轴当前弧度值是否合理。

### 2. 规划并执行单轴 1° 小幅往返（Jog）

以左肩俯仰轴 `left_shoulder_pitch_joint` 为例，从刚采集的实测姿态出发，规划一个 `±1°` 的平滑往返轨迹（其余 13 轴保持不动），会自动完成 URDF 限位和双臂碰撞预检：

```bash
# 1. 离线规划单轴 1° 往返（不发指令）
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-jog --config configs/hardware_field.yaml \
  --snapshot artifacts/site_snapshot_v1 --joint left_shoulder_pitch_joint \
  --amplitude-deg 1 --output artifacts/site_jog_plan_v1

# 2. 实机执行该轨迹（会申请上身调试模式并下发位置流）
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-execute artifacts/site_jog_plan_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_jog_run_v1

# 3. 分析本次执行的发送/接收频率、轨迹跟踪情况
bash scripts/humble/container.sh casbot-calib hardware-analyse \
  artifacts/site_jog_plan_v1 --capture artifacts/site_jog_run_v1 \
  --output artifacts/site_jog_analysis_v1
```

### 3. 跑通小批量静态姿态采集（可选）

在正式跑 64 姿态整批标定前，可以先用 `--max-poses-per-split 1`（每臂仅挑 1 个训练姿态 + 1 个验证姿态）测试静态采集与重力辨识全链路：

```bash
# 1. 生成离线静态姿态预设库（若已生成过可跳过）
bash scripts/humble/container.sh casbot-calib plan-static-field \
  --output artifacts/site_static_preset_v1

# 2. 从实测快照出发，规划小批量静态采集轨迹
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-static artifacts/site_static_preset_v1 \
  --snapshot artifacts/site_snapshot_v1 --config configs/hardware_field.yaml \
  --max-poses-per-split 1 --output artifacts/site_static_plan_small_v1

# 3. 实机执行并生成分析报告与探索 URDF
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-execute artifacts/site_static_plan_small_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_static_run_small_v1

bash scripts/humble/container.sh casbot-calib hardware-analyse \
  artifacts/site_static_plan_small_v1 --capture artifacts/site_static_run_small_v1 \
  --output artifacts/site_static_analysis_small_v1
```

> ⚠️ **注意**：`--max-poses-per-split 1` 只是减少姿态数量，**并不代表目标姿态角度很小**。执行前请务必先完成单轴小幅检查，确保场地空间安全。

---

## 📦 后续：进入 H2 → H3 → H4 整批标定采集

当单轴运动和静态链路确认无误后，可按顺序进入整批采集（所有批次均支持 `--max-jobs N` 单步试跑检查点，以及 `Ctrl+C` 正常中断后的 `--resume` 断点续采）：

### 1. H2 方向、频率与静态重力批次
详见 [H2 现场采集指南](hardware_batch_h2.md)：
```bash
# 规划 H2 批次（14轴往返 + 12条频率测试 + 64个静态姿态）
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-batch \
  --config configs/hardware_batch.yaml --profile configs/hardware_field.yaml \
  --preset artifacts/site_static_preset_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_batch_v1

# 先试跑第 1 个任务确认正常，随后断点续采跑完剩余任务
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_batch_v1 --max-jobs 1
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_batch_v1 --resume
```

### 2. H3 运动摩擦与起动摩擦批次
基于 H2 辨识出的重力模型，继续采集匀速摩擦与零 PD 起动力矩斜坡（详见 [H3 指南](hardware_friction_h3.md)）：
```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-friction \
  --config configs/hardware_friction.yaml --profile configs/hardware_field.yaml \
  --gravity artifacts/site_batch_v1 --domain-id "$calib_domain_id" \
  --output artifacts/site_h3_v1

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h3_v1 --max-jobs 1
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h3_v1 --resume
```

### 3. H4 惯量激励与控制对照批次
基于 H3 的重力与摩擦结果，继续采集周期加速度激励并完成 PD 选择与控制对照（详见 [H4 指南](hardware_inertia_h4.md)）：
```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-inertia \
  --config configs/hardware_inertia.yaml --profile configs/hardware_field.yaml \
  --prior artifacts/site_h3_v1 --domain-id "$calib_domain_id" \
  --output artifacts/site_h4_v1

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h4_v1 --max-jobs 1
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h4_v1 --resume

# 离线重分析与生成 3D 运动回放视频
bash scripts/humble/container.sh casbot-calib hardware-analyse-batch \
  artifacts/site_h4_v1 --output artifacts/site_h4_analysis_v1
bash scripts/humble/container.sh casbot-calib hardware-render-batch \
  artifacts/site_h4_v1 --output artifacts/site_h4_video_v1 --fps 12
```

---

## 📂 输出目录文件速查

| 文件 / 目录 | 说明 |
| :--- | :--- |
| `raw_state/` | S1 原始 CDR 消息、话题端点信息与接收时间戳记录 |
| `execution/commands.csv` | 计划时间槽、实际发布时间戳、14 轴下发角度/速度/前馈力矩/PD |
| `execution/feedback.csv` | 本地接收时间（ns）、消息发布 `stamp`、14 轴实测 `q / dq / effort` |
| `execution/observed_limits.csv` | 运行中触发的位置/速度越限告警与恢复记录 |
| `execution/events.csv`, `result.json` | 模式切换事件、是否发生重规划或中断、最终完成状态 |
| `candidate_exploratory.urdf` | 根据静态采集辨识出的重力参数导出的候选 URDF 模型 |
| `calibration.json`, `report.json` | 辨识出的动力学参数、拟合残差统计及频率响应汇总 |
