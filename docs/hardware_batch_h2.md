# 🚀 H2：实机整批采集（14 轴方向、频率测试与静态重力）

H2 将 [H1 单步命令](hardware_execution_path.md) 打包成一套自动化现场采集批次，按固定顺序依次执行：
**14 轴方向往返 → 100/250/500Hz 频率分档重复测试 → 64 姿态正反静态采集 → 重力参数拟合与 3D 视频回放**。

---

## 🧭 一、默认批次包含哪些任务

默认批次配置为 [configs/hardware_batch.yaml](../configs/hardware_batch.yaml)，底层硬件参数来自 [configs/hardware_field.yaml](../configs/hardware_field.yaml)：

| 阶段 | 任务安排（共 90 个采集单元） | 目的与记录内容 |
| :--- | :--- | :--- |
| **1. 方向检查** | 双臂 14 轴依次各做一次 `±1°` 正反往返（默认 500Hz，共 14 条） | 验证 14 轴指令与反馈映射、运动跨度及正负方向 |
| **2. 频率测试** | 左/右肩俯仰（`shoulder_pitch`）各跑 `100 / 250 / 500Hz`，每档重复 2 次（随机顺序，共 12 条） | 对比不同发送频率下的实际发布率、通信抖动与轨迹响应延迟 |
| **3. 静态重力** | 每臂 24 个训练姿态 + 8 个验证姿态（共 64 个姿态，每个姿态包含正反两个接近方向） | 采集静止窗口内的关节角度与估算力矩均值 |
| **4. 汇总拟合** | 自动合并所有有效静态窗口，调用 Pinocchio 物理一致求解器 | 输出探索重力参数（`calibration.json`）、候选 URDF 及汇总报告 |

* 如果只想先快速跑通整批流程，可在 `configs/hardware_batch.yaml` 中调小 `max_poses_per_split`（限制每臂静态姿态数量），或只保留部分关节/阶段。
* **频率测试如何保证公平对比**：第一条频率任务启动前会记录当前实测姿态作为冻结锚点；后续 100/250/500Hz 任务均使用完全相同的连续参考轨迹（仅客户端发送周期不同），并在报告中单独统计核心比较窗口内的发布表现。

---

## 🛠️ 二、第 1 步：离线生成批次计划（不连机器人）

在仓库根目录下运行以下命令，将配置、碰撞场景和任务顺序打包冻结到批次目录中（输出目录不能提前存在）：

```bash
cd ~/codings/casbot_calib
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
read -r -p "现场 ROS_DOMAIN_ID: " calib_domain_id

# 1. 若尚未生成静态姿态预设库，先生成一份
bash scripts/humble/container.sh casbot-calib plan-static-field \
  --output artifacts/site_static_preset_v1

# 2. 生成 H2 批次计划（仅离线规划，不下发任何运动指令）
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-batch \
  --config configs/hardware_batch.yaml --profile configs/hardware_field.yaml \
  --preset artifacts/site_static_preset_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_batch_v1
```

> 💡 **注意**：批次一旦规划生成，配置即被封存在 `artifacts/site_batch_v1/protocol/` 中。如果之后修改了 `configs/*.yaml`，需要重新指定一个新目录运行 `hardware-plan-batch`。

---

## ▶️ 三、第 2 步：启动采集与断点续采

以下命令会连接实机 SDK，自动申请上身调试模式并依次执行采集单元：

```bash
# 推荐：先用 --max-jobs 1 只跑第 1 个单元，确认机器人动作与日志正常
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_batch_v1 --max-jobs 1

# 确认无误后，加 --resume 继续跑完剩余单元（也可随时加 --max-jobs N 分段跑）
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_batch_v1 --resume
```

### 运行中的中断与重规划规则

1. **发指令前起点微偏（自动重规划）**：每个采集单元启动前都会重新读取当前静止姿态。如果在准备阶段发现当前角度与规划首点偏差超过 `0.005 rad`（尚未申请模式或发指令），程序会自动保存旧快照并重新规划当前单元路径，不会中断整批。
2. **正常暂停与 `Ctrl+C`（可续采）**：
   * 通过 `--max-jobs N` 到达检查点，或在运行中主动按 `Ctrl+C` 中断，当前未跑完的单元会标记为 `interrupted`。
   * 下次带 `--resume` 启动时，已完成单元会自动跳过，被中断的单元会从当前新起姿重新读取快照并执行。
3. **数据质量不阻断采集**：所有运动单元只跑一次，数据原样保存。即使某条数据波动较大或缺少部分数值，也只会在报告中标记该项分析不可用，继续往下采其它任务。
4. **SDK / 通信 / 越限故障（终止整批）**：如果运行中发生超速（`> 1.5 rad/s`）、反馈超时（关节 2s / 模式 3s）或控制模式丢失，程序会立即停止发送、记录故障日志并终止整个批次（故障批次不可用 `--resume` 强行续采，需排查原因后新建批次）。

---

## 🔍 四、第 3 步：离线重分析与生成 3D 回放视频

采集完成后（或中途暂停时），可以在不连接机器人的情况下重新生成分析报告，或将实测反馈渲染为 3D 回放视频：

```bash
# 离线重新运行数据分析与重力拟合
bash scripts/humble/container.sh casbot-calib hardware-analyse-batch \
  artifacts/site_batch_v1 --output artifacts/site_batch_analysis_v1

# 将批次内各单元的实测关节反馈渲染为 3D 视频（每条限制最长展示 4 秒）
bash scripts/humble/container.sh casbot-calib hardware-render-batch \
  artifacts/site_batch_v1 --output artifacts/site_batch_video_v1 \
  --fps 12 --max-seconds-per-attempt 4
```

---

## 📂 五、批次目录结构速查

| 目录 / 文件 | 说明 |
| :--- | :--- |
| `protocol/` | 冻结的硬件配置、任务列表、碰撞场景与静态预设副本 |
| `attempts/<任务名>/attempt_NNN/` | 每个采集单元的独立快照、轨迹计划、原始 CDR、`commands.csv` 与 `feedback.csv` |
| `frequency_anchor/` | 第一条频率测试冻结的 14 轴比较锚点姿态 |
| `reports/` | 汇总报告（`report.md` / `report.json`）、各任务状态表 `jobs.csv`、频率响应图及候选 URDF |
| `faults/` | 发生通信/模式/超速终止时的故障记录日志 |
| `latest.json`, `progress.json` | 当前批次最新进度与报告摘要入口 |
