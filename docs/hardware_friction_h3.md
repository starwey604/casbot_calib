# 🦾 H3：实机运动摩擦与起动摩擦采集指南

在完成 [H2 静态重力采集](hardware_batch_h2.md) 并获得重力候选模型后，H3 阶段将该重力模型**冻结**，继续在实机上完成两项摩擦标定实验：
1. **多姿态、多速度正反向匀速运动**：分离并拟合每轴的**黏滞摩擦系数 $b$** 与**库仑摩擦系数 $f_c$**。
2. **零 PD 静置与力矩斜坡（Breakaway）**：在仅靠外部重力前馈托住机械臂的前提下，缓慢施加线性力矩斜坡，测定每轴正反向的**静摩擦起动阈值区间 $f_s$**。

---

## 🧭 一、默认批次包含哪些任务

默认配置文件为 [configs/hardware_friction.yaml](../configs/hardware_friction.yaml)，底层通信与安全限制来自 [configs/hardware_field.yaml](../configs/hardware_field.yaml)：

| 阶段 | 任务安排（完整默认共 560 个采集单元） | 控制方式与目的 |
| :--- | :--- | :--- |
| **1. 运动摩擦（匀速往返）** | 14 轴逐轴执行：每轴 **2 个训练姿态 + 1 个验证姿态**，分别跑 **3 档训练速度**（`0.04, 0.08, 0.12 rad/s`）与 **2 档验证速度**（`0.06, 0.10 rad/s`），共 **112 条** | 下发位置/速度参考 + 显式保持 PD + **H2 冻结重力前馈**；提取匀速段并通过“相同角度位置正反配对半差”消除残余重力误差，拟合 $b$ 和 $f_c$。 |
| **2. 起动摩擦（零 PD 斜坡）** | 14 轴逐轴执行：每轴 **2 个姿态 × 正反 2 个方向 × 2 种静置时间**（`1s / 2s`）**× 2 种斜坡速率**（`0.12 / 0.24 Nm/s`）**× 2 次重复**，共 **448 条** | 先用 PD 停稳静置，随后**将目标轴设为 $K_p=K_d=0$**（其余 13 轴保持 PD），在冻结重力前馈基础上叠加力矩斜坡（上限 `0.5 Nm`）；一旦检测到位移 $> 0.0002\text{ rad}$ 或速度 $> 0.005\text{ rad/s}$ 持续 `10ms`，立即撤除斜坡并恢复 PD 保持。 |

> 💡 **现场联调建议**：如果第一次在实机上跑 H3，可以先复制或修改 `configs/hardware_friction.yaml`，将 `joints: all` 改为单个关节（例如 `joints: [left_shoulder_pitch_joint]`），新建一个测试批次快速跑通全流程，确认保持增益和动作表现正常后再跑全 14 轴。

---

## 🛠️ 二、第 1 步：离线生成 H3 批次计划（不连机器人）

H3 需要引用 H2 已经生成的重力结果目录（可以直接传 H2 批次根目录 `artifacts/site_batch_v1`）。在仓库根目录下执行（注意输出目录不能提前存在）：

```bash
cd ~/codings/casbot_calib
read -r -p "现场 ROS_DOMAIN_ID: " calib_domain_id

# 生成 H3 摩擦批次计划（仅离线规划与碰撞检查，不下发任何指令）
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-friction \
  --config configs/hardware_friction.yaml \
  --profile configs/hardware_field.yaml \
  --gravity artifacts/site_batch_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_h3_v1
```

* 规划器会自动核对 H2 重力模型的 URDF 摘要、关节顺序、重力方向与力矩尺度，并将模型与配置封存在 `artifacts/site_h3_v1/protocol/` 下。

---

## ▶️ 三、第 2 步：启动采集与断点续采

以下命令会连接实机 SDK，申请 `/motion/upper_body_debug` 调试模式并依次执行采集单元：

```bash
# 1. 先用 --max-jobs 1 试跑第 1 个采集单元，确认实机运动与日志正常
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h3_v1 --max-jobs 1

# 2. 确认无误后，带 --resume 继续跑完剩余单元（支持随时 Ctrl+C 正常中断后续采）
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h3_v1 --resume
```

### 运行与保护机制说明

1. **发令前起点偏差自动重规划**：每个采集单元启动前若发现当前静止姿态与规划起点偏差 `> 0.005 rad`（尚未申请模式或发指令），会自动重读当前姿态并重新生成进入轨迹。
2. **零 PD 起动保护（`SdkBreakaway`）**：
   * 在起动实验进入 `kp=kd=0` 阶段时，如果还没加斜坡力矩关节就已经明显滑动（说明重力前馈与实际重力偏差过大），或者其余 13 个非目标保持轴偏移超过 `0.003 rad`，程序会立即停止并终止批次。
   * 一旦触发起动确认（移动超过 `0.0002 rad` 且速度超 `0.005 rad/s` 持续 `10ms`），C++ 执行器当即切断力矩斜坡、恢复 PD 保持并平滑退回起点。
3. **数据质量不阻断采集**：所有运动单元只采一次；若某个窗口速度波动较大或未能解出数值，只会在报告中标记该项分析不可用，继续采集后续任务。只有发生真正的 SDK 通信超时、模式丢失或超速（`> 1.5 rad/s`）故障时才会终止整批。

---

## 🔍 四、第 3 步：离线重分析与生成 3D 回放视频

采集完成后（或中途暂停时），可随时离线重新运行摩擦拟合或渲染 3D 回放视频：

```bash
# 离线重新运行摩擦参数拟合与验证分析
bash scripts/humble/container.sh casbot-calib hardware-analyse-batch \
  artifacts/site_h3_v1 --output artifacts/site_h3_analysis_v1

# 将实测关节反馈渲染为中文 3D 视频
bash scripts/humble/container.sh casbot-calib hardware-render-batch \
  artifacts/site_h3_v1 --output artifacts/site_h3_video_v1 --fps 12
```

---

## 📂 五、输出结果文件速查

在批次目录的 `reports/report_NNNN/`（或离线分析输出目录）下，主要生成以下文件：

| 文件 / 目录 | 内容说明 |
| :--- | :--- |
| `moving_friction.json` | 14 轴黏滞摩擦 $b$、库仑摩擦 $f_c$ 拟合结果及独立验证集预测残差（RMSE） |
| `breakaway.json` | 零 PD 起动实验的最后静止力矩、首次可分辨运动力矩、确认力矩及正反起动阈值区间 |
| `friction_parameters.csv` | 便于直接查看的 14 轴摩擦参数汇总表（`viscous`, `coulomb`, `static`） |
| `onset_thresholds.csv` | 各姿态、方向、静置时间与斜率下的起动阈值明细表 |
| `runtime_friction.json` | 封存供下一阶段（[H4 惯量标定](hardware_inertia_h4.md)）读取的摩擦参数包 |
