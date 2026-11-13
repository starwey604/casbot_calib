# 🦾 H4：实机惯量激励与控制对照指南

H4 接在 [H3 摩擦标定](hardware_friction_h3.md) 之后，是分阶段动力学标定的收官阶段。它在**同时冻结 H2 重力可观测约束与 H3 摩擦参数**的前提下，依次完成：
1. **多姿态周期加速度激励（6 条）**：14 轴同时跟踪平滑傅里叶周期轨迹，通过频域解析求导与 Clarabel 半正定锥规划（PSD Cone）拟合 140 维刚体惯性参数 $\theta$，并导出候选 `candidate_exploratory.urdf`。
2. **五档 PD 增益选择（5 条）**：在专用选择轨迹上测试 `0.5x / 0.75x / 1.0x / 1.25x / 1.5x` 五档增益缩放，选出最优 PD 档位。
3. **2×2 四组前馈与增益控制对照（4 条）**：在独立验证轨迹上对比“H3 先验重力前馈 vs H4 完整动力学前馈” × “原始 PD vs 优化后 PD”，量化动力学标定带来的真实跟踪收益。

---

## 🧭 一、默认批次包含哪些任务

默认配置文件为 [configs/hardware_inertia.yaml](../configs/hardware_inertia.yaml)，底层硬件参数来自 [configs/hardware_field.yaml](../configs/hardware_field.yaml)：

| 阶段 | 任务安排（共 15 个采集单元） | 轨迹设计与目的 |
| :--- | :--- | :--- |
| **1. 周期惯量激励** | **4 条训练轨迹**（2 个姿态中心 × 2 个随机种子）+ **2 条独立验证轨迹**（1 个独立中心 × 2 个验证种子），共 **6 条** | 周期 $T = 8\text{s}$，连续运行 4 个周期（丢弃前 2 个过渡周期，取后 2 个稳态周期做 16 阶傅里叶拟合），最大速度/加速度限制在 `1.5 rad/s` 和 `1.5 rad/s²` 以内。 |
| **2. 五档 PD 选择** | 同一条选择轨迹（`seed: 601`）依次跑 `0.5, 0.75, 1.0, 1.25, 1.5` 倍 $K_p$（$K_d$ 按平方根缩放），共 **5 条** | 统一使用 H3 物理代表模型前馈，根据稳态窗口的关节跟踪 RMSE 选出最优增益倍数。 |
| **3. 四组控制对照** | 同一条独立验证轨迹（`seed: 701`）依次跑 `prior_original`、`prior_tuned`、`identified_original`、`identified_tuned`，共 **4 条** | 严格分离“完整惯量/科氏力/摩擦前馈的贡献”与“单纯调整 PD 增益的贡献”。 |

> 💡 **现场联调提示**：默认配置为 14 轴同时激励。如果想在现场先单轴小范围测试，可在 `configs/hardware_inertia.yaml` 中将 `joints: all` 改为已跑通 H3 的单个关节名称（其余未选轴会自动保持静止，并在报告中标记为未激励先验）。

---

## 🛠️ 二、第 1 步：离线生成 H4 批次计划（不连机器人）

H4 需要传入已完成 H3（含运动摩擦与起动摩擦）的批次根目录 `--prior`。在仓库根目录下执行（注意输出目录不能提前存在）：

```bash
cd ~/codings/casbot_calib
read -r -p "现场 ROS_DOMAIN_ID: " calib_domain_id

# 生成 H4 惯量与控制批次计划（仅离线规划与连续碰撞检查，不下发任何指令）
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-inertia \
  --config configs/hardware_inertia.yaml \
  --profile configs/hardware_field.yaml \
  --prior artifacts/site_h3_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_h4_v1
```

* 规划阶段会使用精确的 Bernstein 分段多项式系数检查整条进场、周期激励、制动与退出轨迹的位置、速度、加速度（$\le 1.5\text{ rad/s}^2$）、加加速度（Jerk $\le 8.0\text{ rad/s}^3$）及 Coal 双臂 10mm 防碰撞余量。

---

## ▶️ 三、第 2 步：启动采集与断点续采

以下命令会连接实机 SDK，申请 `/motion/upper_body_debug` 模式并依次执行 15 个采集单元：

```bash
# 1. 先用 --max-jobs 1 试跑第 1 条周期激励轨迹，确认机器人动作平顺
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h4_v1 --max-jobs 1

# 2. 确认无误后，带 --resume 跑完剩余的惯量激励、PD 选择与四组控制对照
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h4_v1 --resume
```

### 算法与运行机制要点

1. **为什么不用差分求角加速度？**：
   实机反馈存在缓存重复帧与回调抖动。H4 在稳态周期窗口内对实测位置做带梯形时间权重的 **16 阶傅里叶最小二乘拟合**，直接对拟合出的三角函数做**解析求导**得到角速度与角加速度；同时先在原始实测速度上计算非线性摩擦特征，再与 Pinocchio 140 列刚体回归矩阵统一投影到相同频带。
2. **保证不破坏前序重力与物理合法性**：
   调用 Clarabel 求解 140 维刚体参数时，强制施加 14 个连杆的 $4 \times 4$ 伪惯性矩阵半正定约束（`PSDTriangleConeT(4)`）以及 H2 静态重力等式约束（`ZeroConeT`），并在拟合后自动复核 H2 静态验证集与 H3 匀速验证集残差。
3. **四组控制前馈如何下发？**：
   进入第 3 阶段（四组控制对照）时，Python 端会预先用 Pinocchio 算好整条参考轨迹的完整刚体 + 摩擦力矩时间表（封存为 CSV），由 C++ `position_executor` 按 500Hz 时间槽同步填入 `effort` 字段下发。

---

## 🔍 四、第 3 步：离线重分析与生成 3D 回放视频

```bash
# 离线重新运行惯量拟合、前序复核与控制对比分析
bash scripts/humble/container.sh casbot-calib hardware-analyse-batch \
  artifacts/site_h4_v1 --output artifacts/site_h4_analysis_v1

# 将实测关节反馈渲染为中文 3D 视频
bash scripts/humble/container.sh casbot-calib hardware-render-batch \
  artifacts/site_h4_v1 --output artifacts/site_h4_video_v1 --fps 12
```

---

## 📂 五、输出结果文件速查

在批次目录的 `reports/report_NNNN/`（或离线分析输出目录）下，核心结果文件如下：

| 文件 / 目录 | 内容说明 |
| :--- | :--- |
| `inertia/report.json` | 140 维刚体回归秩、奇异值、独立验证轨迹力矩预测 RMSE、通道时移敏感度（`±5ms`）及前序 H2/H3 复核结果 |
| `inertia/candidate_exploratory.urdf` | 写入辨识后质量、质心与惯性张量的候选 URDF 模型（已通过 Pinocchio 重加载校验） |
| `inertia/inertia_validation.png` | 独立验证轨迹的实测力矩 vs 模型预测力矩对比曲线图 |
| `control_comparison.json` / `.csv` / `.png` | 五档 PD 选择结果及 `prior_original` / `prior_tuned` / `identified_original` / `identified_tuned` 四组跟踪误差对比图表 |
