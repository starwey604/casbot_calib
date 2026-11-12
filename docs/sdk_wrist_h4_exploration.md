# 当前 SDK 的腕部 H4：本机探索

这一轮回答两个问题：**公开 SDK 反馈能否走完双腕采集、惯量拟合和独立预测？缓存及通道时序变化会让结果偏多少？** 使用隔离的 ROS SDK Mock，不连接机器人。

它使用现有 H4 的回调时间观测方法，**不是将已经通过 R1/R2 的 4kHz 持续运动处理直接套到 SDK 数据上**。两种数据的时间含义不同。探索结果用于决定下一次试验，不替代正式资格。

## 后续采用 5ms 标准

2026-11-12 用户确定：后续 SDK 腕部探索采用 **5ms 共享缓存**，新低频对照为 3ms／5ms；历史 20ms 压力测试保留。非换向探针已完成[离线适配与信号预算](sdk_wrist_probe_offline.md)，随后[c00 两条探针](sdk_wrist_probe_pair.md)及[8 条右腕 pitch SDK 训练／删组检查](sdk_wrist_pitch_campaign.md)已完成，[第三姿态冻结候选开发验证](sdk_wrist_pitch_validation.md)也已通过；[c00 右腕 roll 两条筛查](sdk_wrist_roll_screen.md)已完成，下一步先离线处理相位混叠；原低频记录不回写。

## 运行一轮

先准备 [Humble 环境](humble_environment.md) 和可选 SDK overlay：

```bash
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/explore_sdk_wrist_h4.py run \
  --output artifacts/sdk_wrist_h4_exploration_new \
  --operator-probes artifacts/dvt2_004_wrist_sustained_development_v1/r2/operator_probe_set.npz \
  --domain-id 227
```

`--operator-probes` 只读取原 R2 的 128 个公开 `q/dq/ddq` 算子评估点，不读取正式测试数据。这份本地产物不随 Git 分发；缺失时应提供原文件，不能把新随机点叫作原 128 点。

脚本自行启动、关闭所拥有的 Mock 进程；每个连接使用私有命名空间和 localhost。没有接收实机 profile 的选项，底层也拒绝默认硬件连接。输出目录必须是新目录。SDK／通信／模式故障终止整轮，不清故障、不自动恢复或续接；失败记录保留，新尝试另建目录。

## 首轮冻结设计（历史 3ms／20ms）

| 项目 | 首轮设置 |
| --- | --- |
| 激励范围 | 左右腕 yaw、pitch、roll 共 6 轴；其余 8 轴保持 |
| 训练姿态 | 两侧腕 pitch / roll：c00 为 +0.20 / +0.30 rad；c01 为 −0.20 / +0.15 rad |
| 独立预测姿态 | 两侧腕 pitch / roll 为 0 / +0.225 rad |
| 记录 | 每姿态两批，共 4 条训练、2 条独立预测；每个缓存条件各采一组，共 12 条 |
| 周期 | 8 秒 × 3 周期，舍弃第一周期，保留 16 秒；C3 渐入、制动和回位 |
| 参考与保护 | 复用 H4 幅值、500Hz 客户端、1.5 rad/s² 和 8 rad/s³ 连续界、现场跟踪／速度／反馈年龄限制、Coal 路径检查；Mock 看门狗仍为 100ms |
| 缓存条件 | 预登记共享缓存 3ms 与 20ms；公开反馈仍按 1000Hz 发布 |
| 冻结先验 | nominal 重力和声明的合成摩擦夹具；**没有借用或伪造一份已通过 H3 的现场结果** |
| 合成执行增益 | 沿用已有 H4 合成夹具 Kp 下限 30；不修改实机 H4 默认值 |

当前本机没有历史已验收 H3 会话，因此这一入口专门验证腕部观测与拟合功能，不给出 H3 前序验收、PD 选择或四组控制对照结论。完整现场 H4 仍走 [原 H4 入口](hardware_inertia_h4.md)，要求真实完成的 H3 输入。

本轮是 **0.625/0.75Hz 以下的多频参考与最高 2Hz 拟合频带**，不是 5Hz 非换向专用探针，也不是完整 14 轴惯量恢复。

## 分析什么

分析只读取执行器的公开位置、速度、估算 effort、回调时刻及冻结计划：

1. 保留原始 CDR、命令、全部发布／接收记录。相邻数值相同的比例只作描述，不据此去重或计算独立采样率。
2. 复用 `hardware_periodic.periodic_problem`：以回调时间做时间加权周期拟合；实际位置解析求导，与接收速度交叉核对；力矩和 Pinocchio 回归量使用同一 Fourier 频带。唯一观测为估算 Nm，不把命令前馈或内部 plant 输出当测量。
3. 复用 H4 物理一致求解器、重力等式约束和 ridge=1e-4。完整训练与删 c00、删 c01、删 b00、删 b01 均重新拟合；原始每条训练记录的行权重预算固定，删组后不重新归一化。
4. 在原 128 点比较质量矩阵对角项，使用原 10% 数值和 1e-6 kg·m² 分母下限。这里是**探索性删组诊断**，不是完整 P0 R2 验收。
5. 预登记 effort 相位偏移 ±5/±15ms、额外一阶低通 5/20ms，以及共同 Fourier 上限 1/1.5/2Hz。每项完整重拟合及删组；不根据独立预测结果挑最好偏移、滤波或频带。相位／低通为对已拟合公开周期信号施加的压力测试，不是实测出的 SDK 延迟或滤波器。
6. 独立预测沿用 H4 原绝对／归一化残差条件；另报动态力矩量级，避免 0.2Nm 归一化下限掩盖小惯量信号。残差小或删组稳定都不单独证明惯量准确。

## 结果和只读复核

- `protocol/experiment.json`：预登记姿态、种子、缓存与分析变体。
- `cache_*/<记录>/capture/raw_state/raw/`：原始 SDK 记录。
- `cache_*/<记录>/capture/execution/`：命令、公开反馈和执行状态。
- `analysis/public_capture_audit.json`：实际速度、保持轴、力矩、回调空窗与位置／速度一致性。
- `analysis/report.json`：18 个预登记变体、90 次拟合及删组、独立预测和跨缓存比较。

可只读重分析，不需要启动 SDK：

```bash
bash scripts/humble/container.sh python scripts/humble/explore_sdk_wrist_h4.py analyse \
  --source artifacts/sdk_wrist_h4_exploration_new \
  --output artifacts/sdk_wrist_h4_reanalysis_new

bash scripts/humble/container.sh python scripts/humble/verify_sdk_wrist_h4.py \
  artifacts/sdk_wrist_h4_exploration_new --output artifacts/sdk_wrist_h4_verification_new
```

重分析进程禁止打开任何 `evaluation` 目录。复核先在新的禁止真值进程中重建公开观测并重新拟合，再在独立评分中读取预登记的合成 plant 参数，对照惯量误差，输出图与报告。合成真值不进入拟合或选择。

`sample_time_known`、`sampling_qualification_passed`、`formal_hardware_calibration_accepted` 和专用高带宽腕部资格始终为 false。真实采样时间、新帧序号和缓存年龄仍未知；当前 SDK 没有因此新增这些能力。

## 首轮结果：功能跑通，惯量不接受（2026-11-12）

完整会话：`artifacts/sdk_wrist_h4_exploration_v2`；独立复核：`artifacts/sdk_wrist_h4_verification_v2`。

12 条记录、18 个分析变体、90 次物理一致拟合全部完成。新进程禁止读取 `evaluation` 后，从公开反馈重建并重拟合 90 次，参数最大差为 **0**。实际接收路径全部通过原碰撞检查；相关 23 项测试通过。

以下均使用原 128 个算子点、10% 数值及原分母下限。删组和真值误差都没有通过：

| 指标 | 3ms 共享缓存 | 20ms 共享缓存 |
| --- | ---: | ---: |
| 独立预测最大逐轴 RMSE | 0.628 mNm | 0.645 mNm |
| 删构型最差惯量差 | **60.87%**，删 c00，右腕 pitch | **94.70%**，删 c00，右腕 pitch |
| 删批次最差惯量差 | **143.70%**，删 b00，左腕 pitch | **105.20%**，删 b00，右腕 pitch |
| 对独立合成真值的腕部最差惯量误差 | **91.19%** | **90.57%** |
| 对照：nominal 先验的腕部最差真值误差 | 41.97% | 41.97% |
| 对同一批信号施加 ±5ms effort 偏移，完整拟合相对原结果的最差腕部变化 | 44.51% | 73.16% |
| 施加 20ms effort 一阶低通，对原结果的最差腕部变化 | 85.29% | 85.30% |

因此，**本轮残差小，惯量仍不可信，且拟合比 nominal 先验更偏离合成真值**。不能把 H4 的低频预测残差条件当作腕部惯量验收。全部预登记频带／偏移／低通变体的删组诊断也都超 10%，没有从中选一个“最好”的结果替代基准。

两次缓存条件下的完整模型最差腕部差为 47.33%，位于右腕 pitch；这是分次运行的条件比较，包含调度与复测差异，不是对真实 SDK 缓存误差的标定。所有记录的腕部速度 RMS 至少约 0.0529 rad/s，加速度 RMS 至少约 0.1227 rad/s²；其它轴实测最大速度为 0.00172 rad/s。位置解析导数与反馈速度的一致性 RMSE 最大 0.00075 rad/s。观测力矩减去 nominal 重力与声明摩擦后的周期 RMS 约 0.66–1.20 mNm，与拟合残差在相近量级；其中还混有未闭合的摩擦／观测误差，不能全称为惯量信号。

独立评分再用合成真值模型计算公开重建轨迹上的加速力矩：左右腕 pitch 的平均 RMS 约 0.129／0.127 mNm，roll 约 0.063／0.068 mNm（`synthetic_signal_budget.json`）。这些信号小于对应预测残差；当前低频激励尚不足以把腕部惯量与摩擦／观测失配可靠分离。这是模型计算的信号预算，不是新增的独立物理测量。

3ms／20ms 条件中，相邻公开数值完全相同的比例约 72.25%／95.16%。这些记录没有去重；比例不是设备新帧率或缓存年龄。所有正式采样／实机／专用腕部资格仍为 false。

**下一步推荐专用非换向腕部探针的 SDK 观测验证**：提高惯量信号并避开换向静摩擦，在新协议里冻结参考与专用保护包络，再比较力矩／位置相对时序和同带宽处理。当前结果支持改变激励与观测设计，不支持放宽 10% 或把当前候选用于控制。首轮未运行 5Hz 非换向探针，也未运行 H4 的 PD／前馈四组对照；后续小范围探针结果见[两条采集记录](sdk_wrist_probe_pair.md)。

首轮失败会话 `artifacts/sdk_wrist_h4_exploration_v1` 保留：3 条完成，第 4 条出现 119.69ms 命令空窗和 120.31ms 接收空窗，触发原 100ms 合成看门狗并丢失模式，整轮终止、没有清故障／续采。v2 是新建独立会话，未放宽保护；本机完成一轮不构成硬实时保证。
