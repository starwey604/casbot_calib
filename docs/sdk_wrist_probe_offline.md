# 5ms SDK 缓存下的非换向腕部探针

非换向探针已接上 SDK 回调时间观测适配器。它沿用已有 **5Hz、0.004rad 正弦＋0.18rad/s 单向运动、2 秒测量窗**，提高加速力矩，并避开换向时的静摩擦。本文记录离线参考、观测适配与解析夹具验证；后续已完成[c00 右腕 pitch 正反两条 SDK Mock 采集](sdk_wrist_probe_pair.md)，现有训练数据未重拟合。

## 先运行离线适配

不需要 SDK overlay、ROS 服务或 H3 历史会话：

```bash
bash scripts/humble/container.sh python scripts/humble/adapt_sdk_wrist_probes.py \
  --output artifacts/sdk_wrist_probe_offline_new
```

入口复用 `staged_motion.wrist_probe_motion`，对两个中心、六个腕轴、正反方向生成 **24 条参考模板**，同时检查完整连续轨迹与 500Hz 指令折线预览。输出的 `position_plan_preview.csv` 只供离线审查，不是现场执行计划。

- c00：左右腕 pitch / roll 为 +0.20 / +0.30rad。
- c01：左右腕 pitch / roll 为 −0.20 / +0.15rad。
- 接近和回位均为 4 秒，前后保持各 3 秒。测量前后沿用原探针各 1 秒的 C3 渐入／制动。
- 复用专用探针 **1rad/s、8rad/s²、200rad/s³** 原门禁；普通 H4 多频轨迹的 1.5rad/s²、8rad/s³ 默认值不变。
- 没有创建正式 test 的角色或观测。

若只改观测算法，原参考、模型与场景未变，可显式复用已有参考检查：

```bash
bash scripts/humble/container.sh python scripts/humble/adapt_sdk_wrist_probes.py \
  --reuse-reference-checks artifacts/sdk_wrist_probe_offline_v4 \
  --output artifacts/sdk_wrist_probe_reanalysis_new
```

## 后续缓存标准为 5ms

[共享配置](../configs/sdk_wrist_probe.yaml)的 `cache_period_ns=5000000` 是用户确定的后续 SDK 腕部探索标准。新低频对照也改为 **3ms／5ms**，3ms 仅作较快对照；历史 3ms／20ms 数据与报告保留。通用 H1/H2 SDK Mock 的既有默认配置不随本次腕部实验改写。

这里的 5ms 是探索设计值，不是实测缓存年龄。**共享缓存周期与力矩／位置的相对错位分别处理**：前者统一为 5ms；本轮另在同一参考读出夹具上施加 effort 相对 ±5ms，检验其影响。

## 观测适配器怎样处理缓存反馈

`hardware_probe.probe_problem` 接收现有 SDK 执行器的公开 q、dq、effort、单调回调时刻，以及相对计划时间与测量窗。它不读取命令加速度或内部机械状态：

1. 在原始回调时间轴检查整段单向运动：目标轴有符号速度大于 0.02rad/s，其它轴速度不超过原专用探针的 0.001rad/s；不跨接收空窗。
2. 对完整 2 秒单向窗口，用二次趋势＋完整 0–30Hz Fourier 基表示位置、报告速度与唯一估算 Nm 力矩。趋势项保留单向位置漂移，避免将斜坡硬接成周期信号。
3. 加速度来自**拟合后的报告速度的解析导数**，并与位置解析速度交叉核对；不对阶梯状缓存值直接差分。所有发布均保留，不按相同数值去重。
4. 在**同一批回调行**上，将实际反馈重建的 q/dq/ddq 送入 Pinocchio，再对回归量、转子、重力与摩擦特征应用同一个时间加权投影。非线性摩擦特征先按公开报告速度计算，再投影。
5. 舍弃两端各 0.2 秒，输出 2ms 数值积分网格上的回归问题。网格点是计算位置，不是新增采样。源轨迹生成器自带的 4kHz 计划提示已从 SDK 元数据移除。

输出含 `regression`、`extra`、`target`、`static_feature`、q/dq/ddq 和完整预处理说明，可接已有刚体求解器。冻结摩擦后，刚体目标为：

```python
rigid_target = (problem['target']
                - problem['extra'] @ np.r_[viscous, coulomb]
                - static_friction[joint] * problem['static_feature'])
# problem['regression'] 和 rigid_target 交给现有 fit_rigid；重力约束沿用前序。
```

`target` 已扣除匹配投影的转子项，不能再次扣除；Nm 不改名为 current。正式采样、实机和专用高带宽资格仍保持 false。

## 离线结果：信号提高 14–21 倍

完整产物在 `artifacts/sdk_wrist_probe_offline_v4`，图为 `offline_adaptation.png`。24 条完整参考及 500Hz 折线预览均通过原门禁；连续速度／加速度／jerk 最大界分别为 **0.312rad/s、3.954rad/s²、124.24rad/s³**。

下表使用同一个 nominal 模型，对比原低频多轴参考与新分轴非换向参考的加速力矩预算；没有读取历史合成真值或正式测试观测。

| 轴 | 原低频 RMS（mNm） | 非换向 RMS（mNm） | 提高 |
| --- | ---: | ---: | ---: |
| 左 yaw | 0.148 | 2.605 | 17.6 倍 |
| 左 pitch | 0.0595 | 1.247 | 21.0 倍 |
| 左 roll | 0.0372 | 0.546 | 14.7 倍 |
| 右 yaw | 0.175 | 2.589 | 14.8 倍 |
| 右 pitch | 0.0595 | 1.256 | 21.1 倍 |
| 右 roll | 0.0391 | 0.546 | 13.9 倍 |

对 24 条参考构造理想读出：以 1000Hz 发布、共享 5ms 缓存，扫描四种缓存相位，再增加力矩相对 ±5ms，共 **144 组离线读出夹具**。不推进机械模拟器，也不运行采集器。

- **q/dq/effort 共用缓存、没有相对错位时**，全部解析夹具的惯量恢复误差小于 0.00004%。这验证了阶梯缓存处理及力矩／回归量接口在该理想模型上的数值闭合。
- **effort 相对错位 ±5ms 时**，最差 yaw 约 6.8%、pitch 约 12.5%、roll 约 **26.7%**。共享 5ms 缓存本身不是这些错位误差的替代解释。
- 对应 10% 惯量变化的同相力矩 RMS 预算约为 yaw **0.26mNm**、pitch **0.12mNm**、roll **0.055mNm**。相比笼统的总力矩残差，后续更应检查探针频点处的误差。

解析夹具假设轨迹跟踪准确、模型相符；其结果不含真实伺服跟踪、未知 effort 滤波或现场噪声。独立解析转子测试另覆盖 5ms 缓存、缓存帧噪声、回调抖动、停滞／其它轴运动／缺口／速度不一致反例。

## 后续双端采集进展

轨迹模板与公开反馈分析入口已用于[c00 右腕 pitch 正反采集](sdk_wrist_probe_pair.md)：实际速度、保持轴和 5Hz 力矩检查通过。随后[8 条同轴 SDK 训练与删组检查](sdk_wrist_pitch_campaign.md)也已完成，删构型／逻辑批次最差为 1.57%／1.77%；随后[第三姿态冻结候选开发验证](sdk_wrist_pitch_validation.md)也已通过；[c00 右腕 roll 两条筛查](sdk_wrist_roll_screen.md)现已完成，下一步先离线比较惯量与相位的分离方法。缓存、相对错位和力矩滤波分别登记，10% 原门槛不变。
