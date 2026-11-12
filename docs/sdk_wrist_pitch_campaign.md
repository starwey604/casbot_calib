# 右腕 pitch：8 条 SDK 探针的删组稳定性

**6 条补采已完成。** 与原两条只读绑定后，5ms SDK Mock 的两构型×正反向×两逻辑批次共 8 条记录，在原 128 个算子评估点上，删构型、删批次及删实际会话的最差对角惯量差均在 **10%** 内。

本轮会话：`artifacts/sdk_wrist_pitch_campaign_v1`；独立复核：`artifacts/sdk_wrist_pitch_campaign_verification_v1`。本轮只涉及右腕 pitch 的本机 SDK 探索，原 4kHz 训练数据与正式 test 未消费、未改写。

## 记录与批次怎样组成

| 逻辑批次 | 构型 | 方向 | 数据来源／实际 Mock 会话 |
| --- | --- | --- | --- |
| b00 | c00 | 正反两条 | 原 `sdk_wrist_pitch_c00_pair_v1`，只读绑定，实际会话 `source_pair` |
| b00 | c01 | 正反两条 | 本轮新增，实际会话 `b00_extension` |
| b01 | c00、c01 | 各正反两条 | 本轮新增，实际会话 `b01` |

b00 由两个已分开的实际会话组成；因此协议在补采前同时登记了**两逻辑批次**和**三实际会话**的删组检查。原记录的计划、原始反馈和分析未回写。删 b01 与删实际会话 b01 使用相同记录子集，复用同一拟合。

c00 的左右腕 pitch／roll 为 +0.20／+0.30rad，c01 为 −0.20／+0.15rad。继续使用原 5Hz、0.004rad 正弦＋0.18rad/s 单向运动、2 秒测量窗及两端各 0.2 秒裁剪。缓存配置 5ms，命令 500Hz，声明的合成摩擦、名义重力前馈、Kp 下限 30 和既有运动／故障规则保持。

## 数据质量与力矩信号

8 条记录全部通过原速度、保持轴、接收缺口、位置／速度一致性和实际接收路径检查：

- 有符号速度最低 **0.05104rad/s**，全程不换向；最大速度 **0.31462rad/s**。
- 其它 13 个双臂关节最大速度 **0.000471rad/s**，低于原 0.001rad/s 限值。
- 扣声明重力、摩擦后的 5Hz 力矩 RMS 为 **1.994–2.139mNm**。
- 两个新增实际 Mock 会话均正常结束，没有故障恢复或额外采集。

## 物理拟合与原 128 点结果

第一次对这 8 条 SDK 记录拟合完整刚体物理代表：调用已有 `fit_rigid`，冻结 nominal 参数尺度、完整单位二次正则度量、重力等式约束、声明摩擦与 ridge=1e-4。每条记录的平方行权重预算为 256，删组只移除行，不重新归一化其它记录。回归量与力矩继续使用已冻结的同回调行、同时间权重和同频带处理。

这次有 8 个完整／删组变体、7 次不同子集拟合；没有用验证结果选择 ridge、频带或时移。

| 删去的组 | 原 128 点最差对角惯量差 |
| --- | ---: |
| 构型 c00 | 1.140% |
| 构型 c01 | **1.568%** |
| 逻辑批次 b00 | **1.772%** |
| 逻辑批次 b01 | 1.673% |
| 实际会话 source_pair | 0.042% |
| 实际会话 b00_extension | **1.795%** |
| 实际会话 b01 | 1.673% |

表中是完整物理模型 `M(q)` 的删组差；前轮的 7.08% 是两条局部惯量诊断的正反差，二者口径不同。质量矩阵 Frobenius 差、逆动力学差也通过原限值，10% 对角项门槛与 1e-6kg·m² 分母下限未改。

完整模型与各条公开探针的局部惯量诊断最大差 **3.70%**，匹配力矩预测最大 RMSE **0.539mNm**，通过原探针 10%／2mNm 的数值条件。

## 独立复核与合成真值评分

新进程禁止打开 `evaluation`，从全部 8 条原始公开反馈重建问题并重新拟合，参数和回归问题最大差均为 **0**。另用 Pinocchio 原生批量接口独立计算原 128 点质量矩阵和逆动力学，评分量最大差也为 **0**。

拟合结束后，独立评分才读取预登记的合成惯量缩放，评估**右腕 pitch**：在原 128 点，候选对合成真值的最大惯量误差为 **2.013%**，nominal 先验为 **40.172%**。真值没有进入拟合、筛样本或选择模型。

数据回归秩为 9/140。当前结果支持右腕 pitch 的这套 SDK Mock 测量与训练稳定性，其它未激励轴仍依赖先验。完整 P0 R1/R2 的全部阶段、先验扰动和多 ridge 流程没有在这里执行；实机与正式采样资格仍为 false。

## 运行与结果入口

保留原两条会话，另建补采目录；该命令只启动 localhost 私有 SDK Mock，并且只发 6 条新增探针：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/supplement_sdk_wrist_pitch.py run \
  --source artifacts/sdk_wrist_pitch_c00_pair_v1 \
  --operator-probes artifacts/dvt2_004_wrist_sustained_development_v1/r2/operator_probe_set.npz \
  --output artifacts/sdk_wrist_pitch_campaign_new --domain-id 227
```

原 128 点是公开模型查询，不是正式 test 观测。本地产物缺失时，先恢复原评估点；原两条缺失时，从[两条入口](sdk_wrist_probe_pair.md)开始，不自动重造旧记录。

只读重分析和独立复核：

```bash
bash scripts/humble/container.sh python scripts/humble/supplement_sdk_wrist_pitch.py analyse \
  --source artifacts/sdk_wrist_pitch_campaign_v1 \
  --output artifacts/sdk_wrist_pitch_campaign_analysis_new

bash scripts/humble/container.sh python scripts/humble/verify_sdk_wrist_campaign.py \
  artifacts/sdk_wrist_pitch_campaign_v1 \
  --output artifacts/sdk_wrist_pitch_campaign_verification_new
```

- `protocol/experiment.json`：原两条绑定、6 条新增清单、分批／实际会话含义、固定拟合政策。
- `c*_right_pitch_*_b*/capture/`：6 条新增 SDK 原始 CDR、公开反馈及实际命令。
- `analysis/observations.json`：全部 8 条的实际速度、保持轴、频点及预处理检查。
- `analysis/report.json`：完整拟合、删构型／逻辑批次／实际会话比较。
- `analysis/campaign_stability.png`：实际 5Hz 信号和删组差图。
- `analysis/development_candidate/candidate.json`：本次右腕 pitch 开发对照候选，不自动部署。
- 独立复核目录的 `selected_axis_synthetic_evaluation.json`：后置合成真值评分。

## 第三姿态开发验证已完成

当前候选已冻结并用于[第三姿态正反两条独立开发验证](sdk_wrist_pitch_validation.md)：力矩预测 RMSE 为 0.339／0.269mNm，模型与公开局部惯量诊断差为 0.158%／0.197%，没有用新记录重拟合候选。随后[c00 右腕 roll 两条筛查](sdk_wrist_roll_screen.md)已完成，发现明显的相对时序敏感性；下一步先做离线观测设计，不直接扩大补采，也不再追加同配方 pitch 训练记录。该开发验证不占用原正式 test。
