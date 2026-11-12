# 右腕 pitch 补充设计：两中心正 roll、正反非换向

**推荐在 `train_c00` 保持右 roll = +0.30 rad，在 `train_c01` 保持 +0.15 rad，各做右 pitch 正反探针。** 两批各重复一对，每中心 4 条，共 **8 条候选训练记录**。删任一中心时，把归属该中心的新增记录一起删；两中心设计不把新增信息押在同一组。

| 归属中心 | 右 pitch 中心角 | 正 roll 方案 | 等幅负 roll 对照 |
| --- | ---: | ---: | ---: |
| `train_c00` | −0.30 rad | **+0.30 rad** | −0.30 rad |
| `train_c01` | +0.30 rad | **+0.15 rad** | −0.15 rad |

其他轴保持各原中心角。正向、反向是两个独立的轨迹；每条的测量段只动右 pitch，在该段内不换向。沿用现有探针的 **5 Hz、0.004 rad、偏置速度 0.18 rad/s、测量 2 s**，测量段加速度 RMS 约 2.79 rad/s²。把进、退段各从 1 s 延至 1.5 s：原 1 s 草案超出了现有专用探针的 1 rad/s 速度包络。测量波形和门槛不改。

推荐与对照的 **8 条参考**（2 中心 × 2 个 roll 符号 × 2 个 pitch 方向）均通过完整进退的连续碰撞、关节限位和专用运动包络。单条 8.2 s；峰值速度上界 0.831 rad/s，测量段带符号速度连续下界 0.0542 rad/s，高于原 0.02 rad/s；最小采样间隙 11.206 mm，原连续碰撞门槛 10 mm。负 roll 对照另有 8 条记录预算，不默认计入推荐方案。

## 选正 roll 的理由是覆盖整条曲线，不是只修旧最坏点

在原 128 个名义算子评估点上，以右 pitch 对角惯量为目标，保留原重力约束、比较尺度、训练行权重。对两个中心各试 0.15、0.30、0.45、0.60 rad。候选参考沿用原 30 Hz 探针处理，只计右 pitch 力矩行，每条新增记录的权重相当于一个原单轴探针。比较指标是相对当前完整 train 的线性误差放大。

| 方案 | 新记录预算 | 任一删中心后的最差 | 删 c02 后的最差 | 删 c02、原最坏点 |
| --- | ---: | ---: | ---: | ---: |
| 不补 | 0 | 2.810× | 2.810× | 1.523× |
| 最佳单中心正 roll：c01 +0.45 | 4 | 1.479× | 1.475× | 1.475× |
| **两中心正 roll** | **8** | **1.455×** | **1.444×** | **1.444×** |
| 等预算负 roll 对照 | 8 | 2.553× | 2.553× | 1.442× |

如果只盯原最坏点，正、负 roll 两方案几乎一样；正 roll 的优势在于它补上了整个姿态范围里的薄弱位置。单中心四条是更省的试验，但删 c01 时新增数据会全部消失。按当前任务优先推荐两中心方案。原评估域包含冻结名义参考姿态；只读取 train 观测，不读取任何正式测试观测。

这是轨迹筛选证据，不是 R2 结果。若以后补采，应先核对实际速度符号、其它轴保持、目标频点及力矩平衡，再用原 10% 门槛做完整删组验证；不把线性倍数换算成实际惯量差。此轮没有采集、重拟合或更改阈值。

## 复现

```bash
base=artifacts/dvt2_004_wrist_supplement_local_v1
new=artifacts/dvt2_004_wrist_three_axis_integration_v1
bash scripts/humble/container.sh python scripts/humble/design_right_wrist_pitch.py \
  "$base/development" "$base/r2" "$new/right_pitch_information_v1/report.json" \
  "$new/right_pitch_probe_design_recheck"
```

本次 `right_pitch_probe_design_v1/report.json` 保存所有候选得分及删批次对照；`plans/` 保存推荐及对照的完整参考和连续门禁证明。

## 本机正 roll 补采：8/8 通过采集检查

已在 `artifacts/dvt2_004_wrist_three_axis_integration_v1/right_pitch_probe_capture_v1/` 补采两中心×两批×正反方向共 8 条 train 记录，未采负 roll 对照。所有实际观测路径通过连续碰撞门禁，最小采样间隙约 11.206 mm；每条测量段有 8,000 个原始样本（4 kHz）。

| 核对项 | 八条记录的结果 |
| --- | --- |
| 右 pitch 在测量段不换向 | 最小带符号速度 **0.05414 rad/s**，高于原 0.02 rad/s |
| 其余 13 轴保持 | 观测速度均为 0；最大位置波动 **0.85 µrad**，相对参考的最大偏差 **0.169 mrad** |
| 实际右 roll | c00 均值约 **+0.29989 rad**，c01 约 **+0.14987 rad**；最大位置偏差 **0.129 mrad** |
| 右 pitch 5 Hz 力矩信号 | **3.395–3.413 mNm RMS**，高于 0.1 mNm 的信号检查下限 |

力矩数值为测量段中去除二次慢趋势后的合成执行器总力矩，不是孤立惯量力矩。采集协议绑定设计、原 train 和冻结前序；`train_capture_manifest.json` 封存八条新增记录，`report.json` 列逐条核对结果。未重拟合、未改 10% 门槛，正式测试观测未读取或采集。

```bash
base=artifacts/dvt2_004_wrist_supplement_local_v1
new=artifacts/dvt2_004_wrist_three_axis_integration_v1
bash scripts/humble/container.sh python scripts/humble/collect_right_wrist_pitch.py \
  "$base/development" --design "$new/right_pitch_probe_design_v1" \
  --output "$new/right_pitch_probe_capture_recheck"
```

## 接入开发拟合

新会话为 `artifacts/dvt2_004_right_pitch_integration_v1/development/`。原 44 条 train 和 24 条 selection 继续使用原采集身份、角色和权重；四条左腕分频记录继续使用原 9,604 个有效时刻和等记录权重，固定同核高斯 σ=1.5 ms（标称 88 Hz），不按本轮结果改带宽。八条右 pitch 使用原非换向探针的 30 Hz 处理，每条给一个原单轴探针预算，不重新归一化原训练。删组时新增行随原中心、批次一起删除。

```bash
old=artifacts/dvt2_004_wrist_three_axis_integration_v1
left=artifacts/dvt2_004_wrist_supplement_local_v1/three_axis_capture_v1
out=artifacts/my_right_pitch_integration
bash scripts/humble/container.sh python scripts/humble/integrate_right_wrist_pitch.py \
  "$old/development" --left-capture "$left" \
  --right-capture "$old/right_pitch_probe_capture_v1" \
  --left-support "$old/sustained_refit_v1/raw_sample_indices.npz" --output "$out/development"
```

原始 R1 拟合拒绝，保留在 `development/identification_development/report.json`。八条右 pitch 的力矩 RMSE 为 0.546–0.632 mNm（低于原 2 mNm），但等效惯量相对各自单轴力矩平衡估计偏高 **38.47%–44.50%**（门槛 10%）。用同一估计器对候选自身预测力矩求惯量，重现候选矩阵值（差小于 $4\times10^{-13}$ kg·m²）。`r1_verified/verification.json` 复核了 80 条开发参考和观测路径、候选与合成 plant 各 14 条积分步长减半；禁止读取真值的新进程重现原拒绝，算子差为 0。标准 R1 验收和 R2 资格入口继续拒绝这个会话；后续删组仅作诊断。

### R2 删组诊断：删 `train_c02` 的右腕 pitch 恶化到约 31%

原 128 个评估点的 `q/dq/ddq` 逐元素相同。下表为删组模型相对于同档完整训练模型的最大惯量对角项差：

| 删组 | 最差轴 | ridge `1e-5` | ridge `1e-4` | ridge `1e-3` |
| --- | --- | ---: | ---: | ---: |
| `train_c00` | 右腕 yaw | 13.124% | 13.118% | 13.126% |
| `train_c01` | 右腕 pitch | 12.049% | 12.050% | 12.035% |
| `train_c02` | 右腕 pitch | 31.097% | 31.039% | 30.715% |
| `train_batch_00` | 右腕 yaw | 0.142% | 0.141% | 0.147% |
| `train_batch_01` | 右腕 yaw | 0.141% | 0.141% | 0.136% |

与同档、同左腕处理但未加入右 pitch 数据的 **10.120%** 相比，删 `train_c02` 的中档结果 **31.039%** 恶化了 **20.919 个百分点**。三个删构型均超过 10%；两种删批次通过算子比较，但全训练和删批次模型仍未通过腕部训练预测检查。`r2/report.json` 保存全部 30 个变体、33 次拟合；`r2_verified/verification.json` 对 30 个候选独立重算的 Pinocchio 算子与保存结果差为 0，无合格正则化档。未补采负 roll，未改门槛，未读取、采集或改动正式测试观测。

## 同带宽复核：先解决正 roll 下的约束冲突

**回归恶化来自新增探针的实测力矩目标与原训练约束的折中，不是两类记录用了不同的滤波顺序。** 固定中档正则化、全部旧权重、重力等式和原 128 点，逐项替换观测处理后重拟合：

| 对照 | 删 `train_c02` 后右腕 pitch 最大惯量差 |
| --- | ---: |
| 未加入右 pitch 探针 | 10.120% |
| 加入实测探针（原处理） | 31.039% |
| 仅探针改为先构造原始回归量、再与力矩一起做 30 Hz 处理 | 31.039% |
| 再把 c02 周期的右 pitch 行改为先构造原始回归量、再投影到原 40 阶频带 | 31.042% |
| 保留探针原回归行和权重，仅用补采前完整模型预测替换探针力矩 | 8.835% |

最后一行用模型预测替换观测，仅用于区分「新增姿态提供的约束」和「新力矩要求模型改变的方向」。它不能用于训练交付。原 c02 力矩目标的重新投影也复现了冻结目标，差不超过 $10^{-11}$ Nm。

用同一批原始时刻，在标称 8、30、88 Hz 的零相位高斯带宽下重算力矩平衡。新增单轴探针在 +0.30、+0.15 rad 分别给出 **约 0.486、0.494 ×10⁻³ kg·m²**，跨带宽和两批保持稳定。c02 同一 roll 范围内，两种周期波形给出的局部等效值明显不同，且需要候选模型扣除其它轴的惯性耦合。+0.30 rad 还可将右 pitch 限在共同的 −0.317 至 −0.171 rad：在 30 Hz 下，探针的有效 pitch 加速度约 **2.73 rad/s²**，c02 的两种波形只有 **0.27、0.07 rad/s²**，不足以将其局部估计当作同精度的单轴对照。+0.15 rad 没有共同的 pitch 角区间。

从实测删 c02 的拟合中逐块恢复数据，可定位哪部分把模型拉回完整训练解。下表比较各诊断模型与原完整训练模型在同 128 点上的右 pitch 惯量；各行均从同一删 c02 输入独立拟合。

| 恢复哪些 c02 观测行 | 与完整训练的最大右 pitch 惯量差 |
| --- | ---: |
| 不恢复 | 31.039% |
| 只恢复周期记录的右 pitch 力矩 | **8.335%** |
| 只恢复右腕 yaw 单轴探针 | 16.519% |
| 恢复除周期右 pitch 以外的所有行 | 16.614% |
| 不恢复上述两种腕部力矩行，只恢复其他行 | 28.849% |

本次最差点在 **正 roll +0.598 rad**，原完整／删 c02 模型分别给出 **0.608／0.419 ×10⁻³ kg·m²**。建议暂不采负 roll：它不能直接区分这次最差点的正 roll 约束。如果要做下一轮对照，优先离线设计在 c02 同一正 roll 姿态、相近 pitch 区间和加速度下，对照单轴与多轴运动，再决定是否采集。

结果位于 `artifacts/dvt2_004_right_pitch_integration_v1/`：`balance_diagnosis_v1/report.json` 为处理与目标替换对照，`balance_pose_overlap_v1/balance.json` 为逐记录、带宽和姿态范围结果，`balance_restorations_v1/restorations.json` 为逐块恢复。复现：

```bash
d=artifacts/dvt2_004_right_pitch_integration_v1
before=artifacts/dvt2_004_wrist_three_axis_integration_v1/local_bandwidth_v1/sigma_1.5_ms
bash scripts/humble/container.sh python scripts/humble/diagnose_right_pitch_balance.py \
  "$d/development" "$d/r2" "$before" "$d/balance_recheck"
bash scripts/humble/container.sh python scripts/humble/diagnose_right_pitch_balance.py \
  "$d/development" "$d/r2" "$before" "$d/restorations_recheck" --restorations-only
```

本轮只读取训练观测，未补采、未改门槛，R1/R2 资格保持拒绝。

## 同姿态对照：右腕 pitch 单轴与加右肩 pitch

在最差点的 **右腕 roll +0.59777 rad** 保持不动。两批 `train_c02_01` 都实际经过距该查询姿态约 0.000213 rad（14 轴 L2 距离）的位置；在 roll ±0.025 rad 的邻域内，实测右腕 pitch 共同覆盖 **−0.15705 至 −0.08357 rad**。将 pitch 扫描中心设在 −0.12031 rad，其他基准轴采用最差查询姿态。A、B 的 pitch 参考曲线逐系数相同；唯一改变的是右肩 pitch 是否运动。

| 参考 | 0.50 s 测量段 | 位置与加速度 |
| --- | --- | --- |
| A：单轴 | 右腕 pitch 4 Hz，叠加 0.12 rad/s 的带符号扫描速度 | pitch −0.15031 至 −0.09031 rad；加速度 1.563 rad/s² RMS，接近旧周期在同一 roll 邻域的 1.773 rad/s² |
| B：双轴 | 同一腕部波形，叠加右肩 pitch 6 Hz，叠加轴扫描速度 0.065 rad/s | 右肩 pitch 扫描 0.0325 rad；两条参考都不换向 |

选择右肩 pitch，不选力矩预测幅值更大的肘：右肩 pitch 在最相近的两条旧周期里，对右腕 pitch 的交叉惯性力矩最大；保守取三个模型中的最小估计为 **0.704 mNm RMS**，而右肘为 **0.310 mNm**。新参考中，正反向的 6 Hz 肩腕对照预计产生 **0.600–0.690 mNm RMS** 的右腕动力学差，高于已有的 0.1 mNm 信号下限。

四条完整参考均通过关节限位和原 10 mm 连续碰撞检查（最小采样间隙 11.141 mm）。速度、加速度、jerk 连续上界分别为 0.654 rad/s、2.218 rad/s²、56.297 rad/s³，低于 1／8／60 的既有较严包络；测量段腕、肩带符号速度下界分别为 0.0320、0.0253 rad/s，均高于 0.02。建议之后在本机分两批做 **8 条诊断记录**，每批覆盖 A/B × 正/反，第二批交换同向的 A/B 顺序。暂不补负 roll，不把这些记录预分配为拟合训练行。

### 采到以后怎么看

1. 先确认实际 A/B 的腕部位置、速度和加速度可配对，roll 保持、附加轴不换向、其余轴保持。核对原始观测时间轴、实际路径和两频点力矩后再归因。
2. 在原时间轴上先算惯性、重力、速度耦合和力矩，再统一做 30 Hz 零相位处理。用 **完整 0.50 s 测量窗** 联合估计 4 Hz、6 Hz 的两相分量和二次慢漂，保持 4 Hz 速度与加速度相位可分。不要套用旧探针两端各剪 0.20 s 的评分；它会把这个短窗缩成 0.10 s。
3. 先用 A 的 **4 Hz** 惯性同相力矩区分完整／删 c02 模型的自惯量预测，再用同方向 **B−A 的 6 Hz**（扣除重力变化）检查肩腕耦合。已从保存的四条参考回读验证：完整／删 c02 模型的 4 Hz 自惯性力矩约 **0.951／0.656 mNm RMS**，6 Hz 耦合对照约 **0.690／0.600 mNm RMS**；预测 B−A 串到 4 Hz 的动力学分量低于 **0.006 mNm**。若实测只有 B 出现重复的额外残差，先查耦合；若 A/B 在 4 Hz 同向偏离，先查自惯量。未获得可重复且可配对的两类信号就保留未决，不按一次残差更改 R1/R2。

参考、连续证明、候选轴排序和短窗读数保存在 `artifacts/dvt2_004_right_pitch_integration_v1/motion_pair_design_v1/`。复现时用新输出目录：

```bash
d=artifacts/dvt2_004_right_pitch_integration_v1
out="$d/motion_pair_design_recheck"
bash scripts/humble/container.sh python scripts/humble/design_right_pitch_motion_pair.py \
  "$d/development" "$d/r2" "$d/balance_restorations_v1/restorations.json" "$out"
bash scripts/humble/container.sh python scripts/humble/design_right_pitch_motion_pair.py \
  "$d/development" "$d/r2" "$d/balance_restorations_v1/restorations.json" "$out" --verify-only
```

本步只生成离线参考，没有采集或重拟合，原 10% 门槛和正式测试数据不变。

## 两批 A/B 实测：完整模型在单轴下也高估右腕惯量

本机原生 Mock 按交叉顺序采完 8 条诊断记录，四对 A/B 均通过实际路径与信号核对。每条完整短窗有 2,000 个连续原始样本。实际最小采样间隙 **11.141 mm**；腕、肩最小带符号速度分别为 **0.03188、0.02482 rad/s**；保持轴测得速度为 0。配对腕部位置最大差 **2.05 µrad**，同带宽腕部加速度差 **0.104% RMS**。实际 roll 均值为 +0.59766 rad，单条波动小于 0.74 µrad。

| 短窗诊断 | 八条／四对的观测 | 完整训练预测 | 删 `train_c02` 预测 |
| --- | ---: | ---: | ---: |
| 4 Hz 腕部等效自惯量（×10⁻³ kg·m²） | 0.443–0.453 | 0.608 | 0.419 |
| 4 Hz 自惯性同相力矩（mNm RMS） | 0.693–0.709 | 0.951 | 0.656 |
| 6 Hz 肩腕耦合 B−A（mNm RMS） | 0.610–0.638 | 0.700 | 0.609 |

4 Hz 等效惯量用实测加速度定相，扣除冻结摩擦及各模型的重力、其它轴惯性与速度项。用位置谐波的二阶解析导数代替速度差分，4 Hz 加速度幅值最大变化约 0.00041%，惯量范围不变。同方向 A/B 的惯量差不超过 **0.0065 ×10⁻³ kg·m²**，没有随加肩运动一致变大的趋势。6 Hz 用 A/B 各自的实际轨迹扣除重力、摩擦和自惯性，再比较耦合项的两相残差：完整模型 **0.062–0.090 mNm RMS**，删 c02 模型 **0.002–0.029 mNm RMS**，四对结论一致。

因此在这个正 roll 姿态，完整模型的自惯量偏高并不依赖肩轴运动；删 c02 模型在两项读数上都更近，但其自惯量仍偏低。这一轮优先支持检查原 `train_c02` 周期力矩行如何把联合拟合推高，不支持直接删掉整个中心或优先补负 roll。保留这八条作独立诊断，不纳入训练；未重拟合，R1/R2 门槛和正式测试保持不变。

`artifacts/dvt2_004_right_pitch_integration_v1/motion_pair_capture_v1/` 保留采集顺序、原始观测和逐条路径；`motion_pair_analysis_v1/` 保存报告和位置谐波交叉检查。分析在新进程阻止读取真值和原训练／正式测试观测，可用同一采集复算：

```bash
d=artifacts/dvt2_004_right_pitch_integration_v1
bash scripts/humble/container.sh python scripts/humble/analyze_right_pitch_motion_pair.py \
  "$d/motion_pair_capture_v1" "$d/r2" "$d/motion_pair_analysis_recheck"
```

## c02 周期定位：六个半秒窗口足以把惯量推高

从只移除 `train_c02` 四条周期记录的右腕 pitch 力矩行的诊断解出发，其余所有训练行仍在。目标为原第 49 号查询（零基索引，roll +0.59777 rad）的右腕 pitch 对角惯量。这个起点是 **0.507 ×10⁻³ kg·m²**，恢复全部 4,014 行后为 **0.608 ×10⁻³ kg·m²**。将两种 6 s 波形各切成 12 个半秒相位窗，以冻结重力约束、行权重和中档正则化下的局部目标响应排序；同一窗口合并两批，排序不是按力矩残差大小。原轨迹进入周期后丢弃两周期，表中时间从每个剩余周期的零相位起算。

| 波形 | 最推高惯量的相位窗（s） |
| --- | --- |
| `train_c02_00` | 0.5–1.0、1.0–1.5 |
| `train_c02_01` | 0.5–1.0、1.5–2.0、4.0–4.5、5.5–6.0 |

用同一个物理约束求解器、原权重做诊断性恢复：只加这六窗的 **990 行**，惯量升到 **0.634 ×10⁻³ kg·m²**；只加其余 **3,024 行**，则仍为 **0.506 ×10⁻³ kg·m²**。只恢复 roll ≥+0.55 rad 的观测行仅复现约 **10%** 的上升；只恢复低于该角的行却复现约 **94%**。两种波形都能推高，最差查询姿态并不是主要驱动行的唯一位置。

按原 40 阶带宽将模型的力矩改变量拆开，计算它与基准残差同向的加权分量。四条 c02 周期的净推动中，**腕 pitch 自惯性约占 59%，腕 yaw 加速度的交叉惯性约占 35%**；前六窗内肘部交叉惯性也明显，一些肩部项反向抵消。重力与速度耦合的总净贡献接近零，拟合摩擦项改变量约占 1.6%。这是模型用哪些项吸收残差的分解，不等于残差本身来自这些项。将力矩与模型项统一做 30 Hz 处理，在原时间轴逐批逐周期重算，前六窗约 **87%** 的净同向推动落在非持续运动时刻附近。

### 原始时间轴确认：不到 2% 的停滞样本是主要来源

在两段稳态原始周期内，用 `|dq_pitch| ≤ 1e-5 rad/s` 标记停滞；中心及前后 5 ms 均同向且高于 0.02 rad/s 才计作持续运动。其余为过渡。四条记录合计 192,000 个时刻，停滞 **3,792 个（1.98%）**，单条最长停滞 16.75–26.75 ms。此时其它轴仍在动，不能把这些点当作全机静止重力观测。扣除基准模型刚体力矩后，停滞时的反力绝对值第 95 百分位 **68.1–68.4 mNm**，低于冻结静摩擦约 **70.0 mNm**；约 98.8%–98.9% 的点在该冻结区间内。停滞时模型残差约 **52 mNm RMS**，持续运动时约 **0.73–0.83 mNm**。

先在原时间轴按状态拆分残差，再用原 40 阶投影。对照只把指定残差从训练目标中扣除，保持所有采样行、回归量、权重和物理约束不变：

| 目标替换对照 | 最差查询点惯量（×10⁻³ kg·m²） |
| --- | ---: |
| 不替换（原完整模型） | 0.608 |
| 只扣停滞时的残差 | **0.450** |
| 扣停滞及邻近过渡的残差 | **0.453** |
| 只扣持续运动时的残差 | 0.662 |

原始停滞反力先经过全周期投影，因而会进入投影后仍被判为“运动”的行。这解释了为什么短暂停滞可以推动联合模型增大自身惯量和交叉惯性，也说明不应直接删除六个半秒窗口：应在原始时间轴区分停滞和持续运动，再对有效段中的力矩与回归量做同带宽处理。下一步建议把这条处理作为单独开发对照，先复核覆盖和原 128 点；不要把本轮目标替换当作可交付拟合。

本轮未读取八条 A/B 观测，不补采，不改 10% 门槛，不读取或改动正式测试数据。逐窗及目标替换结果保存在 `c02_phase_diagnosis_v1/report.json` 与 `c02_transition_diagnosis_v1/report.json`（均位于本开发运行目录）。复现：

```bash
d=artifacts/dvt2_004_right_pitch_integration_v1
bash scripts/humble/container.sh python scripts/humble/diagnose_c02_pitch_phase.py \
  "$d/development" "$d/r2" "$d/balance_restorations_v1" "$d/c02_phase_recheck"
bash scripts/humble/container.sh python scripts/humble/diagnose_c02_pitch_phase.py \
  "$d/development" "$d/r2" "$d/balance_restorations_v1" "$d/c02_transition_recheck" --transitions
```

## 持续运动验证：右 pitch 删 c02 改善，左 yaw 删 c00 仍超界

把处理作为独立的 train 离线对照，不改原协议或已有候选。先测试只替换 c02 的周期右 pitch 行，再统一处理三个训练中心的 **12 条周期记录**。其它 245,338 行，包括左腕局部处理和单轴探针，数值及行权重不变。

原始时间轴上，pitch 速度中心及前后 5 ms 均同向且大于 0.02 rad/s 才保留；两稳态周期共 576,000 个原始时刻，有 537,516 个满足条件，每 15 点取一个中心，最终保留 **35,840 行**。先用原始 q、dq 和五点速度导数计算 Pinocchio 回归量、摩擦特征和力矩目标，再统一施加 **σ=1.5 ms** 的局部对称高斯核。核不跨无效段，在边缘对称缩短；所有训练中心、批次使用同一规则。旧 11,864 个周期右 pitch 行被替换，每条记录的归一化尺度和总平方权重预算不变，不增加该轴的记录预算。roll、pitch 覆盖端点与原周期相差小于 0.36 mrad。

只处理 c02 时，最差原查询点的完整模型惯量从 0.608 降到 **0.450 ×10⁻³ kg·m²**，但八条已入训练的右 pitch 探针最大惯量不一致仍有 **22.31%**。对所有中心统一处理后，该点为 **0.458 ×10⁻³ kg·m²**，右 pitch 探针最大不一致降到 **0.970%**；所有原单轴探针均通过原 10% 惯量／2 mNm 力矩检查，最大惯量不一致 **4.454%**。原周期／左腕的现有预测检查也均通过。A/B 诊断记录未用于拟合或这些检查。

中档 ridge `1e-4`，在原 128 个点上的删组结果：

| 删组 | 右 pitch 最大惯量差 | 全轴最大惯量差 | 最差轴 |
| --- | ---: | ---: | --- |
| `train_c00` | 6.071% | **11.216%** | 左腕 yaw |
| `train_c01` | 2.669% | 8.688% | 右腕 roll |
| `train_c02` | **5.278%** | 7.908% | 左腕 yaw |
| `train_batch_00` | 0.120% | 0.120% | 右腕 pitch |
| `train_batch_01` | 0.120% | 0.120% | 右腕 pitch |

另外固定检查两侧原 ridge，删 c02 的右 pitch 最大惯量差分别为 **5.399%（1e-5）、5.091%（1e-3）**，改善不依赖只选中档。两侧 ridge 的其他删组尚未重跑，本步不是完整 R2 资格复核。

独立进程从原始观测重新检查支持时刻，以逐行显式高斯核对原生候选总力矩计算 RMSE，复现所有 12 条持续运动评分；11 个保存解在原 128 点的原生算子重算差为 0。结果为 `right_pitch_sustained_v1/report.json`，独立核验为该目录的 `verification.json`。

**结论：持续运动处理解决了右 pitch 这次删 c02 的主要不稳定，但仍不能宣称 R2 通过。** 下一步应在独立开发协议里接入统一处理，并复核完整 R1/R2，同时定位剩余的左腕 yaw 删 c00 的 11.216%；不要调整门槛或直接升格当前诊断解。原记录和正式测试未改，A/B 记录仍独立留存。

```bash
d=artifacts/dvt2_004_right_pitch_integration_v1
out="$d/right_pitch_sustained_recheck"
bash scripts/humble/container.sh python scripts/humble/verify_right_pitch_sustained.py \
  "$d/development" "$d/r2" "$out"
bash scripts/humble/container.sh python scripts/humble/verify_right_pitch_sustained.py \
  "$d/development" "$d/r2" "$out" --verify-only
```

## 新开发协议接入

新会话为 `artifacts/dvt2_004_sustained_pitch_development_v1/development/`。原 44 条主会话 train、24 条 selection 及左右腕 companion 继续绑定原采集身份；左腕有效样本和处理不变。新增 `pitch_sustained_train` 只控制训练周期的右 pitch 行，`right_pitch_sample_indices.npz` 与协议一起封存，逐条重算支持规则，不从结果挑样本或带宽。

原周期预测、原始静止区间、前序与单轴探针门槛均保留；训练右 pitch 增加持续运动评分，必须同时通过原 5% NRMSE 门槛。selection 保留原处理，不因新训练规则重选窗口。R1 重新拟合，而非沿用上一轮诊断解；R2 仍采用原三档 ridge、固定比较尺度、原 128 点和原删组规则。

```bash
old=artifacts/dvt2_004_right_pitch_integration_v1
new=artifacts/my_sustained_pitch_development
bash scripts/humble/container.sh python scripts/humble/integrate_right_pitch_sustained.py \
  "$old/development" "$old/right_pitch_sustained_v1/raw_sample_indices.npz" "$new/development"
bash scripts/humble/container.sh casbot-calib staged-verify \
  "$new/development" --output "$new/r1_verified"
bash scripts/humble/container.sh casbot-calib staged-identify \
  "$new/development" --through stability --output "$new/r2"
bash scripts/humble/container.sh casbot-calib staged-verify \
  "$new/development" --stability-analysis "$new/r2" --output "$new/r2_verified"
```

### 新协议完整 R2：仅删 c00 的左腕 yaw 未通过

R1 五项检查重新拟合后全部通过。`r1_verified_v2/verification.json` 独立复核 80 条开发路径、候选与合成 plant 各 14 条步长减半；禁止真值访问的新进程重分析算子差为 0，复核通过。R2 完成原 **30 个变体／33 次拟合**：每档均有 9 个变体通过，只有删 `train_c00` 的变体被原 10% 惯量差门槛拒绝；删其余构型、删批次、完整训练及四个先验扰动均通过。三档对照为：

| ridge | 删 c00 的全轴最大差（左 yaw） | 删 c02 的右 pitch 最大差 |
| --- | ---: | ---: |
| `1e-5` | 11.219% | 5.399% |
| `1e-4` | 11.216% | 5.278% |
| `1e-3` | 11.189% | 5.091% |

独立 R2 验证重建全部 30 个候选的 Pinocchio 算子，差为 0，原 128 点逐元素相同，合格 ridge 为空。未启动 R3/R4。

### 剩余左 yaw 的主信息来自 c00 的左 pitch 力矩

中档最差为原查询点 19（零基索引），左 pitch／roll 约 **+0.532／−0.358 rad**。完整与删 c00 模型的左 yaw 惯量分别 **1.1118／0.9871 ×10⁻³ kg·m²**。按相同约束与权重对 c00 的阶段／力矩轴分块，并实际恢复拟合：

| 从删 c00 训练中恢复哪些行 | 该点左 yaw 惯量（×10⁻³ kg·m²） |
| --- | ---: |
| 不恢复 | 0.9871 |
| 只恢复四条周期记录的**左 pitch 力矩**，3,952 行 | **1.1169** |
| 恢复 c00 其它全部行，不含周期左 pitch | 0.9868 |
| 恢复全部 c00 | 1.1118 |

所以当前剩余敏感性首先应查的是 **c00 原周期的左 pitch 力矩观测**，不是继续补左 yaw 单轴探针，也不是改四条左腕补采的处理规则。原时间轴的初查也观察到左 pitch 短暂停滞与约 52 mNm 残差，但这些停滞点对当前完整／删 c00 改变量的原始同向推动只约占 13%，还不能套用右 pitch 的停滞主因结论。下一步应区分这些左 pitch 行提供的真实跨轴信息与停滞／投影偏差，再决定是否改变其观测处理。

上述恢复只是定位，不能把恢复后的单点接近完整解记作 R2 资格。78 项相关回归通过。总交付索引为新开发目录的 `delivery.json`，逐块结果为 `left_yaw_diagnosis_v1/report.json`，可复现：

```bash
d=artifacts/dvt2_004_sustained_pitch_development_v1
bash scripts/humble/container.sh python scripts/humble/diagnose_left_yaw_c00.py \
  "$d/development" "$d/r2" "$d/left_yaw_diagnosis_recheck"
```

## 继续到原 10% 门槛：统一原周期腕部观测处理

### 左 pitch 的反事实区分了停滞偏差和真实运动信息

在旧训练上，保持所有行、回归量、权重和约束不变，只在全周期投影前扣除 c00 左 pitch 停滞残差，原查询点 19 的左 yaw 惯量从 **1.1118 降至 0.9755 ×10⁻³ kg·m²**；只扣持续运动残差仍为 **1.1112**。直接采用同带宽持续运动处理也给出 **0.9766**。因此先前约 13% 的原始力矩方向压力不足以判断最终惯量偏移，实际目标替换支持停滞／投影偏差是该点的主要影响来源。

仅统一左 pitch 后，删 c00 超界消失，但删 c02 的左 pitch 最大差变为 **12.165%**，该负结果保存在旧开发目录的 `left_pitch_sustained_diagnostic_v1/`。没有据此选择性删除 c02 或改变四条左腕补采。下一项对照对原主周期的六个腕部力矩通道 **4、5、6、11、12、13** 使用同一固定规则：原始速度持续同向并高于 0.02 rad/s、前后 5 ms 保护、stride 15、同核 σ=1.5 ms。各轴分别用自己的有效时刻，不要求六轴同时运动；三中心和两批均同规则。位置、惯性和摩擦的回归列先由原始信号构造，再与唯一力矩观测通道同核处理。

每个原记录／轴的平方权重预算、归一化尺度以及原 yaw 降权都保留。其它八轴、原单轴探针、四条左腕分频补采及其处理、八条已入训练的右腕 pitch 探针均不变；独立八条 A/B 不读入，selection 沿原规则评分，正式 test 不消费。原全周期预测和原始静止／前序门槛仍须通过，新训练观测还增加原 5% NRMSE 限值下的逐通道评分。

六通道对照的中档删组最大惯量差均低于 10%，因此另建 `artifacts/dvt2_004_wrist_sustained_development_v1/development/` 接入，并重新执行完整 R1、原三档完整 R2 和独立复核；不把诊断解直接升级。各原始样本索引随新协议封存在 `wrist_primary_sample_indices.npz`，逐条重新验证；继承的右 pitch 样本必须完全相同。

新协议 R1 保留 56 条 train、24 条 selection：五项检查通过，原主运动／左腕补采最大 NRMSE **3.0621%**，所有原单轴探针最大惯量不一致 **0.4559%**、力矩 RMSE **0.0645 mNm**；新增持续运动评分最大 NRMSE **0.0475%**。原 5%、10%、2 mNm 和静止／前序门槛均未修改。

中档 R2 对原 128 点的删组最大惯量差：

| 删组 | 最大差 | 最差轴 |
| --- | ---: | --- |
| `train_c00` | 1.2025% | 右腕 yaw |
| `train_c01` | 0.2903% | 左腕 yaw |
| `train_c02` | 0.2706% | 左肩 yaw |
| `train_batch_00` | 0.3327% | 右腕 yaw |
| `train_batch_01` | 0.3325% | 右腕 yaw |

新协议及独立复核的复现入口：

```bash
old=artifacts/dvt2_004_sustained_pitch_development_v1
new=artifacts/my_wrist_sustained_development
bash scripts/humble/container.sh python scripts/humble/integrate_wrist_primary_sustained.py \
  "$old/development" "$old/wrist_primary_sustained_diagnostic_v1/raw_sample_indices.npz" "$new/development"
bash scripts/humble/container.sh casbot-calib staged-verify \
  "$new/development" --output "$new/r1_verified"
bash scripts/humble/container.sh casbot-calib staged-identify \
  "$new/development" --through stability --output "$new/r2"
bash scripts/humble/container.sh casbot-calib staged-verify \
  "$new/development" --stability-analysis "$new/r2" --output "$new/r2_verified"
bash scripts/humble/container.sh python scripts/humble/verify_wrist_raw_force.py \
  "$new/development" "$new/r2" "$new/raw_force_verification.json"
```

### 原 10% 门槛目标已完成（本机开发 R1/R2）

三档完整 R2 的 **30／30 变体、33 次拟合全部通过**，合格 ridge 为 `1e-5、1e-4、1e-3`。每档在原 128 点、所有删构型／删批次／先验变体中的最大惯量差分别为 **1.2014%、1.2025%、1.2128%**，均低于原 10%。正定性、质量矩阵范数、逆动力学、秩、训练预测和前序保护等其余原检查也全部通过。

`r1_verified/verification.json` 复核 80 条参考及实际路径、候选与合成 plant 各 14 条积分步长减半，禁止真值访问的新进程重分析算子差为 **0**。`r2_verified/verification.json` 对全部 30 个候选独立重算，原生算子差为 **0**。`raw_force_verification.json` 对 **12 条原周期 × 6 腕轴 = 72 个通道**，从原始速度重新判定有效支持，以逐点显式高斯核计算原生总力矩，和回归量装配结果的最大差为 **2.03×10⁻¹⁶ Nm**。

原 128 点、逐记录／轴的归一化尺度与平方权重预算、原 yaw 降权、已有左右 companion 绑定和右 pitch 支持均核对一致；81 项相关回归通过。本轮没有补采，原会话及失败记录保留，八条 A/B 仍独立留存，正式测试未消费。新结果只获得本机合成开发 R1/R2 资格，下一阶段为 R3 归因／selection 选参；尚未冻结最终候选或启动 R4／实机标定。

最终结果索引：`artifacts/dvt2_004_wrist_sustained_development_v1/delivery.json`。

2026-11-12 已继续完成[该开发协议的 R3 归因与独立选参复核](staged_dynamics_p0_r3.md#2026-11-12dvt2-持续运动开发协议的-r3)，选择 `1e-3`；正式候选冻结需先登记独立最终参考，尚未启动 R4。R1/R2 索引保持原阶段快照，R3 结果另存 `r3_delivery.json`。
