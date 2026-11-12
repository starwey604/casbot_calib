# 第三姿态：冻结右腕 pitch 候选的独立开发验证

**第三姿态正反两条均通过。** 8 条训练得到的候选在采集前冻结，新数据只用于预测评分和局部惯量对照；没有更新候选参数、重拟合训练或选择频带／时移。

会话：`artifacts/sdk_wrist_pitch_validation_v1`；独立复核：`artifacts/sdk_wrist_pitch_validation_verification_v1`。缓存配置继续为 5ms，仅运行本机隔离 SDK Mock，原正式 test 未消费。

## 验证姿态与固定规则

第三姿态 v00 取两个训练中心的中点：左右腕 pitch／roll 为 **0／+0.225rad**。训练中心分别为 +0.20／+0.30rad、−0.20／+0.15rad；新姿态未参与训练，属于独立插值验证。

在这里采右腕 pitch 正反各一条，保持原 5Hz、0.004rad 正弦＋0.18rad/s 单向运动、2 秒测量窗。主评分保持 **30Hz、零 effort 相对偏移**；其它轴保持、路径检查、原 10% 惯量一致性和 2mNm 力矩 RMSE 条件不变。

采集前，训练候选及其摩擦、重力方向、模型绑定被复制并封存进 `protocol/candidate/`。在线仍只用 nominal 重力和原合成 PD 配置发送，不使用该候选的惯量前馈。

## 主评分结果

| 指标 | 负向 | 正向 |
| --- | ---: | ---: |
| 有符号速度范围（rad/s） | 0.05060–0.31636 | 0.05146–0.31397 |
| 其它 13 轴最大速度（rad/s） | 0.000490 | 0.000298 |
| 扣声明重力、摩擦后的 5Hz RMS（mNm） | 2.081 | 2.077 |
| 冻结模型力矩预测 RMSE（mNm） | **0.339** | **0.269** |
| 冻结模型平均刚体惯量（kg·m²） | 0.00073151 | 0.00073151 |
| 公开探针局部惯量诊断（kg·m²） | 0.00073036 | 0.00073007 |
| 模型与局部诊断的惯量差 | **0.158%** | **0.197%** |

速度、保持轴、预处理及实际反馈路径检查全部通过。局部惯量诊断只回归加速度、速度和慢漂移项，作为冻结候选的独立对照；它不回写候选的 140 个参数。

## 预登记敏感性结果

所有分析都使用同一批公开反馈和同一候选。只改变共同观测／回归频带，或对拟合后的 effort 信号施加相对时间偏移；没有按结果选择最优设置。

| 分析条件 | 两方向最差惯量一致性差 |
| --- | ---: |
| 主评分：30Hz、零偏移 | 0.197% |
| 共同频带 10Hz | 0.114% |
| 共同频带 20Hz | 0.131% |
| effort 相对 −5ms | **9.708%** |
| effort 相对 +5ms | 5.596% |

改变共同频带后，局部惯量结论保持稳定。相对时移 −5ms 已把差异推到接近原 10% 边界，说明字段间时序仍是重要敏感项。这是人为施加的压力测试，不是测出的 SDK 延迟；共享缓存配置 5ms 与字段相对错位分开登记。主方案继续保持零偏移，不据此调参。

## 复现与只读复核

运行入口先绑定已有训练候选，再创建隔离 Mock，仅采第三姿态两条：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/validate_sdk_wrist_pitch.py run \
  --source artifacts/sdk_wrist_pitch_campaign_v1 \
  --output artifacts/sdk_wrist_pitch_validation_new --domain-id 227
```

只读分析和独立复核：

```bash
bash scripts/humble/container.sh python scripts/humble/validate_sdk_wrist_pitch.py analyse \
  --source artifacts/sdk_wrist_pitch_validation_v1 \
  --output artifacts/sdk_wrist_pitch_validation_analysis_new

bash scripts/humble/container.sh python scripts/humble/verify_sdk_wrist_validation.py \
  artifacts/sdk_wrist_pitch_validation_v1 \
  --output artifacts/sdk_wrist_pitch_validation_verification_new
```

分析进程禁止打开训练会话目录及任何 `evaluation` 目录，并禁用刚体训练拟合入口。验证包已经包含完整候选快照和所需配置，分析不要求重新读取训练数据。

- `protocol/candidate/`：采集前冻结的候选快照。
- `protocol/experiment.json`：新姿态、开发验证角色、两条清单、主评分与敏感性网格。
- `v00_right_pitch_*/capture/`：原始 SDK CDR、公开反馈、命令及执行状态。
- `analysis/report.json`：主评分、质量检查与全部敏感性结果。
- `analysis/heldout_prediction.png`：公开力矩与冻结模型预测，使用共同均值显示并保留预测偏置。
- `delivery.json`：结果与下一步范围。

独立复核重新解析新记录，固定候选参数，重建回归问题并重算预测与局部惯量一致性；最大差均为 **0**。相关 14 项测试通过，包含零偏移兼容性、时移只改变力矩、转子项不重复计算、冻结快照不依赖原训练目录。

## 右腕 roll 两条筛查已完成

右腕 pitch 已有训练删组和第三姿态开发验证证据。随后完成的[c00 右腕 roll 正反两条筛查](sdk_wrist_roll_screen.md)显示：运动质量通过、局部惯量方向差 4.51%，但人为 ±5ms 力矩相对时移使估计最多变化 22.03%。下一步先离线比较惯量／相位分离方法，不直接扩大 roll 补采。右腕 pitch 候选的已验证范围仍是 pitch。

本轮没有扩大其它轴或进入控制对照。结果属于 SDK Mock 的独立开发验证，真实采样、实际设备力矩尺度及硬件资格仍按原协议核验。
