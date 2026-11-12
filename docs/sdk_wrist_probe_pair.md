# 5ms SDK Mock：c00 右腕 pitch 正反探针

**两条探针已完成，支持小范围扩大同轴实验。** 实际速度全程保持有符号运动，其余 13 个双臂关节保持通过，5Hz 力矩分量清晰。建议先补齐右腕 pitch 的第二构型与重复批次，再考虑其它腕轴。

本轮会话：`artifacts/sdk_wrist_pitch_c00_pair_v1`；独立复核：`artifacts/sdk_wrist_pitch_c00_pair_verification_v1`。没有连接实机、修改原 10% 门槛或消费正式测试数据。

## 本次实际采了什么

沿用[离线探针配置](../configs/sdk_wrist_probe.yaml)：5Hz、0.004rad 正弦叠加 0.18rad/s 单向速度，2 秒测量窗，两端各舍弃 0.2 秒。c00 两侧腕 pitch／roll 为 +0.20／+0.30rad，仅右腕 pitch 运动，其余双臂关节保持。

实际运行了 `SDKMockSession`、原 SDK 发送器和原始 CDR 记录器，不是理想参考读出夹具。缓存配置为 5ms，公开反馈发布目标 1000Hz，客户端命令 500Hz；继续使用同一名义机械模型、声明的合成摩擦／惯量夹具与 Kp 下限 30。惯量采集只发送 nominal 重力前馈，不发送摩擦或识别出的惯量前馈。

每条探针从新的只读起姿快照生成完整进场、测量、制动与回位，重新检查实际起点对应的连续路径和指令折线。两条均正常完成，发送跳槽数均为 0；Mock 进程正常退出。

## 速度、保持与 5Hz 力矩结果

速度与保持上限统计完整 2 秒测量窗；力矩频点使用共同的 1.6 秒保留窗。

| 实际 SDK 反馈指标 | 负向 | 正向 |
| --- | ---: | ---: |
| 有符号速度范围（rad/s） | 0.05104–0.31101 | 0.05177–0.30976 |
| 有符号平均速度（rad/s） | 0.17998 | 0.17993 |
| 其余 13 轴最大速度（rad/s） | 0.000268 | 0.000471 |
| 其余轴最大位置跨度（rad） | 0.0000248 | 0.0000446 |
| 测量窗最大接收间隔（ms） | 1.172 | 1.132 |
| 估算 effort 的 5Hz RMS（mNm） | 2.168 | 2.345 |
| 扣声明重力、摩擦后的 5Hz RMS（mNm） | **1.994** | **2.136** |
| 上项 5Hz 幅值／相邻频点最大幅值 | **42.3 倍** | **80.1 倍** |
| 公开速度重建的加速度 RMS（rad/s²） | 2.844 | 2.842 |

原探针速度下限为 0.02rad/s，其余轴速度上限为 0.001rad/s，均通过；其它轴最大速度出现在右肘 pitch。位置解析速度与报告速度的一致性 RMSE 最大 0.00310／0.00220rad/s，低于原 0.02rad/s 条件。实际接收路径也通过原碰撞检查。

相邻频点固定为 4.375／5.625Hz，均与 5Hz 使用同一回调窗口、时间权重和二次趋势消除。扣重力、摩擦后的信号仍含未建模耦合和观测误差，不将它全部命名为独立惯量力矩。

回调时间适配器输出的局部惯量诊断为负向 **0.0007014**、正向 **0.0007510kg·m²**，方向间差异 **7.08%**；这是单中心正反向诊断。原 R2 要求的 128 点完整模型、删构型和删批次检查尚未在这套 SDK 数据上执行。

## 后续已补齐同轴 2×2×2 设计

5Hz 力矩信号已经能从这两条公开记录中分离，保持轴也满足原限制，因此可以继续右腕 pitch 的小范围扩展。两方向惯量诊断已有约 7% 差异，先检查姿态和复测稳定性，比直接扩六轴更有价值。

推荐总计 **两构型 × 正反向 × 两批次＝8 条**：若将本次两条显式绑定为 c00 第一批，还需补 **6 条**：c01 第一批正反、c00 第二批正反、c01 第二批正反。c01 继续使用离线冻结的 pitch／roll = −0.20／+0.15rad，缓存与处理规则保持。

下一轮另建协议，只读绑定本次原记录；不把本轮的 7.08% 当作 R2 通过，也不追加到原 4kHz 训练会话。最初这轮仅完成两条；经后续授权，[6 条补采及原 128 点删组检查](sdk_wrist_pitch_campaign.md)已在新会话完成，删构型／逻辑批次最差为 1.57%／1.77%。 随后[第三姿态独立开发验证](sdk_wrist_pitch_validation.md)也已通过，候选未重拟合。

## 复现与检查入口

运行入口仅能创建本机隔离 SDK Mock，固定只采这两条；没有实机 profile 参数：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/capture_sdk_wrist_pair.py run \
  --output artifacts/sdk_wrist_pitch_c00_pair_new --domain-id 227
```

只读分析或独立复核已有记录，不启动 SDK：

```bash
bash scripts/humble/container.sh python scripts/humble/capture_sdk_wrist_pair.py analyse \
  --source artifacts/sdk_wrist_pitch_c00_pair_v1 \
  --output artifacts/sdk_wrist_pitch_c00_pair_analysis_new

bash scripts/humble/container.sh python scripts/humble/verify_sdk_wrist_pair.py \
  artifacts/sdk_wrist_pitch_c00_pair_v1 \
  --output artifacts/sdk_wrist_pitch_c00_pair_verification_new
```

- 原始 CDR：`c00_right_pitch_*/capture/raw_state/raw/`。
- 实际命令、公开反馈及执行状态：`c00_right_pitch_*/capture/execution/`。
- 速度、保持轴、频点、预处理与局部诊断：`analysis/report.json`。
- 实际反馈图：`analysis/pair_public_feedback.png`。
- 扩展判断和下一步范围：`delivery.json`。

独立复核在禁止打开 `evaluation` 的新进程里，直接解析原始公开 CSV，重算速度、保持和 5Hz 力矩；再重建回归问题和局部惯量诊断，最大差均为 **0**。相关 15 项测试通过，包含原始质量拒绝后仍保留诊断、SDK 故障只 log 清故障占位并终止整对、不自动续采的反例。

所有相同数值发布均保留；2 秒窗中各有 2000 次公开回调，不作独立采样计数。真实采样时刻、缓存年龄和逐字段同步仍未测得，正式采样与实机标志保持 false。
