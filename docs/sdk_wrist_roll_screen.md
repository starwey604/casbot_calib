# c00 右腕 roll：两条 SDK Mock 信号筛查

**两条均完成，运动与预处理检查通过；先不扩大补采。** 5Hz 动态力矩可分辨，正反方向局部惯量差为 **4.51%**，但人为施加 ±5ms effort 相对时移后，局部惯量最多变化 **22.03%**。下一步应先离线区分惯量与相位，而不是直接照搬 pitch 的 8 条训练方案。

会话：`artifacts/sdk_wrist_roll_c00_pair_v1`；独立复核：`artifacts/sdk_wrist_roll_c00_pair_verification_v1`。仅运行本机隔离 SDK Mock，采用 **5ms 共享缓存**。原 10% 门槛和正式 test 保持不变。

## 本轮只改变激励轴

复用[右腕 pitch 训练](sdk_wrist_pitch_campaign.md)的封存执行配方、模型、合成 PD、nominal 重力与声明摩擦夹具；左右腕中心 pitch／roll 仍为 **+0.20／+0.30rad**。激励改为右腕 roll（索引 13），正反各一条，其余 13 轴保持。

参考仍是 5Hz、0.004rad 正弦叠加 0.18rad/s 单向运动，测量窗 2 秒。主处理固定 30Hz、零相对时移，首尾各去 0.2 秒；力矩观测与回归量使用共同投影。没有读取 pitch 候选或用其参数解释 roll，也没有拟合新的完整刚体候选。

采集前登记 10／20Hz 共同频带和 effort ±5ms 相对时移作为诊断，不据此选择参数。SDK 故障仍整批终止、不恢复；本次两条均正常完成并关闭所属 Mock。

## 信号存在，速度和保持轴合格

| 指标 | 负向 | 正向 |
| --- | ---: | ---: |
| 有符号实际速度范围（rad/s） | 0.05363–0.30791 | 0.05356–0.31042 |
| 平均有符号实际速度（rad/s） | 0.18003 | 0.17993 |
| 其它 13 轴最大速度（rad/s） | 0.000225 | 0.000158 |
| 其它轴最大位置跨度（rad） | 0.0000376 | 0.0000312 |
| 估算 effort 的 5Hz RMS（mNm） | 1.194 | 1.072 |
| 扣 nominal 重力和声明摩擦后的 5Hz RMS（mNm） | **0.739** | **0.774** |
| 5Hz／较大相邻频点幅值比 | 15.47 | 28.97 |
| 局部刚体惯量诊断（kg·m²） | 0.00026287 | 0.00027472 |

最大回调间隔为 1.17／1.48ms；相邻公开数值完全相同的比例为 82.19%／82.29%，保留全部记录，不将其转换为新帧率。上表扣项后的力矩包含模型、摩擦与观测误差，不是独立测得的纯惯量力矩；相邻频点比也不是统计 SNR。

两方向局部惯量差按 `|J正−J负| / max(|J负|, 1e-6)` 计算，得到 **4.51%**。这是局部诊断，不是原 128 点的删构型／删批次 R2，也未给出真值误差。

## 共同带宽稳定，相对时移影响更大

下表均相对于同一条记录的 30Hz、零偏移局部惯量：

| 诊断条件 | 负向变化 | 正向变化 |
| --- | ---: | ---: |
| 共同频带 10Hz | 0.114% | 0.159% |
| 共同频带 20Hz | 0.076% | 0.008% |
| effort 相对 −5ms | **22.035%** | **17.206%** |
| effort 相对 +5ms | **19.587%** | **14.826%** |

**这里的 ±5ms 是人为施加的字段间错位，不是测出的 SDK 延迟，也不等于共享缓存周期。** 本轮未增加“所有时移变体必须小于 10%”的自动门禁；这些结果用于选择下一项研究。

单频近似 `τ = J·a + b·v + 慢漂移` 下，若将力矩评估时刻改为 `t+δ`，表观惯量为：

```text
J_apparent = J·cos(ωδ) + (b/ω)·sin(ωδ)
ω = 2π·5 rad/s
```

这表明粘性项可经相位错位混入惯量项。一阶相对影响约为 `|b·δ/J|`，本轮两方向约 **20.89%／16.15%**。用主分析的局部 J、b 代入公式，预测与完整时移重处理之差最多为主 J 的 **0.111%**（详见复核目录 `phase_mixing.json`）。它解释了这批诊断中的主要变化；没有据此估计实际设备时延。

## 离线比较已完成：推荐两条双频探针

[冻结摩擦频响与多频设计](sdk_wrist_roll_frequency_design.md)现已完成：现有单频冻结摩擦解的方向惯量差为 24.08%；推荐的最小新设计为 c00 正反各一条，每条同时含 **2.5Hz／0.0075rad 和 10Hz／0.00068rad**，保留 0.18rad/s、2 秒测量窗及原专用包络。

双频在纯相位模型下增加了惯量／粘性／相位的分离信息，但沿用当前频点误差预算的有限扰动最大惯量变化仍为 10.36%，尚不能预言通过原门槛。两条参考已离线检查；[多频观测分析和新协议冻结](sdk_wrist_roll_dual_protocol.md)也已完成，[实际双频两条](sdk_wrist_roll_dual_capture.md)也已完成，预定主模型方向差 8.96%；下一步建议同配方独立会话复测。pitch 候选范围仍为 pitch。

## 运行与只读复核

以下 `run` 会启动并关闭本机 Mock，只采两条；不接受实机 profile。需要已有 pitch 训练的封存执行配方，不需要其候选或训练观测：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/screen_sdk_wrist_roll.py run \
  --source artifacts/sdk_wrist_pitch_campaign_v1 \
  --output artifacts/sdk_wrist_roll_c00_pair_new --domain-id 227

bash scripts/humble/container.sh python scripts/humble/screen_sdk_wrist_roll.py analyse \
  --source artifacts/sdk_wrist_roll_c00_pair_v1 \
  --output artifacts/sdk_wrist_roll_analysis_new

bash scripts/humble/container.sh python scripts/humble/verify_sdk_wrist_pair.py \
  artifacts/sdk_wrist_roll_c00_pair_v1 \
  --output artifacts/sdk_wrist_roll_verification_new
```

分析进程禁止打开原 pitch 会话与任何 `evaluation` 目录，并禁用刚体候选拟合。独立复核从公开 CSV 重算速度、保持轴和力矩频点，再重放主处理及全部诊断；最大差均为 **0**。7 项相关测试通过，包括 roll 通道选择、无 pitch 候选依赖、相位混叠解析基准和原 pitch 故障终止路径。

- `protocol/experiment.json`：两条清单、信号筛查角色、执行配方绑定与固定诊断。
- `c00_right_roll_*/capture/`：原始 SDK CDR、公开反馈、实际命令和状态。
- `analysis/report.json`、`analysis/pair_public_feedback.png`：实际信号及全部诊断。
- `delivery.json`：本轮范围、结果和下一步。

真实采样、字段时序、力矩尺度与硬件资格仍按[SDK 前置条件](sdk_hardware_prerequisites.md)处理；本轮不授予正式 H4 资格。
