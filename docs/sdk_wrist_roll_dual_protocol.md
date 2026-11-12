# 右腕 roll 双频协议：冻结、采集与只读分析

**双频分析与本机 SDK Mock 采集路径已完成实采。** [正反两条结果](sdk_wrist_roll_dual_capture.md)的主模型方向差为 8.96%，后置合成惯量误差最差 6.40%；完整 P0 R2 未执行。

实际完成会话为 `artifacts/sdk_wrist_roll_dual_protocol_v2`。原 v1 启动时被作用域错误的分析隔离规则拦下，0 条探针，失败记录保留；修正后新建 v2，未续接旧尝试。原 5Hz 两条及[离线设计](sdk_wrist_roll_frequency_design.md)保留。

## 合并后的新协议：质量只报告，不阻断

新写入的 JSON 协议版本为 `casbot_sdk_wrist_roll_dual_v2`，采用现场 H4 的最新接口：不再配置或使用 50ms 缺口、非换向实测最小速度、其它轴保持速度、位置／速度一致性等经验阈值来拒绝数值分析。对应统计仍完整保存；数学秩或有限数值确实不足时报告 `available=false` 及原因，保留已采记录，不锁整批、不重采。

现场 profile 新建时使用 1.5rad/s、2秒／3秒接收超时，删除旧跟踪／静态质量字段。10% 仅作惯量差评价，不控制采集推进；S3/P0 正式规则未变。参考规划、碰撞、绑定、SDK／通信／模式故障仍按约定处理。

旧实采目录名 `sdk_wrist_roll_dual_protocol_v2` 的 JSON 实际仍是 **v1 协议**，不是本次新协议版本。它和原数值、原门禁报告均保留，按封存 v1 规则只读重放；不得把新策略写回旧证据。下表配方和以下历史质量阈值属于该原记录。新 v2 冻结与验证在 `artifacts/cloud_h4_integration_v1/new_roll_protocol`。

## 固定的两条与处理规则

| 项目 | 冻结值 |
| --- | --- |
| 清单 | `c00_roll_dual_neg`、`c00_roll_dual_pos`，不自动扩展 |
| 中心／目标轴 | 左右腕 pitch／roll 为 +0.20／+0.30rad；右腕 roll，索引 13 |
| 每条激励 | **2.5Hz／0.0075rad ＋ 10Hz／0.00068rad**，初相均为 0 |
| 基础运动 | 0.18rad/s，测量窗 2 秒；正反符号同时作用于两项正弦和基础运动 |
| 命令／缓存 | 500Hz；5ms 共享缓存夹具，1000Hz 发布 |
| 有效分析窗 | 首尾各去 0.2 秒，共 1.6 秒；固定共同处理频带 30Hz |
| 前馈／保持 | 仍用 nominal 重力、原合成 PD；不加摩擦前馈，其余 13 轴保持 |

新协议移除了旧配方中 5Hz／0.004rad 的单频字段，双频信息只由 `tones` 定义。封存包复制模型 profile、场景、Mock 配置及已有参考预览，不要求在运行／分析时重新读取旧两条观测或原设计目录。模型资源仍由仓库的已绑定 DVT2 上下文加载。

采集共用原测姿态、完整参考规划、Coal 连续检查、位置流及原始 CDR 记录状态机。每条都从当次快照起步，不能直接执行旧预览中的起姿。仅已证明未申请模式、未发命令且无故障的起点变化可重读并重建；分析和复核随 `resolve_field_capture` 读取实际执行的计划／采集。原专用运动包络 1rad/s、8rad/s²、200rad/s³ 和 10mm 碰撞规则保持。

SDK／通信／模式故障仍只 log 清故障占位，然后结束整对；不清故障、不重申请控制、不自动恢复。开始尝试前写入记录，故障、硬中断或状态不明的原尝试不重新进入 SDK；需要新尝试时另建冻结会话。

## 分析并列报告三种模型

报告不再只检查 5Hz，也不逐频独立拟合信号后忽略交叉项；两个频点和二次慢漂移使用同一组曝光权重联合提取。

1. **零相位局部诊断**：保留原时间域加速度、速度与慢漂移回归，作为基线。
2. **冻结 b 的复数频响**：固定声明粘性项，拟合 J 与每条记录的等效相位。
3. **自由 b 的复数频响（主模型）**：同时拟合 J、诊断用 b、每条记录的等效相位，输出跨频残差。

力矩、加速度、速度、nominal 重力、转子和摩擦特征先经原有同带宽处理，再共同提取复数系数。重力按送达 q 用原生 Pinocchio 计算，不用规划中的线性刚度近似代替。

主值相位支固定为 ±25ms，这是 10Hz 模型求解的分支选择，不是 SDK 延迟范围或采样证据。等效相位不写回原始时间，也不改 S3 资格；拟合的 b 只作诊断，不更新 H3 摩擦或刚体候选。

每条报告包含：

- 实际有符号速度、其余轴保持、位置跨度、回调缺口及位置／速度一致性。
- **2.5Hz 与 10Hz** 的实际速度、估算 effort 和扣声明重力／摩擦后的力矩 RMS。
- 实际送达频点的 J/b/相位局部秩；缺少第二频点时报告不可分离，不输出一个假唯一解。
- 冻结 b／自由 b 的 J、等效相位、残差，以及各频点单独对应的等效相位。
- 四次删去 0.4 秒块后的结果；保留原行权重，不把缺口算成曝光时间，也不当作独立采样或置信区间。

历史 v1 封存质量条件为：有符号速度 >0.02rad/s、保持轴速度 ≤0.001rad/s、最大回调缺口 ≤50ms、位置导数／反馈速度一致性 RMSE ≤0.02rad/s。v1 质量拒绝时仍保存两频点的原始信号检查；新 v2 不再据这些阈值拒绝分析。

两方向惯量差按 `|J正−J负| / max(|J负|,1e-6)` 分模型报告，并与原 **10%** 比较；不挑最容易通过的模型。这只是局部方向诊断，完整 P0 R1/R2 与正式硬件资格仍为 false。原设计的条件扰动最大 10.36% 结论没有因代码接入而变成通过。

## 冻结与只读验证：不启动 SDK

已完成的冻结命令如下；重新运行需使用新目录：

```bash
bash scripts/humble/container.sh python scripts/humble/sdk_roll_dual.py freeze \
  --source artifacts/sdk_wrist_roll_frequency_design_v3 \
  --output artifacts/sdk_wrist_roll_dual_protocol_new

bash scripts/humble/container.sh python scripts/humble/verify_sdk_roll_dual.py \
  artifacts/sdk_wrist_roll_dual_protocol_new \
  --output artifacts/sdk_wrist_roll_dual_adapter_review_new
```

验证程序用解析数组构造 5ms 缓存读出，禁止打开 SDK、快照或执行入口。包含正反方向、−5／0／+5ms 人为时移和 0.8／1.0／1.2 倍粘性项，共六组；使用完整 C3 参考和原生重力，不推进机械模拟器。

本轮结果在 `artifacts/sdk_wrist_roll_dual_adapter_verification_v1`：主模型最大惯量误差 **0.00770%**，等效相位误差最大 **1.66µs**。这是理想读出处理链验证，实际两条的信号、等效相位和方向差另见[实采结果](sdk_wrist_roll_dual_capture.md)。25 项相关测试通过，覆盖多频缺失、保持轴拒绝、冻结输入自包含、实际起姿接入、故障整对终止、只读分析入口及旧单频调用。

## 下次采集时使用独立的 collect 动作

**以下命令会启动本机隔离 SDK Mock 并采两条，必须使用新冻结的会话。** 已完成的 v2 和失败的 v1 都不能续采；入口不接受实机 profile，也不扫描域号：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/sdk_roll_dual.py collect \
  --source artifacts/sdk_wrist_roll_dual_protocol_new --domain-id 227
```

采完会在同一会话生成 `analysis/`。之后只读重分析，不启动 Mock：

```bash
bash scripts/humble/container.sh python scripts/humble/sdk_roll_dual.py analyse \
  --source artifacts/sdk_wrist_roll_dual_protocol_v2 \
  --output artifacts/sdk_wrist_roll_dual_analysis_new
```

`analyse` 只消费本会话已完成的封存观测，核对频率、方向、模型、重力、力矩尺度／零偏及命令频率；协议尚无采集时会直接说明缺少观测，不代为启动采集。分析进程禁止访问 `evaluation`，也不拟合完整刚体候选。

## 文件位置

- `protocol/experiment.json`：两条清单、配方、主模型、比较模型、删块规则和 10% 数值。
- `protocol/reference_previews/`：从离线设计复制的参考和证明，仅作预览。
- `c00_roll_dual_*/capture/`：实际 SDK CDR、公开反馈、命令及状态，已在 v2 留存。
- `analysis/report.json`：质量检查、三个模型和方向对照。
- 验证目录 `verification.json`、`dual_frequency_signals.png`：当前六组解析读出验证，图明确标为非采集数据。

当前实采结果支持下一轮同配方独立会话复测，详见[结果与建议](sdk_wrist_roll_dual_capture.md)；不自动扩大到其它姿态或批次。
