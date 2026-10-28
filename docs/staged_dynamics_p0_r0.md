# 动力学 P0 R0：数据契约与候选冻结

后续[R1多构型开发](staged_dynamics_p0_r1.md)已接入。本文记录R0边界；当前配置默认规划R1，以下`--contract-only`仍只运行R0。

2026-10-28。对应[三个 P0 计划](staged_dynamics_p0_plan.md)的第一轮 R0。实现了数据边界和读写门禁，尚未执行多构型动力学验收。既有 M1–M6 验收仍是旧协议记录。

## 运行

在仓库根目录，使用不存在的输出目录：

```bash
bash scripts/humble/container.sh casbot-calib plan-staged \
  --config configs/staged_dual_arm_p0.yaml --contract-only \
  --output artifacts/my_staged_p0_r0
```

CLI 会明确显示“R0 数据契约”，返回值只代表契约与解析资格检查。`plan_report.json` 的 `contract_passed` 可为 true，但 `passed`、`p0_acceptance_passed` 保持 false，`selected_trials` 为空。**此时没有碰撞合格轨迹，也没有可执行的机械臂指令。** 仅R0契约会话不能执行采集。当前R1版本2已接入开发采集/重分析/验证，`staged-suite`及最终控制仍等待R4；不得把R0会话回退到旧执行路径。

`configs/staged_dual_arm.yaml` 与旧 schema 1 的采集、重分析继续保留；新生成的旧协议分析/综合验证/交付报告增加 `p0.status=not_executed` 与 `acceptance_passed=false`。不修改历史原始数据或给旧报告补通过标志。

## 冻结内容与清单

新配置是独立的完整配置，不隐式继承另一个 YAML。命令 500 Hz，惯量原始采样 4 kHz，积分与物理模型保持不变。

| 角色 | 逻辑构型中心 | 主要惯量轨迹 | 腕部专项 |
| --- | ---: | ---: | ---: |
| train | 3 | 12 | 24 |
| selection | 2 | 8 | 16 |
| test | 2 | 12 | 16 |

主要惯量轨迹按原计划设计；腕部在各中心覆盖左右腕 yaw 两轴、正反方向、两个独立采集批次。因此共 32 条周期轨迹和 56 条腕部专项。中心目前是逻辑身份，`center_rad` 与 `reference_sha256` 为 null，不声称已经找到七个分离且可运动的中心。

`split_manifest.json` 逐条记录 `split`、`center_id`、`reference_id`、`reference_seed`、`acquisition_batch_id`、`repeat_id`、`acquisition_seed`、`active_arm_group`，以及相位、谐波或腕部轴/方向。角色间中心与种子互斥；同一参考在两个批次重复，采集种子独立。固定种子只在角色内、专项/主激励及批次块内随机排列，保留计划顺序。实际顺序和实际运动覆盖需 R1 采集记录，不能从当前清单推断。

最终独立前序复核另冻结预算：8 个重力姿态双向接近共 16 条，2 个摩擦姿态 × 14 轴 × 3 档速度共 84 条双向扫描，448 条完整起动专项。该 548 条留出不能复用已用于开发选参的前序验证。参考、时长及实际执行仍待 R1；既有开发前序、方向/时序和控制实验不包含在这些增量数字内。

`protocol.json` 加 `.sha256` 封存公开配置、清单、频点资格和独立噪声记录的摘要，以及源码文件、名义 URDF 来源、几何和实际加载原生绑定的摘要。Git HEAD 同时记录；未提交源码以逐文件摘要追踪。`mock` 配置只存入 `evaluation/plant_config.json`，不进入公开配置或选择规则。协议读取不依赖该目录。

哈希用于追溯和发现意外改动，不是数字签名、权限系统或抵御人为重写全部文件的机制。重分析验证冻结输入，不要求当前源码文件仍与历史采集源码相同；当前原生采集内核和采样条件必须匹配协议。后续分析应另行记录其源码身份。

## 角色读取与不可变数据

`staged_protocol.py` 提供后续轮次接入点：

- `seal_role_records(root, split)`：只在该角色全部声明轨迹齐备后创建 `role_inputs/<split>.json` 与摘要。绑定计划、参考、参考碰撞、原始观测、执行指令、前馈、观测碰撞和采集元数据。检查原始摘要、单位、序号、均匀采样、状态、身份、内核与 500 Hz/4 kHz 条件；同参考重复的参考文件必须相同。缺失或拒绝记录不能择优删除后补齐数量。
- `recordings(..., split=..., purpose=...)`：schema 2 必须指定角色和用途。`fit/stability` 只读 train，`select` 只读 selection，`final_test` 只读 test。先核验该角色所有摘要，再提供数据，不先打开全部观测后筛选。旧 schema 不得声称拥有新三分角色。
- test 的采集与角色封存必须绑定已冻结候选摘要；没有 `candidate_freeze.json` 不能采集或读取最终 test。

R0 不提供伪造采集记录的 CLI。自动测试中使用的短数组和假碰撞报告明确标为测试夹具，只用于检验这些接口，不能作为动力学通过证据。

## 选择与最终测试的边界

`freeze_candidate(root, candidate, selection_report)` 是 R2/R3 的接入点，当前没有真实的稳定性/归因报告生成器，因而 R0 不产生已冻结的动力学候选。

后续选择报告 schema 1 必须包含：协议摘要、恰好 train/selection 两份输入清单摘要、三个正则化候选 `1e-5/1e-4/1e-3`。每个候选给出物理参数包路径/摘要、全部 8 条 selection 轨迹 × 14 轴的 NRMSE，以及稳定性、前序开发复核、分量归因三份证据路径。证据头部绑定协议和候选，给出 `completed` 与 `passed`；稳定性/前序资格只绑定train清单，归因绑定train与selection清单；稳定性和前序复核必须通过，归因必须完成但不强制声称惯量收益。

接口检查物理候选可加载，在合格候选中按轨迹/关节等权平均选择；绝对分数差不超过 `1e-6` 时取较强正则化。没有合格候选则拒绝。接口封存候选、选择报告、开发输入和资格证据；不接受 test 输入或 test 评分列。最终 test 已存在时也拒绝重新冻结。封存文件不覆盖，变更候选/阈值/预处理后需新版本、新独立 test。

这些检查保证文件和决策边界，不代替 R2/R3 对证据内容的数值计算。R2/R3 接入时仍须实现并验收报告中的分析，不能手工填写通过状态。候选冻结文件始终声明 `test_consumed=false`、`final_test_passed=false`、`p0_acceptance_passed=false`；只有未来 R4 的独立最终验收才能新增通过证据。

## 频点资格的独立基准

`staged_frequency_contract.py` 生成四个完整周期的独立白噪声电流测量记录，按配置的电流噪声和已知输出增益换算 Nm，与 Mock 动力学、拟合参数和 test 观测无关。对每轴、1–40 阶谐波，以完整周期 Parseval 一侧谱定义 RMS，冻结：

`门槛 = max(0.0001 Nm, 10 × 四周期中该轴该频点的最大噪声 RMS)`。

另用独立解析转子 `tau=J*qdd` 检查频点、量纲与 RMS 归一化；带 DC 的双谐波解析测试检查能量闭合。没有对不连续 moving 样本拼接 FFT。

这只是**合成传感器白噪声资格**，不测量实机带宽、不覆盖驱动纹波、时钟及求导噪声。速度/加速度最小频点 RMS 均预设为 `0.001` SI 单位；R3 还须对实际观测做有效信号检查，弱信号必须报告证据不足，并补充原始带外残差。不能凭当前资格记录宣称已经证明惯量收益。

## 下一轮

R1 负责将逻辑清单解析为七个实际中心与完整进出轨迹，连续 Coal 门禁、运动覆盖、多 train 平衡拟合、独立批次、前序留出和腕部角色接入。正式 test 观测继续留到 R2/R3 完成候选冻结之后。R0 不修改机械内核，因此本轮不重新生成大规模原始数据或渲染视频。

## 本机验证记录

本机忽略目录 `artifacts/staged_p0_r0_20261028/verification.json` 记录协议和源码摘要校验，`tests.xml` 记录最终相关测试。40 项新契约测试通过；完整 Python 回归曾通过 217 项（开启 EGL 渲染检查），最终身份字段等小改动后复跑相关 48 项通过。该次未重建 C++、未重跑 114 项冻结数值对照或实机实验。

旧会话 `artifacts/staged_release` 经 `staged-identify` 输出到新目录 `artifacts/staged_p0_r0_legacy_reanalysis_20261028`，重力、摩擦、起动、惯量重新通过；在声明的 home/速度/加速度检查点，重新生成模型与旧候选的质量矩阵、逆动力学预测最大差异为 0。该检查只证明旧流程兼容，不替代跨构型 P0 验收。新报告明确标记 P0 未执行。

上述路径是本机产物，新检出仓库不保证存在。未采集正式 test，也未生成新候选冻结文件。
