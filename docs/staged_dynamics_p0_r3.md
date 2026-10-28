# 动力学 P0 R3：同带宽归因与候选冻结

2026-10-28。复用[R1](staged_dynamics_p0_r1.md)的完整train/selection观测及[R2](staged_dynamics_p0_r2.md)三档完整train物理解，不重新采集或拟合。该轮回答预测改善来自哪里，并选择一个候选供R4独立test；不发布模型、不宣称实机精度或完整P0通过。

## 入口

```bash
bash scripts/humble/container.sh casbot-calib staged-identify \
  artifacts/staged_p0_r1_release --through attribution \
  --stability-verification artifacts/staged_p0_r2_verified \
  --output artifacts/my_p0_r3

# 只读独立复核；加 --freeze-candidate 才会将合格候选冻结到父会话
bash scripts/humble/container.sh casbot-calib staged-verify \
  artifacts/staged_p0_r1_release --attribution-analysis artifacts/my_p0_r3 \
  --output artifacts/my_p0_r3_checked --freeze-candidate
```

输出目录必须不存在，R3不提供resume。R2输入必须是`staged-verify --stability-analysis`生成的独立验收目录，而不是未经核验的拟合目录。整个过程复核R2封存、父会话、两个角色的全部输入摘要及nominal几何。正式test和最终前序留出已经存在时拒绝运行R3；看到正式测试后需要新协议和新测试批次。

## 分量与频段

| 对照 | 刚体参数 | 运动摩擦 |
| --- | --- | --- |
| A | 已接受的前序重力物理代表 | 前序冻结值 |
| B | R2固定摩擦初次惯量解 | 前序冻结值 |
| C | R2有限联合修正的刚体解 | 前序冻结值 |
| D | 同一联合修正的刚体解 | 修正值 |

C是诊断对照，没有新增拟合。所有对照保持相同的静摩擦、已知armature、观测和时间处理。Pinocchio分别计算刚体惯性、已知转子、速度耦合与重力；摩擦使用原始速度先计算非线性特征、再投影的结果。原生`inertia`已经包含armature，因此报告刚体惯性时减一次、总和中再加一次，检查与匹配回归量预测闭合。

沿用六秒周期、舍弃前两周期、1024点相位网格和40阶投影；最高保留频率为6.667Hz。每条轨迹先对完整周期做FFT，再进行原有moving/静止专项评分，不拼接moving样本。分别报告DC、该轨迹声明的四个激励谐波、其余保留谐波。使用单边Parseval能量和：DC/Nyquist权重1，其余权重2，不按频点数除平均。同一轨迹的A–D必须同周期、网格、带宽和频点；跨轨迹先算各自误差，再按轨迹和有效轴等权汇总。

原始4kHz电流的两个完整稳态周期另做诊断：从每周期电流谱减去带宽内模型谱，再对各周期能量平均。不会先平均周期、抵消噪声。40阶外保留原始电流能量，不假装带限模型在该处预测为零就证明高频通过；这些内容可能包含驱动、粘滑与噪声，不能替代完整带宽的机械验证。

## 信号资格、偏置和收益状态

R0独立白电流噪声谱给出每轴/频点的力矩门槛（至少0.0001Nm）。R3对声明频点同时要求：

- 原始实测速度的频点RMS至少0.001rad/s；实际位置傅里叶导数的加速度RMS至少0.001rad/s²。
- 观测电流换算力矩扣除冻结A的重力/摩擦后，该频点超过噪声门槛。
- 冻结A的惯性代理（刚体+已知转子）也超过同一门槛，避免只凭重力或摩擦残差判为惯性证据。

资格不依赖ridge、B、C或D。惯性代理是先验模型计算，**不是独立测力或新的惯量可辨识性证明**。每轴至少一个合格频点；不足的轴保留为`insufficient_signal`，不能删掉后宣称全范围惯量收益。本轮实际每条周期轨迹14轴均有四个合格频点。R0冻结了噪声/速度/加速度阈值；上述完整组合规则和最少频点数在R3执行前另行封存，不追溯称为采集前已完成的独立硬件资格。

同步输出含偏置RMSE、平均偏置和去均值形状误差。可选常值诊断偏置仅由train完整周期均值等权估计，并在计算selection诊断前冻结；它不会进入物理候选、URDF或选参评分。selection不逐条拟合可部署偏置。腕部非周期扫描继续使用原有匹配Butterworth滤波与力矩平衡方法，不强行使用周期FFT。

原始未去偏置的逐轴预测和腕部门禁继续执行。B/A及D/A的合格频段逐轴RMSE不得超过`max(1.05×A, A+0.002Nm)`。候选资格要求稳定性、前序开发门禁、原始预测及频段不退化均通过；**预测资格和惯量收益是两个状态**。全范围信号充分、B平均频段误差改善至少1%才支持本角色的惯量更新收益；只有D改善标为联合收益，等效则标为等效。最终收益声明仍需R4正式test复验，不能把selection结论当最终结论。

## 输出与冻结

- `analysis_protocol.json`：读取selection数值前封存本轮规则、R2模型/证明、角色清单、原生绑定及源码摘要；`source/`保存对应Python源码。
- `residual_attribution.json`：分量闭合、A→B→C→D顺序误差变化、偏置、原始带外诊断、原有预测/重复性和候选资格。顺序分解允许分量抵消，不表示唯一因果归因。
- `frequency_budget.json`：逐轨迹/轴/激励点实测信号资格、频段能量及不退化门禁。
- `axis_center_batch.csv`、`residual_attribution.png`：逐轴/构型/批次明细和中文对照图。
- `ridge_*/`：未改动的R2 B/D候选、仅train的偏置封存及可重算分量/谱数组。所有推断入口均不读取`evaluation/`。

独立验收重新读取原始train/selection、构建匹配问题，通过Pinocchio重算全部模型分量，检查Parseval与闭合、冻结偏置、频点资格、原有门禁及选参。按原R0规则：八条selection周期轨迹×14轴未去偏置NRMSE等权平均，绝对差不超过1e-6时选较强正则化。

`--freeze-candidate`在核验之后将小型证明/模型JSON快照写入父会话`selection_r3_<摘要>/`，再生成一次性的`candidate_freeze.json`和摘要。冻结绑定候选、选参报告、角色清单、资格报告及底层证明；改动其中任何一项会使后续test读取拒绝。只有归因完成不足以选参，原始预测或频段不退化失败也会拒绝。冻结仅允许R4独立测试，`publishable=false`、`final_test_passed=false`、`p0_acceptance_passed=false`。

## 本机结果

分析目录`artifacts/staged_p0_r3_release`，最终独立验收及冻结结果`artifacts/staged_p0_r3_verified`，总览为其中的`r3_delivery.json`；本地产物不随Git提供。`artifacts/staged_p0_r3_checked`是先前不冻结的只读核验。分析约63秒，新增采集/拟合均为0。分析及最终复核均在两处`evaluation/`目录物理移走后通过新进程执行，返回0并恢复，记录分别为`artifacts/staged_p0_r3_release_execution.json`和`artifacts/staged_p0_r3_verified_execution.json`。实际中文PNG已查看。

三档ridge均具备资格，按selection规则已冻结`1e-3`；候选参数保持R2原值。父会话中的`candidate_freeze.json`绑定19个输入/证明文件。核验报告先封存再执行冻结，最终动作以`freeze_result.json`和父会话冻结文件为准。该候选的selection结果：

| 指标 | 数值 |
| --- | --- |
| 轨迹/轴等权NRMSE | 0.30265% |
| 最大逐轴NRMSE | 1.35352%，门槛5% |
| A/B/D平均合格频段RMSE | 0.00231795 / 0.000633833 / 0.000639642 Nm |
| B相对A改善 | 72.655%，只支持当前selection角色 |
| D相对A改善 | 72.405% |
| 腕部惯量平衡最大相对差 | 1.33006%，门槛10% |
| train/selection分量最大闭合差 | 6.22e-15 Nm |

保留带外诊断很有必要：40阶外原始电流残差最大RMS约0.01262Nm，不能用较低的带内误差宣称该部分已建模。已规划的28条正式惯量/腕部测试与548条最终前序留出，共576条仍未采集。后续R4使用冻结候选采集和验收，再进行控制对照及中文回放；不得根据正式test结果改模型后沿用同一批test。

本轮52项针对性测试、259项完整Python测试及原A–F/R1/双臂114项冻结数值对照通过。反例覆盖仅偏置收益、仅摩擦收益、弱轴不能删除、跨带宽拒绝、非线性处理顺序及独立解析惯量力矩。60组候选×周期轨迹的保存分量与独立Pinocchio重算差为0，Parseval最大能量差约2.44e-19Nm²。C++/机械积分未修改或重建，本轮未重跑CTest、ROS网络或实机测试。
