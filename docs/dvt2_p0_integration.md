# 用DVT2模型运行动力学P0协议

已有DVT2方向、时序、重力、运动摩擦和起动摩擦数据后，可以将它们绑定为P0的开发参考，再采集多构型惯量数据。入口为`configs/dvt2/staged_dual_arm_p0.yaml`，沿用既有R0–R4命令，不另建采集器或辨识算法。

本协议选择裸法兰整机、固定腰部与地面场景，不安装三表工具和台架。动态模型、Coal场景、数据身份、候选和只读显示共用`model_context`；千分计几何实验仍使用[实际四球装配配置](dvt2_trical_integration.md)。

**物理静止门禁已修复，封存的60条开发数据复评分及独立R1复核通过。** 原拒绝报告保留；新结果见文末，默认随后已整体切换；N4和DVT2的R2–R4进展见[N4收尾](dvt2_n4_closeout.md)。以下命令用于新实验；已有拒绝会话使用文末的显式重评分入口。

## 先准备同源前序，再启动P0

在仓库根目录执行。以下目录必须不存在；若继续已冻结的同一会话，应保留原配置并显式使用`--resume`。

```bash
# 方向/时序→重力→运动摩擦及4/8kHz对照→重力补偿起动
bash scripts/humble/container.sh casbot-calib staged-calibrate \
  --config configs/dvt2/staged_dual_arm.yaml \
  --output artifacts/my_dvt2_predecessors --through breakaway

# 七个中心、32条主要惯量参考、56条腕部参考、548条最终前序参考
bash scripts/humble/container.sh casbot-calib plan-staged \
  --config configs/dvt2/staged_dual_arm_p0.yaml \
  --development-reference artifacts/my_dvt2_predecessors \
  --output artifacts/my_dvt2_p0

# 只采36条train、24条selection，拟合固定1e-4的开发候选
bash scripts/humble/container.sh casbot-calib staged-calibrate \
  --config configs/dvt2/staged_dual_arm_p0.yaml \
  --output artifacts/my_dvt2_p0 --through inertia --resume

# 复核60条参考/观测路径、代表轨迹步长减半、移走真值后的新进程重分析
bash scripts/humble/container.sh casbot-calib staged-verify \
  artifacts/my_dvt2_p0 --output artifacts/my_dvt2_p0_checked
```

只检查协议和噪声频点资格时，使用`plan-staged --contract-only`，不传`--development-reference`；该输出没有运动参考，不能用于采集。

`--development-reference`必须指向相同DVT2上下文、场景、驱动/采样和合成物理系统的schema 1会话，且五个前序阶段完整通过。外部原始记录通过封存路径引用，不复制大数据；搬运P0目录时也须保留被引用的前序会话。前序验证在这里用于开发，不计作新独立留出。

## 新实验保留数值门槛

新实验版本为`synthetic_dvt2_p0_003`，参考、采集、排序、中心/先验/探针及噪声种子与旧nominal实验分开。角色规模和数值门槛不变：

- train三个中心、selection两个、test两个；每条参考两个独立采集批次。
- 命令500Hz，主要惯量及腕部原始观测4kHz；六秒周期、舍弃前两周期、1024相位点、40阶匹配处理。
- 主要活动轴行程>0.15rad、速度RMS>0.01rad/s、加速度RMS>0.02rad/s²；不因新模型缩小激励补数量。
- 动态逐轴NRMSE≤5%，物理静止区间超界≤0.01Nm；腕部惯量差≤10%、预测RMSE≤0.002Nm。物理静止使用下述原始窗口，不用40阶投影的低速样本。
- R2/R3仍用三档ridge、固定尺度/权重/先验边界和前序复核；不通过降低阈值迁移。

R1新增36条train和24条selection，只观察五个开发中心。另两个test中心及548条最终前序只规划，需R3候选冻结后才允许采集。旧nominal正式test不能改名或改写上下文后复用。

## 轻腕使用零刚度与已知速度阻尼

DVT2裸法兰模型的部分腕轴很轻。零PD斜坡下，旧控制/反馈延迟与3ms持续确认会使恢复位移或速度先超限；将确认后的D增益加倍也不能覆盖全部腕轴。失败数据保留，不降低原2mrad/0.4rad/s限幅或改变起动判据。

用户已批准新合成实验使用`breakaway_probe_kd_nm_s_rad: 0.25`：静态等待和力矩斜坡期间，目标轴`Kp=0`、`Kd=0.25 Nm·s/rad`，速度参考为零；起动确认后恢复原位置刚度、2倍原速度阻尼。三个确认条件仍是0.2mrad位移、0.005rad/s速度和3ms持续时间。

阻尼在静止时为零。辨识继续读取电流换算的实际输出力矩，它已包含本地PD作用，不能再加上`Kd*dq`当成第二组测量。确认后力矩不是起动估计值，仍回溯最后静止/首个可分辨运动样本。配置、逐条参考条件、已执行五字段和保护记录保存该控制条件。

旧配置未声明上述字段时保持零PD；实机H3本轮不变。换控制协议后的采集使用新目录，不覆盖或续采已拒绝的旧批次。

## 后续仍沿用R2–R4入口

R1开发检查通过后，按照[R2稳定性](staged_dynamics_p0_r2.md)、[R3归因与冻结](staged_dynamics_p0_r3.md)、[R4正式验收](staged_dynamics_p0_r4.md)推进，将示例父会话换成本次DVT2目录。R4的`--config`必须与父会话一致：003使用`configs/dvt2/staged_dual_arm_p0.yaml`；本轮004开发实验使用`configs/dvt2/staged_dual_arm_p0_v4.yaml`，见[独立开发说明](dvt2_acceleration_development.md)。两者不能混用，也不能在R2/R3通过前采集正式test。

源码中的模型入口已经贯穿多构型回归、前序模型恢复、敏感性、归因、候选冻结/加载、最终评分、控制与显示。候选JSON保存DVT2上下文，加载时同时核对源URDF和关节顺序；仅有相同的140维参数长度不构成兼容。原始记录读取也核对会话上下文，拒绝混入旧模型数据。

所有阶段仍受前序门禁约束；没有完成R2/R3和新正式test时，`p0_acceptance_passed`保持false。实机的力矩尺度、时间/带宽资格、负载和安装条件按[SDK前置条件](sdk_hardware_prerequisites.md)核验。

## 原开发结果与门禁缺陷（历史记录）

汇总为`artifacts/dvt2_n3_p0_delivery_v1/delivery.json`，迁移过程见[N3接入记录](dvt2_n3_integration.md)。

- `dvt2_n3_p0_predecessors_v2`：739条新原生前序记录通过，含448条零刚度＋阻尼起动。
- `dvt2_n3_p0_development_v1`：636条参考规划通过，新增36 train/24 selection记录均通过采集门禁；尚未采集任何正式test。
- R1最大逐轴NRMSE3.135%（门槛5%）、腕部惯量差2.464%（门槛10%）、腕部预测RMSE0.0000795Nm（门槛0.002Nm）。但`train_c02_00`两批的滤波静止超界为0.01168/0.01166Nm，高于0.01Nm，R1整体保持拒绝。
- `stationary_gate_oracle.json`显示：相同处理下连真实参数也报0.01133Nm超界；原始物理数据超界为0。硬截断傅里叶投影不保持逐点区间，不能直接把滤波力矩套进原始静摩擦界。
- 移走两份evaluation、禁止模拟/采集工厂的新进程重分析复现同一拒绝，参数差0，60份原始数组保持封存。

## 修复：用原始静止窗口检验物理区间

`static_intervals.py`由原生staged和合格S4观测共同使用。运动段仍按原40阶处理、拟合和评分；静止门禁独立读取整条轨迹的原始保持段：

1. 原始位置、速度及唯一输出侧力矩通道须有限、同步，采样网格连续均匀且状态有效；不跨缺帧拼接。S4的原始位置与力矩采样时间不同则不授予静止资格。
2. 在至少20ms的居中窗口内，每轴速度绝对值≤`1e-5 rad/s`，位置跨度≤`2e-6 rad`。整条固定基座运动分支都满足才选中。当前固定腰部模型中，一条分支是一整条7轴机械臂；本臂其他轴运动时，单轴过零不算静态姿态。另一臂停稳时可单独观测。
3. 用Pinocchio在原始位置上计算候选重力，与原始Nm观测比较：`max(|g(q)-τ_obs| - f_static, 0)`。不数值求加速度，不筛选力矩残差；静摩擦参数仍冻结在前序结果。
4. 每条主要惯量记录的每轴至少32个合格样本，且最大超界≤**0.01Nm**。缺少真正静止窗口明确拒绝，不以零超界或滤波结果代替观测资格。报告保存逐轴窗口、样本数、阈值及资格原因。

`filtered_stationary_diagnostic`保留原40阶低速超界，不参与验收。这是物理区间的定义修正，未增大静摩擦值或数值门槛。上述速度和位置窗口属于本机合成协议；实机的噪声、采样与静止资格仍需独立验证，不能直接用SDK发布stamp套用。

### 已有封存R1只重评分，不重拟合

使用新目录保留旧拒绝报告和检查点：

```bash
bash scripts/humble/container.sh casbot-calib staged-identify \
  artifacts/dvt2_n3_p0_development_v1 --reuse-development-fit \
  --output artifacts/my_static_revalidation

bash scripts/humble/container.sh casbot-calib staged-verify \
  artifacts/dvt2_n3_p0_development_v1 \
  --development-analysis artifacts/my_static_revalidation \
  --output artifacts/my_static_checked
```

第一条验证原始输入与拟合封存，逐字节复制原候选，只重建观测和评分，不调用机械参数拟合。第二条显式核验新分析的父协议/输入/报告封存，再复核路径、步长及无真值新进程重评分。原会话`workflow.json`不会自动变成通过；不带`--development-analysis`仍拒绝旧检查点，不能静默覆盖历史。

### 本次修复的实际结果

- `artifacts/dvt2_static_gate_revalidation_v2`：同一36 train/24 selection，参数不重拟合。20条周期记录全部获得原始静止资格，逐轴最少4,375个样本；最大物理区间超界**0.000969Nm**。原滤波诊断最大0.011677Nm仍在报告中。
- 原候选、主要运动评分和40条腕部评分保持不变。R1五项检查全部通过；运动NRMSE仍≤3.135%、腕部惯量差≤2.464%。
- `artifacts/dvt2_static_gate_verify_v1/verification.json`：60条参考/观测路径、4组候选及4组声明plant的积分步长减半通过；两份evaluation移走后新进程重评分复现，算子差0，静止指标一致，随后恢复evaluation。
- `static_gate_regression_v1.log`中125项相关回归通过；补充的`static_gate_final_contract_v1.log`中14项检查通过，覆盖真实重力、缺静止证据拒绝及禁止重拟合。两组有交叠，不相加为全量测试。

汇总及中文对照图在`artifacts/physical_static_gate_delivery_v1/`。正式test没有采集，`p0_acceptance_passed=false`。该修复轮完成静止门禁与R1重验，旧失败检查点不升级。随后已完成显式R2接续与默认切换；当前SDK H4代表性验证已通过；004开发实验的R1独立复核通过、R2仍拒绝，详见[N4收尾](dvt2_n4_closeout.md)。

## 将修正后的R1显式交给R2

原拒绝会话保持不变。新R2分析可以明确消费已经通过的独立R1复核，不再依赖把旧`workflow.json`改成true：

```bash
bash scripts/humble/container.sh casbot-calib staged-identify \
  artifacts/dvt2_n3_p0_development_v1 --through stability \
  --development-verification artifacts/dvt2_static_gate_verify_v1 \
  --output artifacts/my_dvt2_r2
```

R2封存所选R1复核、分析及输入身份；续跑时再次检查。没有该参数时，原已通过的R1调用方式保持不变，原拒绝会话则明确拒绝。此入口只解决阶段接续，不改变稳定性指标或三档ridge的资格。R2实际运行与后续结果见[N4收尾](dvt2_n4_closeout.md)。
