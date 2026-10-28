# 动力学 P0 R1：多构型开发会话

对应[三个 P0 计划](staged_dynamics_p0_plan.md)第二轮 R1。名义14轴双臂、腰部刚性固定。Pinocchio/Coal/Boost.Odeint/Clarabel分工不变，无实机接入、无新机械内核。R1通过只表示开发流程通过，稳定性、归因和最终测试仍需R2–R4。

## 运行入口

使用不存在的输出目录，已有前序会话显式绑定为**开发参考**，不复制其数GB数据，不算新的独立批次：

```bash
bash scripts/humble/container.sh casbot-calib plan-staged \
  --config configs/staged_dual_arm_p0.yaml \
  --development-reference artifacts/staged_release \
  --output artifacts/my_p0_r1

bash scripts/humble/container.sh casbot-calib staged-calibrate \
  --config configs/staged_dual_arm_p0.yaml \
  --output artifacts/my_p0_r1 --resume --through inertia

bash scripts/humble/container.sh casbot-calib staged-identify \
  artifacts/my_p0_r1 --output artifacts/my_p0_r1_reanalysis

bash scripts/humble/container.sh casbot-calib staged-verify \
  artifacts/my_p0_r1 --output artifacts/my_p0_r1_verification
```

也可用`staged-calibrate --development-reference ... --output 新目录`合并规划和采集。默认`--through all`在P0 R1只到开发惯量检查点，不启动最终test、控制选增益或视频。`staged-suite`仍拒绝P0配置，等待R4。`plan-staged --contract-only`保留R0契约入口。

`--resume`核对配置、Mock声明、协议、前序冻结及已有原始摘要，不重复采集完整记录。未完成写入保留到`interrupted_records`。执行日志记录开始/完成和元数据摘要；目录锁阻止并发采集。进程崩溃遗留锁时需先检查所有者及日志，再处理锁。已有辨识不能静默覆盖或重新择优。

## 实际中心与完整参考

R0逻辑`split_manifest.json`不改写；新封存协议版本2在封存前生成`motion_plan.json`：

- 固定种子池256个中心，在各轴激励幅值的±0.45范围生成。选择train三个、selection两个、test两个中心。
- 中心同时满足归一化RMS距离≥0.2、至少两轴实际角差≥0.05rad，不用角度wrap。
- 每中心全部唯一参考完整通过才接受。拒绝候选和见证保留，不减小激励、不放宽10mm余量补数量。
- 主要激励活动轴要求行程>0.15rad、速度RMS>0.01rad/s、加速度RMS>0.02rad/s²，完整路径连续速度/加速度/jerk界为2/8/60 SI单位。腕部保留1/8/200界及不反向运动。
- 初始保持、接近、活动、制动、回位、最终保持均复用Bernstein导数界和Coal连续门禁。同参考两批数组摘要相同，身份与采集种子不同。

`staged_motion.py`抽取共用`periodic_motion`、`wrist_probe_motion`，旧新流程不维护两套波形或物理实现。最终前序留出另用独立参考种子预规划16条重力、84条双向匀速摩擦、448条完整起动专项，本轮不采集。

根目录`configuration_coverage.json`是规划覆盖；辨识目录同名文件才是实际覆盖：稳态实际位置均值/范围、原始速度RMS、匹配处理后的加速度RMS及观测中心分离。不能从指令推断实际覆盖。

## 前序与新增采集

`development_reference.json`固定外部原始会话绝对路径及摘要，核验相同模型、几何、场景、驱动、采样等条件。方向、时序、重力、摩擦、起动必须完整有效。分析只读公开记录；采集器另核验声明的合成物理系统相同。

`development_analysis`从原始记录重新辨识并通过前序门禁，`predecessor_freeze.json`冻结结果后才激励。已有前序验证明确是开发验证，不是最终留出。归档/搬迁需保留外部会话，单独复制R1目录不保证自包含。

新增train36条、selection24条，即20条主要周期轨迹和40条腕部专项。按角色内冻结的专项/批次块顺序执行；每条经同一C++ Runner独立初始化，记录独立种子及实际顺序。命令500Hz、原始采样4kHz。Mock两批只代表初始化与噪声变化，不代表跨日、热漂、网络变化。

`role_inputs/train.json`及`selection.json`封存计划、原始观测、指令、前馈与碰撞文件。采样/接收时间分别保留。力矩仍由电流乘声明输出增益得出，不是独立测力。

## 多train拟合与评分

`staged_multifit.py`显式读取角色并拼接全部train周期与腕部数据。每条轨迹/轴的权重只由train确定，完整输出：

`sqrt_weight = sqrt(1024 / (有效样本数 × 同类轨迹数)) / max(train力矩RMS, 0.2Nm)`。

主要激励的腕yaw两轴再乘预声明0.1；腕部单轴专项独立成块。1024是冻结的目标函数总体尺度，避免帧数改变正则化含义，不是统计方差模型。R1固定ridge=1e-4，先固定摩擦拟合刚体，再按原有±5%及小绝对下限边界做一次有限联合修正；重力等式、静摩擦冻结。

selection仅在拟合完成后加载作开发预测检查，不进入参数矩阵，不用于本轮选择正则化。开发`candidate_initial.json`、`candidate.json`均`publishable=false`、`p0_acceptance_passed=false`，不生成`candidate_freeze.json`。R2还须固定比较尺度并实现敏感性，R3须归因，之后才正式选参冻结。

逐轴沿用动态NRMSE≤5%、静止区间超界≤0.01Nm、归一化下限0.2Nm。周期指标保持原有40阶同带宽投影（本配置0–6.667Hz），不减常值偏置；原始带外诊断待R3，不以此宣称全带宽误差。腕部继续30Hz匹配滤波和独立力矩平衡，验收惯量差异≤10%、预测RMSE≤0.002Nm。联合修正后重验前序开发记录。

`batch_repeatability.json`比较同参考两批的完整周期残差均值和RMSE，逐轴均值差≤0.01Nm，两批各自通过预测。train重复仍是拟合内数据，不称独立预测。CSV及分组报告按角色/中心/批次列出最坏值，腕部单列。

## 验证及最终测试边界

R1的`staged-verify`复核所有新增参考和观测插值碰撞，在train/selection周期和左右腕代表轨迹上做步长减半；候选模型与独立合成评估系统分别报告。把R1和外部前序整个`evaluation`目录临时移出后，新进程重分析，恢复目录并比较算子。该检查不替代R2/R3或实机验收。

正式test只预规划，不采集、不读观测。后续`collect_final_test`接入点先验证冻结候选再构造Mock，将最终周期/腕部和完整前序留出绑定同一候选/协议并封存。最终评分、四组控制、完整P0发布、中文回放待R4；接口存在不代表最终验收完成。

## 本机交付与实测软件结果

最终会话为本机忽略目录`artifacts/staged_p0_r1_release`，总览为`r1_delivery.json`；新检出仓库不保证这些产物存在。

| 检查 | 结果 |
| --- | --- |
| 最终会话新增采集 | 60条，36 train / 24 selection；实际观测5个开发中心 |
| 完整参考规划 | 7中心、636条，包含未采集的576条正式test/前序留出 |
| 最差逐轴周期NRMSE | 2.1282%，门槛5% |
| 最差腕部独立惯量差异 | 1.3219%，门槛10% |
| 最差腕部预测RMSE | 0.00011587Nm，门槛0.002Nm |
| 两批同参考逐轴残差均值最大差 | 0.000008053Nm，门槛0.01Nm |
| 步长减半 | 4条代表轨迹×候选/被测系统两种模型均通过；最大峰值变化0.01415%，门槛1% |
| 新进程、两处真值目录均移出 | 重新辨识通过，检查点质量矩阵/逆动力学最大差异0；真值目录已恢复 |
| 回归 | 完整Python 231项；最终输入保护相关56项；冻结数值114项通过 |
| 旧轨迹兼容 | 12条腕部、4条周期的参考多项式数组与旧会话完全相同 |

最终60条为872秒虚拟实验；本次规划后采集及辨识约675.85秒，完成检查点的resume核验约19.57秒。开销以`r1_delivery.json`及`workflow_capture.json`记录，最近一次调用的`workflow.json.wall_s`不当成整次采集耗时。主要惯量求解在正式开发拟合和无真值复现各做初次/联合两次，共4次调用，不含前序或历史回归。

另有一次60条开发探查，复用了相同冻结参考/种子，用于实现验证；不算额外独立批次，不参与最终会话拟合。两份会话总计写入120条新Mock记录，探查耗时和位置在总览中另列。

采集后补充了两项输入保护：完整检查点resume重新核验角色输入封存，分析前拒绝已更换的名义模型。数值模型、权重、处理和门槛未改；采集时对应的两份源码保存在`capture_source`，摘要与原协议相符，差异另存。最终保护检查通过56项，包含错误模型被拒绝的反例。C++未改、未重建，本轮没有重新运行CTest、物理网络或实机测试，也未生成R4视频。
