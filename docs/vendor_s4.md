# S4：合格观测的分阶段动力学闭环

本轮接入的是**离线合格观测→分阶段拟合→前序复核→Mock控制对照**。数据可以来自S3的SDK/主机日志入口，力矩直接用Nm；核心求解器、Pinocchio模型、Coal门禁、Boost积分和Mock内核继续复用。当前甲方的effort/控制/停止等物理条件仍未知，S2的localhost限制保留，没有新增实机运动入口，也没有把这次软件验收写成S4实机交付已完成。

## 新入口与角色冻结

`vendor_staged.py`提供协议冻结、观测适配、检查和顺序编排。`torque_observations.py`让原有重力、摩擦、起动和腕部惯量数值函数消费一份总输出力矩：旧Mock走明确增益的current，新S3记录走torque_nm；同时提供两种通道会拒绝。没有把Nm改名为current，也不会再乘减速比。

S4 spec为`casbot_vendor_s4_spec_v1`，scope区分合成/实机观测，settings的mode必须为`qualified_observations_only`，不允许包含mock真值。最小未知配置见[vendor_s4_unknown.yaml](../configs/vendor_s4_unknown.yaml)，保持500Hz控制/发布默认值，不提供厂家增益、噪声或控制能力默认值。

每条record明确以下内容：

| 字段 | 含义 |
| --- | --- |
| name / stage / split | 原始试验名，gravity/friction/breakaway/inertia/inertia_probe，train/validation/sampling |
| qualification | S3资格目录manifest.json的path与sha256 |
| motion | 原始观测条件/阶段窗口JSON的path与sha256 |
| time_origin_ns | 明确的整数采样时间原点；数值核用相对采样时间，原始S3时刻保持不变 |
| reference_collision / actual_collision | 参考和已观测路径检查的文件与摘要；不推断为未观测物理路径证明 |
| guard | 起动斜坡的实际反馈确认事件及阈值；不能用预定斜坡结束代替 |
| source_kind | 本轮明确区分旧开发记录和新Mock采集 |

`preflight.mapping/timing`绑定方向/映射和时序预检报告。当前示例绑定既有合成预检，不能当作甲方主机或物理网络验证。冻结协议检查原始捕获摘要不能在训练/验证角色之间复用，角色还须匹配原记录的conditions。之后每次分析重新核对协议、引用、S3原始manifest、设备/固件/模型及14轴顺序；新模型不能直接沿用旧协议。

这些文件摘要建立一致性边界，外部契约的物理真实性仍需现场核验。数据就绪不代表执行权限，所有execution_enabled/hardware_executable保持false。

## 顺序与门禁

1. **预检**：没有通过的映射和时序证据则不调用拟合。实机物理方向/安装零位仍不能由合成编码器检查代替。
2. **重力**：多构型正反接近配对。检查配对身份、声明窗口内实际静止，使用原有140参数物理一致求解；独立保持及配对残差分别验收。不声称逐连杆质量/质心唯一。
3. **运动摩擦**：位置匹配的正反匀速半差，多个速度分离黏滞/库仑项。继承S3的来源、时钟、原始4kHz门禁，并要求每轴独立更密采样实验及参数收敛；同频记录改标签不能作更密对照。此入口不从位置响应推电流环时间常数，也不把旧Mock的tau条件当厂家通用要求。
4. **起动摩擦**：冻结重力，消费真实反馈保护事件，回溯最后静止/最早运动的力矩区间。保留较晚确认时刻、噪声区间和独立重复误差；它是操作性阈值，不是材料的最大静摩擦常数。
5. **前序验收冻结**：保存三阶段报告摘要；任一阶段失败则不进入惯量。
6. **惯量**：S3的真实采样时刻/同频带周期投影、Pinocchio回归、独立非换向腕部探测。复用现有PSD伪惯性约束、重力等式约束和预先限定的摩擦修正。之后重新验收重力、运动摩擦和起动阶段的重力变化，并在独立运动记录上验收预测。
7. **控制**：只有被接受的候选才允许在声明的本机Mock执行五档PD选择和四组“原/辨识模型×原/优化PD”对照。选择轨迹与验收轨迹分开，固定PD前馈收益单列。实机控制要另接经确认的执行模式和停止路径。

当前非周期数值核需要整条连续、均匀、位置/力矩同步采样的记录；S3虽可允许≤0.1ms配对误差，S4非周期入口暂不插值或猜测重采样。存在缺帧、非均匀时间或异步配对时明确拒绝；后续可以增加经过独立验证的匹配处理。周期入口直接复用S3的分时钟观测投影。S3政策v2新增inertia_probe的数据资格（至少1s、实际原始≥4kHz）；不把短探测误写成满足12s完整周期要求。

## 运行方式

```bash
# 首次现场尚无合格输入：形成可审阅的阻断报告，返回1是预期
bash scripts/humble/container.sh casbot-calib vendor-plan-staged \
  --spec configs/vendor_s4_unknown.yaml --output artifacts/my_s4_pending
# 证据齐备后冻结自己的spec，输出目录须为新目录
bash scripts/humble/container.sh casbot-calib vendor-plan-staged \
  --spec /path/to/s4_spec.yaml --output artifacts/my_s4_protocol
bash scripts/humble/container.sh casbot-calib vendor-inspect-staged \
  artifacts/my_s4_protocol --output artifacts/my_s4_inspection
bash scripts/humble/container.sh casbot-calib vendor-identify-staged \
  artifacts/my_s4_protocol --through inertia --output artifacts/my_s4_analysis
```

分析按阶段写报告，即使失败也保存analysis.json和manifest。阶段通过、前序检查、模型候选和输入角色都有明确绑定。analysis.passed只表示请求的离线阶段通过；independent_final_test_passed和control_validated默认false，不能据此声称新的最终test或控制收益。

## 本轮合成实验

原生采集脚本`collect_vendor_s4_mock.py`只使用现有C++物理/保护/碰撞代码：从旧schema1复制并核验未改运动参考，显式把新起动观测设为4kHz，在新目录采集56条（14轴×正反×训练/验证）。两批使用不同姿态、静置和斜率；本轮没有全因子采集，不能分别辨识姿态、静置和斜率效应。旧1kHz起动记录保留，不升格使用。

`prepare_vendor_s4.py`只转换旧schema1中已核验的重力64条、运动摩擦210条（含8kHz独立采样对照）、惯量4条和腕部探测12条，再加新起动56条，共346条。旧train/validation明确标为**复用开发记录**，不读取P0的R4正式test、不把这些旧验证当新留出。合成尺度参考仍不是实机增益测量。

```bash
bash scripts/humble/container.sh python scripts/humble/collect_vendor_s4_mock.py \
  --source artifacts/staged_release --output artifacts/my_s4_native
bash scripts/humble/container.sh python scripts/humble/prepare_vendor_s4.py \
  --source artifacts/staged_release --native artifacts/my_s4_native --output artifacts/my_s4_inputs
bash scripts/humble/container.sh casbot-calib vendor-identify-staged \
  artifacts/my_s4_inputs/protocol --output artifacts/my_s4_analysis
# 前一分析接受惯量后，重新采集九条本机控制记录
bash scripts/humble/container.sh python scripts/humble/collect_vendor_s4_mock.py \
  --source artifacts/staged_release --output artifacts/my_s4_native \
  --candidate-analysis artifacts/my_s4_analysis
```

采集脚本可读取明确的合成plant配置以产生数据；拟合/复核进程禁止读取evaluation。控制脚本不会resume或覆盖已有运行。若需重做，使用新目录并重新形成输入绑定。

上述复用命令要求本机既有`artifacts/staged_release`，新检出仓库不保证存在；缺少时先按原分阶段流程取得来源，不用当前拟合结果冒充历史记录。

## 2026-10-29本机验收

四阶段及惯量后前序复核通过；346条记录全部通过对应S3资格。新采集起动56条及控制9条，旧开发输入290条明确保留来源。控制结果为：

| 模型/增益 | 独立轨迹RMSE（mrad） |
| --- | ---: |
| 原模型/原PD | 3.88414 |
| 原模型/优化PD | 2.84456 |
| 辨识模型/原PD | 0.89316 |
| 辨识模型/优化PD | 0.88946 |

固定PD前馈下降77.00%，辨识模型下额外调PD只下降约0.41%；分别保留两种贡献，不把调参收益叠成标定收益。全部为当前合成条件，不承诺甲方设备收益。

最终359项Python回归通过，其中17项S4专项；14条已有Matplotlib/Pyparsing弃用警告。十项独立证据复核通过，四组控制从原始记录重评分；禁止打开evaluation的新进程完整重分析，刚体/黏滞/库仑/起动参数最大差均为0。三个未知能力CLI调用按预期返回1并生成阻断报告。

证据路径：

- 输入与协议：`artifacts/vendor_s4_inputs_v1/protocol_v2`。
- 初次分析：`artifacts/vendor_s4_analysis_v1`；新采集：`artifacts/vendor_s4_native_v1`。
- 独立复核与中文图：`artifacts/vendor_s4_verification_v1/verification.json`、`summary_zh.png`。
- 测试：`artifacts/vendor_s4_tests/full_v3.xml`；交付汇总：`artifacts/vendor_s4_release_v1/delivery.json`。

首次prepare在冻结协议时拒绝旧`left_validation/right_validation`角色名，尚未开始拟合；原spec和失败日志保留在inputs目录。修正为显式验证角色映射后，另写spec_v2和protocol_v2；原始数据、旧协议及正式P0测试均不回写。初次analysis保持其原始`control_validated=false`，后来控制另以`s4_control_binding.json`和独立交付报告绑定，不能回写已封存分析。

本轮没有修改C++物理/积分内核，也没有重跑或将历史114项冻结数值对照称为本轮新结果。MuJoCo/ROS viewer未参与力学或碰撞计算。尚未完成新的硬件或正式P0最终留出验收。

## 实机还缺什么

S4软件管线不能补出SDK没有声明的能力：仍需明确effort总输出含义/单位/尺度、原始采样和滤波、故障/饱和、实际写入/采用历史、上身力矩及内部补偿语义和安全停止；还需真实模型、夹具安装/重力、负载及限制。具备这些条件后，先产生新的硬件S3/S4协议，完成现场路径和停止验证，再接执行端；不能用合成通过状态解锁S2。
