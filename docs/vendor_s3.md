# S3：动力学观测资格与力矩数据入口

S3已实现离线数据入口和逐阶段资格检查：S1反馈或主机CSV/NPZ → 保留原字段与来源 → 显式换算输出侧力矩 → 检查尺度、采样时钟、状态及原始采样率 → 为合格周期记录构建Pinocchio回归问题。它不连接设备、不发布命令、不拟合机械参数，也不开放实机实验。

S1原始记录及保守配置不修改；S2位置执行仍限定本机私有Mock端点。[S4](vendor_s4.md)已接入离线分阶段拟合、前序复核和新的本机控制对照；实机实验协议/执行和现场独立验收仍待设备条件齐备。现有合成辨识算法继续保留。

2026-10-30[厂家答复](sdk_hardware_prerequisites.md#厂家答复记录2026-10-30同步)已说明effort为模组估算Nm、无独立扭矩传感器，但输出侧/尺度/滤波等仍待核验；debug反馈1000Hz只是声明的发布率，可能重复缓存，stamp为发布时间，当前无可用高频日志。此类stamp不能填入下述acquisition字段，当前记录不自动取得任何阶段资格。静态重力可优先准备，但静态缓存替代观测尚未实现/验证；动态4kHz等既有政策不变，不以低通、数值去重或配置改值绕过。以上为用户转述的厂家说明，不是本机实测。

## 数据和证据

新增模块为`dynamics_data.py`、`dynamics_qualification.py`、`dynamics_problem.py`。规范化观测使用`casbot_dynamics_observations_v1`，与旧Mock的`current`输入分开，保存：

| 字段 | 约定 |
| --- | --- |
| `q`, `dq` | 14轴输出侧位置rad、速度rad/s，顺序显式绑定 |
| `signal_raw`, `torque_nm` | 原始信号与有依据转换后的总执行器输出力矩；未知转换得到NaN |
| `position_time_ns`, `signal_time_ns` | 各自真实采样时刻和时钟域；未知为−1，禁止用回调顺序补造 |
| `receive_ns`, `device_sequence` | 原始接收时刻与设备序号；缺少设备序号保持−1 |
| `status_raw`, `temperature_raw` | 每轴状态标志和温度；缺少保持NaN。状态0表示本契约下无故障、无饱和 |

主机CSV必须有表头；每个向量字段显式列出14列。NPZ使用明确的数组键。时间和序号必须是整数，时间单位固定为纳秒；浮点秒不会猜测转换。标准入口不重排关节、不排序时间、不重新编号。若厂家状态编码不同，应先以另存转换结果和来源契约明确规范化含义，不能把未知状态填0。

Profile使用`casbot_vendor_s3_profile_v1`，包含`scope`、`contract`、`scale_validation`、`clock_mapping`、`requested_band_hz`。三个证据引用均为`{path: ..., sha256: ...}`，相对路径按profile目录解析；可空，但缺证据会阻断相关资格。[默认配置](../configs/vendor_s3_unknown.yaml)全部保留未知。完整可运行的合成示例由下方验证脚本产生，不能直接改scope后当作实机证据。

观测契约`casbot_observation_contract_v1`至少明确以下内容：

- `scope`为`synthetic_observations`或`hardware_observations`；`source_sha256`绑定原始NPZ/CSV，S1入口则绑定其`observations.npz`。`joint_names`、`model_id`绑定同一14轴模型。
- `identity`记录数据批次`dataset_id`、URDF的`model_sha256`、机器人序列号、固件、运控版本和来源说明`provenance_reference`。nominal与evt6不得混用。
- `fields`明确`position/velocity/signal/receive/device_sequence/status/temperature`的键或列，缺失可选字段为null；`position_unit=rad`、`velocity_unit=rad/s`。
- `position_time`和`signal_time`各含`field`、`role=acquisition`、`clock_id`。发布时刻、ROS header和接收时刻不自动晋升为采样时刻。
- `signal`明确`origin`、`unit`、`side=output_equivalent`、`includes=total_actuator_output`、14轴`gain_nm_per_raw_unit`与`zero_raw`、`sensor_identity`、`raw_acquisition`及`filter.kind`。

支持的信号来源是`motor_current`（A）、`output_torque_estimate`（Nm）和`independent_output_torque`（Nm）。转换为`(signal_raw-zero_raw)*gain`。输出侧增益已包含减速器效果，禁止再提供减速比乘一次。指令echo、未知来源或扣除了未说明补偿的信号不能直接进入总力矩方程。驱动估算力矩仍不算独立力传感器。

独立尺度记录`casbot_scale_validation_v1`绑定同一scope、关节、model_id、sensor_identity及序列号/固件。它提供至少四组正反向`raw_delta`与`reference_delta_nm`、独立零位`reference_zero_raw`和`zero_uncertainty_raw`。参考类型为已知载荷、已知附加惯量或独立传感器；合成例子另标`synthetic_reference`，不能用于实机资格。当前检查固定增益的相对RMSE≤5%、参考RMS≥0.2Nm、含零位不确定度的误差≤0.01Nm；不在这里重新拟合增益，也不以同一电流自证尺度。

**摘要检查只能验证文件一致性，不能证明厂家字段的物理含义或外部参考真实性。** 实机证据仍需针对具体设备/固件审核，报告保留`physical_semantics_verified_by_software=false`。本机Mock的S1捕获不能被升级为hardware观测契约。

## 时钟、缺帧和阶段门禁

同一采样时钟直接检查位置/力矩配对误差。不同钟域必须提供`casbot_clock_mapping_v1`：源/目标clock_id、整数时间原点、正的`rate_ratio`、不确定度、有效源时间区间，以及至少三对覆盖整段记录的独立验证时刻。采用`target_origin + round((source-source_origin)*rate_ratio)`，先做整数减法，避免大纪元浮点损失。映射验证误差加不确定度及逐行配对误差须≤0.1ms。原始两路时刻保留，映射结果另存；不从RTT推单向延迟。

有效数据必须同时具备有限q/dq/力矩、明确的无饱和/故障状态及合格采样时刻。时间倒退、重复、超过中位间隔1.5倍的缺口、已知设备序号不连续及无效状态都会切段。不会跨洞拼接求导。无设备seq时可按已知采样时刻检查，但明确不能统计设备序号丢失，更不能把源采样缺口叫网络丢包。

| 阶段 | 当前观测资格要求（还须通过来源、尺度、时钟和身份检查） |
| --- | --- |
| 重力 | ≥0.5s静态窗口，速度<0.002rad/s、位置全范围≤0.0005rad，明确无滤波；允许较低采样率 |
| 运动摩擦 | ≥1s连续有效原始记录，实际采样≥4kHz，明确无滤波 |
| 起动摩擦 | ≥0.5s连续有效原始记录，实际采样≥4kHz，明确无滤波 |
| 惯量 | ≥12s连续有效原始记录，实际采样≥4kHz，明确无滤波；声明的整个稳态周期窗口不能穿过缺口 |

动态阶段还检查所请求频带能被采样解析。4kHz来自本项目原生Mock摩擦混叠的既有边界，是当前保守软件政策，不是所有机器人通用的物理常数。已知滤波也不能暂时绕过：尚无该响应的匹配处理；低频周期平均替代方案需独立验证后另加协议。温度缺失会明示，但本轮不把有温度记录解释为热漂已验收。

S4扩展的政策v2另提供`inertia_probe`数据资格：至少1s、原始≥4kHz，并保留相同来源/时钟/状态门禁。具体腕部处理使用30Hz匹配滤波时，还必须明确取得至少30Hz的频带资格；不能把短探测算作12s完整周期。

**500Hz仍是通用Mock的命令/控制/反馈发布默认频率及SDK客户端默认发送计划。** 厂家声明的位置插值1000Hz与debug反馈发布1000Hz单独记录，不改历史条件。一般情况下低频ROS反馈不妨碍另存合格4kHz主机原始记录，但当前厂家已说明无可用高频日志，不能假定已有此替代路径；只收到缓存effort发布不满足动态政策。实验设计是否充分、静摩擦是否真的触发、前序是否验收通过和控制权限均为独立门禁；本报告只声明`observations_eligible`，所有`execution_enabled`始终false。

电流环响应/纯指令延迟辨识也保持阻断：仅有反馈记录不足以取得实际写入历史，不能把整体运动响应全部解释成网络或驱动延迟。

## 周期观测进入统一刚体模型

惯量记录还需在契约中声明`experiment`：`kind=periodic`、共同采样时钟下的`begin_ns/end_ns`、`period_s`、`discard_periods`、`harmonics`和`phase_points`。默认丢弃两个起始周期。傅里叶拟合仅使用同一完整有效段内的实际采样时刻，对q解析求导；力矩用其自己的合格时刻投影到同一相位。拟合频带不能超过资格报告频带。

回归量由现有Pinocchio计算，标准140列；转子贡献只扣一次。原始实测速度先生成非线性摩擦特征，再匹配同一傅里叶频带。输出`regression/friction_basis/rotor/target/torque_nm/q/dq/ddq`，其中target直接使用Nm力矩，不新增假的current字段。SVD记录实际秩，不把140个连杆参数全部称为可独立辨识。

当前桥接只接受已绑定的nominal几何/顺序；新实机URDF到位后需按模型摘要接入并重新验路径。回归问题是既有`rigid_identification.fit_rigid`的输入边界，尚未调用求解器或绕过分阶段先验/物理一致性验收。低速摩擦特征也不能替代起动摩擦的操作阈值。

## 运行

在仓库根目录执行，所有输出必须为新目录。以下入口纯离线，不需要加载ROS SDK overlay：

```bash
# S1规范化记录：未知信息仍保留未知
bash scripts/humble/container.sh casbot-calib vendor-import-dynamics \
  artifacts/my_s1_normalized --source-type vendor \
  --profile configs/vendor_s3_unknown.yaml --output artifacts/my_s3/canonical
# 有明确来源、尺度与采样证据的主机CSV/NPZ
bash scripts/humble/container.sh casbot-calib vendor-import-dynamics \
  /path/to/host_log.npz --source-type host \
  --profile /path/to/verified_profile.yaml --output artifacts/my_host_s3/canonical
bash scripts/humble/container.sh casbot-calib vendor-qualify-dynamics \
  artifacts/my_host_s3/canonical --output artifacts/my_host_s3/qualification
bash scripts/humble/container.sh casbot-calib vendor-dynamics-problem \
  artifacts/my_host_s3/qualification --output artifacts/my_host_s3/problem
```

资格命令在所有阶段被阻断时返回1，同时保存具体原因，这属于预期拒绝。未知SDK记录不能继续运行最后的回归问题命令。每层输出JSON/NPZ/manifest，资格另有CSV；原始输入不回写，重分析不覆盖旧目录。

## 本机验证与后续条件

验证脚本只读取旧schema1合成会话中摘要核验过的两条train观测；不读取P0 R4正式test，不选参、不拟合新机械参数：

```bash
bash scripts/humble/container.sh python scripts/humble/verify_vendor_s3.py \
  --staged-source artifacts/staged_release \
  --vendor-source artifacts/vendor_s2_release_v3/normal_500_0/analysis/observations \
  --output artifacts/my_vendor_s3
```

本机证据位于`artifacts/vendor_s3_release_v2/verification.json`、`qualification_matrix.csv/png`。检查内容包括4kHz原生Mock电流、直接Nm估算力矩、1kHz静态CSV、未知SDK反馈及来源/尺度/滤波/时钟/缺帧反例。合成电流单位参考不算实机输出增益验证。新进程安装审计钩子，拒绝打开任何`evaluation/`路径后仍可构建回归问题。

2026-10-29实际验收：20项离线检查通过、三个CLI调用的通过/预期拒绝退出码正确；26项新增测试及全量342项Python测试通过（14条已有Matplotlib/Pyparsing弃用警告）。旧train记录生成的新/旧Pinocchio回归量最大差2.62e-12、力矩最大差2.67e-14；该条轨迹的缩放回归秩90/140，不声称唯一恢复全部连杆参数。测试XML为`artifacts/vendor_s3_tests/full.xml`，汇总为`artifacts/vendor_s3_release_v2/delivery.json`。本轮未改变C++机械内核，未重跑采集或把旧114项冻结数值对照当成本轮新结果。

首次`vendor_s3_release_v1`的总体false保留：新增真值隔离脚本在审计回调内导入pathlib导致递归导入失败；修正为安装钩子前导入，并主动验证钩子能拒绝访问后，在新目录完整重跑。它不是机械或SDK数据通过的替代证据。

继续S4/实机前优先取得：effort是指令还是总输出估算力矩、输出单位/符号/尺度独立参考、采样时刻事件和钟域、真实原始采样/滤波记录、饱和/故障状态，以及实际执行历史。另需真实URDF、安装重力、负载、设备限制、控制模式和物理停止/碰撞条件。S3不会为了继续流程而给这些未知项填默认值。
