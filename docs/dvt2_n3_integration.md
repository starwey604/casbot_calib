# N3：让各条流程使用同一个DVT2模型

N2已提交为`cfe855e`，包含实际1105_03装配、八组分段进退参考及其验证工具。N3从共享模型入口开始，再迁移消费者，最后一次切换默认值。

**当前已接入共享上下文、动力学/SDK消费者，以及实际四球装配的K2–K4，可用`configs/dvt2/`显式演练。默认CLI尚未切换，P0新协议消费者与N4仍待完成。** 开发验证使用本机SDK Mock或原生虚拟时间Mock，不连接实物。

## 用上下文获取模型，而不是重新指定一份URDF

```python
from casbot_calib.model_context import load_context

context = load_context()
engine = context.engine
home = context.home
parameters = context.transport_parameters()
```

`load_context()`只加载随包的`bare_flange_1105_03_gauge_v2`，不搜索下载目录、不回退到旧nominal。上下文提供：

- 固定腰部、双臂14轴的模型、关节顺序、home及重力条件。
- 源URDF与冻结包身份；后续记录和候选必须绑定这些身份。
- 裸法兰机器人碰撞场景，或左/右换装的实际三表工具与球架场景。
- 实际四球和CAD测量轴构成的K1观测配置。
- ROS节点所需的`model_path`、`model_id`和`joint_names`；不包含连接地址或运动授权。

每次获取`engine`返回独立实例，候选惯性或摩擦变化不会修改共享名义模型。碰撞资产可以复用于几何未变的惯性候选；几何改变时必须重新建立和验证资产。

### 动力学和运动学共用源模型

```python
from casbot_calib.dual_model import nominal_dual_arm
from casbot_calib.kinematics import nominal_kinematics

nominal, source = nominal_dual_arm(context=context)
geometry, manifest = nominal_kinematics(context=context)
profile = context.collision_profile()
```

K1配置直接使用源模型已有的两个TCP，不重复添加frame。四球为原装配A/B/C/D；CAD轴向来自实际装配。表的零位/量程仍为合成配置，球架注册不是现场测量，工具负载未知。该配置没有把旧六球K3数据改名为新数据。

### 候选导出保留源资源与坐标约定

- `export_dynamics_urdf(..., context=context)`和`staged_control.load_candidate(..., context=context)`检查DVT2源身份。导出sidecar保存上下文；`load_exported_dynamics()`和`verify_dynamics_export()`能据此恢复同一来源，不要求调用方再次猜模型路径。
- `kinematic_export.export_urdf(..., context=context)`使用DVT2源URDF；配置已有`model_context`时可自动解析。已有TCP不重复写入，修改关节origin时保留连杆惯性坐标约定。
- 两类导出都重定位网格/纹理引用。候选放入新目录后仍能找到原资源；检查比较解析后的资源对象，不把必要的相对路径变化当成几何变化，也不忽略真正的资源替换。

导出URDF引用随包模型资源，**不是独立网格分发包**。迁移候选到另一台机器时，需要安装同源模型并重新处理资源路径。摩擦和armature继续由动力学sidecar保存，不伪装为URDF通用参数。候选重加载检查序列化与原生算子一致；参数精度由相应独立实验验收。

### 显示不增加一套机械计算

`vendor_viewer.scene_for_vendor(config, output, context=context)`严格检查模型ID和关节顺序，复用冻结CAD场景。无`active_tool`时显示裸法兰整机；指定`left`或`right`时显示对应三表和台架。沿用同一`StateFrame`和`JointView`，MuJoCo仅更新几何。

原生模拟继续使用C++ `ArmSimulationRunner`。SDK Mock已通过同一上下文生成服务端参数和客户端profile，正常使用不需要分别指定两份URDF；具体启动方法见下方第二轮入口。

## 复现实验

在仓库根目录执行，输出目录必须不存在：

```bash
bash scripts/humble/container.sh python scripts/humble/verify_model_context.py \
  --output artifacts/my_n3_integration

bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
  tests/test_model_context.py tests/test_dynamic_export.py tests/test_kinematics_k1.py
```

脚本输出`report.json`、原生Mock观测NPZ、两类合成候选URDF及只读PNG。它用精确重力前馈做10ms保持，随后测试人工指定的候选参数，不是新一批参数辨识或碰撞放行。MuJoCo的step/forward/inverse/collision/rne接口在验证期间被禁止调用。

本机`artifacts/dvt2_n3_integration_v3/report.json`六项检查通过：155个原生frame、60维几何参数图、动力学候选32组随机姿态重加载、原生Mock保持以及显示姿态一致。保持误差约`5.68e-18rad`，几何重加载frame误差约`8.88e-16`，显示与原生FK误差约`2.22e-16`；已查看PNG。45项上下文/导出/K1测试通过，含禁止导入Python Pinocchio/Coal的独立运行进程。运行时计算仍使用原生Pinocchio/Coal；导出核验可使用其Python绑定。旧K3/K4导出消费者另有28项回归通过，1项可选渲染测试因未开启环境开关跳过；本轮新场景已由上述集成脚本实际渲染。后补3项针对性检查通过，覆盖新K1元数据、替换网格拒绝以及缺失TCP的兼容导出。

初次验证脚本使用了错误的力矩限制类型、随后使用了错误的viewer body命名；已修正为标量限制和按关节查询body。对应失败日志保留，v3为有效结果。

## 后续按消费者迁移，再切默认

1. **N3第二轮（已接入）：动力学与SDK消费者。** 已将schema 1 staged、H1–H4、SDK Mock启动参数和客户端profile绑定到上下文，更新home、场景及激励配置，保持同一采集器、500Hz发送及SDK五字段语义。新模型重新规划/检查，不复用旧碰撞证书或参数估计。
2. **N3第三轮（已接入）：K2–K4观测与辨识。** 消费N2冻结进退参考，接入实际四球、一个工具左右依次换装、脱离后转向规则、对中与稳定读表；重新做可辨识性检查和独立验证。旧六球/同时双工具假设不能沿用。
3. **P0协议消费者。** 在默认切换前迁移schema 2的模型/场景与角色协议，重新生成DVT2参考和开发观测；旧P0正式test不能改绑新模型。
4. **原子切换与N4。** 各消费者及配置准备齐后，一次切换默认入口，移除旧运行资产/fallback，再做新模型的安装、动力学/运动学、ROS及渲染验收。旧实验结果保留历史身份，不当作新模型证据。

自由测杆外形、实际工具负载、安装计量、设备限制及采样资格仍按原清单现场核验；它们不阻止本轮离线接口开发，但限制相应实机执行与正式标定资格。

## 第二轮：接入动力学采集与SDK Mock（2026-11-06）

现在可以显式选择`configs/dvt2/`演练新模型。`model_runtime.py`从冻结配置统一解析模型、源URDF和碰撞场景，以下消费者已接入：

- SDK Mock服务端参数和生成的客户端profile；源身份不一致时，在启动进程前拒绝。
- H1快照、方向/频率参考、重力前馈、执行和接收轨迹检查。
- H2静态选点、批次规划/重加载及重力候选拟合和导出。
- H3重力候选加载、保持增益、正反匀速与零PD起动参考。
- H4前序候选、惯量激励、完整前馈CSV、候选导出及控制参考。
- schema 1的staged规划、原生模拟采集、辨识、控制、验证和回放。

新原生采集记录保存`model_context`，读取时与会话核对；不允许将旧模型记录改放进新会话。H3/H4搬运重力候选时会重新定位URDF资源并封存目标目录，避免复制后网格路径失效。SDK原始录制、规范化与批次回放保留DVT2身份；显示使用全身CAD和相应构图，不再沿用旧上身模型的相机高度。

### 启动同一SDK客户端可用的新Mock

在仓库根目录、Humble环境及SDK可选包构建完成后执行：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh casbot-calib sdk-mock \
  --config configs/dvt2/sdk_mock.yaml --profile configs/dvt2/hardware_field.yaml \
  --output artifacts/my_dvt2_mock --domain-id 227
```

H1–H4后续命令的`--profile`使用生成的`artifacts/my_dvt2_mock/client_profile.yaml`。H3和H4的`--config`分别使用`configs/dvt2/hardware_friction.yaml`、`configs/dvt2/hardware_inertia.yaml`。H2的方向/频率组织策略仍可使用原`configs/hardware_batch.yaml`，它不定义机器人几何。

客户端默认发送500Hz；服务端保持既有1000Hz位置参考和反馈发布、3ms合成缓存刷新及2ms插值假设。反馈仍无采样时间、设备序号和原始电流。五字段、本地PD、故障终止与私有ROS端点规则均沿用原SDK路径，没有增加第二套采集器。

本轮动力学场景为**裸法兰整机与地面**，没有三表工具/台架，也没有实测腰部夹具。模型上下文绑定该场景，不能往配置里添加障碍后仍复用旧碰撞资产；新增场景须按同一资源生成路径重新建立。实机安装和末端负载仍按现场流程核验。

### 离线规划仍走原入口

```bash
# 静态选点：双臂各24组训练、8组验证，正反两次接近
bash scripts/humble/container.sh casbot-calib plan-static-field \
  --config configs/dvt2/static_field.yaml --output artifacts/my_dvt2_static

# 完整schema 1参考：方向/时序→重力→运动摩擦→起动→惯量→控制
bash scripts/humble/container.sh casbot-calib plan-staged \
  --config configs/dvt2/staged_dual_arm.yaml --output artifacts/my_dvt2_staged

# 在上述已规划会话内开始模拟采集，只推进到重力验收
bash scripts/humble/container.sh casbot-calib staged-calibrate \
  --config configs/dvt2/staged_dual_arm.yaml --output artifacts/my_dvt2_staged \
  --through gravity --resume
```

新配置保留原扫描角度、实验顺序、独立角色和验收阈值，由新模型重新检查；没有通过缩小所有运动或降低10mm余量来放行。`configs/dvt2/`是N3准备期的显式入口，默认CLI尚未切换。DVT2的P0 schema 2协议迁移仍待完成，当前会明确拒绝该组合；旧P0记录与正式test继续保留原模型身份。

### 本轮证据

汇总入口：`artifacts/dvt2_n3_dynamics_delivery_v1/delivery.json`。

- `artifacts/dvt2_n3_staged_plan_v2`：764条完整参考通过新场景门禁，约5,054秒虚拟时长，默认500Hz。v1完成轨迹检查后因新碰撞元数据缺少`backend`字段停止；已补为实际原生Coal版本，v2重新生成完整会话。
- `artifacts/dvt2_n3_staged_plan_v2`的重力checkpoint：14轴方向、3条时序和64条重力记录，共81条实际原生采集通过；重力独立验证最大轴RMSE约`2.47e-5Nm`，回归秩28/140，单体质量/质心仍是可观组合的物理代表。`artifacts/dvt2_n3_gravity_reanalysis_v1`在新进程禁止读取`evaluation/`后重分析通过，参数差0。
- `artifacts/dvt2_n3_static_plan_v1`：64个姿态、128条正反参考，左右臂各24训练/8验证；`dvt2_n3_static_verify_v1`独立复核通过。该产物只规划参考，不包含参数辨识。
- `artifacts/dvt2_n3_sdk_ros_v1`：同一SDK服务端/客户端实际完成左右肩轴各100/250/500Hz，共六条往返；每侧各频率复用同一参考。请求频率、实际发布/接收频率及整体响应滞后分别记录，不从中推断设备采样频率或单向网络延迟。
- `artifacts/dvt2_n3_sdk_view_inputs_v1`与`dvt2_n3_sdk_view_v2`：正常SDK录制规范化7,081条位置，中文只读视频44帧、1280×720；已完整解码并查看实际画面，显示与原生FK最大差约`6.66e-16`。v1显示沿用了旧相机高度，已修正为整机视野。
- 单元/回归日志：`dvt2_n3_sdk_regression_v1.log`为69项原H1–H4/SDK测试；`dvt2_n3_dynamics_regression_v1.log`为45项相关回归（1项可选渲染跳过）；`dvt2_n3_runtime_tests_v3.log`为6项新模型消费者检查，另有`dvt2_n3_collection_contract_v1.log`覆盖真实C++采集、模型隔离与原始记录重读。`dvt2_n3_p0_boundary_v1.log`中44项原P0协议回归通过，新拒绝用例暴露最小配置先访问`p0`的问题；已收敛公共边界检查，`dvt2_n3_p0_boundary_v2.log`中该用例通过。H3左右匀速/起动、H4激励和完整前馈均由正常规划函数生成；H2候选导出测试使用声明的合成均值观测，staged重力checkpoint使用上述实际模拟采集。

实际装配K2–K4已按下方第三轮接入；下一轮迁移P0协议消费者，再统一切默认并进行N4完整新模型复验。整批H3/H4 SDK采集和新模型惯量/控制验收还需在N4执行，六条频率往返不替代它们。


## 第三轮：实际四球、单工具换装与K2–K4（2026-11-06）

入口为`configs/dvt2/kinematics.yaml`，完整命令、参数预算和相位规则见[实际四球装配指南](dvt2_trical_integration.md)。台架孔位/高度、左90°右0°、10/2/5mm规则沿用N2；不需要重新搜索装配。`dvt2_trical.py`只负责实际装配配置、路线与候选设计；新旧场景复用`MeasurementController.center()`、原生持续Mock、观测格式、MDH/Hayati辨识和K4验收，不另造采集/机械内核。

- **路线**：消费冻结八组进退参考。接触区保持1°包络；左D/右C在三个测头均达到正间隙后才转向。每次访问回共同home，左右换装前检查已送达位置/速度/状态/新鲜度，记录同一工具的人工换装边界。仪表故障只在当前已知相位内尝试保持并锁定人工处理，不走未经证明的直线退出。
- **不确定性与计算**：参考关节管仍为±0.00105rad，显式分成编码器50µrad与跟踪1mrad；刚体误差另加0.1mm位移、旋转预算0。Mock工厂检查两侧装配真值是否在声明族内。执行和重分析可消费同几何/阈值/端点的封存参考证明，但完整原始观测折线及全部内部点包络、朝向与稳定窗口每次独立复核。接触弦段细分后重算既有N2局部运动上界，不降低间隙。
- **选点与辨识**：四球两侧各2训练+1验证，共16/8，40个候选；按工具/球组合做训练信息选点。验证独立随机种子生成，设计间隔0.012rad，实际最近训练/验证距离0.02067rad。原有辨识的0.01rad分离、条件数200和20µm门槛保持。
- **重复性**：K4在首批训练数据上冻结31列参数子集，另采两批，只改变仪表/驱动噪声种子；统一用原独立验证批次。新进程移走evaluation后完成三组拟合。这里只验同一工装/同一物理参数条件下的重复性，未验外部计量、换装/热漂移或实机AP/RP。

### 有效证据

| 产物 | 结果 |
| --- | --- |
| `artifacts/dvt2_n3_k3_plan_v3` | 八组CAD路线、40候选、16训练/8验证；60维图中训练设计选出31列 |
| `artifacts/dvt2_n3_k3_calibration_v3` | 16+8次原生采集、实际原始路径/稳定窗口复核通过；几何层条件数190.37，31/60可辨识代表 |
| 同目录`identification/report.json` | 独立球心约束RMSE：名义40.25µm→零位12.25µm→几何0.879µm；候选URDF重加载位姿误差7.77e-16、质量矩阵差1.11e-16 |
| `artifacts/dvt2_n3_k4_v1/analysis/report.json` | 三批几何约束RMSE分别0.879/1.137/0.970µm；选中参数最大标准差0.02775（阈值0.15），公共验证TCP预测标准差最大0.856µm（阈值20µm） |
| `artifacts/dvt2_n3_k2_allocated_v1`、`dvt2_n3_k2_reference_reuse_verify_v1` | 分配预算后的首测点采集及封存参考复用+独立原始路径重分析通过 |
| `artifacts/dvt2_n3_k2_fault_v1` | 仪表失联按预期拒绝；未自动直线退出或回home，要求人工处理 |

`dvt2_n3_kin_regression_v1.log`：47项原K2/K3/K4回归通过，1项可选渲染跳过；`dvt2_n3_kin_contract_v5.log`：9项新装配检查通过，含60维参数导数的独立有限差分、左右几何预算、单工具场景及原始路径内部点/缺帧；`dvt2_n3_kin_cache_guard_v1.log`补查非静止端点运动不得借用弦段证明。实际新场景由K4另行渲染，不使用该跳过项充当显示证据。

汇总入口为`artifacts/dvt2_n3_kinematics_delivery_v1/delivery.json`。中文视频在`artifacts/dvt2_n3_k4_v1/presentation/kinematic_workflow.mp4`，1280×720、12fps、66秒/792帧，已完整解码并查看实际左D/右C释放转向、回位单工具换装及结果画面。显示与原生TCP最大位姿差1.33e-15；MuJoCo不推进物理。视频中的微米指标来自离线拟合，显示仍使用冻结名义几何，不放大真实关节角。

### 试算与修正记录

`k3_plan_v1`的整段保守界耗尽预算，改用原证明器按小弦段重算局部界；没有减小碰撞余量。`plan_v2`的右C验证候选不足，改为上述设计间隔并使用保留完整不确定性盒的IK限位余量，辨识验收门槛不变。两份失败计划均未放行采集。`k3_calibration_v2`在发现原始路径重复叠加完整跟踪管且在线重复CAD证明的效率问题后主动中断；保留日志和未封存数据，不续采、不作为验收。v3在新采集前记录`capture_protocol.json`，参考总管未变；K4两批沿用该协议，不改写旧批次预算。

本轮没有接入实际仪表或SDK运动学薄适配；CAD注册、合成mastering、未知工具负载及自由测杆外形仍限制硬件执行。下一轮是P0 schema 2消费者迁移，之后才做默认原子切换和N4。
