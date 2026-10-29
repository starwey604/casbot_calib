# CASBOT2 SDK源码接入评估与实施顺序

评估日期：2026-10-29。源码为本机相邻仓库`~/codings/casbot2-ros2-sdk`，Git提交`397dc955980956be3ede7376f81a57fafb2b23fe`。本轮审阅接口、客户端及离线测试，并在现有Humble容器做隔离构建；未连接机器人、切换模式或发布运动指令。本文补充[此前PDF评估](casbot02_sdk_assessment.md)，涉及消息结构时以本轮源码为准；实机仍须匹配其实际安装版本。

**后续实现更新：[S1只读接入与统一ROS显示](vendor_s1.md)、[S2本机位置流程与保护](vendor_s2.md)、[S3离线动力学观测资格](vendor_s3.md)已实现。S2仍阻止实机执行，稳定500Hz资格尚未确认；S3支持有来源/尺度/采样证据的主机日志与直接Nm力矩，当前SDK未知反馈仍阻断。下文保留评估时的顺序，S4及实机资格待后续，不把本机结果当作硬件控制／动力学能力验证。**

## 结论

可以沿用当前Ubuntu22.04／ROS2 Humble环境，以**可选C++厂商ROS适配包＋Python离线规范化**接入，不需要再次迁移包管理器。Pinocchio、Coal、Boost和MuJoCo只读viewer保留。SDK提供消息定义和ROS客户端示例，未包含机器人运控服务端、ros2_control硬件插件、实机URDF或仿真主程序；不能把编译SDK等同于获得完整设备后端。

建议优先使用`/upper_body_debug/joint_cmd`控制双臂，普通观测订阅`/joint_states`，模式判断读取`/motion/status`并检查模式服务结果。SDK没有要求以ros2_control接入，故不把新增hardware_interface作为前置条件。若甲方后续交付已有控制器，则复用其执行端。

本轮已经能确认位置命令、混合控制相关字段及条件化的PD设置入口；**还不能确认纯力矩执行、effort的物理来源、上身PD实际生效、高带宽电流、停止行为或500Hz采用能力**。这些条件决定哪些动力学阶段能开放，不能由字段存在推断。

## 源码证据与新旧差异

以下路径相对于SDK根目录；本机源码可通过[SDK目录](../../casbot2-ros2-sdk/)访问。

| 内容 | 源码证据 | 对calib的影响 |
| --- | --- | --- |
| 上身命令 | `crb_ros_msg/msg/UpperJointData.msg` | 扁平`name/position/velocity/effort/kp/kd`，另有`header/time_ref/vel_scale`；旧PDF嵌套`joint`结构不适用 |
| 自定义全身命令／反馈 | `crb_ros_msg/msg/JointStateData.msg`、`examples/docs/dev-guide/motion-control.md` | `/motion/debug/joint_cmd`、`/motion/debug/joint_state`含增益；该反馈只在全身调试更新路径发布，不能默认上身模式也有 |
| 通用实测／输出镜像 | 同上文档、`casbot2_tools/contracts.py` | `/joint_states`是实测通道；`/joint_control`是输出指令镜像，不能当测量或硬件采用回执 |
| 模式入口 | `/motion/upper_body_debug`，`std_srvs/srv/SetBool` | 成功响应与真实模式都要检查；超时意味着转换状态未知，不能继续执行 |
| 模式观测 | `/motion/status`、`/motion/robot_state`，`std_msgs/msg/String` | 文档约10Hz，不承担500Hz反馈或故障响应承诺；`/motion/current_mode`也局限全身调试更新路径 |
| 查询旧模式 | `GetRobotMode.srv`及运动控制文档 | ZERO/STAND/WALK是兼容映射，不能据WALK判定已进入调试 |
| PD数组 | `examples/casbot2_tools/casbot2_tools/contracts.py`的`pd_by_name` | 按name顺序跳过灵巧手，紧凑数组；包含手/腰/头时不能用原始name索引kp/kd |
| PD生效条件 | `debug_mode_params.use_default_kp_kd`，见运动控制文档 | true使用主机默认值，客户端填写不证明生效；false须提供完整非灵巧手增益。反馈验证路径与所用模式需另核对 |
| 命令缓存 | 运动控制文档“全身调试与反馈” | debug缓存可遮蔽标准`/motion/joint_cmd`；同一会话只用一条确定的命令通道 |
| 停止语义 | 同上文档、模式服务定义 | ZERO是阻尼，退出debug是模式切换，都不是通用停止/抱持；文档指出导航断流清零不完整，debug断流行为仍需单独确认 |
| 日志入口 | `crb_ros_msg/action/LogOutData.action` | 只有start_log/result/state定义，未说明可用endpoint、文件位置、采样率和数据字段；可询问甲方是否提供高频主机记录，不能据此宣布4kHz可用 |
| 故障类型 | `FaultRecord.msg`、`FaultSnapshot.msg` | 包含字符串快照，不等于逐轴同步电流/温度/饱和位；不能填入动力学观测缺失项 |

SDK的`examples/config/runtime-contract.json`记录其审阅的运控提交`c3b2902ea53f0267ae3ef12b2fdc178c7162ea54`、消息提交`3228d83f5629727eb2a8440bf9fd46c9f9fcedbf`及23个接口SHA，另有两个历史扩展。此次可核对本地消息与清单一致；未取得对应hl_motion仓库，不能独立核验服务端实现或认定实际机器人正在运行该版本。SDK也明确没有实机版本兼容矩阵。

`time_ref`虽注释为秒，SDK例子设0；其时间基准、排队/覆盖及插值仍未给出。`vel_scale`是比例，不能换算为明确的rad/s限速。现有C++上身例子以10Hz发送、effort为0，Python工具也主要发送位置；这些例子不能证明500Hz或纯力矩控制已验证。

## 能力与阶段对应

把能力分为“源码提供入口”“实机已验证”“未知／未暴露”，未知项默认阻止依赖它的实验。当前SDK中未找到原始关节电流、设备采样序号、指令采用序号／时刻、高频批量采样、逐轴饱和状态或独立力传感器接口。`header.stamp`不自带采样语义和时钟映射；ROS2 Header也没有ROS1式seq。

| 阶段 | 可复用的入口 | 开放前仍需补齐 |
| --- | --- | --- |
| 只读身份／方向映射准备、频率观测、姿态回放 | `/joint_states`＋状态topic | 14轴具名完整性、字段有效性、单位；观测频率只能标为本机接收频率 |
| 小幅方向检查、位置轨迹／TriCal运动 | 上身位置topic | 实际起始姿态、方向／零位、真实模型／限位／夹具、子集语义和停止方案；TriCal另需真实三表 |
| 多姿态重力辨识 | 位置保持＋可信effort或电流 | 输出侧尺度、effort来源及偏置、内部补偿、有效静态窗口；无独立尺度不宣称绝对质量恢复 |
| 多速度运动摩擦 | 连续位置参考＋动态观测 | 实际速度及加速度、力矩带宽与滤波、饱和信息、有效采样时间；500Hz接收电流不自动替代现有4kHz采样资格 |
| 重力补偿起动摩擦 | 尚需确认effort/零PD控制语义 | 能独立施加已知力矩斜坡、关闭/明确PD及内部补偿、主机侧位移/速度保护和断流策略；位置起动试验不能冒充现有力矩斜坡法 |
| 惯量激励与独立预测 | 周期位置参考可以作为激励入口 | 前序验收、输出力矩尺度、频带／时钟／原始记录资格、实测轨迹与碰撞条件 |
| 固定PD前馈收益与调PD | kp/kd/effort字段提供候选入口 | 实际控制方程、增益生效和读回、补偿开关／叠加、控制频率／限幅确认；缺能力不静默丢弃字段 |

若effort被确认是输出侧Nm且量纲/来源可靠，新增**effort观测路径**即可复用刚体回归与优化，不必强行反算current。若只是指令回显或来源未知，则不能用于力学参数辨识。电流换算力矩始终与独立测力区分。

## 对现有仓库的接入设计

### 可选设备包与共享执行边界

建议新增`ros/casbot_calib_vendor/`，依赖`rclcpp`、`sensor_msgs`、`std_msgs`、`std_srvs`、`crb_ros_msg`；不让离线Python包和原生力学核心强制依赖厂商消息。SDK保留相邻仓库，记录固定commit与接口摘要，不复制整套示例或引入SDK的仿真机械计算。后续可增加显式的SDK overlay构建脚本；默认现有Mock构建不变。

在完整14轴层抽取`capabilities / latest_observation / submit / request_mode / stop`边界，真实端使用异步回调和实际单调时间；Mock继续独享虚拟时间`advance`。当前`simulator.hpp`的`JointDevice`是单轴接口且能力默认全开，不能直接作为实机默认；当前`staged_collection.collect_trial`仍直接调用模拟Runner，不能只加topic重映射就变成真实采集。`trical_device.MockDevice`同样需要分离虚拟推进。

新设备会话使用独立schema并保存原始消息／CDR及本地接收时间，规范化文件另存。至少保留具名轴、position/velocity/effort原值、字段可用性、header原值、接收序号、时钟与时间已知标志；缺失current/status/设备seq保持缺失。现有`JointObservation`固定current/status结构和`staged_identification.read_record`均匀采样假设不完整表达这种数据，不能把缺失值填零后伪装成合成schema3。

机械辨识尽量在输入层统一为带来源／资格的输出侧力矩、q/dq及时间有效片段，下游复用Pinocchio回归、Clarabel、分阶段先验和冻结验证；动态非均匀采样和缺帧先单独处理，禁止跨缺口求导。新版本协议与新实机train/selection/test独立登记，不能复用或修改已消耗的P0正式test。

### 双臂与腰部固定的特有问题

源码公开的7+7轴名称与当前nominal链一致，可以按名称映射；仍需实测方向、机械零位、实际轴数和限位。SDK没有交付URDF，旧PDF腕roll限位冲突并未因此关闭。真实夹具、工具、重力方向及碰撞模型仍需替换。

优先只发送明确允许的双臂14轴子集。上身接口还覆盖腰、头和手；若服务端要求整组，不能用零补齐，也不能自动发送被抱死腰部的目标／增益。必须先明确腰部驱动如何处置及未指定轴行为。全身debug会扩大控制范围，不能仅为了拿到PD反馈就自动切换。

执行前从实测姿态规划保持→接近→测量→制动／回位，复用Coal门禁；不使用Mock的home作为实机当前状态。在线保护与确定的主机停止策略另实现，客户端进程异常或物理断网时不能依赖补发退出服务。MuJoCo消费规范化后的实测q，只负责实时显示与回放。

### 频率／延迟的可观测边界

默认请求仍为500Hz，先测试100/250/500Hz，重复批次、随机化顺序；更高频率只作条件允许的对照。C++定时发送整段预生成参考，迟到跳点而非突发追赶，记录实际发送时刻与调度迟到。先采用运行端已提供的Reliable/Volatile/KeepLast(10)作为待核验配置，查询实际endpoint QoS；改QoS前验证匹配和积压，不把本机Mock的latest语义套给厂商。

| 测量 | 当前SDK能否独立提供 |
| --- | --- |
| 客户端实际发布频率／抖动、本地反馈接收频率／间隔 | 新适配记录后可以；不能称为设备控制频率或原始采样频率 |
| 主机采用频率、命令覆盖／拒绝数 | 无采用回执，需主机埋点或可信日志 |
| 设备采样连续性、网络丢包率 | 无设备seq／发布seq，接收序号或重复数值不足以判定 |
| 指令单向延迟、反馈传输延迟 | 需要两端时钟映射与已确认的时间戳事件 |
| 通信回执RTT | 原接口没有匹配命令序号的回执；另加旁路echo只能测旁路RTT，不证明控制采用延迟，也不除以二 |
| 指令到运动响应 | 可在小幅已授权实验中估计整体响应滞后；包含传输、排队、控制、驱动及机械响应，不能直接叫网络延迟 |
| 电流环时间常数与纯延迟分离 | 需原始电流及主机实际写入历史／采样时刻；当前资料不足以套用M2驱动辨识 |

`ros_benchmark`继续限定本机私有Mock域，不移除其`localhost_ros_mock`保护。真实采集用独立入口显式配置domain、namespace、RMW、网卡／DDS与QoS；离线契约Mock使用厂商原消息，在私有命名空间模拟反馈乱序、字段缺失、模式丢失与中断，不冒充真实主机。

## 分轮实施与验收

| 轮次 | 交付 | 验收边界 |
| --- | --- | --- |
| S1：只读接入 | SDK可选overlay、版本／消息摘要、能力配置、C++状态记录、Python字段规范化、实测q只读viewer；默认不创建命令publisher | 厂商原消息的本机Mock下验证乱序、缺轴／重复、空字段、紧凑PD、时间未知及中断；不下发／不切模式 |
| S2：位置执行 | 完整14轴后端、实际姿态起始、上身模式管理、位置流、500Hz频率／整体响应报告 | 明确子集／腰部／停止语义，先本机契约Mock，再按实机条件验证；低频位置与无力矩能力时后续阶段明确阻断 |
| S3：动力学观测资格 | effort/current来源与尺度、采样时钟／滤波／带宽、主机日志入口、真实数据reader | 实机新协议，不伪造时刻和设备seq；未达动态带宽可只开放符合条件的静态阶段 |
| S4：分阶段实机闭环 | 重力→运动摩擦→起动摩擦→前序验收→惯量→独立验证／控制对照 | 复用模型和算法；依实际能力逐阶段开放，物理路径／停止门禁和新独立留出齐备 |

第一轮宜先做S1。它可在没有实机、没有运控主程序时完成开发和本机协议验证，并为甲方提供明确的现场采样入口。S1不依赖先解决纯力矩，但不标记动力学已可执行。

## 需要甲方优先确认

1. 实际运控／固件及`crb_ros_msg`版本，真实URDF、限位、工具与腰部抱死时的驱动设置；上身是否允许仅14轴子集。
2. `/joint_states.effort`到底是原始电流换算的输出力矩、其他估计还是指令；量纲／增益、滤波、速度来源、饱和／故障与header事件。能否通过主机日志取得4kHz数据或验证有效的平均观测。
3. 上身控制方程、effort叠加位置、零Kp/Kd是否允许与实际生效；内部重力／摩擦补偿、限幅、`time_ref/vel_scale`、接收／采用频率和回执。
4. 当前姿态下可用的停止／保持／故障恢复和断网watchdog；模式退出后行为。能否取得与该上身模式对应的增益读回。

构建和离线测试结果见下节；这些问题不由SDK客户端编译结果替代。

## 本机验证记录

产物目录：`artifacts/sdk_intake_20261029/`。SDK源码保持未修改，SDK overlay与现有calib overlay隔离。构建固定系统Python3.10，不引入conda，也不重建calib原生绑定。

```bash
# 宿主casbot_calib根目录；本轮实际采用的隔离构建入口
bash scripts/humble/container.sh env -u VIRTUAL_ENV \
  PATH=/opt/ros/humble/bin:/usr/bin:/bin \
  colcon --log-base artifacts/sdk_intake_20261029/log build \
  --base-paths /home/ww/codings/casbot2-ros2-sdk \
  --build-base artifacts/sdk_intake_20261029/build \
  --install-base artifacts/sdk_intake_20261029/install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

实际结果：六包全部编译通过，用时约54.8秒；两个C++包只报告未使用的旧`PYTHON_EXECUTABLE`参数警告。加载这个独立overlay后，以系统Python执行SDK的离线测试：

```bash
bash scripts/humble/container.sh bash --noprofile --norc -c '
  source artifacts/sdk_intake_20261029/install/setup.bash
  export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=218
  /usr/bin/python3 -m unittest discover \
    -s /home/ww/codings/casbot2-ros2-sdk/examples/casbot2_py_tests/test -v
'
```

**17项全部通过**，覆盖23接口SHA、紧凑增益、异常数组、服务拒绝／超时、Action业务失败／取消及安装入口；日志为`artifacts/sdk_intake_20261029/offline-tests.log`，构建日志在`log/latest_build/`。这只验证本地SDK与现有Humble环境的构建和客户端契约，不验证机器人服务端或硬件性能。没有运行SDK中的运动／模式切换示例，也没有启动SDK仿真物理后端。calib此次只改文档，未重跑其275项历史测试或力学回归。
