# M2本机验收记录

2026-10-28。实现及协议见[ROS通信说明](ros_transport.md)，后续轮次见[完整计划](humble_migration_plan.md)。

默认命令发布、主机控制和反馈发布均为500Hz，Mock原始采样独立为1kHz。已运行13组真实ROS本机双端实验，包含命令/反馈/控制频率对照、显式1kHz对照、主机软件PD、Best Effort、CPU负载、rosbag及故障注入。

| 项目 | 本机结果 |
| --- | --- |
| ROS协议、超时/拒绝、冻结电流预测 | 13组通过 |
| Python / C++回归 | 89项 / 3项通过 |
| 原A–F、R1、双臂数值对照 | 114项按原容差一致 |
| 14轴冻结预测最大电流RMSE | 0.215mA，合成阈值1.5mA |
| 与合成真值比较的最大电流时间常数相对误差 | 1.253% |
| 最大驱动延迟恢复误差 | 0.01366ms，合成阈值0.1ms |
| 评估真值隔离 | 移走后重分析，参数和冻结预测误差完全一致 |
| rosbag实际记录 | commands 800、events 1916、observations 884条 |
| RMW / 命令CDR负载 | rmw_fastrtps_cpp / 1156字节，不含网络协议开销 |

产物：[完整报告](../artifacts/ros_transport_500hz/analysis/README.md)、[频率及预测图](../artifacts/ros_transport_500hz/analysis/frequency_and_prediction.png)、[数值CSV](../artifacts/ros_transport_500hz/analysis/summary.csv)、[机器验收与源码摘要](../artifacts/ros_transport_500hz/verification.json)、[无真值重分析](../artifacts/ros_transport_500hz/reanalysis_verified/README.md)、[原实验回归](../artifacts/ros_transport_regression/README.md)。

## 频率结果的解释

通过表示协议/超时行为和冻结驱动模型通过，**不表示每个请求频率都合格**。例如默认训练批次实际发布499.7Hz、采用438.5Hz、反馈接收441.4Hz，未达到本次定义的频率筛选条件。固定顺序、每档单批也不足以推断CPU负载或QoS的因果影响；本轮不推荐实机工作频率。500Hz是用户指定的默认配置。

原始序号缺口包含DDS历史覆盖、executor处理和传输因素，不能直接当成网络丢包率。固定KeepLast(1)时，即使Reliable也不代表每个历史指令都进入应用回调。本轮保留这些现象，没有通过改写发送时间或反复拟合来掩盖频率差异。

## 已实现与未覆盖

- 已实现：共享持久化C++物理内核、ROS C++发端/Mock主机端、具名SI命令/观测/事件、三种独立时钟声明、整数时间处理、接收/采用/写入日志、缺口/拒绝/超时统计、参数冻结与真值隔离。
- 已拒绝：重复/旧序号、过期、NaN、错误关节顺序、模式和模型ID。未同步时不直接跨钟相减；设备采样钟未映射到主机写入钟时拒绝驱动辨识；RTT不除以二。
- 未覆盖：物理以太网、甲方topic/ros2_control插件、SDK和实际停止策略；当前C++端点采用wall timer，不构成硬实时或真实设备验收。M3碰撞门禁与M4–M6新辨识流程继续推进。

CI已加入本机ROS矩阵和真值隔离验证，远端CI尚未实际运行。旧1kHz试验产物仍保留，默认500Hz产物使用独立目录，原始CSV未覆写。

## 当前代码的完整回归复测（2026-10-28）

本轮最新产物为 `artifacts/staged_release_ros`，原产物作为历史记录保留。重新构建后13组协议/冻结预测/故障实验、录包和真值隔离重分析全部通过；配套原实验回归为111项Python、3项C++及114项冻结数值。

最新最大电流预测RMSE为0.2141mA，电流时间常数最大合成相对误差2.835%，驱动延迟误差0.02755ms。默认批次实际发布498.8Hz、采用496.3Hz、反馈接收497.6Hz，指令传输P99为0.2844ms。该批次通过本机频率筛选；这不能推翻之前出现438Hz采用频率的记录，也不能推广为跨机以太网或硬实时保证。12组非故障配置通过本次单批筛选；仍不据此推荐实机工作频率。

[本轮报告](../artifacts/staged_release_ros/analysis/README.md) · [本轮机器验收](../artifacts/staged_release_ros/verification.json) · [本轮原实验回归](../artifacts/staged_release_regression/verification.json)。M3–M6当前实现与完整合成验证另见[分阶段流程](collision_staged_workflow.md)。
