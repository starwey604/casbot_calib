# 🤖 实机联调路径与剩余工作

本轮目标是补齐能连接真实 SDK 的软件路径，让机器人到场后可以开始联调。**尚未确认的行为采用明确配置的假设，不再让全部代码开发等待设备能力验收。** 联调可以产出候选和残差；正式标定验收仍须区分假设与实测。

已经接入的 H1 路径为：**只读实际起姿 → 单轴小幅往返 → 静态参考连接 → SDK位置流及原始采集 → 探索重力拟合 → 候选URDF与重加载**。这不是完整摩擦/起动/惯量硬件交付。本机验证使用隔离ROS端点，没有连接物理机器人。

原S2私有入口继续保留。新增硬件命令使用 hardware-*，不是把S2的false改成true；两条路径共享C++发送器。机械计算、碰撞及优化继续复用Pinocchio、Coal与原有物理一致求解器。

## 采用的接口约定

| 内容 | 本轮处理 |
| --- | --- |
| 只发双臂14轴 | 用户已确认，腰/头/手保持；消息不填充这些轴 |
| 断流保护 | 用户要求先假设机器人自行保护；异常时停止发布并记录是否观察到反馈静止，不虚构厂商stop服务 |
| 插值 | 厂家说明仅position线性插值到1000Hz；客户端默认500Hz |
| 默认PD及命令字段 | 按SDK示例，PD数组空、effort=0、time_ref=0、vel_scale=0.1；实际消费行为仍需联调 |
| effort | 模组估算Nm，无独立扭矩传感器；先假设输出侧、增益1、零偏0，支持逐轴配置 |
| 模型和安装 | 暂按nominal、代理夹具、默认重力[0,0,-9.81]；不称为已验证实机几何/限位 |
| 反馈时间和缓存 | 发布stamp/本地接收时刻分别保存，采样时间未知；重复缓存不删除，不计为独立采样 |

默认配置为 [hardware_field.yaml](../configs/hardware_field.yaml)。配置、模型、SDK或执行器更新后建立新计划。

运行保护检查起姿变化、反馈/模式超时、nominal行程、速度、跟踪偏差和竞争发布者。参考和客户端位置插值做碰撞检查，分析另查接收观测的插值；当前未认证真实跟踪/制动包络或物理停止距离。position_allowance_rad是运行保护参数，不是已证明的碰撞安全管道。

## 机器人到场后从这里开始

按[启动指南](dynamics_getting_started.md)准备Humble和SDK环境。在宿主仓库根目录、同一个终端运行；所有输出目录必须不存在：

~~~bash
cd ~/codings/casbot_calib
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
read -r -p "现场 ROS_DOMAIN_ID: " calib_domain_id
~~~

### 1. 只读读取实际起姿

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-snapshot --config configs/hardware_field.yaml \
  --domain-id "$calib_domain_id" --output artifacts/site_snapshot_v1
~~~

快照不创建命令publisher、不请求模式，要求完整14轴且反馈显示静止。默认使用/joint_states，也支持JointStateData；不会为了高频反馈主动切换全身模式。先查看snapshot.csv并核对实际姿态/轴映射。

### 2. 从实测姿态规划1度单轴往返

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-jog --config configs/hardware_field.yaml \
  --snapshot artifacts/site_snapshot_v1 --joint left_shoulder_pitch_joint \
  --amplitude-deg 1 --output artifacts/site_jog_plan_v1
~~~

它仅规划：目标轴正向、反向、回起点，其余13轴保持实测角度，检查完整进退的nominal限位和碰撞。下面的hardware-execute会实际申请上身模式并发送位置指令：

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-execute artifacts/site_jog_plan_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_jog_run_v1

bash scripts/humble/container.sh casbot-calib hardware-analyse \
  artifacts/site_jog_plan_v1 --capture artifacts/site_jog_run_v1 \
  --output artifacts/site_jog_analysis_v1
~~~

观察实际方向/回位后再逐轴扩大覆盖。软件不会把一次往返自动标为机械零位或物理方向已验证。位置报告包含实际发送/接收频率、未对齐误差及整体轨迹偏移；偏移包含插值、缓存和机械响应，不是单向网络延迟。

执行开始要求状态为READY或IDLE_MODE/STAND，不自动接管其他控制器已经持有的上身调试。默认使用主机PD；若实际未运动，分析不会仅因跟踪保护阈值较宽或最终回到起点就判通过，应检查主机增益配置与命令消费。规划快照和执行ROS domain必须一致。

异常时停止后续发布并保留记录；机体断流保护到场后需实测，客户端退出不是硬件急停。正常结束先回到本次实测起姿并确认反馈静止，再退出上身模式。

### 3. 把静态预设接到现场采集

没有静态预设时先生成：

~~~bash
bash scripts/humble/container.sh casbot-calib plan-static-field \
  --output artifacts/site_static_preset_v1
~~~

静态扫描前应重新读取起姿。下例路径仅在上面的快照姿态仍有效时复用；执行器还会复核起姿。计划包含“实测起姿→预设基准→各参考→实测起姿”，不假设机器人已在Mock home。

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-static artifacts/site_static_preset_v1 \
  --snapshot artifacts/site_snapshot_v1 --config configs/hardware_field.yaml \
  --max-poses-per-split 1 --output artifacts/site_static_plan_small_v1
~~~

--max-poses-per-split 1表示每臂每角色最多一个姿态，仅作采集联调；不保证完整可观测性，**也不保证角度小**。先完成单轴小幅检查，再检查静态计划的实际目标与起始过渡。完整扫描去掉此参数。

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-execute artifacts/site_static_plan_small_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_static_run_small_v1

bash scripts/humble/container.sh casbot-calib hardware-analyse \
  artifacts/site_static_plan_small_v1 --capture artifacts/site_static_run_small_v1 \
  --output artifacts/site_static_analysis_small_v1
~~~

别名hardware-execute-static、hardware-analyse-static也可使用。现场入口连接实际硬件网络，不自动启动模拟机器人。

## 怎样解释输出

| 文件/字段 | 含义 |
| --- | --- |
| raw_state/ | S1原始CDR、header、状态/端点与接收记录；采集器被本次执行显式中断属于正常关闭记录，不是机器人停止 |
| execution/commands.csv | 计划槽/时刻、实际发布时间、q/dq；effort固定0，PD数组空 |
| execution/feedback.csv | 整数ns接收时刻、发布stamp、14轴q/dq/原始effort；不伪造电流或设备seq |
| events.csv、result.json | 模式、异常、跳槽、回位/反馈静止；物理断流保护不能由程序自证 |
| static_windows.json | 实际q/力矩窗口均值、接近方向、数据角色；接收条数不当成独立采样数 |
| candidate_exploratory.urdf | 静态可观测组合的物理代表，未观测惯量由先验选择；不覆盖原模型 |
| calibration.json、bodies.csv | 探索候选参数、安装重力、增益/偏置假设、等效体质量/质心；不冒充正式S4导出 |
| urdf_reload.json | 复用已有导出器，检查Pinocchio重加载一致性 |
| report.json | 拟合秩、留出残差、nominal对照、频率与全部假设 |

静态passed表示探索拟合和序列化运行成功；commissioning_validation_passed另比较配置的联调误差门槛。hardware_calibration_accepted、torque_scale_verified、sample_time_known仍为false；不伪装成S3/S4合格观测，不自动部署候选。

## 完整实机路径还剩哪些代码

| 轮次 | 交付范围 | 状态 |
| --- | --- | --- |
| H1：位置与静态起步 | 实际SDK后端、实测起姿、单轴小幅、静态采集、原始记录、探索重力/候选导出 | 本轮接入，隔离ROS协议及S2原生Mock回归；无物理设备验证 |
| H2：现场批次编排 | 14轴人工方向结论绑定、100/250/500Hz重复/随机顺序、稳定窗口重试、受控中断续采、整批状态/回放 | [H2入口已实现](hardware_batch_h2.md)；执行故障只记录占位清故障日志并终止，当前不恢复/续采故障会话 |
| H3：摩擦 | 多构型匀速接SDK、冻结现场重力、运动摩擦处理；真实effort/PD下的反馈守护起动力矩斜坡 | 原求解器/Mock已有；硬件轨迹适配、在线斜坡及PD/力矩消费核对待补 |
| H4：惯量与控制 | SDK加速度激励、缓存/时钟/滤波适配、独立预测/前序复核、前馈与PD四组对照 | 复用求解器；硬件采集/新观测协议待接，不直接降低旧4kHz门槛 |
| H5：现场验收 | 已知载荷尺度、实际几何/限制/保护核验，新留出、最终URDF/运行时参数和部署报告 | 需要实机数据；部署范围由通过的阶段决定 |

机器人到场后可从H1开始单次联调，或用[H2批次入口](hardware_batch_h2.md)完成方向、频率与静态批次。完整动力学尚未完成；后续H3/H4复用本轮执行和原始采集后端。

## 本机验证范围

可选Humble/SDK四包构建通过。verify_hardware_field.py在私有domain、localhost端点调用同一硬件执行器，覆盖只读快照、发送路径、位置缓存重复、静态拟合/URDF重加载、反馈中断后的停止发送和无伪造停止回执。它是ROS协议夹具，不是另一套机械仿真或厂商插值算法认证。

原S2的Pinocchio/Boost物理Mock 500Hz往返回归另跑，仍保持本机scope和原停止服务逻辑。最终协议证据为artifacts/hardware_field_ros_v4（包含小幅jog、静态及中断）与artifacts/hardware_field_s2_regression_v1。本轮完整Python回归401项通过，后续更新的28项相关专项通过；具体为hardware_field_tests_v3/full.xml与hardware_field_tests_v6/focused.xml。旧数值物理内核/采样资格未修改，未重跑历史114项数值交付。汇总见artifacts/hardware_field_delivery_v1/delivery.json。
