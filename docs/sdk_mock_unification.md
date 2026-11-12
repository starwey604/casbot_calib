# 🔌 SDK Mock 与实机共用入口

现场流程现在通过同一套SDK客户端连接模拟设备或实体机器人：**H1–H4 → 厂商消息与模式服务 → SDK Mock服务端或实机主机**。切换连接使用profile；轨迹、碰撞门禁、采集、辨识和报告不分别实现两套。

SDK Mock现在只启动一个sdk_mock_server进程，直接持有共享的C++ SimulatedArmDevice，由既有ArmSimulationSession推进物理、本地PD及驱动响应。内部不再经过ROS命令/反馈转发；位置插值、反馈缓存与公开SDK话题保留。通用通信基准的mock_host复用同一设备组件，仍可独立运行。MuJoCo只显示几何，快速虚拟时间实验继续直接使用原内核。

**DVT2模型迁移准备期**：使用`configs/dvt2/sdk_mock.yaml`与`configs/dvt2/hardware_field.yaml`成对启动，后续H1–H4使用生成的客户端profile。新模型的身份、场景与前馈一起切换，不只替换服务端URDF。用法和本轮验证见[N3接入](dvt2_n3_integration.md#第二轮接入动力学采集与sdk-mock2026-11-06)。默认入口整体切换仍待K2–K4完成。

## 🧭 一、已经对齐的内容

| 项目 | 统一方式 |
| --- | --- |
| 位置命令 | crb_ros_msg/msg/UpperJointData，明确双臂14轴，不填充腰/头/手 |
| 模式服务 | std_srvs/srv/SetBool，语义对应/motion/upper_body_debug |
| 状态 | /motion/status，供同一执行器读取模式和状态 |
| 标准反馈 | sensor_msgs/msg/JointState，/joint_states及/motion/joint_state |
| 调试反馈 | crb_ros_msg/msg/JointStateData，/motion/debug/joint_state |
| 时间和缓存 | 公开消息只携带发布stamp，缓存重复保留；不公开内部采样时间、设备序号或原始电流 |
| 客户端 | H1–H4共用position_executor、原始CDR记录和Python分析，无Mock专属标定算法 |
| 故障 | H2只log占位clear_fault，然后终止整批；不恢复故障会话 |

Mock端点在私有namespace、localhost和显式ROS domain内。生成profile的connection记录namespace、domain及Mock行为协议摘要；SDK端点统一重映射，客户端只订阅SDK话题；当前SDK Mock没有内部plant命令/反馈话题。实机profile不含connection，使用原SDK端点。生成的Mock profile不会隐式切到公共硬件网络。

## ⚙️ 二、行为假设与可配置内容

默认行为见[sdk_mock.yaml](../configs/sdk_mock.yaml)。所有数值为合成测试条件，不是厂家实测参数。

- SDK Mock主机参考更新1000Hz、反馈发布1000Hz，客户端仍默认500Hz，可用100/250/500Hz分档。
- 仅position插值：收到新目标，从当前连续参考线性过渡到新目标；默认时长2ms，可配置。velocity、effort及PD保持最新值，不由位置差分生成。
- 缓存默认约3ms刷新，1ms发布可能重复同一缓存。发布频率不等于真实刷新率；3ms并非厂家规格。
- H3用户确认主机不自动加重力/摩擦，因此当前默认普通输出侧PD+command effort，不添加内部重力/摩擦；外部发送器负责重力前馈。不使用计算力矩加速度伺服，固定增益按初始名义质量矩阵对角项一次性生成，所有发送频率共用。
- H3用户确认effort按Nm叠加PD且零PD生效，当前Mock默认接受非零effort和自定义PD；接口约定不等于已完成实机能力验收。旧sdk_mock_ros_v5证据仍按当时内部重力开启、非零effort关闭的合成条件解释，不回写旧证据。
- SDK路径固定device_pd=true，配置false会在启动前拒绝；通用ROS通信基准保留原主机PD模式。SDK Mock把五字段参考传给原生ArmSimulationSession，由本地状态计算PD，复用同一驱动/积分内核。不是用ROS回传反馈闭合PD；用户已确认CASBOT主机直接转发五项并由模组本地计算PD，此执行位置固定为接口约定，具体PD环频率仍未知；确认记录与ENCOS模式资料见[本地PD能力证据](sdk_hardware_prerequisites.md#因克斯模组本地pd能力证据)。保持增益带Kp=2、Kd=0.12的合成下限，不把默认值当实机整定值。
- 默认断流100ms后模拟重力和PD保持；这是合成保护行为，不能证明实机停止或允许恢复。新服务端没有Mock专用stop RPC。
- 切模式到首帧之间暂保持原保护输出，收到第一条有效命令后才切到用户命令；用户已确认该保持行为，具体设备/固件仍需验证。

用户要求先采用简单线性插值。当前实现不推测厂商缓冲、迟到或插值窗口；后续获知真实规则后修改行为profile/实现，旧采集不回写。time_ref=0取自[SDK上身示例第50行](../../casbot2-ros2-sdk/examples/casbot2_cpp_tests/src/upper_body_debug_test.cpp)，字段定义在[UpperJointData.msg](../../casbot2-ros2-sdk/crb_ros_msg/msg/UpperJointData.msg)。本轮只支持零值，不把它解释为2ms或任意已确认的时间单位。vel_scale只检查范围，其真实速度作用仍待确认。

## ▶️ 三、从Mock切到实机

SDK Mock只需构建可选SDK包；需要运行旧S2或ROS通信基准时另执行build_ros.sh。诊断异步写盘使用libspdlog-dev，已列入install_system.sh和可选SDK包依赖；不加入离线核心或Python wheel依赖：

~~~bash
cd ~/codings/casbot_calib
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
~~~

终端A启动Mock；输出目录必须不存在，运行到Ctrl-C结束：

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib sdk-mock --config configs/sdk_mock.yaml \
  --domain-id 223 --output artifacts/my_sdk_mock_v1
~~~

显示就绪后，终端B运行同一H1入口，只把profile换成生成文件：

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-snapshot \
  --config artifacts/my_sdk_mock_v1/client_profile.yaml \
  --domain-id 223 --output artifacts/my_sdk_snapshot_v1

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-jog \
  --config artifacts/my_sdk_mock_v1/client_profile.yaml \
  --snapshot artifacts/my_sdk_snapshot_v1 --joint left_shoulder_pitch_joint \
  --amplitude-deg 1 --output artifacts/my_sdk_jog_plan_v1

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-execute artifacts/my_sdk_jog_plan_v1 \
  --domain-id 223 --output artifacts/my_sdk_jog_run_v1

bash scripts/humble/container.sh casbot-calib hardware-analyse \
  artifacts/my_sdk_jog_plan_v1 --capture artifacts/my_sdk_jog_run_v1 \
  --output artifacts/my_sdk_jog_analysis_v1
~~~

H2使用同一个profile，后续按[H2指南](hardware_batch_h2.md)执行和受控续采（方向review与质量门禁已删除）：

~~~bash
# 先按H2指南生成静态预设 artifacts/my_static_preset_v1
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-batch \
  --profile artifacts/my_sdk_mock_v1/client_profile.yaml \
  --preset artifacts/my_static_preset_v1 \
  --domain-id 223 --output artifacts/my_sdk_batch_v1

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/my_sdk_batch_v1
~~~

换实机时采用核对后的硬件profile、现场domain和新输出目录，重新读取实测起姿并生成新计划。不要编辑冻结的Mock计划去指向实机。Mock快照与另一连接不能混用；H2续采还会核对连接、模型、SDK及实现身份。

## 📂 四、记录与验证边界

Mock目录的protocol存冻结行为和二进制身份，client_profile.yaml是公开连接入口。evaluation/plant与evaluation/server保存内部采样、指令采用、参考更新及缓存序号，只供测试审计，**辨识和回放不得读取这些内部真值**。客户端自己的采集目录仍只有正常SDK消息、已发送命令和接收记录。

verify_hardware_field.py现转入verify_sdk_mock.py，H1/H2的主要集成验证使用该SDK服务端和完整机械Mock，删除了原脚本中直接修改位置的独立Python夹具。Mock专用故障注入通过私有节点参数执行，只属于测试控制，不是声称SDK提供了这些服务。

旧mock_sdk_bridge与S2专用stop服务保留给legacy回归，不承接H1–H4新功能。S2与现场入口共享sdk_process.py管理录制和执行子进程，端点、停止语义和验收策略由各自入口保留。原生高精度虚拟时间实验也保留，继续用于机械正确性与采样收敛；它们无需伪装成SDK能力。

## ✅ 五、历史双进程版本验证范围

artifacts/sdk_mock_ros_v5/verification.json的21项本机ROS检查通过，覆盖同一个H1/H2客户端、两种反馈类型、三档频率、静态候选/重加载、质量重采、受控中断续采和故障终止。Mock参考更新实测约996.54Hz；客户端三档约100.00、249.99、499.99Hz。整体轨迹偏移约50–54ms包含合成伺服、驱动、插值和缓存响应，不是单向网络延迟。

该集成场景使用左肩0.2°往返、两个静态姿态和单关节三档各一次，其他轴由同一完整机械模型保持；人工方向结论由测试明确标注为夹具观察。它验证完整流程连接，不宣称已执行默认90个单元、全部14轴的机械验收或实体机器人验收。

同目录truth_isolation.json另记录两项新进程复核：禁止打开任何evaluation路径后仍能重新分析，重力参数与原结果最大差0。中文视频已实际解码检查，位置来自SDK观测，MuJoCo不推进物理。内部初始保持需实际稳定后才宣告就绪；静态测试留出稳定时间，未放宽原稳定判据。早期v1–v4的环境/闭环稳定性/等待不足失败保留。

完整Python回归431项通过（含14项新增连接/配置/隔离反例），报告为artifacts/sdk_mock_tests_v2/full.xml。旧S2的500Hz位置回归见artifacts/sdk_mock_legacy_s2_regression_v1，仍按其原合成协议解释。原生四项CTest及新增SDK线性插值策略测试通过；本轮未重跑历史114项冻结数值交付。本机artifacts不随Git交付，最终源码/测试/证据摘要见artifacts/sdk_mock_delivery_v1/delivery.json。

H4可在plant中显式设置14项inertia_scales，缩放各等效体质心惯性，质量/质心保持。它只改变合成plant，用于检查惯量恢复；该数组封存在Mock行为/evaluation边界，不能作为辨识先验读取。省略时全为1，历史条件不变。H4使用相同SDK客户端发送预计算前馈，见[H4指南](hardware_inertia_h4.md)。

## 🧩 六、当前共享代码与日志

- cpp/include/casbot_calib/simulated_device.hpp统一模拟设备配置和反馈交付；力学、积分、内环PD只调用原ArmSimulationSession。
- control_policy.py统一五档PD网格、Kp/Kd缩放、选择及四组相对误差计算。已知采样时间的虚拟实验与SDK回调实验保留各自的时间处理和接受门槛。
- sdk_process.py统一ROS参数编码、录制就绪、执行监视和子进程关闭；录制器退出会终止对应发送器。故障后的clear_fault仍只log并终止批次。

共用position_executor启动时允许话题与模式服务异步发现：在原10s启动超时内等待所需服务，期间不发指令；超时仍结束执行。模式拒绝、运行中失联和停止语义保持不变，SDK Mock与实机使用同一处理。

当前Mock行为记录使用casbot_sdk_mock_backend_v2，标明in_process_shared_SimulatedArmDevice及internal_ros_bridge=false。client_profile.yaml和H1–H4 CLI不变。plant.control_hz和plant.feedback_hz仍分别控制进程内设备推进与反馈交付节奏，公开control_hz/publish_hz、驱动/反馈延迟和缓存配置保持独立。

物理推进由专用线程负责，SDK接口使用单线程ROS执行器，经过进程内指令队列和反馈快照交接。只有物理线程访问模拟设备；ROS线程处理插值、模式、缓存与公开消息。物理推进错过墙上时间槽位时跳过等待，虚拟积分仍覆盖所有物理事件；异常传回ROS线程并退出进程。这样积分不占用ROS执行器，也不需要额外DDS跳转。反馈可用时间记录实际交接时刻，不把虚拟采样时刻冒充ROS接收时刻。

反馈快照先交接，再格式化诊断。SDK Mock的CSV按原字段和17位精度分批交给独立spdlog写盘线程；1024项有界队列满时施加背压，不丢行，关闭时排空并检查写入错误。原始客户端CDR记录仍使用既有独立采集进程。这样诊断写盘不直接占用SDK或物理推进线程。

SDK Mock的evaluation/plant/device_commands.csv记录下发设备的q、dq、feedforward_nm、kp、kd；host_samples.csv记录内部采样与估算电流。它们均为测试诊断，不属于客户端观测。没有再用回传状态推算一次PD后，把该值标成实际内环请求。通用mock_host在device_pd=false时保留旧host_writes.csv的effort字段；device_pd=true时改为明确的五字段列。

移除内部DDS会改变模拟的调度/延迟分布，旧双进程报告按原配置解释，不与当前记录混作同一时序基线。新计划绑定当前实现和Mock行为；旧会话继续只读分析，执行使用新批次。

## 🧪 七、本轮收敛结果（2026-11-02）

本机验证全部使用localhost私有ROS域。H2/H3使用封存数据，最后的诊断写盘隔离通过重新采集SDK接口和H4长时间流程复核；物理方程、指令语义和辨识模型没有因写盘调整而改变。

- **SDK接口**：artifacts/sdk_convergence_io_v1/verification.json的17项检查通过。只有一个设备进程、没有内部plant ROS话题；同一客户端支持两种反馈类型、模式操作、断流终止和服务发现等待。参考更新实测999.07Hz。
- **H2**：artifacts/sdk_convergence_ros_v5/verification.json的27项检查通过，涵盖人工方向门禁、静态质量重采、受控中断续采、故障终止和禁止访问evaluation的新进程分析。客户端100/250/500Hz实测约100.001/250.004/499.985Hz。
- **H3**：artifacts/sdk_convergence_h3_v4/verification.json的10项检查通过。三条匀速记录和四条双向起动记录给出b=0.080026Nm·s/rad、fc=0.030077Nm；合成设定为0.08与0.03。负/正向操作性起动阈值约0.06327/0.06398Nm，合成静摩擦为0.06Nm。独立姿态/重复、无真值重分析及数值收敛均通过。
- **H4**：artifacts/sdk_convergence_h4_v3/verification.json的11项检查通过，完成三条惯量、五条PD选择和四条独立控制记录。前序参数复核、无真值原始重处理与重新拟合、候选一致性均通过。所选轴的等效惯量误差3.09%，选择的Kp比例为0.75。固定PD下辨识模型的RMSE反而比先验高约1.06%，本批次未展示稳定控制收益；完整四组指标保留在batch_result.json。

上述H3/H4使用完整14轴物理模型，但只选择左肩pitch辨识，不是默认全14轴实机验收。100/50µs记录指令回放的位置轨迹及峰值指标满足原数值门限；粘滑瞬态逐点速度差仍单列为诊断，H3/H4最大归一化差分别约8.85%/14.85%，不能概括成所有状态逐点误差均小于1%。H4中文快照已查看，视频已完整解码；MuJoCo仍只回放SDK观测。

完整Python回归467项、核心CTest五项和SDK CTest三项通过；SDK新增检查有界异步CSV排空不丢行。核心测试复用本轮未再变更的设备封装候选，SDK测试在最终构建上执行。旧S2单条500Hz往返在artifacts/sdk_convergence_legacy_s2_v1通过，未重跑历史完整故障矩阵或114项冻结数值交付。本轮汇总为artifacts/sdk_convergence_delivery_v1/delivery.json，Python报告为artifacts/sdk_convergence_tests_v2/full.xml。

保留的失败记录解释了这次修正：早期单线程SDK参考约723Hz，改为独立物理线程；服务发现竞态改为启动限时等待；非目标轴合成静摩擦触发3mrad保持门禁后，仅提高演练保持增益；起动审计改用实际发送/决策时间。H4 v1/v2的长时间反馈空窗分别触发执行/连续性拒绝，最终隔离同步诊断写盘后v3完成全部记录。未放宽原反馈、连续性、保持或碰撞门限，也未续采故障批次。早期目录及原始失败报告保留。

虚拟时间实验继续保留直接批量入口；legacy S2只做兼容，不承接新功能。TriCal实物表具与SDK的联动适配仍属于后续运动学工作，不在本轮收敛范围内。

## 2026-11-06：缓存发布与五字段转发不依赖主机反馈刷新

DVT2的N4复测发现：旧Mock在内部物理推进落后20ms时，会停发公开缓存并停止转发控制。H4一次出现129ms内部交接空窗，客户端随后按100ms失联门禁终止；这不是厂家已说明的“继续发布最新缓存、五字段转发给模组”的行为。

现在正常上身控制中，position插值与五字段转发不再等待主机反馈刷新，本地PD仍由唯一物理设备内核使用本地状态计算。公开话题有缓存且未显式禁用时继续发布，header仍是发布时间；内部真实采样/交接时刻继续只进入诊断，不冒充新采样。合成保护保持、首帧前保持或显式内部重力模式需要位姿，仍保留原内部新鲜度检查。

`feedback_enabled=false`的断流注入、设备异常退出、客户端100ms失联和故障整批终止规则不变。内部物理可能赶不上墙钟，持续发布缓存不代表已证明物理实时性；频率/年龄诊断和正式采样资格继续分开报告。实现及四项SDK CTest见`artifacts/dvt2_n4_sdk_cache_build_v1.log`和`dvt2_n4_sdk_cache_ctest_v1.log`，H4修复前后记录分别保留在`dvt2_n4_h4_sdk_v1/v2`。

物理推进的后续修复见[DVT2耗时剖析](dvt2_mock_performance.md)：晚到后不再额外睡满一周期，共享内核用一次Pinocchio非线性项计算代替重复重力计算。DVT2 H4 v4的12条SDK记录已全部执行，但所选轴惯量真值核对仍超10%；新独立动态夹具v6十项验证通过、所选轴误差5.376%。执行、参数精度和控制收益分别报告，完整结果见[N4收尾](dvt2_n4_closeout.md)。
