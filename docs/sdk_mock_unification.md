# 🔌 SDK Mock 与实机共用入口

现场流程现在通过同一套SDK客户端连接模拟设备或实体机器人：**H1/H2 → 厂商消息与模式服务 → SDK Mock服务端或实机主机**。切换连接使用profile；轨迹、碰撞门禁、采集、辨识和报告不分别实现两套。

SDK Mock内部继续连接原有C++ mock_host，只有该进程持有ArmSimulationSession并推进物理。新sdk_mock_server负责接口、位置参考插值、合成主机PD和反馈缓存；MuJoCo仍只显示几何。SDK消息包和服务器代码是可选依赖，离线虚拟时间实验不必经过ROS。

## 🧭 一、已经对齐的内容

| 项目 | 统一方式 |
| --- | --- |
| 位置命令 | crb_ros_msg/msg/UpperJointData，明确双臂14轴，不填充腰/头/手 |
| 模式服务 | std_srvs/srv/SetBool，语义对应/motion/upper_body_debug |
| 状态 | /motion/status，供同一执行器读取模式和状态 |
| 标准反馈 | sensor_msgs/msg/JointState，/joint_states及/motion/joint_state |
| 调试反馈 | crb_ros_msg/msg/JointStateData，/motion/debug/joint_state |
| 时间和缓存 | 公开消息只携带发布stamp，缓存重复保留；不公开内部采样时间、设备序号或原始电流 |
| 客户端 | H1/H2共用position_executor、原始CDR记录和Python分析，无Mock专属标定算法 |
| 故障 | H2只log占位clear_fault，然后终止整批；不恢复故障会话 |

Mock端点在私有namespace、localhost和显式ROS domain内。生成profile的connection记录namespace、domain及Mock行为协议摘要；SDK端点统一重映射，客户端不订阅内部plant话题。实机profile不含connection，使用原SDK端点。生成的Mock profile不会隐式切到公共硬件网络。

## ⚙️ 二、行为假设与可配置内容

默认行为见[sdk_mock.yaml](../configs/sdk_mock.yaml)。所有数值为合成测试条件，不是厂家实测参数。

- SDK Mock主机参考更新1000Hz、反馈发布1000Hz，客户端仍默认500Hz，可用100/250/500Hz分档。
- 仅position插值：收到新目标，从当前连续参考线性过渡到新目标；默认时长2ms，可配置。velocity、effort及PD保持最新值，不由位置差分生成。
- 缓存默认约3ms刷新，1ms发布可能重复同一缓存。发布频率不等于真实刷新率；3ms并非厂家规格。
- 默认普通输出侧PD，加可配置的内部名义重力补偿；不使用计算力矩加速度伺服。增益按初始名义质量矩阵的对角项一次性生成，所有发送频率共用。
- 默认拒绝非零命令effort和自定义PD。可显式开启Mock假设能力，为后续H3开发提供接口；这不解锁实机能力。假设方程为重力补偿+effort+PD，并统一限幅。
- 默认断流100ms后模拟重力和PD保持；这是合成保护行为，不能证明实机停止或允许恢复。新服务端没有Mock专用stop RPC。

用户要求先采用简单线性插值。当前实现不推测厂商缓冲、迟到或插值窗口；后续获知真实规则后修改行为profile/实现，旧采集不回写。time_ref=0取自[SDK上身示例第50行](../../casbot2-ros2-sdk/examples/casbot2_cpp_tests/src/upper_body_debug_test.cpp)，字段定义在[UpperJointData.msg](../../casbot2-ros2-sdk/crb_ros_msg/msg/UpperJointData.msg)。本轮只支持零值，不把它解释为2ms或任意已确认的时间单位。vel_scale只检查范围，其真实速度作用仍待确认。

## ▶️ 三、从Mock切到实机

先构建已有ROS物理端和可选SDK包：

~~~bash
cd ~/codings/casbot_calib
bash scripts/humble/container.sh bash scripts/humble/build_ros.sh
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

H2使用同一个profile，后续按[H2指南](hardware_batch_h2.md)执行、填写方向结论和续采：

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

旧mock_sdk_bridge与S2专用stop服务保留给冻结历史回归，不再作为H1/H2和后续H3的主要开发入口。原生高精度虚拟时间实验也保留，继续用于机械正确性与采样收敛；它们无需伪装成SDK能力。

## ✅ 五、本轮验证范围

artifacts/sdk_mock_ros_v5/verification.json的21项本机ROS检查通过，覆盖同一个H1/H2客户端、两种反馈类型、三档频率、静态候选/重加载、质量重采、受控中断续采和故障终止。Mock参考更新实测约996.54Hz；客户端三档约100.00、249.99、499.99Hz。整体轨迹偏移约50–54ms包含合成伺服、驱动、插值和缓存响应，不是单向网络延迟。

该集成场景使用左肩0.2°往返、两个静态姿态和单关节三档各一次，其他轴由同一完整机械模型保持；人工方向结论由测试明确标注为夹具观察。它验证完整流程连接，不宣称已执行默认90个单元、全部14轴的机械验收或实体机器人验收。

同目录truth_isolation.json另记录两项新进程复核：禁止打开任何evaluation路径后仍能重新分析，重力参数与原结果最大差0。中文视频已实际解码检查，位置来自SDK观测，MuJoCo不推进物理。内部初始保持需实际稳定后才宣告就绪；静态测试留出稳定时间，未放宽原稳定判据。早期v1–v4的环境/闭环稳定性/等待不足失败保留。

完整Python回归431项通过（含14项新增连接/配置/隔离反例），报告为artifacts/sdk_mock_tests_v2/full.xml。旧S2的500Hz位置回归见artifacts/sdk_mock_legacy_s2_regression_v1，仍按其原合成协议解释。原生四项CTest及新增SDK线性插值策略测试通过；本轮未重跑历史114项冻结数值交付。本机artifacts不随Git交付，最终源码/测试/证据摘要见artifacts/sdk_mock_delivery_v1/delivery.json。
