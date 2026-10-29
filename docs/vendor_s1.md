# S1：SDK只读采集、规范化与姿态显示

本轮实现[SDK源码接入计划](casbot02_sdk_source_intake.md)的S1。可选C++节点订阅厂商反馈，保留原始CDR、字段、本地单调接收时间及端点QoS；Python独立检查并规范化；已有MuJoCo viewer接收有效位置。没有运动命令发布器、模式客户端或新的物理内核。

根据统一显示的需求，新增`ros-view`：现有原生ROS Mock与厂商ROS反馈均通过薄消息适配转换成共享`StateFrame`，再进入同一几何渲染器。文件跟随和离线回放保留为其他输入源，实时显示不再依赖采集文件。

**验收范围为本机真实ROS双进程、厂商原消息的合成协议样本。未连接实机、未验证物理网络，也没有开放动力学阶段。** 这些样本只是测试消息，不由另一套机械仿真产生，不替代现有Pinocchio/Boost Mock。SDK版本核对及尚缺设备能力见[源码评估](casbot02_sdk_source_intake.md)。

## 构建与隔离

SDK默认位于相邻`../casbot2-ros2-sdk`，可通过`CASBOT_SDK_ROOT`指定路径。使用当前Humble容器，不启用conda，不向核心Python包增加ROS强制依赖。

```bash
# 宿主casbot_calib根目录；可选构建只编译厂商消息与只读节点
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
# SDK在其他位置时
bash scripts/humble/container.sh env CASBOT_SDK_ROOT=/path/to/sdk \
  bash scripts/humble/build_vendor.sh
```

输出为`build/vendor`、`install/vendor`、`log/vendor`，不覆盖原`humble-ros` overlay。S2扩展后，该可选构建还包括公共消息和原生核心ament库，供Mock侧合成位置伺服复用；S1节点仍独立只读。`build_vendor.sh`核对SDK清单中的23个接口SHA和两个扩展导出，保存实际25个接口摘要、SDK提交及已审阅运控基准。运行时再次核对已安装接口文件，并记录实际只读可执行文件和消息共享库SHA。这个证明只针对本地构建，不证明机器人端版本相同。

`build_ros.sh`显式忽略可选`casbot_calib_vendor`，原Mock构建无须SDK。S1只增加ROS节点，没有修改或替换力学内核与已安装的pybind11扩展。

## 只读记录入口

先复制[默认配置](../configs/vendor_s1.yaml)，按目标核对topic、消息类型和QoS。主反馈默认`/joint_states`，类型`JointState`；也支持显式选择`JointStateData`。可选debug反馈默认关闭：S1不会为了让它出现而切换到全身debug。状态字符串只被记录，不授予控制权限。

以下命令需要现场已确定的域号。`72`仅为示例，不能据此推断甲方配置；它只订阅，不发送运动指令或模式请求。

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib vendor-capture --config configs/vendor_s1.yaml \
  --output artifacts/my_sdk_readonly --duration 10 \
  --domain-id 72 --scope hardware_read_only
```

输出目录必须不存在。`--domain-id`和`--scope`均为必填；硬件只读scope显式设置`ROS_LOCALHOST_ONLY=0`，本机契约scope设1，并要求所有topic使用私有`/casbot_calib_s1_*`或`/casbot_calib_s2_*`前缀。实际RMW在记录器元数据中保存，网卡／DDS仍由现场配置；本轮容器内本机通信通过不保证跨机发现已配置好。

记录包括：

| 文件 | 内容 |
| --- | --- |
| `session.json` | 配置、SDK身份、本地二进制／消息库摘要、作用域、域号、请求时长、软件／固件版本声明 |
| `raw/cdr.bin` | GenericSubscription收到的序列化字节，在解码前追加保存 |
| `raw/packets.jsonl` | 全局本地回调序号、接收时间、channel/topic/type、CDR偏移／长度、完整具名数组和header／状态原文 |
| `raw/recorder.json` | Linux单调时钟身份、实际RMW、订阅QoS、采样时刻和设备序号未知声明 |
| `raw/publishers.jsonl` | 每0.5秒及结束时观察到的发布端名称、类型、QoS枚举与深度；无发布端时可以为空 |
| `raw/summary.json` | 接收总数、记录起止时间、正常结束／中断原因 |
| `capture.json`、`process.log` | 完成状态、原始文件摘要、进程日志；中断／错误记录保留 |

CDR保留原始IEEE非有限值；JSON字段用null表达NaN/Inf，规范化时拒绝该字段，不能当零。行写入前先保存CDR，失败／强制杀进程可能留下未完成尾部；没有完整摘要或CDR偏移不一致时，规范化拒绝并保留原始目录，不能续写或修剪成通过。可处理SIGINT会结束记录、标为interrupted并保留可独立分析的完整行；这只是停止采集，不涉及机器人停止。

S1默认期望反馈500Hz仅为报告对照值。订阅回调、序列化解码、同步文件日志和图查询都是测量负载的一部分；报告的是实际本机回调频率，不是设备采样／控制采用频率，也不据此判定网络丢包或硬实时。

## 离线规范化与能力门禁

```bash
# 不需要SDK overlay、ROS运行节点或evaluation目录
bash scripts/humble/container.sh casbot-calib vendor-normalize \
  artifacts/my_sdk_readonly --output artifacts/my_sdk_readonly_normalized
```

规范化核验采集完成摘要、原始文件SHA、回调顺序及CDR索引，输出新目录，不修改采集。14轴按显式名称重排，允许消息包含腰／头／手等额外轴，拒绝重复、缺轴及异常位置。速度、effort和PD独立校验，不能因为它们缺失而伪造数值，也不因它们缺失而丢弃有效q。PD按SDK紧凑格式跳过灵巧手；与name等长的手占位数组会被拒绝。

`observations.npz`包含q、dq、effort_raw、可选kp/kd、本地receive_ns／receive_sequence、原始header_ns和可用性mask。不可用数组以NaN＋mask表达；**不输出虚构的current、device sequence、status或sample_ns**。header即使非零、重复或倒退，也只作原始字段保存，未知时钟不与本地时间相减。effort不重命名成电流，也不自动认定为Nm。

`report.json`提供有效／无效位置数、接收频率、间隔分位数、超出默认250ms的接收间隔和最终位置新鲜度；该间隔只表示观测空窗，不能区分设备停发、DDS覆盖、负载或物理网络原因。`diagnostics.json`保留逐帧拒绝原因，`status_events.json`保留模式原文；debug流独立诊断，不与主流拼成同步测量。`capabilities.json`记录实际观察到的字段及未知项，所有运动／动力学阶段保持blocked；配置不能自行声明S1已验证采样时间、力矩尺度或模型映射。`manifest.json`封存规范化产物，回放先核对摘要。

## 实时姿态与只读视频

### 统一ROS订阅入口

```bash
# 厂商反馈：另启订阅进程，可同时运行vendor-capture进行原始记录
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib ros-view --config configs/vendor_s1.yaml \
  --topic /joint_states --message-type sensor_msgs/msg/JointState \
  --domain-id 72 --scope hardware_read_only --window --output artifacts/my_ros_live_view
# 不加--window时按真实经过时间采集显示帧并导出视频，默认5秒
```

支持三种输入类型：`sensor_msgs/msg/JointState`、`crb_ros_msg/msg/JointStateData`及已有Mock的`casbot_calib_interfaces/msg/JointObservation`。渲染器只依赖共享状态帧；消息适配保留各来源的时间和字段可用性，不能把实机未知的current、采样时刻补成Mock已知值。显示订阅使用BestEffort/Volatile/KeepLast(1)，后台线程只保留最新有效姿态，与原始数据记录器独立；显示掉帧不能作为辨识数据。它不创建命令发布器或模式客户端，rclpy会保留其标准参数事件基础设施。

现有`mock_host`已经在私有命名空间发布`observations`，使用同一持久化`ArmSimulationSession`。订阅时必须提供对应源模型ID，避免显示错误版本；例如本轮实测ID为`casbot_20251015_nominal:3a707a8ed703`，真实运行应以会话记录为准：

```bash
bash scripts/humble/container.sh bash scripts/humble/ros.sh \
  casbot-calib ros-view --config configs/vendor_s1.yaml \
  --topic /casbot_calib_bench/YOUR_SESSION/observations \
  --message-type casbot_calib_interfaces/msg/JointObservation \
  --source-model-id casbot_20251015_nominal:3a707a8ed703 \
  --domain-id 166 --scope localhost_ros_mock --window --output artifacts/my_mock_live_view
```

上面namespace和domain也是示例，需要替换为正在运行的Mock会话。可直接运行下文验证脚本，自动生成私有域并启动完整演示。原生Mock在自己的进程计算并发布，renderer仅显示；批量分阶段标定仍在进程内以虚拟时间运行，未强制改为实时DDS，也未更改其既有数据协议或冻结实验。

### 文件跟随与回放

有桌面／显示转发时，在另一个终端读取采集文件流；渲染器不订阅ROS、不持有设备控制接口，不能阻塞C++采集进程：

```bash
bash scripts/humble/container.sh casbot-calib vendor-view \
  artifacts/my_sdk_readonly --live --window --output artifacts/my_sdk_live_view
```

实时显示要求文件来自当前Linux单调时钟域，否则拒绝年龄计算，改用离线回放。只消费完整JSON行；无效位置保留此前姿态并显示诊断，超过新鲜度界显示STALE。GLFW桌面窗口的可用性取决于现场显示配置；本机已验证文件跟随和EGL视频，不宣称已在实体屏幕操作窗口。

已完成的规范化记录可在无显示器环境导出中文视频，或加`--window`交互回放：

```bash
bash scripts/humble/container.sh casbot-calib vendor-view \
  artifacts/my_sdk_readonly_normalized --output artifacts/my_sdk_playback --fps 12
```

现有名义双臂几何与SDK14轴名称一致，映射仍标为unverified，不能用显示结果认证方向／机械零位。MuJoCo只执行几何、相机和渲染；帧位置与Pinocchio FK独立核对，不放大角度，不调用动力学或碰撞API。缺失速度／电流在共享`StateFrame`中为None；不会为了显示补零电流。几何中腰部外形是现有模型，不意味着实际抱紧工装已测量。

## 自动验证

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/verify_vendor_s1.py --output artifacts/my_vendor_s1_check --render
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CASBOT_TEST_RENDER=1 \
  python -m pytest -q tests/test_vendor_s1.py tests/test_telemetry.py tests/test_viewer.py
```

验证脚本在私有domain／topic启动独立C++采集进程，测试程序只发布状态。覆盖标准与厂商反馈、乱序／额外关节、缺轴／重复、空字段／NaN、紧凑PD与错误手占位、重复／倒退header、主动静默及模式变化。独立用ROS生成类型回读CDR，并在ROS图中检查采集节点没有发布端；主动SIGINT验证原始记录保留。真实采样时间、电流、采用回执和所有动力学阶段仍为未知／未开放。

本轮实际验收（2026-10-29）：

- 厂商消息＋只读节点可选overlay构建通过；原Mock三个ROS包在不加载SDK overlay时构建通过。
- `artifacts/vendor_s1_contract_v3/verification.json`通过标准／自定义反馈、CDR独立回读、文件跟随及直接ROS订阅、陈旧检测、模式原文记录和SIGINT保留检查。主动静默／异常输入下全程平均接收约416.7／413.2Hz，这不是500Hz资格测试。
- `artifacts/ros_state_view_v1/verification.json`验证已有原生Mock→ROS→统一显示：接收1775条，拒绝0条；显示与原生采样日志的最大位置差为0，MuJoCo与Pinocchio FK最大差`7.77e-16`，停止Mock后STALE检查通过。
- 42项相关Python／渲染测试通过，最新完整Python回归**301项通过**，只有14条已有库弃用警告；记录为`artifacts/vendor_s1_tests/unified-focused.xml`及`unified-full.xml`。没有重跑历史114项数值对照或把旧结果当新验收。
- 两路中文视频均已解码查看实际帧：原生ROS Mock视频3.08秒，厂商协议记录回放3.75秒，均1280×720、12fps；分别在上述两个会话的`presentation/vendor_read_only_zh.mp4`，复核帧在`artifacts/vendor_s1_review/`。实际桌面窗口及实机未验收。

S2将补实际起始姿态、上身位置执行、控制权和停止状态机；需要先明确腰部驱动／14轴子集、实际几何及停止语义。S1没有绕过这些前置条件。

原生Mock的端到端显示验证不依赖厂商SDK overlay：

```bash
bash scripts/humble/container.sh bash scripts/humble/ros.sh \
  python scripts/humble/verify_ros_view.py --output artifacts/my_native_ros_view
```

该脚本只在私有Mock topic发送命令，复用已有C++ `mock_host`和`stream_client`；不连接实机、不新增物理求解。显示帧逐条与原生采样日志核对，退出Mock后检查STALE。协议样本脚本另验证两种厂商反馈也能直接进入同一ROS显示源。
