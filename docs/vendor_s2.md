# S2：整双臂位置执行与频率／整体响应

本轮继续[SDK接入计划](casbot02_sdk_source_intake.md)，实现观察当前姿态→完整参考门禁→申请上身调试→持续位置指令→回到起始姿态并静止→退出调试的流程。执行环在C++中，Python负责规划、封存、编排与独立分析；统一ROS viewer和S1原始记录器继续复用。

**当前只开放私有本机Mock执行。实机控制资格为false，尚未连接实体机器人或测试物理以太网。** SDK尚未确认的腰部驱动、14轴子集、停止／断流行为、实际几何／限位和命令语义不能由模拟配置补成已确认。Python与C++都拒绝非本机／非私有端点；没有新增绕过门禁的hardware执行参数。

2026-10-30[厂家答复](sdk_hardware_prerequisites.md#厂家答复记录2026-10-30同步)说明仅position线性插值到1000Hz，debug反馈可能重复最新缓存且stamp为发布时间。默认客户端发送仍为500Hz；本机Mock伺服不因此成为厂商插值实现。实际采用、缓冲/迟到、velocity/effort消费及反馈年龄仍未知，现有整体响应不拆解为单向网络或插值延迟；仅更新文档，不修改执行或门禁。

## 结构和边界

- `position_executor`：14轴具名状态核验、模式服务和状态确认、位置流、故障停止与记录；SDK命令使用`UpperJointData`，effort为零、kp/kd为空，不开放力矩或调参。
- `mock_sdk_bridge`：提供SDK原消息和模式服务，连接已有`mock_host`。该Mock侧声明一个固定的合成位置伺服：`a_request = 100(q_ref-q) + 14(v_ref-v)`，通过现有Pinocchio `ArmWorkspace.inverse`生成力矩；两个系数单位分别为s⁻²和s⁻¹，不是设备的Nm/rad增益。真实动力学、驱动响应及积分仍由同一个`ArmSimulationSession`／Boost内核负责，没有第二个机械仿真器，也不由MuJoCo计算。该伺服仅用于执行接口验证，不模仿或认证厂商内部控制器，不构成标定收益对照。
- `vendor_position.py`：实测起始快照、借用`Motion`／Bernstein导数界、Coal连续参考及已发送参考插值门禁、配置与模型／SDK／二进制摘要绑定。
- `vendor_position_analysis.py`：只读核验执行记录、实际发布／接收频率、未对齐跟踪误差、整体轨迹响应偏移及观测插值碰撞；不读取evaluation或重新拟合机械参数。

新节点位于可选`casbot_calib_vendor`包。`build_vendor.sh`构建核心ament封装、公共消息、厂商消息和适配包，输出仍独立于`install/humble-ros`；原生力学源码和pybind11扩展没有变更。S1记录器仍是独立可执行程序，不因同包增加执行节点而获得发布／模式控制功能。

## 执行状态机

| 状态 | 条件与行为 |
| --- | --- |
| WAIT_FEEDBACK | 有完整14轴有限q/dq、足够新鲜且速度小于0.02rad/s；起始模式须为READY或IDLE_MODE/STAND，实际姿态与封存参考起点一致，模式和停止服务可用 |
| ENTERING | 请求上身debug，检查服务success并等待独立模式反馈；失败、状态不确认或超时不继续下发 |
| STREAMING | 默认500Hz；按真实单调时刻取计划槽，迟到跳过旧槽；持续检查反馈／模式年龄、行程／速度、跟踪及同topic竞争发布者 |
| SETTLING | 继续最终保持，观测回到起始姿态并持续静止150ms后才申请退出；不是发布完轨迹就视为结束 |
| EXITING | 检查退出响应和READY反馈，之后正常完成；退出debug不被解释成急停 |
| STOPPING | 停止轨迹发送，调用明确配置的停止服务；确认成功回执与新鲜反馈持续静止100ms；超时／拒绝保留stop_unconfirmed，不自动恢复或回home |

SIGINT/SIGTERM在执行节点中不立即销毁ROS上下文，先进入同一停止流程。硬退出／网络断开不能依赖客户端最后一帧：Mock服务端另有100ms指令看门狗，采用当前姿态保持。`/PRIVATE_ROOT/stop`是**本轮Mock明确提供的SetBool服务**，不是SDK已确认的真实停止接口；后续须以厂商真实策略替换和验证。

执行端不发送腰、头或手的目标，也不把它们补零。测试反馈故意包含腰部轴，验证只选择14臂轴。客户端的模式与发布者检查不能证明其他未知控制通道完全互斥，因此不作为实机控制权认证。

## 轨迹和碰撞

默认动作保持→正向小幅多轴运动→保持→反向运动→保持→回到所观测起点→最终保持，所有接合点q／速度／加速度／jerk连续。起点来自新的SDK反馈快照；执行前再次读取并检查，不能用配置home代替实际起点。

使用现有nominal双臂、腰部固定和未测量夹具代理。默认跟踪检查为0.002rad，参考位置包络为0.003rad，起点容差0.0005rad；数值是合成配置，不是厂家精度／限制。当前几何肩部固定安装面附近约11.25mm间隙，10mm门槛下无法用0.03rad独立扰动包络获得保守证明。本轮保留动作幅度，以更紧的跟踪界限检查；固定误差包络不能靠时间细分缩小，无法认证时快速拒绝，不能穷举大量细分后默认为通过。

参考、命令插值与执行后观测插值分别检查。声明的包络包含一个停止余量，但**没有把该余量认证为物理停止距离**；故障发生后的观测路径另做审计。反馈中断超过允许区间时，不跨缺口宣称实际路径已获认证。配置或完整参考失败时，不进入模式申请。

跟踪保护将已到达的位置与其**本机回调时刻**的参考槽比较，再单独检查此后参考位移是否超出包络；不会把旧位置直接与较晚的定时器参考作比较。SDK采样时间和入队前年龄仍未知，不能把这种处理说成设备采样时间已对齐。

## 使用与数据

```bash
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
# 先由验证程序启动本机Mock；PRIVATE_ROOT与domain必须取该次实际会话。
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib vendor-plan-position --config configs/vendor_s2.yaml \
  --endpoint-root /casbot_calib_s2_YOUR_SESSION --domain-id 190 \
  --output artifacts/my_position_plan
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib vendor-execute-position artifacts/my_position_plan --output artifacts/my_position_run
bash scripts/humble/container.sh casbot-calib vendor-analyse-position \
  artifacts/my_position_run --output artifacts/my_position_analysis
```

目录均须新建，不提供重写原始记录或自动resume。计划保存快照、连续参考、完整CSV、碰撞证据、配置／模型／SDK／二进制身份及SHA；执行前复核摘要、连续模型与CSV、几何和碰撞。二进制或输入变化须重新规划，不继续旧计划。

执行目录保留原始发送／回调CSV、模式／故障事件、结果、S1原始CDR和模式原文，以及全部摘要。S1记录器在执行前启动，结束后以可处理SIGINT完成封存，其interrupted表示记录窗口被显式关闭，与执行是否成功分别保存。缺失的采样时间、电流、设备状态和设备序号仍不伪造；effort来源和量纲仍为unknown。

报告分开给出：

- 实际SDK命令发布和反馈本机回调频率、间隔P50/P95/P99/最大值、跳过的命令槽；不是主机采用或设备采样频率。
- 未对齐轨迹RMSE及最大误差；整体响应只估计固定参考的一个标量时间偏移作诊断，不改机械参数，也不以对齐后的残差替代原始验收。
- 整体偏移包含通信、队列、控制、驱动和机械响应，不能称为网络单向延迟或电流环时间常数。SDK没有采用回执，相关量保持null。
- 故障后是否停止发送、停止回执及实际静止确认。预期故障测试通过不等于该次轨迹成功；原始capture.passed仍为false。中止后的未执行参考不计入响应拟合，故障后的观测仍保留用于路径检查。

## 自动验证与后续

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/verify_vendor_s2.py --output artifacts/my_s2 --render
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CASBOT_TEST_RENDER=1 \
  python -m pytest -q
```

测试矩阵为100／250／500Hz各两批、固定种子随机顺序，复用相同相对连续动作和固定Mock伺服规则；另测模式拒绝、不确认、反馈缺失／无效、模式丢失、停止拒绝、主动中断、竞争publisher和规划后起点改变。各失败记录保留。停止拒绝检查必须看到服务端实际负响应、客户端stop_unconfirmed、无后续轨迹发送，不能要求多个并发保护中某一个必定最先触发。

独立的指令静默看门狗检查（不调用停止服务）为：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/verify_vendor_watchdog.py --output artifacts/my_watchdog
```

它只在已经认证的静止姿态发送位置保持后断流，验证Mock看门狗触发与继续保持；不把静止检查叫作运动中实机刹车认证。

## 2026-10-29本机结果

完整采集为`artifacts/vendor_s2_release_v3/`，独立复核为`review/verification.json`。六条正常位置往返在该批次全部通过完整参考、观测插值、跟踪与返回静止检查：

| 请求频率 | 两批实际SDK发布Hz | 最大未对齐跟踪误差mrad（两批） | 整体偏移ms（两批） |
| --- | --- | --- | --- |
| 100 | 99.998／99.999 | 1.217／1.412 | 6.398／7.471 |
| 250 | 249.920／249.920 | 1.372／1.166 | 3.827／3.837 |
| 500 | 499.370／499.923 | 0.972／1.340 | 3.102／2.810 |

原始总报告`verification.json`保留为false：停止拒绝用例首先被反馈陈旧触发，而旧检查要求先出现模式丢失；实际负停止回执、未确认状态和停止发送行为均保留正确。检查目标修正后，通过`artifacts/vendor_s2_stop_rejection_v1/`单独验证，并对原始事件和SHA独立复核；没有修改运动、碰撞、时间或停止阈值，也没有覆写原始false结果。

**未确认稳定500Hz工作资格。** `vendor_s2_release_v1`／`v2`同一最终伺服配置的500Hz第二批曾发生超过60ms反馈空窗，客户端正确停止。这些记录保留在独立复核中，不能用v3的六条成功覆盖。SDK发布均值接近500不证明主机采用／设备采样稳定500Hz，当前不推荐实机工作频率。

额外的`artifacts/vendor_s2_watchdog_v1/`通过：最后收到有效指令后约101.04ms触发一次看门狗，后续最大观测速度约`8.60e-6 rad/s`，未调用显式停止服务；该数据仅验证本机静止保持。九类故障与拒绝行为逐项核验，故障capture始终保持false。

可选overlay四包构建通过，原生力学源码未修改。最新**316项Python测试通过**，仅14条已有库弃用警告，证据为`artifacts/vendor_s2_tests/full.xml`。没有重跑旧114项冻结数值对照，也没有新增实机、物理网络或硬实时资格。中文视频为`presentation/vendor_read_only_zh.mp4`，1280×720、12fps、13.67秒，已解码查看真实帧；`frequency_response.png`和`summary.csv`分别给出图表和明细。

下一轮S3优先处理真实effort／current来源、尺度、采样／滤波／带宽及主机高频日志。运动摩擦、起动摩擦和惯量暂未开放；不得把S2位置执行成功当作完整动力学实机资格。
