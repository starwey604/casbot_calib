# 实机现场CLI和手动调参台

先用只读CLI看连接，再在GUI上逐轴拖动、读取反馈，最后尝试短轨迹。

2026-11-11按用户最新决定，GUI改为**直接五字段调参台**：删除逐轴确认勾选、实物名称填写、观察结论、备注、review存储与14轴审核门禁。不再要求上传现场前提表；现场人员决定物理防护和操作顺序。S3/S4正式观测资格独立保留；现场H1–H4按下文最新批次规则执行。

限制设置面板已接入：输入范围和反馈观察阈值独立，全轴默认＋逐轴覆盖。位置反馈越界始终只告警/记录；速度反馈阈值新默认1.5rad/s，手动GUI默认只告警，可选速度越限结束。现场CLI速度越限仍停发，运行时跟踪/老化跟踪门禁已删除。在线观察设置须经发送器版本确认，其它设置下次控制、Mock设备项下次启动生效。用法见[可调限制说明](../docs/fieldwork_manual_limits.md)。位置参考平滑/暂停仍属于[四轮计划](../docs/fieldwork_limits_behavior_plan.md)后续范围，不加入模型重力辅助。

准备使用H1–H4 CLI前，查看[现场CLI中断条件清单](../docs/cli_interrupt_limits_audit.md)：GUI设置不作用于这些入口；清单分别列出启动拒绝、运行停发、批次终止和分析拒绝的阈值、代码来源及待决定的改法。

## 现场批次不再做数据质量审核

H1–H4已删除静态稳定/覆盖、匀速平台筛选、50ms缺口、周期一致性及拟合残差等质量门槛；H2方向review入口、发送率90%门槛、静态质量重试预算也已删除。实际数据和可计算的统计保留，分析不可用不锁整批。需要但不存在的数值模型会让相应动作记为缺输入未执行，其它动作继续；不换模型冒充。运行故障、指令绑定与碰撞检查仍有效。新报告passed描述流程处理，采集是否齐全看collection_complete与jobs_not_run；详见[H2指南](../docs/hardware_batch_h2.md)。

## 采集零位与一键回零

“零位”是你保存的一组实际关节位置，不是14轴全0，也不写入机器人编码器偏置。CLI和GUI共用`fieldwork_zero.py`：

- 固定文件：仓库工作区的 **`fieldwork/zero.yaml`**，逐轴名称＋rad值，带采集UTC、模型、14轴顺序、域号与scope。重新采集原子更新同一个文件；采集失败保留原值。文件已加入Git忽略。
- 回零时冻结这组目标，从本次实际起点生成C3单程参考。默认过渡至少5秒，参考速度0.3rad/s、加速度0.5rad/s²、jerk 2rad/s³，可自动延长；另有profile的首尾hold。仍检查参考URDF范围、碰撞、通信和模式。
- 正常结束不再有回位误差、停稳持续时间或回位期限门禁。记录最后接收位置及逐轴误差，不用误差判失败；成功指参考已发送且模式已退出。
- 不加模型重力/摩擦前馈，effort为0。CLI默认Kp=20、Kd=2；GUI使用当前各轴Kp/Kd滑条值。实际位置不一定精确等于目标，结果看反馈。

### GUI

连接并收到实际位置后，点 **采集零位**。它保存当前反馈，不是滑条目标，不要求速度/effort回读。旧反馈超过profile接收期限时提示等待新反馈，不覆盖文件。

要回零，先停止正在发送的手动控制，再点 **回到保存零位**；旁边的 **回零/s** 调整最短过渡时间。回零拥有独立控制子会话，期间手动滑条禁用；**停止发送**、断开或关闭均可取消。完成后点“重读 / 新控制”再继续手动调参。只读连接持续运行，回零记录也可导出视频、保存会话。

### CLI / 脚本

实机连接示例（域号由现场确认，下面变量不会扫描发现）：

```bash
read -r -p "现场 ROS_DOMAIN_ID: " domain
bash fieldwork/capture_zero.sh --domain-id "$domain" --scope hardware_position_stream
bash fieldwork/return_zero.sh --domain-id "$domain" --scope hardware_position_stream --duration 5 --kp 20 --kd 2
```

等价入口为`bash fieldwork/run.sh zero-capture ...`和`zero-return ...`。需要切换配置时加`--profile 路径`；不需要`--output`或v1/v2名称。CLI记录自动放到`artifacts/fieldwork/zero_runs/<UTC_随机标记>/`。

本地Mock使用已有Mock的`client_profile.yaml`或**当前GUI工作区**的`protocol/profile.yaml`，同时传`--scope localhost_contract_mock --domain-id 181`；GUI自建Mock时保持该窗口打开。域号/scope/模型/轴顺序不同会提示重新采集，不自动接另一目标；这些字段不是物理设备身份。

采集入口只订阅位置；回零启动沿用现场起点检查：要求完整新鲜q/dq、全部速度<0.02rad/s和可用模式/服务，原生启动期限10秒。它与已删除的结束回位验收不同。当前GUI限制面板仅用于手动五字段发送，回零使用profile中的现场轨迹策略（默认反馈2秒/模式3秒、速度1.5rad/s越限停发）。SDK故障结束本次控制，不自动重试或恢复；只有申请模式/发指令前的起点变化可重新读状态、检查路径。

CLI每次留存采集或回零记录；GUI记录在当前缓存控制子会话内。零位YAML不会随缓存清理消失；每次回零另存所用零位副本，不受随后重新采集影响。

## 推荐：一条命令打开本地Mock调参台

```bash
bash fieldwork/run.sh gui --mock --domain-id 181 --software-rendering
```

不用先启动另一终端的SDK，也不用填写profile或输出目录。181是主动选定的本地测试域；`--mock`显式启动本窗口独占的私有SDK Mock并自动配对连接。窗口打开后点“连接 / 读取”，收到反馈后再操作滑条和“开始发送”。当前仍是原始五字段模式：本窗口Mock从近垂臂休息姿态初始化（左肩滚转+20°、右肩滚转−20°，其余0），待机、首帧前、活动及断流后均不添加模型重力补偿；物理重力仍正常存在。GUI不提供重力填值；另提供显式“采集零位 / 回到保存零位”，Kp/Kd和手动effort等滑条保持原逻辑。窗口关闭只清理自己启动的Mock；独立启动的服务不受影响。

`--software-rendering`显式选择软件渲染，适用于容器出现`iris`/`dri3`显卡驱动兼容错误；有可用显卡驱动时可以省略。它不改变控制或通信频率。

Wayland桌面可以通过XWayland运行X11版GLFW。GUI现在按GLFW实际创建的上下文选择PyOpenGL后端，不再由`WAYLAND_DISPLAY`猜测；无需手动删除桌面环境变量。独立3D/视频进程仍使用自己的EGL后端。软件渲染开关只选择绘图设备，不能代替这一上下文绑定。

### 显示性能

窗口输入/短轨迹绘图上限30 Hz、反馈运动10 Hz、静止2 Hz；监督独立约60 Hz。软件光栅默认串行。实时视图采用同源轻量显示网格，碰撞、动力学及视频仍用原模型。显示死区仅跳过不超过`1e-4 rad`的微小画面变化，数值和原始记录保持完整。500 Hz发送、Mock物理步长与SDK发布配置不变。设置和CPU实测见[性能说明](../docs/fieldwork_performance.md)。本轮有原生改动，更新后须重编vendor与原生核心，随后重启GUI。自建Mock默认省略私有物理诊断CSV，SDK原始记录保持；需要诊断时加`--mock-diagnostics`。

### 告警不再移动滑条

控制状态、缓存满、保存结果和连接提示统一显示在固定高度状态区；长内容在区内滚动。没有告警时保留同样高度，出现或恢复都不会挤动关节和五字段滑条。右侧模型/限制反馈说明同样固定占位，模型载入和限制版本回执不会推移Kp/Kd等滑条。更新Python代码后重新打开GUI生效，无需重编C++。

### 发送后出现模式超时

若提示`stale_mode`，先看对应运行记录。此前已修复同步写CSV导致约800ms发送停顿的路径，新手动控制CSV采用独立有界写盘；积压满或失败只暂停控制CSV并告警，手动发送继续。按2026-11-12用户决定，新现场profile的关节/模式回调超时改为2秒/3秒，真实超时仍结束控制；模式丢失和设备看门狗保留。已有工作区/显式profile保留原阈值，GUI中可修改“等待与通信”，下次控制生效。对照与具体范围见[超时排查记录](../docs/fieldwork_stale_mode_debug.md)。

### 在GUI中保存，而不是提前起文件夹名

- 默认自动建立**磁盘运行缓存**。点击“保存会话”，使用自动生成的名称或编辑完整目录，再点“保存”。现有目录不会被覆盖。
- 保存在线程中完成，**不中断发送或只读连接**。保存期间仍可拖动和停止；完成后显示正式目录。后续新数据继续留在运行缓存，再次保存会生成另一个快照。对照用的`recording/*.csv`在保存时从快照数据生成，不在自动缓存中反复复制。
- 在线保存只包含每个文件已完成写入的前缀；CSV/JSONL不带半行，CDR截到已保存消息引用的末尾。`saved_session.json`列出各文件截点与`live_prefix`/`finalized_prefix`/`capacity_paused`状态，不把它们当成同一设备采样时刻。
- 已保存目录可用`gui --session 保存目录`查看和导出视频，连接/发送禁用。快照中的活动控制不能恢复执行；原始封存配置与运行记录不改写。

### 恢复缓存和容量

```bash
bash fieldwork/run.sh gui --recover
# 或在窗口顶部“缓存记录”中选择另一份记录。
```

缓存默认在`${XDG_STATE_HOME:-~/.local/state}/casbot-calib/fieldwork/sessions/`，可用环境变量`CASBOT_FIELDWORK_CACHE`更换根目录。运行目录和ID自动生成，正常关窗或异常退出都保留；`--recover`选择最近未被其他窗口占用的缓存，也可跟缓存ID。恢复不自动连接或发送，可以直接另存已有数据。缓存的旧Mock端点不会自动改绑到新Mock；实时演练另开`gui --mock --domain-id 181`即可。

单次缓存默认512 MiB，可在启动时用`--cache-limit-mib 2048`等修改。后台每秒检查，达到上限后停止追加原始记录、控制CSV、进程日志及本窗口Mock诊断，不删除旧数据；实时反馈、手动发送和停止继续，自动短轨迹取消。可能超出一个检查周期内的新增量，固定最新值/视图及最终元数据仍可更新；这是应用层录制额度，不是文件系统硬配额。1kHz原始消息占用较快，长会话应增大容量。

达到容量后红条提示录制已暂停，保存现有前缀或另开新会话继续记录；不会静默覆盖旧数据。缓存不会在保存后或关窗时自动删除；完成核对后可自行清理对应缓存ID目录。全部缓存的累计占用不受单次额度约束。

### 连接已有服务或实机

原有入口保留，**输出目录现在可省略**：

```bash
# 独立启动的Mock必须使用它本次生成的profile，不能沿用已关闭实例的文件。
bash fieldwork/run.sh gui \
  --profile artifacts/fieldwork/manual_mock_v2/client_profile.yaml \
  --domain-id 181 --scope localhost_contract_mock --software-rendering

# 实机仍须现场确认域号并显式指定scope；不扫描域号或自动挑选设备。
bash fieldwork/run.sh gui --profile configs/hardware_field.yaml \
  --domain-id "$calib_domain_id" --scope hardware_position_stream
```

连接后显示实际目标topic、域号、发现的发布者数量与首次反馈等待时间。发布者为0时明确提示目标尚未出现，优先核对服务是否运行、profile是否属于本次Mock实例；相同域号不能替代私有端点绑定。

高级兼容用法：`--output 新目录`仍直接创建指定工作区，存在时拒绝覆盖；`--session 工作区`沿用原封存连接。显式工作区不使用自动缓存额度。下面保留只读CLI和双终端Mock的详细用法。

## 先用只读CLI检查连接

已准备好Humble环境后，在仓库根目录更新依赖并构建可选SDK记录器：

```bash
bash scripts/humble/container.sh python -m pip install -r scripts/humble/requirements.lock
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
bash fieldwork/run.sh --help
```

先向现场确认运控服务的实际域号，再显式输入。下面的命令仅订阅反馈、记录发现信息与本机配置，不切模式或发送运动命令。输出目录必须不存在：

```bash
read -r -p "现场确认的 ROS_DOMAIN_ID: " calib_domain_id
bash fieldwork/run.sh preflight --domain-id "$calib_domain_id" \
  --scope hardware_read_only --output artifacts/fieldwork/site_preflight_v1

bash fieldwork/run.sh observe --domain-id "$calib_domain_id" \
  --scope hardware_read_only --duration 30 \
  --output artifacts/fieldwork/site_observe_v1
```

`preflight`默认记录5秒，`observe`默认30秒。两者输出中文终端摘要、`report.md`和`report.json`，以及完整全轴原始CDR、消息、端点和QoS快照；未收到的非目标轴保持未知。`--config`可选择S1只读配置，默认`configs/vendor_s1.yaml`；需更换反馈topic或可靠性时另建配置和会话，不改正在采集的配置。

### 观测断线后的恢复

现场同意拔插等操作后运行：

```bash
bash fieldwork/run.sh reconnect --domain-id "$calib_domain_id" \
  --scope hardware_read_only --duration 120 --interactive \
  --output artifacts/fieldwork/site_reconnect_v1
```

先等到有效位置反馈，再由你实施现场批准的断开/恢复操作。输入并回车：`d`登记已断开、`r`登记已恢复连接、`n 备注`登记说明、`q`提前结束并保存。键入时刻只是人工标记，不是精确物理事件时刻；脚本不操作网卡、重启服务或恢复运动。也可不加`--interactive`，运行到设定时长后自动分析。

超时阈值默认使用S1配置的`stale_after_ns`（根配置250ms），可用`--timeout 秒数`在本次运行开始时显式覆盖。报告区分原始回调节奏、完整14轴有效位置反馈空窗、轮询实际检出时刻及恢复收数；无效位置消息同样会造成有效反馈空窗，不据此推断网络断线或丢包。端点快照约每500ms一次，DDS发现变化不等于物理断线时刻。

`q`是主动结束，只在记录器正常保存中断摘要与采集结果后接受；Ctrl-C或执行错误保留已有数据并报告未正常完成。软件不清故障、不申请控制，也不会重发旧命令。`reconnect`未观察到完整有效位置反馈中断—恢复周期，或结束时仍超时，会报告条件不足。

### 离线重分析与脚本集成

```bash
bash fieldwork/run.sh report artifacts/fieldwork/site_reconnect_v1 \
  --output artifacts/fieldwork/site_reconnect_analysis_v1 --json
```

重分析不启动ROS节点，输出必须在原会话之外的新目录。它核对已有采集摘要及配置/域号/scope绑定，不改原始记录。四个F1子命令支持`--json`，此时Python CLI的stdout只输出JSON、错误送stderr；与`--interactive`互斥。环境包装脚本的启动提示写到stderr。

退出码：`0`自动只读检查通过，`1`采集/结构/证据失败，`2`数据不足、目标不明确或未观察到所需恢复周期。通过只表示列出的自动检查；实物映射、物理方向、停止和标定资格另外确认，报告始终保持`motion_allowed=false`。

## GUI直接拖动各轴和五字段参数

### 安装和离线预览

```bash
bash scripts/humble/container.sh python -m pip install -r scripts/humble/requirements-gui.lock
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
bash fieldwork/run.sh gui --preview --output artifacts/fieldwork/manual_preview_v1
```

使用Python 3.10与Dear ImGui Bundle 1.5.2。预览不连接ROS；可加`--frames 150 --screenshot 输出.png`保存窗口截图。包装脚本沿用桌面已有DISPLAY/XAUTHORITY等显示配置，不放宽X访问权限。

### 双终端Mock兼容用法（需手工配对profile）

终端一启动同一套原生机械SDK Mock。181是本机主动选择的测试域，不是实机默认域：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib sdk-mock --config configs/fieldwork_mock.yaml \
  --domain-id 181 --output artifacts/fieldwork/manual_mock_v1
```

终端二打开GUI：

```bash
bash fieldwork/run.sh gui \
  --profile artifacts/fieldwork/manual_mock_v1/client_profile.yaml \
  --domain-id 181 --scope localhost_contract_mock \
  --output artifacts/fieldwork/manual_desk_v1
```

**当前手动Mock默认值**：`configs/fieldwork_mock.yaml`使用近垂臂休息姿态：左肩滚转`+0.3490658504 rad`（+20°）、右肩滚转`−0.3490658504 rad`（−20°），其它12轴为0，关闭活动和待机/断流的模型重力前馈；不关闭物理重力；保存零位的显式回零入口见下文。GUI首次连接仍用实际反馈初始化滑条，允许各轴自行调节Kp/Kd、速度及手动effort；PD作用与重力可能产生小的实际位置偏差。

这个起姿是数值筛选出的Mock休息姿态，不是出厂位认定。当前完整碰撞模型的静态最小间隙为11.81 mm；无补偿启动、左肩pitch小幅调节及停止的一组实际反馈采样检查最小11.87 mm。全零位腕部与髋部的重叠已避开。检查只覆盖起姿和所列代表范围，任意滑条操作的整段轨迹仍需另行检查；画面由用户核对。`configs/sdk_mock.yaml`及DVT2标定Mock保留原校验起姿和保护策略，不受这个手动默认值影响。

1. **连接 / 读取**：只读订阅并记录反馈，不创建命令发布者或模式客户端。14个位置滑条用首次完整位置初始化，当前值与本次控制起点分列显示。
2. 点击轴名选中该轴。左侧位置滑条可直接拖动；右侧调该轴的目标速度、Kp、Kd、前馈力矩。发送前的修改仅暂存；**开始发送**后拖动即更新SDK命令。Ctrl+单击滑条可精确输入数值。
3. 图中双臂按SDK实际反馈更新，不是播放滑条目标动画；其它关节保持模型姿态。Mock使用现有原生物理内核，重力、PD、前馈和耦合都会影响实际运动，不保证实际角度等于目标角度。
4. 可点击**所选轴短轨迹往返**，从该轴当前目标作正负往返并回到原目标；其余四字段和其它轴目标保持。顺序由操作者掌握，没有14轴审核表。
5. 启动条件未满足时，顶部显示**等待原因、全部相关轴、当前值、阈值和已等待时间**。例如速度需小于0.02 rad/s；超出时持续等待，不再因手动启动等待超过10秒而退出。条件满足后自动继续这次请求；**取消启动**后不会稍后自行启动。
6. **停止发送**只结束本次控制，自动整理可用记录；只读数值和模型继续更新。**断开连接**另行结束只读订阅；两者都不自动回位。随后点击**导出运动视频**生成`motion.mp4`，无需Mock仍在运行。
7. 一次控制结束后，点击**重读 / 新控制**，从当前反馈重新初始化滑条，再显式点击**开始发送**。这会建立新的控制会话，不清设备故障、不恢复旧会话或重发旧目标。

### 初始化和回读的含义

| 内容 | 初始化 / 显示来源 |
| --- | --- |
| 位置目标 | 首次完整14轴位置反馈；不是机械零位或旧目标回读 |
| 当前位置、速度、effort | SDK反馈；缺项显示“—”，不填零 |
| Kp/Kd | 优先首次可用生效增益反馈；没有时配置默认20 / 2，界面显示来源。首帧后若调试反馈才出现，只显示回读，不覆盖已开始发送的目标 |
| 目标速度 | 默认0；实测速度不是目标速度回读 |
| 前馈effort | 默认0 Nm；反馈总输出不能直接复制为前馈，不隐式加重力或摩擦 |
| 位置min/max | 随包URDF；SDK没有限位查询接口，不标为设备回读 |
| 机械零位、数字硬件电机ID | 当前SDK无对应查询接口；显示精确SDK关节名，序号仅表示14轴数组顺序 |

`JointStateData.msg`含`kp/kd`（不含灵巧手的紧凑数组），普通`sensor_msgs/JointState`没有。连接主反馈之外，同时只读订阅`/motion/debug/joint_state`补充增益；现场是否发布及何时可用以实际接收为准。位置、速度与增益不宣称同一采样时刻。

默认滑条范围：位置为URDF，速度为profile的`maximum_velocity_rad_s`，effort为±10 Nm，Kp为0–1000、Kd为0–100，可在“限制设置→目标输入”调整。这些是软件输入范围，不是设备实测限制；反馈观察范围/动作另设。无回读时的20 / 2只是较低的调试起始配置，并不保证能承托某个实际姿态或负载。

在所用的完整H1 profile内可添加以下配置；原有`collision_scene`等相对路径仍相对该profile解析：

```yaml
manual_tuning:
  kp: 20.0
  kd: 2.0
  effort_limit_nm: 10.0
  kp_max: 1000.0
  kd_max: 100.0
```

五字段按`position, velocity, effort, kp, kd`发送；模组本地PD叠加effort，Kp/Kd允许0。手动GUI显式禁用H1默认的nominal重力前馈，避免滑条以外的隐藏输出；原H1/H2命令的默认补偿不改。

### 实机入口

现场确认目标和域号后显式启动：

```bash
read -r -p "现场确认的 ROS_DOMAIN_ID: " calib_domain_id
bash fieldwork/run.sh gui --profile configs/hardware_field.yaml \
  --domain-id "$calib_domain_id" --scope hardware_position_stream \
  --output artifacts/fieldwork/manual_site_v1
```

“开始发送”会申请上身模式并下发双臂14轴，不填腰/头/手目标。域号不扫描、不猜测；GUI只在显式连接后启动ROS工作进程。实物限位、安装、非目标轴行为及防护由现场人员决定，原始全轴消息/CDR保留供核对。SDK未知项和正式标定门槛见[前置条件](../docs/sdk_hardware_prerequisites.md)。

自动缓存与手动指定的新工作区均可用`gui --session 工作区目录`重新打开，沿用封存连接和模型；不会自动连接或发送。此前`casbot_fieldwork_manual_v2`手动会话只能只读查看/导出，不迁移成新控制。更早的逐轴验收向导会话需使用原版查看；新GUI新建输出目录。

## 记录可直接用于Mock / 实机对照

运行工作区区分持续只读观测与每次显式开始的控制；“保存会话”复制其数据前缀并新增`saved_session.json`：

```text
protocol/                 # 工作区封存连接、模型和初始调参配置
observations/<编号>/      # 每次只读连接：全轴CDR、消息、端点、采集状态
controls/<控制编号>/
  protocol/               # 本次控制封存配置与UTC运行标记
  runs/<运行编号>/
    initial.json            # 首次位置、初始五字段及来源
    control_status.json     # 后端事件、首个故障；不被后续停止观察覆盖
    target_edits.jsonl      # 每次滑条/短轨迹目标修改及本机单调时刻
    latest.csv              # 最新目标邮箱；不是历史重放队列
    capture/
      execution/            # 原生发送命令、接收反馈、模式/中断事件
      raw_state/            # 全轴原始消息、CDR、发布者等
      result.json
    recording/
      commands.csv          # 实际发布时刻 + 五字段14轴完整数组
      feedback.csv          # 接收时刻、14轴q/dq、发布stamp、原始effort
      readback.csv          # 主反馈和debug分别留存q/dq/effort/kp/kd；缺值为空
      events.csv
      recording.json        # 轴顺序、模型、来源、时钟含义
      motion.mp4            # 点击导出后生成；SDK反馈位置的接收时间回放
operations/               # 后台操作请求、结果和日志
view/                     # 实时几何画面的最新帧，不修改封存采集
```

Mock与实机采用同一格式；先核对模型、轴顺序和初值，再对照相对接收时间下的位置/速度/输出及命令变化。各次运行的单调时钟原点不同，不按绝对数值硬对齐。SDK header是发布时刻，不是设备采样时刻；不能用这些记录宣称新帧采样率、缓存年龄或拆分后的单向延迟。所有收到的帧保留，不按数值相等去重。UTC标记只标识运行，不充当设备身份。

## 停止和故障行为

关闭窗口或点击停止会结束发送；不自动回位、不清故障、不重放旧目标。GUI通过管道发送存活/取消消息，独立监督进程在约2秒无更新、钟域失效或owner改变时中断控制；磁盘上的心跳文件仅作归档。窗口短时卡住时，只读观测仍接收反馈，窗口恢复后可查看。关闭窗口则清理它启动的子进程。

运行越限的红色状态区列出每个触发轴及实测值/阈值，并保留首因；后续“未观测到停稳”等状态另列，不覆盖原始原因。反馈/模式/通信故障终止**当前控制会话**，工作区和只读监视继续；硬杀后未封存的旧控制不继续。现场处置后使用“重读 / 新控制”显式开始另一会话。

只读录制写盘失败会显示“录制问题”、取消自动短轨迹，保留实时反馈；该记录不能视为完整采集。控制期间的专用原始记录器目前仍是控制依赖，若它退出，监督器会明确报告并结束本次控制。状态/停止管道与日志归档分离，目标邮箱仍是文件；这轮不把所有录制路径都宣称为完全解耦。

这是软件进程清理机制，不是物理急停。没有可用的厂商stop RPC；断流由设备自保护仍是采用的现场假设。现场停止距离及保护行为不能由Mock证明。

## 程序结构

- `fieldwork_cli.py`：argparse + 可选Rich，只读CLI与显式GUI入口。
- `fieldwork.py` / `fieldwork_analysis.py`：只读记录、连接事件和独立报告，保留F1用途。
- `fieldwork_gui.py`：滑条、回读与图像显示，只写最新完整目标邮箱；不承担500Hz循环。
- `fieldwork_control.py`：手动会话、默认值来源、值范围、记录导出与故障状态；无review/备注表。
- `fieldwork_worker.py`：SDK进程监督、独立几何渲染进程、离线视频。
- `fieldwork_display.py`：显示限帧、可重建邮箱、轻量CAD场景；不调度机器人命令。
- `fieldwork_observer.py`：持续只读SDK订阅，生命周期不依赖本次控制。
- `fieldwork_ipc.py` / `fieldwork_status.py`：管道心跳/停止/实时消息；统一中文状态、逐轴细节与首因保留。
- `position_executor.cpp`：新增显式manual模式，共用模式/反馈监督与500Hz发送；现场封存轨迹允许单程，不再要求结束回位验收；保留参考检查，旧S2私有回归独立。手动目标不是已通过连续碰撞证明的离线参考。
- 3D沿用MuJoCo几何renderer，只用运动学显示；机械推进仍由SDK Mock的原生内核负责。

原F1与前一版向导的历史验证保留在`artifacts/fieldwork_f1_delivery_v1`和`fieldwork_f2_delivery_v2`；不是当前滑条版验收。当前验证命令：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  env LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s '-screen 0 1600x1100x24' \
  python scripts/humble/verify_fieldwork_gui.py --output artifacts/fieldwork_manual_check_v1
```

该脚本只在私有localhost域内启动SDK Mock，通过实际鼠标操作检查五字段滑条、反馈运动、实时模型、短轨迹、停止和视频。`--scenario close`及`--scenario heartbeat`分别检查关闭窗口与GUI卡死后的清理；不连接物理机器人。结果只代表脚本实际执行的轴和范围。


## 休息起姿与移除补偿（2026-11-12）

当前起姿记录：`artifacts/fieldwork_rest_pose_v2/verification.json`；搜索与保留的15°近接反例在`artifacts/fieldwork_rest_pose_v1/`。优先选择只调整两肩滚转的简单姿态：±20°使腕部离开髋部，并为无补偿PD偏差留出空间；没有恢复名义重力前馈，也没有新增回零流程。当前静态与代表反馈采样检查通过，Kp/Kd可调；按用户要求不运行GUI或渲染，外观由用户验证。

原零位/无补偿实现记录`artifacts/fieldwork_zero_v1/verification.json`只验证当时的初始化与输出合同，其零位已被用户确认有碰撞，不作为当前起姿证据。旧GUI自动化脚本不充作本版画面验收。

## 自动缓存与手动保存验证（2026-11-11）

本轮汇总在`artifacts/fieldwork_cache_delivery_v1/delivery.json`，覆盖实际GUI的自动Mock配对、发送中保存、缓存容量、旧端点诊断、缓存恢复、快照显示与独立CDR解码；另验证父GUI被SIGSTOP时额度仍生效、被硬杀后仅清理其自有Mock。13项存储专项通过；原有滑条/短轨迹/视频与S1协议回归通过。已保存快照显示与缓存视频（48帧）实际查看；早期失败及修复记录保留在汇总中。验证只使用私有localhost Mock，不连接实机。

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  env LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s '-screen 0 1600x1100x24' \
  python scripts/humble/verify_fieldwork_cache.py --output artifacts/fieldwork_cache_check_v1
# --scenario crash单独检查GUI硬杀后的EOF清理与缓存保留。
```

## 历史：限制改造第1轮验证（2026-11-11）

汇总：`artifacts/fieldwork_limits_round1_delivery_v1/delivery.json`。该目录是本机产物，不保证新检出仓库存在。

- `fieldwork_r1_waiting_v4`：实际ImGui + 私有SDK协议夹具；缺速度反馈、两轴未停稳等待超过10秒、服务未发现均不发命令/模式RPC。取消后不再启动；新请求在条件恢复后自动继续。只对本测试只读记录器注入EFBIG，GUI告警且反馈继续。固定q/注入dq只验证协议，不作机械证据。
- `fieldwork_r1_fault_v2`：原生机械SDK Mock中默认0前馈仍触发速度越限；红条显示左右肩滚转实际值与0.75 rad/s阈值，后续未观测到停稳不覆盖首因，控制结束后反馈仍更新。截图已查看；这项确认可见性，不表示运动问题修好。
- `fieldwork_r1_heartbeat_v1` / `fieldwork_r1_close_v1`：GUI卡住3.5秒结束控制、保留只读观测；正常关窗停止并保存，停止状态之后没有新命令。
- `fieldwork_r1_sliders_v1`：实际拖动左腕yaw五字段、SDK接收与原生PD采用、短轨迹、停止、记录/视频通过。188帧、9.4秒视频已完整解码并查看代表帧。
- `fieldwork_r1_legacy_v1` / `fieldwork_r1_s1_v1`：原H1快照和左肩pitch 0.2°闭合往返，以及原S1两类消息/CDR/中断契约通过；不是完整H1–H4或S3/S4重验。117项相关Python回归通过，新增行为检查亦通过；交叠测试不累计声称全量验收。

启动等待与故障显示可独立复验（每次换新输出目录）：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  env LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s '-screen 0 1600x1100x24' \
  python scripts/humble/verify_fieldwork_round1.py --scenario waiting \
  --output artifacts/fieldwork_wait_check_v1
# 将--scenario换为fault，检查机械Mock的越限提示与持续观测。
```

第2–4轮仍待实施，61项处置进度见[逐项表](../docs/fieldwork_limits_dispositions.json)。早期信号注入失败和黑屏截图保留在汇总的失败记录中，不充作通过证据。所有运行均为私有localhost或离线窗口，无实机测试。

## 直接滑条版历史验证记录（2026-11-11）

汇总：`artifacts/fieldwork_manual_delivery_v1/delivery.json`。

- `fieldwork_manual_sliders_v5`：真实ImGui鼠标拖动左腕yaw的位置、速度、Kp、Kd、effort；SDK实际接收前三项变化，原生模组PD实际采用增益变化。首次位置/增益初始化、反馈运动、模型更新、C3短轨迹、停止和记录导出通过。导出的19.2秒、384帧MP4已完整解码并查看，画面带反馈来源与相对时间。
- `fieldwork_manual_final_close_v1`：原生发送器按固定时隙取最新值、不积压补发的最终版本，实际窗口关闭后停止并保存；`fieldwork_manual_heartbeat_v1`检查GUI卡死3.5秒后的独立监督终止，STOPPING之后无新命令。
- `fieldwork_manual_legacy_v1`：既有H1快照、封存单轴闭合参考和执行的代表回归通过；不把此项写成完整H1–H4验收。
- `fieldwork_manual_final_preview_v2/window.png`：最终离线窗口与模型已查看。渲染器保留完整CAD形状，关闭阴影/反射开销；软件渲染帧率不等于SDK发送或采样频率。
- 91项相关回归、32项相邻回归及最终24项控制专项通过，专项与前两组有交叠，不累加声称全仓库验收。

该历史版本曾使用名义重力填值演练；现版已移除该按钮，改为手动Mock休息姿态起步、全阶段无模型补偿，历史记录不回写。早期驱动器点击滑块手柄或亚像素拖动未改变部分值，已修正为实际拖动；全字段通过后，软件视频渲染又超过早期150秒测试预算，最终按600秒预算完成，旧失败目录不回写。上述均为私有localhost与软件窗口证据，没有实机、跨机、物理停止或正式标定验收。
