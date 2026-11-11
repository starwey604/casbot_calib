# 实机现场CLI和GUI联调指南

到场后先诊断连接、保留只读记录，再通过GUI逐轴核对实物关节与方向，最后测试短轨迹。工具应自动整理测量和证据，让操作者知道当前结论、失败原因与下一步。

2026-11-11用户确认本方案，补充决定：**短轨迹测试必须放在GUI逐轴映射与方向确认之后。** F1只读CLI、F2逐轴GUI及F3的受限短轨迹入口已接入；主动断流与100/250/500Hz比较的GUI向导仍待后续。`fieldwork/`集中提供现场入口与操作说明。

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

## 在GUI逐轴核对后测试短轨迹

### 安装可选GUI并先离线预览

GUI使用Dear ImGui Bundle，固定在具有Python 3.10二进制wheel的1.5.2版本；不升级Humble的Python。额外依赖与只读CLI分开安装：

```bash
bash scripts/humble/container.sh python -m pip install -r scripts/humble/requirements-gui.lock
bash fieldwork/run.sh gui --preview --output artifacts/fieldwork/gui_preview_v1
```

预览不连接ROS，连接和运动按钮全部禁用，图中姿态明确标作名义模型示意。可加`--frames 60 --screenshot 输出.png`保存实际窗口图像。包装脚本传入宿主已有的DISPLAY、XAUTHORITY等显示配置，不放宽X访问权限；桌面显示不可用时先解决容器的显示连接，不能通过猜DISPLAY或关闭认证处理。

### 先用同一个SDK Mock演练

终端一启动隔离Mock；181是这里主动选择的本机试验域，不是实机域号：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib sdk-mock --domain-id 181 --output artifacts/fieldwork/gui_sdk_mock_v1
```

终端二使用它生成的连接profile，不自动启动或切换另一套Mock：

```bash
bash fieldwork/run.sh gui \
  --profile artifacts/fieldwork/gui_sdk_mock_v1/client_profile.yaml \
  --domain-id 181 --scope localhost_contract_mock \
  --output artifacts/fieldwork/gui_mock_session_v1
```

界面按一次一个轴操作：

1. 读取实际起姿，选择左/右臂关节。
2. 填写实物关节名称并确认已经辨认；设置本次幅度。
3. 规划单轴正反往返，查看同源模型正端点、局部正轴、位置曲线和完整路径检查结果。
4. 明确勾选已检查预览与现场条件，再执行本次计划。执行期间显示接收位置曲线与接收间隔；模型图片是标注的已记录起姿/计划端点，不伪装成实时运动。
5. 执行与分析通过后，填写观察者、映射和方向结论，保存该轴确认。发现不一致则填写备注并终止会话，不自动改符号重试。
6. 重新读取起姿，再检查下一个轴。每份计划只执行一次，用于执行的快照不再次用于规划。

幅度默认0.5°，本工具仅接受(0,1]°；这是软件上限，不是现场安全幅度保证。GUI不开放任意力矩、PD编辑或拖动即下发的滑块。

全部14轴确认后，短轨迹区域开放所选单轴、同侧肩肘、双臂肩俯仰三种组合。先重新读起姿、规划，再明确确认执行；轨迹为有界C3过渡的正端点、负端点、回起点，未选轴保持本次实测角度。后端在规划和执行两个入口都核验14轴证据、会话/模型/域号绑定，缺轴、旧证据变化或未封存执行均拒绝，不仅是界面按钮置灰。

### 实机会话先登记现场前提

先阅读[SDK前置条件](../docs/sdk_hardware_prerequisites.md)。GUI不扫描域号，也不从只读采集推导运动许可。未提供现场条件记录时，实机会话仅开放只读起姿，运动保持锁定。

复制`profiles/site_conditions.example.yaml`，由现场操作者与厂家/安全人员记录三组结论：急停/支撑；真实限制/几何/安装重力/负载；模式及未指定腰/头/手轴保持。每组填写`status: site_confirmed`、具体`conclusion`及`evidence`文件列表，路径相对该YAML。非目标轴行为须明确为`non_target_behavior: hold_prior_target_output`；释放、缺省0或未知均不符合现有执行器前提。模板中的unknown不会放行；物理真实性由现场核验，软件仍保留`physical_motion_qualified=false`。

取得现场证据后，在新目录显式建立会话：

```bash
bash fieldwork/run.sh gui --profile configs/hardware_field.yaml \
  --domain-id "$calib_domain_id" --scope hardware_position_stream \
  --conditions /path/to/completed_site_conditions.yaml \
  --output artifacts/fieldwork/gui_site_session_v1
```

此入口中的执行按钮会真实申请上身模式并发送双臂14轴命令。非目标轴不填充命令，原始全轴CDR仍保留。当前真实几何/夹具/保护尚无实测验收，不沿用Mock演练结果填写现场证据。

已有会话可用`gui --session 会话目录`重新打开，沿用封存的profile和域号，不允许同时传入新连接参数。GUI不会因打开会话而自动执行计划；故障或状态不明的未封存执行会话仍禁止继续运动。协议复用H2的实现/SDK/原生扩展绑定，另绑定GUI、工作进程和新状态机；变更这些实现、profile、模型、安装或现场前提时新建会话。旧开发会话缺少实现绑定时只读查看/导出，不自动补身份或开放运动。

### 停止、关闭与卡死如何处理

“结束试验/停止发布”请求现有发送器中断，不是硬件急停，不自动回位。界面逐帧更新前端存活标记，后台独立进程在现有SDK监督循环中检查；超过2秒未更新、钟域不符或标记不可读时终止命令流并锁定会话。同一会话仅允许一个GUI窗口；每个窗口独立的owner标记防止重开窗口替旧执行续命。窗口正常关闭请求受控中断；只有收到对应完整中断结果且无其他SDK故障时，才允许重新读起姿、创建新尝试。

执行、通信、模式故障或GUI心跳丢失记录终止事件，清故障能力保持未实现，不自动恢复。记录器与发送器仍保存可用原始数据；观测静止不冒充物理停止证明。后续实机保护与断流试验另按现场条件实施。

GUI会话保存`protocol/`、`snapshots/`、`attempts/`、`reviews/`、`events/`、`operations/`和`view/`。其中protocol封存模型/连接/现场证据，attempt含input、计划、执行CDR、分析和outcome，review绑定具体采集manifest；operations保留每次GUI工作进程的请求、结果和日志。模型预览图片在view中，不回写已封存采集。点击顶部“保存现场报告”会在`reports/UTC标记/`生成中文Markdown和JSON；终止会话也可导出，参考拒绝和受控中断分别列出。

## CLI架构与依赖

F1四层分工：`fieldwork_cli.py`负责argparse/Rich显示与参数，`fieldwork.py`负责会话/人工事件/记录器生命周期，已有`vendor_capture.py`和C++ `state_recorder`负责订阅与原始留证，`fieldwork_analysis.py`独立生成诊断和报告。F2的`fieldwork_gui.py`只处理交互，`fieldwork_worker.py`按次执行操作，`fieldwork_control.py`拥有会话状态机、方向证据和短轨迹门禁，复用H1与SDK监督循环，不复制控制循环。

命令解析沿用标准库`argparse`，交互输入采用逐行操作标记；新增可选依赖Rich 14.1.0显示中文表格和状态，不引入Typer/Click或新的ROS Python采集循环。Rich已纳入Humble依赖锁，也提供`fieldwork`安装extra；既有标定入口不强制依赖Rich。安装项目后另有`casbot-fieldwork`入口，`fieldwork/run.sh`通过模块运行，不依赖重新生成console script。

## 开发顺序与每轮交付

| 轮次 | 交付 | 完成条件 |
| --- | --- | --- |
| F1 只读CLI，已实现 | 环境预检、端点/QoS检查、全轴原始采集、只读断线恢复观测、中文报告 | 不创建运动publisher、不切换模式；能区分连接故障，保留中断前后数据；验证范围见文末 |
| F2 逐轴GUI，已接入 | Python ImGui操作台、受限单轴动作、实物映射/方向人工确认、证据绑定 | 每轴独立选择、预览、确认和执行；14轴确认后才开放短轨迹；本机验证范围见文末 |
| F3 短轨迹已接入，其余向导待续 | 受限短轨迹；待补主动断流/模式试验、100/250/500Hz比较向导 | 消费F2有效确认，检查完整路径；故障后无自动恢复或旧轨迹重发 |

F1可独立带去现场使用，不等待GUI。F1可以统计只读反馈接收频率；需要实际运动的频率比较和响应偏移测试属于F3，不提前到GUI之前。F2中的受限单轴动作是建立方向证据的专用试验，不是开放任意轨迹执行。

本次出差优先完成连接、模式/保护行为、14轴映射与方向、短轨迹执行验证。后续动力学采集继续使用既有H1–H5路径，并分别满足观测与标定资格。

## 复用已有采集与执行路径

| 能力 | 已有入口或代码 | 工具包需要补的部分 |
| --- | --- | --- |
| 只读原始记录与端点QoS | `vendor-capture`、`vendor_capture.py`、`state_recorder.cpp` | 诊断向导、全轴对照、断线事件与报告 |
| 实测起姿 | `hardware-snapshot`、`hardware_field.py` | GUI快照与检查结果呈现 |
| 单轴往返及完整参考检查 | `hardware-plan-jog`、Pinocchio/Coal | 逐轴交互与人工观察绑定 |
| 上身位置发送和反馈/模式保护 | `hardware-execute`、`position_executor.cpp` | 操作授权、GUI存活监督、试验状态显示 |
| 频率与整体响应分析 | `hardware-analyse`、`vendor_position_analysis.py` | 统一测量窗、图表与现场结论 |
| 方向review、分档比较、批次封存 | `hardware_batch.py` | GUI逐轴确认接入；不直接采用默认连续执行14轴后统一review的操作方式 |
| 只读姿态与回放 | `vendor-view`、`vendor_viewer.py` | 操作台联动与证据定位 |

GUI只做操作台，Python负责试验编排，现有C++发送器拥有命令循环；不在GUI帧循环里发送500Hz指令。继续复用现有机械内核、碰撞和模型上下文，不复制控制器或辨识算法。

相关约定以[SDK实机前置条件](../docs/sdk_hardware_prerequisites.md)、[H1执行路径](../docs/hardware_execution_path.md)、[H2批次规则](../docs/hardware_batch_h2.md)为准。原S2 localhost限制及S1/S3/S4资格保持独立。

## 现场试验顺序

### 1 环境与连接预检

操作者先确认目标机器人、网卡/网段及实际域号。CLI记录Humble、SDK overlay、RMW、选定的网络配置、topic/service类型、端点发布者与QoS，并检查反馈名称、字段长度和数值有效性。仅采集相关配置，不整份导出环境变量或凭据。

诊断至少区分：环境未加载、未发现端点、发现端点但无数据、QoS不兼容、消息结构不符、目标不明确。没有数据不能单凭这一现象认定域号或QoS错误；无法定位时保留候选原因与证据。

`ROS_DOMAIN_ID`是显式输入，绑定本次计划和记录：

- SDK的`environment.md`说明72仅为示例，`real.md`要求替换为实际域号；`setup_env.sh`不替操作者设置。
- 不能由SDK没有给出实际值推断机器人使用默认0。优先由现场人员确认，必要时核对机器人运控服务的启动脚本、系统服务或容器配置。
- 当前终端的`printenv ROS_DOMAIN_ID`不证明机器人运行进程使用相同值；SDK没有域号查询接口。
- 不遍历域号自动选择设备；`topic list`和`doctor`只记录所选域的发现结果与本机配置。网络拓扑标为本次会话观测。
- 跨机运行需正确配置DDS网络并允许非localhost通信；不自动修改网卡、防火墙或已有服务。

### 2 只读采集与连接恢复观测

不由本工具发送运动命令时，由操作者在现场同意的条件下拔线/恢复连接或重启指定服务。脚本只提示操作、登记人工事件并持续观测，不自动关网卡或重启机器人服务。

记录最后反馈、超时检出、端点消失/重新发现、第一条恢复反馈，以及前后全轴名称、模式与发布者变化；说明是否需要重新启动采集进程。分别报告反馈空窗、超时检出耗时和恢复后收到数据的等待时间。人工拔插事件标明人为记录精度，不冒充精确物理断线时刻。

**只读连接恢复与运动恢复分开。** 可以自动观测重新发现和恢复收数；不会因此重新申请控制、清故障或继续旧运动。接收重新出现也不证明机器人已安全停止或允许运动。

### 3 模式与未指定轴行为

进入GUI单轴试验前，操作者与厂家/现场安全人员先明确急停、安全处置、实际限位、起姿、安装重力、工具和腰部固定条件。模式切换等主动操作需独立确认，不属于F1只读预检。不能用未知停止行为或零PD试探安全性。

只发双臂14轴已获用户允许，不填充腰/头/手命令。腰部固定时未指定轴究竟保持原目标/输出、释放或被当作缺省0，仍未实测选定；现有profile按保持配置。应先取得厂家配置说明，再在受控条件下核对：

- 模式切入前后；
- 首条命令前后；
- 停止命令流前后。

保留完整全轴CDR和解码记录，独立比较非目标轴位移、可用effort、模式和故障。14轴规范化产物不足以证明腰/头/手保持；机械夹紧下无位移也不能单独证明驱动未持续出力。实测若推翻保持前提，停止依赖该前提的执行阶段，修订方案后建立新计划。

### 4 GUI逐轴映射与方向确认

SDK的`UpperJointData`、`JointStateData`使用`name[]`，没有数值电机ID字段。界面核对链为：**SDK名称 → 模型关节 → 实物关节 → 预期正方向**。如有厂家提供的驱动器ID表，另外登记来源；不把数组下标当成电机ID。

每轴操作：选择关节并辨认实物 → 读取新起姿 → 预览预期正方向和完整路径 → 确认受限幅度/速度 → 明确授权执行 → 分析全部轴响应 → 填写人工结论。默认一次只执行一个轴的检查，不自动轮转未确认轴。

自动分析判断是否确有响应、回位及是否出现非预期轴运动；实物关节与物理正方向由具名观察者确认。编码器随指令增大不等于物理方向正确，方向检查也不等于机械零位标定。反馈字段缺失时如实显示，不填零。

人工结论支持一致、不一致、无法判断，绑定本次attempt与原始证据；错轴、反向或无法判断阻断后续轨迹。不自动翻转符号再试。映射、模型、安装/控制条件改变后重新核对受影响项；新运行显式引用有效证据，不自动套用旧review。

首版采用受限动作按钮，不采用拖动滑块即下发目标，不开放任意effort、零PD或增益编辑。现有1°是规划默认值，不作为现场安全幅度承诺；实际幅度、速度、保持增益与重力前馈策略由现场条件确定并封存。

### 5 主动断流与停止行为试验

F3在F2确认及现场安全前提具备后，分别测试正常结束、受控主动中断、命令断流、反馈空窗及模式丢失；各试验单独授权，不把全部故障注入串成一次自动运动。具体注入方法先在隔离Mock验证。

记录故障后最后命令、是否继续发布、模式、可用反馈与现场观察。软件显示“结束试验/停止发布”，不能标作硬件急停；通信中断造成不可观测时，物理状态写为无法判断。遥测静止不替代物理停止证明，不安排故障后的自动回位。

SDK执行、通信或模式故障终止整批，`clear_fault`只log，原故障会话禁止resume。人工处置完成后，新会话重新读起姿并复核条件。正常检查点、受控主动中断和正常回位后的有限静态质量重采，继续沿用H2各自规则；硬杀/未封存且状态不明不当作正常恢复。

### 6 频率比较与短轨迹

**只有GUI完成全部14轴的有效映射与方向确认，才开放本节执行。** 每次还需复核实际起姿、模式和现场运动前提。CLI后端同样执行门禁，不能只把GUI按钮置灰；即使某条短轨迹只激励一个轴，也不绕过这一步。

先做单轴平滑往返，再做小范围多轴协调、双臂短轨迹，最后才考虑静态姿态序列。每次预览完整接近、动作、正常回位路径并做连续检查；不直接使用Mock home、合成夹具余量或完整标定激励。初始轴映射尚待验证时的专用小幅检查，不能把模型碰撞证明当成实物映射已正确的证明。

频率比较复用100/250/500Hz、同连续参考和同锚点、多次重复及随机顺序，不按频率重新拟合机械模型。测量窗、门槛、重复数和保护参数在执行前冻结，保留失败试次；不自动降频后宣布原档通过。

输出实际发布/接收频率、间隔p50/p95/p99和最长空窗、调度跳槽、未对齐跟踪误差与整体轨迹偏移。若有已确认的只读查询接口，可另外测服务请求—响应耗时，并标明该接口。

频率与延迟的解释遵循：

- 发布stamp是发布时间，不填为采样时刻；本地接收时刻和钟域另外保存。
- 1000Hz发布不证明1000Hz新采样或命令采用；相同数值不据此去重。
- 空窗不直接报告为网络丢包率；服务往返耗时不除以二当单向延迟。
- 整体轨迹偏移包含插值、缓存和机械响应，不报告为单向网络延迟。
- 高频主机日志、新帧序号和缓存年龄已明确不能提供，记为明确不支持；不反复等待，也不降低既有4kHz等资格门槛。动态摩擦/起动/惯量的正式资格仍阻断，替代观测另立协议验证。

## GUI布局与后端约定

- 顶部：目标会话、域号、当前模式、最后反馈接收距今多久、当前允许运行的试验；该接收间隔不标为设备数据年龄。
- 左侧：14轴确认进度；非目标轴另设只读页。
- 中间：选中实物/模型关节、预期方向、动作预览及实测曲线；复用只读几何显示。
- 右侧：人工观察、具名确认、备注及照片/视频附件。
- 底部：事件时间线、结束当前试验、报告位置。

GUI不能成为第二个命令循环。后端监督GUI存活；窗口关闭、卡死、后端退出、反馈断流时的行为须在实现中明确，并先经Mock测试，不能留下失去操作者监督的持续发送。输入焦点变化、重复点击和重开窗口不应重发动作。失联后界面保留最后姿态但明确标记不可观测，不外推运动。

## 目录与报告约定

现有现场文件为`README.md`、`run.sh`和`profiles/site_conditions.example.yaml`；检查清单及专用薄脚本按后续实际需要加入，不创建不可执行的占位命令。原计划布局如下：

```text
fieldwork/
  README.md                 # 本计划与后续现场操作入口
  checklist.yaml            # 试验项、依赖及自动/人工判据
  profiles/                 # 现场配置模板
  run.sh                    # 统一薄入口
  scripts/
    preflight.py
    observe.py
    reconnect.py
    axis_check.py
    timing.py
    trajectory.py
    report.py
```

可复用编排与GUI模块放在`python/casbot_calib/`，脚本调用包内能力。环境沿用Humble与现有容器/vendor脚本，GUI依赖保持可选，不增加只读CLI安装负担。出差机准备应验证同一入口可用、模型/LFS资源齐全、显示转发及输出权限；首版不依赖现场临时下载。

每次通过`--output`指定新会话目录，建议放在`artifacts/fieldwork/`。F1保存`session.json`、`observation.yaml`、`environment.json`、`capture/raw/`、`events/`、`report.json`和`report.md`；启动失败另存`capture_error.json`。`measurements/`、`reviews/`留待后续阶段按需增加。原始证据与已封存记录不覆盖；重分析另存目录，原始全轴记录和14轴分析结果分开。

会话复用`run_marker.py`的UTC标记，保存所选domain、scope、模型与14轴具名顺序、实际试验参数及现有数据/模型绑定；时间戳仅标识运行，不充当设备身份。设备身份政策仍按SDK前置条件执行，不重新增加版本字段门禁。

每个试验给出结论、测量值及单位、测量窗/时钟含义、适用条件、证据路径、人工确认、失败影响与下一步。执行结果分为通过、失败、待人工确认、明确不支持、条件不足；人工说明、配置假设和实测分别登记，能力状态沿用SDK清单。总报告突出下一项可做的操作，不以一个总`passed`覆盖未测或不支持项。

## 后续实现验证与责任

代码开发与自动验证由开发方负责；实物关节/方向观察、目标机器人与域号确认由现场操作者负责；腰部驱动、限位和保护处置由现场操作者与厂家/安全人员共同明确。软件开发测试不连接物理机器人，实机命令只在用户显式调用对应入口时执行。

F1验证无运动输出、反馈缺失/结构错误、QoS诊断、断线后恢复收数、未知字段和全轴记录保留。F2验证实际GUI渲染与关键交互、逐轴人工review绑定、错轴/反向拒绝、旧条件不能冒用、重复点击及GUI存活监督。F3验证前序门禁、故障终止/拒绝resume、无旧命令补发、完整参考/观测检查及报告时间含义。自动测试只声明实际执行的Mock范围，物理停止、跨机性能和实机标定由现场证据另外验收。

### F1本机验证记录

2026-11-11完成以下验证，汇总在本机`artifacts/fieldwork_f1_delivery_v1/delivery.json`：

- 87项相关Python测试通过：新F1、S1、位置流程、H1及H2；结果为`artifacts/fieldwork_f1_tests_v2.xml`。不包含历史全量数值交付。
- `artifacts/fieldwork_f1_ros_v2/verification.json`：统一SDK机械Mock只读预检、反馈中断/恢复及零控制请求；另用状态协议夹具验证两种反馈类型、全轴CDR、发布者重建、无效字段、QoS不匹配、错误类型、无数据/无端点、多发布者及中断保留。检查CLI记录器没有应用publisher或service client。
- `artifacts/fieldwork_f1_interactive_v2/verification.json`：连续输入人工断开/恢复/备注和`q`的实际进程试验，检查超时/恢复事件与主动结束保存；终端中文表格和全轴表已读回查看。
- `artifacts/fieldwork_f1_s1_regression_v1/verification.json`：旧S1两种消息、只读显示源、CDR重解码和主动中断回归通过。

初次F1 ROS验证因节点尚未发现时查询其client清单而中止，修正测试夹具的发现竞争后在v2完成。交互v1暴露文本流预读导致连续输入的`q`留在缓冲区，已改用文件描述符分段读取并补回归；v2实际交互通过。失败目录保留。以上均为私有域、localhost内的Mock/协议测试，尚无物理机器人或跨机网络证据。

### GUI和短轨迹本机验证记录

2026-11-11在Xvfb实际窗口及私有localhost SDK Mock内验证，未连接实机：

本机交付汇总为`artifacts/fieldwork_f2_delivery_v2/delivery.json`。

- `artifacts/fieldwork_f2_frozen_gui_v1/verification.json`：封存完整实现绑定后，通过真实ImGui鼠标/文本输入逐轴完成14次SDK往返和确认，随后通过双臂肩俯仰短轨迹；另在命令流运行中停止GUI帧3秒，后台因2秒存活超时终止并锁定会话。测试输入均明确登记为Mock软件检查，不作为实物方向证据。前序开发记录`fieldwork_f2_gui_v5`保留，不补写实现绑定。
- `artifacts/fieldwork_f2_owner_stop_v1/verification.json`：实际点击停止按钮中断SDK流，验证完整受控中断结果与重新读起姿要求。
- 早期Stop/Close专项分别接收88/86条命令后中断，却误用了全流程的“超过100条”计数条件；已按专项预定的30条触发点区分判据。原失败报告保留，`artifacts/fieldwork_f2_interrupt_review_v1/verification.json`对相同封存记录离线复核通过，包括STOPPING后无新命令；不把专项计数当成吞吐验收。
- `artifacts/fieldwork_f2_frozen_report_v1/verification.json`：在已终止的封存14轴会话中实际点击导出报告，无须重启SDK或恢复运动。实际截图在上述各目录的`window.png`，已查看窗口渲染和关键状态。
- `artifacts/fieldwork_f2_tests_v3.xml`：121项相关回归通过，涵盖F1、S1、原位置/H1/H2、SDK Mock和GUI控制门禁。最终24项控制/绑定专项见`artifacts/fieldwork_f2_bound_tests_v1.xml`，与该回归有交叠，不累加为全量验收数量。

初期窗口检查发现Hello ImGui的全屏边距使用em而非像素，导致按钮被裁切；已修正边距并由完整14轴窗口操作验证。旧字体API/资源路径尝试、规划事件参数冲突及预览scene重复创建的拒绝记录保留在早期测试目录。当前模型画面复用既有只读MuJoCo几何renderer，命令与机械计算仍走原有C++/Pinocchio/Coal路径。

初次汇总误用浮点逐位相等检查未激励臂轴的参考常量，最大差为8.88e-16rad；已按原有对应单元测试的1e-12rad绝对容差重评分，未改数据或控制。v1汇总false保留，v2汇总通过；该容差仅检查参考数值保持，不表示实物跟踪精度。

后续继续F3的主动断流/模式试验和频率比较向导；实物方向、保护行为与正式标定资格由现场证据分别验收。
