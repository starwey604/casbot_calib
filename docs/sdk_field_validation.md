# 机器人到场：SDK 能力测试与动力学起步

到场先做**只读留证**，然后用厂家确认的操作验证模式切换和断流，再做单轴往返。位置链路走通后，优先做 **H2 静态重力探索**。正式重力拟合另需可信力矩与采样证据；现场 H3/H4 可按记录的假设开展探索、保存结果；正式动态摩擦、起动摩擦和惯量验收仍需合格观测或独立验证的替代协议。

| 作业 | 用现成什么 | 结果决定什么 |
| --- | --- | --- |
| 到场前 | Humble、SDK overlay、隔离 SDK Mock | 工具链是否就绪 |
| 只读 | `vendor-capture`、`vendor-normalize`、`hardware-snapshot` | 目标端点、全轴反馈覆盖、14 轴映射与静止起姿 |
| 厂家辅助验证 | 原始 CDR + 厂家模式／驱动诊断和现场操作 | 能否安全接管、只发 14 轴、结束指令流 |
| 单轴与 H2 | `hardware-plan-jog` → `hardware-execute`；方向、频率、静态批次 | 何时可扩大运动；能否得到静态重力探索候选 |
| 尺度与正式观测 | 独立已知载荷试验；离线 S3／S4 | 哪个阶段有可用输入，下一处应补什么 |

[SDK 能力与前置条件](sdk_hardware_prerequisites.md)是证据和安全边界的依据。下面的真实运动命令只由现场人员显式执行。输出目录每次用新名；发生 SDK／通信／模式／运动保护故障立即结束整批，清故障占位只记日志，故障会话不得续采。

## 1. 到场前：验证工具链

在仓库根目录操作。初次安装见[启动指南](dynamics_getting_started.md)；已安装的机器不必为只读采集重跑历史全量回归。

```bash
bash scripts/humble/setup.sh --doctor
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
bash scripts/humble/container.sh casbot-calib hardware-snapshot --help
# 推荐在到场前完成：隔离 SDK Mock，检查 H1/H2 客户端和故障路径
bash scripts/humble/container.sh python scripts/humble/verify_sdk_mock.py \
  --h2 --output artifacts/site_mock_h2_v1
```

保留 Mock 验证报告，现场用于排查软件差异。H3／H4 合成验证入口分别为 [verify_hardware_friction.py](../scripts/humble/verify_hardware_friction.py) 和 [verify_hardware_inertia.py](../scripts/humble/verify_hardware_inertia.py)；它们不是实机测试脚本。 当前 SDK 双腕回调观测的专项探索见[运行与结果](sdk_wrist_h4_exploration.md)，同样只连接隔离本机 Mock。

## 2. 到场只读：先记录全轴，再取静止起姿

请现场负责人给出实际域号，并核对目标机器人、网络和是否有其他控制源。不要扫描猜测域号。下列 ROS 查询显式使用与 SDK 客户端相同的环境；域号不写进长期默认配置。

```bash
read -r -p "现场 ROS_DOMAIN_ID: " calib_domain_id
site=artifacts/site_session_v1
mkdir -p "$site"

site_ros() {
  bash scripts/humble/container.sh env ROS_DOMAIN_ID="$calib_domain_id" ROS_LOCALHOST_ONLY=0 \
    bash scripts/humble/vendor.sh "$@"
}

site_ros ros2 doctor --report > "$site/doctor.txt"
site_ros ros2 topic list -t > "$site/topics.txt"
site_ros ros2 topic info -v /joint_states > "$site/joint_states.txt"
site_ros casbot-calib vendor-capture --config configs/vendor_s1.yaml \
  --domain-id "$calib_domain_id" --scope hardware_read_only --duration 10 --output "$site/s1_raw"

bash scripts/humble/container.sh casbot-calib vendor-normalize \
  "$site/s1_raw" --output "$site/s1_normalized"
```

- 默认订阅 `/joint_states`。若发现的类型、QoS 或覆盖范围不同，复制 [S1 配置](../configs/vendor_s1.yaml)，按实测修改 `joint_topic`、`joint_type`、`reliable`、`depth`；只订阅当前已发布的通道。可用 `debug_topic` 附加已存在的 `JointStateData`，不要为了找反馈主动切全身模式。
- 看 `s1_raw/raw/packets.jsonl` 的 `message.name` 和实际字段，保存同目录 `cdr.bin`、`session.json` 及端点清单。**原始记录保留消息里的全部轴；规范化只取配置中的 14 轴。** 若选定反馈不含腰／头／手、目标或驱动状态，记为未观测，另取厂家诊断；位置不动（尤其机械抱死时）不能判定驱动保持。
- `s1_normalized/report.json` 用来检查 14 轴位置有效包、到达节奏和空窗。固定参考下核对具名姿态和单位；物理方向留到小幅试验，机械零位另做外部基准核对。若话题是 best-effort-only，S1 可改订阅 QoS，但现有 H1 执行器固定 reliable；如无兼容端点，先补执行器 QoS 适配并在隔离 Mock 验证，再取 H1 快照或发命令。

H1 的快照要求同一显式域、完整 14 轴位置／速度、静止和近期模式反馈：

```bash
site_ros casbot-calib hardware-snapshot --config configs/hardware_field.yaml \
  --domain-id "$calib_domain_id" --output "$site/snapshot"
```

核对 `snapshot/result.json`、`snapshot/execution/snapshot.csv` 和 `snapshot/raw_state/raw/`。设备序列号、固件与运控版本不作为当前软件门禁；`run_id`（有提供的报告）或会话目录用于找记录，不作设备身份。正式接入实机数据前按[现场条件](sdk_hardware_prerequisites.md#现场条件与接口可读性2026-11-09-用户确认)重新建立设备绑定。

## 3. 发首帧之前：把不能用只读回答的问题交给正确的试验

进入上身模式、发首帧、断流都是不同事件。S1 能在旁记录，但不会触发这些事件；H1 会切模式并真正发指令，**不能用一次 H1 往返代替首次安全确认**。

1. **先取可核验的运行条件。** 现场负责人和厂家核对上身调试可用模式、当前控制源、实际位置／速度／力矩限制、急停及失联保护、真实几何／夹具／安装重力与负载；确认腰部抱死时允许的驱动设置。软件停止发布不等于物理急停。
2. **再做厂家主持的受控检查。** 在厂家明确可安全复现的支撑、防护和急停条件下，由厂家用其认可的模式切换、首帧和断流操作，旁路持续运行第 2 节的 `vendor-capture`（时长覆盖操作和退出），记录三个事件前后原始全轴 CDR、目标／输出／状态与故障诊断。记录手段应能区分未列轴**保持原目标／输出、释放、还是被当作 0**，以及抱死后是否持续异常出力。没有可安全操作的方法或诊断分不清，就保持未知，停在只读；本仓库没有独立的模式／首帧／断流物理验收脚本。
3. **只在现有前提吻合后进入 H1。** 允许只发 14 轴已经是接口约定；实机仍须确认未指定轴的实际行为、首帧前输出与失联保护。若发现释放、缺省 0 或异常出力，先由厂家确定驱动设置并修改相应执行策略与模型，再重新验证；不能只换 profile 字符串。

## 4. 首次运动与 H2：先验证方向，再采静态重力

更新现场 profile 后重新取静止快照。从 1° 或现场确认的更小幅度开始；先打开计划核对起姿、轴、方向、幅值和全路径，由现场人员确认物理防护与停止方式。软件不再以跟踪误差或正常结束回位误差作运行门禁；配方中已有返回段仍会发送。

```bash
site_ros casbot-calib hardware-plan-jog --config configs/hardware_field.yaml \
  --snapshot "$site/snapshot" --joint left_shoulder_pitch_joint \
  --amplitude-deg 1 --output "$site/jog_plan"

# 以下命令会切模式并发运动指令
site_ros casbot-calib hardware-execute "$site/jog_plan" \
  --domain-id "$calib_domain_id" --output "$site/jog_run"

bash scripts/humble/container.sh casbot-calib hardware-analyse "$site/jog_plan" \
  --capture "$site/jog_run" --output "$site/jog_analysis"
```

现场观察物理方向、回位、退出模式；结合 `execution/commands.csv`、`feedback.csv`、`events.csv` 与分析报告。接收间隔是到达节奏，整体响应包含缓存、插值和机械响应。它们不提供主机采用率、真实采样率或单向延迟。

通过后按 [H2 完整作业](hardware_batch_h2.md)执行。规划时必须提供静态预设：

```bash
bash scripts/humble/container.sh casbot-calib plan-static-field --output "$site/static_preset"
site_ros casbot-calib hardware-plan-batch --config configs/hardware_batch.yaml \
  --profile configs/hardware_field.yaml --preset "$site/static_preset" \
  --domain-id "$calib_domain_id" --output "$site/batch"

# 以下命令会实际运动；完成一个单元即停在检查点
site_ros casbot-calib hardware-run-batch "$site/batch" --max-jobs 1
```

继续采集时：

1. 核对首个单元结果，再用 `hardware-run-batch "$site/batch" --resume` 续采。正常检查点可续采；故障不可续采。
2. 方向观察直接记录在会话台账，不再导出／导入人工审核表；原方向模板与 review 命令已删除。按冻结顺序继续，不因数据质量或人工审核锁批次。
3. 查看 `hardware-analyse-batch` 的批次报告与 `gravity/report.json`：记录默认档与全部 100／250／500Hz 档结果、窗口统计和独立留出误差；这些数值只报告，不触发质量重采。H2 候选用于探索；若尺度未确认，残差与模型均受其假设限制，不部署。H3/H4 依赖可计算的前序数值输入；确实缺少输入时列明未执行任务，不以 prior 冒充 identified。批次 `passed` 表示无执行故障，`collection_complete` 才表示全部任务实际采到；探索完成不等于正式辨识通过。

## 5. 测完写哪里

每项把**结论、适用的模式／模型／负载、原始路径、下一步**写进本次会话台账。建议用一张 Markdown 表；状态用「已验证／已说明待验证／明确不支持／未知」。例如：`SDK-07/08 | 明确不支持 | 厂家现接口无新帧时刻／缓存年龄 | 不进入既有 S3 动态路径`。原始数据与封存计划不回写，变更只进下一轮 profile／实现。

| 能力 | 现场证据与应填写的值 | 代码／配置落点及复验 |
| --- | --- | --- |
| SDK-01／02 连接、轴与单位 | 端点和 QoS、完整轴名、实测方向与单位 | 域号运行时传入；S1 `joint_topic/joint_type/reliable/depth/joint_names`。H1 `joint_topic/feedback_type/mode_topic`，运动顺序来自所绑定模型；若方向或单位不是既有 SI 契约，改映射／适配并测反例，不能把标签改成 rad 就算转换。重跑只读、快照和受控方向检查。 |
| SDK-02／09／10 未指定轴、限制与安装 | 三事件记录、夹具与腰驱动设置、负载／姿态、设备限制和停止证据 | [现场 profile](../configs/hardware_field.yaml) 的 `gravity_world`、`maximum_velocity_rad_s` 等为程序实际输入；逐轴限位来自模型。按实际变化更新模型／场景构造（`model_runtime.py`）和执行策略（`hardware_field.py`、`position_executor.cpp`）；更新后重建计划、重做全路径检查。当前不填虚构 stop 服务。 |
| SDK-03／04 力矩来源、尺度 | 用独立已知载荷和力臂得到逐轴输出侧参考力矩，记录正反载荷变化、符号、零点、不确定度及适用构型；不要用正在辨识的 nominal 重力自证 | H2 探索配置 `effort_gain/effort_zero_nm` 可逐轴填写（当前实现为 `(raw − effort_zero_nm) × effort_gain`，偏置填在乘增益之前）；正式 S3 另建契约和尺度证据（`(raw − zero_raw) × gain`）。若确认是输出侧 Nm，不再乘减速比。重跑 H2 分析适用的新批次及 S3 资格；详见 [S3 尺度要求](vendor_s3.md#数据和证据)。仓库没有实机独立载荷试验脚本。 |
| SDK-05／06 命令与 PD | 现有接口约定是模组本地 PD、五字段转发、前馈 Nm 相加、零 PD 生效、主机不补偿；现场核对限幅、默认增益、采用规则及真实响应 | profile 的 `assumptions.*` 是报告声明，不会配置主机增益、改变插值或设置模组频率。要改变实际行为，更新 `hardware_field.params`／对应阶段配置及共享 C++ 发送器，先用隔离 Mock 验证。 |
| SDK-07／08 采样、状态和时钟 | 记录可见发布／到达时刻、反馈可用字段、来源和滤波。真实采样时刻、新帧序号／缓存年龄和高频主机日志当前不可取得 | 发布时刻继续只作发布时刻。缺失状态不得填 0；不按数值相同去重。若引入独立测量，另建采集／契约并做时间、状态、尺度及带宽对照；不改 S1 unknown 或下调 S3 资格阈值。 |

现场配置以本次实际使用的文件为准；若维护 `configs/dvt2/` 下对应发行配置，同步适用的变更。不要手改既有计划、快照或摘要。联调保护阈值是配置约束，不是实测物理停止距离。

## 6. 从探索转入正式拟合

**推荐先做静态重力替代观测验证。** 可以复用 H2 的正反接近与独立留出设计：在若干构型和已知载荷变化下，用独立参考量化估算力矩的尺度、零偏、迟滞和稳定窗口误差；若仅靠缓存反馈不能给出可信刷新／误差界，就使用独立静态观测。不能把等待更久或重复发布条数当成独立样本。替代协议需另行设计并独立验证，再决定能否正式拟合；其间仍可保存 H2 探索候选。

现有 S3 对静态重力也要求合格采样时刻、状态和尺度；动态另外要求实际 4kHz 原始采样，运动摩擦还需独立更密对照。当前 SDK 缺的不是配置值，而是观测能力。这约束正式验收，不阻止已授权的现场探索；其数值结果与诊断可保存，不能冒充正式动态资格。

若后续取得**符合现有契约**的观测，按 [S3 导入示例](vendor_s3.md#运行)执行 `vendor-import-dynamics` → `vendor-qualify-dynamics`，查看 `stages.csv` 和 `report.json`。输入是 S1 **规范化目录**（`--source-type vendor`）或有独立来源契约的 CSV/NPZ（`--source-type host`），不是原始 CDR 目录。证据齐备后另建 [S4 spec](vendor_s4.md#运行方式)，绑定预检、路径证据和独立训练／验证记录。当前 S4 读取器要求仓库默认 DVT2 URDF 摘要与 14 轴顺序匹配；若实测后换用修订模型，先让 S4 显式消费该模型并回归验证，不把新记录标回旧模型：

```bash
# S4_SPEC 指向按实证新建的 spec；仓库的 unknown 模板不提供合格数据。
bash scripts/humble/container.sh casbot-calib vendor-plan-staged \
  --spec "$S4_SPEC" --through gravity --output "$site/s4_protocol"
bash scripts/humble/container.sh casbot-calib vendor-inspect-staged \
  "$site/s4_protocol" --through gravity --output "$site/s4_inspection"
bash scripts/humble/container.sh casbot-calib vendor-identify-staged \
  "$site/s4_protocol" --through gravity --output "$site/s4_analysis"
```

`data_ready` 是所选范围的记录检查，最终看 `analysis.json` 的预检、拟合及独立验证结果。`execution_blockers` 说明实机执行边界，不是 S4 离线拟合的放行列表。只做重力不必凑齐后续动态数据。

## 7. 门槛审计：拆哪一层

| 看起来卡住的地方 | 处理 |
| --- | --- |
| 首帧／断流证据被写成可以靠 S1 测完 | 已改作业顺序：只读打底，厂家辅助验证模式／首帧／断流，然后才允许 H1。无已知安全测试路径就停在对应步骤。 |
| S4 只准备重力，但规划／检查按全阶段返回失败 | 规划与检查增加和拟合一致的 `--through`；默认仍检查至惯量，不影响既有完整流程。 |
| S1 强制 unknown，报告不出合格力矩／时钟 | 保留。S1 的职责是存原始观测；S3 直接读取 S1 规范化数据并接独立契约，不要求先修改 S1 unknown。 |
| H1 profile 精确匹配 `arm_subset/disconnect`，H1/S3 固定输出若干 false | 精确字符串只是现有支持范围声明；不是实测证据，`mode_first_frame` 也不是执行门禁。实测结论写进台账。若现场推翻现有行为假设，应改执行策略而不是放宽文字匹配后照旧下发。 |
| S2 localhost、故障禁止续采、参考规划／碰撞和 S3 采样条件 | 保留。H1–H5 已有独立实机路径；故障恢复、真实夹具／停止及观测协议各自解决，不互相代替。 |

现场 H1–H4 最新策略：通用速度默认 1.5rad/s、关节／模式接收超时 2秒／3秒；反馈位置越界只记录，跟踪／正常结束回位及经验数据质量门禁已删除。输入范围、参考碰撞、模式／通信故障和数学可计算性仍保留。旧封存 profile 与记录不回写，关键代码／执行器更新后另建计划。见[本轮接入](cloud_h4_integration.md)。
