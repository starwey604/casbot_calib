# 🚀 H2：实机整批采集、频率分档与受控续采

H2 将 [H1 实机入口](hardware_execution_path.md) 组织成一份冻结的现场批次：**逐轴方向检查 → 人工方向结论绑定 → 100/250/500Hz重复测试 → 逐姿态正反静态采集 → 汇总重力候选与只读回放**。

H2 负责采集编排，继续使用同一个C++ SDK位置发送器、Pinocchio回归、Coal碰撞和已有物理一致求解器。它不增加真实力矩斜坡或惯量硬件标定，也不将缓存反馈转换成已知采样时刻。

无实机时可按[统一SDK Mock指南](sdk_mock_unification.md)启动完整机械Mock，用生成的client_profile.yaml代替硬件profile；H2自动冻结本机连接，执行入口不变。

**按用户最新要求：执行、通信或模式故障发生后，默认clear_fault只记录日志，然后结束整个采集流程。该故障会话不能通过--resume继续。现在不尝试清故障、不请求恢复、不自动重新申请控制。**

## 🧭 一、默认批次做什么

默认配置为 [hardware_batch.yaml](../configs/hardware_batch.yaml)，硬件参数来自 [hardware_field.yaml](../configs/hardware_field.yaml)。

| 阶段 | 默认安排 | 通过依据 |
| --- | --- | --- |
| 方向 | 14轴，各做1°正反往返，默认500Hz | 软件先检查确有运动；物理方向由具名观察者填写，绑定该轴已接受的观测摘要 |
| 频率 | 左/右肩pitch，各100/250/500Hz、每档2次，共12条，固定种子随机化顺序 | 使用实际比较窗的发布频率与运动响应；默认均值至少达到请求频率的90%，报告抖动与整体偏移 |
| 静态 | 默认每臂24训练+8验证，共64个姿态；每姿态两种接近方向 | 整个正反接近对作为一个采集单元，实际回位后检查稳定窗口 |
| 静态重试 | 首次之外最多2次，仅为质量不合格重采 | 每次为两个接近分别增加1s稳定等待；原记录和原角色不改变 |
| 汇总 | 合并已接受的窗口，复用H1重力拟合/候选导出 | 不跨记录拼接时间、求导或伪造采样，正式实机标定接受仍为false |

静态预设可通过max_poses_per_split限制每臂每角色的姿态数，用于先跑采集链路；少量姿态不等于充分可辨识，也不保证角度小。批次可配置单关节/部分阶段，报告会显示方向范围，不冒充完成14轴检查。

### 频率比较怎样保持一致

首条频率实验读取实际姿态后，冻结一个14轴比较锚点。同一目标关节在各档使用完全相同的连续参考，SHA记录在计划和分析中；只有客户端发送周期变化，不按频率重新拟合机械模型。

每次尝试仍重新读取起姿，接近冻结锚点，执行比较动作后回到这次实测起姿。比较窗内只动目标轴；接近/返回阶段其他臂轴可能回到锚点，整个路径照常检查。频率报告单独统计比较窗，避免初始等待或回位掩盖实际发布情况。

非默认档位不合格可以继续记录其他档位；配置中选定的默认频率（目前500Hz）任一必要重复不通过，则在静态采集前停止。程序不自动换成较低频率，也不宣称推荐某一实机频率。

## 🛠️ 二、规划批次：不连接机器人

先按[启动指南](dynamics_getting_started.md)准备Humble及SDK overlay，在宿主仓库根目录操作：

~~~bash
cd ~/codings/casbot_calib
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh
read -r -p "现场 ROS_DOMAIN_ID: " calib_domain_id

# 尚无预设时先生成
bash scripts/humble/container.sh casbot-calib plan-static-field \
  --output artifacts/site_static_preset_v1

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-batch \
  --config configs/hardware_batch.yaml --profile configs/hardware_field.yaml \
  --preset artifacts/site_static_preset_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_batch_v1
~~~

输出目录必须不存在。规划会复制小型预设和场景到protocol中，冻结配置、设备声明、模型、SDK/执行器身份、关键代码及完整任务顺序，不依赖其他历史artifacts；此命令不创建运动节点或下发指令。规划报告的passed=false不表示“规划失败”，它表示整批采集尚未完成，state=planned。

协议冻结后，修改原YAML不会改变这个批次；需要变更模型、采样/控制规则、判据、SDK或关键实现时，建立新批次，不改旧协议。

## ▶️ 三、开始执行与绑定人工方向结论

下面的run会真实发送SDK位置命令：

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_batch_v1
~~~

每个采集单元都重新读取实际静止姿态，再生成和检查本次完整路径，执行后确认回位。原始CDR、实际命令和回调反馈继续由H1保存。

默认先完成14轴往返，然后暂停为awaiting_direction_review，尚不进行频率和静态扫描。软件能判断编码器是否响应，但不能从编码器自身确认物理方向或机械零位。

~~~bash
bash scripts/humble/container.sh casbot-calib hardware-direction-template \
  artifacts/site_batch_v1 --output artifacts/site_direction_review_v1.yaml
~~~

在导出的YAML中填写observer、每个joint的observed_same_direction（true或false）及note。保持模板的attempt_manifest_sha256，它绑定具体观测；不能将另一轮或旧记录的结论套入当前批次。填写的是实际观察，不要将所有项自动设为true。

~~~bash
bash scripts/humble/container.sh casbot-calib hardware-direction-review \
  artifacts/site_batch_v1 --file artifacts/site_direction_review_v1.yaml

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_batch_v1 --resume
~~~

可分批导入已观察的轴；未齐或有否定结论时，后续运动继续暂停。复核修正另存为新review，不覆盖旧结论。机械零位/几何标定不是H2的产物。

## ⏯️ 四、检查点、重试与故障的区别

### 正常检查点或主动中断：可以续采

~~~bash
# 本次最多新完成2个采集单元，再正常暂停
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_batch_v1 --resume --max-jobs 2
~~~

后续--resume会校验已完成单元摘要，只跳过已接受的记录。主动Ctrl-C会要求当前发送器停止，将该尝试标为interrupted；续采创建新的尝试，重新读起姿，不追加旧CSV或沿用中断前的姿态。已完成单元不重采。

如果外部模式仍不允许新执行，后续尝试会终止，不会帮机器人解除保护。进程被硬杀、写盘中断而留下未封存且状态不明的尝试，当前按终止故障处理，不能当作正常受控续采。

### 正常回位但静态窗口不稳：有限重采

只有SDK执行正常完成、回位/静止已确认，且拒绝原因属于静态窗口不稳定/覆盖不足时，才自动重试同一个正反接近对。原失败文件保留；新尝试按冻结规则延长等待，训练/验证身份不变。

碰撞、数据摘要不符、无效字段、模式或通信异常不属于这种质量重试。达到预算仍失败则停止为data_rejected；反复调用resume不会增加重试预算，也不会选择“误差最好”的一组记录。

### 执行、模式、通信故障：记录后结束整批

默认API为SDKBatchBackend.clear_fault(reason)，实现只写日志，返回implemented=false、performed=false。随后登记terminal_fault并终止整批，当前不执行任何恢复动作；即使再次显式--resume也会拒绝。

占位日志不代表故障已清除、机体已停止或模式已恢复。后续获得可靠恢复方法时再设计新的恢复协议；不要在当前会话中删故障事件、改状态或伪造回执继续采集。

## 📂 五、记录和报告

| 目录/文件 | 用途 |
| --- | --- |
| protocol/ | 冻结的profile、配置、模型/SDK/代码身份、任务顺序、场景与静态预设 |
| events/ | 按序列和前项摘要串联的不可变事件；原子提交，不靠可变进度文件决定是否跳过数据 |
| attempts/任务/attempt_NNN/ | 每次独立profile、快照、完整计划、原始采集、检查、outcome与摘要 |
| reviews/ | 具名人工方向结论、原始输入文件及观测绑定 |
| frequency_anchor/ | 第一条频率实验冻结的实际比较锚点及来源摘要 |
| faults/ | 默认清故障占位日志；有记录不等于发生过恢复 |
| reports/ | 各检查点/终止时刻的不可变汇总、jobs.csv、频率PNG及可用的重力候选 |
| latest.json、progress.json | 最新报告指针/进度缓存，不是观测或恢复状态的唯一依据 |

进程锁阻止两个控制器同时运行同一批次。已经封存的attempt、review和report不覆盖；未完成文件保留，续采不静默删除或修改。

完成时查看collection_complete、passed、direction_review_complete、default_frequency_passed与gravity；all_frequency_profiles_passed单独描述全部档位，某个非默认档失败不会伪装成全部档位成功。硬件正式验收、真实采样率、物理方向的软件验证和自动部署标志不因此置true。

## 🔍 六、只读重分析与回放

~~~bash
bash scripts/humble/container.sh casbot-calib hardware-analyse-batch \
  artifacts/site_batch_v1 --output artifacts/site_batch_analysis_v1

bash scripts/humble/container.sh casbot-calib hardware-render-batch \
  artifacts/site_batch_v1 --output artifacts/site_batch_video_v1 \
  --fps 12 --max-seconds-per-attempt 4
~~~

分析不运行SDK节点或采集动作，只消费封存记录。静态模型只合并已接受的完整正反窗口；各记录自己的时钟和角色保留，不把多次尝试拼成一条均匀采样序列。

回放复用统一MuJoCo几何renderer，不推进物理；展示每次尝试的真实已接收角度，包括失败或重采的可用记录。试次间留分隔，不插值或推测缺失运动；时间可压缩，角度不放大。无有效反馈或尾行不完整的尝试会在回放报告中说明未展示原因。

## ✅ 七、适用范围

H2提供实机批次代码，不代替设备能力确认。position插值、缓存反馈、发布stamp、估算Nm及断流保护的已知边界继续沿用[SDK记录](sdk_hardware_prerequisites.md)。本轮不连接实体机器人；自动化验证使用隔离ROS协议夹具及既有数值测试，[H3运动摩擦/起动](hardware_friction_h3.md)和[H4惯量/控制](hardware_inertia_h4.md)的软件路径现已接入，完成H2后按顺序继续；H5正式现场验收仍需实机证据。

### 本机验证记录

完整Python回归415项通过，随后补充的H2专项13项通过，覆盖质量预算耗尽不再重采及只显示实际送达姿态等反例。测试报告分别为artifacts/hardware_batch_tests_v3/full.xml与artifacts/hardware_batch_tests_v4/h2.xml；这不是重新执行历史114项冻结数值交付。

artifacts/hardware_batch_ros_v2/verification.json的17项隔离ROS检查通过，其中8项为H2：人工方向结论绑定、整批完成、失败/重采记录保留、三档相同参考、几何回放、受控Ctrl-C续采、通信故障后拒绝续采、默认clear_fault无实际动作。夹具只模拟最新位置指令的响应和缓存发布，不认证厂商插值或物理保护。首次v1因测试进程的ROS信号处理同时关闭夹具而失败，原记录保留；v2隔离信号后重新验证。

默认完整预设另完成无设备I/O规划：14方向+12频率+64静态，共90个单元，见artifacts/hardware_batch_default_plan_v1。中文视频为artifacts/hardware_batch_ros_v2/h2_replay/vendor_read_only_zh.mp4；重放几何与Pinocchio FK最大差约3.33e-16，角度未放大。交付摘要为artifacts/hardware_batch_delivery_v1/delivery.json。artifacts属于本机生成物，不随Git提供。
