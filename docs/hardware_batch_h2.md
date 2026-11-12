# 🚀 H2：实机整批采集、频率分档与受控续采

H2 将 [H1 实机入口](hardware_execution_path.md) 组织成一份冻结的现场批次：**逐轴方向检查 → 100/250/500Hz重复测试 → 逐姿态正反静态采集 → 汇总重力候选与只读回放**。

H2 负责采集编排，继续使用同一个C++ SDK位置发送器、Pinocchio回归、Coal碰撞和已有物理一致求解器。它不增加真实力矩斜坡或惯量硬件标定，也不将缓存反馈转换成已知采样时刻。

无实机时可按[统一SDK Mock指南](sdk_mock_unification.md)启动完整机械Mock，用生成的client_profile.yaml代替硬件profile；H2自动冻结本机连接，执行入口不变。

**按用户最新要求：执行、通信或模式故障发生后，默认clear_fault只记录日志，然后结束整个采集流程。该故障会话不能通过--resume继续。现在不尝试清故障、不请求恢复、不自动重新申请控制。**

## 🧭 一、默认批次做什么

默认配置为 [hardware_batch.yaml](../configs/hardware_batch.yaml)，硬件参数来自 [hardware_field.yaml](../configs/hardware_field.yaml)。

| 阶段 | 默认安排 | 记录内容 |
| --- | --- | --- |
| 方向 | 14轴，各做1°正反往返，默认500Hz | 保存实际反馈跨度与跟踪误差，不设运动跨度通过比例或人工review门禁 |
| 频率 | 左/右肩pitch，各100/250/500Hz、每档2次，共12条，固定种子随机化顺序 | 保存实际发布频率、抖动与整体偏移；低于计划90%也继续 |
| 静态 | 默认每臂24训练+8验证，共64个姿态；每姿态两种接近方向 | 整个正反接近对作为一个采集单元，保存计划测量窗口中的实际数据，速度/跨度/覆盖/缺口不判合格与否 |
| 汇总 | 合并有数值的窗口，复用H1重力拟合/候选导出 | 不跨记录拼接时间、求导或伪造采样，正式实机标定接受仍为false |

静态预设可通过max_poses_per_split限制每臂每角色的姿态数，用于先跑采集链路；少量姿态不等于充分可辨识，也不保证角度小。批次可配置单关节/部分阶段，报告会显示方向范围，不冒充完成14轴检查。

### 频率比较怎样保持一致

首条频率实验读取实际姿态后，冻结一个14轴比较锚点。同一目标关节在各档使用完全相同的连续参考，SHA记录在计划和分析中；只有客户端发送周期变化，不按频率重新拟合机械模型。

每次尝试仍重新读取起姿，接近冻结锚点，执行比较动作后回到这次实测起姿。比较窗内只动目标轴；接近/返回阶段其他臂轴可能回到锚点，整个路径照常检查。频率报告单独统计比较窗，避免初始等待或回位掩盖实际发布情况。

各档实际发布率都只作为测量结果，不再影响后续静态采集。程序不自动换成较低频率，也不宣称推荐某一实机频率。

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

## ▶️ 三、开始执行

下面的run会真实发送SDK位置命令：

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_batch_v1
~~~

每个采集单元都重新读取实际静止姿态，再生成和检查本次完整路径，执行末帧后退出模式，不再以回位位置/停稳作为完成门槛。原始CDR、实际命令和回调反馈继续由H1保存。

默认依次执行14轴往返、频率实验和静态扫描，不要求人工review，不再提供`hardware-direction-template`或`hardware-direction-review`入口。实际物理方向仍由现场人员观察，软件不代替判断。

## ⏯️ 四、检查点、分析不可用与故障的区别

### 发指令前起点变了：重建当前单元

起点偏差阈值仍为0.005rad。原生执行器明确回报“未申请模式、未发布指令、需要重规划”时，将旧尝试封存为`start_replan`，输出最大偏差，自动重新读取快照、规划和检查当前任务；不调用clear_fault、不锁整批，也不重跑已接受的单元。重复变化时可以Ctrl+C取消；频率比较锚点、静态角色及H3/H4前序输入保持原绑定。

此分支不接收已申请模式/已发送指令后的异常。SDK、模式、通信故障仍按下述规则结束整批。关节/模式接收超时的新profile默认为2秒/3秒；启动/运行速度和模式故障处理保留。实现更新后建立新批次，旧批次不原地改规则。

### 正常检查点或主动中断：可以续采

~~~bash
# 本次最多新完成2个采集单元，再正常暂停
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_batch_v1 --resume --max-jobs 2
~~~

后续--resume会校验已完成单元摘要，只跳过已完成发送的记录。主动Ctrl-C会要求当前发送器停止，将该尝试标为interrupted；续采创建新的尝试，重新读起姿，不追加旧CSV或沿用中断前的姿态。已完成单元不重采。

如果外部模式仍不允许新执行，后续尝试会终止，不会帮机器人解除保护。进程被硬杀、写盘中断而留下未封存且状态不明的尝试，当前按终止故障处理，不能当作正常受控续采。

### 运动完成后，数据照录，不做质量重试

静态速度/位置跨度/窗口覆盖、回调间隔、匀速平台质量、周期速度一致性、发送率和拟合残差都不作为批次门禁。静态重试预算、settle递增策略和质量拒绝状态已删除，每个任务正常执行一次；只有发指令前的起点变化仍会重建参考。

分析能计算时保存数值，缺少数据或数值求解无法完成时在`inspection/report.json`记录`available=false`及原因，继续下个任务，不调用清故障、不锁整批。SDK执行失败仍遵循下一节的终止规则。

H4后续动作若必须使用尚未生成的惯量候选或所选控制参数，则记录`not_run_missing_input`并处理其它任务，不替换为别的模型。`jobs_not_run`列出这些动作及原因。

### 执行、模式、通信故障：记录后结束整批

默认API为SDKBatchBackend.clear_fault(reason)，实现只写日志，返回implemented=false、performed=false。随后登记terminal_fault并终止整批，当前不执行任何恢复动作；即使再次显式--resume也会拒绝。

占位日志不代表故障已清除、机体已停止或模式已恢复。后续获得可靠恢复方法时再设计新的恢复协议；不要在当前会话中删故障事件、改状态或伪造回执继续采集。

## 📂 五、记录和报告

| 目录/文件 | 用途 |
| --- | --- |
| protocol/ | 冻结的profile、配置、模型/SDK/代码身份、任务顺序、场景与静态预设 |
| events/ | 按序列和前项摘要串联的不可变事件；原子提交，不靠可变进度文件决定是否跳过数据 |
| attempts/任务/attempt_NNN/ | 每次独立profile、快照、完整计划、原始采集、检查、outcome与摘要 |
| frequency_anchor/ | 第一条频率实验冻结的实际比较锚点及来源摘要 |
| faults/ | 默认清故障占位日志；有记录不等于发生过恢复 |
| reports/ | 各检查点/终止时刻的不可变汇总、jobs.csv、频率PNG及可用的重力候选 |
| latest.json、progress.json | 最新报告指针/进度缓存，不是观测或恢复状态的唯一依据 |

进程锁阻止两个控制器同时运行同一批次。已经封存的attempt和report不覆盖；未完成文件保留，续采不静默删除或修改。

新报告为`casbot_hardware_batch_report_v2`：`passed`表示任务已处理且无执行故障，`workflow_processed`表示流程处理完，`collection_complete`仅在所有任务确有成功运动记录时为true。缺输入跳过的动作列在`jobs_not_run`；分析结果能否计算另看各阶段`available`。因此“流程已处理”不冒充“所有运动都已采到”，也不表示辨识质量通过。

旧`direction_review_complete`、`direction_gate_satisfied`、`default_frequency_passed`及`all_frequency_profiles_passed`字段已删除。新尝试状态为`recorded`，不以数据质量接受/拒绝命名；旧封存记录不回写，代码改变后运行须新建批次。

## 🔍 六、只读重分析与回放

~~~bash
bash scripts/humble/container.sh casbot-calib hardware-analyse-batch \
  artifacts/site_batch_v1 --output artifacts/site_batch_analysis_v1

bash scripts/humble/container.sh casbot-calib hardware-render-batch \
  artifacts/site_batch_v1 --output artifacts/site_batch_video_v1 \
  --fps 12 --max-seconds-per-attempt 4
~~~

分析不运行SDK节点或采集动作，只消费封存记录。静态模型使用计划角色中的可用数值窗口；各记录自己的时钟和角色保留，不把多次尝试拼成一条均匀采样序列。

回放复用统一MuJoCo几何renderer，不推进物理；展示每次尝试的真实已接收角度，包括失败或重采的可用记录。试次间留分隔，不插值或推测缺失运动；时间可压缩，角度不放大。无有效反馈或尾行不完整的尝试会在回放报告中说明未展示原因。

## ✅ 七、适用范围

H2提供实机批次代码，不代替设备能力确认。position插值、缓存反馈、发布stamp、估算Nm及断流保护的已知边界继续沿用[SDK记录](sdk_hardware_prerequisites.md)。本轮不连接实体机器人；自动化验证使用隔离ROS协议夹具及既有数值测试，[H3运动摩擦/起动](hardware_friction_h3.md)和[H4惯量/控制](hardware_inertia_h4.md)的软件路径现已接入，完成H2后按顺序继续；H5正式现场验收仍需实机证据。

### 历史版本本机验证记录（含当时已存在、现已删除的质量/review策略）

完整Python回归415项通过，随后补充的H2专项13项通过，覆盖质量预算耗尽不再重采及只显示实际送达姿态等反例。测试报告分别为artifacts/hardware_batch_tests_v3/full.xml与artifacts/hardware_batch_tests_v4/h2.xml；这不是重新执行历史114项冻结数值交付。

artifacts/hardware_batch_ros_v2/verification.json的17项隔离ROS检查通过，其中8项为H2：人工方向结论绑定、整批完成、失败/重采记录保留、三档相同参考、几何回放、受控Ctrl-C续采、通信故障后拒绝续采、默认clear_fault无实际动作。夹具只模拟最新位置指令的响应和缓存发布，不认证厂商插值或物理保护。首次v1因测试进程的ROS信号处理同时关闭夹具而失败，原记录保留；v2隔离信号后重新验证。

默认完整预设另完成无设备I/O规划：14方向+12频率+64静态，共90个单元，见artifacts/hardware_batch_default_plan_v1。中文视频为artifacts/hardware_batch_ros_v2/h2_replay/vendor_read_only_zh.mp4；重放几何与Pinocchio FK最大差约3.33e-16，角度未放大。交付摘要为artifacts/hardware_batch_delivery_v1/delivery.json。artifacts属于本机生成物，不随Git提供。
