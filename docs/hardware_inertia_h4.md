# 🦾 H4：惯量激励与 SDK 控制对照

H4接在[H3摩擦流程](hardware_friction_h3.md)之后，完成 **冻结重力与摩擦 → 多姿态周期加速度激励 → 物理一致惯量候选 → 独立轨迹预测 → 前序复核 → 五档PD选择 → 四组控制对照**。Mock与实机继续使用同一个SDK客户端、批次封存、原始记录和回放。

CASBOT主机转发position、velocity、kp、kd、effort五项，模组本地计算PD，effort作为Nm前馈叠加；主机不自动补重力/摩擦。这些已按用户确认固定。主机position的1000Hz插值与模组PD环频率分别记录，后者仍未知。

## 🧭 一、先准备通过 H3 的输入

配置见[hardware_inertia.yaml](../configs/hardware_inertia.yaml)。--prior指向完成H3且最新报告通过的批次根目录；H3必须包含起动阶段，目标轴必须已有运动摩擦与起动候选。H4检查模型、关节顺序、effort增益/零位及安装重力一致，复制并封存所需参数和H3接收观测，用于后续前序复核。原始H3目录不回写。

默认14轴同时激励，两种训练中心×两条训练轨迹、一个独立中心×两条验证轨迹，共6条惯量记录；随后5条PD选择和4条控制验收，共15条记录。也可先选少数具名轴联调，其他轴保持；报告会保留该范围，不把未激励轴的先验代表写成已辨识。

这些中心和幅值属于nominal实验设计，不能当作已测量的实机home或厂家限值。每次尝试重新读取起姿，Coal检查完整保持、进场、激励、制动和回位。连续加速度/jerk由已有Bernstein轨迹导数界检查，速度和位置沿用现场门禁；失败不自动缩小到无运动，也不放宽保护阈值。

## ▶️ 二、规划与采集

准备Humble/SDK overlay，见[启动指南](dynamics_getting_started.md)。新输出目录必须不存在：

~~~bash
cd ~/codings/casbot_calib
read -r -p "现场 ROS_DOMAIN_ID: " calib_domain_id

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-inertia \
  --config configs/hardware_inertia.yaml \
  --profile configs/hardware_field.yaml \
  --prior artifacts/site_h3_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_h4_v1
~~~

规划不连接机器人，不下发指令。Mock演练用[sdk-mock](sdk_mock_unification.md)生成的client_profile.yaml替换--profile；其它入口相同。

下列执行命令会真实发送SDK指令。先完成一个单元检查，再显式续采：

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h4_v1 --max-jobs 1

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h4_v1 --resume
~~~

惯量阶段只补冻结的重力，保留显式PD；不预先加入待辨识的惯量前馈。起姿、轨迹、控制配置、SDK/内核/实现摘要均封存，运行中不要改代码或替换二进制。受控中断/检查点沿用H2续采；SDK、通信、模式或运动保护故障只log默认清故障占位，结束整批，故障会话不能resume。

## 📐 三、惯量怎样辨识

周期轨迹复用已有多频解析参考，用C3幅值包络渐入/渐出，并通过静止中心连接实际起姿与回位。包络默认2秒，避免长过渡匹配高阶末端导数时放大位移；连续位置界使用精确Bernstein分段系数，不以离散采样代替。训练、预测验证、PD选择和控制验收使用不同种子，训练/预测中心也分离。默认周期8秒，4周期丢弃前2周期；不是对命令参考求导来冒充实际加速度。

SDK适配器保留全部原始发布/回调记录，在声明的连续稳态窗口上复用已有Fourier最小二乘：

1. 以整数纳秒原点形成回调相对时间，检查缺口和至少两个保留周期。梯形时间权重减少回调成簇对拟合权重的影响，不推断独立采样数。
2. 拟合实际接收的位置，解析求速度和加速度；另外拟合报告速度，与位置解析导数比较。不同步/明显不一致会拒绝，不跨缺口差分。
3. 同时保留位置/effort未被拟合频带解释的回调信号RMS，避免只报告滤波后的好看残差。力矩使用唯一的SDK估算Nm通道；摩擦非线性特征先按接收速度计算，再按同一频带投影。Pinocchio产生140列刚体回归及单独的转子项。
4. 固定H3运动/起动摩擦，以冻结重力模型为先验，调用原有Clarabel物理一致性求解器。正定伪惯性与重力等式约束保持，不另写一套刚体模型。
5. 用整条独立轨迹预测力矩，记录绝对/归一化残差、均值偏差、激励强度、奇异值与零空间。验收不通过时不进入控制选择。

每连杆质量、质心和惯量不能从这里宣称分别唯一。M(q)、重力和科氏/离心力矩来自同一物理模型；低频弱方向由先验及物理约束选择代表。仅激励少数轴时，候选更不能解释为整臂全部参数恢复。

## 🕒 四、缓存、相位和前序复核

这里使用的是**回调可用时间**，不是电机采样时间。sample_time_known、sampling_qualification_passed、formal_hardware_calibration_accepted保持false。未知缓存年龄、字段同步及effort滤波可能偏置惯量；时间加权和周期平均不能消除这一边界。

effort_relative_offset_s是显式配置值，默认0；另在冻结模型上报告预登记相对偏移网格的预测敏感性，不在验证集上重拟合或挑最好偏移，也不宣称已标定出物理链路延迟。滤波失配保留在残差里，不统一解释为纯延迟。控制端500Hz、主机参考1kHz、反馈发布及真实采样分别处理。

候选必须重新通过：

- 新的确定性全构型Pinocchio重力查询，检查冻结重力函数是否保持；同时保留原静态留出残差。
- H3独立匀速记录的重新配对/预测，包含候选速度耦合变化，不能只复制旧passed。
- H3起动路径上的重力变化；起动阈值本身冻结，不重新移动阈值解释惯量收益。

该SDK低频协议不替代原S3/S4的原始带宽/时钟门禁，也不替代专用高频腕部惯量探测。报告中的dedicated_high_bandwidth_wrist_validation_passed仍为false。后续取得真实采样/滤波证据后，另建协议和新数据，不能把旧结果改成正式验收。

## 🎛️ 五、PD选择与四组对照

H4的初始增益由nominal质量矩阵对角项和配置holding一次生成，默认自然频率15rad/s、阻尼比0.8，并保留Kp/Kd下限；这是合成起步配置，不是实机整定结论。五档Kp比例为0.5、0.75、1、1.25、1.5，Kd按比例平方根缩放，所有模型共用同一个选择结果。

先用冻结H3物理代表模型的前馈，在专用选择轨迹上比较跟踪RMSE，受相同速度/effort范围约束。选择封存后，独立轨迹执行：

| 前馈模型 | 原增益 | 已选增益 |
| --- | --- | --- |
| H3重力阶段物理代表，惯量尚未更新 | prior_original | prior_tuned |
| H4惯量候选，H3摩擦冻结 | identified_original | identified_tuned |

这样分别报告惯量前馈和PD调整的贡献。这里的prior已包含前序重力，不称为纯反馈基线。负收益如实保留，不自动回选验证集最优组合，也不预设实机一定改善20%。如果选择结果为1.0倍，原/已选两组增益相同，此时观测误差差值只是复测波动，不能归因为PD调整收益。

控制前馈由现有Pinocchio批量计算完整参考的刚体与摩擦力矩，独立CSV与位置流同网格封存；C++按跳槽后的同一索引发送，不重复加重力。实际发送effort/Kp/Kd与冻结计划逐项核对。仍没有设备采用回执或可信饱和状态，不把发布成功写成已采用/无饱和。

各组都从实测状态进场，稳态按同一个周期参考及回调时间计分，不逐组优化滞后。完整起姿和过渡保留，不宣称物理初态完全相同，不做状态瞬移。

## 📊 六、报告与回放

~~~bash
bash scripts/humble/container.sh casbot-calib hardware-analyse-batch \
  artifacts/site_h4_v1 --output artifacts/site_h4_analysis_v1

bash scripts/humble/container.sh casbot-calib hardware-render-batch \
  artifacts/site_h4_v1 --output artifacts/site_h4_video_v1 --fps 12
~~~

inertia_checkpoint封存独立验收通过的惯量候选，controller_selection封存五档选择。报告保存inertia/report.json、candidate.json、参数CSV、candidate_exploratory.urdf及Pinocchio重加载检查；摩擦保留在JSON中，不塞进URDF刚体惯性。URDF是候选，不覆盖nominal，不自动部署。

control_comparison.json/CSV/PNG保存四组误差与收益，inertia_validation.png显示独立力矩预测。中文回放只消费实际送达的位置；MuJoCo仍只更新几何、不参与力学或碰撞计算。

H4代码路径完成后，H5仍需真实模型/安装/限制、尺度与采样证据、设备保护核验及新的现场留出数据。软件执行通过不等于上述物理验收完成。

## 🔬 七、本机验证入口

已有通过H3的单轴合成会话时，可运行真实ROS双端演练；下列脚本只用localhost私有域，不能用于物理机器人：

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/verify_hardware_inertia.py \
  --prior artifacts/site_mock_h3 --output artifacts/site_mock_h4_new --render

bash scripts/humble/container.sh \
  python scripts/humble/verify_h4_references.py --output artifacts/site_h4_references_new
~~~

第一条针对一个合成轴、完整14轴机械模型，执行3条惯量、5条PD选择、4条控制记录；plant显式改变质心惯性而保持质量/质心，辨识不读取该设定。另在禁止访问evaluation的新进程中从原始反馈重建特征、重新拟合并比较冻结候选，回放实际解码检查；记录指令回放进行步长减半检查。第二条只审查默认全14轴的8条不同参考，不连接SDK、不代表真实起姿/跟踪/停止验收。生成文件均为本地artifacts，不随Git提交。

本轮本地交付证据为artifacts/hardware_inertia_ros_v6/verification.json，12个单元全部通过；默认14轴参考审查为hardware_inertia_references_v1/verification.json，8条不同参考全部通过。实际SDK演练只激励左肩pitch，不能算完整14轴采集。中文视频在hardware_inertia_ros_v6/replay；更严格的重复差异解释在hardware_inertia_readonly_v2。

| 指标 | 本次合成结果 |
| --- | --- |
| 独立力矩预测RMSE | 0.00121756 Nm |
| 回归秩 | 17/140；不唯一方向保留先验依赖 |
| 所测构型M00：plant / 候选 | 0.21785294 / 0.22143827 kg·m² |
| 该M00相对误差 | 1.6458%；仅独立评估读取plant |
| PD选择 | 1.0倍，原/已选增益相同 |
| 固定PD的两条前馈记录RMSE差值 | 2.7772%；小于同增益复测差异，稳定收益未证明 |
| 实测客户端发布频率 | 约499.27–499.77 Hz；不代表模组采用/采样频率 |

演练使用8秒周期、3周期丢弃1周期，显式1ms内部采样与100µs积分；默认机械积分仍50µs。已记录指令100/50µs独立回放的位置最大差约7.94µrad，峰值相对变化及逐点速度归一化变化均小于1%。新进程禁止evaluation访问后，原始反馈重建特征差与重新拟合的参数差均为0。所有这些结果仍是合成/本机范围，不获得正式硬件采样资格。

早期hardware_inertia_ros_v1–v5的跟踪保护、过松的位置控制点包络、jerk拒绝及高速缓存敏感失败全部保留。修正为显式H4增益、连续位置界细分、C3幅值包络及默认8秒周期后新建批次；没有放宽原跟踪/反馈年龄/碰撞门槛。v6采集后只增加数值位置界的极小舍入余量与报告解释字段，未改变已发送指令；当前参考审查和只读报告按更新后的代码重新运行。

本轮456项完整Python回归、最终18项H4专项、4项核心C++及2项SDK检查通过；旧S2私有域500Hz往返也通过。最终专项包含全量运行后补充的禁用/缺失起动源拒绝和收益解释反例。证据在hardware_inertia_tests_final_v1、hardware_inertia_s2_regression_v1，汇总见hardware_inertia_delivery_v1/delivery.json。未重跑历史114项冻结数值交付，未测试Windows或物理网络。
