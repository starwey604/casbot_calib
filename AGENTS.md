# casbot_calib 协作约定

适用于本仓库全部目录。面向用户的说明、报告和演示默认使用中文，代码标识符沿用英文。开始修改前查看工作区状态，保留用户已有改动。

**实机接入前必须阅读并逐项弄清[SDK待确认能力与前置条件](docs/sdk_hardware_prerequisites.md)**。反馈/指令effort含义、输出尺度、PD和内部补偿、采样/时钟/实际采用频率、停止与设备限制、真实模型和腰部固定条件须有对应设备/版本的证据与结论。未确认或明确不支持时，阻断依赖它的阶段；不得以字段存在、配置布尔值、SDK编译或Mock验收替代。只读采集可保留未知字段，不授予运动/标定资格；不要直接改S1 unknown或移除S2 localhost限制来接实机。

**用户2026-10-30本轮进一步授权：现在补齐真实SDK执行路径，不确定行为先询问或配置假设，到场联调再修正，不让正式观测资格阻断其余代码开发。** 用户确认只发双臂14轴时腰/头/手保持；断流由机体自保护是用户要求采用的假设，尚未实测。新增[硬件联调H1–H5路径](docs/hardware_execution_path.md)和显式`hardware-*`入口支持真实位置发送、静态探索；原S1未知配置、S2私有scope与S3/S4/P0资格保持独立。不要再把“原S2未开放”等同于“仓库没有硬件执行命令”，也不要把探索拟合通过写成正式标定通过。硬件命令只在用户调用相应入口时执行，开发/测试不得连接物理机器人。未知项记入profile/报告，不编造采样时间、独立扭矩传感器、stop RPC或物理停止证明。

**H2最新恢复约定：用户要求API中的解除故障默认先留空、只log，随后结束整个采集流程；后续再探索恢复。** SDK执行/通信/模式故障必须终止整批并禁止该会话resume，不因占位方法返回而继续；当前不清故障、不重新申请控制、不尝试自恢复。受控主动中断或正常检查点可以新尝试续采；正常回位后的静态窗口质量不合格可按冻结预算重采，这不属于故障恢复。硬杀/未封存且状态不明的中断当前也终止，不静默当作已恢复。

**H3本轮用户确认：CASBOT主机直接转发position、velocity、kp、kd、effort五项到因克斯模组，由模组使用本地反馈计算PD；command effort按Nm前馈叠加PD，kp=kd=0会生效，主机不自动加入重力/摩擦补偿。** 此执行位置已固定为接口约定，不再询问或标为转发未知；position仍保留已说明的主机1000Hz线性插值，不将其等同于模组PD环频率。PD环具体频率仍未知。不再把这些确认项写成完全未知；仍是用户转述、待绑定设备/固件实测。现场流程由发送器补重力：H1/H2默认nominal起步，H3冻结前序重力；不加摩擦前馈以免污染标定。SDK Mock运行态同步采用无内部补偿、允许effort/自定义PD。用户另确认模式切入至首帧会保持原目标/输出，Mock据此保持旧保护输出到首条有效命令；具体机型/固件仍需实测，S1/S3正式采样/尺度资格不因此改值。

2026-10-30厂家答复由用户转述，记为**已说明待验证**：反馈effort是模组驱动估算Nm，无独立扭矩传感器；电流乘转矩常数只是推测，输出侧/尺度/滤波/补偿仍未闭合。仅position线性插值到1000Hz；debug的`/motion/joint_state`和`/motion/debug/joint_state`据称1000Hz发布、可能重复最新缓存，其header.stamp为发布时刻，当前无可用高频日志。不能把发布时刻当采样时刻、把相同数值当旧帧去重或将velocity/effort视为同样插值。设备/版本、上身模式可用性、字段/逐轴同步、新帧更新率/缓存年龄和实际命令消费规则仍待核验；详情及阶段影响见[答复记录](docs/sdk_hardware_prerequisites.md#厂家答复记录2026-10-30同步)。该次厂家边界同步未修改资格政策；本轮H3的控制约定与Mock默认值变化见上段和H3指南，历史记录不回写。

## 当前目标与已完成范围

- **2026-11-02用户确定整体替换默认模型**：来源为`~/下载/casbot_hl12p1_dvt2`，大型资源使用Git LFS；实施见[DVT2模型替换计划](docs/dvt2_model_migration_plan.md)，N1已提交`10fb5be`。2026-11-05的1105_03 STEP已明确实际装配：从a2_ff拆普通手，一个三表工具左/右依次换装，另一臂裸法兰停放；左相对STEP+90°、右0°。不要再询问装配选择或擅自选b3。前序N2实际候选`derived/bare_flange_1105_03_v1`及STL见[孔位/平台试算](docs/trical_1105_hole_trials.md)，普通手候选仅为前序技术验证。实际STEP与登记在`references/cad/trical_1105_03`；运行不依赖下载/FreeCAD。装配确认不是实物计量确认；工具总质量/质心未知、自由测杆全行程未测。
- **本轮孔位/升降边界**：整柱换台面孔、侧球换已有柱孔、平台整体升降均已实现。平台升降必须带动台面/随动支架/柱/球，机器人与固定底座不动；每个双臂批次内高度和球架布局锁定不变，实际行程未验证。141条试算含对照/重复布局，最多6/8名义候选；原位及+50mm代表方案均5/8通过连续门禁（左A/B/C、右B/D），不是整批通过。左D/右C缺候选、右A关节包络未获证明。保持普通10mm、精确近接2mm、三对有界接触5mm，不导入review的腰/肩排除、不以采样粗筛代替连续检查。肩腰/表体核对已完成，结果见下条；本轮跨侧退入参考已8/8闭合，见后续条目；N3默认整体切换/N4未完成。旧nominal未局部替换，旧正式test不重标为新模型证据。
- **2026-11-06肩腰/跨侧核对**：见[碰撞审计](docs/trical_1105_collision_audit.md)。右C原姿态肩腰源网格/凸包均4.849mm，左D10.920mm；每臂837点审计未发现分离区的肩腰凸包误判，保留该检查对。真正的保守性来自细杆/圆表体单凸包，左D表体5160—柱旧凸包5.001mm而源表面12.040mm。已从同一CAD工作状态将六表体各切6段后外包+0.2mm，并集缺失/多余体积均0；新资产`trical_1105_03_gauge_cover_v2`、新候选`derived/bare_flange_1105_03_gauge_v2`。复用旧显示STL，不改源CAD/URDF动力学、不覆盖旧包或默认。`trical_assembly.asset_directory(bundle)`按绑定元数据选资产，旧包无该字段仍读旧资产。原参考重检仍5/8；左D和右A的±0.00105rad盒角点已有当前保守包络9.635/9.856mm反例（源表面分别9.635/10.186mm，右A不可说成实物碰撞），不能只提高细分预算或将未获证明解释为算法过于保守。`trical_access.py`做自由朝向/双端点姿态搜索，单点改善不是路径放行；后续分段退入已按下一条完成，保持10/2/5mm规则。
- **2026-11-06分段退入参考已8/8闭合**：见[冻结参考与使用](docs/trical_1105_release_then_turn.md)。原台架孔位/高度、左90°右0°不变；`configs/trical_1105_release_turn_reference.json`与`trical_1105_release_turn_settings.json`可直接重建，不依赖artifacts或重新优化。`trical_withdrawal.py`先用角点约束改进保持位，再沿释放锥退出；模型中三个测头均≥0.5mm正间隙后才允许转向，后续不再使用负行程例外，最终自由空间恢复全部10mm。接触区1°姿态包络包含每轴±0.00105rad误差；左D/右C转向约7.95°/6.40°，右A基本不转。`trical_paths.py`按种子withdrawal字段生成同一C3/500Hz参考；旧种子默认规则不变。新withdrawal模式的retract_m为TCP到球心径向距离，不是曲线弧长；trical_holes直线粗筛不得吞掉该相位规则。
- **本轮证据与剩余范围**：`artifacts/trical_release_turn_frozen_v1`从冻结配置重建8/8（搜索原记录在`trical_release_turn_routes_v4/plan`）；`trical_release_turn_frozen_verify_v1`新进程禁止优化/plant后192段复验；`trical_release_turn_corners_v1`3,968点完整Coal额外角点采样；中文MP4在`trical_release_turn_render_v1`。局部运动上界复用原生Pinocchio轴线/顶点位姿与全局后代位移界，不重写FK；仅新冻结设置显式开启`local_motion_bounds`及100k/32层离线证明预算，旧调用保持原默认。工作状态测杆几何范围未扩大为实物自由伸出保证，`hardware_executable=false`；千分计采集/几何辨识、工具负载、实物外形与N3/N4仍未完成，不把八组进退参考叫作整套实机标定通过。
- **2026-11-06 N3第一轮**：N2已提交`cfe855e`。`model_context.load_context()`显式选择`bare_flange_1105_03_gauge_v2`，统一原生模型、身份、碰撞、K1四球观测、transport模型参数和只读CAD场景；详见[N3接入](docs/dvt2_n3_integration.md)。动力学/运动学候选可从同一新源导出，资源引用重定位；新动力学sidecar绑定上下文并可自动重加载。此轮**未切默认**，不把显式上下文测试当成SDK Mock/H1–H4/K2–K4整链迁移。第二轮动力学与SDK消费者进展见下条；K2–K4及P0协议消费者准备好后原子切换；不要单独改`nominal_dual_arm()`默认或只换Mock服务端。新上下文无旧模型selector/下载fallback；旧默认只是迁移准备状态，完成后移除。K1配置的CAD球位不是现场计量，工具负载未知，两个TCP不表示同时装两套工具。
- **2026-11-06 N3第二轮**：`model_runtime.py`按冻结config解析同源模型/URDF/碰撞；`configs/dvt2/`显式接入SDK Mock、H1–H4、静态预设及schema 1 staged。服务端与客户端须有相同model_context，不能只替换其中一端；500Hz客户端、位置插值与未知采样语义、故障整批终止均保持。SDK录制/规范化/批次显示保留新身份；重力URDF复制必须经`hardware_friction.copy_gravity`重定位资源并封存，不能直接copytree后留下失效相对路径。新原生采集记录绑定上下文，不能混用旧记录。当前新场景为裸法兰整机+地面，无TriCal工具/台架或实测腰夹具；P0 schema 2新模型协议尚未迁移。764条参考、64姿态128条静态参考独立复核及左右肩100/250/500Hz六条SDK往返通过；另有81条原生方向/时序/重力采集，重力checkpoint及禁止evaluation的新进程重分析通过；详细证据和范围见[N3](docs/dvt2_n3_integration.md)。实际装配K2–K4已按下条接入；下一轮在默认切换前补齐P0协议消费者，再统一默认并做N4；不要把本轮规划/代表SDK往返写成全14轴H3/H4采集或新惯量/控制验收。
- **2026-11-06 N3第三轮**：`configs/dvt2/kinematics.yaml`接实际四球/单工具左右换装，沿N2冻结进退参考；新旧采集共用`MeasurementController.center()`、原生持续Mock和K3/K4内核。见[实际装配指南](docs/dvt2_trical_integration.md)及[N3](docs/dvt2_n3_integration.md)。参考管±0.00105rad分配为编码器50µrad+跟踪1mrad，几何位移0.1mm/旋转0；Mock工厂检查两侧误差族。执行/重分析可消费封存同几何/阈值/端点的参考证明，但实际原始观测折线全部内部点、朝向和稳定窗口必须每批独立复核，不在在线控制循环重跑离线CAD树。旧记录无分配字段则维持原完整预算，不补写。接触小弦段重算既有N2局部界，不降低10/2/5mm；左右换装只在新鲜反馈确认home停稳时，故障不直线退出/自动回位。K3 v3实际16训练/8验证、31/60可辨识代表，条件190.37、独立位置约束RMSE0.879µm；K4三批重复性/新进程无真值拟合通过，预测标准差最大0.856µm。CAD球位/mastering/工具载荷和自由测杆外形仍非实测，不授予硬件资格。下一轮迁移P0 schema 2，再原子切默认/N4；不要把旧P0正式test或六球合成批次改绑新模型。
- 当前优先级是**整机完整装配、腰部刚性固定的14轴双臂动力学标定**。腰部抱紧位置为前腹部圆形开口附近；实际夹具尺寸、安装变换与重力方向尚需测量，当前盒体仅为合成代理。
- 截至2026-10-28，M1–M6本机合成流程已完成：Humble迁移及原实验回归、真实ROS双端Mock、频率/延迟实验、碰撞门禁、重力→运动摩擦→起动摩擦→惯量分阶段辨识、独立验证和中文视频。
- 动力学[P0计划](docs/staged_dynamics_p0_plan.md)已实现[R0数据契约](docs/staged_dynamics_p0_r0.md)及[R1多构型开发](docs/staged_dynamics_p0_r1.md)：七中心连续参考门禁、32条惯量/56条腕部角色与548条最终前序参考；本轮只采集train/selection共60条。已有前序显式绑定为开发参考并重新辨识，不能算新的独立留出。[R2稳定性](docs/staged_dynamics_p0_r2.md)已实现，复用train完成30变体/33次拟合、固定尺度/度量/行权重、前序复核和原生算子独立核验；三档ridge仅获稳定性资格。右腕yaw对角项删组敏感性约9.01%，接近10%合成门槛。[R3归因/selection选参](docs/staged_dynamics_p0_r3.md)已实现：复用R2的B/D解，匹配40阶频带，显式信号不足、原始偏置和带外诊断，独立重算后冻结候选；[R4正式验收](docs/staged_dynamics_p0_r4.md)已通过：新576条标定记录、9条独立控制记录，636+9条路径独立复核、16组步长减半和无真值新进程重评分；275项Python及114项冻结数值回归通过。仅声明范围的合成P0通过，实机/发布标志仍为false。旧schema 1继续运行，不能替代P0验收。
- 该次验收记录为111项Python测试、3项C++测试、114项冻结数值对照、13组ROS实验和764条分阶段轨迹。数字属于历史版本记录，后续结果以实际运行报告为准，不能据此直接宣称新改动通过。
- **尚未连接实机或实现ros2_control硬件插件，尚未验证物理以太网或硬实时能力。** 已新增显式硬件位置联调入口；原S2和S4执行范围不变。当前ROS端点使用wall timer，同机回环不作为跨机性能承诺。
- 2026-10-29已审阅相邻`../casbot2-ros2-sdk`提交`397dc955980956be3ede7376f81a57fafb2b23fe`，完成六包隔离构建与17项SDK离线契约测试；[源码评估](docs/casbot02_sdk_source_intake.md)的[S1](docs/vendor_s1.md)已实现：可选C++只读记录、CDR／回调时刻／端点QoS、字段规范化及能力门禁，统一ROS只读显示兼容厂商两种反馈和已有原生ROS Mock。不创建厂商命令publisher或模式client；S1不含控制；S2本机执行、S3离线资格及S4离线闭环见下条，实机控制和S4现场闭环尚待后续。上身消息是扁平数组，旧PDF嵌套joint不能沿用；PD须核对主机配置/生效反馈，effort不默认等于原始电流或独立力矩。原Mock私有域入口保持隔离，批量虚拟时间实验不强制经过DDS。
- S1/统一显示当轮验收：301项Python测试及42项相关测试通过，可选SDK两包与不加载SDK的原ROS三包构建通过；厂商协议／中断证据为`artifacts/vendor_s1_contract_v3`，原生Mock→ROS→显示为`artifacts/ros_state_view_v1`（1775条，线上的q与显示q差0）。中文视频已实际解码查看；没有实机、跨机或桌面窗口资格。
- [S2本机位置流程](docs/vendor_s2.md)已实现：实际姿态快照／二次核对、Coal连续参考及0.003rad条件包络、C++上身模式与位置流、停止确认／故障拒绝和原始SDK CDR。`position_executor`只允许localhost私有端点，`mock_sdk_bridge`用现有Pinocchio逆动力学实现声明的固定合成伺服，原生`mock_host`仍独占物理推进；实机资格和S4现场执行未开放。`artifacts/vendor_s2_release_v3/review/verification.json`复核6正常＋9故障，停止拒绝另在`vendor_s2_stop_rejection_v1`验证；原始总报告false及v1/v2反馈空窗失败保留，不能据此宣称稳定500Hz。静止断流看门狗另有`vendor_s2_watchdog_v1`，该轮316项Python测试通过。物理停止距离未认证，不把观测插值或模拟负回执验收写成实机停止证明。
- [S3离线观测资格](docs/vendor_s3.md)已实现：S1及主机CSV/NPZ的摘要绑定契约、输出侧力矩/尺度/零位、采样钟域映射、状态与逐阶段原始采样门禁，直接Nm数据无需伪造current；合格周期记录生成Pinocchio140列回归问题，不拟合/放行实机。`artifacts/vendor_s3_release_v2/verification.json`有20项离线检查，旧schema1 train回归量最大差2.62e-12、力矩差2.67e-14，新进程禁止evaluation访问仍可重建；全流程不消费P0正式test。26项新增/全量342项Python测试与三个CLI通过/预期拒绝检查通过，汇总在该目录`delivery.json`。当前SDK未知反馈全部阻断，1kHz静态CSV只获重力观测资格。外部证据的物理真实性仍需人工/设备核验，摘要不是物理证明。首次v1审计脚本导入失败保留，v2新目录重跑。
- [S4离线闭环](docs/vendor_s4.md)已接入：冻结S3观测角色，直接Nm复用原重力/摩擦/起动/惯量求解器，逐阶段失败阻断、惯量后前序复核及双腕正反train/validation覆盖。`vendor_s4_inputs_v1/protocol_v2`绑定346条（旧开发重力64、摩擦210、惯量4、腕部12，新4kHz起动56）；旧1kHz起动不升格，旧train/validation不算新正式留出。`vendor_s4_analysis_v1`四阶段通过，`vendor_s4_native_v1`补采九条控制记录，固定PD前馈RMSE下降约77.00%；`vendor_s4_verification_v1`十项复核通过，禁止evaluation访问的新进程重分析参数差0。最终359项Python测试（含17项S4专项）通过，未知能力三个CLI按预期拒绝。初次spec拒绝旧左右单臂validation角色，原spec/失败日志保留；v2显式映射并保留source_batch。S4实机执行、物理网络和新正式最终test仍未验收，S2 localhost限制保留。所有上述路径均在artifacts下，最终证据以`vendor_s4_release_v1/delivery.json`为准。
- 用户现已启动千分计/TriCal运动学工作包：K1基础已提交`c2d2e6e`，K2/K3已提交`cedac82`，包含合成三表设备、碰撞门禁、自动对中、稳定采样，以及六球选点、Pinocchio几何导数、MDH/Hayati和有界辨识。K4接入同工装三批复测、独立位置计量数据契约和中文只读回放；状态与边界见[K1–K5计划](docs/kinematics_trical_plan.md)、[K3](docs/kinematics_trical_k3.md)/[K4说明](docs/kinematics_trical_k4.md)。实物外测、间隔/拆装/热漂与实机仍未验收。不要把软件编码器零位检查或闭环对中误差写成机械零位/几何标定已完成。

优先阅读：

0. 实机相关工作先读[SDK实机接入前置条件](docs/sdk_hardware_prerequisites.md)，按阶段核对能力、证据和未决项。
   新用户操作入口为[动力学标定启动指南](docs/dynamics_getting_started.md)：Humble 安装、schema 1 Mock、静态参考、SDK 只读及可选 S4 导出；不把教学/回归命令写成 P0 正式或实机执行。更新 CLI/安装入口时同步该文档与 README 示例。
1. [实施计划与进度](docs/humble_migration_plan.md)、[当前完整交付](docs/staged_release_validation.md)。
2. [分阶段流程与碰撞门禁](docs/collision_staged_workflow.md)、[ROS协议与时钟](docs/ros_transport.md)。
3. [Humble环境](docs/humble_environment.md)、[需求对应表](docs/requirements_matrix.md)、[论文落实](docs/references.md)。

本机验收会话为`artifacts/staged_release`，原实验回归为`artifacts/staged_release_regression`，ROS会话为`artifacts/staged_release_ros`。这些是被Git忽略的本地产物，新检出仓库不保证存在；不得伪造缺失的验收证据。`dual-calibrate`及旧双臂文档是历史回归基线，新开发使用`staged-*`入口。

## 架构与目录

- `cpp/include/casbot_calib/`、`cpp/src/`：C++17设备/状态接口、动力学、模拟、反馈保护、碰撞查询及pybind11绑定。批量Runner与ROS Mock共用持久化`ArmSimulationSession`，`simulated_device.hpp`封装共同的设备配置/反馈交付，不复制物理内核。
- `python/casbot_calib/`：激励、时间处理、参数辨识、阶段编排、控制选择、报告与只读回放；`assets/`存随包模型、映射和碰撞缓存。
- `staged_motion.py`/`staged_plan.py`负责完整轨迹与参考门禁，`staged_collection.py`负责采集，`staged_identification.py`/`staged_breakaway.py`/`staged_inertia.py`负责辨识，`staged_control.py`负责四组控制对照。
- `staged_analysis.py`是无真值重分析入口；`staged_validation.py`负责综合验证；`staged_delivery.py`汇总交付及独立合成真值评估；`staged_presentation.py`只读回放。
- `staged_protocol.py`负责P0角色/输入封存与候选冻结；`staged_frequency_contract.py`只做独立噪声/解析频点资格。R0封存版本1仅契约；R1版本2额外绑定实际参考与外部开发输入。`staged_multiconfiguration.py`负责选中心、完整参考及最终前序留出规划；`staged_development.py`负责开发采集/前序冻结/后续最终test薄入口；`staged_multifit.py`负责多train平衡拟合和开发评分；`staged_development_validation.py`负责参考/观测复核、代表轨迹收敛、移走两处真值后的新进程分析。schema 2不得回退到旧全量读取；test绑定冻结候选。`staged_stability.py`编排R2，`staged_stability_math.py`处理受重力约束公共基与比较，`staged_stability_prior.py`缓存原门禁的线性投影，`staged_stability_validation.py`核验封存/Pinocchio算子及跨ridge公共参考。R2只读train，稳定性/前序资格只绑定train；R3归因才允许绑定train+selection，不手填通过状态。`staged_residuals.py`负责完整周期Parseval预算和A–D分量闭合；`staged_attribution.py`编排R3，`staged_attribution_validation.py`从原始数据/Pinocchio独立重算并可冻结候选。归因完成不足以获得资格，还须原始预测与合格频段不退化通过；收益状态单独保存，诊断偏置不可用于选参。`staged_final.py`负责R4预登记、正式采集/只读评分及交付，`staged_final_validation.py`负责全部路径、数值、禁用拟合/采集入口的无真值新进程复核和独立控制审计，`staged_final_presentation.py`消费已送达观测生成中文演示。R4独立验收器/渲染器另存实际源码，核心/模型/规则及原始数据不回写。
- `kinematics.py`负责共享几何身份、具名工具和候选修正，`trical_data.py`负责三表记录，`kinematics_checks.py`仅作K1数值检查。C++ `trical.*`提供理想读数，`trical_mock.*`包装同一持久化ArmSimulationSession产生实际状态读表。`trical_workflow.py`只消费已送达观测，`trical_fixture.py`审计三对测头/目标球接触，`trical_analysis.py`独立复核，`trical_evaluation.py`单独评分。K2不做参数辨识、不模拟表杆接触载荷；实例单一所有者。
- `kinematic_parameters.py`将MDH/Hayati常数参数映射至原生模型，FK与几何回归量仍走Pinocchio；`kinematic_design.py`仅按公开训练设计选点，`kinematic_identification.py`只读观测/计量输入，`kinematic_workflow.py`复用K2采集并隔离真值，`kinematic_export.py`导出候选几何/URDF，`kinematic_evaluation.py`单独评分。参数已写入origin后不能再次加零位到q。
- `kinematic_acceptance.py`负责K4预登记、独立采集与无真值新进程验收，冻结同一训练选参后比较重复性；`kinematic_metrology.py`消费独立配准的静态外部位置记录，未知时间/配准不能推断通过；`kinematic_presentation.py`仅按接收时间回放已送达数据。K4三个训练批次共用K3留出验证会话，不等于新空间外测或间隔漂移验收，产品通过标志保持false。
- `ros/casbot_calib_core`为ament薄封装，`ros/casbot_calib_interfaces`定义协议，`ros/casbot_calib_transport`为C++通信实验端点。
- `ros/casbot_calib_vendor`是可选SDK适配包，包含独立只读`state_recorder`、本机受限`position_executor`及复用原生物理端的`mock_sdk_bridge`；默认`build_ros.sh`忽略它，`build_vendor.sh`单独构建到`install/vendor`。`vendor_identity.py`核对接口和版本，`vendor_capture.py`启动只读进程，`vendor_data.py`保留未知字段并规范化；`vendor_position.py`负责S2快照／完整门禁与封存执行，`vendor_position_analysis.py`独立报告原始频率／响应／故障。`ros_state.py`适配ROS来源，`vendor_viewer.py`复用同一几何renderer处理ROS、文件流和回放。姿态帧允许缺失dq/current/status，不给实机填零或伪造采样序号。
- `dynamics_data.py`负责S3不可变观测和来源/尺度/时钟证据，`dynamics_qualification.py`输出各阶段数据资格（从不授予执行权限），`dynamics_problem.py`将合格周期输出力矩投影为既有Pinocchio刚体求解入口。S1未知配置保持保守，不把SDK effort重命名为电流；高频原始记录/经验证替代观测及S4分阶段验收仍是独立门禁。
- `vendor_staged.py`负责S4角色冻结、观测reader与阶段编排；原`staged_*`数值函数通过显式records/config回调复用，默认旧入口保持原政策。`torque_observations.py`只选择一个总输出力矩通道；S4不伪造current或设备seq。非周期处理要求全记录连续/均匀/同步，腕部30Hz处理不超资格频带；惯量按明确角色分train/validation，不按名字猜。Mock采集脚本可读取合成plant，分析与复核不得读取evaluation；控制对照另存绑定，不能回写已经封存的analysis。
- `dynamic_export.py`提供[动力学候选URDF导出/重加载](docs/dynamic_urdf_export.md)：消费已接受且封存的S4 gravity/inertia，Pinocchio转换空间惯性，每运动关节写一个等效体；固定手部后代保留几何/帧但不重复添加惯性，固定根部属性不标为已标定。摩擦/转子/世界重力留在配套JSON，重加载不能用JSON刚体值覆盖URDF结果。源模型摘要必须匹配；独立输出、新进程校验，不修改nominal或旧分析。静态候选的惯量不可观测部分仍是先验选择，序列化通过不等于实机/参数唯一性通过。本机375项Python测试（含16项新增）通过，两种候选各32构型验证及禁止访问原分析/evaluation的新进程复核通过，证据为`artifacts/dynamic_urdf_release_v1/delivery.json`。新S4分析记录源模型摘要；旧gravity-only缺绑定时拒绝，旧完整S4可使用封存的inertia候选绑定。
- `static_field.py`提供[静态现场预设](docs/static_field_preset.md)：`plan-static-field`/`verify-static-field`仅规划/复核，复用Motion、Pinocchio静态回归与Coal完整路径门禁；配置为`configs/static_field_nominal.yaml`。每臂具名7轴扫描+五轴组合，角度是相对配置基准的偏移；保留正反单轴诊断及成对接近，参考信息选24训练+8验证，不能把组合验证遗漏腕部roll的秩13当作完整覆盖。每臂训练/验证需分别覆盖候选池静态方向；不得宣称140参数或各连杆质量/质心唯一。输出为独立参考规划包，不是可直接运行的staged会话或S4观测协议；1s窗口不是测量，保留缓存/采样资格与S2 localhost限制。nominal肩部外展基准不是实机home，现场到基准的路径、实际插值/跟踪包络、限制和停止仍须另验。失败v1/覆盖不足v2保留，最终离线参考为`artifacts/static_field_nominal_v3`及其review。
- `hardware_field.py`与`configs/hardware_field.yaml`实现H1：实际起姿、单轴小幅、从实测起姿到静态预设再返回的完整序列、原始CDR/整数ns回调/发布stamp/effort记录、探索重力与URDF重加载。`position_executor`共享显式hardware scope（默认S2仍本机），位置/velocity默认配合nominal外部重力effort及主机默认PD；异常停止发布，绝不伪造服务回执。现场fit使用回调窗口，独立采样数未知，不是S3采样资格，正式接受/自动部署保持false。reference与命令插值及观测插值检查不等于真实跟踪/制动包络已获证。快照只读，执行前复核起姿；计划绑定模型/SDK/二进制/ROS domain，分析绑定实际执行CSV摘要。H3已补硬件摩擦/起动候选流程，H4已接惯量与控制候选，不能宣称H1等于完整硬件标定；H2已补整批编排与受控续采，故障恢复按用户要求暂不实现。
- `hardware_batch.py`、`configs/hardware_batch.yaml`及[H2说明](docs/hardware_batch_h2.md)组织冻结批次：方向观测与具名人工结论绑定、同一测得锚点/连续参考下的100/250/500Hz随机重复、静态正反姿态对的原子尝试与有限稳定等待重采。协议/模型/SDK/关键代码不变才续采；事件原子提交并串联摘要，已封存attempt不改写，未知不完整attempt终止。clear_fault默认仅日志占位，调用后整批终止且不能resume。H1的static_means/write_gravity_candidate由H1/H2共用，不复制求解器、不跨记录造采样序列。`hardware_batch_viewer.py`复用统一几何renderer，按单次回调时间回放，试次间不插值、只压缩时间、不放大角度。H2不等于H3/H4硬件摩擦/惯量或正式实机验收。
- `hardware_friction.py`/`hardware_breakaway.py`、`configs/hardware_friction.yaml`及[H3指南](docs/hardware_friction_h3.md)复用H2批次，冻结H1/H2重力、不同姿态/速度匀速、独立验证后冻结摩擦，再进行零PD静置/反馈守护力矩斜坡。复用`staged_motion.add_friction_sweep`及`staged_identification`空间配对/非负鲁棒拟合；C++ `sdk_breakaway.hpp`只在显式H3时启用，不另建SDK Mock算法。正常续采/故障终止约定不变；回调窗口及起动区间为探索证据，不授予采样资格，旧4kHz政策不降低。候选含JSON/CSV、原始来源及中文只读回放。起动允许偏移的Coal门禁可细分时间×关节盒，保留10mm余量、审计排除表和预算不足拒绝。默认560单元仅完成规划/单元检查，本机ROS演练只覆盖1轴3条匀速+4条斜坡，不宣称全14轴执行通过。
- H3本轮本机回归441项Python、4项核心C++、2项SDK检查通过；H1/H2统一SDK回归23项及旧S2单条500Hz往返通过。交付见`artifacts/hardware_friction_delivery_v1/delivery.json`。H3ROS验证范围为1轴7尝试，独立记录指令100/50µs比较位置轨迹/峰值通过，粘住切换的逐点速度2.03%差异单列诊断。原始1msSDK观测仍不取得采样资格；本轮未重跑历史114项冻结数值交付。
- `hardware_inertia.py`/`hardware_periodic.py`/`hardware_control.py`与[H4指南](docs/hardware_inertia_h4.md)接入H3冻结输入、共享多频参考/C3幅值包络激励、回调窗口Fourier适配、原`fit_rigid`物理一致140列求解、独立预测及H3观测重评分。惯量/控制选择通过后才冻结进入下一阶段；五档PD只用selection轨迹，四组control独立，回调误差不逐组优化滞后。完整前馈由Pinocchio批量计算、单独CSV封存并与位置同索引发送；不重复补重力，SDK故障继续整批终止。URDF与摩擦JSON分开，候选不自动部署。未知采样/相位/滤波、弱惯量方向和原专用腕部门禁仍保留，不把低频候选升格为S3/S4/P0。SDK Mock可用inertia_scales改变等效体质心惯性，默认全1；不可让分析读取该合成真值。
- H4本轮本机交付为`artifacts/hardware_inertia_delivery_v1/delivery.json`：456项Python完整回归、最终18项H4专项（与全量有交叠）、4项核心C++/2项SDK检查及旧S2单条500Hz往返通过。SDK Mock仅1轴12单元通过，默认14轴8条不同参考离线门禁通过，不能宣称全14轴已采集。独立Nm预测RMSE约0.00122，所测M00合成误差1.65%，回归秩17/140。选择1.0倍PD，前馈记录差2.78%小于同增益复测波动，稳定控制收益未证明。原始反馈新进程重建/重拟合禁止evaluation且参数差0，100/50µs记录指令回放通过；无实机/物理网络/正式采样资格。H4使用C3幅值包络，避免长高阶匹配过渡造成回摆；不通过放宽跟踪/反馈/碰撞阈值掩盖失败。v1–v5失败保留。
- **SDK Mock收敛约定**：新增[统一入口](docs/sdk_mock_unification.md)，`sdk_mock.py`/`configs/sdk_mock.yaml`只启动一个`sdk_mock_server`，通过共享`SimulatedArmDevice`独占其ArmSimulationSession；SDK路径不再运行内部plant ROS桥。通用`mock_host`复用相同设备组件，每次实验仍只有一个物理状态所有者。新SDK Mock固定device_pd=true让原生内核用本地状态计算PD；普通ROS实验默认不受影响。用户已确认CASBOT主机转发五字段、模组本地PD，固定device_pd=true作为SDK路径约定；本地环具体频率仍未知，见[能力证据](docs/sdk_hardware_prerequisites.md#因克斯模组本地pd能力证据)。H1–H4共用SDK客户端、厂商消息和记录，只切换生成profile中的`sdk_connection.py`连接配置，不再另写直接修改q的集成夹具。新服务器仅position插值到配置参考频率，velocity/effort/PD保持；公开反馈不提供内部采样时间/序号/电流，`evaluation/`内部诊断只给独立测试。用户要求先按简单线性插值处理，默认当前参考到新目标2ms明确为合成假设，time_ref=0不解释成已确认时长。新入口无专用stop RPC；旧mock_sdk_bridge仅保留冻结S2回归，不扩成第二条现场开发路径。H3用户确认effort叠加/零PD有效后，当前Mock默认开启对应接口、关闭运行态内部补偿；旧v5条件与证据保留，不能解锁正式实机采样资格。
- 2026-11-02收敛：`control_policy.py`复用五档PD、缩放和四组差值策略；虚拟采样/SDK回调时间的预处理、资格与接受阈值保持各自原契约。`sdk_process.py`复用H1–H4和legacy S2的参数编码、录制/执行监视及子进程回收；保留S2私有scope/stop语义及现场故障终止规则。SDK Mock行为记录升级backend_v2，五字段下发日志与估算反馈分别命名，不能用外层ROS反馈重新计算的PD充当真实内环请求。旧双进程时序报告保留，不冒充当前基线；新实现用新会话验证。
- 共用执行器启动时允许话题/服务异步发现，在原10s内等服务、期间不下发；超时仍故障终止。H3零PD审计按实际`publish_ns`与确认决策时刻，不按计划槽位推断控制状态。SDK Mock物理推进使用专用线程、SDK使用单线程ROS执行器，进程内队列/快照交接，只有物理线程访问模拟设备；积分异常传回并退出进程，析构先停止并回收物理线程。
- Mock诊断CSV通过独立spdlog有界阻塞队列写盘，反馈先交接、不等诊断格式化或写盘；退出先回收物理线程，再排空诊断并检查错误。`libspdlog-dev`仅为可选SDK包原生依赖，离线核心/绑定不强制依赖它。采集/回放仍只读正常SDK消息，不读内部诊断。当前收敛验收和适用范围见[统一入口的本轮结果](docs/sdk_mock_unification.md)。
- `tests/`、`cpp/tests/`、`tests/installation/`分别覆盖Python、C++和安装后消费者。`configs/`存实验条件，`scripts/humble/`存目标环境自动化。
- `artifacts/`、`build/`、`install/`、`log/`及虚拟环境为生成物，不提交Git。

## DVT2源资源接入（N1）

- 唯一受控源包在`python/casbot_calib/assets/models/casbot_hl12p1_dvt2/`；312个OBJ使用LFS，源数据禁止换行转换，读取前须有实际网格，不得运行时回退到下载目录。目录内`source/`保持供应商原文件，派生模型/场景另存，不直接改原件。
- `model_sources.py`显式选择源装配并解析资源；N1不选默认装配、不运行源MJCF的机械计算，也不把完整源模型当作14轴SDK模型。源URDF身份是必要的模型契约；供应商`.mesh_index.json`仅原样保留，不重复计算全部文件摘要。
- Python wheel携带同源数据；CMake/ament通过`casbot_calib_MODEL_DIRECTORY`提供资源目录。Models为独立安装组件，不让纯Core消费者新增模型安装前提。验证入口为`scripts/humble/verify_dvt2_sources.py`、`tests/test_model_sources.py`及启用`CALIB_CHECK_MODEL_SOURCES`的安装消费者。
- 当前默认仍是旧模型，按替换计划待场景、配置及消费者就绪后在N3统一切换；后续不保留旧默认或fallback。证据与范围见[源模型说明](docs/dvt2_model_sources.md)。

## 必须保留的建模边界

- **Pinocchio**计算FK、质量矩阵、逆/正动力学、速度耦合、回归量和解析导数；**Coal**计算碰撞/距离；**Boost.Odeint**统一积分。优先复用库和工作区，不手写另一套机器人递归力学或刚体回归展开。Eigen用于所需线性代数。
- **MuJoCo只作viewer**：只更新几何与渲染，不调用`mj_step`、`mj_forward`、`mj_inverse`或`mj_collision`承担机械/碰撞计算。Mock、实机记录和回放共享状态接口；不得为了演示放大实际角度或由viewer提供辨识真值。
- C++批量推进期间释放GIL，不逐积分步回调Python。真实设备适配接口不承担虚拟时间推进；不要因新增SDK而复制辨识算法或模拟内核。
- 采用输出侧等效模型，不细化无法调整的减速器内部结构。等效惯量、输出电流—力矩增益和减速比只计一次；PD增益与物理刚度分开。厂家未确认开放的电流环PI、滤波或双编码器数据不能假定可用。
- 完整刚体模型使用标准140参数及物理一致伪惯性PSD约束，Clarabel求解；不可观测部分明确为先验选择的物理代表。`M(q)`、重力和科氏/离心力矩来自同一模型，不分别拟合任意常数矩阵。
- 训练和验证按完整轨迹/独立批次分离。前序阶段验收失败阻止后续；惯量阶段的有限联合修正须重新通过前序验收。固定PD的前馈收益与调PD收益分开报告，不把原模型前馈对照叫作纯反馈对照。
- 保留起动摩擦的粘住/脱离与反馈保护；不以`tanh`零速模型替代静摩擦，不以较晚确认力矩替代起动阈值。耦合停止不能逐轴随意截断速度而注入能量。
- nominal新CASBOT与evt6模型不得混用。几何、关节顺序、工具、模型版本、参数尺度及有效范围均须可追溯；合成默认参数不是厂家规格或实测先验。

## 时序、采样、数据和碰撞约束

- 现有通用Mock默认**命令、主机控制、ROS反馈发布均500Hz**，SDK客户端发送计划仍默认500Hz。厂家声明的位置插值1000Hz与debug反馈发布1000Hz须单独记录，不能据此更改Mock历史条件或宣称实测通过；发布频率、参考更新、实际采用及设备原始采样分别处理。
- 当前通用Mock原始采样1kHz，运动摩擦与惯量4kHz，摩擦另有8kHz收敛对照。1kHz瞬时电流曾导致摩擦混叠，不能只靠后处理低通取消这道门禁。实机4kHz/8kHz能力尚未确认，需主机原始记录、批量回传或独立验证的周期平均观测。
- 使用整数纳秒，分别记录发送、接收、采用、原始采样与反馈接收时刻/时钟域。未知时钟映射不直接相减，RTT不除以二；命令纯延迟、电流响应和反馈偏移分开处理。报告实际频率、抖动和序号缺口，不用配置值替代测量值，不把所有缺口叫网络丢包。
- SDK缓存反馈保留原始发布/接收记录，不对发布stamp直接差分求速度/加速度，不把重复发布计为独立样本；无新帧证据不报告真实刷新率。当前仅发布时刻的记录不满足S3采样时刻条件，静态缓存或低频周期替代观测须另加经独立验证的协议，不能直接降4kHz阈值或把未知滤波设为无滤波。优先准备静态重力；现有起动/惯量精确验收仍待观测资格。ZOH加一阶电流实验不直接代表厂商位置插值链路。
- 当前机械积分最大50µs，时序脉冲25µs；步长在事件边界截断。变更动力学/摩擦/积分需检查步长减半及能量规律，不放宽阈值掩盖误差。
- 位置、速度、电流是观测；电流换算effort不是独立测力。无尺度锚点不宣称绝对惯性恢复。辨识/回放不得读取`evaluation/`，合成真值仅供独立评估；删除真值后应仍能重分析。
- 原始观测、执行指令、条件和摘要不可覆盖。新运行使用新目录；`--resume`只继续已核对条件的会话，中断部分保留，禁止并发写同一会话。重分析另建目录，不跨缺帧直接求导，不改旧数据适配新schema。
- 碰撞门禁覆盖初始保持、接近、测量、制动、回位及停止/恢复；参考和Mock实际观测路径均检查。保留审计过的排除表和保守运动界；未决细分应拒绝，不能仅凭离散点无碰撞认定连续安全，也不能缩小到几乎不动来通过。
- 更换URDF或几何后更新并核验缓存与摘要，不盲用SRDF的Never项。当前10mm余量、夹具代理及实验速度/力矩包络属于合成条件；观测门禁只证明声明的插值，不证明未观测实机运动。

## 主环境与构建

主环境为Ubuntu22.04/ROS2 Humble，使用已有Docker后端distrobox `ros2-humble`；在宿主仓库根目录通过下列入口执行。无需宿主bash激活conda。

```bash
bash scripts/humble/container.sh cmake --preset humble
bash scripts/humble/container.sh cmake --build --preset humble
bash scripts/humble/container.sh ctest --preset humble
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CASBOT_TEST_RENDER=1 python -m pytest -q
```

首次安装见环境文档。Python分析使用系统Python3.10建立的`.venv-humble`，不启用system-site-packages；ROS underlay/overlay由脚本显式接入。原生库用apt/rosdep，Python额外依赖按`scripts/humble/requirements.lock`安装，不用pip再装另一套Pinocchio/Coal。当前验证版本为Pinocchio4.0.0、Coal3.0.3、Boost1.74，旧conda环境只留作对照；不混用PATH、ABI或构建缓存。Windows离线入口及CI保留，本机Linux通过不等于Windows已验收。

**cpp/核心C++变更后必须更新实际加载的绑定**，单独CMake build不会刷新已安装的扩展。仅ros/内的适配节点变化时构建对应colcon包即可，不重编未变化的Python绑定：

```bash
bash scripts/humble/container.sh python -m pip install -e . --no-deps --no-build-isolation \
  --config-settings=build-dir=build/humble --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON
bash scripts/humble/container.sh bash scripts/humble/build_ros.sh
```

ROS消息生成/colcon使用系统Python，分析使用同版本venv；脚本同时设置`Python3_EXECUTABLE`与旧`PYTHON_EXECUTABLE`。长时间采集期间不要替换内核；记录实际运行二进制的摘要，而非稍后重建的同路径文件摘要。

## 运行与验收入口

以下示例输出目录必须不存在；已存在时换名，只有明确继续同一已核验会话才使用`--resume`。完整命令链使用本机已有冻结基线`artifacts/humble_migration/baseline/comparison`；执行前确认存在，缺失时先按迁移计划取得真实基线，不用当前结果冒充旧基线。

```bash
# 原A–F/R1/旧双臂回归、构建、安装消费者和wheel检查；不包含新版M2–M6全部实验
bash scripts/humble/container.sh bash scripts/humble/validate.sh artifacts/my_regression \
  artifacts/humble_migration/baseline/comparison
# 本机真实ROS双端频率/延迟实验及无真值验收
bash scripts/humble/container.sh bash scripts/humble/ros.sh casbot-calib ros-benchmark --output artifacts/my_ros
bash scripts/humble/container.sh python scripts/humble/verify_m2.py --folder artifacts/my_ros --regression artifacts/my_regression
# 新版完整分阶段流程、综合复核与中文视频
bash scripts/humble/container.sh casbot-calib staged-suite --output artifacts/my_staged --fps 12 \
  --ros-report artifacts/my_ros/analysis/report.json
# 汇总已有M1–M6证据；此命令本身不替代前面的实验
bash scripts/humble/container.sh python scripts/humble/verify_release.py \
  --regression artifacts/my_regression --ros artifacts/my_ros --staged artifacts/my_staged \
  --output artifacts/my_staged/release.json
```

`validate.sh`不传基线时也能运行构建和原实验，但其`baseline_metrics_compared`为0；后续`verify_m2.py`与`verify_release.py`要求实际完成114项对照，不能将前一种结果当作完整证据。不要覆盖冻结基线或放宽原容差。

`plan-staged`仅规划；`staged-calibrate --through gravity|friction|breakaway|inertia|all`用于阶段采集/辨识；`staged-identify`只重分析；`staged-verify`做综合复核；`render-staged`只读回放。完整合成交付查看`delivery.json`，M1–M6汇总查看`release.json`，单个检查点通过不能代替全部验收。P0的R4使用`staged-calibrate --final-test 父会话 --attribution-analysis R3目录 --config configs/staged_dual_arm_p0.yaml --output 新R4目录`；只读重评分为`staged-identify 父会话 --through final --final-protocol R4目录 --output 新目录`，回放传R4目录。旧`staged-suite`仍为schema 1路径，不能把P0配置直接交给它。

按改动选择验证：文档改动检查链接、命令及diff；行为变更补有意义的回归/反例；物理内核、时序、几何、辨识或控制变更运行相应数值与阶段验收，并按影响补原实验回归/完整流程。优先独立解析基准、固定种子、明确容差、真值隔离和独立轨迹，不仅验证同一模型往返自证。EGL渲染检查需`CASBOT_TEST_RENDER=1`；演示改动还应查看实际快照/视频。不要为纯文档或低影响可逆修改新增机械式测试。

## 后续推进顺序

当前动力学P0 R0–R4已完成本机合成闭环，交付为`artifacts/staged_p0_r4_release/delivery.json`及`r4_delivery.json`。后续优先甲方设备/ROS接口、实际几何与夹具、尺度/方向/时钟/采样带宽资格，再推进实机独立实验。R4频段/控制收益不代表实机或全频带性能；40阶外残差仍单独保留。新分析用新目录，resume要求同一实现/输入。正式test已经采集，禁止再用当前会话选择候选或改阈值；修改模型/方法后须升级协议并使用新独立测试批次。

1. **确认甲方接口与能力配置**：以现有SDK原生ROS topic为首选；ros2_control仅在交付端已提供时复用，不自行新增重复硬件循环。整段轨迹下发能力未知，不依赖它。确认消息类型/模式、单位、关节映射、采用回执、原始采样/时钟、电流滤波/带宽、限幅与停止方式。不要默认回到SOEM/EtherCAT方案。
2. **实现薄适配并复用现有管线**：优先接甲方已有控制器/hardware_interface，避免另起重复硬件控制循环。保留本机ROS Mock；未知能力显式声明，不填成已知时间/独立力矩。实机连接与下发按用户任务授权范围执行，离线实现和验证可先推进。
3. **做物理网络与设备预检**：独立测物理方向/机械零位、输出增益尺度、时钟映射及频率/延迟；多批次、随机化顺序测试，冻结模型验证，不能逐频重拟合掩盖差异。确认高带宽原始记录或有效替代观测。
4. **替换真实模型/夹具条件后开展分阶段实机实验**：实际几何、安装重力方向、工具与设备限制到位后重做碰撞/过渡/停止门禁，再按重力→匀速摩擦→重力补偿起动→前序验收→惯量激励→独立验证→控制对照推进。真实试验开始前观测并确认起始姿态，不能照搬Mock初始化。
5. **推进运动学K5与实测验收**：K4合成复测/回放之后接实际球架/工具计量、独立外测、间隔复标和实机薄适配。K2默认四球左右各两个仅验测量闭环；K3每臂三非共线基准点消除两球连线转动歧义，仍须SVD检查。固定球架世界变换/工具是合成计量锚点，实机未知时不能套用。10mm非接触余量和三对有界行程接触分开审计；几何误差族另加刚体位移预算。三个表与关节不默认同步，球心位置约束不代表姿态精度。更新共享几何/版本后重验碰撞和动力学；实际千分计工具负载单独建模。其他控制/载荷扩展按观测条件分轮推进。

## 代码风格、Git与交付

- Python四空格；C++沿用周边格式（通常两空格）。函数/模块用`snake_case`，类用`PascalCase`；公共头文件放`cpp/include/casbot_calib/`。未配置统一formatter/linter，不做无关重排。
- SI单位、输出侧约定、整数纳秒及schema/模型版本显式记录。维护CMake、Python安装与ament消费边界，避免引入不必要的强制依赖。
- 不回退用户改动；按用户授权提交，使用本机Git身份，提交聚焦。历史使用简短行动式标题，无强制前缀。PR说明问题、最终行为、验证和实际限制；数值/演示变更附报告或图像。
- 不提交构建、原始采集、生成视频或整套`artifacts/`；确需版本化的大型源资产使用Git LFS。论文优先引用`../casbot_tasks/references/`及已有索引，不重复复制大PDF。
- 更新实现时同步相关文档的已完成/待完成边界；报告合成/本机/实机范围，不把预设值、旧产物或文献精度当成当前设备验收结论。
