# 双臂千分计运动学标定：计划与K1基础

2026-10-28。整机保持完整装配，腰部刚性抱紧。当前使用nominal双臂14轴；evt6右臂旧演示仅为方法参考，参数和结果不直接迁移。动力学M1–M6历史交付保持独立，运动学本轮开始K1。

## 分轮范围

| 轮次 | 内容 | 当前状态 |
| --- | --- | --- |
| K1 | 左右具名TCP、固定坐标系、几何参数约定、三表数据、合成四球工装、独立正确性验证 | 已实现，验证见下文 |
| K2 | 带碰撞门禁的接近/退出，三表Mock噪声/量程/接触/时序/故障，自动对中和稳定采样状态机 | [已接入合成测量闭环](kinematics_trical_k2.md)；不含几何辨识/实机 |
| K3 | 候选姿态与可辨识性选点、几何参数导数、规范自由度处理、MDH/Hayati映射、分层辨识 | [已接入合成几何辨识](kinematics_trical_k3.md)；每臂三个非共线基准点、独立批次留出验证；不含实机验收 |
| K4 | 独立批次验收、误差及重复性报告、真值隔离、中文双臂视频 | 待实现 |
| K5 | 实际工装测量、千分计读取和ROS薄适配、实机低速验证 | 待实现 |

`kinematics-check`保留为K1基础检查，**不是运动学标定命令**，不执行机器人运动、IK、自动对中、参数求解或碰撞认证。K2新增`trical-mock`/`trical-analyse`/`trical-suite`，范围及新合成工装见[K2说明](kinematics_trical_k2.md)。K1/K2没有新增视频；MuJoCo仍只作viewer。

## 测量与模型边界

- 默认末端安装三轴千分计工装，基准球架固定；三个通道对应有明确法向和符号的平面测头。单只千分计或球形测头需要重新定义观测模型。
- 左右臂使用完整同一模型，顺序测量，另一臂状态也记录。共用球架，或使用相对位置已测量的两组球架。共享几何基准不代表已能独立观测所有肩部安装参数。
- 固定基准球的相对坐标需要独立测量和尺度来源；球架在机器人坐标系中的安装位姿与内部球坐标分开保存。K1文件均是合成值，**未证实可达、无碰撞或制造精度**。
- 球心位置约束不能作为末端角度精度验收。最后一个腕轴零位是否可辨识取决于工具偏置和采样姿态；七轴冗余构型要在K3选点时覆盖。
- 几何与柔顺性分层。首版不联合拟合温漂、背隙和未知接触力；工具/负载、接近方向与温度需要记录并控制。

## C++具名坐标系

[arm_dynamics.hpp](../cpp/include/casbot_calib/arm_dynamics.hpp)新增`ArmFrame`：

- `name`：唯一名称；`universe`保留。
- `parent`：`-1`为模型基座，其他值为模型关节索引。
- `placement`：将坐标从该frame转换到parent。基座frame的世界位姿包含`ArmModel.base`一次。

`ArmModel.frames`保存这些无质量观测坐标系。从URDF导入时保留固定link和joint frames，包含掌部和腰部。源URDF固定子树惯性已经聚合，不因增加frame重复加入质量。

`ArmDynamics`与独占`ArmWorkspace`提供：

```python
engine.frame_names()
engine.frame_pose(q, 'left_trical_tcp')       # T_world_sensor, [4,4]
engine.frame_jacobian(q, 'left_trical_tcp')   # [6,14]
engine.frame_batch(positions, 'right_trical_tcp')
# 返回 poses[N,4,4] 与 jacobians[N,6,14]
```

FK/Jacobian由Pinocchio计算。雅可比采用`LOCAL_WORLD_ALIGNED`，前三行为TCP世界线速度，后三行为世界角速度，列顺序与模型一致；另一臂列应为零。批量调用复制输入、独占workspace、计算期间释放GIL，没有逐点Python回调。`ArmWorkspace`的单次Python调用保留GIL。

旧`fk()`输出数量/含义不变；树模型旧`tool_jacobian()`继续拒绝未指定左右的单一TCP。旧串联单臂仍保持原接口。

## 几何身份与参数约定

[kinematics.py](../python/casbot_calib/kinematics.py)加载当前nominal URDF，核对原文件摘要，给`left_palm_base_link`和`right_palm_base_link`附加合成`left_trical_tcp`/`right_trical_tcp`。工具在掌部局部Z方向偏置-100mm，仅为基础测试，不包含实物工具质量/安装尺寸承诺。

`geometry_manifest()`输出独立几何版本和SHA256，覆盖基座、关节拓扑/顺序、原点/轴、观测frame和工具。惯性/摩擦不属于几何摘要；动力学格式仍为`tree_rigid_v1`，几何版本另存，不能因此混淆版本或沿用旧碰撞认证。

K1支持基于**传入名义模型**构造候选几何：

```text
T_origin' = [R_origin Exp(tilt_local) Exp(axis * zero_offset),
             t_origin + translation_parent]
```

| 字段 | 单位/表达坐标 | 约定 |
| --- | --- | --- |
| `zero_offset_rad` | rad，正方向为名义关节轴 | 对纯零位修正，`FK_corrected(q) = FK_nominal(q + zero)` |
| `translation_parent_m` | m，parent坐标轴 | 加到origin平移，不旋转惯性坐标系 |
| `tilt_local_rad` | rad，名义joint局部旋转向量 | 垂直于名义关节轴；轴向旋转仅用zero表示 |

这些是候选修正约定，不是可观测参数列表。`apply_joint_corrections()`不会拟合参数；每次应从冻结的nominal输入构造，不能对已修正结果反复累计。观测`q`和设备限位继续使用输出侧编码器单位，零位已写入模型origin，适配器不得再加一次。已通过纯零位修正下FK、雅可比及逆动力学等价校验。

K3已实现MDH/Hayati与Pinocchio链的等价映射，采用1mrad/0.2mm参数尺度、训练数据选列和条件数门禁，详见[K3说明](kinematics_trical_k3.md)。不能把所有origin六维误差、零位、基座/工具变换全部无约束放开。基座规范、球架安装、首轴零位、工具与末轴零位存在耦合；先锚定规范再报告缩放后的SVD和可观测子空间条件数。位置雅可比是对关节角求导，**不能直接充当几何参数雅可比**；K3使用Pinocchio关节放置回归量及参数映射导数。

## 三表观测几何与数据

[trical.hpp](../cpp/include/casbot_calib/trical.hpp)提供理想的mastering后几何投影：

```text
z = A R_world_sensor^T (p_ball_world - p_sensor_world) + z_center
offset_sensor = solve(A, z - z_center)
```

`A`每行为sensor坐标下单位测量法向，可以记录实际非正交和反向通道；`z_center`为对中读数。球/主球半径差和仪表原始符号应经独立mastering折算到这些配置，不能再次加半径。函数检查旋转、轴退化和有限数；**只计算理想几何，不判定接触、行程、接触力或对中完成**。K2的设备Mock才负责接触状态和噪声等行为。

[trical_data.py](../python/casbot_calib/trical_data.py)定义`trical_observations_v1`。一行是关联记录，不默认三表与关节同时采样：

| 数组 | 形状/类型 | 内容 |
| --- | --- | --- |
| `q_rad`、`dq_rad_s` | N×14，float | 实际观测；不是指令位置 |
| `joint_sequence`、`joint_sample_ns`、`joint_receive_ns` | N，int64 | 关节包序号、采样/接收时刻 |
| `joint_sample_known` | N，bool | 未知采样时刻必须存-1 |
| `indicator_readings_m` | N×3，float | 按x/y/z通道保存、已归一化到米的原始仪表读数 |
| `indicator_sequence`、`indicator_sample_ns`、`indicator_receive_ns` | N×3，int64 | 各表独立序号及时间，不伪造同步 |
| `indicator_sample_known` | N×3，bool | 未知时间=-1，不能用接收时刻冒充 |
| `indicator_status` | N×3，uint8 | VALID=0、NO_CONTACT=1、OUT_OF_RANGE=2、STALE=3、DISCONNECTED=4 |
| `ball_id`、`tool_frame` | N，Unicode | 外部基准球和具名测量工具 |

JSON保存schema、joint order、几何内容/摘要、工装配置/摘要、SI单位、通道顺序、状态码、关节/三表各自的采样时钟域和接收时钟域。只有明确同域时才检查采样先于接收；不自动把跨域时刻相减或给出“已同步”。

NPZ/JSON/摘要文件仅写入新目录，读取核对文件和几何摘要，禁止pickle。VALID通道必须读数有限且在声明量程内；无效通道保留原读数或NaN及状态，不先删掉故障记录。此处验证的是存储契约，不是采样稳定、无丢包、可用于辨识或真实接触的验收；时钟映射、缺帧、稳定窗口和重复旧反馈等在K2处理。未来仪表适配器还应保存原始设备报文和单位换算来源。

## 运行与验证

主环境沿用[Humble](humble_environment.md)，没有新增依赖。可直接运行：

```bash
bash scripts/humble/container.sh casbot-calib kinematics-check \
  --config configs/kinematics_trical.yaml --output artifacts/kinematics_k1
```

输出目录须不存在，默认配置随Python包发布，因此不传`--config`也能运行。YAML当前只支持选择固定profile和控制周期；修改合成工装的源码配置见[profile](../python/casbot_calib/assets/trical_nominal_v1.json)。默认命令周期2ms/500Hz不代表千分计采样率。

- `report.json`：16组固定种子姿态的原始URDF XML独立FK对照、完整6×14雅可比有限差分、跨臂独立性、解析三表符号校验。
- `geometry.json`、`profile.json`：共享模型与合成条件。
- `protocol_example/`：保存/读取示例；全部标记NO_CONTACT，未知仪表时间=-1，**不是已对中采集数据**。

独立XML坐标链仅位于验证模块；运行FK仍使用Pinocchio，不新增替代后端。C++额外以解析平面二连杆验证位置/雅可比，以90度旋转场景验证读数符号。Python回归包括全部URDF link、基座变换、非法数据反例、整数纳秒精度、摘要篡改和纯零位一致性。安装消费者和隔离wheel也检查新公共接口与随包profile。

本机K1报告：FK最大矩阵元素差`6.66e-16`，雅可比有限差分最大差`1.10e-10`以内，对侧臂列为零。它们是实现数值一致性，**不是机器人实物位置精度**。

本轮Humble完整回归通过：131项Python测试（含20项K1专项）、4项CTest、原A–F/R1/旧双臂114项冻结数值对照；独立CMake/ament消费者、隔离wheel及原viewer回归通过。证据存`artifacts/kinematics_k1_regression/verification.json`和该目录的测试/比较记录。本次没有重跑新版M2–M6完整采集、物理网络或Windows验收，不复用旧版本通过标志冒充新验收。

## 后续验收与衔接

K2状态机为：预检→接近→量程/接触检查→小步修正→等待稳定→采样→退出。每一步有超时、仪表失联、超量程和退回策略。Coal覆盖全部运动段；仅允许指定测头与目标球的受限接触，不能排除整个球架。当前profile未生成工装碰撞模型，不能交给实机执行。

K3训练与验证按整组姿态/构型和独立批次分开；共用球架的留出验证仍共享计量系统误差。无独立外测时报告“基准球位置约束验证”，不称全空间绝对精度或末端姿态精度。需求0.7mm/内部0.4mm、条件数200和60分钟保留为待明确口径的产品目标。论文自动对中平均30s，双臂各80点仅对中就约80min；选点需联合可辨识性和实测节拍，不照搬论文点数。

几何发布后，所有相关碰撞缓存、TCP和动力学包必须绑定新版本；先重验重力/逆动力学预测，必要时重辨识。工具质量/质心和装配载荷仍需单独来源，增加无质量观测frame不意味着已正确建模实际千分计工装负载。

参考：[TriCal原论文](https://doi.org/10.3390/s20205919)、[旧evt6演示](../../casbot_tasks/scripts/trical_sphere_calibration/README.md)、[需求对应表](requirements_matrix.md)、[现有分阶段动力学](collision_staged_workflow.md)。原论文PDF保留在相邻task，不重复复制。
