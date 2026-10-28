# casbot_calib 协作约定

适用于本仓库全部目录。面向用户的说明、报告和演示默认使用中文，代码标识符沿用英文。开始修改前查看工作区状态，保留用户已有改动。

## 当前目标与已完成范围

- 当前优先级是**整机完整装配、腰部刚性固定的14轴双臂动力学标定**。腰部抱紧位置为前腹部圆形开口附近；实际夹具尺寸、安装变换与重力方向尚需测量，当前盒体仅为合成代理。
- 截至2026-10-28，M1–M6本机合成流程已完成：Humble迁移及原实验回归、真实ROS双端Mock、频率/延迟实验、碰撞门禁、重力→运动摩擦→起动摩擦→惯量分阶段辨识、独立验证和中文视频。
- 动力学[P0计划](docs/staged_dynamics_p0_plan.md)已实现[R0数据契约](docs/staged_dynamics_p0_r0.md)及[R1多构型开发](docs/staged_dynamics_p0_r1.md)：七中心连续参考门禁、32条惯量/56条腕部角色与548条最终前序参考；本轮只采集train/selection共60条。已有前序显式绑定为开发参考并重新辨识，不能算新的独立留出。[R2稳定性](docs/staged_dynamics_p0_r2.md)已实现，复用train完成30变体/33次拟合、固定尺度/度量/行权重、前序复核和原生算子独立核验；三档ridge仅获稳定性资格。右腕yaw对角项删组敏感性约9.01%，接近10%合成门槛。[R3归因/selection选参](docs/staged_dynamics_p0_r3.md)已实现：复用R2的B/D解，匹配40阶频带，显式信号不足、原始偏置和带外诊断，独立重算后冻结候选；[R4正式验收](docs/staged_dynamics_p0_r4.md)已通过：新576条标定记录、9条独立控制记录，636+9条路径独立复核、16组步长减半和无真值新进程重评分；275项Python及114项冻结数值回归通过。仅声明范围的合成P0通过，实机/发布标志仍为false。旧schema 1继续运行，不能替代P0验收。
- 该次验收记录为111项Python测试、3项C++测试、114项冻结数值对照、13组ROS实验和764条分阶段轨迹。数字属于历史版本记录，后续结果以实际运行报告为准，不能据此直接宣称新改动通过。
- **尚未接实机、甲方SDK或ros2_control硬件插件，尚未验证物理以太网或硬实时能力。** 当前ROS端点使用wall timer；同机回环通过不能作为跨机性能承诺。
- 用户现已启动千分计/TriCal运动学工作包：K1基础已提交`c2d2e6e`，K2/K3已提交`cedac82`，包含合成三表设备、碰撞门禁、自动对中、稳定采样，以及六球选点、Pinocchio几何导数、MDH/Hayati和有界辨识。K4接入同工装三批复测、独立位置计量数据契约和中文只读回放；状态与边界见[K1–K5计划](docs/kinematics_trical_plan.md)、[K3](docs/kinematics_trical_k3.md)/[K4说明](docs/kinematics_trical_k4.md)。实物外测、间隔/拆装/热漂与实机仍未验收。不要把软件编码器零位检查或闭环对中误差写成机械零位/几何标定已完成。

优先阅读：

1. [实施计划与进度](docs/humble_migration_plan.md)、[当前完整交付](docs/staged_release_validation.md)。
2. [分阶段流程与碰撞门禁](docs/collision_staged_workflow.md)、[ROS协议与时钟](docs/ros_transport.md)。
3. [Humble环境](docs/humble_environment.md)、[需求对应表](docs/requirements_matrix.md)、[论文落实](docs/references.md)。

本机验收会话为`artifacts/staged_release`，原实验回归为`artifacts/staged_release_regression`，ROS会话为`artifacts/staged_release_ros`。这些是被Git忽略的本地产物，新检出仓库不保证存在；不得伪造缺失的验收证据。`dual-calibrate`及旧双臂文档是历史回归基线，新开发使用`staged-*`入口。

## 架构与目录

- `cpp/include/casbot_calib/`、`cpp/src/`：C++17设备/状态接口、动力学、模拟、反馈保护、碰撞查询及pybind11绑定。批量Runner与ROS Mock共用持久化`ArmSimulationSession`，不要再复制一套物理内核。
- `python/casbot_calib/`：激励、时间处理、参数辨识、阶段编排、控制选择、报告与只读回放；`assets/`存随包模型、映射和碰撞缓存。
- `staged_motion.py`/`staged_plan.py`负责完整轨迹与参考门禁，`staged_collection.py`负责采集，`staged_identification.py`/`staged_breakaway.py`/`staged_inertia.py`负责辨识，`staged_control.py`负责四组控制对照。
- `staged_analysis.py`是无真值重分析入口；`staged_validation.py`负责综合验证；`staged_delivery.py`汇总交付及独立合成真值评估；`staged_presentation.py`只读回放。
- `staged_protocol.py`负责P0角色/输入封存与候选冻结；`staged_frequency_contract.py`只做独立噪声/解析频点资格。R0封存版本1仅契约；R1版本2额外绑定实际参考与外部开发输入。`staged_multiconfiguration.py`负责选中心、完整参考及最终前序留出规划；`staged_development.py`负责开发采集/前序冻结/后续最终test薄入口；`staged_multifit.py`负责多train平衡拟合和开发评分；`staged_development_validation.py`负责参考/观测复核、代表轨迹收敛、移走两处真值后的新进程分析。schema 2不得回退到旧全量读取；test绑定冻结候选。`staged_stability.py`编排R2，`staged_stability_math.py`处理受重力约束公共基与比较，`staged_stability_prior.py`缓存原门禁的线性投影，`staged_stability_validation.py`核验封存/Pinocchio算子及跨ridge公共参考。R2只读train，稳定性/前序资格只绑定train；R3归因才允许绑定train+selection，不手填通过状态。`staged_residuals.py`负责完整周期Parseval预算和A–D分量闭合；`staged_attribution.py`编排R3，`staged_attribution_validation.py`从原始数据/Pinocchio独立重算并可冻结候选。归因完成不足以获得资格，还须原始预测与合格频段不退化通过；收益状态单独保存，诊断偏置不可用于选参。`staged_final.py`负责R4预登记、正式采集/只读评分及交付，`staged_final_validation.py`负责全部路径、数值、禁用拟合/采集入口的无真值新进程复核和独立控制审计，`staged_final_presentation.py`消费已送达观测生成中文演示。R4独立验收器/渲染器另存实际源码，核心/模型/规则及原始数据不回写。
- `kinematics.py`负责共享几何身份、具名工具和候选修正，`trical_data.py`负责三表记录，`kinematics_checks.py`仅作K1数值检查。C++ `trical.*`提供理想读数，`trical_mock.*`包装同一持久化ArmSimulationSession产生实际状态读表。`trical_workflow.py`只消费已送达观测，`trical_fixture.py`审计三对测头/目标球接触，`trical_analysis.py`独立复核，`trical_evaluation.py`单独评分。K2不做参数辨识、不模拟表杆接触载荷；实例单一所有者。
- `kinematic_parameters.py`将MDH/Hayati常数参数映射至原生模型，FK与几何回归量仍走Pinocchio；`kinematic_design.py`仅按公开训练设计选点，`kinematic_identification.py`只读观测/计量输入，`kinematic_workflow.py`复用K2采集并隔离真值，`kinematic_export.py`导出候选几何/URDF，`kinematic_evaluation.py`单独评分。参数已写入origin后不能再次加零位到q。
- `kinematic_acceptance.py`负责K4预登记、独立采集与无真值新进程验收，冻结同一训练选参后比较重复性；`kinematic_metrology.py`消费独立配准的静态外部位置记录，未知时间/配准不能推断通过；`kinematic_presentation.py`仅按接收时间回放已送达数据。K4三个训练批次共用K3留出验证会话，不等于新空间外测或间隔漂移验收，产品通过标志保持false。
- `ros/casbot_calib_core`为ament薄封装，`ros/casbot_calib_interfaces`定义协议，`ros/casbot_calib_transport`为C++通信实验端点。
- `tests/`、`cpp/tests/`、`tests/installation/`分别覆盖Python、C++和安装后消费者。`configs/`存实验条件，`scripts/humble/`存目标环境自动化。
- `artifacts/`、`build/`、`install/`、`log/`及虚拟环境为生成物，不提交Git。

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

- 默认**命令、主机控制、ROS反馈发布均500Hz**；设备原始采样独立设置，不能把发布频率当采样频率。1000Hz通信只作显式对照，历史回归的固定条件保留。
- 当前通用Mock原始采样1kHz，运动摩擦与惯量4kHz，摩擦另有8kHz收敛对照。1kHz瞬时电流曾导致摩擦混叠，不能只靠后处理低通取消这道门禁。实机4kHz/8kHz能力尚未确认，需主机原始记录、批量回传或独立验证的周期平均观测。
- 使用整数纳秒，分别记录发送、接收、采用、原始采样与反馈接收时刻/时钟域。未知时钟映射不直接相减，RTT不除以二；命令纯延迟、电流响应和反馈偏移分开处理。报告实际频率、抖动和序号缺口，不用配置值替代测量值，不把所有缺口叫网络丢包。
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

**C++变更后必须更新实际加载的绑定**，单独CMake build不会刷新已安装的扩展：

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

1. **确认甲方接口与能力配置**：以太网持续ROS topic命令是当前预期，ros2_control是预计接入方式；整段轨迹下发能力未知，不依赖它。确认消息类型/模式、单位、关节映射、采用回执、原始采样/时钟、电流滤波/带宽、限幅与停止方式。不要默认回到SOEM/EtherCAT方案。
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
