# SHM 执行器与物理 Mock 移植计划

先交付**不依赖 ROS、可在本机 Humble 与 Orin 原生编译的 SHM 执行器和物理 Mock**，打通私有端点上的完整指令—动力学—整数反馈链路，再迁移现场上层入口。模型、规划、碰撞和辨识内核继续复用，不按文件数量估算重写比例。

分支基线为 `f360a11`，本地 `ros` 保留该版本，`main` 继续实施。现已有独立 CMake、35槽线格式、真实IPC兼容层、双臂位置/速度指令转换、私有协议夹具和逐轴原型；ID12已完成实机控制流程。当前可用命令及源码部署方式见[原生构建](shm_build.md)与[逐轴原型](shm_joint_test.md)。已增加面向C00–C06的`shm_executor`、共用runtime、时钟映射与fieldwork CLI；通用CSV/手动流、任意五字段输入及物理`shm_mock_server`仍待实现，不将整个M1或M2记为完成。

## 1. 分支与迁移顺序

- `ros`：保存 `f360a11` 的完整 ROS/SDK 路径，包括当前文档与 TriCal 固件，不因主线迁移跟着重构。
- `main`：新增独立 SHM 路径，按下述阶段提交可用增量。每阶段提交后保持工作区无未提交改动；构建和运行产物留在已忽略目录。
- 按2026-11-14最新决定，主线主动退役孤立/过时ROS链路，不长期维护双后端；旧实现回`ros`分支查看。删除前抽出仍被主线消费的数学/文件/记录工具，调用方、CLI、配置和CI一起迁移或退役，不留下可见但不能工作的入口；不要求每个旧ROS实验都重做SHM对应物。具体删除批次见第7节。
- 原 ROS 使用方式留在 `ros` 分支；主线中的旧记录不改写，SHM 新记录显式标记传输与时间语义，不能为了复用读取器伪造 ROS 消息、CDR、SDK 身份或域号。

## 2. 构建判断：Humble 通过是第一步，Orin 还需原生验收

本次本机容器实查：Ubuntu 22.04.5、x86_64、CMake 3.22.1、GCC 11.4；此前 Orin 勘察记录为 aarch64、CMake 3.22.1、GCC 11.4。工具版本接近，适合保持 **C++17 + CMake 3.22** 的共同基线，但本机编出的可执行文件不能直接复制到 Orin 运行。

首轮选择**同一份源码、两处原生编译**，不先搭交叉编译平台。CMake 默认使用宿主工具链；跨架构编译则需显式指定目标工具链和依赖搜索路径，见 [CMake 工具链文档](https://cmake.org/cmake/help/v3.22/manual/cmake-toolchains.7.html#cross-compiling-for-linux)。

### 现有构建与后续边界

根 `CMakeLists.txt` 已按`CALIB_BUILD_CORE`条件查找算法依赖；SHM协议目标可独立构建。当前`shm_joint_test`为读取URDF链接`calib_core`，因此仍需算法依赖。旧vendor目标由`ament/colcon`构建，新执行器不沿用这条构建链。

根 CMake 已提供核心、Python与SHM协议的独立组合，默认保持现有算法构建可用；完整执行器及`CALIB_BUILD_SHM_MOCK`仍待后续。最终边界如下：

| 构建项（拟新增者标明） | 作用 | 依赖边界 |
| --- | --- | --- |
| `CALIB_BUILD_CORE`（已实现，默认 ON） | 现有算法核心及其测试 | Eigen/Boost、Pinocchio、Coal；关闭时不查找这些依赖 |
| `CALIB_BUILD_PYTHON`（已有） | 现有 `_core` 绑定 | 核心、Python、pybind11；核心关闭时须显式关闭，矛盾组合明确报错 |
| `CALIB_BUILD_SHM`（已实现协议库，默认 OFF） | 当前协议库/诊断/测试，后续接执行器 | 当前仅C++标准库；IPC加入Linux POSIX、Threads，执行器配置/记录再引入`nlohmann_json`、spdlog、Zlib；不需要ROS/Python/动力学库 |
| `CALIB_BUILD_SHM_MOCK`（新增，默认 OFF） | 完整物理 `shm_mock_server` | SHM 库与现有 `calib_core`，要求核心和 SHM 开启；不需要 ROS/SDK/Python 绑定 |

`find_package`、测试、安装和导出随对应目标条件化；不只是构建时跳过目标、配置阶段仍强制找全套依赖。复用现有核心，不为第一轮拆一套重复的动力学库；因此完整 Mock 首轮仍需要 Coal，即使其物理积分本身主要使用 Pinocchio。

Humble 中 Pinocchio/Coal 的 CMake 包位于 `/opt/ros/humble`，可以显式把这一前缀用于查找算法库；**算法库安装在 ROS 前缀不等于目标依赖 ROS 通信**。新脚本不 source ROS 环境、不调用 colcon，不链接 `rclcpp`、`rmw`、SDK 消息或 ament。

Orin 按此前记录优先使用系统 Pinocchio 4.0.0、Coal 3.0.3，不继承运控私有 Pinocchio 3.2.0 或整套 `LD_LIBRARY_PATH`。现有根构建请求 Pinocchio 3.9；须实际核对包版本匹配规则及源码 API 对 4.0 的兼容，发现差异在构建/适配处修复，不能凭“已安装”宣称 Mock 能链接。Python 环境和 GUI 依赖不进入第一轮原生构建要求。

### 计划交付的构建入口

已有 `scripts/shm/build.sh` 调用普通 CMake，接受构建目录、安装前缀和依赖前缀，不启动机器人业务或安装系统软件。`shm-native`/`shm-core` preset 在本机与Orin分别原生使用，架构由编译器识别，不写死x86参数，不使用`-march=native`生成部署包。

最终将支持以下组合。**当前仅核心/SHM协议组合已实现；`CALIB_BUILD_SHM_MOCK`及完整物理Mock留M2，不要把下面的未来Mock选项当现有目标。** 当前提前编译物理核心请使用原生构建手册的`--with-core`。

```bash
# 轻量执行器/协议测试：无需 Pinocchio、Coal、ROS 或 Python。
cmake -S . -B build/shm-native -DCMAKE_BUILD_TYPE=Release \
  -DCALIB_BUILD_CORE=OFF -DCALIB_BUILD_PYTHON=OFF \
  -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=OFF -DBUILD_TESTING=ON

# 完整物理 Mock：两架构分别在本机编译，填本次明确采用的算法库前缀。
cmake -S . -B build/shm-physics -DCMAKE_BUILD_TYPE=Release \
  -DCALIB_BUILD_CORE=ON -DCALIB_BUILD_PYTHON=OFF \
  -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=ON -DBUILD_TESTING=ON \
  -DCMAKE_PREFIX_PATH="$CALIB_DEPENDENCY_PREFIX"
```

接着 `cmake --build`、运行 SHM 对应 CTest 和私有端点联跑。先用 `-j1` 限制编译内存，不把编译卡住误判为接口问题；执行器与 Mock 安装到本仓库独立前缀，不覆盖厂家文件。

## 3. 头文件策略：自写小型兼容层，不链接整套运控

**推荐自己维护线格式与适配代码，厂家头文件到位后作对照。** 此前检索没有找到可直接拷回的 SHM 源文件/头文件，已有安装 ELF、DWARF 布局、实际配置及原始帧在本地；无需为找不到头文件停止开发。

实现分成两层：

1. **线格式**：`cpp/include/casbot_calib/shm/wire35.hpp`，固定宽度整数、35 槽、外层对齐空洞、双手布局，编译期检查大小/关键偏移。命令/状态分别为 744/696 字节；不把整个外层标成 packed，也不把手模式枚举的 int 宽度误当线格式的 uint8。
2. **IPC**：`cpp/src/shm/`，共享内存映射、两方向队列、同步和生命周期。线格式正确不等于队列同步正确：依据已有反汇编核对信号量名称/方向、索引更新和内存可见性，调用有界等待。不会用一个“自己和自己能通信”的任意新队列宣称兼容现有主站。

头文件只放消息字段和偏移，不放 `std::string`、指针、平台私有 `sem_t` 或随意构造的 `std::atomic` 对象到消息区。读取 packed 字段用安全复制/编解码，不依赖 x86 能容忍的未对齐解引用；队列同步另按其实际布局处理，在 aarch64 上同样测试。

将可公开的最小布局说明、逐轴方向/零偏配置、几个小型原始帧夹具及来源说明纳入源码树。厂家 ELF、完整系统配置包和大日志继续留在本地 `artifacts/jetson_shm_recon_20261113/`，不是发布依赖。以后取得厂家 `mc_motor_msg.h`、`sem_shm.hpp`、`ring_buffer.hpp`、`semaphore.hpp` 时对照或替换 IPC 适配，不把上层执行器绑定到厂家 C++ 类布局，也不把 aarch64 `.so` 拿到 x86 上链接。

第 4/5/6 项继续采用[既定开发默认](shm_field_validation.md#第-456-项采用开发默认源码到位后复核)：它们的真实行为后续复核，不新增源码到位门禁。基础消息/IPC 的实际兼容性是适配实现本身的工作，不能以这些默认替代。

## 4. 代码边界与复用选择

计划目录如下，避免新增另一份 ROS 节点外壳：

```text
cpp/include/casbot_calib/shm/       线格式、转换、传输接口、执行状态
cpp/src/shm/                       IPC、时钟映射、执行器状态机、物理 Mock 适配
cpp/tools/shm_executor.cpp         唯一 SHM owner 的命令行入口
cpp/tools/shm_mock_server.cpp      私有主站/驱动模型入口
cpp/tests/shm_*                    ABI、转换、队列、时钟与状态机测试
tests/fixtures/shm/                小型原始帧与预期解码，不提交厂家二进制
configs/shm/                      现场假设/整帧策略与独立 Mock 配置
scripts/shm/                      原生构建、私有 Mock 联跑、逐例确认入口
python/casbot_calib/shm_*.py       后续进程启动、记录适配与现场编排
```

### `shm_executor`：一个 owner，观测与控制分开

- 一个进程接真实或私有 SHM，内部分发全轴观测；不再让 recorder 和 executor 各自开厂家消费者。记录与 Python/GUI 都不阻塞周期路径。
- 周期执行只做当前参考求值、整帧编码、入队/出队和状态处理；采用 1ms 绝对单调时钟调度，晚到不连发追赶历史参考。不承诺仅靠 RT 内核就达成硬实时。
- 无控制时保留观测能力；每次控制显式建立子会话。包括初始化、先失能切模式、使能、执行和选定退出路径，完整 F0 包含非目标组与手。实际支持的退出配方按现场确认记录选择，不把 SDK 的模式服务/断流自保护直接搬过来。
- 首条 SHM 写入与首条轨迹执行分开标记；状态毫秒 stamp 映射到同机单调时间，保留 raw stamp、帧号、接收时刻和时钟配对。A 的选窗原点沿用首条执行命令。
- 初始支持只读观测、静态保持、计划 CSV 五字段流和选定单轴用例；随后接入手动信箱与 TriCal 流。重力前馈只在明确的标定配方中启用，手动不隐藏补偿。
- 将现有 `trical_stream.hpp`、`numeric_text.hpp`、`file_stamp.hpp`、`manual_policy.hpp`、`async_csv.hpp`、`gzip_chunks.hpp` 中适用的无 ROS 能力移到公共位置，更新调用方；`vendor_common.hpp` 中混杂的 ROS 消息转换不带入新目标。旧 S2 guard、SDK 缓存和位置插值不作为 SHM 基础依赖。

### `shm_mock_server`：真物理核心，新的 SHM 外壳

- 复用 `SimulatedArmDevice` / `ArmSimulationSession` / `ArmDynamics`，保留物理重力、惯量、摩擦和本地五字段 PD；不新写简化弹簧模型取代现有机械内核。
- 从原始命令做方向/零偏/定点解码后驱动物理模型，再把反馈编码成整数状态。沿用模型电流到输出力矩的 gain，不能把 `ArmSample.current` 直接标作 Nm；当前 SDK 外壳使用的乘 2 来自其模拟 gain，要在新适配中显式绑定该值。
- 私有 FIFO 容量 2、约 1kHz 主站交换、毫秒墙钟状态 stamp、无新命令沿用旧目标、反馈 0.01 Nm 量化；不沿用 ROS 重复缓存、主机位置插值或 SDK watchdog 保护。
- 物理积分步长与 SHM 交换周期分开，继续用已验证的积分器；SHM 基线三通道取同一物理步。模拟延迟、噪声和驱动响应采用明确配置，不凭空加滤波或为亚刻度拟合制造噪声，也不悄悄改变旧 SDK Mock 默认。
- 双臂做完整动力学。腿、腰、头、手首先提供明确标为协议夹具的字段/状态响应，以便测试整帧策略；不把它们宣称为全身物理或腿部四连杆仿真，更不能用夹具结果代替现场保持/释放观察。
- 每次创建本轮独占私有名称，绝不默认打开 `shm_mc`；拥有者清理自己创建的资源，不清理厂家对象。模拟故障可注入，真实主站的故障不清除、不自动恢复。

## 5. 按可交付增量实施

| 阶段 | 修改与交付 | 完成证据 |
| --- | --- | --- |
| M1 协议与独立构建 | 条件化根 CMake；35 槽 codec、方向/零偏/范围检查、时钟映射；IPC 最小适配及私有队列测试；原生构建脚本 | Humble 无 ROS 通信依赖构建通过；原始帧按明确字段值解码，编码检查截断/边界/偏移；队列顺序/满/空/生命周期；同毫秒与时钟跳变处理；Orin 同源码编译及对应测试 |
| M2 私有物理闭环 | 完整 Mock 与执行器只读/保持/计划流，整帧 F0、状态机、异步全轴记录、故障结束路径 | 两个独立进程通过私有 SHM 跑短轨迹；原始命令→物理响应→整数反馈全过程；断流确实维持旧命令、故障不自动 resume、日志阻塞不阻塞发送；双方模型/14轴顺序一致 |
| M3 双架构安装验收与现场脚本 | 在 Orin 独立前缀安装两程序及实际模型资产，私有 Mock 联跑；实现 C00–C06 单用例编排和反馈记录 | Orin 不启动/停止现有厂家控制业务，仅跑私有端点；实际链接库、安装后启动与短闭环通过；记录调度实测。Humble 成功但此项未做时，只交付本机构建结果，不能标记 Orin 已支持 |
| M4 上层迁移 | 新 Python SHM launcher/记录模型；A 显式 observation time；先 H1 短轨迹/手动，再 H2–H4、回零、GUI、K5 分批接入 | `commands/feedback/events/result` 的物理含义与消费者一致；没有伪造 publish/sample/设备身份；数值内核、碰撞和故障合同保持；用户负责 GUI 外观及实机观察 |
| M5 主线收敛 | 移除已替代的 ROS 包、启动器/默认配置/CI 调用，更新 README 和分发清单 | 主线默认路径确实不再要求 ROS；可达入口、测试、文档与依赖一起清理，原 ROS 流程仍能从 `ros` 基线找回 |

M1 的 ABI/协议测试与 M2 物理联跑都必要：仅两端共用同一份错误头文件的往返不能验证厂家兼容。Orin 上编译/私有 Mock 不需要真实控制权，也不依赖非目标轴/停止的现场结论；真实 `shm_mc` 的主动用例另按[现场手册](shm_field_validation.md)由代理运行、用户观察。

### 上层不承诺“零改动”

`hardware_field.run_executor()` 当前同时启动原生执行器和原始 recorder，按 ROS 域号/SDK 身份封存；`vendor_capture.py`、`fieldwork.py`、`fieldwork_zero.py`、GUI、`trical_sdk.py` 也有 SDK 路径绑定。SHM 的单 owner 不能只替换一个 binary 路径就沿用双消费者结构。

保留 SI 数值列、14轴顺序、参考和分析内核；新记录额外保存整帧与 SHM 时间字段，并由新适配显式提供 A 所需观测时间。接入 `hardware_probe.py` 时，同毫秒多帧先按既定时间组权重处理，不能把纳秒接收时刻塞回观测列逃过严格递增检查。原有 `receive_ns` 继续表示接收时刻，不能改名义不改消费者；旧 SDK 封存记录仍按其原语义读取。

记录格式/GUI 连接方式的改变与传输迁移一起完成，不在第一轮强行重写 H1–H4 或 TriCal 数学部分。第 4/5/6 项只留后续复核，正式 S3/S4/P0 仍是各自独立合同，不拿来阻断上述开发与探索。

## 6. ID12之后：先交付明天可用的SHM fieldwork

### 现状与这轮的完成目标

截至2026-11-13，ID12已完成失能→MIT→使能→单轴轨迹→失能应答，用户观察到正常运动。实际反馈未完整跟上±0.5rad目标，后续报告继续显示指令/反馈两组范围，不把流程完成当作跟踪验收。用户随后自行尝试ID13，报告Orin似乎卡住；尚未收集该次记录，原因未知，按用户决定暂不深入排查。

**下一交付优先让现场人员完成：进入独占窗口→全轴观测→预览一轴→显式执行→查看报告→记录观察→停在检查点。** 先做CLI，不等完整GUI、H1–H4或物理Mock全部迁完再开始现场测试。现有`shm_joint_test`继续可用，不在它已有替代前删掉。

### F1：现场入口与记录（已实现本地版本）

已新增`fieldwork/shm/run.sh`及`python/casbot_calib/shm_fieldwork.py`；原ROS目录迁到`fieldwork/ros/`，根脚本保留转发。Shell入口自行定位仓库、使用明确安装前缀，不调用distrobox、ROS脚本、域号或SDK模式服务；Python只编排和离线分析，不做1kHz循环。第一版只需系统Python标准库和本地C++程序；GUI依赖留后面。

下表是当前接口；完整执行参数和接管顺序见[SHM CLI手册](../fieldwork/shm/README.md)：

| 子命令 | 行为 |
| --- | --- |
| `inspect` | 原控制程序仍在时被动快照，不出队、不发命令；保留手部背景来源 |
| `prepare` | 显式调用既有停业务/只启主站脚本；不藏在其它命令或部署中 |
| `activate` | 显式调用既有fresh-master握手；正常连接不自动post或修复旧互斥量 |
| `observe --duration 10` | 独占窗口内持续出队观测，不发控制；打印35槽及双手，双臂额外给SI值，结束生成报告 |
| `run C03 --joint-id 13` | 默认预览配方，不连接SHM；提供`--snapshot`时额外列已保存起点/限位/原始编码，执行仍重读起点 |
| `run C03 --joint-id 13 --execute` | 重新读取起点、检查双向限位，一次一轴，执行后退出，不自动接下一轴 |
| `note RUN_DIR --text '实际观察'` | 离线追加用户原话，保留记录时间与可选阶段，不重新连接设备 |
| `report RUN_DIR` | 离线汇总原始记录；未封存记录显示可恢复前缀与缺失项，不触发控制 |

运动参数沿用原型：ID12–25、±0.5rad、每段5秒、20/2、前馈0；显式参数可以修改，越URDF限位跳过、不夹取或自动缩幅。首轮只保留已授权背景：腿/腰/头失能、其余双臂固定本次实际起点、双手冻结显式来源字节。逐轴用例继续不查碰撞；该例外不传到H1–H4/K5。

每次自动生成新目录，记录实际命令/版本、用例ID、退出码、原始文件、运行前后摘要及观察；正常完成、限位跳过、执行故障、记录不完整分别显示。Orin墙钟不可靠时以本地唯一序号标识运行，墙钟按原值保存，不能当作可信UTC日期。用户不必手动拼`run`变量或从原始整数猜状态。

报告重点：

- 状态机各阶段、逐轴模式/状态字/错误码，尤其区分“程序退出”“收到失能应答”“用户看到停止”。
- 指令与反馈起点、范围、末值、非目标轴变化；有变化就显示，不先加跟踪或背景静止通过门槛。
- 成功入队/出队节奏、跳过/失败次数、反馈间隔、原始stamp重复/倒退、源帧号缺口；不把这些写成逐轴采样率或单向执行延迟。
- 记录是否完整、最后阶段及最后成功发送/接收位置；缺少`result`保留为未完成，不自动续跑。

先基于现有原型完成包装、记录与离线报告；持续`observe`现已由公共runtime提供，不用反复启动`snapshot`冒充持续观测。`inspect/prepare/activate`是独立显式动作，不由`run Cxx`串起来自动执行。

### F2：共享runtime已落地，继续扩通用executor

已从`shm_joint_test.cpp`提取公共runtime/recorder到`cpp/src/shm/field_runtime.cpp`；原型和新`cpp/tools/shm_executor.cpp`调用同一份代码。时钟映射为公共`clock.hpp`，没有复制进入/退出状态机。先保留已经跑通的时序和默认参数，不混入无依据的使能策略改动。

1. 后续补齐任意五字段输入（本轮仍固定effort=0）；双臂q/v/effort解码和命令编码记录已提供。补齐输入时保留方向、零偏、Nm与0.01Nm整数边界及溢出检查。目前`arm_target()`将torque固定为0，不能把它直接当完整五字段接口。
2. 已加同机墙钟/单调钟配对，原始stamp、观测时间、接收时间分别保存；区分首条SHM写入与首条执行命令，按已确定的A时间约定落盘。同毫秒不造新时间，跳钟分段。
3. 已提供单owner纯观测，以及有限时长的`hold`、`jog`；每例结束停在检查点，不在等用户反馈时继续隐式保持。无控制时不发背景帧。
4. 已使用按端点区分的本工具owner锁，避免自己的两个`observe/jog`同时消费；锁随进程退出释放，不删除厂家对象、不接管外部进程。这不代替既定的停业务安排。
5. 已保留有界异步记录，将终端摘要与原始写盘隔离；慢SSH输出不拖住周期，也不应堵住唯一原始写盘线程。按固定短间隔提交可恢复前缀，保存少量阶段/故障元数据，不给每帧强制fsync。
6. 已提供正常退出及SIGINT/TERM/HUP有界失能；故障终止不重启、不清故障。控制线程、记录线程和监督端分别给出状态，程序硬死时不能承诺软件失能。

本轮fieldwork运行一个用例只起一个owner。以后GUI需要持续观测时，由同一owner接受显式控制子会话并分发反馈，不能另起第二个SHM recorder。先不做这个常驻GUI协议，保持runtime边界即可。

通用executor随后增加封存CSV五字段参考输入：显式参考时间、1kHz按当前单调时刻求值、晚到不补发历史命令；插值/求值规则单独规定并测试，不继承SDK插值假设。该能力是接H1的入口，不要求明天逐轴测试前完成。

### F3：私有协议联跑已提供，物理Mock随后接入

现有`shm_joint_integration`只把命令位置复制成反馈，是协议夹具。本轮增加独立`shm_field_fixture`、无pytest依赖的`scripts/shm/verify_fieldwork.py`（也纳入CTest）和`tests/test_shm_fieldwork.py`，覆盖F1/F2的启动、限位零发送、状态机、故障终止、报告、纯观测零发送、重复owner拒绝、正常中断、C05静默及C06被动探针/未完成前缀恢复。测试只用`/casbot_calib_test_*`，不能默认连接`/shm_mc`。

然后新增独立进程`shm_mock_server`，复用`SimulatedArmDevice/ArmSimulationSession`：

- 双臂保留重力、惯量和本地PD；无隐藏重力/摩擦补偿。通过驱动使能状态控制是否施加主动输出，失能不能简单冻结位置。
- 显式设置SHM基线延迟、噪声、交换/积分周期；不直接继承`ArmSimulationConfig`当前1.35ms命令延迟、0.7ms反馈延迟和噪声默认值。基线不凭空加滤波/噪声，整数反馈量化保留。
- 按显式gain把物理current换为输出Nm，再编码；命令effort作前馈，不把反馈总effort回灌。
- 队列容量2留空一槽、每周期取命令、无新命令沿用旧目标；不沿用SDK断流自保护。驱动错误按既定故障流程测试。
- 腿/腰/头/手只提供标明为协议夹具的响应；不做全身动力学移植。

fieldwork的`--mock`最终创建本次独占端点并启动该物理进程，跑相同`observe/hold/jog/report`；只清理自有资源。物理Mock未完成前，不把协议夹具包装成`--mock`交付。

### 明天现场顺序

1. 用户到场后确认Orin是否重启。如果重启，新建接管目录，先保留停业务前手部背景，重新停业务→只启主站→显式握手；不能继续宣称今天窗口有效。若未重启且旧测试状态未知，先用被动信息定位当前进程/主站状态，不靠重复握手解卡。
2. `observe`先测10秒：全轴数据、错误码、模式及节奏；有结果后再选主动用例。若F2持续观测尚未交付，明确只可使用已有`snapshot`，不称为观察测试已完成。
3. 先ID12复现已知配方，再单独新开ID13。ID13本次换轴是否动对、方向、首帧和退出行为由用户观察；不自动循环后续12轴。
4. 逐轴结果正常再继续C03/C04，始终保存全部背景轴。Kp20/Kd2承载不足时先报告实际响应，不为追上±0.5rad自动加增益或补偿。
5. 首帧/正常退出记录充分后，再安排有独立配方的停止命令写入C05、进程消失C06；这两项不是明天必须做完的任务，不以关闭SSH碰运气替代测试。
6. 当天记录和观察落入台账，按实际结论修正runtime/Mock；原始数据不回写。

ID13疑似卡住暂不专项排障。下一次运行仍保留本例命令、终端日志和记录前缀；若再现，停止该例，区分终端/SSH无响应、测试进程退出/阻塞与整机无响应后再决定修复，不自动杀主站、重启或重试。

### 交付优先级与证据

| 优先级 | 可交付结果 | 所需证据 |
| --- | --- | --- |
| 第一批，现场必需 | 定位路径的CLI、逐轴预览/执行、离线报告与观察登记；现有原型仍可单独调用 | 私有协议夹具运行；预览零发送、限位拒绝零发送、单轴正常/故障退出；报告与原始帧对得上 |
| 第二批，优先争取明天可用 | 公共runtime、持续只读、有限时长hold、五字段/时钟完整记录 | 私有只读零发送、状态/退出与原型合同一致、端点owner互斥、时钟和整数边界；Humble构建，再在Orin编译安装和私有运行 |
| 第三批 | 双臂物理SHM Mock与相同fieldwork配方 | 独立两进程走原始ABI、物理响应而非目标复制、量化、旧命令延续、失能/故障路径；两架构原生运行 |
| 后续上层 | CSV参考→H1短轨迹→A时间适配→H2–H4→手动GUI/回零→K5 | 每个消费者按新SHM记录含义接入，复用数值内核；不再启动第二个原始消费者 |

现在用户在家，只做本地开发/私有测试，不连接真实SHM、不跑停业务或运动脚本。Orin原生编译与私有联跑在可连接时完成，真实用例等明天用户在场。F1/F2的现场CLI、公共runtime与私有协议联跑现已落地；原生Orin新版本运行及实际C00–C06验证仍待明天现场。任意五字段/CSV输入、GUI和物理Mock继续按后续阶段实施。

## 7. 下一轮：通用executor、物理Mock与ROS退役（2026-11-14计划）

本轮只制订计划，尚未执行以下源码迁移/删除。基于`627e5dd`继续开发；当前fieldwork单例入口和既有记录保持可用，现场C00–C06结论不作为私有开发的等待条件。

### 推荐顺序：先清旧支线，再打物理闭环，随后迁移上层并删除SDK底座

| 增量 | 主要交付 | 同批退役/修复 | 完成条件 |
| --- | --- | --- | --- |
| R1 公共能力归位＋第一批删除 | 提取后续确实使用的无ROS工具；固定SHM五字段/参考/记录契约 | 退役老ROS stream benchmark、viewer订阅入口及legacy S2夹具链；清掉对应CLI/测试/配置/CI依赖 | 现有SHM fieldwork及离线数值入口仍可运行；没有残留import、CLI和构建目标引用已删链路 |
| R2 通用executor＋物理Mock | 完整五字段codec、计划参考求值、双臂物理`shm_mock_server`、`fieldwork/shm --mock` | 复用R1公共能力，不复制ROS executor/server | 两个独立进程通过真实ABI跑参考；数据体现PD/重力/惯量及整数边界；失能/断流/故障路径正确；Humble、Orin分别原生编译及私有联跑 |
| R3 第一条上层SHM链 | H1短轨迹/静态采集与A观测时间消费者，继而H2–H4 | 分离`hardware_field.py`里的SDK启动器与可复用规划/分析；退役对应vendor采集/归一化依赖 | 新记录能重新分析；不启动第二个SHM recorder；同毫秒和跳钟处理与既定含义一致 |
| R4 GUI/K5消费者与SDK底座退役 | 手动五字段、保存零位/回零、GUI反馈与记录、TriCal持续流逐批迁移 | 删除剩余SDK服务/消息/客户端、`fieldwork/ros`及根转发入口、相关ROS脚本/配置/依赖；更新正式入口和CI | 主线有效入口不再要求rclpy/rclcpp/ament/colcon；离线核心及模型/数学测试保留；ROS旧流程在备份分支使用 |

R1和R2组成下一次主要交付，R3/R4不挤入同一个大提交。删除不以备份为顾虑，但以依赖图决定成组边界。

### 7.1 executor：保留跑通的控制流程，增加真正通用的输入

当前`shm_executor`仍是现场用例执行器：`field_runtime.cpp`同时承担CLI解析、状态机和记录；`arm_target()`将前馈写死为0。下一步拆成有实际调用方的小组件：

- `arm_codec`：双臂q/v/effort/PD双向转换、方向/零偏、整数范围、量化与编码后有效值；不在GUI和Mock各维护一份换算表。
- `reference`：本机单调时间下的参考求值；读文件、配置解析、校验和预计算在周期前完成。
- `executor`：唯一SHM owner、进入/运行/退出状态机、完整F0和每次控制状态；既有C00–C06调用该组件，不再复制控制循环。
- `recorder`：保留全35槽/双手、原始时间和时钟配对，记录实际成功入队及编码后的命令；后台磁盘/界面不参与周期。

**参考方向选定为复用`staged_motion.Motion`的七次Hermite轨迹。** 现有对象已保存knots以及每个结点q/dq/ddq/jerk。将这些冻结为参考bundle，C++按当前单调时间求同一条曲线；不把500Hz位置表重新做一套隐式线性插值。CSV保留为导入/导出与检查表，不能恢复原曲线的输入按显式新参考处理并重新验证。

- 轨迹模式：q与目标速度来自同一参考的值/导数；Kp/Kd及前馈独立冻结，明确其时间表和保持/过渡规则。首版固定PD，前馈表显式ZOH并由规划器输出足够密的时间点；需要更平滑时由参考生成阶段提供，不在执行时自动补偿。
- 手动模式：position、velocity、Kp、Kd、effort仍是可独立指定的五字段，不强迫velocity等于位置目标的导数，允许零PD/非零前馈。
- 原生循环仍1ms绝对单调时钟调度，晚到跳到当前参考、不补发历史帧；队列满计数，状态切换只在成功写入后推进。
- fieldwork手动/逐轴不隐藏补偿；H1等标定配方需要重力时由规划侧明确生成前馈。
- 当前fieldwork双臂用例不查碰撞的例外保持原范围；上层标定参考继续使用既有规划/碰撞合同。

### 7.2 Mock：新SHM外壳，复用唯一物理内核

新增`cpp/tools/shm_mock_server.cpp`及`cpp/src/shm/`适配，使用既有`SimulatedArmDevice`/`ArmSimulationSession`、Pinocchio和Boost积分器。`shm_field_fixture`继续是快速协议夹具，不被改名冒充物理Mock。

现有内核的实际缺口是**逐轴驱动使能状态**：`ArmCommand`只有q/dq/torque/kp/kd，`ArmSimulationSession`没有Controlword/失能语义。需要在通用物理设备边界补充带时间的使能掩码/事件，并保持旧调用默认全使能：

- 驱动使能时计算本地五字段PD＋前馈，纯前馈/零PD照常有效。
- 理想驱动基线中，失能切断有源输出，机械状态继续在重力/摩擦/耦合作用下演化；不能冻结q，也不能把Kp=Kd=0当失能。使能与命令按明确事件顺序生效，旧pending命令不能在失能后重新打开输出。
- 固定基座不等于每个关节有抱闸。机械支撑/制动若进入Mock，必须是单独显式的模拟场景设置，不用隐藏重力保持伪装；实际抱闸/失能衰减未确认的部分记为模型假设。
- 反馈从物理current按显式gain换算Nm，再经过方向和整数编码；0.01Nm/LSB保留，不复制目标到位置反馈，不为亚刻度效果人为加噪声。
- SHM首版明确配置：1ms交换、50µs积分、无人工延迟/噪声、无内部重力/摩擦补偿；响应时间和力矩限幅单独配置，基线值作为合成假设，不从SDK默认的1.35ms/0.7ms延迟继承。
- 无新命令时沿用上次已采用的命令/驱动状态；不加SDK watchdog自动保护。故障状态保持，不自动清故障或重新使能。
- 仅双臂做物理；腿/腰/头/手提供明确标为协议夹具的背景响应。

`CALIB_BUILD_SHM_MOCK`单独控制该目标，要求CORE与SHM开启；不需要Python绑定/ROS。`fieldwork/shm/run.sh --mock`的实际参数位置在实现时统一到对应子命令：自动创建本次独占私有端点、生成本次明确的模拟背景、启动Mock与同一个executor、结束只清理自有资源。每例使用明确初态，不为长期无支撑失能状态偷偷加保持。

修改物理核心时，补使能/失能、重力自由响应、零PD前馈、步长减半及原核心回归；同步重建实际Python绑定。旧调用的时序、物理和默认值保持原合同，不将新的SHM默认改到全部SDK/离线模拟上。

### 7.3 ROS删除清单：删整条已退役链，不留假兼容入口

**第一批，R1可直接退役的独立旧支线：**

- `ros/casbot_calib_transport/`中的`mock_host`/`stream_client`及其ROS benchmark链；`python/casbot_calib/ros_benchmark.py`、`ros_state.py`、对应`ros-benchmark`/`ros-view`命令与专用测试。
- `ros/casbot_calib_vendor/src/mock_sdk_bridge.cpp`、`legacy_position_guard.hpp`及legacy S2专用目标/用例。`vendor_position.py`目前仍被`hardware_field.py`导入，不能先删整个文件：先提取当前仍用的binary定位/通用分析，再移除S2入口与实现。
- 被以上链路独占的配置、探针脚本、colcon构建项；主线CI中ROS benchmark及`--ros-report`消费/旧release聚合依赖一同退役，不填假成功或把SHM指标改名写进旧ROS报告。

**先搬公共部分、再删外壳：**

- `trical_stream.hpp`、`numeric_text.hpp`、`file_stamp.hpp`、`manual_policy.hpp`、`async_csv.hpp`、`gzip_chunks.hpp`：只迁移新消费者确实需要的部分及测试到非ROS公共目录。当前原生recorder已具备的能力不再平行复制。
- `vendor_identity.py`的通用文件绑定工具目前被模型/静态规划/辨识等大量模块导入；提到中性模块后再删SDK身份审计部分，不按文件名直接整删。
- `vendor_data.py`、`vendor_position_analysis.py`、`raw_archive.py`、`fieldwork_streaming.py`等分别混有旧记录解析、公共数学/流式能力，按调用关系拆开。旧SDK采集和缓存不迁入新记录契约，纯数值核保留。

**随后随消费者迁移退役：**

- `state_recorder.cpp`、`position_executor.cpp`、`sdk_mock_server.cpp`、`vendor_common.hpp`及`vendor_capture.py`、`sdk_connection.py`、`sdk_process.py`、`sdk_mock.py`的ROS专用部分。
- 依赖SDK的`hardware_field`启动/封存、旧GUI observer/launch/mock、SDK腕部采集脚本及`trical_sdk`外壳；保留其中规划、数学分析、可视化组件，改接SHM后删旧外壳。
- 当自定义msg不再有消费者时删除`ros/casbot_calib_interfaces/`；当vendor包不再使用ament核心包装时删除`ros/casbot_calib_core/`，最终移除整个`ros/`及旧ROS依赖安装/发现入口。
- 最终删除`fieldwork/ros/`和根目录ROS转发脚本，将正式fieldwork入口转到SHM；不保留空壳或“去备份分支”占位可执行程序。历史文档集中标为ROS分支归档，当前README/帮助只给实际可用主线命令。

保留本机Humble/distrobox容器入口和已验证算法库：这是当前编译工具链，不是应随ROS通信删除的东西。后续去掉`run.sh`的隐式ROS source/colcon路径时，按显式依赖前缀及实际动态库加载验证；不要求为去ROS先搬走`/opt/ros/humble`里的Pinocchio/Coal。Linux SHM测试与跨平台离线核心测试分开，Windows不承担POSIX SHM验收。

### 下一增量的完成线

一条命令启动私有物理Mock，同一executor完成一条小幅C3参考，报告能看到实际PD/惯量响应与整数反馈；原有fieldwork用例继续可用；第一批ROS旧支线及其CLI/测试/CI依赖已退役。随后在Orin原生编译并跑同样的私有闭环，真实运动仍等现场显式调用。通用GUI、全部H1–H4与K5迁移不算进这一增量的完成条件。

### R1执行记录（2026-11-14）

第一批已退役ROS transport/benchmark、ros-view、legacy S2 bridge/guard/位置规划执行及专用配置/验证脚本；CLI与CI不再引用这些链路，旧release聚合脚本随之退役。通用文件绑定归入`artifact_io.py`，时间统计归入`stream_metrics.py`；剩余SDK调用从`sdk_binary.py`定位原执行器，未替换其运行合同。剩余ROS position_executor要求显式hardware scope，不再存在默认S2执行路径。原生回归记录改为`casbot_native_regression_v2`，不再报告ament消费者通过；旧记录不回写。

验证：全Python测试收集923项，无导入错误；54项相关统计/H1/profile/重规划/S1测试通过；剩余ROS position_executor编译通过。R2物理闭环继续实施，不以完成清理代替下一增量。
