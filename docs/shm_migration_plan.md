# SHM 执行器与物理 Mock 移植计划

先交付**不依赖 ROS、可在本机 Humble 与 Orin 原生编译的 SHM 执行器和物理 Mock**，打通私有端点上的完整指令—动力学—整数反馈链路，再迁移现场上层入口。模型、规划、碰撞和辨识内核继续复用，不按文件数量估算重写比例。

分支基线为 `f360a11`，本地 `ros` 保留该版本，`main` 继续实施。第一增量已开始交付独立 CMake、35槽线格式库和离线ABI诊断；当前可用命令及源码部署方式见[原生构建](shm_build.md)。`shm_executor`、物理 `shm_mock_server`、IPC/转换/时钟等尚待后续，不将整个M1或M2记为完成。

## 1. 分支与迁移顺序

- `ros`：保存 `f360a11` 的完整 ROS/SDK 路径，包括当前文档与 TriCal 固件，不因主线迁移跟着重构。
- `main`：新增独立 SHM 路径，按下述阶段提交可用增量。每阶段提交后保持工作区无未提交改动；构建和运行产物留在已忽略目录。
- 不先整目录删除 `ros/`。先从中提取仍有价值的无 ROS 工具，再迁移调用方；某条主线路径已有 SHM 替代、调用及测试已更新时，可以删除被替代的 ROS 适配，无需长期维护双后端兼容开关。
- 原 ROS 使用方式留在 `ros` 分支；主线中的旧记录不改写，SHM 新记录显式标记传输与时间语义，不能为了复用读取器伪造 ROS 消息、CDR、SDK 身份或域号。

## 2. 构建判断：Humble 通过是第一步，Orin 还需原生验收

本次本机容器实查：Ubuntu 22.04.5、x86_64、CMake 3.22.1、GCC 11.4；此前 Orin 勘察记录为 aarch64、CMake 3.22.1、GCC 11.4。工具版本接近，适合保持 **C++17 + CMake 3.22** 的共同基线，但本机编出的可执行文件不能直接复制到 Orin 运行。

首轮选择**同一份源码、两处原生编译**，不先搭交叉编译平台。CMake 默认使用宿主工具链；跨架构编译则需显式指定目标工具链和依赖搜索路径，见 [CMake 工具链文档](https://cmake.org/cmake/help/v3.22/manual/cmake-toolchains.7.html#cross-compiling-for-linux)。

### 现有构建需要改哪里

当前根 `CMakeLists.txt` 在配置时无条件查找 Eigen、Boost、Pinocchio、Coal，即使关掉 Python 也一样；现有 vendor 目标则由 `ament/colcon` 构建，链接 ROS 和 SDK 消息。新执行器不沿用这条构建链。

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

## 6. 本计划之后的第一步

M1的独立CMake、35槽线格式与双架构构建已完成；核心及持续模拟设备测试也已在Orin通过。接下来完成IPC、方向/零偏转换、时钟映射，再接M2的执行器和完整物理Mock。两进程私有联跑仍待实现，1kHz实机表现留到显式现场运行测量。当前证据见[构建结果](shm_build.md#2026-11-13-实际部署结果)。
