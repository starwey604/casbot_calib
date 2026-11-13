# Jetson 共享内存接口只读勘察 2026-11-13

## 结论与重构方向

本次 SSH 找到了可用于离线适配的安装二进制、调试类型、实际配置与运行日志，**没有在已检索目录中找到 SHM 中间件的源文件或可直接使用的头文件**。不必再仅凭 PDF 猜结构布局；可以先据本次材料开发私有 SHM Mock 和执行器，再用厂家头文件闭合实现细节。

最影响实现的发现是：

1. 本次主站实际选择 **35 槽兼容 ABI**，内部类型却已扩为 40 槽。客户端命令/状态结构为 **744/696 字节**。
2. 运控在 SHM 边界执行方向变换、定点转换和**非零关节零偏**，这些不能随 ROS 一起删掉。
3. 状态 `timestamp` 是主站转换结束时取得的 **`gettimeofday()` 毫秒时间**；`frame_counter` 是主站转换计数，不是各轴采样序号。
4. 客户端初始化会写共享队列索引，`receiveData()` 也会消费队列。不能把厂家客户端直接作为旁路只读 recorder。
5. 当前路径中命令 `crc=0`，状态 `crc=65535`；字段存在不代表本层实现了有效 CRC32。
6. 主站读取命令失败后仍继续使用原缓冲；不能沿用“SDK 客户端断流后机体自保护”的假设来设计 SHM 停止。

因此第一轮应做**独立于 ROS 的执行/观测边界**，而不是给原执行器替换消息收发。物理模型、规划、辨识和碰撞内核继续复用；控制权交接采用下述用户确定的停业务后独占测试流程，非目标轴策略和物理停止另行核对。

## 本次接入与证据位置

- 本机接口：`enp0s20f0u2u2c2`，`172.16.0.20/24`；目标：`nvidia@172.16.0.10`。
- 本地采集标记：见 `artifacts/jetson_shm_recon_20261113/collection.json` 的 `created_utc`。
- Jetson 内核：`5.15.148-rt-tegra`，aarch64；本次系统实时时钟在 **1970-01-01**。未修改时钟；远端原始时间不能冒充本机 UTC。
- 所有检查为只读文件/进程信息读取、无 attach/run 的 ELF 调试分析，以及 `O_RDONLY`/`pread` 字节快照。未实例化厂家 SHM 客户端，未等待/释放信号量、未出队、未清队列，未切模式、停进程或发送控制。
- SSH 结束时原 `ethercat_station`（PID 5609）和 `hlorin`（PID 6154）仍存在；这只是进程存在性观测。

本地材料根目录：`artifacts/jetson_shm_recon_20261113/`（Git 忽略，不提交厂家二进制）：

| 路径 | 内容与用途 |
| --- | --- |
| `ssh_readonly.log` | 第二段 SSH 的原始终端记录，含源码查找、配置、启动日志和线程状态 |
| `interface_findings.json` | 布局、14轴方向/零偏、时间字段及队列生命周期的结构化结论，供后续适配开发读取 |
| `binaries/` | 下载的 `ethercat_station`、`libhl_ethercat.so`、`libhl_common.so` |
| `site_materials.tar`、`site/` | 按原目录留存的配置、脚本、两个启动日志、`libmanager.so`、`libjoint_publisher.so` 等 |
| `offline/master_types.txt`、`client_types.txt` | GDB 从安装 ELF 的 DWARF 解析出的类型和偏移 |
| `offline/*_interface_evidence.txt` | 关键函数反汇编，按符号及地址检索 |
| `offline/function_evidence_index.json` | 关键符号与对应证据文件索引 |
| `offline/vendor_source_paths.txt` | 编译时源文件路径线索，不代表这些源码在设备上存在 |
| `passive_queue_snapshots.jsonl` | 约 5 秒、1000 次被动队列快照，保留两槽原始字节与读取前后队列头 |
| `passive_summary.json`、`passive_latest_frames.json` | 快照统计与末次原始字段解码 |
| `fixtures/*.bin` | 首次快照的命令/状态各两槽及队列头，供离线协议测试 |
| `collect_passive.py`、`analyse_passive.py` | 本次只读探针及本地解码脚本 |

## 安装版本与实际配置必须分开识别

`dpkg-query` 与 `version.desc` 显示本次安装包：

| 包 | 安装版本 | 构建信息 |
| --- | --- | --- |
| `embed-fusion` | `2.3.7` | 20260811_1140，commit `48b4f3f101f6d9d1fe54ba6e580bce73d3456a18` |
| `casbot-motion` | `0.0.32` | 20260903-101057，commit `60eae1cb15904d49f2ba2ed7840d70856095928b-dirty-unpush` |

主站启动日志另打印 `VERSION:v2.1.3`。包版本、程序内版本字符串、旧现场记录中的 deb 文件名是不同线索，不能互相替代，更不是设备身份。

有三处特别容易读错的配置：

- `/workspace/global_robot.yaml`：`model=casbot_02`、`max_joints=35`、`physical_connection=1`、`motion_to_station=[6,6,7,7,3]`、`hand_type=2`。主站启动日志确认采用 35 槽、双臂各 7 轴、P1L 手的布局。
- `/workspace/embed_fusion/config/global_robot.yaml`：另有 `max_joints=40`、双臂各 5 轴、另一手型等默认内容，**不能把这一份直接当本次有效配置**。
- 运控启动日志明确加载 `/workspace/robot_config/motion/motion_params.yaml`，零偏文件也在该目录。`/workspace/hl_motion/hl_config/joint_pos_offset.yaml` 的全零值不是本次实际使用的零偏。

主站启动日志记录 1 个 `CAS_EAT2CANFD` 从站，PDO 发送/接收大小为 818/761 字节，SM 模式、默认周期 **1,000,000 ns**。YAML 的 `perform: 1000000` 附近有误标“微秒”的注释，以此日志的 ns 单位和被动计数证据区分。启动时有 WKC 异常记录，未将其解释为当前持续故障或做恢复操作。

## ABI 已从两端安装 ELF 核对

`libhl_ethercat.so` 的 `CommandSharedMemory`/`StatusSharedMemory` 与主站 `hl::ethercat::*SharedMemory35` 对应：

| 类型 | 大小（字节） | 关键偏移（字节） |
| --- | ---: | --- |
| `JointCmdData` | 18 | Controlword=0，pos=2，vel=6，torque=10，mode=12，kp=14，kd=16 |
| `JointStatusData` | 16 | Statusword=0，pos=2，vel=6，torque=10，mode=12，error=14 |
| 命令35槽 | 744 | frame=0，joint_count=8，hand_count=10，joints=12，hands=642，timestamp=728，crc=736 |
| 状态35槽 | 696 | frame=0，joint_count=8，hand_count=10，joints=12，hands=572，timestamp=680，crc=688 |
| 主站内部命令40槽 | 832 | joints=12，hands=732，timestamp=816，crc=824 |
| 主站内部状态40槽 | 776 | joints=12，hands=652，timestamp=760，crc=768 |

外层结构**不是整体 packed**，存在对齐空洞和尾填充。每个 `HandCommand` 为 40 字节，`HandStatus` 为 52 字节；实际 `HandJointCommand.mode` 是 **uint8_t**。PDF 中列出了 `HandControlMode`，而调试信息中的枚举类型本身是 int，不能把它直接代入字节布局。

本次 `shm_mc` 为 16 MiB，队列布局结合构造代码及被动快照得到：

- 偏移 0：命令方向；偏移 8 MiB：状态方向。
- 每个方向前 40 字节为队列头，其后为两个固定大小槽。
- 队列头字段：int32 magic、int32 capacity、int64 count、int64 writeIndex、int64 readIndex、int64 maxReadIndex。
- 本次 magic 为 `0x11223344`，capacity 为 2。两槽轮换不等于广播订阅或任意多消费者可独立回读。

证据：`master_types.txt`、`client_types.txt`；主站 `command35ToInternal`（0x985b8）、`statusInternalTo35`（0x9872c）；客户端构造函数（0x10220）。

## 运控转换不能省略零偏和方向

`libhl_ethercat.so` 的双臂/头腰转换覆盖全局索引 **12..28**。以本次运控上层坐标 `q,v,effort,Kp,Kd` 表示，安装代码的转换为：

```text
s = +1 if joint_direction is true else -1
pos_target    = trunc(s * (q - offset) * 1,000,000)
vel_target    = trunc(s * v * 1,000,000)
torque_target = trunc(s * effort * 100)
kp_raw        = trunc(Kp * 10)
kd_raw        = trunc(Kd * 10)

q_feedback      = s * pos_real * 0.000001 + offset
v_feedback      = s * vel_real * 0.000001
effort_feedback = s * torque_real * 0.01
```

这是**软件转换公式**，物理关节方向、模型零位和输出力矩尺度的独立验证仍需现场实验。下发整数使用截断路径，不能默认为四舍五入；新适配应先检查可表示范围，不能靠窄整数溢出实现限幅。

本次实际配置（按每臂肩 pitch、roll、yaw、肘 pitch、腕 yaw、pitch、roll）：

| 轴组 | 方向符号 | 零偏 rad |
| --- | --- | --- |
| 左臂12..18 | `+,-,-,-,-,+,+` | `[-0.019698, 0.00643665, 0.005416, 0.007078, 0.026015, -0.025037, -0.007323]` |
| 右臂19..25 | `-,-,-,+,-,-,+` | `[-0.013976, -0.00224065, -0.01494833, 0.007841, -0.01723733, -0.01549655, 0.00541233]` |

例如直接把模型位置乘 1e6 写入原始槽，会同时丢掉方向和最大约 0.026 rad 的零偏，远大于当前 K5 的位置参考管。新 SHM profile 必须显式保留这层坐标绑定，实机观察起点不能沿用 Mock 起姿。

`JointStatusData` 没有 Kp/Kd 字段。可以保存发送的增益，但不能把命令快照叫作模组生效增益回读。GUI 的缺省显示应据新后端能力处理。

证据：`convertUpperJointCmdToSharedMemory`（0x8290）、`convertSharedMemoryToUpperJointStatus`（0x8ce4），写线程中的偏置相减、读线程中的偏置相加，以及 `site/workspace/robot_config/motion/` 的两份实际配置。客户端的浮点常量另存于 `offline/client_layout_constants.txt`。

## 时间字段和 CRC 的真实含义

主站 `EthercatDynamicBuffer::convertToStatusSharedMemory`（0x7c808）：

1. 清零内部状态结构；将本地成员计数递增并填入 `frame_counter`。
2. 根据配置和缓冲区转换各轴状态。
3. 调用 `HL::TimeUtil::current_datetime_ms()`，写入 `timestamp`。
4. 把 `crc` 写为常数 **0xffff**。

`current_datetime_ms()`（0x17c07c）实际调用 `gettimeofday()`，计算 `tv_sec*1000 + tv_usec/1000`。它是**主站软件转换时的墙钟毫秒时间**，不是编码器或驱动采样时间；时钟校准也可能影响它的单调性。

客户端写线程用 `steady_clock` 的毫秒数填命令 `timestamp`，递增自己的命令计数，`crc` 写 0。**命令和状态 timestamp 使用不同的时钟来源，不能直接相减计算延迟。**

本次约 5 秒被动快照：

| 指标 | 命令方向 | 状态方向 |
| --- | ---: | ---: |
| 首/末最高槽帧号 | 869953 / 874948 | 869945 / 874940 |
| 计数增量 | 4995 | 4995 |
| 按探针本机 monotonic 时长折算 | 999.965 次/s | 999.965 次/s |
| timestamp 首/末原值 | 925003 / 929998 | 937743 / 942738 |
| crc 观察值 | 0 | 65535 |
| joint_count 观察值 | 29 | **0** |
| hand_count 观察值 | 2 | 2 |

快照持续 4.995173448 s；1000 次快照均观察到读取前后队列头相同，但**没有获得原子帧或逐轴采样一致性保证**。计数增长证明本次约 1kHz 的软件交换/转换活动，不证明每轴 1kHz 新采样。状态 `joint_count=0` 时槽内12..28仍有状态值；不能仅靠该字段推断本机有效轴，需与厂家核对。

末次槽中12..28轴的 Statusword 均为 567（0x0237），mode=-1、error_code=0。这只是本次被动字节解码，未做故障或停止试验。

## 队列、停止和接管是独立的执行契约

### 厂家客户端不是无副作用观察器

客户端 `RingBufferQueue<StatusSharedMemory>::SetRingBufferShareArgs`（0xfc60）在连接已存在的队列头后，仍将共享的 writeIndex、readIndex、maxReadIndex 置零；命令方向也走对应初始化。`Dequeue`（0xd0b0）会复制槽、CAS推进共享 readIndex并减少 count。

因此不能并行启动第二个厂家客户端来“只收反馈”。推荐一个原生进程拥有真实 SHM 收发，再将收到的全轴数据分发给 recorder、GUI和监督逻辑。离线 Mock 需显式描述出队、满队列和启动行为，不能用广播内存快照冒充同一协议。

本次探针只用 `os.open(O_RDONLY)` 与 `os.pread` 读取字节，不调用上述构造/出队函数。它用于取证，不作为标定采集器或控制反馈入口。

### SHM 断流不等于 SDK 断流

主站 `Update`（0x87d8c）调用 `receiveData` 后，不依据其返回值中断后续路径，继续把缓冲转换为内部命令、生成 PDO 请求。已检查路径未建立以命令年龄为依据的停止结论；驱动器自身 watchdog 和抱闸仍需厂家说明及实测。

此外，安装 ELF 中名为 `disableMotor()` 的函数（0x8c78c）为空实现；另有 `PdoPowerOff()`，但未调用，也未证明其适合作为标定停止方式。不能把符号名存在或写 `0x06` 等同于物理安全停止。

客户端写线程本次按全部29个电机关节的 enable 标志填 `0x0f`/`0x06`。在绕过 `hlorin` 后，原“只填14轴名称、其它轴由运控保持”的协议不再自动成立。非目标轴和手部如何处理仍待确定；控制者交接与中途不重启插队按下一节约定。本次没有停止任何现有控制者。

### 用户确定的停业务后独占测试流程

**采用流程：停业务 → 单独启动EtherCAT中间件 → 本仓库EtherCAT/SHM测试 → 下次Orin重启结束本轮窗口。** 用户2026-11-13明确指定：窗口中途认定不会有其它程序启动插队。这一项作为运行假设采用，不再要求证明所有自启动机制均被永久移除；不做永久disable/mask，也不自动恢复原业务或旧控制会话。

第二次只读SSH已找到厂家总停止入口：

```bash
sudo bash /workspace/embed_fusion/bin/launcher.bash stop
```

核对的脚本路径为`main → auto_stop → stop_apps`。`stop_apps`先终止`monitor_process.bash`，然后停止24组配置应用，包含`hlorin`、行为树、手柄发布/订阅、控制仲裁和旧EtherCAT主站；旧主站走`run_ethercat_station.bash stop`。`auto_stop`还请求停止`hl_orin_app.service`，并清理启动器（保留`set_cmd`进程）。实际unit的`Restart=no`。脚本的通用`kill_process`使用SIGKILL，不能当作机器人受控停稳流程。

**覆盖范围需要补齐两项独立服务**：本次`zero_calibration.service`为active，`arm_debug_tool.service`为inactive，两者都是独立unit、`Restart=on-failure`，不在上述24组停止清单中。因此停业务阶段先显式停止这两个unit，不能只杀其进程。正式操作顺序为：

```bash
# 在Jetson上，由现场人员安排承重/支撑后显式执行；会中断现有控制。
sudo systemctl stop zero_calibration.service arm_debug_tool.service
sudo bash /workspace/embed_fusion/bin/launcher.bash stop

# 确认停业务阶段完成后，仅启动EtherCAT主站，不重新启动launcher/hlorin。
sudo bash /workspace/embed_fusion/bin/run_ethercat_station.bash
# 主站保持运行，本仓库测试使用另一个终端。
```

停业务完成时一次性核对主应用服务与原控制进程已退出；随后到重启前按用户假设使用该测试窗口，不增加中途竞争者发现/自动抢权逻辑。重启后必须重新执行停业务步骤。

本次`/tmp/launcher/flag_manual_run.flag`和launcher白名单文件均不存在，脚本会选择全部24组。如果该flag存在，停止入口会改为逐项询问，并受对应模块选择影响；须完成全部业务停止，不能把交互中的部分选择视为同一流程。不要通过删除flag或改厂家脚本来跳过选择。

本轮**仅核对源码、unit与进程状态，未执行上述命令**。SSH结束时原主站、运控和零位网页进程仍在。取证记录为`artifacts/jetson_takeover_recon_20261113/ssh_readonly.log`；该流程安排不确定非目标轴处置，也不提供驱动断流或物理停止证明。

### 实时调度有现成参照，但仍要实测

启动日志和线程快照显示：PDO线程 FIFO 50/CPU4；主站 update FIFO 50/CPU5；运控 EtherCAT 读/写线程 FIFO 50/CPU5。同一CPU上已有多个实时线程，不能因使用 SHM 和 RT 内核就宣称无延迟或没有抖动。新执行器的记录、压缩、规划不能阻塞周期路径，信号量等待也必须纳入超时/退出设计。

## 可以开始的开发与仍需取得的材料

现在可开展：35槽布局的离线编码/解码、原始槽夹具测试、显式方向/零偏转换、独占私有 SHM Mock、原生定周期执行和进程内观测分发。复用已有 `SimulatedArmDevice`；不默认搬入 SDK 的缓存重复发布和位置插值。

厂家材料请求已经可以具体到文件：

- 客户端 `hl_common/sem_shm/{mc_motor_msg.h,sem_shm.hpp,ring_buffer.hpp,semaphore.hpp}`。
- 主站 `component/share_memory/master_shm/` 三个头文件，以及 `shm_compat_types.hpp`、`ethercat_data.h`。
- 主站状态转换和请求/断流处理的相关源码；或针对本次安装版本给出同等可核验接口说明。

这些路径来自 DWARF 编译记录；本次查到它们在设备安装目录中缺失。用户随后决定：命令采用/满队列/帧号检查、通道更新/滤波及高精度力矩/亚刻度探针效果先按[开发默认](shm_field_validation.md#第-456-项采用开发默认源码到位后复核)推进，留到源码到位或后续数据复核，不阻断 SHM 执行器、私有 Mock、A 路径适配与现场探索。非目标轴和实际停止按现场逐例确认安排；正式观测的采样和尺度证据另按其原合同处理。控制者交接流程已按上节用户决定收敛，不再列为待定项。

部署方面，本机系统已有 CMake 3.22.1、GCC 11.4、Python 3.10.12、系统 Pinocchio 4.0.0/Coal 3.0.3；运控私有目录另带 Pinocchio 3.2.0。新程序应隔离链接路径，避免继承整套运控 `LD_LIBRARY_PATH`。未加载 ROS 环境的系统 Python 探测中 Clarabel、Pinocchio、Coal、MuJoCo 不可导入；本次未安装依赖或进行编译验收。

S1/S3/S4/P0正式资格、模型/数据绑定和已有碰撞阈值不因本次勘察改值。本次只读材料支持后端设计，不包含 SHM 实机控制验收。

后续现场确认按 [SHM 逐组确认手册](shm_field_validation.md)开展：代理逐例运行、用户观察反馈，先落清完整背景帧、非目标部位及首帧/停止行为；用例和脚本契约已登记，实机用例均未运行，脚本尚待实现。A 路径采用现有状态毫秒 stamp 映射到同机单调时间的约定也记在该手册中。
