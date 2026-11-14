# SHM 原生构建与 Orin 源码部署

当前提供可独立消费的`casbot_calib::shm`协议库、IPC/时钟工具、共用runtime的`shm_executor`与[`shm_joint_test`原型](shm_joint_test.md)，以及双臂物理`shm_mock_server`。执行器支持C00–C06、冻结七次Hermite参考和独立五字段。用法见[fieldwork手册](../fieldwork/shm/README.md)；CSV导入、在线手动信箱和H1–H4/GUI/K5消费者仍待迁移。

## 构建、测试和安装

本机 Humble 不需要 ROS 环境或 Python venv：

```bash
bash scripts/humble/exec.sh --no-venv -- bash scripts/shm/build.sh
install/shm-native/bin/shm_abi_check
```

Orin 或其它 Linux 原生环境直接运行相同构建脚本：

```bash
bash scripts/shm/build.sh
```

默认使用 `build/shm-native`、`install/shm-native`，单线程编译并运行全部所选测试，安装仅 Shm 组件。可用 `--build-dir`、`--install-prefix`、`--jobs` 修改；CMake 3.22、C++17 和普通 Make 即可，不查找 Eigen、Boost、Pinocchio、Coal、ROS 或 Python。

也可直接使用 `cmake --preset shm-native`、`cmake --build --preset shm-native`、`ctest --preset shm-native`。现有 `dev/conda/humble` preset 及核心/Python 默认构建保留。

执行器读取URDF需要核心，完整物理Mock使用`--with-mock`（同时开启核心）：

```bash
bash scripts/shm/build.sh --with-mock \
  --build-dir build/shm-core --install-prefix "$PWD/install/shm-core" \
  --dependency-prefix /opt/ros/humble
```

此组合包含现有核心、模拟设备测试、SHM协议/fieldwork及物理闭环测试。只需要执行器可用`--with-core`。算法库前缀显式选择，不source ROS或继承厂家运控`LD_LIBRARY_PATH`。算法包自身可能通过CMake查找其Python开发依赖；本工程Python绑定仍关闭。

也可用`shm-physics`的configure/build/test preset，并通过`CMAKE_PREFIX_PATH`显式提供算法库前缀。选项`CALIB_BUILD_SHM_MOCK=ON`要求`CALIB_BUILD_CORE=ON`与`CALIB_BUILD_SHM=ON`。参考解析复用已有Boost.PropertyTree，不新增JSON、ROS或SDK运行依赖。

```bash
# 在本机Humble工具链原生构建：
bash scripts/humble/exec.sh --no-venv -- bash scripts/shm/build.sh --with-mock \
  --build-dir build/shm-core --install-prefix "$PWD/install/shm-core" \
  --dependency-prefix /opt/ros/humble
# 本机容器内运行安装后的双进程闭环，无numpy/pytest/Python绑定：
bash scripts/humble/exec.sh --no-venv -- python3 scripts/shm/verify_physics.py --bin-dir install/shm-core/bin
```

`Shm`组件安装执行器/Mock，`Core`组件安装算法静态库及其消费接口；`--with-core/--with-mock`构建脚本会安装两者。运行仍需Pinocchio/Coal等外部动态库。模型URDF从仓库解析，脚本不复制Models组件。跨架构必须在目标机重编，不能传x86_64二进制到Orin。

### 安装包的外部调用

```cmake
find_package(casbot_calib REQUIRED CONFIG COMPONENTS Shm)
target_link_libraries(my_target PRIVATE casbot_calib::shm)
```

独立消费测试为 `tests/shm_installation`，配置时指定 `CMAKE_PREFIX_PATH` 为安装目录。`Core`/`Shm` 组件分别加载依赖与导出目标；原来不写组件的核心调用仍可用。构建脚本不复制大型 Models 组件，完整模型资产按后续部署步骤另行提供。

## 通过 SSH 从本地 Git 克隆到 Orin

直连网络使用现有脚本。2026-11-14接线后的网卡名为`enp0s20f0u1c2`（更换USB口后与原默认名不同）：

```bash
bash scripts/jetson_network.sh up enp0s20f0u1c2
```

配置为本机`172.16.0.20/24`、Orin`172.16.0.10`，不设置网关或DNS。若该网卡正被其它连接占用，脚本明确拒绝；本次确认它是等待DHCP的“有线连接 2”后，仅断开该连接，再启用专用profile。Wi-Fi默认路由不变。

在本机仓库根目录运行：

```bash
bash scripts/shm/deploy_git.sh --host nvidia@172.16.0.10
```

默认新克隆到 `/home/nvidia/casbot_calib_shm`。脚本在本机 loopback 临时启动**只读 Git 服务**，通过 SSH 反向隧道让 Orin 执行 `git clone`；源码不经过 GitHub，也不 push。没有公网监听，不写 SSH 密码；认证使用正常 SSH 交互、agent 或 ControlMaster。

脚本正常或异常结束时关闭自己创建的转发与 Git 服务。Orin 的 origin 为 `git://127.0.0.1:19418/casbot_calib/.git`，仅隧道开启时可用；后续重跑同一脚本即重新开隧道并 `fetch/merge --ff-only`。它拒绝远端改动、不同 origin/分支和分叉历史，不 reset 或清理远端目录。

本地默认要求工作区已提交；如其它任务正修改固件，可显式加 `--committed-only`，只部署 main 已提交内容，不包含也不改动那些工作。`--destination`、`--branch`、`--port` 可显式修改；已有目标不会被覆盖为新仓库。

这一步只取 Git 源码，`GIT_LFS_SKIP_SMUDGE=1` 保留大型资产指针，无需 Orin 安装 Git LFS 或访问远程 LFS 服务。35 槽帧夹具和 URDF 文本在 Git 中，可运行本轮协议及 URDF 动力学测试；碰撞网格、视频和 GUI 的完整资产须在对应接入阶段物化，不能把指针当模型资产已部署。

克隆后通过普通 SSH 在 Orin 运行上面的 `build.sh`，构建与安装都在新仓库内部。不会停止现有控制、启动 EtherCAT 主站或访问真实 `shm_mc`。机器人重启不影响这些离线构建步骤；真正主动测试前仍需重新走停业务流程。

## 本增量的验证范围

- 编译期检查消息/手部大小、关键偏移与外层对齐；固定夹具独立预期字段检查，保留所有字节，拒绝错误长度，验证非对齐输入复制。
- 独立安装后 `find_package(... COMPONENTS Shm)` 的 C++ 消费者可链接运行。
- SHM 单独构建不需要算法库；核心开启时原测试继续使用原数值模型。
- 队列同步、双臂五字段定点转换、分段时钟映射、原始记录和私有物理联跑均有测试。真实通道采样/滤波、力矩尺度、退出物理行为仍按[现场手册](shm_field_validation.md)确认。

### 2026-11-13 实际部署结果

编译验证的代码提交为 `5f6eefb`（后续文档更新不改变该代码）。Orin 通过上述 SSH 隧道从本机直接 Git 克隆，目录为 `/home/nvidia/casbot_calib_shm`，未安装系统依赖或修改厂家目录。

| 环境 | 实际结果 |
| --- | --- |
| Humble / x86_64 / GCC 11.4 / CMake 3.22.1 | SHM 独立构建、2项协议测试、安装消费通过；原核心及Python目标编译通过，合计7项CTest通过，原核心安装消费通过 |
| Orin / aarch64 / GCC 11.4 / CMake 3.22.1 | SHM独立构建、2项协议测试、安装消费通过；核心＋SHM构建与安装通过，7项CTest全部通过，含刚体动力学和持续模拟设备测试；核心安装消费及组合安装中的Shm独立消费均通过 |

Orin 使用 `/opt/ros/humble` 的 Pinocchio 4.0.0、Coal 3.0.3 和系统 Boost 1.74；该 Pinocchio 版本文件接受现有 `3.9` 最低版本要求，本轮无需修改版本门槛或算法源码。构建时 `LD_LIBRARY_PATH` 为空。Orin未编译本工程Python绑定；其NumPy/Python环境另留后续上层移植，不由本轮C++验收覆盖。

程序安装在 `install/shm-native` 和 `install/shm-core`。`shm_abi_check` 实际为 ARM aarch64 ELF，输出命令/状态 744/696 字节、小端、64位。动态链接检查使用系统算法库，未链接ROS通信库。

Orin墙钟仍为1970，本轮日期是本地执行记录标记。系统库文件时间晚于墙钟，Make出现`Clock skew`告警，可能重复重链；本轮使用新构建目录完成编译并实际运行测试。未校时或改厂家文件；后续若安排校时，应另用新构建目录，避免沿用跨时钟跳变的增量缓存。

完整远端日志保存在本机 `artifacts/shm_cmake_orin_20261113/native_build.log`，安装消费/动态链接检查在同目录 `installed_consumers.log`。这些是离线代码与安装验证，未运行真实 SHM 或机器人动作；大LFS资产尚未物化。

### 2026-11-14 通用执行器与物理Mock的本机结果

Humble/x86_64已编译完整核心、绑定及SHM/Mock，12项CTest通过。59项相关Python测试通过，后续新增4项自有IPC回收、启动失败、配置和错误报告测试通过；最后的配置工厂复用与反馈类型调整后，模拟设备、codec和物理CLI三项针对性CTest通过。实际加载的Python扩展已更新；全Python测试收集937项，无导入错误（不是937项全部执行）。

安装后的系统Python标准库联跑通过，证据为`artifacts/shm_round2_20261114/installed_physics/verification.json`：同一执行器运行小幅七次参考，物理反馈不等于目标；C05静默区间零写入、驱动保持使能，结束失能并回收私有IPC。最后安装版的一行C03物理运行也已完成，记录为`final_installed_cli/`，执行完成且Mock正常退出/无回收错误。默认10秒无命令观测另存`default_observe/`，没有隐藏保持。协议单独构建/测试/安装仍通过，日志为同目录`protocol_build.log`。

本机交付时网络未连通，Orin验收随后按下节完成。2026-11-13的ARM结果仍只覆盖当时版本，不回写旧证据。

### 2026-11-14 Orin R2原生编译与安装验收已完成

代码提交`cddeab4`已通过SSH Git隧道从`e368e55`快进至Orin的`/home/nvidia/casbot_calib_shm`。使用前述`build.sh --with-mock`命令原生构建，核心与SHM组件安装到`install/shm-core`；没有修改算法源码、安装系统依赖或编译本工程Python绑定。

- 环境：ARM aarch64、GCC11.4、CMake3.22.1、`5.15.148-rt-tegra`，使用`/opt/ros/humble`的Pinocchio4.0.0、Coal3.0.3及系统Boost1.74。12项CTest全部通过，包含fieldwork协议用例和双进程物理闭环。
- 安装后系统Python标准库验收通过：七次参考使用物理反馈，C05静默窗口150条反馈保持使能且零命令写入，最后失能并正常回收自有IPC。参考运行3558条反馈；目标周期1ms，实际STREAM最大发送间隔约4.00ms。
- 独立五字段入口另用显式零重力夹具验证：Kp=Kd=0、velocity=0.05rad/s、effort=0.129Nm编码为0.12Nm，81条STREAM命令字段正确，物理反馈出现输出，结束收到失能应答。该夹具只用于软件语义验证。
- 安装后的`shm_executor`、`shm_mock_server`确认为ARM aarch64 ELF；清空`LD_LIBRARY_PATH`/`PYTHONPATH`即可启动，动态库解析正常，没有链接ROS通信库。

在Orin重现安装后验收：

```bash
cd /home/nvidia/casbot_calib_shm
python3 scripts/shm/verify_physics.py --bin-dir install/shm-core/bin
# 或运行单个现场配方的私有Mock：
bash fieldwork/shm/run.sh run C03 --joint-id 12 --amplitude .01 --segment-s 1 --mock --execute
```

本机完整证据为`artifacts/shm_orin_round2_20261114/`：`network.txt`、`environment.txt`、`deploy.log`、`build.log`、`installed.log`，以及从Orin取回的`installed_physics/`和`direct_fields/`原始记录/报告/verification。后两目录同时保留在Orin仓库同名artifacts目录下。

Orin墙钟仍为1970，Make出现已知的未来文件/Clock skew告警；本轮编译、安装及实际测试成功，未校时。当前已观察到重新开机（连接时uptime仅数分钟），**旧接管窗口结束，本轮没有新开实机控制窗口**。真正运动测试前重新走停业务→只启主站→fresh-master握手；本轮只跑私有端点，不改变C00–C08实物台账。


### 2026-11-14 Orin R3 H1/A数值环境与安装闭环已完成

源码`ef3e5e0`已部署到`/home/nvidia/casbot_calib_shm`，包括`db53ef9`的H1在线限制及`7f4bbde`的跨架构固定frame恢复。Orin已原生构建Python绑定与全部SHM目标，安装目录仍为`install/shm-core`；项目数值解释器为`.venv-shm/bin/python`。本轮直连接口为`enp0s20f0u1u4c2`，已配置172.16.0.20/24。

- ARM64/Python3.10离线wheelhouse保存在Orin的`artifacts/shm_h1_wheels_20261114/`。venv使用NumPy1.26.4、SciPy1.15.3、Clarabel0.11.1、pybind11 2.13.6，不更改系统Python/厂家环境。CMake显式定位venv的pybind11，避免误选系统2.9.1。
- `setup_h1.sh`同时构建数值绑定与SHM程序；独立URDF导出/重加载复用`/opt/ros/humble`中Pinocchio4.0.0/eigenpy/Coal的Python绑定，项目venv仅链接这三个模块。清空`LD_LIBRARY_PATH`和`PYTHONPATH`的数值导入及实际导出已通过，不source ROS。
- Git源码部署已恢复；Git LFS3.8.1仅安装到Orin用户的`~/.local/bin`，仓库配置跳过自动smudge。默认模型的碰撞缓存已复制，37MiB已物化网格通过rsync定点同步；模型绑定照常检查全部包资源，其它模型/原CAD不因此全量下载。
- ARM解析的固定观测frame与冻结值最大差`1.1102230246251565e-16`。加载器只在固定frame矩阵的绝对差不超过`8×float64 epsilon`时恢复冻结数值，并再次要求精确几何摘要；运动轴几何、frame拓扑、URDF/资源绑定保持精确。证据为`orin_geometry_difference.json`和12项相关模型回归。
- 首次完整数值构建12项CTest通过。H1在线限制增量后重编共享执行层及三个工具，新增限制测试和受影响的fieldwork/物理CLI三项CTest通过；其它未变核心结果复用，不把历史12项写成新13项全量重跑。
- **安装版H1/A私有闭环通过**：小幅参考4983条反馈；两姿态/四窗口静态采集5478条反馈；A可计算，PSD拟合及候选URDF独立重加载通过。连续参考与接收位置折线均检查；这不是全64姿态或任何实机运动验收。

Orin复现入口：

```bash
cd /home/nvidia/casbot_calib_shm
bash fieldwork/shm/run.sh h1 --help
.venv-shm/bin/python scripts/shm/verify_h1.py --bin-dir install/shm-core/bin \
  --output artifacts/新的私有验收目录
```

操作手册见[SHM H1/A](shm_h1_a.md)。本机证据为`artifacts/shm_h1_r3_20261114/`：`orin_setup_final.log`、`orin_final_validation.log`、`orin_h1_final.log`、`orin_final_entrypoints.log`、`orin_assets_rsync.log`、`reload_readback.log`及取回的完整`orin_h1_final/`。对应原始闭环在Orin的`artifacts/shm_h1_r3_orin_20261114_final/`保留。

本轮只跑私有SHM端点，没有停业务/启动真实主站或发送机器人命令。Orin墙钟仍为1970，构建日志有未来文件告警，测试时间依本机执行记录标识，不冒充设备时间已校准。用户下一步完成fieldwork与H1实机验证；GUI随后推进。
