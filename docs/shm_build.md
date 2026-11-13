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

**新版Orin原生验收待补。** 本次原直连网卡已不存在，172.16.0.10的ping/SSH未连通；没有部署新版或接触真实SHM。恢复网络后，先提交并用上面的Git部署入口同步，再在Orin运行`build.sh --with-mock`及安装后的`verify_physics.py`。2026-11-13的ARM结果只覆盖当时版本，不覆盖本次Mock/参考实现。
