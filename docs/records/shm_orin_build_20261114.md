# Orin 原生构建与部署记录（2026-11-13 ~ 2026-11-14）

### 2026-11-13 实际部署结果

编译验证的代码提交为 `5f6eefb`（后续文档更新不改变该代码）。Orin 通过 SSH 隧道从本机直接 Git 克隆，目录为 `/home/nvidia/casbot_calib_shm`，未安装系统依赖或修改厂家目录。

| 环境 | 实际结果 |
| --- | --- |
| Humble / x86_64 / GCC 11.4 / CMake 3.22.1 | SHM 独立构建、2项协议测试、安装消费通过；原核心及Python目标编译通过，合计7项CTest通过，原核心安装消费通过 |
| Orin / aarch64 / GCC 11.4 / CMake 3.22.1 | SHM独立构建、2项协议测试、安装消费通过；核心＋SHM构建与安装通过，7项CTest全部通过，含刚体动力学和持续模拟设备测试；核心安装消费及组合安装中的Shm独立消费均通过 |

Orin 使用 `/opt/ros/humble` 的 Pinocchio 4.0.0、Coal 3.0.3 和系统 Boost 1.74；该 Pinocchio 版本文件接受现有 `3.9` 最低版本要求，本轮无需修改版本门槛或算法源码。构建时 `LD_LIBRARY_PATH` 为空。

程序安装在 `install/shm-native` 和 `install/shm-core`。`shm_abi_check` 实际为 ARM aarch64 ELF，输出命令/状态 744/696 字节、小端、64位。动态链接检查使用系统算法库，未链接ROS通信库。

Orin墙钟仍为1970，本轮日期是本地执行记录标记。完整远端日志保存在本机 `artifacts/shm_cmake_orin_20261113/native_build.log`，安装消费/动态链接检查在同目录 `installed_consumers.log`。

### 2026-11-14 通用执行器与物理Mock的本机结果

Humble/x86_64已编译完整核心、绑定及SHM/Mock，12项CTest通过。安装后的系统Python标准库联跑通过，证据为`artifacts/shm_round2_20261114/installed_physics/verification.json`：同一执行器运行小幅七次参考，物理反馈不等于目标；C05静默区间零写入、驱动保持使能，结束失能并回收私有IPC。

### 2026-11-14 Orin R2原生编译与安装验收已完成

代码提交`cddeab4`已通过SSH Git隧道从`e368e55`快进至Orin的`/home/nvidia/casbot_calib_shm`。原生构建核心与SHM组件安装到`install/shm-core`。

- 环境：ARM aarch64、GCC11.4、CMake3.22.1、`5.15.148-rt-tegra`，使用`/opt/ros/humble`的Pinocchio4.0.0、Coal3.0.3及系统Boost1.74。12项CTest全部通过。
- 安装后系统Python标准库验收通过：七次参考使用物理反馈，C05静默窗口150条反馈保持使能且零命令写入，最后失能并正常回收自有IPC。参考运行3558条反馈；目标周期1ms，实际STREAM最大发送间隔约4.00ms。
- 独立五字段入口另用显式零重力夹具验证：Kp=Kd=0、velocity=0.05rad/s、effort=0.129Nm编码为0.12Nm，81条STREAM命令字段正确，物理反馈出现输出，结束收到失能应答。
- 安装后的`shm_executor`、`shm_mock_server`确认为ARM aarch64 ELF；清空`LD_LIBRARY_PATH`/`PYTHONPATH`即可启动，没有链接ROS通信库。

本机完整证据为`artifacts/shm_orin_round2_20261114/`。

### 2026-11-14 Orin R3 H1/A数值环境与安装闭环已完成

源码`ef3e5e0`已部署到`/home/nvidia/casbot_calib_shm`。Orin已原生构建Python绑定与全部SHM目标，安装目录为`install/shm-core`；项目数值解释器为`.venv-shm/bin/python`。

- ARM64/Python3.10离线wheelhouse保存在Orin的`artifacts/shm_h1_wheels_20261114/`。venv使用NumPy1.26.4、SciPy1.15.3、Clarabel0.11.1、pybind11 2.13.6。
- 独立URDF导出/重加载复用`/opt/ros/humble`中Pinocchio4.0.0/eigenpy/Coal的Python绑定，项目venv仅链接这三个模块。
- Git LFS 3.8.1 安装到 Orin 用户的 `~/.local/bin`，默认模型的碰撞缓存与 37MiB 视觉网格已物化。
- ARM解析的固定观测frame与冻结值最大差`1.1102230246251565e-16`（不超过`8×float64 epsilon`时恢复冻结数值）。
- 安装版H1/A私有闭环通过：小幅参考4983条反馈；两姿态/四窗口静态采集5478条反馈；PSD拟合及候选URDF独立重加载通过。

### 2026-11-14 可复用准备位、统一起点默认与H1实机

- `f465f7c`：`shm_ready.py`将URDF-only准备前缀与碰撞检查的正式H1合成一条C3参考。
- `013871b`：起点停稳速度默认统一0.2rad/s。
- 实机同一boot窗口两次±1°、两次±10°；最终`site_v6_10deg`获用户确认完整动作流程正常。详见[现场记录](shm_field_observations_20261114.md#h1脚本准备位与id12往返)。

### 2026-11-14 少量静态重力与真实接近位移诊断

`0e7316f`完成四目标/八窗配置与实机301.411秒采集，详见[静态重力报告](shm_static_gravity_20261114.md)。
