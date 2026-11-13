# SHM 原生构建与 Orin 源码部署

本轮先交付可独立构建、安装和消费的 `casbot_calib::shm` 协议库，以及离线诊断工具 `shm_abi_check`。工具检查 35 槽布局并输出 JSON，不打开共享内存。`shm_executor`、物理 `shm_mock_server` 和 IPC 循环留下一增量；没有用同名空程序替代它们。

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

要提前验证物理 Mock 将依赖的现有动力学核心：

```bash
bash scripts/shm/build.sh --with-core \
  --build-dir build/shm-core --install-prefix "$PWD/install/shm-core" \
  --dependency-prefix /opt/ros/humble
```

此组合包含现有核心、模拟设备测试及 SHM 协议测试，不是完整 SHM Mock。算法库前缀显式选择，不 source ROS 或继承厂家运控 `LD_LIBRARY_PATH`。算法包自身可能通过 CMake 查找其 Python 开发依赖；这与本工程 Python 绑定关闭是两回事。

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
- 本轮尚不包含队列同步、定点转换、时钟映射和双进程物理联跑；后续顺序见[移植计划](shm_migration_plan.md)。
