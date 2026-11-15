# Orin 交叉构建 Step 1（2026-11-15）

## 交付入口

在 `casbot_calib-cross` 工作树运行：

```bash
bash scripts/cross/build.sh
```

自动准备镜像、用 `orin-cross` preset 编译全部 C++/SHM 目标、安装 Core/Shm/CrossTests，
最后执行 ARM64/QEMU OpenMP 与 ABI 检查。产物在 `install/shm-core-aarch64/`，
构建和 ccache 分别在 `build/orin-cross/`、`build/ccache-orin-cross/`。

- builder：`casbot-calib/orin-x86-cross-builder:humble`，Ubuntu 22.04 amd64，Clang/LLD 22.1.8。
- runtime：`casbot-calib/orin-arm64-runtime:humble`，Jammy ARM64，Pinocchio 4.0.0、Coal 3.0.3、Boost 1.74。
- 编译：`aarch64-linux-gnu`、`cortex-a78ae`、`-O3`、ThinLTO、LLVM ar/ranlib。
- OpenMP：`-fopenmp=libomp`；`lib/libomp.so.5` 和许可文件随 Core/Shm 安装。
- crossenv：1.6.1；Python 3.10.12；目标 SOABI 为 `cpython-310-aarch64-linux-gnu`。
- LLVM/OpenMP 包固定为 `1:22.1.8~++20260613092327+e80beda6e255-1~exp1~20260613092437.81`。

## 验证结果

| 检查 | 结果 |
| --- | --- |
| 完整 C++/SHM 交叉构建 | 57 个 Ninja 构建步骤通过；7 个 SHM 工具、OpenMP 探针及全部 C++ 测试目标生成 |
| 安装树 ELF 与依赖 | 7 个 SHM 工具和探针均为 AArch64；清空 `LD_LIBRARY_PATH`、`PYTHONPATH` 后全部依赖可解析 |
| OpenMP 单线程 | `_OPENMP=202011`，`threads=1 tids=1 sum=499500` |
| OpenMP 多线程 | `_OPENMP=202011`，`threads=4 tids=4 sum=499500` |
| 私有运行库 | 探针通过 `$ORIGIN/../lib` 加载安装树中的 `libomp.so.5`；该库最高 GLIBC 符号版本为 2.34 |
| 安装 RUNPATH / 导出接口 | 安装程序保留相对 lib 路径与 ROS 路径；程序 RUNPATH、Core/Shm 导出文件均无 sysroot、OpenMP staging、构建目录路径 |
| SHM ABI | command 744 字节、status 696 字节、64 位、小端；诊断程序不打开 SHM |
| ARM64 数值/协议测试 | `integration_tests`、`core_tests`、`arm_core_tests`、`kinematics_tests`、`simulated_device_tests`、`shm_wire_tests`、`shm_limits_tests` 共 7 项通过 |
| ARM64 安装消费 | 既有 Core、Shm consumer 分别交叉编译并在 QEMU 执行通过；运行时清空 `LD_LIBRARY_PATH`、`PYTHONPATH` |
| Humble 原生回归 | 重建 `_core` 与全部 C++/SHM 目标后，14/14 CTest 通过，包含私有 SHM 和物理 Mock |
| 原生 Python 回归 | 使用本工作树重建的 `_core`，动力学/Pinocchio/碰撞和交叉脚本测试共 37 项通过，其中脚本测试 8 项 |
| 原生协议入口 | `scripts/shm/build.sh` 独立协议构建、安装及 6/6 CTest 通过；原生 Core/Shm 安装消费通过 |

额外的 7 项 ARM64 测试直接执行 `build/orin-cross` 中的二进制，使用
`LD_LIBRARY_PATH=$PWD/install/shm-core-aarch64/lib` 提供 libomp。安装树验证由
`verify_openmp.sh` 单独执行，使用安装后的 RUNPATH。

原生 Python 回归选择 `test_arm_pinocchio.py`、`test_arm_dynamics.py`、
`test_collision_local_bounds.py`、`test_orin_cross_scripts.py`，通过 `importlib` 从
`build/humble/_core*.so` 加载本次扩展后调用 pytest。

## 构建问题与处理

- sysroot 加入 `/usr/share`，提供 Eigen3 CMake 配置；绝对软链归一化为 sysroot 内的相对软链。
- ROS 包的绝对 Python 模块路径由 `/opt/ros/humble` 别名解析；builder 中 urdfdom 导出的 TinyXML 绝对路径改指 sysroot。
- crossenv 使用完整 ARM64 wheel 标签和二进制依赖包；为宿主 UID 开放读取权限。CMake 显式选择目标 NumPy 头文件，`Development.Module` 仅预设解释器和 include 路径。
- Clang 发现 `field_runtime.cpp` 引用了 `model()` 临时返回值中的关节。改为保存关节值，原生 C03/C04 私有协议用例通过。
- `arm_dynamics.cpp` 的 JointIndex 比较改用一致类型，消除有符号/无符号比较警告。
- CMake 3.22 的 FindBoost 对 `python310` 发出“缺少 header 检查映射”提示；实际已解析到 ARM64 Boost.Python，Core consumer 链接和执行通过。原生配置也有同一提示。
- BuildKit 提示 amd64 `FROM` 使用固定平台；这里固定 builder 为 x86_64，以直接执行编译器。原生 GCC 的 LTO 串行任务提示沿用现有配置。

## 证据与后续

日志位于本工作树 `artifacts/orin_cross_step1/`：

- `toolchain.log`：工具链版本、目标 Python SOABI 与 libomp 文件架构。
- `cross-build.log`：首次完整交叉构建与 QEMU OpenMP 结果。
- `cross-warning-fixes.log`：源码警告修复后的重新编译。
- `cross-final.log`：最终一键入口完整执行，退出码 0。
- `qemu-numerical.log`、`cross-consumers.log`：ARM64 数值/协议与安装消费。
- `native-final.log`、`native-python-final.log`：最终原生 CTest/Python 回归。
- `native-protocol.log`、`native-consumers.log`：原生协议入口和安装消费。

下一阶段按 [交叉构建计划](../orin_cross_build_plan.md) 接入 Python wheel 和产物打包；
完整 Python/QEMU 依赖栈验证放在 Step 3。现有 Humble 与 Orin 原生构建、同步和安装入口保持原用法。
