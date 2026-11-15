# Orin (`aarch64`) LLVM + ThinLTO 全流程交叉编译与 QEMU 验证实施计划

Step 1～4 已实现。后续交付布局和测试分层按 [瘦身计划](build_test_slimming_plan.md) 执行；瘦身结果见 [实施记录](records/build_test_slimming_20261115.md)，当前命令见 [构建指南](shm_build.md)，Step 1～4 的原始测量保留在 `docs/records/orin_cross_step*.md`。

本文档面向在本工作树（分支 `orin-cross-build`）执行实施的 Agent。开始前请先阅读根目录 `AGENTS.md`，检查 `git status`，**不得破坏现有 x86_64 Humble 原生开发流与 Orin 原生回退构建链**，也**不得连接真实机器人或 `/shm_mc`**。

---

## 1. 目标与交付物

将原先每次部署需在 Jetson AGX Orin 上受限于内存（`jobs=1..2`）原生编译数分钟的 C++/Python 流程，升级为 **Ubuntu 22.04 专用 x86_64 容器内运行 LLVM 22.1.8，交叉编译 ARM64 产物，经 QEMU 验证后在 Orin 直接安装**。编译器使用 `clang++` + `ld.lld` + ThinLTO，目标 CPU 为 `cortex-a78ae`，OpenMP 使用随产物提供的 `libomp.so.5`。

### 核心交付标准
1. **一个 Dockerfile、三个构建阶段、两个交付镜像（`scripts/cross/Dockerfile.orin`）**：
   - `orin-arm64-runtime`（`linux/arm64`）：基于 `ros:humble-ros-base-jammy`，安装与实机同源的运行/开发包（Pinocchio 4.0、Coal 3.0、Boost 1.74、Eigen 3.4、Python 3.10、`libgomp1`、`libgcc-11-dev`）。作为只读 sysroot 来源和 QEMU 验证镜像，保持系统原有 OpenMP 依赖。
   - `orin-arm64-openmp`（中间阶段，基于上一个阶段）：从 LLVM 官方 Jammy 22 仓库下载固定版本的 ARM64 `libomp5`、`libomp-22-dev` 并解包，提取运行库、`omp.h` 和许可文件。运行验证使用前一个阶段。
   - `orin-x86-cross-builder`（`linux/amd64`）：基于 `ubuntu:22.04`，安装固定版本的 `clang-22`、`lld-22`、`llvm-22`、CMake、Ninja、ccache、Python 3.10 与 crossenv。合入 `/sysroot/arm64`，将 ARM64 OpenMP 文件单独放在 `/opt/orin-openmp/`。所有编译器在 x86_64 CPU 上直接执行。
2. **CMake 工具链与 Preset（`cmake/toolchains/orin-aarch64-llvm.cmake` + `CMakePresets.json`）**：
   - 新增 `orin-cross` configure/build preset，启用 `-target aarch64-linux-gnu`、`--sysroot=/sysroot/arm64`、`-mcpu=cortex-a78ae`、`-O3 -flto=thin -fuse-ld=lld`、`-fopenmp=libomp`。
   - 保持 `CMakeLists.txt` 对原生 GCC/Clang 完全向后兼容。
3. **一键交叉构建产出（`scripts/shm/cross_build_orin.sh`）**：
   - 在 x86_64 `orin-x86-cross-builder` 容器内利用 `ccache` + 多核 Ninja 一次性产出到 `dist/orin-aarch64/`（目录入 `.gitignore`）：
     - `casbot_calib-0.1.0-cp310-cp310-linux_aarch64.whl`（含 `_core.cpython-310-aarch64-linux-gnu.so`、`casbot_calib/.libs/libomp.so.5` 与 OpenMP 许可文件）
     - `shm-core-aarch64.tar.gz`（干净 staging 仅收集运行产物：`bin/shm_executor`、`shm_mock_server`、`shm_gui_owner`、`shm_reference_check`、`shm_abi_check`、`shm_channel_start` 及 `lib/`、许可文件，其中 `lib/libomp.so.5` 与 wheel 使用同一 ARM64 包来源，并附带许可文件）
     - `manifest.json`（记录源码 commit、工作树状态、LLVM/OpenMP 包版本、编译参数、产物列表与分阶段耗时）
   - 性能目标：热 `ccache` 全量打包 **< 10s**，单文件增量编译 **< 3s**；分别记录冷构建、热构建、打包、QEMU 验证、首次安装与更新安装耗时。
4. **QEMU `linux/arm64` 验证入口（`scripts/shm/verify_orin_qemu.sh`）**：
   - **默认快速门禁（`--smoke`，每次 `cross_build_orin.sh` 自动调用，热环境目标 < 8s）**：
     - 静态 ELF 审计：校验 `EM_AARCH64`、目标库符号版本与 Jammy 兼容；按第 3 节设置安装后 RUNPATH，清空 `LD_LIBRARY_PATH` 后解析到随包的 `libomp.so.5` 和 `/opt/ros/humble` 数学库。检查安装产物的 RUNPATH 与导出链接接口中无 `/sysroot`、`/opt/orin-openmp` 或构建目录。
     - 在 `orin-arm64-runtime`（`linux/arm64` QEMU）容器内运行：`shm_abi_check`（744/696 字节对齐验证）、纯计算 C++ CTest（`core_unit_tests`、`shm_unit_tests`）、`build/orin-qemu-venv` 安装交叉编译 `.whl` + `link_math_bindings.py` + OpenMP 实际线程数检查、`casbot_calib._core` 单线程/多线程 RNEA 与碰撞批处理数值一致性校验；安装验收另跑完整 `setup.sh --check`。
   - **扩展回归模式（`--pytest`）**：在 `orin-arm64-runtime` 容器内运行非实时 Python `pytest` 子集（自动跳过 wall-clock 微基准与 1 kHz 周期抖动敏感用例）。
5. **Orin 预编译部署与安装集成（`calib/setup.sh` + `scripts/shm/deploy_git.sh`）**：
   - `scripts/shm/deploy_git.sh` 新增可选 `--with-prebuilt`，将 `dist/orin-aarch64/` 同步到 Orin 的 `artifacts/prebuilt_aarch64/`。
   - `calib/setup.sh` 新增 `--prebuilt [DIR]`：直接用离线 wheelhouse + `casbot_calib-*.whl` 安装进 `.venv-shm`，解压 `shm-core-aarch64.tar.gz` 到 `install/shm-core`，链接 `/opt/ros/humble` 数学绑定并运行绑定与运行资产短检查；完整检查用 `setup.sh --check`，全程跳过 Orin 本机 CMake/C++ 编译。不带 `--prebuilt` 时保留现有 Orin 原生编译入口。更新安装按 link_math、pip、tar、ABI、短检查分段计时。

---

## 2. 已核实的宿主与 Orin 实机事实（直接复用，无需猜测）

### 宿主环境（当前开发机 `x86_64`）
- Docker `29.9.3` + BuildKit `buildx-stable-1` 已就绪。
- 内核 `/proc/sys/fs/binfmt_misc/qemu-aarch64` 与 `/usr/bin/qemu-aarch64-static` **已注册生效**，`docker run --rm --platform linux/arm64` 可直接运行 `aarch64` 容器。
- 主工作树 `/home/ww/codings/casbot_calib/artifacts/wheelhouse_aarch64/` 已缓存全套 25 个 Python 3.10 `aarch64` 离线 `.whl`（含 `numpy==1.26.4`、`scipy==1.15.3`、`clarabel==0.11.1`、`cvxpy_base==1.7.5`、`orjson==3.13.0`、`cma==4.5.0`、`matplotlib==3.10.0`、`pybind11==2.13.6`、`scikit_build_core==0.11.6`）；缺省时也可跑 `bash scripts/shm/fetch_wheels.sh --aarch64-only` 重新拉取。

### 目标 Orin 实机基线（来自 `docs/records/shm_orin_build_20261114.md` 与 `installed_consumers.log`）
- **CPU 微架构**：Jetson AGX Orin — **Arm Cortex-A78AE**（`armv8.2-a+fp16+dotprod+crypto+rcpc`），内核 `5.15.148-rt-tegra`。
- **系统与 ABI**：Ubuntu 22.04 (`jammy`)，`glibc 2.35`，GCC `11.4.0`（`libstdc++.so.6` CXX11 ABI），`libgomp.so.1`，系统 Python `3.10.12`。
- **`/opt/ros/humble` 动态库版本**：
  - `libpinocchio_default.so.4.0.0`、`libpinocchio_parsers.so.4.0.0`（Pinocchio `4.0.0`，兼容 `find_package(pinocchio 3.9 REQUIRED)`）
  - `libcoal.so.3.0.3`（Coal `3.0.3`）
  - `liburdfdom_world.so.3.0`、`liburdfdom_model.so.3.0`、`libconsole_bridge.so.1.0`
  - 系统库：`libboost_filesystem.so.1.74.0`、`libboost_serialization.so.1.74.0`
- **硬性运行约束**：现场启动 `shm_executor` 与 Python 进程时**会主动清空 `LD_LIBRARY_PATH` 和 `PYTHONPATH`**（`env -u PYTHONPATH -u LD_LIBRARY_PATH`），安装产物通过 `$ORIGIN` 相对路径加载私有 OpenMP 库，并通过 `/opt/ros/humble/lib/aarch64-linux-gnu` 与 `/opt/ros/humble/lib` 加载数学库。

### OpenMP 验证结果（2026-11-15）

采用 LLVM 官方 Jammy 包 `22.1.8`，完整包版本为 `1:22.1.8~++20260613092327+e80beda6e255-1~exp1~20260613092437.81`。x86_64 编译器在现有 Jammy/Humble 容器中运行，ARM64 程序使用 Jammy sysroot，并通过本机 QEMU 执行。

| 检查 | 实测结果 |
| --- | --- |
| Clang 22.1.8 + `-fopenmp=libgomp` | `_OPENMP` 未定义；按项目条件编译方式构造的探针执行 1 个线程，求和 499500；ELF 依赖 `libgomp.so.1` |
| Clang 22.1.8 + `-fopenmp=libomp` | `_OPENMP=202011`；生成 `__kmpc_*` 调用；x86_64 和 ARM64/QEMU 均执行 4 个不同 Linux TID，求和 499500 |
| ARM64 + `-O3 -flto=thin -mcpu=cortex-a78ae` | 4 个线程，求和 499500；普通 `-O2` 版本的单线程对照也得到 499500 |
| 私有库加载 | 清空 `LD_LIBRARY_PATH`、`PYTHONPATH`；ARM64 可执行文件通过 `$ORIGIN/../lib`、动态加载的 `.so` 通过 `$ORIGIN/.libs` 找到随包运行库，均执行 4 个线程 |
| CMake 3.22 交叉探测与安装 | 补上 ARM64 libomp 的 `-L` 搜索路径后，`find_package(OpenMP REQUIRED)`、`_OPENMP` 编译检查、链接和安装通过；安装后的探针在 QEMU 下执行 4 个线程 |
| ARM64 `libomp.so.5` | 1,199,248 字节（约 1.14 MiB）；SONAME 为 `libomp.so.5`；直接依赖 `libc.so.6`、`ld-linux-aarch64.so.1`，最高 GLIBC 符号版本为 2.34 |
| x86_64 Python 数值库共存 | NumPy 1.26.4、SciPy、Pinocchio 4.0.0、Coal 与探针交换加载顺序，两种顺序均通过 4 线程检查、线性求解与 RNEA 有限值检查；进程加载了 libomp 和 OpenBLAS |
| x86_64 双 OpenMP runtime 探针 | GCC/libgomp 与 Clang/libomp 的 `.so` 分别先加载、libgomp 以 `RTLD_GLOBAL` 先加载，均通过各自 4 线程与求和检查；各 runtime 使用独立工作线程池 |

验证脚本、包清单和日志保存在本工作树 `artifacts/openmp_assessment_20261115/`，主要日志为 `native.log`、`qemu.log`、`qemu-shared.log`、`load-native.log`、`cmake-fixed.log`。

**采用随产物动态打包 libomp 的方案。** 库体积小，GLIBC 需求落在 Orin 的 2.35 基线上，安装只需复制文件。C++ 安装树和 wheel 各带一份相同运行库，各自可独立加载；系统 `libgomp` 继续服务已有依赖。Step 3 已验证完整 ARM64 Python 依赖栈、项目批处理、两种导入顺序和实际工作线程。

---

## 3. 工具链与运行库配置

### 1. Ubuntu 22.04 builder 与固定版本 LLVM

- builder 使用 `FROM --platform=linux/amd64 ubuntu:22.04 AS orin-x86-cross-builder`。
- 配置 `https://apt.llvm.org/jammy/` 的 `llvm-toolchain-jammy-22` 源及官方签名密钥，安装固定版本的 `clang-22`、`lld-22`、`llvm-22`。ARM64 OpenMP 包采用同一版本；包名以已验证仓库为准：运行库是 `libomp5`，开发包是 `libomp-22-dev`。
- 版本配置集中放在 Dockerfile。编译器自身使用 amd64 运行依赖；ARM64 产物使用 Jammy sysroot 的 GCC 11 头文件、libstdc++、libgcc 和 glibc。
- QEMU 用于 ARM64 包准备和产物执行。C++ 编译、模板实例化、归档、ThinLTO 和链接均在 amd64 builder 中运行。

### 2. `pinocchioTargets-none.cmake` / `coalTargets-none.cmake` 硬编码绝对路径与 sysroot 绝对软链
- **已查明事实**：ROS Humble apt 包里的 `/opt/ros/humble/lib/aarch64-linux-gnu/cmake/pinocchio/pinocchioTargets-none.cmake` 第 66 行写死了 `_IMPORT_CHECK_FILES_FOR_pinocchio::pinocchio_pywrap_default "/opt/ros/humble/lib/python3.10/..."`（未加 `${_IMPORT_PREFIX}`），且 `/sysroot/arm64/usr/lib/aarch64-linux-gnu/*.so` 存在指向 `/lib/aarch64-linux-gnu/...` 的绝对软链。
- **处理方式**（在 `Dockerfile.orin` 的 `orin-x86-cross-builder` 构建层内一次性处理）：
  1. 从 `orin-arm64-runtime` 拷贝 `/lib`、`/usr/lib`、`/usr/include`、`/usr/share`、`/usr/bin/python3.10`、`/etc/ld.so*`、`/opt/ros/humble` 到 `/sysroot/arm64/`，保留目标 Python 标准库与 sysconfig 数据。
  2. 用一段简短 Python 脚本把 `/sysroot/arm64` 内所有以 `/` 开头的绝对符号链接改写为等价的 `os.path.relpath` 相对符号链接，防止 `ld.lld` 逃逸到宿主根文件系统。
  3. 在 `orin-x86-cross-builder` 中建立符号链接 `ln -s /sysroot/arm64/opt/ros/humble /opt/ros/humble`，让 CMake 的绝对路径检查找到目标文件。交叉目标显式设置第 3 小节的安装 RUNPATH，关闭这些目标的 `INSTALL_RPATH_USE_LINK_PATH`，保持 sysroot 与 OpenMP staging 路径只用于构建。
  4. 将 builder 内 `urdfdomExport.cmake` 的 `/usr/lib/aarch64-linux-gnu/libtinyxml.so` 改为 `/sysroot/arm64/usr/lib/aarch64-linux-gnu/libtinyxml.so`，使 Ninja 的绝对文件依赖指向目标库。`/usr/share` 同时提供 Eigen3 的 CMake 配置。

### 3. Clang OpenMP 与私有 libomp

项目 `arm_bindings.cpp` 和 `collision.cpp` 使用 `_OPENMP` 控制并行代码。Clang 22 驱动为 `libomp`/`libiomp5` 启用前端 OpenMP；原计划的 `-fopenmp=libgomp` 会移除这些并行分支。交叉构建统一采用 `-fopenmp=libomp`。

**编译与链接**：

- 从 ARM64 包提取实际文件 `/usr/lib/aarch64-linux-gnu/libomp.so.5` 至 `/opt/orin-openmp/lib/libomp.so.5`；从 `libomp-22-dev` 提取 `omp.h` 至 `/opt/orin-openmp/include/`。本次包中 `/usr/lib/llvm-22/lib/libomp.so.5` 指向 `../../x86_64-linux-gnu/libomp.so.5`，因此按 multiarch 实际文件路径提取。在 staging 的 `lib/` 中创建 `libomp.so -> libomp.so.5`，供编译驱动的 `-lomp` 查找。
- toolchain 预置交叉探测信息：
  ```cmake
  set(OpenMP_CXX_FLAGS "-fopenmp=libomp -isystem /opt/orin-openmp/include" CACHE STRING "")
  set(OpenMP_CXX_LIB_NAMES "omp" CACHE STRING "")
  set(OpenMP_omp_LIBRARY "/opt/orin-openmp/lib/libomp.so.5" CACHE FILEPATH "")
  foreach(kind EXE SHARED MODULE)
    string(APPEND CMAKE_${kind}_LINKER_FLAGS_INIT " -L/opt/orin-openmp/lib")
  endforeach()
  ```
  CMake 的 OpenMP 探测链接会由 Clang 添加 `-lomp`；上述搜索路径使其选中 ARM64 库。本次省略该路径的探测选中了 amd64 libomp，并报架构冲突。
- `orin-cross` 对 OpenMP 使用 `REQUIRED COMPONENTS CXX`，并以编译探针检查 `_OPENMP`。将 OpenMP 编译选项应用到 `calib_core` 和 `_core`，链接到相同 ARM64 运行库；原生 preset 保留现有 OpenMP 探测行为。
- Clang 使用 `--gcc-toolchain=/sysroot/arm64/usr` 查找 GCC 11 的启动对象、libgcc 与 libstdc++。探针与 Python 扩展通过 `OpenMP::OpenMP_CXX` 传递编译选项及目标库完整路径；Core 公开链接 `casbot_calib::openmp_runtime`，编译选项设为 private，编译器运行依赖留在 amd64 环境。

**打包与加载**：

| 产物 | 随包运行库 | 安装后 `DT_RUNPATH` |
| --- | --- | --- |
| `install/shm-core/bin/*` 中使用 OpenMP 的程序 | `install/shm-core/lib/libomp.so.5` | `$ORIGIN/../lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib` |
| wheel 中的 `casbot_calib/_core.*.so` | `casbot_calib/.libs/libomp.so.5` | `$ORIGIN/.libs:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib` |

- 两个安装组件分别纳入运行库：`Shm` 安装至 `lib/`，`Python` 安装至 `casbot_calib/.libs/`；单独安装 `Core` 时也提供其链接所需的运行库。
- CMake 导出的 Core 链接接口用安装前缀相对路径表示 libomp 依赖，同步更新 `casbot_calibConfig.cmake.in`。原生 GCC/Clang 的 Core、Shm 消费入口保持现有行为。
- 保留 `libomp.so.5` 的 SONAME。仅打包 CPU OpenMP 运行库及许可文件；系统 `libgomp` 保持原路径，交付包省略 LLVM 开发包里的 `libgomp.so*`、`libiomp5.so` 兼容别名。
- C++ 安装树附 `share/licenses/casbot_calib/openmp/`，wheel 附 `casbot_calib/.libs/licenses/`，包含 LLVM OpenMP `LICENSE.TXT` 与所用 Debian 包的 copyright 文件。
- Python 数值库共存检查覆盖导入顺序、实际线程数和已加载运行库路径。验证环境使用 `OMP_DYNAMIC=FALSE`、`OMP_NUM_THREADS=1/4`、`OPENBLAS_NUM_THREADS=1`；现场线程设置沿用现有入口。

### 4. ThinLTO 静态库归档器必须用 `llvm-ar` / `llvm-ranlib`
- **配置依据**：开启 `-flto=thin` 后，静态库包含 LLVM bitcode。归档、符号索引与链接统一使用 LLVM 22 工具链。
- **解法**：在 `orin-aarch64-llvm.cmake` 中显式固定：
  - `CMAKE_C_COMPILER=clang-${LLVM_VER}`、`CMAKE_CXX_COMPILER=clang++-${LLVM_VER}`
  - `CMAKE_AR=/usr/bin/llvm-ar-${LLVM_VER}`、`CMAKE_RANLIB=/usr/bin/llvm-ranlib-${LLVM_VER}`、`CMAKE_NM=/usr/bin/llvm-nm-${LLVM_VER}`
  - 提供 CMake 开关 `CALIB_ORIN_THINLTO`（默认 `ON`）：开启时传入 `-flto=thin -fuse-ld=lld -Wl,--thinlto-cache-dir=${BINARY_DIR}/thinlto-cache`，兼顾跨文件内联优化与增量链接缓存速度。

### 5. `scikit-build-core` + `eigenpyConfig.cmake` 的 Python 3.10 交叉扩展构建
- **原因**：`CMakeLists.txt` 与 `eigenpyConfig.cmake` 在 configure 阶段都会调用 `find_package(Python / Python3 COMPONENTS Interpreter Development.Module ...)` 及 `Boost::python310`。
- **解法**：
  - 在 `orin-x86-cross-builder` 镜像构建阶段，用 `crossenv` 预建好交叉虚拟环境 `/opt/crossenv`（build Python `/usr/bin/python3.10` + target Python `/sysroot/arm64/usr/bin/python3.10`，两端采用匹配的 Python 完整版本及配置），并在 `build-pip` 和 `cross-pip` 中预装 `pybind11==2.13.6`、`scikit-build-core==0.11.6`、`wheel==0.45.1`、`numpy==1.26.4`。
  - 在 `orin-aarch64-llvm.cmake` 中显式设置：
    - `Python_EXECUTABLE=/opt/crossenv/cross/bin/python`、`Python_INCLUDE_DIR=/sysroot/arm64/usr/include/python3.10`
    - `Python3_EXECUTABLE=/opt/crossenv/cross/bin/python`、`Python3_INCLUDE_DIR=/sysroot/arm64/usr/include/python3.10`
    - `NUMPY_INCLUDE_DIRS`、`Python3_NumPy_INCLUDE_DIR`、`Python_NumPy_INCLUDE_DIR` 指向 `/opt/crossenv/cross/lib/python3.10/site-packages/numpy/core/include`
  - crossenv 1.6.1 的平台标签包含 `manylinux_2_17_aarch64`、`manylinux2014_aarch64` 和 `linux_aarch64`，`cross-pip` 使用 `--only-binary=:all:`；配置阶段执行 amd64 Python/NumPy，编译使用 ARM64 Python/NumPy 头文件。镜像赋予 `/opt/crossenv` 全体用户读/目录遍历权限，以支持宿主 UID 执行。
  - builder 的 build/cross pip 固定为 25.2，以支持 `-C` 配置参数。crossenv 设置 `CPATH=`、`LIBRARY_PATH=`，由 Clang 的 sysroot 和 GCC toolchain 决定搜索路径；自动追加 `CPATH=/sysroot/arm64/usr/include` 会改变 `include_next` 顺序，导致 C++ `<cmath>` 找不到 `math.h`。
  - wheel 命令显式传入 ROS `CMAKE_PREFIX_PATH` 和目标 NumPy include 路径，覆盖 scikit-build-core 的初始 cache；设置 `CMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF`，保留项目显式的 ThinLTO，避免 pybind11 自动添加另一组 LTO 参数。
  - CMake 3.22 的 `Development.Module` 探测只预设解释器和 include 路径；预设 `Python3_LIBRARY` 会使其内部版本探测变为 `.` 并报缺少 Module。
  - Step 2 使用 `/opt/crossenv/cross/bin/pip wheel . --no-deps --no-build-isolation` 构建，可直接吐出标准 `casbot_calib-0.1.0-cp310-cp310-linux_aarch64.whl`，内部扩展名自动为 `_core.cpython-310-aarch64-linux-gnu.so`。

---

## 4. 分阶段实施步骤（建议按 Step 1 → Step 5 顺序推进并逐级验收）

### Step 1（已完成）：容器与工具链底座（`scripts/cross/` + `cmake/toolchains/`）
1. 新建 `scripts/cross/Dockerfile.orin`：
   - `orin-arm64-runtime`（`linux/arm64`）：基于 `ros:humble-ros-base-jammy`，安装 `ros-humble-pinocchio`、`ros-humble-coal`、`libeigen3-dev`、`libboost-dev`、`libboost-filesystem-dev`、`libboost-serialization-dev`、`libboost-python-dev`、`python3-dev`、`python3-venv`、`python3-numpy`、`libgomp1`、`libgcc-11-dev`、`libstdc++-11-dev`、`zstd`、`patchelf`、`file`、`binutils`。
   - `orin-arm64-openmp`：按第 3 节提取固定版本的 ARM64 libomp、头文件与许可文件。
   - `orin-x86-cross-builder`（`linux/amd64`）：基于 `ubuntu:22.04`，从 LLVM 官方源安装固定版本的 `clang-22`、`lld-22`、`llvm-22`、`cmake`、`ninja-build`、`ccache`、`python3-dev`、`python3-venv`、`python3-pip`、`file`、`binutils-aarch64-linux-gnu`、`zstd`；拷贝 `/sysroot/arm64`，运行相对软链归一化脚本，建立 `/opt/ros/humble -> /sysroot/arm64/opt/ros/humble`，合入 `/opt/orin-openmp`，初始化 `/opt/crossenv`。
2. 新建 `scripts/cross/ensure_images.sh`：幂等构建 `casbot-calib/orin-arm64-runtime:humble`（`--platform linux/arm64`）与 `casbot-calib/orin-x86-cross-builder:humble`（`--platform linux/amd64`），支持 `--rebuild`。
3. 新建 `cmake/toolchains/orin-aarch64-llvm.cmake` 与 `CMakePresets.json` 中的 `orin-cross` preset（`binaryDir`: `${sourceDir}/build/orin-cross`，`CMAKE_INSTALL_PREFIX`: `${sourceDir}/build/orin-cross-dev-install`）。
4. 提供 `scripts/cross/run.sh`（当前 UID、独立 ccache）、`build.sh`（构建与安装 Core/Shm/CrossTests 后自动验证）、`verify_openmp.sh`（安装树 ELF/依赖/RUNPATH、1/4 线程与 ABI）。
5. **Step 1 验收**：在 `orin-x86-cross-builder` 中跑 `cmake --preset orin-cross -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=ON -DCALIB_BUILD_PYTHON=OFF && cmake --build --preset orin-cross`，产出全部 `aarch64` 可执行文件。运行 OpenMP 编译探针与 ARM64/QEMU 4 线程探针，检查 ELF 架构、依赖和安装后 RUNPATH；记录并处理构建警告。已通过：7 个 SHM 工具与 OpenMP 探针均为 AArch64；安装树依赖解析完整；1/4 个实际线程分别得到 499500；ABI 为 744/696 字节。实现与回归证据见 [Step 1 记录](records/orin_cross_step1_20261115.md)。

### Step 2（已完成）：Python `linux_aarch64` Wheel + 一键打包脚本（`scripts/shm/cross_build_orin.sh`）
1. 产物使用已被 `.gitignore` 忽略的 `dist/`（`build/` 与 `install/` 同样已忽略）。
2. 编写 `scripts/shm/cross_build_orin.sh`：
   - 挂载宿主机持久化 ccache 目录（如 `build/ccache-orin-cross`）与当前仓库工作区，以当前宿主机 `$(id -u):$(id -g)` 用户在 `orin-x86-cross-builder` 中执行（避免 root 污染工作区权限）。
   - 先用 Ninja 构建并安装 C++ `Core` + `Shm` + `CrossTests` 组件（含 `shm_executor`、`shm_mock_server`、`shm_gui_owner` 等及 C++ 测试二进制）到 `build/orin-cross` / `install/shm-core-aarch64`，按第 3 节安装 libomp 和许可文件，再打包为 `dist/orin-aarch64/shm-core-aarch64.tar.gz`。
   - 再用 `/opt/crossenv/cross/bin/pip wheel . --no-deps --no-build-isolation -w dist/orin-aarch64`（复用同一工具链与 ThinLTO 标志）构建 `casbot_calib-0.1.0-cp310-cp310-linux_aarch64.whl`。
   - 写出 `dist/orin-aarch64/manifest.json`（含 `git_head`、`dirty`、LLVM/OpenMP 完整包版本、flags、产物列表与分阶段耗时）。
   - 默认自动调用 `scripts/shm/verify_orin_qemu.sh --smoke`（可通过 `--no-verify` 跳过）。
3. **Step 2 验收**：记录冷缓存与热 `ccache` 全量重编耗时；确认 wheel 包含 `_core`、`.libs/libomp.so.5` 和许可文件，tar 包包含 `lib/libomp.so.5` 和许可文件。将两类产物安装至不同临时目录，分别检查其相对加载路径。

已通过独立安装、ELF/标签/RUNPATH/许可检查和 ARM64/QEMU 1/4 线程验证。wheel 的 32 行批处理与逐行参考最大误差为 0；冷缓存全量构建与打包 92.457 秒，热缓存全量重编与打包 15.385 秒；热运行 44 次编译全部命中 ccache。热构建仍高于 10 秒目标，wheel 构建/压缩占 12.206 秒；增量轮中同阶段为 11.870 秒。完整数据与命令见 [Step 2 记录](records/orin_cross_step2_20261115.md)。

`verify_orin_qemu.sh --smoke` 现已扩展为下列完整数值/Python 检查。

### Step 3（已完成）：QEMU `linux/arm64` 离线验证与可选 Pytest（`scripts/shm/verify_orin_qemu.sh`）
1. 扩展 `scripts/shm/verify_orin_qemu.sh`：
   - `--smoke`（默认）：
     1. 审计 `shm_executor`、`shm_mock_server`、`_core.*.so` 的 ELF 头（`AArch64`）、实际依赖闭包中的 Pinocchio/Coal 版本，以及 OpenMP 使用目标对 `libomp.so.5` 的依赖；检查随包 libomp 的架构、SONAME、GLIBC 需求与第 3 节的 RUNPATH。
     2. 启动保持系统基线的 `casbot-calib/orin-arm64-runtime:humble`（`--platform linux/arm64`），安装待测产物并使用 `env -u LD_LIBRARY_PATH -u PYTHONPATH` 运行：
        - `shm_abi_check`
        - `core_unit_tests`、`shm_unit_tests`
        - 用与项目相同 OpenMP 配置构建的测试目标执行并行区域，`OMP_NUM_THREADS=4` 时记录 4 个不同线程 TID，并校验 reduction 结果；记录动态加载到的 libomp 文件路径。
        - 在 `build/orin-qemu-venv` 中用离线 `wheelhouse_aarch64` + 新编的 `casbot_calib-*.whl` 建好 ARM64 数值环境，执行 `link_math_bindings.py`，分别用独立进程设置 `OMP_NUM_THREADS=1` 和 `4`，批量至少 32 行，覆盖动力学的 16 行并行阈值及碰撞的 8 行阈值。比较两组 RNEA / 碰撞结果，并与 Python Pinocchio/Coal 参考值比较，误差 `< 1e-12`。交换 `_core` 与 NumPy/SciPy/Pinocchio/Coal 的导入顺序，检查线程执行与运行库加载路径；完整 preflight 由安装验收执行。
   - `--pytest [PYTEST_ARGS...]`：
     - 在同一个 `orin-arm64-runtime` 容器的 `build/orin-qemu-venv` 下运行 `pytest`。按用例与 fixture 排除实时 SHM/物理 Mock 调度、6 个原生墙钟阈值用例、宿主 Docker 编排及 GUI 绘制测试；保留离线运动门禁、求解器、模型、记录解析与非实时故障检查。逐项排除原因写入 `pytest-selection.json`，原生与 QEMU 均用 slow 分层，完整原生回归为 `pytest -o addopts='' -q`。
2. 扩展 Step 1 已加入的轻量单元测试 `tests/test_orin_cross_scripts.py`（检查脚本 `--help`、参数校验、产物布局与版本记录；不强制每次常规 `pytest` 都拉起 Docker 构建，保持现有常规测试入口）。

Step 3 验收结果：完整 Python 数值依赖栈、7 个 C++ 数值/协议测试、ABI 744/696、
两份私有 libomp、1/4 线程及两种导入顺序、离线 preflight 全部通过。32 行、14 轴、
502 个碰撞对的 RNEA 参考误差为 4.44e-15，碰撞参考误差为 0，单/多线程误差均为 0。
碰撞查询改为独立求解器，保留 Coal 默认收敛设置，消除跨行/线程的求解状态差异；
安装 wheel 通过共享项目根路径配置读取仓库配置与脚本。

非实时 pytest 为 **241 passed、81 deselected**，耗时 2155.79 秒；排除项为
50 个实时 SHM/物理 Mock、24 个宿主编排、6 个墙钟阈值和 1 个宿主 GUI 绘制测试。
原生 CTest 14 项、相关 Python 回归 21 项及宿主单元测试 26 项均已通过。
热环境单独冒烟 36.613 秒，仍高于 8 秒目标；性能优化留在 Step 4。
完整证据与修复过程见 [Step 3 记录](records/orin_cross_step3_20261115.md)。

### Step 4（功能已完成，耗时见下）：ThinLTO 性能对照与部署链路打通（`calib/setup.sh` + `scripts/shm/deploy_git.sh` + 文档）
1. **性能与体积基准对照**：
   - 在 `orin-x86-cross-builder` 中对比相同 `-mcpu=cortex-a78ae` 下 `-O3` 与 `-O3 -flto=thin` 的编译耗时、`.so`/`shm_executor` 体积，单列 libomp 的打包体积。QEMU 执行功能与数值回归；Orin 运行性能在用户安排的后续上机阶段测量。
   - 同时确认本机 x86_64 原生构建套件（`bash scripts/humble/container.sh ... pytest -q` 与 `bash scripts/shm/build.sh --with-mock`）完全不受影响。
2. **Orin 预编译安装入口**：
   - 在 `calib/setup.sh` 中增加 `--prebuilt [DIR]`（未传目录时默认探测 `$repo/artifacts/prebuilt_aarch64`）：
     - 完成 `.venv-shm` 离线依赖检查/安装 → `pip install --no-deps --no-index --force-reinstall "$prebuilt_dir"/casbot_calib-*.whl` → `link_math_bindings.py` → 解压 `shm-core-aarch64.tar.gz`（含私有 libomp）到 `install/shm-core` → 清空 `LD_LIBRARY_PATH`/`PYTHONPATH` 执行 `shm_abi_check` 与 `preflight --offline`。
   - 在 `scripts/shm/deploy_git.sh` 中增加 `--with-prebuilt`：与 Git 源码、LFS 模型资产和 `wheelhouse_aarch64` 一起，通过同一条复用 SSH 连接将 `dist/orin-aarch64/` 流式解压至远端 `$destination/artifacts/prebuilt_aarch64/`。
3. **文档更新**：
   - 在 `docs/shm_build.md` 中简明补充「本机 LLVM 交叉编译与预编译部署」章节，说明构建、验证、安装命令及耗时输出。

Step 4 实测与选择：

- 同配置、独立空缓存、jobs=2 顺序测试：O3 冷/热全量构建打包 70.722 / 6.404 秒，
  ThinLTO 为 86.441 / 6.286 秒。继续默认 ThinLTO：`_core` 小 24.6%，`shm_executor` 小 62.1%。
- 交叉 wheel 的 860 个模型资产改为 ZIP stored，wheel 从约 95 MB 增至 305 MB；
  原生 wheel 设置不变。热缓存全量构建打包已达到 10 秒目标。
- QEMU 四组线程/导入顺序独立进程并行，与父进程数值依赖检查和 C++ 测试重叠；
  完整 Python Coal 参考每轮计算一次，父进程将四组结果与参考比较。
  运行冒烟从 Step 3 的 36.613 秒降至 13.288 秒，含装包与 ELF 审计为 17.539 秒；
  最终无源码变化的一键构建、打包与验证合计 22.541 秒。
- 断网 ARM64/QEMU 首次安装 74.163 秒、同轮更新 24.068 秒（同时有原生构建）；
  无其它构建并发时更新为 20.002 秒。首次/更新后的 `--check`、H1 离线规划、
  私有 libomp 与安装后的 Core 消费均通过。
- 原生完整 pytest 为 326 passed、0 failed、0 skipped；SHM 构建与 CTest 14/14 通过。
- 尚待优化的耗时目标：QEMU 冒烟 < 8 秒、更新安装 2～3 秒；单文件增量 < 3 秒尚未单独测量。
  本轮未连接实机。完整命令、尺寸与日志索引见 [Step 4 记录](records/orin_cross_step4_20261115.md)。

### Step 5：最终回归检查清单（完成后逐项打钩）
- [x] `bash scripts/shm/cross_build_orin.sh` 一键完成交叉编译 + QEMU `--smoke` 验收，退出码为 `0`。
- [x] `bash scripts/shm/verify_orin_qemu.sh --pytest` 在 `linux/arm64` QEMU 容器内跑通核心回归用例。
- [x] 在 `orin-arm64-runtime` 容器内模拟 Orin 执行 `bash calib/setup.sh --prebuilt dist/orin-aarch64 --wheelhouse /path/to/wheelhouse_aarch64` 完成首次离线安装及依赖已就绪的更新安装，分别记录耗时（更新安装目标 2～3 秒），并跑通 `bash calib/setup.sh --check`。
- [x] 随包 libomp 在 C++ 安装树与 wheel 中分别正确加载；ARM64 实际线程数、导入顺序、单/多线程数值一致性与 Core 安装消费通过。
- [x] 现有本机原生回归全绿：
  - Humble 容器内 `bash scripts/shm/build.sh --with-mock --jobs 2`（14/14 CTest passed，Step 4）。
  - Humble preset 构建与 CTest（14/14 passed，Step 3）。
  - Humble 容器内完整 pytest（326 passed，0 failed，0 skipped，Step 4）；命令与私有 Mock fixture 路径见 Step 4 记录。
- [ ] `git status` 干净，不提交 `build/`、`install/`、`dist/`、`artifacts/`，按聚焦行动式标题提交在 `orin-cross-build` 分支（**除非用户要求，不执行 `git push`**）。

## 5. 技术来源

- [LLVM 官方 Jammy 软件源](https://apt.llvm.org/)：LLVM 22 的 amd64 工具链与 ARM64 OpenMP 包。
- [Clang 22 驱动实现](https://github.com/llvm/llvm-project/blob/release/22.x/clang/lib/Driver/ToolChains/Clang.cpp)：OpenMP runtime 选择与前端参数。
- [LLVM OpenMP runtime 配置](https://openmp.llvm.org/design/Runtimes.html)：线程数、动态线程与线程池配置。
- [LLVM OpenMP 许可证](https://github.com/llvm/llvm-project/blob/release/22.x/openmp/LICENSE.TXT)：随包保留的许可文本。
- [crossenv 构建要求](https://crossenv.readthedocs.io/en/stable/quickstart.html)：构建端与目标端 Python 版本配套。
