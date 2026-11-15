# Orin (`aarch64`) LLVM + ThinLTO 全流程交叉编译与 QEMU 验证实施计划

本文档面向在本工作树（分支 `orin-cross-build`）执行实施的 Agent。开始前请先阅读根目录 `AGENTS.md`，检查 `git status`，**不得破坏现有 x86_64 Humble 原生开发流与 Orin 原生回退构建链**，也**不得连接真实机器人或 `/shm_mc`**。

---

## 1. 目标与交付物

将原先每次部署需在 Jetson AGX Orin 上受限于内存（`jobs=1..2`）原生编译数分钟的 C++/Python 流程，升级为 **本机 x86_64 容器内原生运行 LLVM (`clang++` + `ld.lld` + `-flto=thin` + `-mcpu=cortex-a78ae`) 秒级交叉编译 + QEMU `linux/arm64` 自动验证 + Orin 2 秒预编译安装**。

### 核心交付标准
1. **单镜像双阶段基础设施（`scripts/cross/Dockerfile.orin`）**：
   - Stage 1（`--platform=linux/arm64`，目标名 `orin-arm64-runtime`）：基于 `ros:humble-ros-base-jammy`，安装与实机完全同源的 `arm64` 运行/开发包（Pinocchio 4.0、Coal 3.0、Boost 1.74、Eigen 3.4、Python 3.10、`libgomp1`、`libgcc-11-dev`）。**既作为交叉编译的只读 sysroot 来源，也直接作为 QEMU `aarch64` 运行验证容器**。
   - Stage 2（`--platform=linux/amd64`，目标名 `orin-x86-cross-builder`）：基于 `ubuntu:22.04`，安装 x86_64 原生 `clang`、`lld`、`llvm`（`llvm-ar`/`llvm-ranlib`）、`cmake`、`ninja-build`、`ccache`、`python3.10-venv`、`crossenv`，并合入 `/sysroot/arm64`。
2. **CMake 工具链与 Preset（`cmake/toolchains/orin-aarch64-llvm.cmake` + `CMakePresets.json`）**：
   - 新增 `orin-cross` configure/build preset，启用 `-target aarch64-linux-gnu`、`--sysroot=/sysroot/arm64`、`-mcpu=cortex-a78ae`、`-O3 -flto=thin -fuse-ld=lld`、`-fopenmp=libgomp`。
   - 保持 `CMakeLists.txt` 对原生 GCC/Clang 完全向后兼容。
3. **一键交叉构建产出（`scripts/shm/cross_build_orin.sh`）**：
   - 在 x86_64 `orin-x86-cross-builder` 容器内利用 `ccache` + 多核 Ninja 一次性产出到 `dist/orin-aarch64/`（目录入 `.gitignore`）：
     - `casbot_calib-0.1.0-cp310-cp310-linux_aarch64.whl`（含 `_core.cpython-310-aarch64-linux-gnu.so`）
     - `shm-core-aarch64.tar.gz`（打包完整的 `install/shm-core`：`bin/shm_executor`、`shm_mock_server`、`shm_gui_owner`、`shm_joint_test`、`shm_reference_check`、`shm_abi_check`、`shm_channel_start` 及 `lib/`、`include/`）
     - `manifest.json`（记录源码 `git rev-parse HEAD`、编译器版本、编译参数与产物 SHA256）
   - 热 `ccache` 全量打包目标耗时 **< 10s**，单文件增量编译 **< 3s**。
4. **QEMU `linux/arm64` 验证入口（`scripts/shm/verify_orin_qemu.sh`）**：
   - **默认快速门禁（`--smoke`，< 8s，每次 `cross_build_orin.sh` 自动调用）**：
     - 静态 ELF 审计：校验 `EM_AARCH64`、无 `/sysroot` 路径泄漏、`DT_RUNPATH` 精确指向 `/opt/ros/humble/lib/aarch64-linux-gnu`、清空 `LD_LIBRARY_PATH` 下 `ldd` 无 `not found`、无 `libomp.so` 依赖（仅用 `libgomp.so.1`）。
     - 在 `orin-arm64-runtime`（`linux/arm64` QEMU）容器内运行：`shm_abi_check`（744/696 字节对齐验证）、纯计算 C++ CTest（`integration_tests`、`core_tests`、`arm_core_tests`、`kinematics_tests`、`simulated_device_tests`、`shm_wire_tests`、`shm_limits_tests`）、`.venv-shm` 安装交叉编译 `.whl` + `link_math_bindings.py` + `casbot_calib._core` OpenMP RNEA/碰撞批处理数值一致性校验 + `python -m casbot_calib.pipeline.preflight --offline`。
   - **扩展回归模式（`--pytest`）**：在 `orin-arm64-runtime` 容器内运行非实时 Python `pytest` 子集（自动跳过 wall-clock 微基准与 1 kHz 周期抖动敏感用例）。
5. **Orin 秒级预编译部署与安装集成（`calib/setup.sh` + `scripts/shm/deploy_git.sh`）**：
   - `scripts/shm/deploy_git.sh` 新增可选 `--with-prebuilt`（默认在存在与当前分支 commit 匹配的 `dist/orin-aarch64/manifest.json` 时也可显式开启），将 `dist/orin-aarch64/` 同步到 Orin 的 `artifacts/prebuilt_aarch64/`。
   - `calib/setup.sh` 新增 `--prebuilt [DIR]`：直接用离线 wheelhouse + `casbot_calib-*.whl` 安装进 `.venv-shm`，解压 `shm-core-aarch64.tar.gz` 到 `install/shm-core`，链接 `/opt/ros/humble` 数学绑定并运行 `preflight --offline`，全程跳过 Orin 本机 CMake/C++ 编译。不带 `--prebuilt` 时保留原汁原味的 Orin 原生编译能力。

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
- **硬性运行约束**：现场启动 `shm_executor` 与 Python 进程时**会主动清空 `LD_LIBRARY_PATH` 和 `PYTHONPATH`**（`env -u PYTHONPATH -u LD_LIBRARY_PATH`），因此所有交叉编译产物的 `DT_RUNPATH` 必须直接解析到 `/opt/ros/humble/lib/aarch64-linux-gnu`。

---

## 3. 五个关键技术坑点与确定性解法（实施时严格照此处理）

### 坑点 1：严禁在 `--platform=linux/arm64` 里通过 QEMU 跑编译器
- **原因**：QEMU TCG 翻译跑 `clang++` 模板展开比 Orin 本机还慢 5～8 倍。
- **解法**：`Dockerfile.orin` 第二阶段固定 `FROM --platform=linux/amd64 ubuntu:22.04 AS orin-x86-cross-builder`，只运行 x86_64 原生 `clang-15` + `lld-15`（Ubuntu 22.04 `jammy-updates` 官方源自带，原生支持 `-mcpu=cortex-a78ae` 与 `-flto=thin`，不依赖易被墙的第三方 apt 源；若网络畅通也可做成 `ARG LLVM_VERSION=15` 可配置）。

### 坑点 2：`pinocchioTargets-none.cmake` / `coalTargets-none.cmake` 硬编码绝对路径与 sysroot 绝对软链
- **已查明事实**：ROS Humble apt 包里的 `/opt/ros/humble/lib/aarch64-linux-gnu/cmake/pinocchio/pinocchioTargets-none.cmake` 第 66 行写死了 `_IMPORT_CHECK_FILES_FOR_pinocchio::pinocchio_pywrap_default "/opt/ros/humble/lib/python3.10/..."`（未加 `${_IMPORT_PREFIX}`），且 `/sysroot/arm64/usr/lib/aarch64-linux-gnu/*.so` 存在指向 `/lib/aarch64-linux-gnu/...` 的绝对软链。
- **确定性解法**（在 `Dockerfile.orin` 的 `orin-x86-cross-builder` 构建层内一次性处理）：
  1. 从 `orin-arm64-runtime` 拷贝 `/lib`、`/usr/lib`、`/usr/include`、`/etc/ld.so*`、`/opt/ros/humble` 到 `/sysroot/arm64/`。
  2. 用一段简短 Python 脚本把 `/sysroot/arm64` 内所有以 `/` 开头的绝对符号链接改写为等价的 `os.path.relpath` 相对符号链接，防止 `ld.lld` 逃逸到宿主根文件系统。
  3. 在 `orin-x86-cross-builder`（不含 `amd64` ROS）中建立符号链接 `ln -s /sysroot/arm64/opt/ros/humble /opt/ros/humble`。这样既让硬编码 `/opt/ros/humble/...` 的 CMake 检查直接通过，又让 `INSTALL_RPATH_USE_LINK_PATH TRUE` 自动写入 `/opt/ros/humble/lib/aarch64-linux-gnu`（而非 `/sysroot/arm64/...`）！同时在 toolchain / CMake 中显式兜底 `CMAKE_INSTALL_RPATH="/opt/ros/humble/lib/aarch64-linux-gnu;/opt/ros/humble/lib"`。

### 坑点 3：Clang OpenMP 默认链接 `libomp.so`（Orin 未装）及 CMake `FindOpenMP` 交叉探测失败
- **解法**：
  - 编译选项使用 `-fopenmp=libgomp`。
  - 在 `cmake/toolchains/orin-aarch64-llvm.cmake` 中预置：
    ```cmake
    set(OpenMP_CXX_FLAGS "-fopenmp=libgomp" CACHE STRING "")
    set(OpenMP_CXX_LIB_NAMES "gomp" CACHE STRING "")
    set(OpenMP_gomp_LIBRARY "/sysroot/arm64/usr/lib/aarch64-linux-gnu/libgomp.so.1" CACHE FILEPATH "")
    ```
  - 同时确认 `/sysroot/arm64/usr/lib/gcc/aarch64-linux-gnu/11` 加入 `-L` 或 Clang `--gcc-toolchain=/sysroot/arm64/usr` 搜索路径，以便 `ld.lld` 找到 `crtbeginS.o` / `libgcc_s.so` / `libgomp`。

### 坑点 4：ThinLTO 静态库归档器必须用 `llvm-ar` / `llvm-ranlib`
- **原因**：开启 `-flto=thin` 后，`calib_core` 和 `calib_shm`（`STATIC` 库）中的 `.o` 是 LLVM bitcode；GNU `ar`/`ranlib` 无法为其生成正确符号索引，会导致链接 `_core` 或 `shm_executor` 时报未定义符号。
- **解法**：在 `orin-aarch64-llvm.cmake` 中显式固定：
  - `CMAKE_C_COMPILER=clang-${LLVM_VER}`、`CMAKE_CXX_COMPILER=clang++-${LLVM_VER}`
  - `CMAKE_AR=/usr/bin/llvm-ar-${LLVM_VER}`、`CMAKE_RANLIB=/usr/bin/llvm-ranlib-${LLVM_VER}`、`CMAKE_NM=/usr/bin/llvm-nm-${LLVM_VER}`
  - 提供 CMake 开关 `CALIB_ORIN_THINLTO`（默认 `ON`）：开启时传入 `-flto=thin -fuse-ld=lld -Wl,--thinlto-cache-dir=${BINARY_DIR}/thinlto-cache`，兼顾跨文件内联优化与增量链接缓存速度。

### 坑点 5：`scikit-build-core` + `eigenpyConfig.cmake` 的 Python 3.10 交叉扩展构建
- **原因**：`CMakeLists.txt` 与 `eigenpyConfig.cmake` 在 configure 阶段都会调用 `find_package(Python / Python3 COMPONENTS Interpreter Development.Module ...)` 及 `Boost::python310`。
- **解法**：
  - 在 `orin-x86-cross-builder` 镜像构建阶段，用 `crossenv` 预建好交叉虚拟环境 `/opt/crossenv`（host Python `/usr/bin/python3.10` + target Python `/sysroot/arm64/usr/bin/python3.10`），并在 `build-pip` 和 `cross-pip` 中预装 `pybind11==2.13.6`、`scikit-build-core==0.11.6`、`wheel==0.45.1`、`numpy==1.26.4`。
  - 在 `orin-aarch64-llvm.cmake` 中显式设置：
    - `Python_EXECUTABLE=/opt/crossenv/cross/bin/python`、`Python_INCLUDE_DIR=/sysroot/arm64/usr/include/python3.10`
    - `Python3_EXECUTABLE=/opt/crossenv/cross/bin/python`、`Python3_INCLUDE_DIR=/sysroot/arm64/usr/include/python3.10`
    - `Python3_NumPy_INCLUDE_DIRS=/sysroot/arm64/usr/lib/python3/dist-packages/numpy/core/include`
  - 使用 `/opt/crossenv/cross/bin/pip wheel . --no-deps --no-build-isolation` 构建，可直接吐出标准 `casbot_calib-0.1.0-cp310-cp310-linux_aarch64.whl`，内部扩展名自动为 `_core.cpython-310-aarch64-linux-gnu.so`。

---

## 4. 分阶段实施步骤（建议按 Step 1 → Step 5 顺序推进并逐级验收）

### Step 1：容器与工具链底座（`scripts/cross/` + `cmake/toolchains/`）
1. 新建 `scripts/cross/Dockerfile.orin`：
   - `orin-arm64-runtime`（`linux/arm64`）：基于 `ros:humble-ros-base-jammy`，安装 `ros-humble-pinocchio`、`ros-humble-coal`、`libeigen3-dev`、`libboost-dev`、`libboost-filesystem-dev`、`libboost-serialization-dev`、`libboost-python-dev`、`python3-dev`、`python3-venv`、`python3-numpy`、`libgomp1`、`libgcc-11-dev`、`git-lfs`、`zstd`、`patchelf`、`file`。
   - `orin-x86-cross-builder`（`linux/amd64`）：基于 `ubuntu:22.04`，安装 `clang-15`、`lld-15`、`llvm-15`、`cmake`、`ninja-build`、`ccache`、`python3-dev`、`python3-venv`、`python3-pip`、`file`、`binutils-aarch64-linux-gnu`、`zstd`；拷贝 `/sysroot/arm64`，运行相对软链归一化脚本，建立 `/opt/ros/humble -> /sysroot/arm64/opt/ros/humble`，初始化 `/opt/crossenv`。
2. 新建 `scripts/cross/ensure_images.sh`：幂等构建 `casbot-calib/orin-arm64-runtime:humble`（`--platform linux/arm64`）与 `casbot-calib/orin-x86-cross-builder:humble`（`--platform linux/amd64`），支持 `--rebuild`。
3. 新建 `cmake/toolchains/orin-aarch64-llvm.cmake` 与 `CMakePresets.json` 中的 `orin-cross` preset（`binaryDir`: `${sourceDir}/build/orin-cross`，`CMAKE_INSTALL_PREFIX`: `${sourceDir}/install/shm-core-aarch64`）。
4. **Step 1 验收**：在 `orin-x86-cross-builder` 中跑 `cmake --preset orin-cross -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=ON -DCALIB_BUILD_PYTHON=OFF && cmake --build --preset orin-cross`，确认零警告产出全部 `aarch64` 可执行文件，且 `readelf -h` / `readelf -d` 符合预期。

### Step 2：Python `linux_aarch64` Wheel + 一键打包脚本（`scripts/shm/cross_build_orin.sh`）
1. 更新 `.gitignore` 忽略 `dist/`（`build/` 与 `install/` 已忽略）。
2. 编写 `scripts/shm/cross_build_orin.sh`：
   - 挂载宿主机持久化 ccache 目录（如 `build/ccache-orin-cross`）与当前仓库工作区，以当前宿主机 `$(id -u):$(id -g)` 用户在 `orin-x86-cross-builder` 中执行（避免 root 污染工作区权限）。
   - 先用 Ninja 构建并安装 C++ `Core` + `Shm` 组件（含 `shm_executor`、`shm_mock_server`、`shm_gui_owner` 等及 C++ 测试二进制）到 `build/orin-cross` / `install/shm-core-aarch64`，打包为 `dist/orin-aarch64/shm-core-aarch64.tar.gz`。
   - 再用 `/opt/crossenv/cross/bin/pip wheel . --no-deps --no-build-isolation -w dist/orin-aarch64`（复用同一工具链与 ThinLTO 标志）构建 `casbot_calib-0.1.0-cp310-cp310-linux_aarch64.whl`。
   - 写出 `dist/orin-aarch64/manifest.json`（含 `git_head`、`dirty`、编译耗时、编译器与 flags、各文件 sha256）。
   - 默认自动调用 `scripts/shm/verify_orin_qemu.sh --smoke`（可通过 `--no-verify` 跳过）。
3. **Step 2 验收**：记录冷缓存与热 `ccache` 全量重编耗时；确认 `unzip -l dist/orin-aarch64/casbot_calib-*.whl` 内含 `casbot_calib/_core.cpython-310-aarch64-linux-gnu.so`。

### Step 3：QEMU `linux/arm64` 离线验证与可选 Pytest（`scripts/shm/verify_orin_qemu.sh`）
1. 编写 `scripts/shm/verify_orin_qemu.sh`：
   - `--smoke`（默认）：
     1. 审计 `shm_executor`、`shm_mock_server`、`_core.*.so` 的 ELF 头（`AArch64`）、`DT_NEEDED`（确认链接 `libpinocchio_default.so.4.0.0`、`libcoal.so.3.0.3`、`libgomp.so.1`，不含 `libomp.so`）、`DT_RUNPATH`（不含 `/sysroot`）。
     2. 启动 `casbot-calib/orin-arm64-runtime:humble`（`--platform linux/arm64`），使用 `env -u LD_LIBRARY_PATH -u PYTHONPATH` 运行：
        - `shm_abi_check`
        - `integration_tests`、`core_tests`、`arm_core_tests`、`kinematics_tests`、`simulated_device_tests`、`shm_wire_tests`、`shm_limits_tests`
        - 在容器临时目录（或 `build/orin-qemu-venv` 缓存目录）用离线 `wheelhouse_aarch64` + 新编的 `casbot_calib-*.whl` 建好 `.venv-shm`，执行 `link_math_bindings.py`，跑一段校验脚本（验证 `casbot_calib._core` 14 轴批量 RNEA / 碰撞距离与 Python/Pinocchio 参考值误差 `< 1e-12`），最后运行 `python -m casbot_calib.pipeline.preflight --offline`。
   - `--pytest [PYTEST_ARGS...]`：
     - 在同一个 `orin-arm64-runtime` 容器的 `.venv-shm` 下运行 `pytest`，默认跳过对实时调度抖动或 x86 CPU 墙钟阈值敏感的测试（例如通过 `-k "not benchmark and not gui_storage"` 或对 4 个纯 wall-clock 微基准断言读取环境变量 `CASBOT_QEMU_EMULATED=1` 放宽耗时上限，**严禁削弱算法精度与安全门禁断言**）。
2. 新增本机轻量单元测试 `tests/test_orin_cross_scripts.py`（检查脚本 `--help`、参数校验、manifest 校验逻辑；不强制每次常规 `pytest` 都拉起 Docker 构建，保持现有 295 个单元测试秒级完成）。

### Step 4：ThinLTO 性能对照与部署链路打通（`calib/setup.sh` + `scripts/shm/deploy_git.sh` + 文档）
1. **性能与体积基准对照**：
   - 在 `orin-x86-cross-builder` + QEMU（以及后续上机时）对比 `-O3`（无 LTO）与 `-O3 -flto=thin -mcpu=cortex-a78ae` 的编译耗时、`.so`/`shm_executor` 体积。
   - 同时确认本机 x86_64 原生构建套件（`bash scripts/humble/container.sh ... pytest -q` 与 `bash scripts/shm/build.sh --with-mock`）完全不受影响。
2. **Orin 预编译安装入口**：
   - 在 `calib/setup.sh` 中增加 `--prebuilt [DIR]`（未传目录时默认探测 `$repo/artifacts/prebuilt_aarch64`）：
     - 校验 `manifest.json` 中的 `git_head` 与当前仓库 `HEAD` 一致（不一致则清晰报错提示重跑交叉构建或改用不带 `--prebuilt` 的原生 `calib/setup.sh`）。
     - 完成 `.venv-shm` 离线依赖检查/安装 → `pip install --no-deps --no-index --force-reinstall "$prebuilt_dir"/casbot_calib-*.whl` → `link_math_bindings.py` → 解压 `shm-core-aarch64.tar.gz` 到 `install/shm-core` → 执行 `shm_abi_check` 与 `preflight --offline`。
   - 在 `scripts/shm/deploy_git.sh` 中增加 `--with-prebuilt`：在同步完 Git 源码、LFS 模型资产与 `wheelhouse_aarch64` 后，校验 `dist/orin-aarch64/manifest.json` 的 `git_head` 与所部署 `--branch` 的 commit 一致，随后通过同一条复用 SSH 连接将 `dist/orin-aarch64/` 流式解压至远端 `$destination/artifacts/prebuilt_aarch64/`。
3. **文档更新**：
   - 在 `docs/shm_build.md` 中简明补充「本机 LLVM 交叉编译与秒级部署」章节（遵循 `AGENTS.md`：只写“怎么跑、结果怎么看”，简短直接，不写防御性免责声明）。

### Step 5：最终回归检查清单（完成后逐项打钩）
- [ ] `bash scripts/shm/cross_build_orin.sh` 一键完成交叉编译 + QEMU `--smoke` 验收，退出码为 `0`。
- [ ] `bash scripts/shm/verify_orin_qemu.sh --pytest` 在 `linux/arm64` QEMU 容器内跑通核心回归用例。
- [ ] 在 `orin-arm64-runtime` 容器内模拟 Orin 执行 `bash calib/setup.sh --prebuilt dist/orin-aarch64 --wheelhouse /path/to/wheelhouse_aarch64` 全程无需编译器、3 秒内完成并跑通 `bash calib/setup.sh --check`。
- [ ] 现有本机原生回归全绿：
  - `bash scripts/shm/build.sh --with-mock`（13/13 CTest passed）
  - `bash scripts/humble/container.sh bash -c 'cmake --build --preset humble && ctest --preset humble'`（5/5 CTest passed）
  - `bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q < /dev/null`（全部通过，0 failed，0 skipped）
- [ ] `git status` 干净，不提交 `build/`、`install/`、`dist/`、`artifacts/`，按聚焦行动式标题提交在 `orin-cross-build` 分支（**除非用户要求，不执行 `git push`**）。
