# SHM 构建与 Orin 部署

原生构建安装到仓库内的 `install/shm-core`。ARM64 交叉开发安装树为 `build/orin-cross-dev-install`，测试安装树为 `build/orin-cross-tests`。

## 1. 通过 SSH 从本机同步到 Orin

直连网络配置（本机 `172.16.0.20/24`，Orin `172.16.0.10`）：

```bash
bash scripts/jetson_network.sh up <本机有线网卡名>
```

在本机仓库根目录将当前已提交分支同步到 Orin（默认 `/home/nvidia/casbot_calib_shm`）：

```bash
bash scripts/shm/deploy_git.sh --host nvidia@172.16.0.10
```

- 通过 SSH 反向隧道与本机临时只读 `git daemon` 克隆或快进更新，不经过外网或 GitHub。
- 默认同步裸法兰模型的 URDF、描述符、碰撞 NPZ，以及 TriCal gauge-cover 的碰撞 NPZ、manifest 和描述符；仅传源码可加 `--no-assets`。远端沿用 Git LFS 管理这些资产。显示网格留在本机 GUI 工作区。

## 2. Orin 一键环境初始化与编译（`calib/setup.sh`）

登录 Orin 后在 `/home/nvidia/casbot_calib_shm` 执行：

```bash
cd /home/nvidia/casbot_calib_shm
bash calib/setup.sh
```

`calib/setup.sh` 自动完成：
1. 创建/复用 `.venv-shm` 并安装数值依赖（离线时自动探测本地 `artifacts/*wheels*`，也可显式传 `--wheelhouse DIR`；依赖已齐备时不访问外网）。
2. 将 `/opt/ros/humble` 中的 `pinocchio`、`eigenpy`、`coal` 链接进 `.venv-shm`（不 `source` ROS，不污染 `LD_LIBRARY_PATH`）。
3. 编译 Python `_core` 扩展、动力学/碰撞核心、`shm_executor`、`shm_channel_start`、`shm_mock_server`、`shm_gui_owner`，运行 CTest 并安装到 `install/shm-core`。
4. 执行模型 LFS 资产与二进制完整性预检。

随时单独运行预检（不重编）：

```bash
bash calib/setup.sh --check
```

## 3. 单独构建 C++ SHM 组件（`scripts/shm/build.sh`）

仅需构建 C++ 执行器与物理 Mock（不建 Python venv）时：

```bash
# 默认使用 build/shm-core -> install/shm-core，存在 /opt/ros/humble 时自动选用
bash scripts/shm/build.sh --with-mock

# 安装后双进程物理闭环验收（私有 SHM 端点，不连接 /shm_mc）
python3 scripts/shm/verify_physics.py --bin-dir install/shm-core/bin
```

只构建纯 SHM 协议组件（不含 Pinocchio/Coal 核心）可不带 `--with-core` / `--with-mock` 运行 `bash scripts/shm/build.sh`。

## 4. 本机开发与测试（Humble 容器）

```bash
# C++ 构建与测试
bash scripts/humble/container.sh bash -c 'cmake --build --preset humble && ctest --preset humble --output-on-failure' < /dev/null

# Python 快速测试
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q < /dev/null

# 完整回归（含长物理闭环与 full 规划）
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -o addopts='' -q < /dev/null

# 准备本机 GUI 原生程序（安装至 install/shm-core）
bash scripts/shm/setup_gui.sh
```

历次部署与实机编译记录归档于 [shm_orin_build_20261114.md](records/shm_orin_build_20261114.md)。

## 5. 本机交叉构建 Orin C++/SHM（Step 1）

开发机已配置 Docker 和 `qemu-aarch64` binfmt 时，在仓库根目录运行：

```bash
bash scripts/cross/build.sh
```

首次运行创建 ARM64 Jammy/Humble runtime 和 amd64 LLVM 22 builder 镜像，随后用
`orin-cross` preset 构建 C++ 核心、SHM 执行器、物理 Mock 和测试程序。构建目录为
`build/orin-cross`，开发安装目录为 `build/orin-cross-dev-install`，测试安装目录为 `build/orin-cross-tests`；ccache 保存在
`build/ccache-orin-cross`。

脚本最后通过 QEMU 执行安装后的 OpenMP 探针和 SHM ABI 检查。成功输出包括
`threads=1`、`threads=4`、`sum=499500`，运行库路径指向安装树的 `lib/libomp.so.5`，
SHM 命令/状态大小为 744/696 字节。

```bash
# 单独复查安装后的 OpenMP
bash scripts/cross/verify_openmp.sh

# 在 builder 内执行开发命令
bash scripts/cross/run.sh cmake --build --preset orin-cross

# 修改 Dockerfile 后更新镜像（保留可复用的构建层）
bash scripts/cross/ensure_images.sh --rebuild
```

这个入口用于 C++/SHM 开发。需要同时交付 Python wheel 时，使用下一节的一键打包入口；
Orin 原生部署继续使用前面的命令。

本次验证结果见 [Step 1 记录](records/orin_cross_step1_20261115.md)。

## 6. 一键打包 ARM64 wheel 与 C++ 安装包（Step 2）

```bash
bash scripts/shm/cross_build_orin.sh
```

生成 `dist/orin-aarch64/` 下的三个文件：

- `casbot_calib-0.1.0-cp310-cp310-linux_aarch64.whl`：Python 包、assets 根目录小型 JSON/XML、ARM64 `_core`、私有 `.libs/libomp.so.5` 与许可文件。
- `shm-core-aarch64.tar.gz`：从干净 staging 动态收集运行工具 `bin/`、`lib/libomp.so.5` 和 OpenMP 许可。开发库、头文件、CMake 导出与测试程序留在独立安装树。
- `manifest.json`：源码 commit、工作树状态、LLVM/OpenMP 版本、编译参数、文件大小、分阶段耗时和验证结果。

构建脚本默认将 C++ 包解压到临时目录，将 wheel 安装到独立 ARM64 venv，
然后在 ARM64/QEMU 下清空 `LD_LIBRARY_PATH`、`PYTHONPATH` 执行验证。
成功输出包括 C++ 探针 1/4 线程、32 行批处理误差、wheel 私有运行库路径和 ABI 744/696。
完整验证使用独立 ARM64 venv，安装 `requirements-shm.txt` 的数值依赖并链接 Humble 数学绑定。

```bash
# 对已有产物重新验证
bash scripts/shm/verify_orin_qemu.sh --smoke

# 清理目标文件后全量重编，复用 ccache 和 ThinLTO 缓存
bash scripts/shm/cross_build_orin.sh --rebuild

# 单独测量构建与打包时间
bash scripts/shm/cross_build_orin.sh --no-verify
```

开发计时可用 `--build-root build/orin-benchmark` 将构建和缓存放在独立目录。
首次使用新目录测冷缓存，随后加 `--rebuild` 测热缓存全量重编；直接再次执行测增量打包。
镜像准备完成后再开始比较。`wheel_build_pack` 包含 wheel 编译、安装暂存及压缩，
`qemu_verify` 包含独立安装与 QEMU 执行，`total` 包含容器启动和所有阶段。

本次打包测量见 [Step 2 记录](records/orin_cross_step2_20261115.md)。预编译部署用法见第 8 节。

## 7. 完整 ARM64/QEMU 验证（Step 3）

```bash
# C++ 测试、完整 32 行数值对照、OpenMP 与安装包 CLI
bash scripts/shm/verify_orin_qemu.sh --smoke

# 继续执行非实时快速 pytest
bash scripts/shm/verify_orin_qemu.sh --pytest

# 用已有离线 wheelhouse；pytest 参数放在 --pytest 后
bash scripts/shm/verify_orin_qemu.sh --wheelhouse artifacts/wheelhouse_aarch64 --pytest -k collision
```

首次运行补齐 ARM64 wheelhouse 并创建 `build/orin-qemu-venv`；随后复用该环境，
重新安装待测 wheel。数值依赖沿用 `scripts/shm/requirements-shm.txt`，回归工具为 pytest 8.4.2 与 pytest-xdist 3.8.0。
`link_math_bindings.py` 把同一个 Humble prefix 的 Pinocchio、Coal、eigenpy 链接进 venv。

每轮输出到自动生成的 `artifacts/orin-qemu/<时间>/`：

- `math-*.json/npz`：32 行、14 轴、502 个碰撞对，记录 1/4 线程、两种导入顺序的结果、工作线程 TID 和实际库路径。
- `smoke.json`：依赖版本、单/多线程误差、冒烟总耗时及 `timings_seconds` 子任务墙钟耗时（并行区间会重叠）。RNEA 对照 Python Pinocchio，碰撞距离对照 Python Coal，最大绝对误差要求 `< 1e-12`，最差碰撞对索引要求一致。
- `pytest.xml`、`pytest-result.json`、`pytest-selection.json`：回归结果、实际选择的用例和平台排除原因。
- `verification.json`：整条验证命令的总耗时。

pytest 排除实时 SHM/物理 Mock 调度、带原生墙钟阈值的性能测试、amd64 构建编排和宿主 GUI 绘制测试。
保留离线运动门禁、求解器、模型/导出、记录解析、无效参考拒绝和故障处理的非实时检查。
QEMU 用 [pytest-xdist load 调度](https://pytest-xdist.readthedocs.io/en/stable/distribution.html) 分配用例，按可用 CPU 最多启动 4 个 worker，各 worker 的 OpenMP 为 1 线程。复用上一轮 JUnit 耗时配对长短用例，分散预取队列中的重计算；pytest 与数值/C++ 验证并行启动。QEMU CPU 设为交叉编译目标 `cortex-a78ae`，OpenMP 空闲线程使用被动等待。独立数学冒烟仍验证 1/4 线程。调试单进程可在 `--pytest` 后加 `-n 0`。
原生与 QEMU 默认均过滤 `slow`；全量回归加 `-o addopts=''`，QEMU 仍排除实时调度用例。

验证入口自动设置 `CASBOT_PROJECT_ROOT` 并写入 venv 的 `casbot-prebuilt` 标记。
Python 包及 `_core` 从 venv 加载，模型从项目的 `python/casbot_calib/assets/models` 加载。
真实 `dynamics_h1.sh` 子进程的导入路径记在 `cli-imports.log`。

Step 3 验证：241 项 ARM64 非实时回归通过，当时完整冒烟 36.613 秒。数值明细见
[Step 3 记录](records/orin_cross_step3_20261115.md)。

## 8. 预编译部署与安装（Step 4）

本机生成产物，再同步已提交分支和预编译包：

```bash
bash scripts/shm/cross_build_orin.sh
bash scripts/shm/deploy_git.sh --host nvidia@172.16.0.10 --branch orin-cross-build --with-prebuilt
```

在 Orin 的项目目录执行首次安装或更新，命令相同：

```bash
bash calib/setup.sh --prebuilt
bash calib/setup.sh --check
```

`--prebuilt` 默认读取 `artifacts/prebuilt_aarch64/`，也可直接指定已复制的目录：

```bash
bash calib/setup.sh --prebuilt dist/orin-aarch64 --wheelhouse artifacts/wheelhouse_aarch64
```

安装复用 `.venv-shm` 的数值依赖，替换 Python wheel，链接 Humble 数学绑定，
解压 C++ 包至 `install/shm-core`，运行 ABI、绑定导入和运行资产检查。
`SETUP_TIMING` 分别报告 link_math、pip、tar、shm_abi_check、check 秒数。
完整数值依赖、求解器与碰撞资产验证使用 `bash calib/setup.sh --check`。
`dynamics.sh`、分阶段脚本、`dev.sh` 和 `setup.sh --check` 自动使用安装的 wheel；
配置、输出与大型模型都在项目目录；wheel 保留 assets 根目录的小配置。

需要切回原生编译时运行 `bash calib/setup.sh`，原有 CMake/CTest/安装流程照常执行。

### 本机复现安装验收与构建对照

```bash
# 在断网 ARM64/QEMU 容器内验证首次安装、更新、--check 和 H1 离线规划
python3 scripts/cross/verify_install.py

# 保持 CPU 参数相同，单独构建不带 ThinLTO 的对照
bash scripts/shm/cross_build_orin.sh --thinlto off --build-root build/orin-o3 --output-dir dist/orin-o3
```

安装验收创建独立的 `build/orin-install-<时间>/` 项目，结果写入
`artifacts/orin-install/<时间>/`。构建对照可用 `--rebuild --no-verify` 测热缓存全量重编；
`--dist DIR` 可让 QEMU 验证已有对照产物。

默认使用 ThinLTO、函数/数据分段、未引用段回收（`--gc-sections`）与安全 ICF 合并；运行包剥离符号表后用 gzip -9 压缩。原生与交叉 wheel
统一排除 models/ 和 fieldwork_display/，使用相同资产解析规则。
手动安装原生 wheel 时设置 `CASBOT_PROJECT_ROOT` 为源码仓库根目录；`setup.sh --prebuilt` 自动记录项目根目录。
QEMU 四组独立进程覆盖 32 行、1/4 线程与两种导入顺序，每组均做完整 Coal 对照及碰撞反例。
完整 preflight 由安装验收的 `--check` 执行。

热路径的 C++ 核心测试按三个独立采样周期并行运行，输出 `TEST_TIMING` 定位各组耗时。

瘦身实测与迁移覆盖见 [重构记录](records/build_test_slimming_20261115.md)。
