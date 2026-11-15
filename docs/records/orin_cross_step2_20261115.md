# Orin 交叉构建 Step 2（2026-11-15）

## 一键构建与产物

```bash
bash scripts/shm/cross_build_orin.sh
```

使用 Step 1 的 Ubuntu 22.04 amd64 builder，Clang/LLD/OpenMP 22.1.8，目标为
`aarch64-linux-gnu`、Cortex-A78AE、Release + ThinLTO。原生 Humble 与 Orin 构建入口保持原用法。

输出到 `dist/orin-aarch64/`：

- `casbot_calib-0.1.0-cp310-cp310-linux_aarch64.whl`：94,959,300 字节，包含 Python 包、模型资源、ARM64 `_core`、`.libs/libomp.so.5` 与许可文件。
- `shm-core-aarch64.tar.gz`：本次增量产物 3,315,558 字节，含 7 个 SHM 工具、Core/Shm 静态库和头文件、OpenMP 探针、7 个数值/协议测试程序、`lib/libomp.so.5` 与许可文件。
- `manifest.json`：commit、dirty、LLVM/OpenMP 完整包版本、目标、编译参数、产物大小、各阶段秒数和验证结果。最终 `verification` 为 `passed`。

`--no-verify` 单独运行构建和打包；`--rebuild` 清理目标文件再全量重编并保留缓存。
`--build-root` 可指定仓库内独立构建/缓存目录。

## 冷、热与增量耗时

镜像已准备完成，Ninja 并行数 2。使用新的 `build/orin-step2-benchmark`，
冷轮开始时 CMake 构建目录、ccache 和 ThinLTO 缓存均为空。

```bash
# 冷缓存
bash scripts/shm/cross_build_orin.sh --build-root build/orin-step2-benchmark --no-verify
# 热缓存全量重编
bash scripts/shm/cross_build_orin.sh --build-root build/orin-step2-benchmark --rebuild --no-verify
# 已有目标文件的增量打包，并执行 QEMU 验证
bash scripts/shm/cross_build_orin.sh --build-root build/orin-step2-benchmark
```

| 阶段（秒） | 冷缓存全量 | 热缓存全量重编 | 增量打包 |
| --- | ---: | ---: | ---: |
| CMake configure | 1.823 | 0.995 | 0.890 |
| C++ 编译/链接 | 55.747 | 0.918 | 0.031 |
| C++ 安装 | 0.089 | 0.060 | 0.033 |
| wheel 构建/暂存/压缩 | 33.371 | 12.206 | 11.870 |
| C++ tar 打包 | 0.256 | 0.265 | 0.252 |
| 独立安装与 QEMU 验证 | — | — | 12.839 |
| 总耗时（含容器启动等） | **92.457** | **15.385** | **26.741** |

冷轮 44 次编译调用中 37 次 miss、7 次预处理命中（C++ 与 wheel 复用核心源文件）。
热轮新增的 44 次调用全部直接命中，miss 总数保持 37；Ninja 清理后重新生成了目标文件。

热构建距离计划的 10 秒目标还有 5.385 秒，主要耗时在 wheel 构建/暂存/压缩。
已有目标文件时这一阶段仍需 11.870 秒；wheel 保留了现有完整 Python 包与约 292 MiB 模型资源。
继续使用 scikit-build-core 的标准打包路径，性能对照在 Step 4 继续记录。

## 随包 libomp 验证

`verify_orin_qemu.sh --smoke` 在 builder 内将 wheel 用 pip 安装到一个临时目录，
将 C++ tar 解压到另一目录。目标 NumPy 1.26.4 取自 builder 的 ARM64 crossenv。
随后在原有 ARM64 runtime 镜像中执行，清空 `LD_LIBRARY_PATH`、`PYTHONPATH`，
设置 `OPENBLAS_NUM_THREADS=1`、`OMP_DYNAMIC=FALSE`。

- wheel 标签为 `cp310-cp310-linux_aarch64`；`_core`、两份 libomp 和 tar 中全部可执行文件均为 AArch64。
- wheel RUNPATH 为 `$ORIGIN/.libs` 加 ROS 库路径；C++ RUNPATH 为 `$ORIGIN/../lib` 加 ROS 库路径。检查 CMake 导出文件和安装 RUNPATH 均无构建/staging 路径。
- 两份 libomp 的 SONAME 为 `libomp.so.5`，两类产物均含 `LICENSE.TXT` 与 Debian copyright。
- C++ OpenMP 探针：1/4 个不同 TID，reduction 均为 499500；加载 tar 安装目录的 `lib/libomp.so.5`。
- wheel：32 行 `evaluate_batch` 分别在 1/4 线程环境执行，新增工作线程分别为 0/3 个；与逐行 `inverse` 结果最大误差为 0。`/proc/self/maps` 确认唯一 libomp 路径来自 wheel 的 `casbot_calib/.libs/libomp.so.5`。
- `shm_executor`、`shm_mock_server` 的动态依赖全部解析；ABI 检查为 744/696 字节、64 位、小端。

## 实现中处理的问题

- pip 22.0.2 不支持 `-C`：builder 的 build/cross pip 固定到 25.2。
- scikit-build-core 的初始化 cache 会覆盖 ROS prefix 和 NumPy include：wheel 构建显式传入目标路径。
- crossenv 自动添加的 `CPATH` 把 sysroot 系统 include 提前，破坏 C++ `include_next <math.h>`：初始化 crossenv 时清空 `CPATH`、`LIBRARY_PATH`，使用 Clang 的 `--sysroot` 和 `--gcc-toolchain`。
- wheel 关闭 CMake 自动 IPO，沿用项目显式 `-flto=thin`，避免 pybind11 自动 LTO 选项叠加。
- `CrossTests` 安装组件加入 7 个数值/协议测试程序，供后续完整 QEMU 回归直接消费 tar。

## 回归与证据

- 交叉脚本单元测试：17 项通过，覆盖入口参数、镜像复用、UID/参数转发、软链归一化、wheel 配置、失败阶段和 ELF/RUNPATH 检查。
- Humble 原生 configure/build/CTest：14/14 通过；本次新增的 CMake 安装规则仅在交叉构建中启用。
- 最终一键入口退出码 0；独立验证入口通过。

日志和计时快照位于 `artifacts/orin_cross_step2/`：
`cold.log/json`、`warm.log/json`、`incremental.log/json`、
`cold-ccache.txt`、`warm-ccache.txt`、`verify-first.log`、`script-tests.log`、`native.log`。

下一阶段扩展现有 `--smoke`，接入完整 ARM64 Python 依赖栈、碰撞与 RNEA 参考比较、
离线 preflight 和可选 pytest。
