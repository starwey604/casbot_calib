# 构建、交付与测试瘦身记录（2026-11-15）

实施工作区：`casbot_calib-cross`，分支 `orin-cross-build`。规格：[build_test_slimming_plan.md](../build_test_slimming_plan.md)。

## 已落地的结构

- 原生、ARM64 wheel 统一排除 `assets/models/`、`assets/fieldwork_display/`，保留 assets 根目录全部 8 个 JSON/XML。模型、显示资产和 SHM 默认 URDF 统一使用包内目录优先、项目目录回退的解析规则。
- `deploy_git.sh` 和隔离安装共用 `scripts/shm/runtime_assets.py` 的 8 文件清单：裸法兰模型 5 文件、gauge-cover 3 文件。无头部署不传显示 STL；显示网格校验移到 `viewer_adapter()`。
- 开发安装树 `build/orin-cross-dev-install` 保留库、头文件和 CMake 导出；`build/orin-cross-tests` 放测试与独立 libomp；每次 runtime 打包都使用新的临时 staging，动态扫描运行工具。预编译更新重建生成的 `install/shm-core`，清除旧测试程序、开发库与头文件；项目配置保留。
- 删除 `shm_joint_test`、旧单自由度 simulator、arm_guard、trical_mock、Python 仿真绑定、static_field 入口、两套过时模型及旧顶层配置。H1 选点移入 `h1_selection.py`，共用 `_move()` 移入 `staged_motion.py`。
- 保留 C++ `arm_session.cpp`、TriCal、`ArmDynamics.point_geometry_batch`、`trical_assembly` 和当前两套碰撞资产。
- 原生 `calib/setup.sh`、`scripts/shm/build.sh` 与 Humble preset 继续使用；预编译更新走短检查，完整数值检查仍用 `calib/setup.sh --check`。

## 测试覆盖迁移

| 原覆盖 | 当前归属 |
| --- | --- |
| `test_arm_dynamics.py`、`test_arm_pinocchio.py` 的保留接口断言 | `test_dual_arm.py`，原测试函数全部迁入；无重名覆盖 |
| 逐轴失能、待处理命令、失能后物理演化、重新使能 | `cpp/tests/session_tests.cpp` |
| 电流解析响应、因果延迟、能量与半步长收敛、随机可重复性 | `cpp/tests/session_tests.cpp` |
| 1 ms / 250 μs / 125 μs 采样混叠与摩擦辨识误差 | `cpp/tests/session_tests.cpp`，保留原误差阈值 |
| 三阶段物理闭环、阶段衔接、准备位单次使能、8 窗成对采样 | 原生 `slow` 集 |
| 完成会话的 resume、安装检查、CLI 无效参数、全 14 轴滞回拟合 | 流水线/数值集成 `slow` 集 |
| H1/H2/H3 小规划与合成分析 | 默认快速集；H1 单关节与 smoke 配方另放 `slow` |
| 全源网格、full 规划、CPU 基准 | `slow` 集 |

C++ 测试合并为 `core_unit_tests` 和 `shm_unit_tests`，保留独立辅助程序与物理 CLI 验收。

快速分析夹具使用较短 H1 运动及 5 ms 合成采样；H2 快速集用独立解析力矩检验 Stribeck 参数恢复和验证集误差，完整合成采集、偏置诊断与导出重载进入 `slow`，保留两组训练姿态、一组独立验证姿态、4 个训练速度、3 个验证速度及 1 ms 采样。H3 快速集保留小 CMA 规划与谐波回归核验，完整刚体/转子联合辨识进入 `slow`。旧 9 阶段全轴配方放入 `slow`，Motion 解析测试仍在快速集。观察时间夹具保留原 4 秒时轴和时钟跳变，降低重复行数量。

60 万帧读取基准曾连续超过原 1.2 秒阈值。`ObservationReader` 改为只读 mmap 读取记录、原位缩放三个输出数组，避免复制整份命令日志和多份浮点中间量；返回值仍为独立 ndarray。二进制/CSV、真实私有 Mock 采集逐项对照和原读取/摘要耗时阈值通过。

## 验证与耗时

环境：6 个可用 CPU、约 11.3 GiB 内存；Ubuntu 22.04/Humble 容器；LLVM 22.1.8；Pinocchio 4.0.0、Coal 3.0.3。时间均为本机墙钟时间。

| 检查 | 结果 | 证据（`artifacts/build-slimming/`） |
| --- | --- | --- |
| 原生默认快速集 | 237 passed、70 deselected，11.50 s | `fast-layered.log` / `.xml` |
| 原生完整 pytest | 306 passed，251.89 s | `native-accepted.log`、`native-accepted/pytest.xml` |
| 原生 CTest | 5/5 passed，16.80 s | `native-accepted/ctest.xml` |
| 非 editable 原生 wheel | 706,282 B（0.674 MiB），安装校验通过 | `native-accepted/wheel/verification.json` |

全量运行后新增一项快速 H2 解析拟合测试，并恢复完整 H2 集成夹具：两项定向测试通过（3.98 s，`h2-layered.log`），随后该轮 307 项中的 237 项快速集通过，其余 70 项为 slow。

### 产物与交叉构建

- ARM64 wheel：**1,081,850 B（1.032 MiB）**。
- ARM64 runtime：**1,046,127 B（0.998 MiB）**；共 6 个运行程序、libomp 与 2 份许可文件。
- 开发树约 5.9 MiB，测试树约 2.2 MiB，分别用于 consumer 和 QEMU C++ 测试。
- 热缓存一键构建及完整冒烟 **25.657 s**：configure 1.279 s、C++ build 0.243 s、install 0.087 s、wheel 4.254 s、runtime pack 0.311 s、验证入口 18.549 s（其中冒烟 15.649 s）。证据：`dist/orin-aarch64/manifest.json`、`artifacts/build-slimming/cross-accepted.log`。
- 此次热构建总时间高于原 22.54 s 基线；当前完整冒烟还执行了从 Python 下沉的物理积分、能量、采样等 C++ 断言。该轮留下的热路径优化见下一节。

### 热路径优化：并行独立采样仿真

为冒烟增加各子进程计时，并在 `core_unit_tests` 中打印 `TEST_TIMING`。同一轮剖析定位到：

- `core_unit_tests` 16.262 s，其中 session 测试 14.156 s；三个采样周期串行耗时分别为 6.083 / 3.066 / 3.860 s，合计 **13.009 s**。
- 四组数学进程各约 8.7–9.3 s，最长执行链在新增的物理采样测试。
- 原 Step 4 冒烟 13.288 s；本轮调整前复测 16.340 s。wheel 打包复测为 2.107 s，低于旧基线 2.839 s；上轮的 4.254 s 未复现。

将 1 ms、250 μs、125 μs 三个独立采样周期通过 `std::async` 并行执行。每组单独创建 plant、workspace、随机状态和输出缓冲；主线程汇总拟合系数后执行原误差断言。保留每周期三个速度、每次 2 秒仿真、原积分/采样步长和全部收敛阈值。生产代码、原生入口与打包流程沿用原实现。

6 个可用 CPU 下，依赖和构建缓存已准备，顺序执行三次无源码变更的一键构建：

| 墙钟时间（s） | 调整前复测 | 优化后 1 | 优化后 2 | 优化后 3 |
| --- | ---: | ---: | ---: | ---: |
| wheel 构建/打包 | 2.107 | 2.839 | 2.573 | 2.394 |
| C++ 核心测试子进程 | 15.256 | 10.212 | 11.772 | 10.722 |
| 完整数值/C++ 冒烟 | 16.340 | 12.871 | 12.880 | 12.394 |
| 验证入口（安装 + ELF + QEMU） | 19.423 | 15.792 | 15.834 | 15.491 |
| **一键总耗时** | **23.680** | **20.970** | **20.625** | **20.148** |

优化后三轮总耗时中位数 **20.625 s**。用户提示历史测试可能存在接电/电池模式差异；供电状态未记录，历史 22.541 s 基线保留作参考。本轮聚焦已测出的串行热点。

`smoke.json` 新增 `timings_seconds`，记录四组数学进程、C++ 核心/协议测试、ABI、OpenMP、父进程导入、CLI 与结果比较耗时。并行任务的计时区间会重叠。

本轮验收：原生重建与 CTest **5/5** 通过（16.39 s），构建编排测试 **29 项**通过（1.57 s）；ARM64/QEMU **205 项**通过（pytest 53.88 s，整条命令 **60.858 s**），四组完整 32 行数学对照、碰撞反例及私有 libomp 均通过。整条非实时回归本轮高于计划的 60 s 目标 0.858 s；保留此次数据，优化收敛在已定位的串行采样热点。

证据：`artifacts/hot-build/native-check.log`、`qemu-pytest.log`、`before.{log,json}`、`profile.log`、`parallel-first.log` 和 `after-{1,2,3}.{log,json}`。`parallel-first.log` 含改动后的重新编译，三轮热缓存统计从随后无源码变更的运行开始。

### ARM64/QEMU 快速回归

瘦身初验：**205 项通过，pytest 50.51 s，整条 `verify_orin_qemu.sh --pytest` 56.786 s**，达到 60 s 门槛。完整数学对照与 C++ 测试同轮通过。证据：`artifacts/build-slimming/qemu-a78ae.log`、`artifacts/orin-qemu/20261115T143119.166581Z/`。

上述回归使用已准备的镜像、依赖及上一轮 JUnit 耗时。本机 6 个可用 CPU 下，6 worker 的整体验证约 70 s；4 worker 配合并发冒烟约 68 s。将 QEMU CPU 从默认模型设为交叉编译目标 `cortex-a78ae` 后降至 56.786 s。所有 205 项、四组 32 行数学对照及物理 C++ 断言保持执行；上一轮 JUnit 耗时仅用于安排用例顺序。

### ARM64 隔离首次安装与更新

断网容器中仅放 8 个无头资产文件，运行首次安装、更新、各自的完整 `--check`、H1 离线规划与 OpenMP 探针。更新前额外放入旧测试程序、静态库、头文件和 CMake 导出；更新后全部清除，操作员配置保留。

| 阶段 | 首次安装（s） | 更新安装（s） |
| --- | ---: | ---: |
| 总时间 | 61.399 | **8.826** |
| link_math | 1.535 | 1.541 |
| pip（项目 wheel） | 2.800 | 2.960 |
| tar | 0.147 | 0.150 |
| shm_abi_check | 0.066 | 0.068 |
| 绑定及资产短检查 | 1.596 | 1.675 |
| 后续完整 `--check` | 8.651 | 7.434 |

首次安装总时间包括创建 venv 和离线安装数值依赖。另测实际安装路径检查 0.827 s、H1 规划 11.932 s、OpenMP 探针 0.552 s。直接导入和真实 `dynamics_h1.sh` 子进程均加载安装包内的 Python 模块和 `_core`。

证据：`artifacts/orin-install/20261115T140958Z/`、`artifacts/build-slimming/install-accepted.log`。

已完成的独立检查：

- 纯 SHM、`CALIB_BUILD_CORE=OFF` 构建与 2 项 CTest 通过（`protocol.log`）。
- 原生及 ARM64 开发安装树的 C++ consumer 编译、运行通过（`native-pack.log`、`cross-consumer.log`、`arm-consumer.txt`）。
- wheel 根目录 8 个小配置齐全；大资产目录已剥离；runtime 不含测试、静态库、头文件和 CMake 文件。
- ARM64 四组独立进程覆盖 1/4 线程及两种导入顺序；每组完整 32 行、14 轴、502 碰撞对。RNEA 对 Pinocchio 最大误差 `4.44e-15`，碰撞对 Coal、单/多线程结果误差均为 `0`；碰撞反例回归通过。
- OpenMP 探针报告实际 1/4 线程，wheel 与 C++ 测试分别加载自身的 `libomp.so.5`。

## 使用入口

```bash
# 原生快速集
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q < /dev/null
# 原生完整验收，包括 consumer 和非 editable wheel
bash scripts/humble/container.sh bash scripts/humble/validate.sh < /dev/null
# ARM64 产物与完整数值冒烟
bash scripts/shm/cross_build_orin.sh
# ARM64 非实时快速回归；完整非实时集追加 -o addopts=''
bash scripts/shm/verify_orin_qemu.sh --pytest
# 断网隔离项目的首次安装、更新、完整 --check 与 H1 规划
python3 scripts/cross/verify_install.py
```

QEMU pytest 使用最多 4 个 worker、每个 worker 单线程 OpenMP，与数值/C++ 验证并行。运行镜像通过 `QEMU_CPU=cortex-a78ae` 使用与编译目标一致的 CPU，空闲 OpenMP 线程采用被动等待；独立数学冒烟保留 1/4 线程。未连接实机。
