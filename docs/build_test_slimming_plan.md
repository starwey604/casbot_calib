# 编译、打包、部署与测试激进精简计划（V2）

本文档面向在 `casbot_calib-cross` 工作区（分支 `orin-cross-build`）执行的 Agent。核心策略：**不做防御性兼容**，允许跨原生与交叉构建的破坏性重构，优先彻底清理冗余资产、过期模型/配置与遗留仿真绑定，同时严格遵守主线 `K1–K3` 运动学重构边界（见主工作区 `docs/kinematics_refactoring_plan.md`）和 `AGENTS.md` 安全红线。

---

## 实施状态（2026-11-15）

- [x] WP-1A～1C：统一轻量 wheel、三路安装树、最小运行资产、预编译安装分段计时。
- [x] WP-2A～2C：旧模型/配置/入口/仿真绑定清理，保留接口的独有断言迁移。
- [x] WP-3A～3C：fast/slow 分层、物理闭环去重、C++ 单元测试合并。
- [x] 体积门槛：原生 wheel 0.674 MiB、ARM64 wheel 1.032 MiB、runtime 0.998 MiB。
- [x] 原生快速集 237 项 / 11.50 s；完整回归、consumer、非 editable wheel 校验通过。
- [x] ARM64/QEMU 205 项、四组完整 32 行 Coal 对照和碰撞反例通过。
- [ ] QEMU 整条验证 < 60 s：瘦身初验 56.786 s；采样并行优化后的本轮复测 60.858 s，功能回归通过。
- [x] 断网首次/更新安装、完整 `--check`、真实 H1 CLI 安装路径和私有 libomp 加载通过；更新安装 8.826 s。

- [x] 热路径已定位并优化：独立采样仿真并行执行，三轮一键构建加完整冒烟为 20.148～20.970 s，中位数 20.625 s。

测试迁移、测量条件与原始证据见 [实施记录](records/build_test_slimming_20261115.md)，当前命令见 [构建指南](shm_build.md)。

---

## 0. 实测基线与精简目标

| 维度 | 当前实测 | 主要冗余来源 | 改造方向与目标 |
| --- | --- | --- | --- |
| `.whl` 体积（交叉 & 原生） | **291 MiB**（305,134,254 B，860+ 文件） | 99.5% 为 `assets/models/`（277.42 MiB）与 `assets/fieldwork_display/`（11.94 MiB）；代码 + `_core.so` + `libomp.so.5` 仅 **~2.9 MiB** | **< 4 MiB**（交叉与原生统一剥离大资产，仅保留 ~120 KB 小型 JSON/XML 配置） |
| C++ 交付包 `shm-core-aarch64.tar.gz` | **3.2 MiB** tar / **8.2 MiB** 解压 | 含 8 个 `CrossTests`（1.76 MB）、重复工具 `shm_joint_test`（377 KB）、开发态 `.a`（3.35 MB）与头文件/CMake 导出文件 | **< 1.0 MiB** tar（独立干净 staging 仅装运行时 `bin/` + `libomp.so.5` + OpenMP 许可） |
| 热缓存打包 + QEMU `--smoke` | **22.54 s**（打包 2.84 s + 冒烟 17.54 s） | `pack_assets.py` 拷贝 860 文件；`verify_package.py` 解压 305 MB wheel；冒烟内拉起全量 `preflight` | 去掉无效打包、解包大资产与冒烟重 `preflight` 后显著压缩（不削减 32 行 Coal 数学对照） |
| `--prebuilt` 更新安装 | **20.00 s**（ARM64/QEMU 下仅导入 `preflight` 1.39 s + 数值依赖 4.33 s 即 **5.71 s**） | `pip` 覆写 860 资产；更新链路启动了导入 `cvxpy/cma/scipy` 的全量 `preflight`；重算 37 MB STL 哈希 | 先做全分段计时；`--prebuilt` 与完整 `--check` 拆路，去掉重导入与 STL 哈希 |
| 原生完整 `pytest` | **326 用例 / 349 s** | 前 5 个文件占 **259 s**：4 次重复 1 kHz 物理 Mock 长跑（151 s）+ 遗留 `test_static_field.py`（49 s）+ 重复 `test_wave*` | 默认快速集 **< 25 s**；显式提供完整回归命令 `pytest -o addopts=''` |
| QEMU 非实时 `pytest` | **241 用例 / 2156 s（36 分钟）** | 14 轴 `full` 规划门禁与 `test_static_field` 占 15+ 分钟 | 默认快速集 **< 60 s** |

---

## 1. 执行前提、K1–K3 边界与安全红线

1. **分支与现状**：
   - 开始前先执行 `git status`，保留当前未提交的 Step 1–4 交叉构建成果。
   - `main` 分支领先 `orin-cross-build` 共同祖先 `399b0b0` 共 7 个提交（含 `kinematics_refactoring_plan.md`、H1 调优、`deploy_orin.sh` 等）；按需择机同步，避免直接覆盖。
2. **为 `K1–K3` 必须保留的最小集合（严禁误删）**：
   - **C++ 与 `_core`**：`cpp/include/casbot_calib/trical.hpp`、`cpp/src/trical.cpp`（`_core.trical_readings`、`_core.trical_offset`、`TricalCalibration`）、`_core.ArmDynamics.point_geometry_batch`、`cpp/src/arm_session.cpp`（供 `shm_mock_server` 静态链接）。
   - **Python 过渡模块**：`python/casbot_calib/trical_assembly.py`、`kinematics.py`、`model_context.py`。
   - **资产**：
     - `python/casbot_calib/assets/trical_1105_release_turn_reference.json`（48 KB）
     - `python/casbot_calib/assets/trical_measurement_defaults.json`（1 KB，由 `model_context.py:83` 通过 `importlib.resources` 读取）
     - `python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2/`
     - `python/casbot_calib/assets/models/trical_1105_03_gauge_cover_v2/`（2.62 MiB，默认模型 `model.json` 绑定）
     - `python/casbot_calib/assets/models/trical_1105_03/`（`bare_flange_1105_03_gauge_v2/scene_{left,right}.xml` 仍通过 `../../../trical_1105_03/meshes/part_*.stl` 引用其网格，`trical_assembly.py:18` 亦以它为无 bundle 兜底名，本轮保留不删）。
   - **可扩展设计**：WP1 将新增 `shm_trical_executor`；打包脚本收集运行时 `bin/` 时**动态扫描目录，严禁写死工具名清单**。
3. **保留全部硬件安全逻辑**：连续碰撞/URDF 限位/速度检查、准备位前缀限位/速度检查、起点静止与无故障校验、C++ 起点复核、单 owner、`SIGINT/TERM/HUP` 失能、故障即中止。开发与测试只连 `/casbot_calib_test_*`。

---

## 2. 推荐实施顺序与详细工作包

按照**先切交付物与测速、再砍遗留资产与旧代码、最后分层与合并测试**的顺序推进，每阶段均可独立验收。

### 阶段一：交付包瘦身、运行包隔离与安装分段计时（最高 ROI）

#### WP-1A：统一剥离所有 Wheel 中的大型资产（交叉 + 原生一致，< 4 MiB）
直接采用破坏性统一方案：**不再维持两套 wheel 打包语义**，原生的独立包自带 289 MiB 模型历史包袱一并清除。
1. **`pyproject.toml` 排除大资产但保留小配置文件**：
   - 在 `[tool.scikit-build]` 中配置 `wheel.exclude` 排除 `casbot_calib/assets/models/**` 与 `casbot_calib/assets/fieldwork_display/**`。
   - **保留** `casbot_calib/assets/` 根目录下的 ~120 KB 小型 JSON/XML（如 `trical_measurement_defaults.json`、`trical_1105_release_turn_reference.json`、`trical_capture_defaults.json`、`casbot_right_arm_fk_reference.json` 等），保证 `importlib.resources.files('casbot_calib')` 零改动可用。
2. **统一 `model_directory()` 与显示资产回退解析（复用既有 `CASBOT_PROJECT_ROOT`，不新增环境变量）**：
   - 修改 `python/casbot_calib/model_sources.py` 的 `model_directory()`：若包内 `files('casbot_calib')/'assets/models'` 存在目录则优先使用，否则直接回退到 `project_paths.ROOT / 'python/casbot_calib/assets/models'`。
   - 同步为 `python/casbot_calib/fieldwork_display.py:46` 加上 `project_paths.ROOT / 'python/casbot_calib/assets/fieldwork_display/dvt2_v1'` 回退。
3. **彻底移除资产回填与 QEMU 遮蔽挂载**：
   - 删除 `scripts/cross/pack_assets.py`；从 `scripts/cross/package_orin.py` 移除回填调用与传给 scikit-build 的临时资产 exclude 特判（已由 `pyproject.toml` 统一）。
   - 修改 `scripts/cross/verify_package.py`：**删除 `runtime_command(..., overlay=True)` 挂载**（该挂载用只读 wheel 包覆盖仓库 `python/casbot_calib`，一旦 wheel 不含 `models/` 会遮蔽仓库资产），向测试用 `VENV` 写入 `casbot-prebuilt` 标记并配置 `CASBOT_PROJECT_ROOT`（确保 `scripts/shm/python_path.sh` 走预编译路径而不重新注入源码 `PYTHONPATH`）。验证时除直接 `import` 断言 `_core.__file__` 位于 `VENV` 之外，**还必须通过真实 CLI 子进程（如 `calib/dynamics_h1.sh`）验证子进程同样加载已安装的 Python 包与 `_core`**。
4. **同步完善跨端部署与隔离验证的最小真实资产集**：
   - 修改 `scripts/shm/deploy_git.sh` 与 `scripts/cross/verify_install.py`（不新增多余 CLI 开关）：
     - 默认物化 **两套** 运行所需碰撞资产：`bare_flange_1105_03_gauge_v2/{model.json,model.urdf,collision.json,collision.npz,trical_scene.json}` **和** `trical_1105_03_gauge_cover_v2/collision.npz`（+ 其描述符）。
     - 不再随无头运行部署拉取 `visual_meshes/*.stl`（37 MiB），显示网格校验归口到 GUI/显示测试本身。
   - 同步更新 `scripts/humble/verify_wheel.py`：验证轻量 wheel 安装时直接设置 `CASBOT_PROJECT_ROOT` 指向仓库根目录；更新 `tests/test_orin_cross_scripts.py` 的 wheel 校验断言为“不含 `assets/models/` 与 `assets/fieldwork_display/`，保留 `assets/*.json` 与 `_core.*.so`/`.libs/libomp.so.5`”。

#### WP-1B：新建干净 runtime staging 打包 + 隔离 `CrossTests`（< 1.0 MiB tar）
修正直接复用 `COMPONENT Shm` 安装目录的隐患（当前 `cpp/src/shm/CMakeLists.txt:49-55` 会将 `libcalib_shm.a`、`include/casbot_calib/shm` 和 `casbot_calibShmTargets.cmake` 一并归入 `Shm` 组件，且复用旧前缀会残留 `CrossTests`/`libcalib_core.a`）。
1. **删除重复工具 `shm_joint_test`**：
   - 删除 `cpp/tools/shm_joint_test.cpp`（与 `shm_executor.cpp` 完全相同），从 `cpp/src/shm/CMakeLists.txt` 与 `scripts/cross/verify_openmp.sh` 工具列表中移除。
2. **开发安装树、运行交付包、测试安装树三路分离**：
   - 修改 `scripts/cross/package_orin.py`：
     - 完整 CMake 安装（含 `Core` / `Shm` 的 `.a`、头文件、`.cmake` 导出）继续安装到构建期开发前缀（例如 `build/orin-cross-dev-install`），保留给 C++ consumer 验证（`tests/installation/main.cpp`）和后续 `K1–K3` 开发使用。
     - 将 `CrossTests` 组件安装到**独立的全新临时/测试目录**（例如 `build/orin-cross-tests`），并为其配置正确的 `libomp.so.5` 加载路径（RPATH 或拷贝 `lib/libomp.so.5` 配合 `LD_LIBRARY_PATH`），严禁污染运行前缀。
     - 每次打 `shm-core-aarch64.tar.gz` 时从**新建清空的临时 staging 目录**仅收集运行必需产物：扫描运行端 `bin/*`（自动接纳未来新增的 `shm_trical_executor` 等工具，排除 `*_tests` 与 `openmp_probe`）、`lib/libomp.so*`、`share/licenses/casbot_calib/openmp/`。**不再打包 `CrossTests`、任何 `.a`、头文件或 `.cmake`**。
   - 更新 `scripts/cross/verify_openmp.sh`、`scripts/cross/verify_package.py`（第 51 行 `openmp_probe` ELF 审计路径）与 `qemu_package_smoke.py`：`CrossTests` 与 `openmp_probe` 统一从独立测试目录审计与执行，现场运行工具从解包后的运行目录执行。同步修改 `scripts/cross/verify_install.py`（第 64 行）末尾的 `openmp_probe` 调用，改为从独立测试目录执行，不再假设运行包内包含 `openmp_probe`。

#### WP-1C：`calib/setup.sh --prebuilt` 分段计时与更新/全检自然职责拆分
实测已确认 ARM64/QEMU 下仅导入 `preflight`（1.39 s）及其数值依赖（4.33 s）就耗时 **5.71 s**，因此**不预设未实测的“2–3 秒”承诺**，先测量再结构性加速，同时**不新增 `--quick` / `--viewer` 等命令行开关**：
1. **先加分段耗时埋点**：
   - 在 `calib/setup.sh` 与 `scripts/cross/verify_install.py` 中对 `link_math_bindings.py`、`pip install --no-deps --no-index --no-compile`、解包 `shm-core-aarch64.tar.gz`、`shm_abi_check`、Python 验收步分别输出毫秒级阶段耗时，定位真实热点（特别是重复拉起 Python 进程与 `pip` 耗时）。
2. **用单默认职责拆分替代新增参数**：
   - `calib/setup.sh --prebuilt` 结束时**不再调用全量 `preflight --offline`**（不再为了升级验证去 `import cvxpy/cma/scipy` 与遍历 37 MB STL/重建 BVH），直接在单进程短校验中导入 `_core`、`pinocchio`、`coal` 并验证运行态 LFS 非指针文件（`bare_flange_1105_03_gauge_v2/{model.urdf,collision.npz}`、`trical_1105_03_gauge_cover_v2/collision.npz`）。
   - `calib/setup.sh --check` / `preflight.py` 保持唯一默认离线校验路径（求解器 + URDF + 碰撞资产检查）；`preflight` 与 `dvt2_bundle.check_model_bundle()` 默认**不再**因无头环境缺少 `visual_meshes/*.stl` 而阻塞（显示网格完整性由本机 GUI 启动与 `test_fieldwork_display.py` 负责，不通过环境嗅探也不新增 `--viewer` 参数）。

---

### 阶段二：激进清理遗留资产、Python 仿真绑定与 `static_field`

#### WP-2A：删除过时模型资产包与过期顶层配置
1. **删除死模型衍生目录（立即释放 ~40 MiB 仓库资产与清理测试包袱）**：
   - 删除 `python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_v1/`（37.91 MiB，全仓零引用）。
   - 将 `tests/test_dvt2_bundle.py`、`scripts/humble/verify_wheel.py`、`tests/installation/main.cpp` 从候选旧包 `a2ff_stock_hands_candidate_v1` 迁移为校验当前唯一生产模型 `bare_flange_1105_03_gauge_v2`，随后**删除** `python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/a2ff_stock_hands_candidate_v1/`（2.08 MiB）与 `configs/dvt2_model_candidate.yaml`。
2. **清理过期顶层配置（对齐 `kinematics_refactoring_plan.md` 第 3.2 节）**：
   - 在排查零生产引用后删除：`configs/static_field_nominal.yaml`、`configs/kinematics_trical.yaml`、`configs/staged_dual_arm.yaml`、`configs/dvt2/`。

#### WP-2B：彻底清除 `static_field` 旧规划入口与测试（省 49 s 原生 / 428 s QEMU）
经检查，`h1_gravity.py:32` 从 `static_field.py` 导入 6 个符号（`_move`、`_select_arm`、`candidate_pool`、`information`、`paired_motions`、`screen_candidates`），此外 `test_wave1b_cvxpy_motion_cma.py:18` 导入了 `_SEPTIC_ANALYTIC_PEAKS`、`_SEPTIC_BERNSTEIN8_PEAKS`、`_move`，`h2_friction.py`、`h3_inertia.py`、`field_motion.py`、`shm_ready.py`、`shm_gui_zero.py` 导入了 `_move`。
1. **迁移全部复用实现（不留旧模块兼容壳）**：
   - 将 `_move()` 及相关样条峰值常数/辅助迁入 `python/casbot_calib/staged_motion.py` 并直接更新上述 6 个调用方与 `test_wave1b_cvxpy_motion_cma.py` 的导入路径。
   - 将 H1 现行规划直接依赖的 `_select_arm`、`candidate_pool`、`information`、`paired_motions`、`screen_candidates` 迁入动力学规划模块（`stages/dynamics/h1_gravity.py` 或同目录辅助模块）。
2. **彻底删除遗留接口与专属测试**：
   - 删除 `plan_static_field` / `verify_static_field`（乃至清空或删除 `python/casbot_calib/static_field.py` 及其 CLI/配置引用）。
   - 删除 `tests/test_static_field.py`（16 个用例）。

#### WP-2C：激进裁剪 C++ 遗留仿真模块与 Python 仿真绑定
1. **删除零生产意义的 C++ 模块及同步测试目标**：
   - 从 `CMakeLists.txt` 的 `calib_core` 移除并删除文件：
     - `cpp/src/simulator.cpp` + `cpp/include/casbot_calib/simulator.hpp`（1-DOF 虚拟关节）
     - `cpp/src/arm_guard.cpp` + `cpp/include/casbot_calib/arm_guard.hpp`（旧单臂反馈保护）
     - `cpp/src/trical_mock.cpp` + `cpp/include/casbot_calib/trical_mock.hpp`（旧 50 Hz 传感器仿真）
   - **同阶段立即移除依赖 `simulator.cpp` 的 `core_tests` CMake 目标并删除 `cpp/tests/core_tests.cpp`**（不等到阶段三，确保阶段二可独立完成编译与 `ctest` 验收）。
   - 从 `cpp/src/bindings.cpp` 移除 `SimulationConfig` / `SimulationRunner` 绑定；同步清理 `scripts/humble/collect_environment.py:36` 中对 `_core.SimulationRunner.integration_backend()` 的调用（改为保留 `_core.ArmDynamics.backend_version()` 或与真实积分器查询对齐）；更新 `tests/installation/main.cpp` 移除 `TricalMockSession`。
2. **移除 Python 侧仿真绑定（`_core.ArmSimulation*`）并将物理断言下沉至 C++**：
   - 全仓 `python/`、`calib/`、`scripts/cross/` **无任何生产代码**调用 `_core.ArmSimulationSession` / `_core.ArmSimulationRunner` / `_core.ArmGuardConfig` / `_core.TricalMockSession`。
   - 先将现有覆盖迁移至 C++ 编译态测试（`core_unit_tests`，编译与运行耗时均远低于 Python/QEMU）：
     - `tests/test_drive_power.py` 的逐轴失能、待处理命令不可重新使能、失能后物理状态继续演化断言 → 迁入 `cpp/tests/` 测 `ArmSimulationSession`。
     - `tests/test_dual_arm.py:49-71` 与 `tests/test_friction_sampling.py` 中对 `ArmSimulationRunner` 的多轴积分能量收敛、可重复性与采样混叠断言 → 迁入 `cpp/tests/` 测 `ArmSimulationRunner` / `ArmSimulationSession`。
   - 迁移完成后，**整体移除 `cpp/src/arm_simulator_bindings.cpp`**（并删除 `test_drive_power.py`、`test_arm_guard.py`、`test_friction_sampling.py` 中仅依赖 Python 仿真绑定的测试，更新 `verify_wheel.py` 改为测 `ArmDynamics` + `trical_readings`）。
   - *说明*：C++ 本体的 `arm_session.{hpp,cpp}` 完整保留给 `shm_mock_server` 与 C++ 单元测试。
3. **迁全保留接口的独有断言后清理 `arm_model.py` 与单臂旧测试**：
   - 将 `arm_model.py` 仅余测试所需的解析 2-link 模型生成内联为测试 fixture（供 `test_dual_arm.py` 使用）。
   - **删除前提**：将 `tests/test_arm_pinocchio.py` 与 `tests/test_arm_dynamics.py` 中覆盖保留接口（`ArmDynamics` / `ArmModel` / `ArmWorkspace`）的**全部独有断言**迁移至 `test_dual_arm.py`（以下清单为重点而非穷举）：
     1. `ArmWorkspace` 在 owner (`ArmDynamics`) 销毁后依然存活；
     2. 并发 `workspace` 隔离性与多线程只读求值一致性；
     3. 非连续 numpy 数组批处理输入及零行批处理边界；
     4. 完整摩擦导数验证，以及耗散性 / Stribeck 曲线特性；
     5. 刚体参数排列（`rigid_parameter_names`）、质心惯量到原点惯量换算，以及返回参数副本的独立性（原地修改返回值不影响后续求值）；
     6. 回归矩阵（`torque_regressor`）仅依赖几何结构，不依赖名义质量、质心、惯量、转子惯量与摩擦参数；
     7. 模型快照不可变性（构造后修改外部 `model` 或 `engine.model` 快照不影响已构造的动力学对象）；
     8. 平面 TCP 坐标系与 Jacobian 行序约定校验。
   - **所有独有断言迁移完成并验证通过后**，再删除 `python/casbot_calib/arm_model.py`、`tests/test_arm_dynamics.py`、`tests/test_arm_pinocchio.py`。

---

### 阶段三：测试分层、物理闭环去重与 C++ 测试合并

#### WP-3A：建立 `fast` / `slow` 测试分层并明确 K1–K3 全量回归命令
1. **配置默认跳过 `slow`**：
   - 在 `pyproject.toml` `[tool.pytest.ini_options]` 增加：
     ```toml
     markers = [
         "slow: full 14-axis planning gates, long 1kHz physical loops, and full asset source scans",
     ]
     addopts = "-m 'not slow'"
     ```
   - 同步在 `scripts/cross/pytest_qemu.py` 默认走相同过滤（透传 `-o addopts=''` 或 `-m slow` 时执行慢测）。
   - **明确全量验收入口**：因为默认 `pytest -q` 将只跑快速集，在本计划与 `kinematics_refactoring_plan.md` 的验收指令中显式约定全量命令为：
     `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -o addopts='' -q`。

#### WP-3B：按验证行为合并物理闭环与轻量替代长规划
1. **轻量化可无代价加速的测试（不降覆盖）**：
   - `test_dynamics_stages.py::test_cross_session_prior_auto_discovery`（8 s 原生 / 81 s QEMU）：改用轻量合成 `h1`/`h2` stage session JSON fixture 直接测 `resolve_stage_prior()`（**< 0.2 s**）。
   - `test_shm_static_small.py` 的 `solver_unavailable` 分支：改为复用合成二进制 capture（`_write_synthetic_binary_capture`），不再额外启 22 秒物理 Mock。
   - `test_model_sources.py`（17 用例）：保留目录解析与路径逃逸校验，将全量 5 个 `source/` 变体与 625 个 `.obj` 逐个扫描移入 `@pytest.mark.slow`。
   - 精简合并 `test_dvt2_bundle.py`、`test_dvt2_bundle_portability.py`、`test_dvt2_defaults.py`、`test_model_context.py`，消除重复夹具构建，聚焦 `bare_flange_1105_03_gauge_v2`。
2. **按真实验证点重组 1 kHz 物理 Mock 闭环（保留 `--resume`、单次使能、成对采样、阶段衔接，不盲目删行为）**：
   - 拆清现存 4 条长闭环的各自职责，消除重复：
     - **完整三阶段 H1→H2→H3 闭环 + 阶段衔接（`@pytest.mark.slow`）**：在 `test_dynamics_stages.py` 保留 1 个精简配方闭环，覆盖 H1→H2→H3 工件传递与分析。
     - **`--resume` 断点恢复**：不再从头跑完整个 H1 再 resume 全跑，改为通过合成已完成 H1 会话或极短前置触发 resume 衔接，避免 H1 双跑。
     - **准备位单次使能（`test_shm_ready.py`）与成对采样（`test_shm_static_small.py`）**：保留独立精简闭环与核心断言；缩短 `test_shm_fieldwork.py` / `test_shm_h1.py` / `test_pipeline.py` 的无意义等待（注意：`test_shm_ready` 与 `test_shm_static_small` 已使用 `measure_s=0.1`、`settle_s=0.05`，压缩运动/保持等待需先用 `--durations` 实测收益后再调整，不拍脑袋缩采样窗口）。
3. **合并 `test_wave*` 与 `test_dynamics_stages` 重复规划/分析用例**：
   - 快速集仅保留各阶段 1 份 `small` 规划与合成分析门禁；所有 14 轴 `full` 规划门禁（21–25 s 原生 / 473 s QEMU）与 CPU 基准标为 `@pytest.mark.slow`。
4. **QEMU 数学验证保持单默认完整对照（不拆 `--full` 参数）**：
   - `scripts/cross/qemu_math.py` **不新增 `--full` 或抽样模式开关**，保持单一默认行为：每次均将 **全部 32 行** 送入 `_core` 触发 16 行/8 行 OpenMP 并行阈值，并执行 **全部 32 行** Python `coal.distance()` 参考对齐以及已发现的碰撞求解反例回归，同时打印实际参考行数与反例结果。
   - `qemu_package_smoke.py` 去掉重导入全量 `preflight` 后，默认冒烟本身即既快又包含完整数学对照，无需维护两套参数与脚本分支。

#### WP-3C：C++ 测试可执行文件合并（低优先级收尾）
测试程序已在阶段一从 Orin 交付包移出，本项主要缩短 CMake 链接体积与测试目录文件数：
- 删除未入 CTest 的死文件 `cpp/tests/shm_joint_integration.cpp`（`core_tests.cpp` 已在 WP-2C 删除）。
- 将 `arm_core_tests.cpp`、`simulated_device_tests.cpp`、`kinematics_tests.cpp`、`integration_tests.cpp` 及从 Python 迁入的 `ArmSimulationSession`/`ArmSimulationRunner` 物理断言合并为 `core_unit_tests`。
- 将 6 个 SHM 单元测试合并为 `shm_unit_tests`。
- 保留独立辅助：`shm_field_fixture`、`openmp_probe`、`shm_abi_check`。

---

## 3. 终局验收门禁

每阶段完成后按对应检查验证，全部收尾后执行以下完整验收（同步更新 `scripts/humble/validate.sh` 的全量 `pytest` 参数），并将实测耗时、产物大小、数学参考行数与碰撞反例结果写入 `docs/records/`：

```bash
# 1. 原生 Humble 重建并重装当前源码绑定（避免继续使用旧 build/humble-python 扩展）+ CTest + 快速 pytest
bash scripts/humble/container.sh bash -c '
  cmake --preset humble -DCALIB_BUILD_PYTHON=ON -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=ON &&
  python -m pip install -e . --no-deps --no-build-isolation --config-settings=build-dir=build/humble --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON &&
  cmake --build --preset humble &&
  ctest --preset humble --output-on-failure &&
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q --durations=10
' < /dev/null

# 2. 原生全量验证（含 slow 用例、C++ consumer 与非 editable 原生 wheel 构建/校验）
#    复用并同步更新 scripts/humble/validate.sh（其第 13 行改为 python -m pytest -o addopts='' -q）
bash scripts/humble/container.sh bash scripts/humble/validate.sh < /dev/null

# 3. ARM64 交叉构建 + 瘦身打包 + QEMU 默认冒烟（含分段耗时 + 完整 32 行 Coal 对照与反例）
bash scripts/shm/cross_build_orin.sh

# 4. QEMU 离线首次/更新安装分段耗时验收（openmp_probe 走独立测试树）
python3 scripts/cross/verify_install.py

# 5. QEMU 非实时默认快速 pytest（与完整 32 行数学对照同跑，或透传 -o addopts='' 跑全量）
bash scripts/shm/verify_orin_qemu.sh --pytest
```

### 硬性验收线
- 所有产物 `.whl`（交叉 `dist/orin-aarch64/*.whl` 与 `validate.sh` 生成的非 editable 原生 wheel）均 **< 4 MiB**，不含 `assets/models/` 与 `assets/fieldwork_display/`，保留 `assets/*.json`（含 `trical_measurement_defaults.json` 与 `trical_1105_release_turn_reference.json`）。
- `dist/orin-aarch64/shm-core-aarch64.tar.gz` **< 1.0 MiB**，**不含任何 `*_tests`、`.a` 静态库、头文件及 `.cmake` 导出文件**，保留运行时 `bin/`（动态扫描，兼容后续 `shm_trical_executor`）、`lib/libomp.so*` 与 OpenMP 许可。
- `verify_install.py` 输出明确的分段实测耗时（`link_math` / `pip` / `tar` / `shm_abi_check` / 快速检查）。
- `verify_package.py` 与 `verify_install.py` 中的直接导入及真实 CLI 子进程均命中预编译安装包的 `_core`，不回退源码 `PYTHONPATH`。
- 原生默认快速 `pytest -q` **< 25 s**；QEMU 默认 `verify_orin_qemu.sh --pytest` **< 60 s**；原生全量 `pytest -o addopts='' -q`（经 `validate.sh`）与 `cross_build_orin.sh` 内含的完整 32 行 Coal 参考 + 碰撞反例全部通过。
- `_core.trical_readings`、`_core.trical_offset`、`_core.ArmDynamics.point_geometry_batch`、`trical_assembly` 及 `bare_flange_1105_03_gauge_v2` / `trical_1105_03_gauge_cover_v2` 全部工作正常。
