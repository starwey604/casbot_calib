# `orin-cross-build` 合入 `main` 与运动学（K1–K3）衔接执行计划

本文档供多 Agent 交叉核对与按序执行。遵循“**加东西不如删东西、不搞双轨防御分支**”原则，先将 `/home/ww/codings/casbot_calib-cross`（`orin-cross-build` @ `0811afc`）合入 `main`（@ `c7389b3`），收敛部署与验收入口并跑通全套原生 + ARM64/QEMU/断网安装门禁，再在单工作区推进 [kinematics_refactoring_plan.md](kinematics_refactoring_plan.md) 的 WP1–WP5 实施。

---

## 1. 分叉基线与收敛策略

两分支共同祖先为 `399b0b0`：
- **`main`（`4c72b1b..c7389b3`）**：H1 实机全臂正反接近（`approach_joints`）、库仑正则解耦、镜像增益调优与实机诊断记录；`shm_gui_remote` 解耦 `_core`；`deploy_orin.sh` / `run_orin_tests.sh` / USB 配网脚本；运动学蓝图 [kinematics_refactoring_plan.md](kinematics_refactoring_plan.md)。
- **`orin-cross-build`（`4369de6..0811afc`）**：LLVM `22.1.8` + ThinLTO（`cortex-a78ae`）交叉镜像与工具链、私有 `libomp.so.5`、`dist/orin-aarch64/` 打包、`calib/setup.sh --prebuilt`、`deploy_git.sh --with-prebuilt`、QEMU 冒烟/回归与断网安装验收；Coal 跨行状态与 SHM 关节生命周期修复；二进制观测只读 `mmap`；剥离 wheel 大资产（统一走 `project_paths.ROOT`）、清理旧仿真/旧模型/旧配置、提取 `stages/dynamics/h1_selection.py`、合并 C++ 单测并通过 `@pytest.mark.slow` 分层测试。

### 针对跨分支重叠的减法收敛决策
1. **删除过渡期脚本 [run_orin_tests.sh](../scripts/shm/run_orin_tests.sh)（不修补、不加双模式）**：
   - 该脚本是交叉构建落地前在 Orin 实机强跑开发 `pytest` 的临时产物（全仓零引用，内含 `/usr/lib/python3/dist-packages` 注入与手工 LFS `--ignore` 表）。
   - 现 ARM64 单元/求解器/离线回归统一在本机 `verify_orin_qemu.sh --pytest` + `verify_install.py` 完成；Orin 本机仅需 `calib/setup.sh --check`（环境+资产+通道预检）与 `scripts/shm/verify_physics.py --bin-dir install/shm-core/bin`（实机 RT 内核 1 kHz 闭环），直接 `git rm scripts/shm/run_orin_tests.sh`。
2. **[deploy_orin.sh](../scripts/shm/deploy_orin.sh) 单一化为交叉预编译流（删掉 `--jobs`，不加 `--prebuilt`/`--cross-build` 组合开关）**：
   - 从 x86_64 开发机一键部署到 Orin 时，唯一正式路径就是交叉预编译：`bash scripts/shm/deploy_orin.sh` 默认直接执行 `cross_build_orin.sh --no-verify`（热缓存 ~3.8 s）→ `deploy_git.sh --with-prebuilt` → 远端 `calib/setup.sh --prebuilt`（`--check-only` 时跳过构建及预编译包同步，仅同步源码/资产/轮子并跑远端 `calib/setup.sh --check`；`--no-setup` 时仅构建并同步）。
   - Orin 本机无参数运行 `bash calib/setup.sh` 仍保留原生编译回退，不在主机一键脚本里暴露双模式开关。
3. **`calib/setup.sh --prebuilt` 源头清理残留原生构建目录**：
   - 在 `--prebuilt` 分支重建 `install/shm-core` 时，同步执行 `rm -rf -- "$repo/install/shm-core" "$repo/build/shm-python" "$repo/build/shm-core"`（并在 `verify_install.py` 更新阶段顺带放入假 `build/shm-python` 断言被清除），从根源杜绝“原生切预编译后残留旧构建目录”，无需在下游脚本写防御判断。

---

## 2. 阶段 A：`orin-cross-build` 合入 `main` 逐文件实施清单

实施及验收结果见 [合并记录](records/cross_merge_20261115.md)。

在 `/home/ww/codings/casbot_calib`（`main`）执行 `git merge --no-commit orin-cross-build` 后按以下清单处理：

1. **H1 `approach_joints` 跨文件语义补齐（必改）**
   - `git rm python/casbot_calib/static_field.py tests/test_static_field.py`；
   - 在 `python/casbot_calib/stages/dynamics/h1_selection.py` 补入 `main`（`4c72b1b`）的两处改动：
     - `paired_motions()`：使用 `row.get('approach_joints', row['active_joints'])` 计算 `near`，且当 `'approach_joints' in row` 时写入 `conditions['approach_joints'] = list(row['approach_joints'])`；
     - `screen_candidates()`：端点筛选改用 `x[row.get('approach_joints', row['active_joints'])] += sign * config['approach_offset_rad']`；
   - 将 `4c72b1b` 的 `test_explicit_full_arm_approach_keeps_target_axes_and_other_arm_unchanged` 迁入 [test_wave2a_h1.py](../tests/test_wave2a_h1.py)（改从 `casbot_calib.stages.dynamics.h1_selection` 导入）。
2. **无头模型测试与显示资产解耦**
   - 将 `tests/test_model_context.py` 中 `test_runtime_context_does_not_import_python_pinocchio_or_coal` 末尾两行 `viewer_adapter(context.bundle)` 断言移入 `tests/test_fieldwork_display.py`，确保 `test_model_context.py` 在仅含 8 个无头运行资产的布局下零显示网格依赖。
3. **部署与安装脚本收敛**
   - `git rm scripts/shm/run_orin_tests.sh`；
   - `scripts/shm/deploy_git.sh`：直接取 `orin-cross-build` 版本（已由 `test_deploy_streams_assets_wheels_and_prebuilt_over_one_connection` 覆盖，无需额外增加远端 SSH 探测往返）；
   - `calib/setup.sh`：取 `orin-cross-build` 版本，在 `--prebuilt` 分支补入清理 `build/shm-python` 与 `build/shm-core`，在原生回退分支保留 `>= 16 GB` 内存选 `jobs=4` 及清理旧 `build/shm-core`；
   - [deploy_orin.sh](../scripts/shm/deploy_orin.sh)：删除 `--jobs`，固定串联 `jetson_network.sh` / 时钟对齐 → `cross_build_orin.sh --no-verify`（非 `--check-only` 时）→ `deploy_git.sh --with-prebuilt`（`--check-only` 时不带 `--with-prebuilt`）→ 远端 `calib/setup.sh --prebuilt`（或 `--check`），并在 `tests/test_orin_cross_scripts.py` 增加 1 个替身测试覆盖默认流、`--check-only` 与 `--no-setup`。
4. **其余冲突文件**
   - [test_wave2a_h1.py](../tests/test_wave2a_h1.py)：保留 `main` 的 `approach_offset_deg == 8.0`、`approach_joints` 断言、`aarch64` 15 s 墙钟上限与 `undithered` 参数化断言，同时保留 `orin-cross-build` 的 `@pytest.mark.slow` 分层。
   - [shm_build.md](shm_build.md)：取 `orin-cross-build` 全文，在第 8 节预编译部署处合入 `bash scripts/shm/deploy_orin.sh` 一键说明。
   - [kinematics_refactoring_plan.md](kinematics_refactoring_plan.md)：终局验收命令同步改为调用 `scripts/humble/validate.sh`（保证重装 `_core` 扩展）。

### 阶段 A 提交前完整验收门禁
必须在提交合并并清理 `casbot_calib-cross` 工作树**之前**全部跑通（注意用 `validate.sh` 强制重装 `.venv-humble` 中的 `_core` 绑定）：

```bash
# 1. 本机 Humble 完整验收（重装 editable _core + CTest + 全量 pytest -o addopts='' + C++ consumer + wheel 校验）
bash scripts/humble/container.sh bash scripts/humble/validate.sh < /dev/null
bash scripts/shm/build.sh --with-mock

# 2. 本机交叉编排单测（含新增 deploy_orin.sh 替身测试）
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_orin_cross_scripts.py -q < /dev/null

# 3. ARM64 ThinLTO 交叉打包 + QEMU 冒烟 + QEMU 非实时 pytest + 断网无头首次/更新安装验收
bash scripts/shm/cross_build_orin.sh
bash scripts/shm/verify_orin_qemu.sh --pytest
python3 scripts/cross/verify_install.py
```

全部通过后提交合并 commit，随即执行 `git worktree remove /home/ww/codings/casbot_calib-cross` 与 `git branch -d orin-cross-build`。

---

## 3. 阶段 B：运动学（K1–K3）基于新基线的实施约束

不新增额外阶段或新配置层，严格按 [kinematics_refactoring_plan.md](kinematics_refactoring_plan.md) 的 WP1–WP5 执行，仅对齐以下 5 条新基线规则：

1. **复用已完成清理，不重复造轮子**
   - `trical_mock.{hpp,cpp}`、`kinematics_trical.yaml`、`static_field_nominal.yaml`、`dvt2/staged_dual_arm.yaml`、`dvt2_model_candidate.yaml` 已在 `orin-cross-build` 删除；
   - 直接复用现存 `cpp/include/casbot_calib/trical.hpp`、`ArmDynamics::point_geometry_batch` 与 `trical_assembly.py`（迁入 `trical/assembly.py` 时将默认资产目录对齐为 `trical_1105_03_gauge_cover_v2`）。
2. **WP1 C++ 二进制与测试分层**
   - `shm_trical_executor` 加入 `cpp/src/shm/CMakeLists.txt` 的 `Shm` 工具循环（`package_orin.py` 会自动 strip 并打进 `shm-core-aarch64.tar.gz`）；`preflight.py` 同步校验该二进制。
   - 纯确定性逻辑（C3 七次多项式、200 ms 滤波停稳滑窗、`max_windup_rad` 拦截、JSONL 状态机、`--bare-flange` 744B 零帧）编入 `shm_unit_tests`（自动参与 QEMU `--smoke`）；1 kHz 实时双进程闭环（`shm_mock_server` + `shm_trical_executor` 5 s 保持与故障失能）注册为原生 `shm_trical_cli`（`LABELS "shm;physics"`），避免 QEMU 用户态调度抖动。
3. **CLI 与路径兼容非 editable Wheel**
   - `calib/kinematics*.sh` 统一通过 `source "$repo/scripts/shm/python_path.sh"` 启动；Python 侧读取 `configs/kinematics/*.yaml` 与默认输出目录统一走 `casbot_calib.project_paths.ROOT`。
4. **无头运行资产与断网安装覆盖**
   - `scripts/shm/runtime_assets.py` 补入 `trical_1105_03_gauge_cover_v2` 下的 `seeds.json`、`orientation_seeds.json`、`pillar_holes.json`、`table_holes.json`、`gauge_cover_evidence.json`；`scripts/cross/verify_install.py` 追加一行 `bash calib/kinematics.sh --plan-only` 断网冒烟。
5. **Pytest 快速集与 `LIVE_SHM` 分层**
   - `test_trical_indicators_lowrate.py`、`test_trical_routes_and_solver.py` 及离线 `--plan-only` / `K3` / `--reanalyse` 保持在默认快速集（自动跑进 QEMU `--pytest`）；
   - 拉起真实 1 kHz `shm_mock_server` + `shm_trical_executor` 的端到端闭环测试标记 `@pytest.mark.slow` 并登记到 `scripts/cross/pytest_qemu.py` 的 `LIVE_SHM`。

### 阶段 B 终局验收门禁
```bash
# 1. 原生完整验收（含 _core 重装、全量 CTest、全量 slow pytest、consumer 与 wheel 校验）
bash scripts/humble/container.sh bash scripts/humble/validate.sh < /dev/null
bash scripts/shm/build.sh --with-mock

# 2. 运动学 CLI 烟雾（离线规划 + 3Hz Mock 全链路 + 免动重算）
bash scripts/humble/container.sh bash calib/kinematics.sh --plan-only < /dev/null
bash scripts/humble/container.sh bash calib/dev.sh kinematics-mock --mode small < /dev/null
bash scripts/humble/container.sh bash calib/kinematics.sh --reanalyse < /dev/null

# 3. ARM64 交叉打包 + QEMU 冒烟/非实时回归 + 断网无头安装验收（含 kinematics --plan-only）
bash scripts/shm/cross_build_orin.sh
bash scripts/shm/verify_orin_qemu.sh --pytest
python3 scripts/cross/verify_install.py
```
