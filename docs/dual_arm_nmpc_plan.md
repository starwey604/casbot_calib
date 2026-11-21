# 双臂 NMPC 遥操作与自碰撞避让实现计划

本机 MuJoCo Mocap 交互拖动控制 CASBOT DVT2 双臂（14 轴裸法兰）：本机运行移植自 `~/codings/Winora/nmpc_local/` 的 `acados SQP-RTI + HPIPM` 运动学 NMPC 与自碰撞避让，叠加标定动力学（重力 + 摩擦 + 惯量）逆动力学前馈 $\tau_{\mathrm{ff}}$，经现有 Orin SHM 运控链路实现低刚度丝滑运动。

本文档供执行 Agent 直接实施。原则：**less is more——复用现有模型、连续碰撞核、动力学核与 `shm_gui_owner`，不写重复轮子；`acados / CasADi` 仅存在于本机环境，不污染主 `CMakeLists.txt`、Humble 容器和 Orin 交叉编译；不改动现有标定与 `--hold-body` 路径。**

---

## 1. 已确认前提与复用边界

- **本机环境**：直接复用 `/home/ww/.conda/envs/casbot_calib`（Python 3.12、Pinocchio 3.9、Coal、Boost 1.88、MuJoCo 3.16、CasADi 3.7.2）。需在 Phase 0 重建匹配当前源码的 `_core` 绑定，并单独编译链接 conda 库的宿主机版 SHM Mock/Owner 到 `build/nmpc/shm-host/`（不覆盖 Humble 容器产物 `install/shm-core/`），再补装固定版本 `acados v0.6.0`（commit `503364817c872d474ab5bed219c26760ac267769`）。
- **末端与构型**：基于默认裸法兰派生模型 `derived/bare_flange_1105_03_gauge_v2/`，跟踪 `left_flange_frame` 与 `right_flange_frame`。
- **控制结构**：采用**14 轴 3 阶运动学 NMPC + `ArmDynamics::inverse` 标定前馈 + 低刚度 PD** 解耦架构。本机运动学仿真的控制周期为 `0.04 s`，8 个预测节点覆盖 0.32 s，15 段停车尾段覆盖 0.60 s。MuJoCo 状态显示按 30 Hz 同步。
- **实机边界**：开发与测试只连纯运动学仿真或私有 SHM Mock 端点 `/casbot_calib_test_*`；**严禁未经用户明确要求连接真机 `/shm_mc`**。

---

## 2. 工作区代码组织

```text
casbot_calib/
├── configs/nmpc/
│   └── dvt2_teleop.yaml             # 14 轴 NMPC 配置（步长、权重、零空间偏好位、按对碰撞阈值、低刚度 PD、mocap 滤波）
├── python/casbot_calib/nmpc/        # 独立子包（标定主流程与默认 pytest 不 import）
│   ├── __init__.py
│   ├── model.py                     # 14 轴 + 固定障碍模型契约（移植自 Winora model.py）
│   ├── qp_options.py                # HPIPM fast 默认数值选项（移植自 Winora qp_options.py）
│   ├── import_dvt2.py               # 基于 CollisionProfile.pairs 审计原始凸包间距、拟合胶囊与 per-pair 阈值
│   ├── build.py                     # CasADi + acados SQP-RTI 生成与本机编译
│   ├── runtime.py                   # ctypes 封装 libcasbot_nmpc_runtime.so
│   ├── dynamics_ff.py               # 加载 delivery/（calibrated.urdf + runtime_dynamics.json）或名义模型算 τ_ff
│   ├── mocap_viewer.py              # 独立进程 MuJoCo 交互窗口（渲染反馈 + 拖动左右手 mocap + SE(3) 滤波）
│   ├── bridge.py                    # 闭环编排：mocap -> NMPC -> 连续碰撞认证 -> τ_ff -> 运动学 / RemoteClient
│   └── cli.py                       # 统一 CLI：import / build / sim / mock / teleop
├── cpp/nmpc/                        # 独立 CMake 工程，不挂入根目录 CMakeLists.txt，不参与 Orin 交叉编译
│   ├── CMakeLists.txt
│   ├── runtime.cpp                  # 移植自 Winora runtime.cpp（2×7=14 轴 + 固定障碍 arm=-1 + per-pair 阈值）
│   └── runtime_api.h
├── scripts/nmpc/
│   ├── setup.sh                     # 重建 conda 版 _core + 编译宿主机版 shm-host + 安装 acados v0.6.0
│   └── run.sh                       # 统一入口（配置 ACADOS_SOURCE_DIR / LD_LIBRARY_PATH / 单线程 BLAS）
├── third_party/                     # 加入 .gitignore（存放本机 acados v0.6.0）
└── tests/nmpc/                      # 专项测试（缺 acados 或原生库时自动 skip，不影响默认 pytest）
    ├── test_import_dvt2.py
    ├── test_nmpc_core.py
    ├── test_dynamics_ff.py
    └── test_shm_mock_loop.py
```

生成产物（模型 JSON、acados 代码、`libcasbot_nmpc_runtime.so`、宿主机版 `shm-host/`）统一落在 `build/nmpc/`（已在 `.gitignore` 中）。

---

## 3. 核心设计要点

### 3.1 碰撞对复用与结构性近邻处理（最优先）
已离线实测确认：DVT2 原始 CAD 凸包在准备位最小间距为 **11.811 mm**，`home_rad` 为 **11.206 mm**（最近对均为 `waist_yaw_link` ↔ `left_shoulder_roll_link`）。因此**不能用全局统一的 15 mm / 25 mm 阈值**，也不可通过扩充排除表绕过：
1. **唯一碰撞对来源**：直接复用 `dvt2_bundle.robot_profile()` 的 `CollisionProfile.pairs`（保留全部现有排除规则，不增删排除对）。
2. **按碰撞对设定阈值（`pair_clearance_m`）**：
   - 普通碰撞对（双臂互碰、前臂/腕部对躯干/头部/双腿等）使用目标默认裕度（仿真 `0.015 m`，真机初测 `0.025 m`）；
   - 结构性近邻对（在准备位或 `home_rad` 下原始凸包间距低于默认裕度者，如 `waist_yaw_link` ↔ 左/右 `shoulder_roll_link`）在导入阶段基于原始凸包实测间距单独核定可行阈值（例如准备位实测值的保守下限，确保从准备位和 `home_rad` 均可合法启动，同时绝不放任穿透）。
3. **QP 线性化几何策略（胶囊为主、逐对回退凸包）**：
   - `import_dvt2.py` 对 `CollisionProfile.pairs` 逐对对比准备位/`home_rad` 的原始凸包间距与外包胶囊间距；
   - 正常满足阈值的对直接用保守外包胶囊进入 C++ `runtime.cpp`（支持固定机身障碍）；
   - 所有碰撞对均按自己的阈值选择代理。若任一审计姿态的原始凸包通过而胶囊未通过，QP 和快速初筛均使用该对的原始凸包。每个回退对用其两个几何体构造专属 `CollisionChecker`，用该对的最近点计算距离雅可比；全模型查询只返回全局最近对的最近点。固定端点的雅可比为零，活动端仍贡献距离梯度。

### 3.2 14 轴 NMPC 与连续碰撞认证
- **模型泛化**：Winora 的 `6/12/36` 泛化为 `7/14/42`（`[q(14), v(14), a(14)]`，控制量为归一化 jerk `u(14)`，`origins (2,7,4,4)`、`axes (2,7,3)`、`tools (2,4,4)` 来自 `load_context()`）。
- **代价函数**：双臂法兰位置 + 旋转误差、**7 轴零空间姿态正则** $w_{\mathrm{post}}\|q - q_{\mathrm{nom}}\|^2$（防肘部漂移）、速度/加速度/jerk 平滑项、可选 7 轴奇异惩罚（$J \in \mathbb{R}^{6\times 7}$）。
- **限位与停车尾段**：位置/速度限位取 URDF 与 `configs/robot/dvt2.yaml`（`1.5 rad/s`）的保守交集；加速度/jerk 限值在 `dvt2_teleop.yaml` 配置；保留停车尾段（终端 $v=a=0$）与缩步/停车后备（`backup`）。
- **两级碰撞检查（满足“正式参考发送前连续碰撞检查”要求）**：
  1. **快速初筛**：以起点距离、关节运动半径及三次关节曲线的 Bernstein 包络计算间距下界。下界超过该对裕度的区间直接通过；
  2. **严格连续认证**：对从拟发送段起直至静止的整条可执行路径（包括中间预测段和停车尾段），直接复用现有 `collision.py` 的 `certify_reference()`（或等价的 `CollisionProfile.check()` + `motion_radii()` 区间距离下界检查，使用同一份 `pair_clearance_m`），确保相邻采样点之间无穿透漏检。连续认证未通过时禁止发送，转缩步或切入已认证的 `backup` 停车计划。**连续认证耗时计入每个控制周期总耗时统计**。

### 3.3 标定动力学前馈（`dynamics_ff.py`）
- 优先加载 `--delivery <session>/delivery`（`calibrated.urdf + runtime_dynamics.json`，经 `dynamic_export._restore_runtime`）；未传时尝试 `artifacts/calib/latest_dynamics/delivery/`；标定未完成前自动回退名义模型并允许在 YAML 里配置占位摩擦/转子惯量。
- 调用 `engine.inverse(q_ref, v_ref, a_ref)` 计算完整重力 + 摩擦 + 惯量（含转子惯量）前馈 $\tau_{\mathrm{ff}}$；库仑摩擦施加平滑速度下限（默认 `0.03 rad/s`）防零速抖振；超 `feedforward_limit_nm` 直接拒绝发送，不静默夹取。支持 `--no-ff` 开关便于对比。

### 3.4 MuJoCo 双向 Mocap 交互（`mocap_viewer.py`）
- 用 `context.write_view_scene()` + `lightweight_scene()` 构建场景，注入两个 `mocap="true"` 手柄 `left_goal / right_goal`（半径 `0.035 m` 半透明球 + `0.10 m` RGB 轴，`group="0"`、`contype="0" conaffinity="0"`）及可选碰撞胶囊显示层。
- 独立子进程运行 `mujoco.viewer.launch_passive`：双击选中手柄后 `Ctrl + 右键` 平移、`Ctrl + 左键` 旋转；快捷键支持 `R`（对齐当前法兰）、`Space`（暂停/恢复）、`C`（切换胶囊显示）、`Esc`（停止）。
- **防抢鼠标与防跳变**：仅在初始化、暂停或按 `R` 时写 `data.mocap_pos / data.mocap_quat`；跟踪态在 `with viewer.lock():` 内只读 mocap 传回控制侧、只更新机器人 `data.qpos`；对原始 mocap 位姿做 SE(3) 线速度/角速度限幅滤波（默认 `0.35 m/s`、`1.2 rad/s`）。

### 3.5 SHM 桥接、入队确认与故障安全（`bridge.py`）
- 直接复用现有 `RemoteClient` + `shm_gui_owner`（零改动 Orin 二进制），通过 `settings` 显式配置限位、`effort_limit`、`kp_max`、`kd_max`。
- **应对 `fields` 覆盖机制（基于 `revision` 与 `applied_ns` 闭环推进）**：
  - `RemoteClient` 与 `shm_gui_owner` 均只保留最新待发 `fields`，因此本机绝不盲目按固定高频超前积分。
  - 将候选生成与执行进度提交分开。候选、缩步和连续认证均保留上次已提交的状态及后备计划。认证失败时从该状态沿已认证后备减速。
  - 每次只允许一个待确认的五字段命令。以单调时钟计时，遥测确认同一 `control/revision` 已入队且 `applied_ns` 有效后，才按该命令实际对应的段或子步提交参考状态与剩余后备。纯运动学仿真在应用该命令后提交。
  - 入队确认超时或执行进度无法确定时立即失能。仅当通信正常、进度已知且实际状态仍处于已认证运动范围时，才使用相应的停车后备。
- **故障立即失能原则（严格对齐 `AGENTS.md`）**：
  - 仅在通信正常时使用本地停车尾段平滑减速；
  - 一旦判定为**通信故障、租约超时、断链、驱动或模式故障**，严格沿用 `shm_gui_owner` 现有逻辑**立即失能并等待 `disable_ack`，结束会话，不清故障、不自动恢复**（不尝试断链后沿尾段继续运动）。

---

## 4. 分阶段实施步骤与验收标准

### Phase 0：本机环境重建 + 碰撞对几何审计与导入（最先完成）
1. `.gitignore` 增加 `third_party/`。
2. 编写 `scripts/nmpc/setup.sh` 与 `scripts/nmpc/run.sh`，在 `/home/ww/.conda/envs/casbot_calib` 中完成：
   - 在指定 conda 环境中编译并安装当前源码的 Python 绑定 `_core`；
   - 调用现有 `scripts/shm/build.sh --with-mock`，复用本机编译缓存，把宿主机版 `shm_gui_owner / shm_mock_server` 安装到 `build/nmpc/shm-host/`；
   - 检出并编译 `acados v0.6.0` 到 `third_party/acados`，安装 `acados_template`。
3. 编写 `configs/nmpc/dvt2_teleop.yaml`、`python/casbot_calib/nmpc/{model.py,qp_options.py,import_dvt2.py}` 和 `tests/nmpc/test_import_dvt2.py`：
   - 以 `CollisionProfile.pairs` 为准，离线审计准备位与 `home_rad` 下每对原始凸包与外包胶囊间距；
   - 生成 `build/nmpc/dvt2_bare_flange.json`（含 `pair_clearance_m` 清单、胶囊代理及需回退原始凸包的全部碰撞对列表）。

**验收标准**：
- 环境三项验证全部在 `/home/ww/.conda/envs/casbot_calib/bin/python` 下通过：① `load_context()` 正常加载；② `CollisionProfile.check()` 成功查询准备位与 `home_rad` 距离；③ 用 `build/nmpc/shm-host/bin/shm_executor` 成功拉起私有端点 `/casbot_calib_test_*` 的 `SHMMock`、读到有效反馈并干净关闭；
- 移植后 NumPy FK 与 `ArmDynamics.frame_pose(..., 'left_flange_frame'/'right_flange_frame')` 误差 $< 10^{-10}$；
- 准备位和 `home_rad` 下，所有碰撞对均严格满足其 `pair_clearance_m`（且包含结构性近邻的显式审计记录，未新增任何排除对）。

本机运行 Phase 0：

```bash
bash scripts/nmpc/setup.sh
scripts/nmpc/run.sh import
~/.conda/envs/casbot_calib/bin/python -m pytest -q -o addopts='' tests/nmpc/test_import_dvt2.py
```

导出的 `pairs` 顺序与原始碰撞核一致。每行的 `convex_gap_m` 和 `capsule_gap_m` 按准备位、`home_rad` 排列；`modes` 给出各配置的准入间距和选用几何。配置中列出的近邻使用左右镜像的阈值，单位为米。

### Phase 1：14 轴原生 NMPC + 连续碰撞认证离线验证
1. 移植并泛化 `python/casbot_calib/nmpc/{build.py,runtime.py}` 与 `cpp/nmpc/{CMakeLists.txt,runtime.cpp,runtime_api.h}`（支持 14 轴、`arm=-1` 固定障碍、per-pair 阈值、7 轴零空间正则、奇异惩罚与停车后备）。
2. 在 Python 侧使用碰撞核的运动半径，并对外包胶囊补足半径。整段下界不足时，由 C++ 对相关碰撞对逐段认证，必要时先分五段、再递归二分。无法认证的候选进入缩步或停车后备。
3. 编写 `tests/nmpc/test_nmpc_core.py`（C++ FK 一致性、梯度有限差分 $< 10^{-6}$、双臂互撞与躯干逼近探针、子步通过而连续认证失败时保持原后备、连续碰撞认证通过、单周期总耗时统计）。

**验收标准**：
- `scripts/nmpc/run.sh build` 产出 `build/nmpc/dvt2_bare_flange/libcasbot_nmpc_runtime.so`；
- 所有碰撞探针不仅通过子步检查，还 100% 通过连续碰撞认证（含停车尾段）；
- 报告完整控制周期耗时（含连续认证），确认在设定 `dt` 预算内（若超 20 ms 则按实测调大 `dt` 并同步更新配置）。

### Phase 2：纯运动学 MuJoCo Mocap 交互仿真（可拖 mocap 看效果）
1. 实现 `python/casbot_calib/nmpc/{mocap_viewer.py,bridge.py,cli.py}`（`sim` 纯运动学模式）。
2. 启动后自动置于准备位并对齐左右 `mocap` 手柄，终端实时显示最小间距、连续认证耗时、求解耗时与控制状态。

**验收标准**：
- `scripts/nmpc/run.sh sim` 打开本机 MuJoCo 窗口，拖动 `left_goal / right_goal` 双臂丝滑跟随、肘部不乱漂；
- 将双手拖至交叉重合或拖入躯干/头部区域时，机械臂沿安全边界平滑避让/停稳，连续碰撞认证始终通过；
- `R` 复位、`Space` 暂停/恢复、`C` 切换胶囊显示、关窗或 `Ctrl+C` 干净退出。

本机运行 Phase 1–2：

```bash
scripts/nmpc/run.sh build
scripts/nmpc/run.sh sim
```

启动后处于 `ready`。双击蓝色左手柄或橙色右手柄，按住 Ctrl 并拖动右键平移，拖动左键旋转。可先试单手小幅移动，再让双手交叉或向胸前靠近。终端的“间距”是所有碰撞对中的最小几何间距，各对有各自的准入裕度。`damped` 表示候选已缩步，`backup` 表示正在沿上次认证的路径减速。显示的周期计算耗时包括目标滤波、求解、连续认证及仿真状态更新。

在本机保留仿真窗口，用以下命令复测控制耗时。三组目标各运行约 4.5 s，输出各自的 P95、最大耗时、超过控制周期的次数及最小间距裕量。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ~/.conda/envs/casbot_calib/bin/python -m pytest -q -s -o addopts='' tests/nmpc
```

### Phase 3：标定动力学前馈 + SHM 物理 Mock 交互仿真（带动力学可拖 mocap）
1. 实现 `python/casbot_calib/nmpc/dynamics_ff.py` 与 `tests/nmpc/test_dynamics_ff.py`。
2. 在 `bridge.py` 接入 `RemoteClient`，实现基于 `revision / applied_ns` 的推进闸门、追踪偏差保护与故障立即失能；`cli.py` 提供 `mock`（默认指向 `build/nmpc/shm-host/bin/shm_executor`）与 `teleop` 子命令及 `--no-ff` 开关。
3. 编写 `tests/nmpc/test_shm_mock_loop.py`，覆盖：
   - 自动化目标轨迹 + 碰撞逼近全链路连续认证闭环；
   - 低刚度 PD 下开启前馈 vs `--no-ff` 的跟踪误差对比；
   - 注入发送/反馈延迟，验证已发送但入队确认超时后立即失能；核对被拒候选和未确认子步均未移动后备起点；
   - 模拟超时/断链验证立即失能并拿到 `disable_ack=true`。

**验收标准**：
- `scripts/nmpc/run.sh mock` 打开带真实物理动力学的 SHM Mock + MuJoCo mocap 交互窗口，拖动流畅，开启前馈后低刚度 PD 跟踪误差显著小于 `--no-ff`；
- `tests/nmpc/` 全部通过，且原有 Humble 主回归测试不受任何影响：
  - `bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q < /dev/null`
  - `bash scripts/humble/container.sh bash -c 'cmake --build --preset humble && ctest --preset humble' < /dev/null`

**本机人工验收**

```bash
scripts/nmpc/run.sh mock
```

物理 Mock 从准备位启动，窗口显示实测关节反馈。操作与 Phase 2 相同。先向前平移单手约 5 cm，再将两手向胸前和交叉方向拖动；按空格观察减速停车。终端显示实测最小间距裕量、最大关节跟踪误差和入队确认耗时。按 Esc 结束并等待失能。

本机缺少默认动力学交付时使用名义模型。可用 `mock --delivery <交付目录>` 载入标定模型，用 `mock --no-ff` 对照相同手势。每次会话在 `artifacts/nmpc/` 新建目录，`owner/native/result.json` 记录退出原因和失能应答。物理闭环的参考速度、关节限位预留及碰撞间距预留见 `configs/nmpc/dvt2_teleop.yaml` 的 `shm` 段。持续驱动故障时，记录中会保留未收到就绪应答的状态。

### Phase 4：真机联调（仅在用户明确要求上机后执行）
- 先直接复用现有 Orin `shm_gui_owner` 运行 `scripts/nmpc/run.sh teleop --host nvidia@172.16.0.10 --delivery <标定交付目录>`（低速、按碰撞对核定后的真机 `pair_clearance_m`、低刚度 PD），按现场安全规范确认外部支撑/继电器后，先单臂小幅拖动再双臂联动。
- 不预设额外 Orin 协议开发：仅在真机实测确认存在可感知阶梯感或需要无工装站立保持时，再单独立项评估 Orin 端插值或 `non_arm_hold` 扩展。
