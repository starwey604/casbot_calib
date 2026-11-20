# 无工装辅助站立模式（`--hold-body`）实现计划

面向工装到货前的动力学标定过渡场景：在外部支撑/吊装具备独立承重能力的前提下，机器人双脚平贴地面、躯干直立，采集期间以启动时电机位置为目标对全部非双臂关节（双腿 12 轴 + 头 2 轴 + 腰 1 轴）进行 PD 保持，抑制手臂运动引起的机身晃动与扭转。

本文档供执行 Agent 直接实施。原则：**少加阶段、少加门禁、不破坏既有默认路径（不传 `--hold-body` 时行为完全不变）**。

---

## 1. 范围、现场前提与底层事实

### 1.1 生命周期与现场条件（必须明确）
- **外部支撑独立承重与故障继电器兜底**：`--hold-body` 复用现有单次 `reference.json` 生命周期，仅在 `shm_executor` 执行 `phase 2–3` 期间使能出力。停业务接管、主站启动、握手、快照、规划、阶段切换、离线分析及正常失能后，全 29 轴均处于失能松软状态；**故障或异常退出时以 `disable_ack` 为准——未收到失能应答时按驱动仍可能沿用最后命令继续出力处理，须由现场独立继电器切断驱动电源**（命令通道堵塞时软件无法保证送达失能帧）。因此**从接管前至最后一次失能结束后，外部吊装/支撑必须能独立承重并防倾倒**（双脚贴地、吊绳无大松弛余量）。若将来需要全程不断电常驻承重，需另立常驻保持守护进程，不在本计划范围。
- **姿态对准现有模型**：DVT2 碰撞与动力学模型（`derived/.../model.json`）将 15 个非臂关节固定在 `0 rad`，且 H1 假设 `gravity_world = [0, 0, -9.81]`。本计划不改碰撞场景或 `model_context`（`--reanalyse` 与跨会话衔接保持兼容）；现场启动前须在外部支撑下将双腿扶至接近直立（`~0 rad`）、双脚平贴地面、腰（ID28）和头（ID26–27）归正、躯干竖直，避免大幅偏角影响双臂碰撞裕度与 H1 重力方向。
- **零新增阶段 / 零新增验收门禁**：不改 `stage.py`、`session.py`、H1/H2/H3 辨识与防退化门禁；不改 `gui_runtime.cpp` 与 `trical_runtime.cpp`（`field_common.hpp` 保留原 14 轴 `ArmMask` 重载）。

### 1.2 SHM 协议事实（来自 `artifacts/jetson_shm_recon_20261113/`）
- 非双臂 15 轴槽位：左腿 `0..5`（`4, 5` 为并联脚踝电机）、右腿 `6..11`（`10, 11` 为并联脚踝电机）、头 `26..27`、腰 `28`。
- 厂家运控对髋/膝/头/腰（`0..3, 6..9, 26..28`）发 **MIT 模式（`mode = -1`）**，刻度 `kp_raw = trunc(kp * 10)`、`kd_raw = trunc(kd * 10)`；对并联脚踝 4 电机（`4, 5, 10, 11`）经 `Space4Bar::set_s2p_inout` 变换后发 **CST 力矩模式（`mode = 10`）**，刻度 `torque_raw = trunc(torque_nm * 100)`。
- 在电机原始编码器空间以启动时刻 `state.joints[slot].position` 为目标做定点 PD 保持时，SHM 原始 `position / velocity / torque` 共用同一方向符号，MIT 与 CST 闭环均自动消去方向符号与固定零偏，无需移植 `Space4Bar` 或依赖未验证的腿/腰零偏表。注意定点 PD 保持在承重时允许存在静态位置误差。

---

## 2. 配置、Schema 与 CLI 入口

### 2.1 `configs/robot/dvt2.yaml`

新增可选块 `non_arm_hold`（默认 `enabled: false`；`--hold-body` 覆盖为 `true`）：
- 髋/膝/头/腰（`0..3, 6..9, 26..28`）：采用厂家 `motion_params.yaml` 中 `ready_mode` 的 MIT 增益作为初值。
- 并联脚踝 4 电机（`4, 5, 10, 11`）：厂家 `ready_mode` 中的 `120 / 80` 为关节空间参数（写 SHM 前经 `Space4Bar` 变换），不能直接等同于电机空间等效刚度或承重能力；下方 `kp = 120.0, kd = 3.0` 标明为**待验证的电机空间试验参数**，`torque_limit_nm = 30.0` 为**拟用命令限幅**，须在外部承重下先跑低增益冒烟核对。若后续实测确认脚踝固件支持 MIT，只需改配置为 `mode: mit`。

```yaml
non_arm_hold:
  enabled: false
  ramp_s: 1.0
  cst_comm_timeout_ms: 20
  slots:
    # 左腿髋/膝（MIT，参考厂家 ready_mode）与并联脚踝（CST 电机空间试验参数）
    0:  {mode: mit,    kp: 400.0, kd: 20.0}
    1:  {mode: mit,    kp: 500.0, kd: 19.0}
    2:  {mode: mit,    kp: 380.0, kd: 5.0}
    3:  {mode: mit,    kp: 440.0, kd: 17.0}
    4:  {mode: cst_pd, kp: 120.0, kd: 3.0, torque_limit_nm: 30.0}
    5:  {mode: cst_pd, kp: 120.0, kd: 3.0, torque_limit_nm: 30.0}
    # 右腿髋/膝（MIT）与并联脚踝（CST 电机空间试验参数）
    6:  {mode: mit,    kp: 400.0, kd: 20.0}
    7:  {mode: mit,    kp: 500.0, kd: 19.0}
    8:  {mode: mit,    kp: 380.0, kd: 5.0}
    9:  {mode: mit,    kp: 440.0, kd: 17.0}
    10: {mode: cst_pd, kp: 120.0, kd: 3.0, torque_limit_nm: 30.0}
    11: {mode: cst_pd, kp: 120.0, kd: 3.0, torque_limit_nm: 30.0}
    # 头 pitch/yaw 与腰 yaw（MIT）
    26: {mode: mit,    kp: 30.0,  kd: 2.0}
    27: {mode: mit,    kp: 30.0,  kd: 2.0}
    28: {mode: mit,    kp: 500.0, kd: 5.0}
```

### 2.2 Reference Schema（防旧执行器静默丢弃）

- 普通参考（未启用 `non_arm_hold`）：继续写 `schema: "casbot_shm_reference_v1"`，不含 `non_arm_hold` 字段，完全兼容旧版执行器。
- 启用身体保持的参考：写 `schema: "casbot_shm_reference_v2"`，必须包含 `non_arm_hold` 与 `observed_velocity_limit_rad_s`。旧版 `shm_executor` 遇到 `v2` 会在接管 SHM 前直接报 `Unsupported SHM reference schema`，避免静默退化为腿腰失能。

### 2.3 CLI 入口

- `calib/dynamics.sh`、`calib/dynamics_h1.sh`、`calib/dynamics_h2.sh`、`calib/dynamics_h3.sh`（经 `pipeline/runner.py`）：新增 `--hold-body`。
- `calib/dynamics_verify.sh`（经 `pipeline/verify_dynamics.py`）：新增 `--hold-body`。
- `calib/dev.sh`（经 `shm_fieldwork.py`）：新增 `bash calib/dev.sh hold-body [--duration 10] [--scale 0.5] [--mock] [--execute]`：
  - 复用快照与 `export_motion()` 生成双臂原地静止 + `non_arm_hold`（按 `--scale` 缩放 `kp/kd`）的 `v2` `reference.json`，显式写入 `observed_velocity_limit_rad_s`（取机器人配置默认 `1.5`），直接交由 `shm_executor --execute --reference` 运行，不写第二套执行逻辑。

---

## 3. C++ 执行器改动（`cpp/src/shm/` 与 `cpp/tools/`）

### 3.1 `cpp/src/shm/reference.hpp` & `cpp/src/shm/reference.cpp`

1. 新增保持配置结构并在 `Reference` 暴露 `const NonArmHoldConfig& non_arm_hold() const`：
   ```cpp
   enum class HoldMode : std::int16_t { mit = -1, cst_pd = 10 };
   struct NonArmHoldSlot {
     int slot = -1;
     HoldMode mode = HoldMode::mit;
     double kp = 0.0, kd = 0.0, torque_limit_nm = 0.0;
   };
   struct NonArmHoldConfig {
     double ramp_s = 0.0;
     std::int64_t cst_comm_timeout_ns = 20000000LL;  // 20 ms（反馈/写命令双向断流上限）
     std::vector<NonArmHoldSlot> slots;
     bool enabled() const { return !slots.empty(); }
     bool has_cst_pd() const;
   };
   ```
2. `Reference::load()`：
   - 接受 `casbot_shm_reference_v1`、`casbot_shm_reference_v2`、`casbot_shm_fields_v1`。
   - 规则：`v1` 禁止包含 `non_arm_hold`；`v2` 必须包含有效 `non_arm_hold`（`slots` 非空，底层解析器允许 `0..11, 26..28` 的任意非空子集以支持局部协议测试）且必须读取并校验 `observed_velocity_limit_rad_s`（`(0, 1.5]`），不受 `purpose` 白名单限制（确保 `calib/dev.sh hold-body` 冒烟同样启用过速保护）。
   - 校验项：
     - `ramp_s`：`0 <= ramp_s <= 10.0`；
     - `cst_comm_timeout_ms`（可选，默认 `20`）：`2 <= ms <= 200`；
     - `slot`：仅允许 `0..11` 或 `26..28`，禁止双臂 `12..25`、保留槽 `29..34` 和重复槽；
     - `mode`：仅允许 `"mit"` 或 `"cst_pd"`；
     - `kp ∈ [0, 1000]`、`kd ∈ [0, 50]`（`mit` 校验 `checked_truncate<std::int16_t>(kp * 10)` 与 `kd * 10`）；
     - `cst_pd` 必填 `torque_limit_nm ∈ (0, 100]`（校验 `checked_truncate<std::int16_t>(torque_limit_nm * 100)`）。

### 3.2 `cpp/src/shm/field_common.hpp`

保留现有 14 轴 `ArmMask` 重载给 GUI 和 TriCal，新增 29 轴通用重载：
```cpp
using JointMask = std::array<bool, 29>;
using JointModes = std::array<std::int16_t, 29>;
```
- `drive_ack(s, word, selected29, modes29, check_mode)`：
  - `selected29[i] == true`：要求 `drive_state_is(s.joints[i].status_word, word)`，且 `check_mode` 时 `s.joints[i].mode == modes29[i]`；
  - `selected29[i] == false`：要求 `!(s.joints[i].status_word & 4)`。
- `drive_phase_next(phase, s, selected29, modes29)`：沿用 `phase 0..4`（全失能 `0x21` → 切目标 `mode` → 使能 `0x27` → 执行 → 全失能 `0x21`），不新增 phase。

### 3.3 `cpp/src/shm/field_runtime.cpp`

1. **29 轴掩码与 `settings.txt`**：双臂 `12..25` 按原 `selected(j, o)` 填 `-1`；启用 `non_arm_hold` 时填入各保持槽的 `selected29` 与 `modes29`，`settings.txt` 如实记录 `non_arm=hold(...)`。
2. **preflight 静止与防漂移复核**：若启用 `non_arm_hold`，在两次 `fresh()` 之间同步检查每个保持槽 `std::abs(state2.joints[slot].position - state1.joints[slot].position) <= 5000`（`0.005 rad`）且 `std::abs(state2.joints[slot].velocity) * 1e-6 < start_velocity_limit`（复用 `field_defaults.hpp` 的 `0.2 rad/s`），未通过则零命令写入退出。
3. **冻结起点与 1kHz 控制律**：
   - 进入 `run()` 时冻结 `hold_pos_raw[slot] = state.joints[slot].position`（全程不随反馈更新）。
   - `phase == 2`（使能瞬间）：保持槽以零增益、零力矩使能（`kp = 0, kd = 0, torque = 0, velocity = 0`；`mit` 填 `position = hold_pos_raw[slot]`，`cst_pd` 填 `position = 0`）。
   - `phase == 3`（运行期）：
     - 29 轴统一 `drive_ack(state, 0x27, selected29, modes29, true)` 监控。
     - **CST 双向断流保护（反馈 + 写命令）**：若 `non_arm_hold().has_cst_pd()`：
       - `now - last_receive > non_arm_hold().cst_comm_timeout_ns` → 立即抛出 `cst_feedback_stale`（防丢反馈时用旧误差发力）；
       - `now - last_send > non_arm_hold().cst_comm_timeout_ns` → 立即抛出 `cst_command_stale`（防命令队列堵塞时主站持续沿用最后一条非零力矩缓冲）；
       - 二者均走既有 `catch` 全轴失能路径，并如实记录 `disable_ack`（若命令通道持续堵塞则 `disable_ack = 0`，现场以独立继电器断电）。
     - 计算五次平滑斜坡 `scale`（`u = clamp(elapsed / ramp_s, 0, 1)`，`scale = u^3(10 - 15u + 6u^2)`；`ramp_s <= 0` 时为 `1.0`）：
       - `HoldMode::mit`：`position = hold_pos_raw[slot]`, `velocity = 0`, `torque = 0`, `kp = trunc(cfg.kp * scale * 10)`, `kd = trunc(cfg.kd * scale * 10)`。
       - `HoldMode::cst_pd`：`position = 0`, `velocity = 0`, `kp = 0`, `kd = 0`，
         ```cpp
         const double pos_err = static_cast<double>(hold_pos_raw[slot] - state.joints[slot].position) * 1e-6;
         const double vel = static_cast<double>(state.joints[slot].velocity) * 1e-6;
         const double tau = std::clamp(scale * (cfg.kp * pos_err - cfg.kd * vel),
                                       -cfg.torque_limit_nm, cfg.torque_limit_nm);
         command.joints[slot].torque = checked_truncate<std::int16_t>(tau * 100.0);
         ```
     - **保持槽过速保护**：收到新状态且 `observed_velocity_limit() > 0` 时，检查各保持槽 `std::abs(state.joints[slot].velocity) * 1e-6 > limit`，超限记录 `observed_limits.csv` 并抛出 `observed_velocity_limit_slot_<slot>` 触发全轴受控失能。
   - `phase == 4` 及 `catch` 异常退出：覆盖全部 `0..28` 轴写 `control_word = 6, velocity = 0, torque = 0, kp = 0, kd = 0`，等待全 29 轴 `drive_ack(state, 0x21, selected29, modes29, false)` 应答（若存在锁存故障驱动则超时后如实记录 `disable_ack = 0`，不清故障）。
4. **记录**：`commands.bin`、`feedback.bin` 与 `feedback.csv` 已记录完整 35 槽，无需改格式。

### 3.4 `cpp/tools/shm_mock_server.cpp`

面向私有 `/casbot_calib_test_*` 测试做两项最小增强：
- 非臂槽（`0..11, 26..28`）补齐与双臂一致的协议检查：使能态改 `mode` 报 `0x1001`、非法 `control_word` 报 `0x1002`、仅 `control_word == 15 && (mode == -1 || mode == 10)` 且无错误时置 `0x237`（故障保持锁存 `0x238`）。
- 增加可选测试注入字段（默认关闭，不改现有 Mock 配置）：
  - `non_arm_test_motion`（按槽注入已知 `pos_raw_offset` / `vel_raw`，用于验证 `cst_pd` 输出力矩的方向、斜坡与限幅，以及保持轴过速触发）；
  - `stop_status_after_commands`（收到指定条数命令后停发状态但继续消费命令，验证 `cst_feedback_stale`）；
  - `stop_command_consume_after`（消费指定条数命令后暂停消费命令但继续发状态，验证 `cst_command_stale`）。

---

## 4. Python 改动清单

- **`python/casbot_calib/pipeline/robot.py`**：
  - `load_robot_config(source=DEFAULT_ROBOT_CONFIG, *, hold_body=False)` 解析并校验 `non_arm_hold`；`hold_body=True` 时置 `enabled=True`。
  - **15 轴完整性校验**：凡启用 `non_arm_hold`（无论是 `--hold-body` 还是 YAML 开启），强制要求 `slots` 集合**恰好等于全部 15 个非臂槽 `{0..11, 26..28}`**，缺任一腿/踝/头/腰槽直接报错，避免漏配单侧关节上机。
- **`python/casbot_calib/shm_ready.py`**：
  - `preparation(start, engine, maximum_velocity_rad_s=1.5, *, start_hold_s=0.5)` 将首段 `ready_start_hold` 时长参数化（默认仍为 `0.5`，保持原行为）。
- **`python/casbot_calib/shm_reference.py`**：
  - `export_motion(..., non_arm_hold=None)`：传入启用状态的 `non_arm_hold` 时写 `schema = "casbot_shm_reference_v2"` 与 `non_arm_hold` 字段；未传入或未启用时保持 `v1` 不变。
- **`python/casbot_calib/stages/dynamics/common.py`**：
  - 新增小型辅助函数 `ensure_leading_hold(motion, min_hold_s)`：计算 `motion` 开头连续静止时长，若小于 `min_hold_s` 则用 `prepend()` 补齐一段静止保持并返回 `(motion, extra_offset_ns)`。
  - `compose_stage_reference()`：
    - `ready_pose=True` 且启用保持时，向 `preparation()` 传 `start_hold_s = max(0.5, ramp_s)`；
    - `ready_pose=False` 且启用保持时，对 `formal_motion` 调 `ensure_leading_hold(formal_motion, ramp_s)` 并将 `extra_offset_ns` 计入返回的 `offset_ns`（H2 的 `sweeps`、H3 的 `blocks` 与 `markers` 均自动消费 `offset_ns`，无需改动 `h2_friction.py` / `h3_inertia.py`）；
    - 调用 `export_motion()` 时透传 `non_arm_hold`。
- **`python/casbot_calib/stages/dynamics/h1_gravity.py`**：
  - `ready_pose=True` 时，第 1407 行 `preparation()` 传入 `start_hold_s = max(0.5, ramp_s)`；
  - `ready_pose=False` 且启用保持时，对 `motion` 调 `ensure_leading_hold(motion, ramp_s)` 并将 `extra_offset_ns` 平移到 `trials` 与 `markers`；
  - 第 1481 行 `export_motion()` 同样透传 `non_arm_hold`（避免 H1 独立导出路径漏掉身体保持）。
- **`python/casbot_calib/pipeline/native.py`**：
  - `read_start_state()`：启用 `non_arm_hold` 时同步检查每个保持槽的起始速度 `abs(vel_raw) * 1e-6 < threshold`。
- **`python/casbot_calib/pipeline/runner.py` & `pipeline/verify_dynamics.py`**：
  - 新增 `--hold-body` 并透传给 `load_robot_config(..., hold_body=args.hold_body)`。
- **`python/casbot_calib/shm_fieldwork.py` & `calib/dev.sh`**：
  - 新增 `hold-body` 站立冒烟子命令（复用 `load_robot_config(..., hold_body=True)`，生成带 `observed_velocity_limit_rad_s` 的 15 槽 `v2` 参考并执行）。
- **`docs/dynamics.md`**：
  - 补充 `--hold-body` 用法、外部支撑独立承重条件、按 `disable_ack` 配合独立继电器处置故障、启动前直立归正要求及先跑 `calib/dev.sh hold-body` 的操作说明。

---

## 5. 验证与构建部署

### 5.1 自动化测试

1. **C++ 单元测试（`cpp/tests/shm_reference_tests.cpp`）**：
   - `v2` 合法加载（含混合 `mit`/`cst_pd`、`cst_comm_timeout_ms`、`observed_velocity_limit_rad_s`，以及合法非空子集）；
   - 拒绝项：`v1` 带 `non_arm_hold`、`v2` 缺 `non_arm_hold` 或缺 `observed_velocity_limit_rad_s`、非法槽位（`12..25`、`29..34`、重复槽）、非法模式、`cst_pd` 缺 `torque_limit_nm`、超范围参数。
2. **Python 规划与私有 Mock 端到端测试（`tests/test_shm_fieldwork.py` / `tests/test_pipeline.py` / `tests/test_wave2a_h1.py`）**：
   - **15 轴完整性与导出检查**：`load_robot_config(..., hold_body=True)` 缺任一槽直接报错；`dynamics_h1 --hold-body` 与 `compose_stage_reference` 生成的 `reference.json` 均为 `v2` 且包含全部 15 个非臂槽；未加 `--hold-body` 时为 `v1` 且无 `non_arm_hold`。
   - **斜坡与首次动作对齐（含开/关准备位）**：
     - `ready_pose: true` 且起点偏离准备位：断言双臂首个运动时刻不早于 `ramp_s`；
     - `ready_pose: false`（如 H2/H3 配方 `initial_hold_s = 0.5 s < ramp_s = 1.0 s`）：断言实际首段静止延长至不短于 `ramp_s`，且 `sweeps`/`blocks`/`markers` 时间戳同步平移正确。
   - **MIT 与 CST 分开断言**：
     - MIT 槽：目标 `position` 固定为起点原始值、`kp/kd` 按五次曲线从 0 拉满；
     - CST 槽：命令 `position/kp/kd` 恒为 0、`mode == 10`；通过 Mock 注入已知非臂位置偏差与速度，断言输出 `torque` 符号、斜坡与 `±torque_limit_nm` 限幅正确。
   - **站立冒烟过速保护**：直接运行 `calib/dev.sh hold-body` 生成的参考，验证其 `observed_velocity_limit_rad_s > 0` 且注入保持槽超速时触发 `observed_velocity_limit_slot_*` 并全轴失能。
   - **CST 双向断流故障**：
     - 反馈断流（`stop_status_after_commands`）：约 `cst_comm_timeout_ms` 级触发 `cst_feedback_stale` 并全轴发失能；
     - 命令断流（`stop_command_consume_after`，状态持续到达但命令队列满/停消费）：约 `cst_comm_timeout_ms` 级触发 `cst_command_stale`、终止参考并尝试全轴失能，如实记录 `disable_ack`。
   - **正常/中断 vs 锁存故障失能验收分开**：
     - 正常结束与 `SIGINT`：全 29 轴发送 `control_word = 6`、零力矩且 `disable_ack = 1`；
     - 驱动锁存故障（Mock 保持 `0x238`）：全 29 轴发送 `control_word = 6` 与零力矩、保留原故障（不清故障）、会话失败，按实测状态如实记录 `disable_ack`。
   - **兼容回归**：对 `--hold-body` 会话跑 `--reanalyse` 正常通过。

### 5.2 运行命令（注意显式覆盖默认排除 `slow` 的 `addopts`）

```bash
# C++ 构建与 CTest
bash scripts/humble/container.sh bash -c 'cmake --build --preset humble && ctest --preset humble' < /dev/null
# Python 快速集 + 新增端到端 / slow 回归（明确运行新增用例，避免被默认 addopts 跳过）
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q < /dev/null
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -o addopts='' -q tests/test_shm_fieldwork.py tests/test_pipeline.py tests/test_wave2a_h1.py < /dev/null
```

### 5.3 Orin 部署与实机顺序

```bash
# 交叉编译并部署到 Orin（或 Orin 原生执行 bash calib/setup.sh）
bash scripts/shm/cross_build_orin.sh
bash scripts/shm/deploy_git.sh --with-prebuilt
# Orin 侧安装
bash calib/setup.sh --prebuilt
```

实机顺序（全程外部支撑具备独立承重能力；启动前直立贴地、腰头归正至接近 `0 rad`、脚底防滑）：
1. 半增益站立冒烟 10 秒，核对 15 轴（尤其脚踝 `cst_pd`）无报错、无啸叫、失能平稳：
   `bash calib/dev.sh hold-body --scale 0.5 --duration 10 --execute`
2. 全增益站立冒烟 10 秒复核：
   `bash calib/dev.sh hold-body --scale 1.0 --duration 10 --execute`
3. 执行动力学标定：
   `bash calib/dynamics_h1.sh --mode full --robot DVT2_01 --hold-body`
   `bash calib/dynamics.sh --mode full --robot DVT2_01 --hold-body`
