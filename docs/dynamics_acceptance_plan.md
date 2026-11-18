# 双臂动力学标定实机验收计划（乐队演奏轨迹闭环跟踪）

本文档定义双臂动力学标定（`H1–H3`）完成后的实机闭环验收规格，供编码 Agent 直接按规范实现。

---

## 0. 背景、范围与入口

### 0.1 现场前提与范围边界
1. **装配与场地条件**：拆双手、**裸法兰**运行，双腿/腰/头失能并由固定机架支撑；双臂活动范围内**无琴架、鼓架、谱架或棒槌工装等外部障碍物**（当前碰撞模型 `DVT2 bare-flange full robot and floor` 仅覆盖裸法兰机器人本体与地面）。
2. **待解决的问题**：[configs/robot/dvt2.yaml](../configs/robot/dvt2.yaml) 现有增益（`kp: [65, 65, 30, 30, 30, 20, 20] × 2`, `kd: [5, 5, 4, 2, 2, 2, 2] × 2`）是在无前馈时为克服重力与静摩擦调高的，实机存在轻微抖动。本验收回答两个具体工程问题：
   - 交付动力学包在闭环前馈控制下，相对无前馈、CAD 名义模型及 H2 阶段模型能否改善法兰与关节跟踪精度；
   - 降低 PD 增益并挂载完整标定前馈后，能否在降低高频速度抖动的同时保持不劣于原硬增益无前馈的跟踪精度。
3. **非本验收范围**：不测外部配重物理力矩尺度（C08），不做零刚度悬停专项；验收结果反映裸法兰轨迹前馈控制改善情况，乐器实装上线时的棒槌/手指尖端落点与节拍绝对容差由下游应用另行标定。

### 0.2 验收设计（不新增标定阶段，零侵入现有流水线）
**不注册新的 `Stage`、不修改 `stage.py` / `session.py` / `runner.py`、不新建阶段 YAML 配置文件**。新增独立模块 `python/casbot_calib/pipeline/verify_dynamics.py`，直接挂载在已完成全流程标定的会话目录下读写 `<calib_session>/verify/`，完全复用现有接管、起点快照、C3 参考合成、原生 `shm_executor` 执行与反馈复核原语。

### 0.3 命令入口

```bash
cd /home/nvidia/casbot_calib_shm

# 实机执行闭环验收（自动选取最近一次通过完整 H1+H2+H3 的同机器标定会话，也可传会话路径）
bash calib/dynamics_verify.sh --robot DVT2_01
bash calib/dynamics_verify.sh artifacts/calib/DVT2_01/20261117_103000

# 仅离线生成完整五组参考轨迹并执行连续碰撞 / URDF 限位 / 速度包络校验，不连机器人、不改会话软链
bash calib/dynamics_verify.sh --plan-only

# 免动机器人，对最近一次（或指定目录）已完成采集的验收记录重算指标与报告
bash calib/dynamics_verify.sh --reanalyse
bash calib/dynamics_verify.sh --reanalyse artifacts/calib/DVT2_01/20261117_103000

# 本机私有 1kHz 物理 SHM Mock 闭环演练（只连 /casbot_calib_test_*）
bash calib/dev.sh dynamics-verify-mock
```

---

## 1. 基准演奏轨迹筛选与离线资产固化

### 1.1 原始数据特征与碰撞裕度事实
`references/datas/` 包含 5 首电子琴（`*keyboard1*.data`）与 5 首架子鼓（`*drum1*.data`）100Hz 轨迹（共 `78 MB`，不同步至 Orin）：
- **电子琴**：14 轴角度无越界，`waist_yaw` 全 `0`，全轴差分速度 P99 为 `1.08 rad/s`，落在 `shm_executor` 的 `1.5 rad/s` 上限内，可直接取 `1.0×` 原速片段。
- **架子鼓**：4 首 `waist_yaw == 0`，肘/腕高频折返原始差分峰值达 `14~19 rad/s`，不能按原始离散差分峰值生硬拉伸（例如 `15.46 rad/s` 窗口直接均匀拉伸至 `1.35 rad/s` 会膨胀至 `25 s` 以上），**必须先平滑滤波并构建最终七次样条后，按连续样条速度包络筛选与定标时长**。
- **碰撞筛选距离下界**：当前碰撞模型中 `robot_waist_yaw_link_0` 与 `robot_right_shoulder_roll_link_0` 根部固有间距约为 `11.9 mm`（相对 `clearance_m = 0.010 m` 余量约 `+1.9 mm`），预筛候选窗口时统一要求全碰撞对距离 `>= 0.0115 m`。

### 1.2 先按最终样条定标时长，再固化资产
提供离线提取脚本 `scripts/extract_band_clips.py`，生成轻量随包资产 `python/casbot_calib/assets/band_acceptance_clips.json`（`< 60 KB`，随 Git 与 Python wheel 自动分发），严格按以下流程构建并做全链路离线验证后才写出文件：

1. **电子琴片段 `keyboard_clip`（原速，目标约 `7 s`）**：
   - 在 `waist_yaw == 0` 的 `*keyboard1*.data` 中枚举 `7.0 s` 候选窗（采样间距 `>= 0.0115 m`）。
   - 经 `8 Hz` 零相位低通滤波、首尾 `0.6 s` 五次光滑窗收束至静止端点（端点 `dq = ddq = jerk = 0`）、按 `0.10 s` 步长重采样构造七次 Hermite `Motion` 段。
   - 保留满足连续 `gate_motion(..., clearance_m=0.01)` 通过且 `derivative_bounds(curve, 1).max() <= 1.25 rad/s` 的候选窗，取左右双臂速度 RMS 综合得分 $\min(v_{\text{rms,L}}, v_{\text{rms,R}}) + 0.5(v_{\text{rms,L}} + v_{\text{rms,R}})$ 最高者。
2. **架子鼓片段 `drum_clip`（平滑样条时间缩放，目标约 `6 ~ 9 s`）**：
   - 在 `waist_yaw == 0` 的 `*drum1*.data` 中枚举原始时长 `2.0 ~ 3.5 s` 候选窗（采样间距 `>= 0.0115 m`）。
   - 先对原始窗口做零相位低通平滑与首尾收束，再用二分法求最小时间拉伸系数 $s \ge 1$，使重采样（步长约 `0.08 s`）构造的最终七次 Hermite `Motion` 段满足 `derivative_bounds(curve, 1).max() <= 1.32 rad/s` 且连续 `gate_motion` 通过。
   - **剔除拉伸后时长 `> 9.5 s` 的极端孤立尖峰窗口**，在剩余合格窗口中选取肘/腕（ID 15–18, 22–25）连续样条角加速度 RMS 最大者。
3. **转场与完整参考联合预检（禁止硬编码死转场时长）**：
   - 单轮循环 `Cycle`：`q_anchor (= keyboard_start) -> keyboard_clip -> 转场至 drum_start -> drum_clip -> 转场回 q_anchor`。
   - 所有转场（含准备位 `ready_position()` 到 `q_anchor`、琴鼓间转场、回 `q_anchor`、末段回 `ready_position()`）一律调用现有 [`staged_motion._move`](../python/casbot_calib/staged_motion.py) 按运动学包络（`speed_rad_s: 1.25, acceleration_rad_s2: 2.5, jerk_rad_s3: 10.0`，`minimum_duration: 1.5 s`）**自动计算满足连续导数上界的实际转场时长**。
   - 提取脚本最后用整条五组串联参考直接调用 `compose_stage_reference`（校验连续碰撞、URDF 限位与 C++ `1.5 rad/s` Bernstein 速度包络），全部通过后才落盘 `band_acceptance_clips.json`。

---

## 2. 交付包选择规则与单轨五组对照设计

### 2.1 待验收交付包严格筛选与加载（防回退包误入）
`urdf_reload_passed` 仅证明导出一致性，不能防止 H3 验收失败回退至 H2 的交付包被误当成完整 H1–H3 模型。`resolve_verified_delivery(target_session, output_root, robot_id, robot_config)` 规则如下：
1. 候选标定会话须满足：`family == 'dynamics'`、`not plan_only`、`robot_id` 与 `model_context` 匹配，`delivery/calibration_bundle.json` 中 `gravity_world` 一致且 `urdf_reload_passed is True`。
2. **完整性硬条件**：`bundle['selected_calibration_source'] == 'session_h3'`，且 `provenance` 中 `gravity` 来自通过门禁的 `h1`、`friction` 来自通过门禁的 `h2`、`inertia == 'session_h3'`、`rotor_inertia == 'session_h3'`，同时会话内存在通过门禁的上游 H2 产物（`h2/analysis/candidate.urdf` + `h2/analysis/runtime_dynamics.json`，或 `provenance['friction']` 指向的上游会话对应文件）。不满足时直接报错指出缺少完整通过的 H1–H3 交付链（仅 `--plan-only` 无历史会话时允许回退 `nominal` 仅测轨迹几何与限速）。
3. **从真实交付文件加载引擎**：统一使用 `_core.ArmDynamics.from_urdf(...)` 配合 [`dynamic_export._restore_runtime`](../python/casbot_calib/dynamic_export.py#L88-L95) 直接从磁盘文件构建三个对比模型引擎：
   - `nominal_engine`：原始未标定 DVT2 模型（零摩擦，CAD 刚体与名义转子惯量）；
   - `h2_engine`：从上游通过的 `h2/analysis/candidate.urdf` + `h2/analysis/runtime_dynamics.json` 恢复（含 H1 重力 + H2 摩擦 + H2 阶段先验刚体/转子惯量）；
   - `h3_engine`：从最终交付 `delivery/calibrated.urdf` + `delivery/runtime_dynamics.json` 恢复（含 H1 重力 + H2 摩擦 + H3 辨识刚体与转子惯量）。

### 2.2 五组对照定义（统一逆动力学口径）
记 [configs/robot/dvt2.yaml](../configs/robot/dvt2.yaml) 基准增益为 $(K_{p0}, K_{d0})$，柔和档缩放因子固定为 `soft_pd_factor = 0.5`，按 [`control_policy.scale_pd`](../python/casbot_calib/control_policy.py#L13-L15) **线性缩放 `kp`、按平方根缩放 `kd`**：
$$K_p(\alpha) = \alpha K_{p0}, \quad K_d(\alpha) = \sqrt{\alpha}\, K_{d0}$$

单条 `reference.json` 连续执行以下固定 5 组（每组前在 `q_anchor` 静止保持 `group_hold_s = 1.0 s` 完成增益与前馈平滑切换）：

| 顺序 | 组标识 | 增益因子 $\alpha$ | 前馈模型引擎 | 对照回答的具体问题 |
| :---: | :--- | :---: | :--- | :--- |
| 1 | **`G0_stiff`** | `1.0` | 无前馈（`0 Nm`） | 当前硬增益无前馈基线（误差与高频抖动基准）。 |
| 2 | **`G0_soft`** | `0.5` | 无前馈（`0 Nm`） | 单纯降增益消抖后的重力下垂与动态滞后基线。 |
| 3 | **`G1_soft`** | `0.5` | `nominal_engine` 完整逆动力学 | 未标定 CAD 模型的逆动力学补偿水平。 |
| 4 | **`G2_soft`** | `0.5` | `h2_engine` 完整逆动力学 | H1 重力 + H2 摩擦辨识相对 CAD 的收益（与 `G3_soft` 共用同一重力与摩擦）。 |
| 5 | **`G3_soft`** | `0.5` | **`h3_engine` 完整逆动力学** | **H3 连杆与转子惯量辨识相对 `G2_soft` 的净收益，以及最终交付包整体表现**。 |

### 2.3 前馈生成与 `pd_schedule` 实现细节
1. **统一逆动力学前馈与过零平滑**：
   - `G1_soft`、`G2_soft`、`G3_soft` 共用同一个逆动力学求值函数：先将引擎副本中非零摩擦轴的 `smoothing_velocity` 设为 `max(raw_smoothing_velocity, 0.015)` rad/s（`nominal_engine` 摩擦全零不受影响），再直接取 `engine_ff.evaluate_batch(q, dq, ddq)['total']`。
   - [`ArmWorkspace::components`](../cpp/src/arm_dynamics.cpp#L265-L275) 内部 `pinocchio::crba` 已包含 `model.armature`（`rotor_inertia`）一次，**禁止在外部重复叠加 `rotor_inertia * ddq`**。
2. **`pd_schedule` 与前馈表边界契约**：
   - C++ [`Reference::at`](../cpp/src/shm/reference.cpp#L122-L129) 对 `pd_schedule` 相邻节点做五次多项式插值。每组评测运动区间 $[t_{\text{run0},k}, t_{\text{run1},k}]$ 必须成对写入 `(t_run0_k, kp_k, kd_k)` 与 `(t_run1_k, kp_k, kd_k)`，保证运动评测期内增益恒定，仅在组间 `1.0 s` 静止窗内插值切换增益与线性混合前馈。
   - 在 `markers` 与 `plan.json` 中记录每组 `keyboard` 与 `drum` 评测子段的精确起止纳秒时间戳（如 `hv_G0_stiff_kbd_begin` / `hv_G0_stiff_kbd_end`）。

---

## 3. 会话隔离、执行与时间窗有效性

### 3.1 独立验收路径（不污染 `latest` 与 `latest_dynamics`）
- **实机与 `--mock` 执行**：定位目标标定会话 `<calib_session>`，在其下准备 `<calib_session>/verify/`（若已存在旧 `verify/`，归档为 `verify.previous_<monotonic_ns>`），内部结构为 `verify/{plan,capture,analysis,report.md}`。采集完成后仅更新独立软链 `artifacts/calib/latest_verify -> <calib_session>/verify`，**绝不创建新 `Session`，也绝不改动 `latest` 或 `latest_dynamics`**。
- **`--plan-only`**：输出至显式 `--output` 或默认预览目录 `artifacts/calib/verify_plan_preview`（覆写或带时间戳），不触碰任何标定会话目录与软链。
- **`--reanalyse [TARGET]`**：
  - 未传 `TARGET` 时，优先解析 `artifacts/calib/latest_verify`，否则按时间倒序找同 `robot_id` 下含已完成 `verify/capture` 的最新 `<calib_session>/verify`；若传入 `<calib_session>` 或 `<calib_session>/verify` 均可识别。
  - 调用 [`resolve_effective_plan_and_capture(verify_dir / 'plan', verify_dir / 'capture')`](../python/casbot_calib/stages/dynamics/common.py#L306-L318) 获取**当次实际生效的 `plan.json` 与 `reference.json`**（兼容使能前起点漂移自动重规划后的 `replanned/` 目录），归档旧 `verify/analysis` 后重算并更新 `verify/report.md`。

### 3.2 自动安全检查与观测时间窗校验
1. **安全链路自动执行**：准备位前缀 `20/2` 展开（仅跳过展开段碰撞）、正式段 `gate_motion` 连续碰撞（`>= 10 mm`）与 Bernstein 位置/速度包络检查、起点静止（`< 0.2 rad/s`）与零驱动错误码检查、单 SHM owner 锁、`read_verified_stage_capture` 校验 `execution_completed == '1'` / `disable_ack == '1'` 及实测反馈折线 `certify_polyline` 连续防碰撞复核。
2. **时间窗有效性与毫秒时间戳去重契约**：
   - 每个子段窗口 `[t_a, t_b]`（单位秒，相对 `origin_ns`）一律先通过 [`ObservationReader.window((t_a, t_b))`](../python/casbot_calib/shm_observations.py#L305-L350) 提取，自动拦截墙钟跳变模糊区间（`time_ambiguous`）、跨时钟分段与样本缺口。
   - 裁剪至窗口内 `elapsed_s` 落在 `[t_a, t_b]` 的样本；由于主站状态帧时间戳为 `1 ms` 量化，相邻接收帧可能具有相同 `observation_time_ns`。按唯一 `observation_time_ns` 升序分组并对同毫秒帧的 `q, dq, effort_raw` 取算术平均，得到严格递增序列 $t_0 < t_1 < \dots < t_{M-1}$。
   - **窗口缺样检查**：若唯一时刻样本数 $M < \max(50, \lfloor 0.5 \cdot (t_b - t_a) \cdot 1000 \rfloor)$，或相邻唯一时刻最大间隔 $\max \Delta t_k > 0.020\text{ s}$，或首末覆盖缺口 $> 0.010\text{ s}$，标记该窗口无效并写出具体 `invalid_reason`，不静默拼凑指标。

---

## 4. 量化指标、判定准则与推荐配置契约

在严格递增的去重观测序列 $t_k$ 上，由实际执行的 `reference.json` 多项式求得 $q_{\text{ref}}(t_k), \dot{q}_{\text{ref}}(t_k)$，按梯形时间权重 $w_k$（$\sum w_k = t_{M-1} - t_0$）分别统计每组的 **`keyboard`**、**`drum`** 与 **`overall`（两段加权合并）** 指标：

1. **法兰笛卡尔跟踪误差（mm）**：
   - 调用 [`nominal_engine.frame_batch(q, 'left_flange_frame')['poses'][:, :3, 3]`](../cpp/src/arm_bindings.cpp#L153-L161) 与 `'right_flange_frame'`（包含腕部至法兰约 `71.6 mm` 固定平移偏置）：
     $$e_{\text{flange}, A}(t_k) = 10^3 \cdot \| p_A(q(t_k)) - p_A(q_{\text{ref}}(t_k)) \|_2, \quad A \in \{L, R\}$$
   - 输出双臂合并与左右分臂的 **`flange_pos_rmse_mm`**、**`flange_pos_p95_mm`**、**`flange_pos_max_mm`**（及姿态旋转角误差 `flange_rot_rmse_deg`）。
2. **关节位置跟踪误差（deg）**：
   - 输出全 14 轴、近端肩肘 8 轴（ID 12–15, 19–22）、远端腕部 6 轴（ID 16–18, 23–25）及逐轴的 `joint_rmse_deg` 与 `joint_max_deg`。
3. **等效相位滞后 `phase_lag_ms`（网格分辨率 `0.5 ms`）**：
   - 在内缩 `0.3 s` 的 `[t_a + 0.3, t_b - 0.3]` 均匀 `1 ms` 评估网格上，对去重实测序列 $(t_k, q(t_k))$ 做线性插值，在偏移网格 $\delta \in [-80.0, +80.0]\text{ ms}$（步长 `0.5 ms`）上求使 $\sum_{j=0}^{13} \sum_m (q_j(\tau_m + \delta) - q_{\text{ref},j}(\tau_m))^2$ 最小的 $\Delta t_{\text{lag}}$（正值表示实测滞后于参考）。
4. **高频速度残差 `jitter_rms_rad_s`**：
   - 将去重速度残差 $\dot{q}(t_k) - \dot{q}_{\text{ref}}(t_k)$ 线性插值到窗口内均匀 `1000 Hz` 网格，经 4 阶 `15 Hz` 高通零相位 Butterworth 滤波（`sosfiltfilt`），计算全轴与逐轴 RMS（rad/s）。
5. **PD 反馈纠偏力矩 `pd_effort_rmse_nm`**：
   - 按组内恒定增益计算 $\tau_{\text{pd}}(t_k) = K_p(q_{\text{ref}}(t_k) - q(t_k)) + K_d(\dot{q}_{\text{ref}}(t_k) - \dot{q}(t_k))$ 的时间加权均方根（Nm）。

### 4.1 验收判定规则（直接对齐相对门槛，不加放水裕度）
全部 10 个评测子窗（5 组 × 2 乐器）均有效，且 `overall` 指标同时满足以下两条时，判定 `accepted = True`：
1. **同增益前馈提精**：柔和增益下，完整交付模型 `G3_soft` 的 `flange_pos_rmse_mm` 与 `joint_rmse_deg` 均严格小于无前馈 `G0_soft` 且严格小于 CAD 名义前馈 `G1_soft`；
2. **降增益保持跟踪并降低抖动**：`G3_soft` 的 `flange_pos_rmse_mm <= G0_stiff.flange_pos_rmse_mm`，且 `G3_soft` 的 `jitter_rms_rad_s < G0_stiff.jitter_rms_rad_s`。

*(注：`G3_soft` 对 `G2_soft` 的分项误差对比与 `phase_lag_ms` 完整展示在报告表中供分析 H3 惯量贡献与相位改善，不额外叠加容易受 `0.01 Nm` 量化影响的子段硬门禁。)*

### 4.2 推荐运行配置产物契约
- **仅当 `accepted is True` 时**生成 `<calib_session>/verify/analysis/recommended_band_gains.yaml`（未通过时不生成该文件，并在 `report.md` 写明未通过项）。
- 为确保下游复现与本次验收完全一致的过零前馈行为，该 YAML 必须显式包含验收实际使用的平滑下限参数，不得只给原始 `runtime_dynamics.json` 路径：
  - `kp`、`kd`：`G3_soft` 实测通过的 14 轴增益数组；
  - `calibrated_urdf`：指向该会话 `delivery/calibrated.urdf`（附 SHA256 供核对）；
  - `feedforward_smoothing_velocity_floor_rad_s: 0.015`；
  - `rotor_inertia` 与**已将 `smoothing_velocity` 抬升至 `>= 0.015 rad/s` 后的完整 14 轴 `friction` 字典**（即下游直接读取此文件与 `calibrated.urdf` 即可复现 `G3_soft` 前馈）。

---

## 5. 最小文件改动清单与回归要求

不改动 C++ 源码，也不改动 `stage.py`、`session.py`、`runner.py`、`common.py` 及 `h1–h3` 现有文件：

| 操作 | 文件路径 | 职责 |
| :---: | :--- | :--- |
| 新增 | `scripts/extract_band_clips.py` | 平滑重采样、按最终样条定标时长与 `_move` 转场，全参考通过连续碰撞与限速检查后导出片段。 |
| 新增 | `python/casbot_calib/assets/band_acceptance_clips.json` | 固化的 `keyboard_clip` 与 `drum_clip` 节点数据及来源记录。 |
| 新增 | `python/casbot_calib/pipeline/verify_dynamics.py` | 完整实现交付包筛选、五组参考合成、`verify/` 执行与重分析、时间窗去重、五维指标与报告/推荐配置输出。 |
| 新增 | `calib/dynamics_verify.sh` | 薄封装：`exec "$python" -m casbot_calib.pipeline.verify_dynamics "$@"`（`chmod +x`）。 |
| 修改 | `calib/dev.sh` | 增加 `dynamics-verify-mock` 子命令转发至 `verify_dynamics --mock`。 |
| 修改 | `docs/dynamics.md` | 补充实机验收命令、裸法兰无外设场地前提与 `verify/report.md` 查看说明。 |
| 新增 | `tests/test_dynamics_verify.py` | 覆盖以下全部验证点。 |

### 必过测试清单
开始前检查 `git status`，保留工作区已有 `firmware/` 改动。
1. **快速单元测试（纳入默认 pytest）**：
   - **交付包防回退测试**：构造 `selected_calibration_source == 'session_h2'`（H3 被拒回退，但 `urdf_reload_passed == True`）的会话，断言 `resolve_verified_delivery` 拒绝该会话并选取完整通过 `h1+h2+h3` 的会话；
   - **轨迹与增益表契约测试**：验证 `--plan-only` 生成的完整五组 `reference.json` 通过连续碰撞（`>= 10 mm`）与 Bernstein 限位/限速（`< 1.5 rad/s`），验证每组评测窗内 `pd_schedule` 插值增益恒定不变，且 `--plan-only` 不修改 `latest` / `latest_dynamics` 软链；
   - **时间窗与产物契约测试**：构造含重复毫秒时间戳的反馈序列验证指标去重与插值正常；构造跨时钟分段或大缺口序列验证窗口报无效；验证 `accepted == False` 时不生成 `recommended_band_gains.yaml`，`accepted == True` 时生成的 YAML 内 `smoothing_velocity >= 0.015`。
2. **1kHz 物理 SHM Mock 闭环测试（`@pytest.mark.slow`）**：
   - 在私有端点 `/casbot_calib_test_*` 上运行完整五组闭环采集与 `--reanalyse`（包含模拟一次使能前起点轻微漂移触发 `replanned/` 后再 `--reanalyse` 的场景），断言 `G3_soft` 通过验收且重算结果完全一致。
3. **回归命令**：
   ```bash
   bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q < /dev/null
   bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -o addopts='' -q tests/test_dynamics_verify.py tests/test_dynamics_stages.py < /dev/null
   bash scripts/humble/container.sh bash -c 'cmake --build --preset humble && ctest --preset humble' < /dev/null
   ```
