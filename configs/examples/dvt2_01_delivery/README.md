# DVT2_01 动力学标定与实机闭环验收

2026-11-21 在 DVT2_01 上用裸法兰和 1105_03 固定工装完成闭环验收，启用下肢与腰头原地保持（腰部增益 `Kp=800.0`、`Kd=6.5`）。原始会话在 Orin 的 `/home/nvidia/casbot_calib_shm/artifacts/calib/DVT2_01/19700109_013625`，完整验收采集在其中的 `verify/`。

## 标定交付

H1 静态重力、H2 摩擦以及 H3 惯量与转子惯量均通过各阶段的留出验证。最终交付包采纳这三阶段的结果。质量、质心和刚体惯量读取 URDF，运行时摩擦、转子惯量和重力向量读取配套 JSON。

- [标定 URDF](calibrated.urdf)
- [运行时动力学参数](runtime_dynamics.json)
- [标定来源与各阶段验收门禁](calibration_bundle.json)
- [交付总表](summary.md)

## 闭环验收

本轮跑完五组参考（`140.36 s`）并收到驱动失能应答，闭环验收判定为 **PASS**。柔和档采用 `0.5× Kp`、`1.0× Kd`，前馈采用 `1 ms` 采样步长、剥离静摩擦破冰尖峰并将过零平滑速度下限设为 `0.10 rad/s`。

- G3_soft 全程法兰位置 RMSE 为 `6.32 mm`（左臂 `7.16 mm`、右臂 `5.36 mm`），G0_stiff 对应值为 `27.00 mm`（左臂 `23.40 mm`、右臂 `30.18 mm`）；相位滞后由 `+20.5 ms` 收窄至 `+11.0 ms`，PD 纠偏力矩 RMS 由 `1.87 Nm` 降至 `0.32 Nm`。
- 右肘 pitch（SHM ID 22）全程峰值速度降至 `1.112 rad/s`（`2.0 rad/s` 与 `1.5 rad/s` 口径下均为 0 帧超限）。
- 琴鼓转场中，最窄处为 G0_soft 从架子鼓回电子琴时左腕 roll 连杆与台板之间。实测关节反馈在工装 CAD 中回放的采样最小净空为 `18.41 mm`，连续回放下界为 `18.22 mm`。

[原始验收报告](verification.md) · [推荐演奏运行配置](recommended_band_gains.yaml) · [逐组跟踪、右肘速度与转场净空数据](tracking-and-clearance.json)
