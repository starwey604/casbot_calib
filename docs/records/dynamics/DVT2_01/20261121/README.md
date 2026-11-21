# DVT2_01 动力学标定与实机闭环验收

2026-11-21 在 DVT2_01 上用裸法兰和 1105_03 固定工装完成闭环验收。验收运行版本为 `80e0d72`，腰部增益为 `Kp=800.0`、`Kd=6.5`。原始会话在 Orin 的 `/home/nvidia/casbot_calib_shm/artifacts/calib/DVT2_01/19700109_013625`，完整验收采集在其中的 `verify/`。

## 标定交付

H1 静态重力、H2 摩擦以及 H3 惯量与转子惯量均通过各阶段的留出验证。最终交付包采纳这三阶段的结果。质量、质心和刚体惯量读取 URDF，运行时摩擦、转子惯量和重力向量读取配套 JSON。

- [标定 URDF](calibrated.urdf)
- [运行时动力学参数](runtime_dynamics.json)
- [标定来源与各阶段验收门禁](calibration_bundle.json)
- [交付总表](summary.md)

## 闭环验收

本轮跑完五组参考并收到驱动失能应答。按闭环门禁判定为 **FAIL**，唯一未通过项为 G3_soft 的高频速度残差 RMS 高于 G0_stiff，分别为 `0.019879` 和 `0.017191 rad/s`。

- G3_soft 的左、右法兰位置 RMSE 分别为 `6.45 mm` 和 `3.53 mm`。G0_stiff 对应值为 `24.55 mm` 和 `30.47 mm`。
- 右肘 pitch（SHM ID 22）全程峰值为 `1.763 rad/s`，出现在最后返回准备位的路段。本轮 `2.0 rad/s` 速度保护下超限 0 帧。
- 琴鼓转场中，最窄处为 G0_soft 从架子鼓回电子琴时左腕 roll 连杆与台板之间。实测关节反馈在工装 CAD 中回放的采样最小净空为 `17.94 mm`，连续回放下界为 `17.45 mm`。

[原始验收报告](verification.md) · [逐组跟踪、右肘速度与转场净空数据](tracking-and-clearance.json)
