# CASBOT 双臂动力学标定交付总表

- **机器编号**：`DVT2_01`
- **会话目录**：`/home/nvidia/casbot_calib_shm/artifacts/calib/DVT2_01/19700109_013625`
- **采纳标定源**：`session_h3`（重力：`session_h1`，摩擦：`session_h2`，惯量：`session_h3`，转子惯量：`session_h3`）
- **交付产物**：`delivery/calibrated.urdf` + `delivery/runtime_dynamics.json` + `delivery/calibration_bundle.json`

## 1. 阶段验收门禁汇总

| 阶段 | 模式 | 先验基线 RMSE (Nm) | 候选留出 RMSE (Nm) | 验收判定 | 说明 |
| --- | :---: | ---: | ---: | :---: | --- |
| H1 静态重力 | `full` | 0.1905 | 0.1210 | **采纳** | 留出验证改善并通过防退化门禁 |
| H2 关节摩擦 | `full` | 0.4227 | 0.0729 | **采纳** | 留出验证改善并通过防退化门禁 |
| H3 惯量与转子 | `full` | 0.1765 | 0.1750 | **采纳** | 留出验证改善并通过防退化门禁 |

## 2. 14 轴交付动力学参数一览

| SHM ID | 关节 | 质量 (kg) | 粘性 b (Nm·s/rad) | 库仑 fc (Nm) | 静摩擦 fs (Nm) | 转子惯量 Ir (kg·m²) |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 12 | 左肩pitch (`left_shoulder_pitch_joint`) | 4.2588 | 0.0000 | 0.5317 | 0.5558 | 0.00000 |
| 13 | 左肩roll (`left_shoulder_roll_joint`) | 3.2206 | 0.2506 | 0.7205 | 1.4073 | 0.00000 |
| 14 | 左肩yaw (`left_shoulder_yaw_joint`) | 0.5850 | 0.0000 | 0.1988 | 0.2342 | 0.00000 |
| 15 | 左肘pitch (`left_elbow_pitch_joint`) | 0.3836 | 0.0000 | 0.4401 | 0.6136 | 0.00000 |
| 16 | 左腕yaw (`left_wrist_yaw_joint`) | 1.5417 | 0.0000 | 0.1950 | 0.2918 | 0.00000 |
| 17 | 左腕pitch (`left_wrist_pitch_joint`) | 0.0170 | 0.0000 | 0.1650 | 0.2699 | 0.00000 |
| 18 | 左腕roll (`left_wrist_roll_joint`) | 0.2057 | 0.0000 | 0.1746 | 0.3162 | 0.00236 |
| 19 | 右肩pitch (`right_shoulder_pitch_joint`) | 2.3164 | 0.0000 | 0.6009 | 0.8664 | 0.00000 |
| 20 | 右肩roll (`right_shoulder_roll_joint`) | 5.2120 | 0.1248 | 0.5997 | 0.8114 | 0.00000 |
| 21 | 右肩yaw (`right_shoulder_yaw_joint`) | 3.1604 | 0.0000 | 0.2822 | 0.3628 | 0.00000 |
| 22 | 右肘pitch (`right_elbow_pitch_joint`) | 2.5508 | 0.0000 | 0.4352 | 0.5976 | 0.02484 |
| 23 | 右腕yaw (`right_wrist_yaw_joint`) | 0.5548 | 0.0000 | 0.1906 | 0.3053 | 0.00000 |
| 24 | 右腕pitch (`right_wrist_pitch_joint`) | 0.0067 | 0.0000 | 0.1737 | 0.3112 | 0.01014 |
| 25 | 右腕roll (`right_wrist_roll_joint`) | 0.0447 | 0.0000 | 0.1845 | 0.2595 | 0.00000 |
