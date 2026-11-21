# 运动学标定 K1–K3

在 Orin 仓库运行。K1 测左臂并失能退出；换装同一套三表工装后续跑 K2；K3 自动离线辨识。

## 开始前

- 固定四球台架，确认机器人支撑、周围空旷，急停继电器在手。
- 拆除双手，把工装装在左腕。三表用标准球清零；换装右腕后保持这次清零值。
- 在 `configs/kinematics/k.yaml` 填写 USB 串口、x/y/z 各表的 BLE 地址、地址类型与符号、清零读数和工装质量/质心。`tool_com_m` 以工装 TCP 为参考，单位 m。
- 唤醒三表后，用 `bash calib/dev.sh trical-probe status --port /dev/ttyACM0` 核对三块表的地址，逐表确认单位与轴向，再按实施计划的板卡实验确定丢弃条数。`unit_tag` 是通知里的单位前缀 ASCII 值，`unit_scale_m` 将数值换成 m。

## 怎么跑

```bash
bash calib/kinematics.sh --robot DVT2_01       # K1 左臂
# 失能后，工装换至右腕
bash calib/kinematics.sh --resume             # K2 -> K3
bash calib/kinematics.sh --reanalyse          # 从 feedback.csv.zst 重算窗口均值和 K3
```

首次上机可用 `--mode smoke`（每臂 A 球 1 点）；快速检查用 `--mode small`（每臂四球各 1 点）。续跑继承会话模式。两种快速模式只生成名义残差报告；默认 `full` 使用每球 4 个训练点、2 个留出点并生成交付文件。

采集目录中的 `native/commands.csv.zst` 和 `native/feedback.csv.zst` 可用 `zstdcat` 查看。旧格式会话的 `--reanalyse` 使用切换前的版本。

只检查路线：

```bash
bash calib/kinematics.sh --plan-only
```

仪表故障、读数超出捕获范围、修正后仍未入量程或执行故障都会结束采集并请求失能。查看本阶段报告、`capture/executor.log`、`capture/native/result.txt` 和原始仪表日志；排除原因后续跑失败阶段。首点入球读数不随位移变化时，检查表头接触、地址与轴向映射与仪表缓存设置。

## 结果怎么看

最近的运动学会话在 `artifacts/calib/latest_kinematics/`。

- `k1/measurements.json`、`k2/measurements.json`：窗口时间、三表读数、读数极差、实测关节均值与训练/留出分组。
- `k3/report.md`：按臂/球的名义残差；full 另列名义、零偏层、几何层的留出 RMSE。
- `delivery/summary.md`：最终选用层级、台架修正、零偏表和验收结果。内部目标 0.4 mm，交付阈值 0.7 mm；几何层使任一臂留出 RMSE 变差时选择零偏层。
- `delivery/joint_offsets.yaml`：SHM ID 12–25 的弧度修正，按 `q_true = q_encoder + offset` 使用。
- `delivery/calibrated.urdf`：几何修正后的模型，保留源惯性参数；配合独立零偏文件使用。
- `delivery/calibration_bundle.json`：参数定义、选中参数、台架估计与来源。

GUI 保存位文件为 `fieldwork/gui_pose.yaml`。已有 Orin 文件由操作员在部署时一次性执行 `mv fieldwork/zero.yaml fieldwork/gui_pose.yaml`。

## 开发验证

```bash
bash calib/dev.sh kinematics-mock --mode small
bash calib/dev.sh kinematics-mock --mode full
```

Mock 使用私有 SHM 物理进程和独立节拍的三表缓存。默认提高 Mock 表频缩短采集用时；真实三表配置保存在同一配方的仪表与窗口字段中。
