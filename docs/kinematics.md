# 运动学标定 K1–K3

在 Orin 仓库根目录运行。顺序分为 **K1 左臂采集**（完成后失能退出）→ **工装换装右腕** → `--resume` 续跑 **K2 右臂采集** 与 **K3 离线辨识**。

## 上机前确认

- 固定四球台架，确认机体支撑稳固、双臂活动范围无遮挡，急停继电器在手。
- 拆除双手，先将 TriCal 工装装在左腕，用标准球将三块表清零；换装到右腕后保持这次清零值，不要重新清零。
- 在 [k.yaml](../configs/kinematics/k.yaml) 填写 USB 串口、x/y/z 三表的 BLE 地址、地址类型、正负号、清零读数以及工装质量/质心（`tool_com_m` 以工装 TCP 为参考，单位 m）。
- 唤醒三表后检查链路：运行 `bash calib/dev.sh trical-probe status --port /dev/ttyACM0` 核对三表地址、单位与轴向，再用同一串口核对读数窗口并按仪表响应调整丢弃的旧读数条数（`unit_tag` 为通知单位前缀 ASCII 值，`unit_scale_m` 将读数换算为米）。

## 怎么跑

```bash
bash calib/kinematics.sh --plan-only           # 仅检查规划轨迹，不上机
bash calib/kinematics.sh --robot DVT2_01       # K1 左臂采集
# 失能并确认安全后，把工装从左腕换至右腕
bash calib/kinematics.sh --resume              # 续跑 K2 右臂采集 + K3 辨识
bash calib/kinematics.sh --reanalyse           # 从已有记录重算窗口均值与 K3
```

- **模式选择**：默认 `full` 为每球 4 个训练点、2 个留出点，辨识后生成运动学交付；`--mode smoke` 每臂只采 A 球 1 点用于首次通断验证，`--mode small` 每臂四球各 1 点用于快速核对，两种快速模式仅输出名义残差报告。`--resume` 自动继承原会话模式。
- **原始记录与异常处理**：`native/commands.csv.zst` 与 `native/feedback.csv.zst` 可用 `zstdcat` 解压查看；切换前的旧 `.bin` 会话需使用旧版本重分析。仪表断连、读数超量程或执行异常都会终止采集并请求失能，先查阅阶段报告、`capture/executor.log`、`capture/native/result.txt` 及仪表日志，排查后再 `--resume`。若首点入球时读数不随位移变化，优先检查表头接触、三表轴向映射与缓存丢弃条数。

## 结果怎么看

最近一次运动学会话软链在 `artifacts/calib/latest_kinematics/`：

- `k1/measurements.json`、`k2/measurements.json`：采样窗口时间、三表读数及极差、实测关节均值和训练/留出分组。
- `k3/report.md`：分臂/分球名义残差；`full` 模式同时列出名义、零偏层、几何层的留出 RMSE。
- `delivery/summary.md`：最终选用层级、台架位姿修正、零偏表与验收结果（内部目标 0.4 mm，交付阈值 0.7 mm；当几何层使任一臂留出 RMSE 变差时自动退回零偏层）。
- `delivery/joint_offsets.yaml`：SHM ID 12–25 的关节零偏（弧度），按 $q_{\mathrm{true}} = q_{\mathrm{encoder}} + \mathrm{offset}$ 使用。
- `delivery/calibrated.urdf`：几何修正后的模型（保留源惯性参数，需配合 `joint_offsets.yaml` 使用）。
- `delivery/calibration_bundle.json`：参数定义、选用层级、台架估计和上游来源。

GUI 保存位路径现统一为 `fieldwork/gui_pose.yaml`。若 Orin 仍留有旧文件名，在部署目录执行一次 `mv fieldwork/zero.yaml fieldwork/gui_pose.yaml`。

## 本地 Mock 验证

```bash
bash calib/dev.sh kinematics-mock --mode small
bash calib/dev.sh kinematics-mock --mode full
```

Mock 在私有 SHM 端点运行物理仿真并模拟三表读数，默认提高模拟表频以缩短测试耗时，实际接线配置仍保存在同一配方中。
