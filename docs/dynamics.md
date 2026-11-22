# 双臂动力学标定与闭环验收

在 Orin 仓库根目录运行。流程依次完成 **H1 静态重力**、**H2 摩擦** 和 **H3 连杆与转子惯量** 辨识；每阶段自动衔接上游参数，并在阶段切换前失能。

## 上机前确认

- 拆除双手和腕部工具，在**裸法兰状态**下运行 H1–H3 与闭环验收。
- 固定机体支撑，清空双臂全程活动范围，急停继电器在手。需要保持下肢与腰头姿态时传 `--hold-body`，此时须保证吊装或外部支撑能独立承重并防倒，双脚平贴防滑地面、腿接近直立、腰和头归正。
- 若本机正在运行调参 GUI，先停止运动并失能，再断开 GUI 连接以释放 SHM 占用。
- 闭环验收使用 `1105_03` CAD 配准位置的固定工装，提前撤走动作范围内的琴架、鼓架、谱架和线缆。

**采集中的中断处理**：按一次 `Ctrl-C`（或发 `SIGUSR1`）会沿已校验路径退回本次起点后再失能；退回期间再次 `Ctrl-C`，或收到 ``Ctrl-\``、`SIGTERM`、`SIGHUP`，会立即请求失能。若异常退出后日志中的 `disable_ack` 为 `0` 或缺失，请立即用独立继电器切断驱动电源。

## 怎么跑

```bash
bash calib/dynamics.sh --plan-only
bash calib/dynamics_h1.sh --mode smoke
bash calib/dynamics.sh --mode full --robot DVT2_01
bash calib/dynamics.sh --resume
bash calib/dynamics.sh --reanalyse
```

- **离线检查**：`--plan-only` 只生成轨迹并完成连续碰撞、URDF 限位与参考速度检查，不连接设备。
- **标定规模**：配置文件默认采用 `small` 模式（仅跑左右肩 pitch，适合快速链路验证）；正式标定请加 `--mode full` 覆盖双臂 14 轴。`--mode smoke`（或 `--smoke`）用于 H1 准备位往返冒烟。
- **单阶段执行**：可使用 `calib/dynamics_h1.sh`、`calib/dynamics_h2.sh`、`calib/dynamics_h3.sh` 单独运行对应阶段，并用 `--config <YAML>` 指定自定义配方。
- **机器标识与续跑**：`--robot` 在首次新建会话时写入状态，后续 `--resume` 或 `--reanalyse` 可省略。发生驱动或通信故障时，先按报告定位原因，再用 `--resume` 续跑中断阶段。
- **站立保持**：开启 `--hold-body` 时，每次采集、续跑或闭环验收命令行都需要显式加上该参数。

机器人增益、默认重力与力矩上限见 [dvt2.yaml](../configs/robot/dvt2.yaml)（数组顺序依次为左臂 SHM ID 12–18、右臂 ID 19–25）；各阶段配方见 [configs/dynamics/](../configs/dynamics/)。

## 结果怎么看

最近一次会话软链在 `artifacts/calib/latest_dynamics/`。优先查看 `delivery/summary.md` 中的阶段来源与留出集 RMSE，再按需查阅各阶段的 `report.md`：

- `delivery/calibrated.urdf`：标定后的连杆质量、质心和刚体惯量。
- `delivery/runtime_dynamics.json`：配套运行时摩擦、转子惯量与重力方向。
- `delivery/calibration_bundle.json`：各部分参数来源与 URDF 重加载复核结果；若某阶段留出拟合未通过，会自动沿用上游/先验参数。
- `<阶段>/capture/`：实际执行尝试目录下的 `native/result.txt` 记录退出原因与 `disable_ack`；`native/commands.csv.zst` 和 `native/feedback.csv.zst` 可直接用 `zstdcat` 解压查看。
- `--reanalyse [会话目录]`：不连硬件离线重算。对完整采集会更新交付目录；对半程中断的采集仅把诊断写入 `<阶段>/diagnostic/report.md`，不改动会话状态与已有交付。注意 CSV/Zstd 切换前的旧 `.bin` 会话需要使用切换前版本工具读取。

## 闭环验收

```bash
bash calib/dynamics_verify.sh --plan-only
bash calib/dynamics_verify.sh --robot DVT2_01
bash calib/dynamics_verify.sh --reanalyse
```

默认选用该机器最近一次包含有效交付的会话，也可以直接将 `<会话目录>` 作为位置参数传入。结果保存在会话目录下的 `verify/report.md`，从法兰跟踪误差、关节跟踪误差、相位滞后和高频速度残差五个动作组进行对比：

- G2 对照组优先加载同一会话中 H2 的无惯量对照模型；若 H2 对照缺失，则退回名义前馈，报告会标明 G2 实际来源以及它与最终模型之间摩擦参数的差异。
- 验收通过（退出码 `0`）会生成 `verify/analysis/recommended_band_gains.yaml`。退出码 `1` 表示验收指标未通过或有效反馈不足，`2` 表示执行或分析异常中止。

数学模型、PSD 约束与五组验收对照定义见[标定原理与算法参考](methodology.md)。
