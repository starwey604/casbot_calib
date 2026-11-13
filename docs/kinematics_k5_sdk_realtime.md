# ⚡ K5 第一轮：原生 ROS 2 SDK 实时发送与闭环对中指南

K5 阶段将 [K2–K4 运动学三表自动对中流程](kinematics_trical_plan.md) 接入了**真实的 ROS 2 SDK 持续指令流（`position_executor`）**。
在本轮实现中，C++ 执行器以 **500Hz** 持续向 SDK 发送五字段控制流，三表读数由收到的实时 SDK 关节位置反馈驱动生成，完整跑通了 **“只读起姿对齐 → 自由空间接近 → 局部路线插入 → 读表与雅可比微调修正 → 140ms 稳定窗口采样 → 安全退出回位”** 的实时闭环。

> ⚠️ **安全边界说明**：
> 当前入口强制校验私有本地连接（`localhost_contract_mock`），Python 与 C++ 双层拒绝连接公共实机端点（`/upper_body_debug/joint_cmd`），**不提供实机碰球入口**。真实千分计硬件驱动、实测球架安装坐标、Mastering 零点与自由测杆全行程包络留待后续现场静态配对完成后接入。

---

## 🚀 一、快速运行：演练左臂 A 球实时 SDK 闭环

在仓库根目录下运行以下命令（注意 `--output` 目录不能提前存在，且需显式指定测试域号）：

```bash
# 1. 确保已编译 casbot_calib_vendor ROS 2 组件（首次或更新 C++ 代码后执行）
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh \
  --packages-select casbot_calib_vendor

# 2. 一键启动私有 SDK Mock 并执行左臂 A 球 (left_A) 接近、闭环对中、采样与退出
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python -m casbot_calib.trical_sdk --mock --domain-id 86 \
  --case left_A --output artifacts/my_k5_point
```

* `--mock`：自动在后台拉起本次命令独占的本地 `sdk_mock_server`（存储在输出目录的 `mock/` 下），任务结束后自动清理。
* 如果你已经单独启动了私有 SDK Mock，也可以去掉 `--mock`，改传 `--profile <实例目录>/client_profile.yaml`（必须与 `--domain-id` 一致）。
* **运行完整单点与四类故障注入验证脚本**：
  ```bash
  bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
    python scripts/humble/verify_trical_sdk.py \
    --output artifacts/my_k5_verification --domain-id 86
  ```

---

## ⚙️ 二、Python 规划器与 C++ 500Hz 发送器如何实时协作？

在 ROS 2 SDK 调试模式下，主机要求客户端持续高频发包（500Hz），而 Python 端的 Coal 连续碰撞证明、Pinocchio 雅可比求逆和 140ms 稳定窗口等待需要几百毫秒到数秒时间。为了防止 Python 计算期间断流触发看门狗，K5 采用 **“单次模式常驻 + 原子信箱分段下发（`TricalStream`）”** 机制：

1. **首帧实测起点保持与进场连接**：
   * 启动时先只读读取 SDK 当前实际关节角 $q_{\text{start}}$，在申请切入调试模式前先用 Coal 证明“从 $q_{\text{start}}$ 到冻结路线 `home`”的过渡段安全无碰撞。
   * 切入 `/motion/upper_body_debug` 后的第一帧指令严格保持 $q_{\text{start}}$，绝不拿配置文件里的名义 `home` 冒充当前实际位置。
2. **全程不退模式、不断流（原子信箱 `segment.csv`）**：
   * 同一个 C++ `position_executor` 进程从首次切入调试模式一直维持到整个测点正常退出。
   * Python 每次只需把通过碰撞检查的单段起止点写入原子信箱文件 `segment.csv`（共 31 个字段：`[generation, action, duration_s, q_from(14), q_to(14)]`）。
   * C++ 端的 `TricalStream`（[trical_stream.hpp](../ros/casbot_calib_vendor/include/trical_stream.hpp)）在收到新 `generation` 后，立即返回 `TRICAL_ACK` 确认，并在 C++ 内部以 **500Hz** 计算 C3 七次多项式参考轨迹下发五字段指令；当前段走完后，**自动无限期维持终点 `q_to` 的 500Hz 发送**，因此在等待千分计稳定或 Python 后台算碰撞时，500Hz 指令流一帧都不会断。
3. **显式名义重力前馈與速度降档余量**：
   * 执行器在 `effort` 中显式下发当前名义模型的重力前馈，PD 增益采用 K2 逐轴配置，不叠加摩擦或惯量前馈。
   * 进退路线参考速度上限设为 **`0.25 rad/s`**（加速度 `0.5 rad/s²`、Jerk `4.0 rad/s³`），为 C++ 端 **`0.35 rad/s`** 的实测速度上限留出充足的动态跟踪余量。
4. **`1 mrad` 接收位置空间弦段监测**：
   * 保留自由空间 `10mm`、近接对 `2mm`、测头接触 `-5mm`、`1°` 接触朝向锁定的参考管道检查。
   * 在线收到每帧关节反馈时，不把 ROS 发布 `stamp` 当作真实采样时刻，而是检查当前 14 轴实测位置偏离本段参考起止点连线（空间弦段）的最大距离是否 **$\le 1\text{ mrad}$**。
5. **反馈驱动的模拟三表与闭环微调**：
   * `SimulatedIndicators` 根据 `position_executor` 实际收到的 SDK 关节角计算三表读数，并故意注入 `(+120, -80, +60) µm` 初始偏置，触发控制器执行真正的雅可比闭环修正、退出路径重证明与 140ms 稳定窗口采样。

---

## 🛡️ 三、停止与故障处理规则

* **正常完成**：测点采样并退回 `home` 后，Python 提交 `action=2`（结束），C++ 执行器向 `/motion/upper_body_debug` 发送 `false` 请求退出调试模式，并核对服务应答与 `READY / IDLE` 状态反馈。
* **异常与故障终止**：若发生操作取消、千分计断线/超量程/无接触、140ms 稳定窗口超时、关节/模式反馈超时或异步写盘错误，**立即终止当前会话并停止发包**。故障后不会自动重申模式、不会在盲态下自动退刀，也不支持 `--resume`。

---

## 📂 四、输出目录文件说明

单点运行结果位于 `<output>/point/`（若未用 `--mock` 则直接在 `<output>/` 下）：

| 文件 / 目录 | 说明 |
| :--- | :--- |
| `report.json` | 本次 K5 单点运行总报告（`casbot_trical_sdk_report_v1`），含对中残差、发送帧数与最大间隔统计 |
| `capture/execution/commands.csv` | C++ 实际下发的 500Hz 五字段指令表（含 `source_evaluation_ns` 与 `segment_generation`，可精确重建每帧参考） |
| `capture/execution/feedback.csv` | C++ 执行器收到的 14 轴实时反馈与发布 `stamp` |
| `capture/raw_state/raw/` | 全轴原始 JSON / CDR 字节流留档 |
| `motions.json` / `events.json` / `measurements.json` | 每段轨迹的碰撞证明、C++ `TRICAL_ACK` 应答时间戳、读表修正记录与 140ms 稳定窗口详情 |
| `observations.npz` | 模拟三表读数与 SDK 关节反馈的配对快照（注：`joint_sample_ns` 固定记为 `-1`、`joint_sample_known=false`，不伪造底层采样时钟） |

---

## 📊 五、本机验证基准结果（2026-11-13）

验证证据保存在 `artifacts/k5_sdk_realtime_v3/verification.json`（引用单点记录 `artifacts/k5_sdk_realtime_v2/point`）：
* **闭环对中精度**：左臂 A 球（`left_A`）经过 1 次雅可比微调后进入稳定窗口，接受窗口内最大三表偏移仅 **`0.258 µm`**（远小于 `30 µm` 阈值）。
* **500Hz 持续发送稳定性**：单次模式会话连续发送 **23,514 帧**（包含 1 秒规划停顿及离线碰撞计算期间的持续保持），最大发送间隔 **`7.04 ms`**（远低于 Mock 的 `100 ms` 看门狗阈值）；C++ 原生生成的七次多项式参考与 SciPy 独立重建的最大差值仅 $1.11 \times 10^{-14}$。
* **故障安全拦截**：操作取消、模拟仪表断线、模式丢失、关节断流及起点超差反例均按预期安全终止会话。
