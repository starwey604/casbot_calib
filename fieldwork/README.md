# 实机接入与现场联调手册 (`fieldwork`)

本目录提供接入 CASBOT 实机的第一阶段联调工具（CLI 与 GUI），核心用于完成三件事：
1. **测通信与反馈**：只读检查 SDK 连接、话题发布频率、14 轴数据完整性及断线恢复表现。
2. **存安全零位**：将机器人当前安全的实测姿态保存为现场零位，支持随时一键平滑回零。
3. **MIT 控制与单轴验证**：在 GUI 中直接调节 14 轴的 `position / velocity / kp / kd / effort`（MIT 五字段），或运行单轴短轨迹往返，核对关节 ID 映射与正负运动方向。

---

## 🧭 标准操作流程（速查）

到达现场、机器人上电并确认 `ROS_DOMAIN_ID` 后，建议按以下顺序推进：

1. **只读预检（不发控制指令）**：运行 `preflight`（5 秒）和 `observe`（30 秒），确认能收齐双臂 14 轴反馈、检查接收频率是否达标。
2. **记录安全零位**：确认机器人当前处于安全姿态后，运行 `capture_zero.sh`（或打开 GUI 点击 **采集零位**），将当前姿态保存到 `fieldwork/zero.yaml`。
3. **打开 GUI 调参台**：启动 `fieldwork/run.sh gui`，先点 **连接 / 读取** 观察 3D 模型是否与实机姿态一致；确认无误后点 **开始发送**，逐轴小幅拖动滑条或点 **所选轴短轨迹往返**，验证关节 ID、方向及 PD 响应。
4. **一键回零**：测试结束后，停止手动发送，点击 **回到保存零位**（或运行 `return_zero.sh`）让双臂平滑回到初始安全姿态。

> 如果需要用命令行跑固定的单轴 1° 往返规划、100/250/500Hz 频率分档对比或后续重力/摩擦/惯量整批采集，请参考 [实机 CLI 联调与标定路径](../docs/hardware_execution_path.md)。

---

## 🛠️ 一、环境准备与本地演练

出发前或接实机前，可在仓库根目录下编译 SDK 组件，并用内置的本地物理 Mock 演练整套 GUI 流程：

```bash
# 1. 安装依赖并编译 SDK 接口组件（首次或更新代码后执行）
bash scripts/humble/container.sh python -m pip install -r scripts/humble/requirements.lock
bash scripts/humble/container.sh python -m pip install -r scripts/humble/requirements-gui.lock
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh

# 2. 一键打开带本地物理 Mock 的 GUI 调参台（测试域号固定用 181）
bash fieldwork/run.sh gui --mock --domain-id 181 --software-rendering
```

* `--mock`：自动在后台启动本窗口独占的本地 SDK 物理模拟器并完成配对，无需另外开终端。Mock 默认从双臂自然下垂的休息姿态起步（左肩滚转 `+20°`、右肩滚转 `-20°`，其余轴为 `0°`），方便避开零位手腕与髋部干涉。
* `--software-rendering`：强制使用 CPU 软件渲染。如果容器内显卡驱动正常，可以去掉该参数以获得更流畅的帧率；如果在容器内遇到 `iris` / `dri3` 等 OpenGL 报错，加上它即可解决。

---

## 🔍 二、第一步：只读检查 SDK 与反馈数据（CLI）

以下命令**只订阅反馈，不切换机器人控制模式，也不下发任何运动指令**。注意每次运行指定的 `--output` 目录必须是尚未创建的新路径：

```bash
read -r -p "请输入现场确认的 ROS_DOMAIN_ID: " calib_domain_id

# 1. 快速预检（默认 5 秒）：检查网络发现、端点 QoS 与 14 轴位置数据
bash fieldwork/run.sh preflight --domain-id "$calib_domain_id" \
  --scope hardware_read_only --output artifacts/fieldwork/site_preflight_v1

# 2. 持续观测（默认 30 秒）：统计实际接收频率（期望 500Hz）、抖动分位数及各轴数值跨度
bash fieldwork/run.sh observe --domain-id "$calib_domain_id" \
  --scope hardware_read_only --duration 30 \
  --output artifacts/fieldwork/site_observe_v1
```

运行结束后，终端会直接打印中文检查表（包含接收条数、平均 Hz、各轴首末角度与跨度），并在输出目录下生成 `report.md` 和 `report.json`。默认读取配置为 [configs/vendor_s1.yaml](../configs/vendor_s1.yaml)（默认订阅 `/joint_states`，超时阈值 250ms）。

### 断线重连测试（可选）

如果需要测试网线拔插或运控服务重启后的反馈恢复情况：

```bash
bash fieldwork/run.sh reconnect --domain-id "$calib_domain_id" \
  --scope hardware_read_only --duration 120 --interactive \
  --output artifacts/fieldwork/site_reconnect_v1
```

开启 `--interactive` 后，可在终端输入指令并回车打时间戳标记：
* `d`：登记已手动断开连接
* `r`：登记已恢复连接
* `n 备注内容`：记录文字备注
* `q`：提前结束采集并生成报告

### 离线重新生成报告

对已采集的目录重新运行分析（不启动 ROS）：

```bash
bash fieldwork/run.sh report artifacts/fieldwork/site_reconnect_v1 \
  --output artifacts/fieldwork/site_reconnect_analysis_v1
```

---

## 📍 三、第二步：采集零位与一键回零

这里的“零位”是指**你随时保存的一组 14 轴实测安全角度**（保存在 `fieldwork/zero.yaml`），既不是让 14 轴强行回 `0 rad`，也不会改写机器人底层编码器零偏。

### 1. 命令行采集与回零

```bash
read -r -p "现场 ROS_DOMAIN_ID: " domain

# 只读采集当前 14 轴实际角度，保存至 fieldwork/zero.yaml
bash fieldwork/capture_zero.sh --domain-id "$domain" --scope hardware_position_stream

# 发送平滑单程轨迹回到保存的零位（默认过渡至少 5 秒，Kp=20, Kd=2, effort=0）
bash fieldwork/return_zero.sh --domain-id "$domain" --scope hardware_position_stream --duration 5 --kp 20 --kd 2
```

* 回零轨迹会根据最大速度（`0.3 rad/s`）和加速度（`0.5 rad/s²`）自动规划 C3 平滑过渡，并检查 URDF 限位与自碰撞。
* 执行记录会自动保存在 `artifacts/fieldwork/zero_runs/` 下。

### 2. GUI 内采集与回零

* **采集零位**：在 GUI 点击“连接 / 读取”收到实机反馈后，点击 **采集零位** 按钮即可将当前实测位置写入 `fieldwork/zero.yaml`。
* **回到保存零位**：先点击 **停止发送** 结束当前手动控制，再点击 **回到保存零位**（可在旁边修改过渡秒数）。回零期间滑条会自动锁定，使用当前滑条上的 `Kp / Kd` 值（`effort` 固定发 0）；回零完成后，点击 **重读 / 新控制** 即可继续手动调参。

---

## 🎛️ 四、第三步：GUI 手动调参台（MIT 控制与短轨迹）

### 1. 启动并连接实机

```bash
read -r -p "现场确认的 ROS_DOMAIN_ID: " calib_domain_id
bash fieldwork/run.sh gui --profile configs/hardware_field.yaml \
  --domain-id "$calib_domain_id" --scope hardware_position_stream
```

*(注：无需在命令行指定输出目录，GUI 会自动写入后台运行缓存，你可以随时在界面上点击“保存会话”。)*

### 2. 操作步骤

1. **连接 / 读取**：点击后进入只读订阅状态，不申请控制模式。界面会显示目标话题、域号和发现的发布者数量（若发布者为 0，请检查机器人运控服务是否已启动或域号是否正确）。收到首帧完整反馈后，14 个位置滑条会自动初始化为当前实测角度，右侧 3D 视图实时跟随机器人实际反馈姿态。
2. **调整五字段参数**：点击某个关节名称选中该轴。左侧滑条调目标位置（`position`），右侧滑条调目标速度（`velocity`）、`Kp`、`Kd` 和前馈力矩（`effort`）。**按住 `Ctrl` 单击滑条可直接键盘输入精确数值。**
   * 底层控制律为模组本地 MIT 混控：$\tau = K_p(q_{\text{ref}} - q) + K_d(\dot{q}_{\text{ref}} - \dot{q}) + \tau_{\text{effort}}$。
   * 手动 GUI 默认不叠加任何隐式模型重力补偿，所见即所得。
3. **开始发送**：点击后程序会向机器人申请上身调试模式（`/motion/upper_body_debug`），并以 500Hz 下发双臂 14 轴五字段指令（不填充腰、头、手）。此时拖动滑条会实时改变下发目标。
4. **所选轴短轨迹往返**：选中某轴后点击该按钮，会在该轴当前目标基础上自动做一次小幅正负平滑往返并回到原目标（默认幅度上限 1°，时长 2–12 秒可调），非常适合逐轴核对 ID 映射和正负旋转方向。
5. **停止发送与重新开始**：
   * 点击 **停止发送** 会立即停止下发指令并退出控制模式，但只读反馈和 3D 模型仍继续实时刷新。
   * 停止后若想开启下一轮控制，点击 **重读 / 新控制**，用当前最新实测位置重新对齐滑条，再点 **开始发送**。
6. **导出视频**：控制停止后，点击 **导出运动视频** 可将刚才实测反馈的运动过程渲染为 `motion.mp4`。

### 3. 各字段默认值与数据来源

| 字段 | 初始值 / 显示来源 | 默认可调范围（可在“限制设置”修改） |
| :--- | :--- | :--- |
| **位置 (`position`)** | 首次收到的 14 轴实测位置反馈 | 随包 URDF 关节上下限 |
| **目标速度 (`velocity`)** | 默认 `0 rad/s`（实测速度单独显示） | `±1.5 rad/s` |
| **增益 (`Kp` / `Kd`)** | 若收到 `/motion/debug/joint_state` 则用回读值，否则默认 `20 / 2` | `Kp`: `0 ~ 1000`，`Kd`: `0 ~ 100` |
| **前馈力矩 (`effort`)** | 默认 `0 Nm` | `±10 Nm` |

### 4. “限制设置”面板说明

点击界面顶部的 **限制设置** 可调整保护阈值与输入范围，修改后点击 **应用设置**：

* **反馈观察（在线立即生效）**：
  * **位置越界**：实测位置超出阈值时**只标红告警并记录，不会中断发送**。
  * **速度越限**：默认阈值 `1.5 rad/s`。在 GUI 手动模式下默认**只告警不停流**，你也可以勾选“结束本次控制”让其在超速时自动停发。
* **目标输入（下次点“开始发送”生效）**：修改滑条允许输入的位置、速度、前馈力矩和 `Kp/Kd` 上下限。支持全轴统一设置或单轴独立覆盖。
* **等待与通信（下次点“开始发送”生效）**：可调启动停稳阈值、关节/模式回调超时时间（默认 2 秒 / 3 秒）、发送频率（100 / 250 / 500Hz）及短轨迹幅度/时长。
* **Mock（下次启动 `gui --mock` 生效）**：调整本地模拟器的力矩限幅与看门狗参数。

---

## 💾 五、数据缓存、保存与恢复

### 1. 随时保存会话
GUI 启动后默认将数据录在本地缓存目录（`~/.local/state/casbot-calib/fieldwork/sessions/`，单次上限默认 512 MiB，可通过 `--cache-limit-mib 2048` 调大）。
* 在界面中点击 **保存会话**，输入或确认目标文件夹路径即可保存。
* 保存操作在后台线程完成，**不会打断正在进行的控制或只读观测**。

### 2. 恢复历史缓存或打开已存会话
```bash
# 恢复最近一次未占用的自动缓存（如果 GUI 意外关闭，用它找回数据）
bash fieldwork/run.sh gui --recover

# 以只读模式打开已保存的会话目录（可查看记录、导出 MP4 视频）
bash fieldwork/run.sh gui --session artifacts/fieldwork/my_saved_session
```

### 3. 保存目录核心文件结构
每次控制运行（`controls/<控制编号>/runs/<运行编号>/`）下最常用的记录文件如下：
* `recording/commands.csv`：实际下发的每帧时间戳 + 14 轴五字段完整指令。
* `recording/feedback.csv`：本地接收时间戳、消息发布 `stamp`、14 轴实测位置 `q`、速度 `dq`、估算力矩 `effort`。
* `recording/readback.csv`：主反馈与 debug 通道回读的 `q / dq / effort / kp / kd` 对照。
* `recording/events.csv` & `control_status.json`：模式切换、越限告警、停止原因及首个故障记录。
* `recording/motion.mp4`：导出生成的 3D 实测运动回放视频。

---

## 🛡️ 六、安全与故障处理机制

1. **启动静止检查**：点击“开始发送”或“回到保存零位”时，程序要求所有关节实测速度 `< 0.02 rad/s`。如果机器人还没停稳，界面顶部会列出尚未静止的关节名称和当前速度并持续等待（可随时点“取消启动”）。
2. **通信与进程监控**：
   * 独立监督进程通过管道监控 GUI 心跳（约 2 秒超时），若 GUI 异常卡死或关闭，监督进程会自动终止底层发送器。
   * 运行中若关节反馈中断超过 **2 秒** 或模式反馈中断超过 **3 秒**（`stale_feedback` / `stale_mode`），会自动停止发送指令。
3. **故障后如何恢复**：
   * 发生越限停止或通信超时后，**当前控制会话结束，但 GUI 窗口不会关闭，只读状态订阅继续运行**。
   * 现场排查完原因后，只需点击 **重读 / 新控制** 同步当前位置，再点击 **开始发送** 即可开启新一轮控制。
4. **物理急停提醒**：软件层面的“停止发送”只是停止发布 ROS 控制消息并请求退出调试模式，**不能代替硬件急停按钮**。现场调试请务必保持物理急停在手边。
