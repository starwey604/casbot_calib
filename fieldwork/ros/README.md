# 实机接入与现场联调手册 (`fieldwork`)

本目录提供接入 CASBOT 实机的第一阶段联调工具（CLI 与 GUI），核心用于完成三件事：
1. **测通信与反馈**：只读检查 SDK 连接、话题发布频率、14 轴数据完整性及断线恢复表现。
2. **存安全零位**：将机器人当前安全的实测姿态保存为现场零位，支持随时一键平滑回零。
3. **MIT 控制与单轴验证**：在 GUI 中直接调节 14 轴的 `position / velocity / kp / kd / effort`（MIT 五字段），或运行单轴短轨迹往返，核对关节 ID 映射与正负运动方向。

本文对应现有 ROS SDK 工具。直接接入 Jetson 共享内存的逐部位确认流程见 [SHM 现场逐组确认手册](../../docs/shm_field_validation.md)；SHM入口为[`fieldwork/shm/run.sh`](../shm/README.md)，不使用下面的SDK命令替代。

---

## 🧭 标准操作流程（速查）

到达现场、机器人上电并确认 `ROS_DOMAIN_ID` 后，建议按以下顺序推进：

1. **只读预检（不发控制指令）**：运行 `preflight`（5 秒）和 `observe`（30 秒），确认能收齐双臂 14 轴反馈、检查接收频率是否达标。
2. **记录安全零位**：确认机器人当前处于安全姿态后，运行 `capture_zero.sh`（或打开 GUI 点击 **采集零位**），将当前姿态保存到 `fieldwork/zero.yaml`。
3. **打开 GUI 调参台**：启动 `fieldwork/run.sh gui`，先点 **连接 / 读取** 观察 3D 模型是否与实机姿态一致；确认无误后点 **开始发送**，逐轴小幅拖动滑条或点 **所选轴短轨迹往返**，验证关节 ID、方向及 PD 响应。
4. **一键回零**：测试结束后，停止手动发送，点击 **回到保存零位**（或运行 `return_zero.sh`）让双臂平滑回到初始安全姿态。

> 如果需要用命令行跑固定的单轴 1° 往返规划、100/250/500Hz 频率分档对比或后续重力/摩擦/惯量整批采集，请参考 [实机 CLI 联调与标定路径](../../docs/hardware_execution_path.md)。
>
> 2026-11-13 的 Orin 实机只读接入记录（含关节命名差异、实测频率与现场网络注意事项）见[现场记录：Orin 实机只读接入](../../docs/fieldwork_site_orin_20261113.md)。

---

## 🛠️ 一、环境准备与本地演练

出发前或接实机前，可在仓库根目录下编译 SDK 组件，并用内置的本地物理 Mock 演练整套 GUI 流程：

```bash
# 1. 安装依赖并编译 SDK 接口组件（首次或更新代码后执行）
bash scripts/humble/container.sh python -m pip install -r scripts/humble/requirements.lock
bash scripts/humble/container.sh python -m pip install -r scripts/humble/requirements-gui.lock
bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh

# 2. 一键打开带本地物理 Mock 的 GUI 调参台（测试域号固定用 181）
bash fieldwork/ros/run.sh gui --mock --domain-id 181 --software-rendering
```

* `--mock`：自动在后台启动本窗口独占的本地 SDK 物理模拟器并完成配对，无需另外开终端。Mock 默认从双臂自然下垂的休息姿态起步（左肩滚转 `+20°`、右肩滚转 `-20°`，其余轴为 `0°`），方便避开零位手腕与髋部干涉。
* `--software-rendering`：强制使用 CPU 软件渲染。如果容器内显卡驱动正常，可以去掉该参数以获得更流畅的帧率；如果在容器内遇到 `iris` / `dri3` 等 OpenGL 报错，加上它即可解决。

---

## 🔍 二、第一步：只读检查 SDK 与反馈数据（CLI）

以下命令**只订阅反馈，不切换机器人控制模式，也不下发任何运动指令**。注意每次运行指定的 `--output` 目录必须是尚未创建的新路径：

```bash
read -r -p "请输入现场确认的 ROS_DOMAIN_ID: " calib_domain_id

# 1. 快速预检（默认 5 秒）：检查网络发现、端点 QoS 与 14 轴位置数据
bash fieldwork/ros/run.sh preflight --domain-id "$calib_domain_id" \
  --scope hardware_read_only --output artifacts/fieldwork/site_preflight_v1

# 2. 持续观测（默认 30 秒）：统计实际接收频率（期望 500Hz）、抖动分位数及各轴数值跨度
bash fieldwork/ros/run.sh observe --domain-id "$calib_domain_id" \
  --scope hardware_read_only --duration 30 \
  --output artifacts/fieldwork/site_observe_v1
```

运行结束后，终端会直接打印中文检查表（包含接收条数、平均 Hz、各轴首末角度与跨度），并在输出目录下生成 `report.md` 和 `report.json`。默认读取配置为 [configs/vendor_s1.yaml](../../configs/vendor_s1.yaml)（默认订阅 `/joint_states`，超时阈值 250ms）。

### 断线重连测试（可选）

如果需要测试网线拔插或运控服务重启后的反馈恢复情况：

```bash
bash fieldwork/ros/run.sh reconnect --domain-id "$calib_domain_id" \
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
bash fieldwork/ros/run.sh report artifacts/fieldwork/site_reconnect_v1 \
  --output artifacts/fieldwork/site_reconnect_analysis_v1
```

---

## 📍 三、第二步：采集零位与一键回零

这里的“零位”是指**你随时保存的一组 14 轴实测安全角度**（保存在 `fieldwork/zero.yaml`），既不是让 14 轴强行回 `0 rad`，也不会改写机器人底层编码器零偏。

### 1. 命令行采集与回零

```bash
read -r -p "现场 ROS_DOMAIN_ID: " domain

# 只读采集当前 14 轴实际角度，保存至 fieldwork/zero.yaml
bash fieldwork/ros/capture_zero.sh --domain-id "$domain" --scope hardware_position_stream

# 发送平滑单程轨迹回到保存的零位（默认过渡至少 5 秒，Kp=20, Kd=2, effort=0）
bash fieldwork/ros/return_zero.sh --domain-id "$domain" --scope hardware_position_stream --duration 5 --kp 20 --kd 2
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
bash fieldwork/ros/run.sh gui --profile configs/hardware_field.yaml \
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
6. **导出视频**：控制停止后，点击 **导出运动视频** 可将刚才实测反馈的运动过程渲染为 `motion.mp4`。反馈CSV按需逐行读取，不再整表载入内存；按接收时刻20 fps零阶保持，支持取消。

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
GUI 默认写入本地缓存（`~/.local/state/casbot-calib/fieldwork/sessions/`）。自动缓存、显式 `--output` 工作区和本窗口自有 Mock 共用记录额度，默认 512 MiB，用 `--cache-limit-mib 2048` 调大。顶部显示已用容量、当前写入速度及预计剩余秒数。
* 达到额度后停止追加原始观测、手动控制记录与自有 Mock 诊断，告警但不停止正常发送和实时反馈；约每秒检查，允许检查周期内新增量、已入队尾段和收尾元数据超出。GUI 回零执行也使用同一停录标记；规划必需的短起点执行反馈、参考和最终结果仍会生成。
* **停止发送不停止持续观测记录**；要结束只读采集，点击 **断开连接**。
* 点击 **保存会话**，输入目标目录，即可在后台保存各文件的完整前缀；不会自动生成回读 CSV，也不再复制第二套命令/反馈 CSV。
* 保存仍是独立快照，不是移动：缓存不会自动删除，多次保存会占用多份空间；存档目标不计入正在运行的记录额度。

### 2. 恢复历史缓存或打开已存会话
```bash
# 恢复最近一次未占用的自动缓存（如果 GUI 意外关闭，用它找回数据）
bash fieldwork/ros/run.sh gui --recover

# 以只读模式打开已保存的会话目录（可查看记录、导出 MP4 视频）
bash fieldwork/ros/run.sh gui --session artifacts/fieldwork/my_saved_session
```

### 3. 保存目录核心文件结构
每次控制运行位于 `controls/<控制编号>/runs/<运行编号>/`：
* `capture/execution/commands.csv`：实际发送时刻与14轴五字段指令，唯一命令表。
* `capture/execution/feedback.csv`：执行器接收时刻、发布 stamp 与14轴实测位置、速度、估算力矩，唯一执行反馈表。
* `capture/execution/events.csv` 与 `control_status.json`：模式切换、停止原因和首个故障。
* `recording/recording.json`：`casbot_fieldwork_recording_v3` 清单，以相对路径引用上述文件和共享原始数据，不复制 CSV。发生起点重规划时，清单指向实际执行的子目录。
* `recording/motion.mp4`：按需导出的实测运动回放。

工作区的 `saved_session.json` 使用 `casbot_fieldwork_snapshot_v3`。GUI仅在 `observations/<观测编号>/capture/raw/chunks/` 保存一套后台无损压缩原始记录，完整全轴JSON字段与原始CDR字节一起保留。每次控制的 `capture/raw_reference.json` 引用该观测及本次接收时间区间；GUI回零也共用它。活动块为 `.open.gz`，封存块为 `.bin.gz`；保存时将活动块的完整帧前缀重新封存。请保存完整工作区，不单独复制控制子目录。

原始块约4 MiB未压缩载荷或1秒换块，约20ms成组提交。硬杀可能丢失未提交/未flush尾段，已写完整帧可恢复；正常停止会排空队列。记录器退出会终止控制，不自动resume。新布局不提供旧 `recording/*.csv` 副本，格式与证据见[记录瘦身](../../docs/fieldwork_recording_storage.md)。

需要主反馈/debug的五字段对照时，显式导出选定时间窗和轴：

```bash
bash fieldwork/ros/run.sh export-readback \
  --run 'artifacts/fieldwork/my_saved_session/controls/<控制编号>/runs/<运行编号>' \
  --output /tmp/readback.csv --start 1 --end 3 --joint 0 --joint 7
```

时间窗相对该运行第一条 joint/debug 接收时刻，左闭右开；省略时间窗导出全程，省略 `--joint` 导出14轴。输出文件须不存在且在源运行目录之外；相同数值的帧仍逐条保留。分析与视频直接读取原生执行表，无需先导出回读。

### 大记录无需预先切分

`a-evidence`逐行统计完整CSV，分位数用临时SQLite精确求值；指定`--window`后，仅窗口内反馈与IMU进入拟合内存，统计仍覆盖全程，拟合算法和带宽不变。暂存位于输出目录，结束/异常/中断时清理；长记录建议将输出放在磁盘而非内存型`/tmp`。拟合本身的内存仍随窗口和带宽增长。

视频按当前与下一条反馈流式取帧；相同接收时刻取最后一条，异常尾行、非有限位置或时刻倒退会明确报错。取消/失败不生成最终MP4，可能保留`.pending.mp4`供人工清理。内存与数值对照见[记录瘦身：第三轮](../../docs/fieldwork_recording_storage.md#大csv分析与视频读取第三轮)。

### 4. 用已保存的数据评估手腕惯量辨识能力（`a-evidence`）

做完 GUI 短轨迹或手动测试并保存会话后，可以用 `a-evidence` 工具评估当前机器人的控制与反馈质量，判断**能否直接用简单的“路线 A（直接同带宽拟合）”完成手腕等小惯量关节的动力学标定**（以惯量偏差 **$\le 10\%$** 为合格标准），还是需要降频、放慢动作或改用“路线 B（双频频响拟合）”。

#### 第一步：怎么采对照数据

1. **优先复用已有记录**：在 GUI 已保存的会话中，挑选**同一关节、相近起姿、相同 `Kp/Kd` 和负载**的重复运动记录，且每段需包含**完整的单向加减速过程**（不能只截纯匀速段，否则加速度与速度无法解耦）。
2. **先测 500Hz 基线**：尽量包含正向、反向各一段，并独立重复一轮。
3. **按需补测对照（每次只改一个变量，另存为新会话）**：
   * **如果怀疑 500Hz 发送节奏不稳**：在 GUI 顶部“限制设置 → 等待与通信”中把发送频率切到 **250Hz**（必要时再试 100Hz），重新跑相同的短轨迹。
   * **如果发送很稳，但对几毫秒的通道延迟太敏感**：保持发送频率不变，把短轨迹时长拉长（降低运动频率/带宽）再测一轮。
4. **同步记录机身 IMU（排查自平衡站立晃动）**：
   * 重新编译 `bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh` 后，GUI 在连接实机时会自动同步录制 `/motion/imu`（保存在共享的 `observations/*/capture/raw/chunks/*.bin.gz`，分析工具直接读取）。
   * 若使用只读 CLI 采集，也可加 `--imu-topic /motion/imu` 显式录制：
     ```bash
     bash fieldwork/ros/run.sh observe --domain-id "$calib_domain_id" \
       --scope hardware_read_only --imu-topic /motion/imu --duration 30 \
       --output artifacts/fieldwork/site_imu_observe
     ```

#### 第二步：运行 `a-evidence` 分析脚本

```bash
# 1. 先不带窗口参数，快速汇总会话内每段控制的实际收发频率、缺帧情况与 IMU 统计
bash fieldwork/ros/run.sh a-evidence \
  --source artifacts/fieldwork/my_saved_session \
  --output artifacts/fieldwork/a_inventory

# 2. 指定关节与单向加减速时间窗，计算路线 A 惯量、±5ms 错位敏感度与 IMU 干扰影响
# （下例中 13 为右腕 roll，8~10 秒为相对于该次运行首帧指令的时间窗，30Hz 为离线共同处理带宽）
bash fieldwork/ros/run.sh a-evidence \
  --source artifacts/fieldwork/my_saved_session \
  --joint 13 --window 8 10 --band 30 \
  --output artifacts/fieldwork/a_window
```

> 💡 **提示**：如果会话里有多次控制且动作发生在不同秒数，可以把 `--source` 直接指向单次运行目录 `controls/<控制编号>/runs/<运行编号>/` 分别指定 `--window`。

运行后会生成易读的 `report.md` 和完整数据 `report.json`，包含以下核心诊断：
* **收发节奏**：实际发送/接收的平均 Hz、间隔 `p99`/最大值、发送比计划晚到的毫秒数（`send_lateness_ms`）。
* **路线 A 拟合结果**：该窗口算出的基准惯量 $J_A$（$\text{kg}\cdot\text{m}^2$）与粘性摩擦系数 $b$。
* **人为错位压力测试**：故意将力矩波形相对运动平移 **$\pm 1\text{ms}, \pm 2\text{ms}, \pm 5\text{ms}$** 后，看惯量 $J$ 的相对变化百分比。
* **10% 惯量误差对应的力矩预算**：给出 $0.1 \times |J_A| \times \text{RMS}(a_\perp)$（单位 $\text{Nm}$），代表要让 $J$ 的误差控制在 10% 以内，未建模的外界干扰力矩最多允许有多大。
* **机身自平衡晃动（IMU）影响**：将同一时间窗内的 IMU 角速度、角加速度、线加速度及离心项作为干扰基底投射掉，看惯量 $J$ 变化了百分之几（如果变化很大或提示“与惯量激励共线”，说明机身晃动严重污染了手腕数据）。

#### 第三步：怎么看指标下结论（以 10% 为界）

对照 `report.md` 中的数字，按以下顺序排查并做出 3 选 1 决策：

| 检查项 | 关注指标 | 应对动作 |
| :--- | :--- | :--- |
| **1. 发送稳定性** | `sender` 平均 Hz、间隔 `p99`/最大值、`send_lateness_ms` | 若 500Hz 经常出现大间隔且伴随力矩/跟踪异常，切换 **250Hz** 复测对比 |
| **2. 惯量正负与重复性** | $J_A > 0$；正反向相对差 $\frac{|J_+ - J_-|}{|J_-|}$；同方向跨轮相对差 $\frac{|J_2 - J_1|}{|J_1|}$ | 若 $J_A \le 0$ 先查力矩正负号与激励强度；若正反差或跨轮差 **$> 10\%$**，继续往下查是哪项干扰导致 |
| **3. 选窗敏感度** | 将窗口前后对半切开或删块，对比算出的 $J$ 是否剧烈跳变 | 若必须刻意挑某个窄窗口才能进 10%，说明动作本身不稳，需调整轨迹时长或速度后复测 |
| **4. 机身晃动干扰** | `A.body_motion` 中加入 IMU 项后的 $J$ 变化率 | 若加入 IMU 后 $J$ 变化明显或提示共线，先换机器人姿态、改变运动速度或固定腰部/躯干 |
| **5. 时序错位余量** | `A.perturbations`（$\pm 1, \pm 2, \pm 5\text{ms}$ 下的 $J$ 变化率） | 若轻微时移就导致 $J$ 变化远超 **10%**，优先放慢轨迹（降低运动带宽）以增大时间错位容忍度 |

**最终决策（三选一）**：
1. **直接沿用路线 A**：正反差、跨轮复测差及合理时序错位下的 $J$ 变化均在 **10% 以内** $\rightarrow$ 锁定当前发送频率和运动速度，直接用路线 A 推进标定。
2. **降档后沿用路线 A**：原 500Hz 或快速运动超 10%，但切到 **250Hz 发送** 或 **放慢轨迹速度** 后的实测复核进入 **10% 以内** $\rightarrow$ 采用降档后的配置走路线 A。
3. **排查 SDK 并启用路线 B**：降档后仍超 10% $\rightarrow$ 明确最大干扰源（发送卡顿、底层缓存/低通滤波、或力矩尺度问题）反馈给厂家优化，同时在标定侧改用多频激励的**路线 B**。

> 📎 **进阶探针退化测试**：如果采集了专门的单频/双频封存腕部探针，还可运行 `bash fieldwork/ros/run.sh sensitivity --source 探针目录 --output 输出目录`（跨轮对比可加 `--reference-assessment 旧评估目录`），一次性对 12 类、163 种缓存/滤波/滞后退化条件进行 A/B 路线全面对比，详见 [SDK 反馈敏感性评估指南](../../docs/sdk_feedback_sensitivity.md)。

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
