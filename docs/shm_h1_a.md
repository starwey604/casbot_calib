# SHM H1/A：先做现场确认，再做短轨迹和静态重力

先按[fieldwork手册](../fieldwork/shm/README.md)完成本次开机窗口的接管与选定用例。H1/A已独立于ROS：Orin本机C++负责1ms执行与完整帧记录，Python负责快照、碰撞规划和数值分析。GUI已接入，用户已在实机确认双臂14轴ID、方向、零点及渲染对应一致，见[现场记录](shm_field_observations_20261114.md#最新gui双臂14轴映射确认)。

H1正式标定段检查原模型的连续参考碰撞及实际接收位置折线。2026-11-14新增用户明确批准的**脚本准备前缀**：只有这一段跳过碰撞，连续URDF位置及参考速度仍检查；旧入口不加选项时维持原完整检查。当前场景仍是DVT2裸法兰整机/地面。

## 从自然下垂姿态直接开始：脚本准备位 → H1

手臂自然下垂贴腿时，使用`--with-ready`，不需要GUI手动展开或先运行回零。准备位为ID13左肩roll **+20°**、ID20右肩roll **−20°**，其余双臂关节 **0°**。这是保存的参考角度，不修改编码器零点。

```bash
R=artifacts/shm_h1/ready_$(date +%s%N)
bash fieldwork/shm/run.sh h1 snapshot --output "$R/start"
bash fieldwork/shm/run.sh h1 plan-jog "$R/start" --with-ready \
  --joint-id 12 --amplitude-deg 10 --output "$R/plan"
HAND=artifacts/本次接管/before/native/command.bin
bash fieldwork/shm/run.sh h1 execute "$R/plan" \
  --hand-background "$HAND" --output "$R/capture" --execute
bash fieldwork/shm/run.sh h1 analyse "$R/plan" "$R/capture" --output "$R/analysis"
```

同一个原生发送器完成：

1. 当前实际位置保持0.5秒，再用C3参考同步展开/清零双臂，最少5秒，参考速度不超过0.15rad/s，幅度大时自动增加时长；Kp20/Kd2、前馈0。
2. 在准备目标保持位置参考，用2秒平滑接入本次H1的PD与显式重力前馈，再保持1秒。默认仍20/2；不复制反馈总effort为前馈。
3. 直接接入通过完整碰撞检查的H1参考，中间不失能、不换owner。小幅正向→反向→准备位；最终请求失能并等待应答。

上例按用户最新选择使用ID12 **±10°**；不传幅度时仍默认1°，允许显式(0,10]°，不夹取超范围输入。正式jog参考速度上限保持0.05rad/s，幅度增加自动延长各段；10°完整参考约43秒（含准备），不是把1°的时长直接套给10°。

所有正式参考在发送前完成证明；准备段单独检查URDF/速度。`preparation.json`记录跳过范围、目标、PD及交接时刻；`calibration_motion.json`是正式参考，`motion.json`/`reference.json`是实际完整时间轴。`collision_reference.json`明确仅覆盖正式段，并给出相对完整执行的偏移。原生事件`ready_begin`、`ready_field_handover_begin`、`h1_calibration_begin`可用于现场对照。

分析保留全部原始反馈，但碰撞折线检查只覆盖准备完成后的正式phase3区间；准备、进入/退出失能区间明确排除。新版反馈CSV末尾增加`phase`，表示记录器执行阶段，不是驱动状态字；旧无准备前缀的记录仍可分析。静态窗口和jog比较窗已平移到完整执行时间轴，不把展开数据混进标定。A的0秒仍是首条执行命令，选窗应读取`plan.json`的`preparation.calibration_begin_ns`，不会偷偷把准备段从时钟中扣掉。

`plan-static`也支持`--with-ready`：先准备，再走准备位到预设工作姿态、全部静态任务及返回准备位的完整碰撞检查路径。起点自动重规划会保留准备配方；驱动/通信/模式故障仍结束整个会话，不自动恢复。

复用组件位于`python/casbot_calib/shm_ready.py`，与GUI无关；后续执行流程可组合它与各自正式参考，不必复制复位逻辑。

## 在Orin运行

从GUI切换时，先点“停止并失能”，再点“断开”释放只读owner；仅停止控制仍占有状态出队，不能与H1 CLI并开。沿用当前有效接管窗口，不重复prepare/activate；Orin重启后才重新建立窗口。

```bash
cd /home/nvidia/casbot_calib_shm
bash fieldwork/shm/run.sh h1 --help
```

H1/A自动使用仓库的`.venv-shm/bin/python`；其它fieldwork命令仍只需系统Python标准库。不source ROS、不填域号。可以显式用`CASBOT_SHM_PYTHON`选择解释器。

首次准备数值环境：

```bash
bash scripts/shm/setup_h1.sh --wheelhouse artifacts/shm_h1_wheels_20261114
```

脚本在独立`build/shm-python`构建中安装项目venv、本工程绑定及SHM原生程序，并运行CTest，不改系统Python或厂家环境。离线wheelhouse包含ARM64/Python3.10依赖；没有wheelhouse时使用正常pip源，需可用网络和可信系统时间。Debian缺ensurepip时，离线入口用官方pip wheel初始化venv，无需改系统包。模型包`derived/bare_flange_1105_03_gauge_v2/`须完整物化：包括`collision.npz`及`visual_meshes/`。现有模型绑定合同也核对显示资产，不能只复制碰撞缓存或用LFS指针代替。源码可继续用Git部署，缺失资产可用rsync从本机已物化包定点同步，不必等待远端LFS下载。

独立URDF导出/重加载校验还使用与CMake相同依赖前缀的Python Pinocchio/eigenpy/Coal。setup仅在项目venv链接这三个数学模块，不source ROS、不导入ROS通信包或把整个系统site-packages加入venv。Orin已验证Pinocchio4.0.0与项目NumPy1.26.4兼容。

ARM原生解析固定观测frame时出现最大`1.11e-16`浮点差异。加载器保留URDF/资源精确绑定、运动轴几何及frame拓扑精确比较；仅固定frame矩阵在`8×float64 epsilon`绝对误差内恢复为冻结值，恢复后几何摘要必须完全一致。不会重写原模型身份或接受改变的几何。

## 第一步：只读快照，规划一条H1小幅轨迹

输出目录均须是新目录；下面的名称是示例，可自行更换。

```bash
bash fieldwork/shm/run.sh h1 snapshot --output artifacts/site_h1/start
bash fieldwork/shm/run.sh h1 plan-jog artifacts/site_h1/start \
  --joint-id 12 --amplitude-deg 1 --output artifacts/site_h1/jog
```

快照保存完整35槽状态、当前模型/连接范围；起点停稳速度采用统一现场默认`|v|<0.2rad/s`，配置键`start_velocity_rad_s`。该默认覆盖H1–H4、K系列接入、GUI手动和回零，不改变测量窗口判据或运行中的速度上限。小幅轨迹保持原H1“正向→反向→起点”波形，不是C03的先负后正波形。所有结点先规范到整数纳秒，再做连续碰撞、URDF位置与参考速度检查。

默认profile为`configs/shm/hardware_field.yaml`，逐轴Kp20/Kd2。H1**明确开启名义重力前馈**：在参考位置上用Pinocchio计算，冻结为1ms ZOH时间表；整数输出未变化的相邻条目合并，实际编码不变。它与旧SDK按实际位置在线计算补偿不同，配方/计划均写明`nominal_reference`。不加摩擦前馈，GUI手动模式仍不隐藏补偿。需要无重力前馈时另建profile显式选择`none`，不要编辑已封存计划。

## 第二步：预览，再显式执行和分析

`HAND`必须指向本次接管前保存的完整命令快照；只取其中的手部字段，绝不重放全身旧目标。

```bash
HAND=artifacts/本次接管/before/native/command.bin
bash fieldwork/shm/run.sh h1 execute artifacts/site_h1/jog \
  --hand-background "$HAND" --output artifacts/site_h1/jog_run
# 上一条仅预览，不创建采集目录。
bash fieldwork/shm/run.sh h1 execute artifacts/site_h1/jog \
  --hand-background "$HAND" --output artifacts/site_h1/jog_run --execute
bash fieldwork/shm/run.sh h1 analyse artifacts/site_h1/jog artifacts/site_h1/jog_run \
  --output artifacts/site_h1/jog_analysis
```

背景沿用现场已定设置：腿/腰/头失能、双臂按完整参考使能、手部冻结明确来源。正常结束请求失能并等状态应答；没有结束位置误差/停稳/回位验收。实际存在返回段时仍执行返回段。反馈位置超URDF只告警记录；反馈速度超过profile上限（默认1.5rad/s）结束控制，进入有界失能退出。切换、发送和退出阶段均检查，轴/值/阈值/动作写入`native/observed_limits.csv`并显示在CLI。参考/输入限位仍在发送前拒绝。故障结束整次collection，不清故障、不重新使能或resume。

执行前原生重新检查全部起点。**只有证明零写入、未进入模式切换且快照无故障的起点变化**，才自动新建只读快照、按原封存配方重规划；旧尝试记录`start_replan`，新尝试另存。碰撞/连接/驱动/通信错误不走此路径。也可手动`h1 replan 新快照 原计划 --output 新计划`。

`jog_run/result.json`指向实际生效的计划/采集尝试，analyse自动解析，不能只看第一个attempt。运行后的H1采集封存；分析另建目录，不修改原记录。

## 第三步：接已有静态预设并拟合重力

复用原有离线静态预设生成器；这一步不连接机器人：

```bash
.venv-shm/bin/python -m casbot_calib.cli plan-static-field \
  --config configs/static_field_nominal.yaml --output artifacts/site_h1/preset
bash fieldwork/shm/run.sh h1 snapshot --output artifacts/site_h1/static_start
bash fieldwork/shm/run.sh h1 plan-static artifacts/site_h1/static_start \
  --preset artifacts/site_h1/preset --max-poses-per-split 1 --output artifacts/site_h1/static_plan
```

`--max-poses-per-split 1`先取每臂train/validation各一个已选姿态，保留各姿态正反接近对；这不是完整覆盖。也可用重复的`--candidate-id ID`精确选取已选姿态；不写上限则取全部选定姿态。模型、场景和预设封存必须一致，整个实测起点到静态目标及返回路径重新检查。

然后用上节相同的`h1 execute`/`h1 analyse`，替换计划和输出目录。`analyse --windows-only`只输出静态窗口。完整分析复用原PSD求解器、独立train/validation及候选URDF重加载，输出`calibration.json`、`candidate_exploratory.urdf`和报告。

静态均值以同毫秒组的时间曝光权重计算；重复帧不按数值去重。空窗、时钟不可定位或无法拟合时，报告`analysis_available=false`及原因，不用prior冒充identified、不自动质量重试；执行故障和模型/封存/碰撞错误仍终止。结果仍是探索重力候选，不自动部署、不宣称单体参数唯一或正式S3/S4/P0通过。

## A直接读取SHM观测时间

```bash
bash fieldwork/shm/run.sh a-evidence artifacts/site_h1/jog_run \
  --window 1 4 --joint-id 12 --band 10 --output artifacts/site_h1/a
```

窗口以本次**首条成功执行命令**为0秒，需按实际运动选择，不照搬示例窗口。A也可以消费普通fieldwork主动运行记录，无需H1格式。

- `observation_time_ns = timestamp_ms * 1000000 - 本机配对offset`；它是主站转换附近时间，不是驱动采样时刻。
- `receive_mono_ns`仍是出队完成时刻，单独保存；初始化首条SHM写入也单独保存，不作选窗原点。
- 同毫秒的原始行全部保留，一组时间曝光由组内各行等分；力矩与全部模型列使用同一组权重/投影。加速度继续由拟合的报告速度解析求导。
- 墙钟跳变分段，不排序、不补样。两次配对之间无法定位的跳变区间不参与拟合；需要时用`--clock-segment N`选稳定段。
- 不加载SDK的5ms缓存假设，不从帧号造1ms采样时刻，不估计或补偿未知通道滤波。力矩保留0.01Nm量化；没有IMU时身体运动项明确不可计算。

A输出惯量局部诊断、速度/慢漂移可分离性、额外通道错位敏感性和信号量级。数学秩不足就给出不可计算原因，不修改既有算子或门槛来凑正结果。

## 在私有物理Mock先演练

新增准备位闭环使用同一个安装版执行器，从显式的近腿下垂夹具开始，验证一次使能内展开→H1→失能、前缀零前馈、正式路径及分析窗口：

```bash
.venv-shm/bin/python scripts/shm/verify_ready.py --bin-dir install/shm-core/bin \
  --output artifacts/h1_ready_private_check
```

夹具只模拟动力学/驱动整数接口，不模拟手臂与腿的接触力。

```bash
.venv-shm/bin/python scripts/shm/verify_h1.py --bin-dir install/shm-core/bin \
  --output artifacts/h1_private_check
```

此验收包含真实只读快照、H1小幅参考与碰撞检查、同一个原生执行器、A报告，以及两姿态/四窗口的静态PSD拟合和导出；不是全64姿态或实机验收。

也可逐步执行上述命令，在`h1 snapshot`及`h1 execute`加`--mock`，不要提供真实手部背景。Mock计划禁止切到真实端点，反向亦然。

H1默认使用单独的`configs/shm/h1_mock.json`：**启动前已明确设置使能、本地PD和固定初始前馈的五字段命令**，以提供停稳起点；响应时间0是显式合成夹具设置。它没有运行时隐形重力补偿。原`configs/shm/mock.json`仍默认失能并在重力下自由响应。两者都保留物理重力、惯量、整数接口和无watchdog的旧命令延续。


## 本轮交付与现场接续

2026-11-14已完成Orin原生构建/安装和私有物理闭环：小幅轨迹、两姿态四窗口静态PSD拟合/候选URDF重加载、A分析均通过。完整证据见[构建记录](shm_build.md#2026-11-14-orin-r3-h1a数值环境与安装闭环已完成)。本轮没有控制真实机器人。

GUI映射现场确认已完成。首次H1从自然下垂起点规划时，起点保持段左髋roll/左腕pitch间距约3.26mm，小于10mm要求，未发送运动；用户随后批准上述脚本准备前缀。下一步用新版`--with-ready`做ID12 ±1°，再接少量正反接近静态姿态。采样窗口不要手托/推手臂，避免外力混入重力拟合。
