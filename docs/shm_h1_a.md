# SHM H1/A：先做现场确认，再做短轨迹和静态重力

先按[fieldwork手册](../fieldwork/shm/README.md)完成本次开机窗口的接管与选定用例。H1/A已独立于ROS：Orin本机C++负责1ms执行与完整帧记录，Python负责快照、碰撞规划和数值分析；GUI留到本轮现场反馈后接入。

H1与C03的区别是：**H1始终检查原模型的连续参考碰撞及实际接收位置折线**。逐轴fieldwork的URDF-only例外不传入H1。当前场景仍是DVT2裸法兰整机/地面，不能套用到未建模的工具或障碍物。

## 在Orin运行

```bash
cd /home/nvidia/casbot_calib_shm
bash fieldwork/shm/run.sh h1 --help
```

H1/A自动使用仓库的`.venv-shm/bin/python`；其它fieldwork命令仍只需系统Python标准库。不source ROS、不填域号。可以显式用`CASBOT_SHM_PYTHON`选择解释器。

首次准备数值环境：

```bash
bash scripts/shm/setup_h1.sh --wheelhouse artifacts/shm_h1_wheels_20261114
```

脚本在独立`build/shm-python`构建中安装项目venv、本工程绑定及SHM原生程序，并运行CTest，不改系统Python或厂家环境。离线wheelhouse包含ARM64/Python3.10依赖；没有wheelhouse时使用正常pip源，需可用网络和可信系统时间。Debian缺ensurepip时，离线入口用官方pip wheel初始化venv，无需改系统包。模型碰撞数据`derived/bare_flange_1105_03_gauge_v2/collision.npz`须物化，不能用LFS指针代替；这与可选GUI显示网格无关。

## 第一步：只读快照，规划一条H1小幅轨迹

输出目录均须是新目录；下面的名称是示例，可自行更换。

```bash
bash fieldwork/shm/run.sh h1 snapshot --output artifacts/site_h1/start
bash fieldwork/shm/run.sh h1 plan-jog artifacts/site_h1/start \
  --joint-id 12 --amplitude-deg 1 --output artifacts/site_h1/jog
```

快照保存完整35槽状态、当前模型/连接范围；规划沿用原H1停稳起点条件`|v|<0.02rad/s`。小幅轨迹保持原H1“正向→反向→起点”波形，不是C03的先负后正波形。所有结点先规范到整数纳秒，再做连续碰撞、URDF位置与参考速度检查。

默认profile为`configs/shm/hardware_field.yaml`，逐轴Kp20/Kd2。H1**明确开启名义重力前馈**：在参考位置上用Pinocchio计算，冻结为1ms ZOH时间表；整数输出未变化的相邻条目合并，实际编码不变。它与旧SDK按实际位置在线计算补偿不同，配方/计划均写明`nominal_reference`。不加摩擦前馈，GUI未来仍不隐藏补偿。需要无重力前馈时另建profile显式选择`none`，不要编辑已封存计划。

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

```bash
.venv-shm/bin/python scripts/shm/verify_h1.py --bin-dir install/shm-core/bin \
  --output artifacts/h1_private_check
```

此验收包含真实只读快照、H1小幅参考与碰撞检查、同一个原生执行器、A报告，以及两姿态/四窗口的静态PSD拟合和导出；不是全64姿态或实机验收。

也可逐步执行上述命令，在`h1 snapshot`及`h1 execute`加`--mock`，不要提供真实手部背景。Mock计划禁止切到真实端点，反向亦然。

H1默认使用单独的`configs/shm/h1_mock.json`：**启动前已明确设置使能、本地PD和固定初始前馈的五字段命令**，以提供停稳起点；响应时间0是显式合成夹具设置。它没有运行时隐形重力补偿。原`configs/shm/mock.json`仍默认失能并在重力下自由响应。两者都保留物理重力、惯量、整数接口和无watchdog的旧命令延续。
