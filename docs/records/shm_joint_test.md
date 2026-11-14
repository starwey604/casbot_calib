# 双臂原生 SHM 逐轴试验

`shm_joint_test` 先打印所有电机关节槽和双手原始状态，再按本次实际双臂位置检查 URDF 位置限位及参考速度。主动入口一次只跑一个双臂 ID，默认 `q0 → q0−0.5 → q0+0.5 → q0`，每段5秒的C3平滑轨迹，Kp=20、Kd=2、前馈=0；峰值参考速度0.4375rad/s。

**2026-11-13用户指定现场条件**：先测双臂14轴，所有状态照常打印；机器人有固定支撑，腿/腰/头可以失能，现场有外置远程继电器保护。此轮是中间件控制流程探索，不需要先完成正式标定观测资格。

## 已部署旧版本的Orin窗口直接执行

下列是2026-11-13已验证旧版本的直接用法；新[fieldwork CLI](../../fieldwork/shm/README.md)待到场部署。该次主站已启动并完成握手；**没有重启Orin时，不要重复停业务或运行`shm_channel_start`**。SSH登录后运行下面的命令，每次只测一轴：

```bash
cd /home/nvidia/casbot_calib_shm
id=12                         # 左肩pitch；左臂12–18，右臂19–25
run="artifacts/shm_manual/$(date +%s%N)_id${id}"
install/shm-core/bin/shm_joint_test --execute --joint-id "$id" \
  --urdf python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2/model.urdf \
  --hand-background artifacts/shm_joint_test_20261113/before_stop/command.bin \
  --amplitude 0.5 --segment-s 5 --kp 20 --kd 2 \
  --output "$run"
cat "$run/result.txt"
```

这条命令会使能双臂、保持其它双臂轴的本次读取目标，并仅对所选轴发送`q0 → q0−0.5 → q0+0.5 → q0`，约15.5秒后请求双臂失能；腿/腰/头失能，手部沿用本次明确保存的背景字节。只检查URDF位置/参考速度，不检查碰撞。`Ctrl-C`请求有界失能退出；实机停止异常时由现场使用外置继电器。

仅查看状态与限位、不发控制帧时：

```bash
install/shm-core/bin/shm_joint_test --snapshot \
  --urdf python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2/model.urdf \
  --output "artifacts/shm_manual/$(date +%s%N)_snapshot"
```

只在本次独占窗口运行一个读取/控制进程，不与逐轴命令同时开快照。若Orin已重启，先按下文“现场先读，再单轴执行”重新保存手部背景、停业务、启动主站并握手，不能直接套用当前窗口命令。

## ID 与完整背景帧

- ID12–18为左肩pitch/roll/yaw、肘pitch、腕yaw/pitch/roll；ID19–25为右侧同顺序。ID直接使用PDF中的SHM数组下标，不引入1–14别名，也不代表CAN电机ID。入口拒绝12–25以外的运动ID。全部35个槽仍按真实ID打印，29–34标为保留槽。
- 早期v1记录里的`arm_ID1–14`是内部别名，依次对应真实ID12–25；旧记录保持原样，新入口和新记录统一真实ID。
- 腿0–11、头26/27、腰28发送`0x06`失能，目标填当前原始位置，不向零位跳转；不做腿部串并联坐标运动。
- 双臂先失能、切MIT、使能到起点保持；当次只改变所选轴，其余双臂目标固定为本次读取值。力矩前馈始终0，不暗加重力/摩擦。
- 手部不纳入运动试验。通过显式`--hand-background`文件冻结原命令的手部字段；只取其中双手字节和hand_count，不重放其中腿/腰/头/双臂旧目标，也不复制反馈总力矩到前馈。该手部背景选择随记录保存。

某轴的`q0−0.5`或`q0+0.5`超出URDF限位时，打印两端目标和限位，写`SKIP_urdf_limit`，**不发任何控制帧**。不夹目标、不自动缩幅。按用户本轮决定，本试验入口不执行碰撞检测，也不需要碰撞场景文件；原H1–H4/K5等标定与规划入口的碰撞规则不变。

## 编译

```bash
# Orin上：独立原生编译，使用前一轮已验证的算法库。
bash scripts/shm/build.sh --with-core --build-dir build/shm-core \
  --install-prefix "$PWD/install/shm-core" --dependency-prefix /opt/ros/humble
```

代码通过[Git部署入口](../shm_build.md#通过-ssh-从本地-git-克隆到-orin)传到Orin，不再导出或传输碰撞场景。

程序位于`install/shm-core/bin/shm_joint_test`。安装RPATH使用本次显式链接的算法库前缀，不要求加载厂家运控环境。

## 现场先读，再单轴执行

以下路径以Orin仓库根目录为工作目录，输出均须为新目录。

```bash
tool=install/shm-core/bin/shm_joint_test
urdf=python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2/model.urdf

# 原业务还在运行时，只做O_RDONLY/pread取证，不出队、不清索引、不发命令。
"$tool" --inspect --output artifacts/my_shm_test/before_stop
```

随后按[既定停业务流程](jetson_shm_interface_recon_20261113.md#用户确定的停业务后独占测试流程)，停止独立服务和launcher，只启动EtherCAT主站。此窗口中按用户假设认定无其它控制程序插队，直到下次重启；本程序不执行停业务脚本或恢复旧业务。

显式入口为`bash scripts/shm/prepare_field.sh artifacts/my_shm_test/takeover`（会中断现有控制，日志目录须新建）；它只包装已核对的厂家停止/启动顺序，不由构建或部署入口隐式调用。主站启动后先检查该目录的`station.log`及进程，再取新快照。

厂家stop可能在打印完成后因其清理/服务退出返回非零；入口保存返回码，并以本次三个服务已inactive/failed且原运控/主站进程已退出作为停业务完成条件。若停完后入口已中断，`--start-only 同一个takeover目录`只复核停止状态再启动主站，不重复停业务、不清故障，也不自动恢复业务。

```bash
# 独占窗口内只出队读取：打印全轴并生成双臂±0.5rad资格表。
install/shm-core/bin/shm_channel_start --fresh-master /shm_mc
"$tool" --snapshot --urdf "$urdf" --output artifacts/my_shm_test/snapshot

# 会切换状态、使能双臂并发运动命令；一次一个ID。
"$tool" --execute --joint-id 12 --urdf "$urdf" \
  --hand-background artifacts/my_shm_test/before_stop/command.bin \
  --output artifacts/my_shm_test/id12
```

每轴重新读取起点并检查双向限位；开始发送前又读新帧核对起点连接（0.005rad）。检查不通过就不使能、不发轨迹；不会用上轴旧起点继续。代理运行一例，用户观察反馈后再继续下一例。`--amplitude`只允许显式减小到(0,0.5]，不自动调整；`--kp/--kd/--segment-s`可显式修改，速度仍不超过URDF及1.5rad/s上限。

新主站先等待客户端启动握手，三个信号量均为0时不会产生反馈。`shm_channel_start --fresh-master`仅在两方向计数/索引全0且back/front/start均为0时，按原运控init顺序依次post一次；它启动共享I/O，但不下发电机命令、不清队列。普通连接不post、不重置索引，不能用此入口修复被占用的互斥量或自动恢复故障会话。

周期为1ms，队列满跳过本次、不补发历史轨迹。每方向容量2但厂家环形队列留空一槽，实际最多1帧待取；此结论来自当前安装二进制的Enqueue/Dequeue，客户端不复刻其重置索引的构造副作用。运行时与主站使用同名front/back互斥信号量；消费者仅有本程序一个。

正常结束发送`0x06`并等待双臂失能状态应答。Ctrl-C、监督端结束或故障会尝试有界失能后退出，不清故障、不重新使能；进程硬杀/通信失效不能保证指令到达，现场使用外置继电器处理。模式应答与物理停止分别记录。

## 输出与证据

- `arms.csv`：全部14轴当前位置、URDF上下限、双向目标与限位结果。
- `commands.csv/feedback.csv/events.csv`：成功入队/出队时刻、主站原始毫秒stamp、阶段和位置。成功入队不作模组执行证明；未把stamp当驱动采样时刻。
- `commands.bin`：每条为8字节小端单调入队完成时刻＋744字节原始命令；`feedback.bin`为8字节接收完成时刻＋696字节原始状态，保留全轴与双手。
- `status.bin`、`hand_background_source.bin`、`settings.txt`保存起点、手部背景和`collision_check=disabled_by_user`。旧运行中的`collision_reference.txt`只属于当时启用检查的版本。
- `result.txt`：执行结果、原因、最大成功发送间隔与记录前缀状态；`execution_completed`不表示物理往返误差或正式标定通过。

控制周期与磁盘/终端打印分离；有界记录队列满或写盘失败时停录保留前缀，控制不因慢日志阻塞。初始全轴状态打印并持续保存完整原始帧；新fieldwork入口将终端日志写入本例文件，并显示阶段变化，不依靠SSH输出维持周期。

私有`shm_channel_tests`检查真实IPC布局、保留一槽的满/空行为及连接不清索引；`shm_jog_tests`检查方向/零偏、截断、双向限位与波形。`shm_joint_integration`使用独占私有协议夹具测试使能—轨迹—失能、故障结束和限位拒绝零下发；夹具不是物理Mock，不作为实机运动证明。

## 2026-11-13 实机首轴检查记录

代码 `8a33224` 已统一真实ID，并在Humble及Orin通过关节换算/限位单测、私有协议正常往返/故障终止/限位零发送三例；本段仅记录实机已发生的步骤。

- 停业务、只启动主站并完成一次显式新主站握手后，可以持续读取真实全轴状态。未恢复ROS业务，接管窗口持续到重启。
- 早期 `id1_v1` 使用旧别名1（实际ID12），起点右腕pitch—右髋pitch模型距离为−3.248mm，检查拒绝，控制帧发送数0。
- 用户重新摆放后，`snapshot_repositioned` 的最小模型距离为11.933mm；这仅是起点检查，不表示整个±0.5rad参考通过。
- 更新真实ID程序后运行 `id12_v2`，新起点ID12为0.017114rad，目标−0.482886/+0.517114rad均在URDF范围内。但两次读取之间，左肩roll从0.152732变成0.108100rad，右肩roll从−0.163414变成−0.127555rad；新起点左腕pitch—左髋pitch模型距离4.777mm、右腕roll—右髋pitch6.408mm，低于10mm。参考首个查询即拒绝，仍未发送任何控制帧，未开始使能/运动。

材料：Orin仓库 `artifacts/shm_joint_test_20261113/{snapshot_repositioned,id12_v2}/`；本机对应副本位于同根目录的 `orin/`，对比诊断为 `snapshot_repositioned_diagnosis.json` 和 `id12_v2_diagnosis.json`。原始状态与历史别名记录不回写。

随后用户明确要求关闭本逐轴试验的碰撞检测，只检查URDF limit。当前入口已移除碰撞场景加载与参考证明，保留双向位置/速度限位、起点新读、通信/驱动故障终止及失能流程；不再要求现场为通过模型碰撞检查重新摆位。上述旧检查结果保留原意。修正状态字后的实际流程结果见下节。

### 关闭碰撞后的首轮控制及状态字修正

`fc1bd40` 的 `id12_v3_no_collision` 已通过URDF检查并发出3189条控制帧，但控制字全部为0x06；收到3187条状态，双臂始终为0x0231、mode=0、error=0。旧程序等待低字节严格等于0x21，3秒后以 `transition_timeout_phase_0` 终止并有界发送失能，未切MIT、未使能、未发轨迹。原始数据及本机 `analysis.json` 位于本轮 `orin/id12_v3_no_collision/`。

判定改为 `(Statusword & 0x006f)`：Ready to switch on为0x21，Operation enabled为0x27；0x0231中的bit4是电压存在位，不应参与状态比较。依据为[CiA402状态表及bit4说明](https://doc.synapticon.com/circulo_safe_motion/system_integration/status_and_controlword.html)，结合本机0x0231以及此前原运控启用时0x0237的实际观测采用这一解释；模式回读和error检查保留。单测覆盖两种电压位、故障/快停/禁用状态拒绝，私有正常夹具改用实测0x0231/0x0237，故障夹具仍用旧0x21/0x37。

用户已明确批准修正后新开ID12试验，不resume上述超时会话，不清故障。

### ID12新会话已完成控制流程

`8729c7a` 在Humble与Orin完成修正后的单测及三例私有协议测试后，按用户授权新开 `id12_v4_status_mask`，不重启主站、不恢复旧会话。实机按阶段0/1/2/3/4完成失能→MIT→使能→轨迹→失能应答，结果为 `execution_completed=1`、`completed_with_disable_ack`。

- 实际起点0.059457rad，指令范围−0.440543～+0.559457rad，每段5秒，Kp20/Kd2/前馈0；本入口无碰撞检查。
- 反馈范围−0.284632～+0.389053rad，末段附近及最后反馈均为0.043054rad。实际没有完整跟上目标幅度；本轮未调整增益或加前馈。
- 成功入队15583帧、记录反馈15582帧；最大成功入队间隔1.054095ms，记录未停录。29个电机关节反馈错误码均为0；ID12观测到0x0231/0x0237及mode 0/−1。
- 原始命令/状态、CSV及起点/设置在Orin `artifacts/shm_joint_test_20261113/id12_v4_status_mask/`，本机副本位于同根目录 `orin/id12_v4_status_mask/`，数值汇总为本机 `analysis.json`。

用户随后现场反馈：**“正常。没问题。能看到在动。”** 记为本次ID12可见运动正常的现场观察，和上述原始反馈/流程结果共同归档。停在ID12检查点，尚未跑其它13轴。主站继续留在本次独占窗口，不自动恢复ROS业务。

### 用户随后自行尝试ID13

用户报告：ID12暂时没问题，随后换ID13试了一次，Orin似乎卡住；用户决定暂不深究。尚未取得这次运行的命令、原始数据与退出结果，原因未定，不能归为Bash操作问题，也不能记为ID13完成。下次到场先确认当前开机/主站与测试进程状态，再开新会话，不resume旧尝试。后续优先建设fieldwork逐例记录和报告，见实施顺序。
