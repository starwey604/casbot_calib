# SHM现场逐例确认

先列出用例，再预览一例；只有追加`--execute`才运行主动用例。原生程序负责1kHz收发，Python只编排、记录观察和生成报告。每例结束退出，不自动跑下一轴。

```bash
# 在Orin仓库内；脚本也可从其它目录用绝对路径调用。
cd /home/nvidia/casbot_calib_shm
bash fieldwork/shm/run.sh list
bash fieldwork/shm/run.sh run C03 --joint-id 13
```

第二条只显示配方，不连接SHM。可加`--snapshot 只读记录目录`预览已保存起点、双向URDF限位及原始位置；执行仍重新读取起点。不加快照时不编造起点/限位结果。

## 先编译安装（不连接机器人）

```bash
bash scripts/shm/build.sh --with-core --build-dir build/shm-core \
  --install-prefix "$PWD/install/shm-core" --dependency-prefix /opt/ros/humble
```

需要CMake3.22/C++17，以及已在Orin验证的Pinocchio/Coal算法库。Python入口只用系统Python3标准库，不source ROS、不需要venv、域号或distrobox。当前`shm_executor`链接核心用于读取URDF；普通SHM协议构建仍可关闭核心。

可用不含pytest的私有软件联跑，**不会连接`shm_mc`或停业务**：

```bash
python3 scripts/shm/verify_fieldwork.py --bin-dir build/shm-core/cpp/src/shm
```

`build.sh`运行的CTest也包括该项。这里使用目标位置回显的协议夹具，物理Mock仍待实现。

## 下次到场重新接入：先留背景，再停业务，只启主站

如果Orin已重启，新开窗口；若未重启但上次测试状态未知，先核对当前进程/主站状态，不能重复握手来解卡。以下接管步骤只在确实需要新开主站时执行。

```bash
# 原业务还运行时：O_RDONLY/pread取证，不出队。
bash fieldwork/shm/run.sh inspect --output artifacts/shm_tomorrow/before

# 显式停止业务、只启主站；有固定支撑和现场继电器后运行。
bash fieldwork/shm/run.sh prepare --execute --output artifacts/shm_tomorrow/setup
# 查看厂家日志和进程，确认主站启动：
tail -n 30 artifacts/shm_tomorrow/setup/takeover/station.log

# 仅新主站握手一次，不发关节命令，不清索引。
bash fieldwork/shm/run.sh activate --execute --output artifacts/shm_tomorrow/activate
```

`inspect`需要命令/状态队列均有已发布帧；刚启动还没有帧时会明确失败，不把残留/未初始化内存称作实时反馈。失败记录保留在新目录中。

若厂家stop后入口中断，先查看setup日志。确认确实已停完时可显式续“只启动主站”步骤，**不是恢复旧控制会话**：

```bash
bash fieldwork/shm/run.sh prepare --execute \
  --start-only artifacts/shm_tomorrow/setup/takeover \
  --output artifacts/shm_tomorrow/start_only
```

已有有效独占窗口就跳过停业务与握手，直接观测。窗口持续到下次Orin重启，不自动恢复ROS业务。

## 全轴只读与C01/C02：先看首帧和退出

```bash
bash fieldwork/shm/run.sh observe --duration 10 --output artifacts/shm_tomorrow/observe
bash fieldwork/shm/run.sh snapshot --output artifacts/shm_tomorrow/start

hand=artifacts/shm_tomorrow/before/native/command.bin
bash fieldwork/shm/run.sh run C01 --duration 3 --hand-background "$hand"
bash fieldwork/shm/run.sh run C01 --duration 3 --hand-background "$hand" --execute
```

`observe`/`snapshot`只有一个出队owner，不发命令，不要与其它控制/观测owner并开。`inspect`与`probe`仅被动读取，可留故障证据，但不是一致的控制反馈；被动probe不报告真实采样率。

C01使能双臂、在本次实际起点保持3秒，再请求失能。C02同样有有限保持与失能退出，以下两例分别观察保持阶段与退出阶段，不能把“持续保持”误解为程序结束后仍发送：

```bash
bash fieldwork/shm/run.sh run C02 --variant hold --duration 5 --hand-background "$hand" --execute
bash fieldwork/shm/run.sh run C02 --variant disable --duration 3 --hand-background "$hand" --execute
```

所有主动例：腿0–11、头26/27、腰28失能；双臂默认全部使能，未测试轴固定本次实际起点。手部仅冻结显式背景的双手字段，不重放原全身目标。35槽和双手完整记录。模式变化先失能再MIT，应答按`status & 0x006f`判断。

## C03/C04：逐轴与背景部位

```bash
# 先复现ID12，再单独试ID13；一次执行一条，不写全轴自动循环。
bash fieldwork/shm/run.sh run C03 --joint-id 12 --hand-background "$hand" --execute
bash fieldwork/shm/run.sh run C03 --joint-id 13 --hand-background "$hand" --execute

# 同一轴观察对侧、腿/腰/头/手，再单独改变对侧臂的使能策略。
bash fieldwork/shm/run.sh run C04 --joint-id 12 --hand-background "$hand" --execute
bash fieldwork/shm/run.sh run C04 --joint-id 12 --other-arm disabled --hand-background "$hand" --execute
```

ID12–18左臂、19–25右臂，均按肩pitch/roll/yaw、肘pitch、腕yaw/pitch/roll排序。当前只允许双臂主动运动，头/腿/腰/手只读或沿用明确背景设置，不提供未知尺度的主动jog。

默认`q0 → q0−0.5 → q0+0.5 → q0`，每段5秒，Kp20/Kd2/前馈0，无隐藏补偿。可显式设置`--amplitude`、`--segment-s`、`--kp`、`--kd`。双向超URDF限位就跳过，返回码4，控制帧0；不夹取、不自动缩幅。按用户此前授权，这组双臂现场用例不查碰撞，只查URDF位置与参考速度；不改变其它标定入口。

## C05：停止写入但继续读

在支撑、继电器及受控退出已经安排好后，先预览，再执行：

```bash
bash fieldwork/shm/run.sh run C05 --duration 3 --quiet-s 3 --hand-background "$hand"
bash fieldwork/shm/run.sh run C05 --duration 3 --quiet-s 3 --hand-background "$hand" --execute
```

先保持3秒，再有3秒**完全不写任何命令**、仅继续接收，最后请求失能。`events.csv`记录静默边界；此间不补发、不重新使能。通信/驱动故障仍终止本例并尝试有界失能。观察是否沿用目标/释放/出现其它行为，由你反馈，不预设watchdog。

## C06：本次发送进程消失

先做受控SIGTERM，再考虑SIGKILL；两者是不同用例。独立被动探针启动并准备好后，仅终止本次创建的发送器，不杀主站/无关进程。

```bash
bash fieldwork/shm/run.sh run C06 --termination sigterm --duration 3 --quiet-s 3 \
  --hand-background "$hand" --execute

# 只有准备好独立物理停止，才执行这个硬杀例：
bash fieldwork/shm/run.sh run C06 --termination sigkill --duration 3 --quiet-s 3 \
  --hand-background "$hand" --execute
```

SIGTERM进入有界失能路径；SIGKILL不执行清理，**脚本不再起控制消费者补发失能，旧目标可能无限延续，现场用继电器结束输出**。正常原型无watchdog保障；无新状态的被动快照可能一直重复。硬杀可能留下互斥量不可用，入口不自动post/reset解锁。重开测试前确认当前状态。

C06执行完成只表示计划终止与被动观察结束，不表示驱动已失能/物理停止。Ctrl-C/TERM/HUP正常情况下转发给自有发送器尝试有界失能；整机或控制进程失效时由现场物理保护处理。

## 查看结果、登记观察和逐例台账

每例打印实际记录目录。下面用`RUN`指代该目录：

```bash
bash fieldwork/shm/run.sh report RUN
bash fieldwork/shm/run.sh note RUN --phase jog --text '实际动的是左肩roll，方向与模型一致，其余部位未见明显变化'
bash fieldwork/shm/run.sh note RUN --phase exit --conclusion inconclusive --text '夹具固定，无法仅凭不动区分释放和保持'
bash fieldwork/shm/run.sh status artifacts/shm_fieldwork
```

默认自动目录位于`artifacts/shm_fieldwork/`；自定义输出也必须是新目录。记录/报告/备注都可离线用，不需要原生程序或机器人。没有观察不算人工确认；无法区分就记`inconclusive`。用户观察原话和软件结果分开。

- `case.json`：配方、背景策略、模型路径、运行时间原值/boot_id。运行时间不冒充设备身份或可信UTC。
- `process.json`、`terminal.log`：实际命令、自有PID、退出码、计划终止及时间。
- `native/background.bin`：本次从新鲜实际状态生成的完整F0；`hand_background_source.bin`：显式手部来源。
- `native/commands.bin`：8字节小端成功入队完成单调时刻＋744字节原始命令。
- `native/feedback.bin`：8字节小端接收完成单调时刻＋696字节完整状态。
- `native/commands.csv`、`feedback.csv`：14轴SI五字段指令、q/v/effort反馈，35槽状态/模式/错误码及原始时间。
- `native/events.csv`、`clock_pairs.csv`、`queues.csv`：阶段/故障/静默、墙钟配对、诊断队列计数/索引。队列诊断是各字段读取，不是原子一致控制快照。
- `probe/`：C06独立被动记录，另存`passive_commands.bin`观察命令，绝不计为成功发送。被动快照无一致性/新采样保证。
- `observations.jsonl/md`、`result.json`、`report.md`：用户原话、软件结果、双臂与全部背景槽/双手变化。

原始写盘与终端隔离，有界队列满/写盘错误时停录、保留前缀，不阻塞发送。正常约20ms提交，硬杀可能缺少未提交尾段；离线报告保留完整帧并指出残缺字节/缺失封存，不resume。当前单例记录不使用旧GUI磁盘缓存/压缩/额度系统，控制时长有显式上限，现场需预留磁盘空间。

状态stamp仍是主站转换附近的毫秒墙钟，保存同机墙钟/单调钟配对映射；`receive_mono_ns`不改含义。首条SHM写入与首条执行命令分开。重复毫秒保留，本版每秒配对一次，偏移变化超过5ms加配对区间时新开分段；两次配对之间的跳变边界保留原始stamp/接收时刻供离线定位，不插值/排序补造时间。pair区间及1ms量化单列，A分析消费者仍待接入。

## 本轮可完成哪些确认

C00–C06已有CLI机制；到场后逐例填[确认台账](../../docs/shm_field_validation.md)，不能由软件测试替代实物观察。C03当前只覆盖双臂，C04只提供既定全身背景与对侧臂保持/失能对照；未实现头部主动jog、腿腰保持配方和手部主动映射。C07手模式/单位未定，只读留证；C08按已定安排后续源码/尺度核验。本轮不把这些未测部分记为全部确认完成。
