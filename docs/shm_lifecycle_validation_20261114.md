# 2026-11-14 SHM生命周期：中断可退出，三秒停写不自动失能

本轮只核对用户已运行的Ctrl-C、C05及C06 SIGTERM三项，没有新增运动、SSH断线或硬杀试验。Orin记录为`artifacts/shm_fieldwork/lifecycle_69415435241802/`，boot_id仍为`d4d463d3-a56d-4885-8858-7b313ed12fd5`。

## 三项结论及对GUI的影响

| 项目 | 软件记录 | 用户观察 | 结论 |
| --- | --- | --- | --- |
| Ctrl-C → 新只读连接 | 原生`operator_interrupt`、`disable_ack=1`；随后只读2000行主站反馈、0命令，双臂状态0x21 | 用户提供after_ctrl_c观测完成输出；没有单独描述该次物理释放过程 | 本次受控中断已请求失能并获应答，原owner释放后可重新只读连接 |
| C05停止写入 | 静默3.001000686秒内成功发送0条，收到3001行主站反馈；双臂状态始终0x27，结束请求失能并获应答 | “停写命令，仍然没有明显释放。一直到请求失能才松下来。” | 本次三秒停写没有自动失能/明显释放；显式失能才松软 |
| C06 SIGTERM → 新只读连接 | 监督端向本次发送器发SIGTERM；原生`operator_interrupt`、`disable_ack=1`；后续新只读连接2000行、0命令、双臂状态0x21 | 用户报告C06完成，未另描述物理释放过程 | 本次SIGTERM退出和重新只读连接正常；不扩为任意崩溃、SSH断线或SIGKILL结果 |

三项采集的电机槽0–28所记录错误码均为0。两次后续只读各2000行的帧号各不相同；这是主站反馈帧，不另认定为独立电机采样次数。

有活动控制时，GUI停止/断开应主动请求失能并记录应答，不能只停写；只读连接不发控制命令。重新连接默认只读，显式新建控制，不自动续旧轨迹。三秒静默结果只覆盖本次设置，不据此指定更长时间的设备watchdog行为。

## Ctrl-C：保留原中断结果，不改成运动完成

`ctrl_c`原计划15秒保持，约进入保持1.257秒后触发中断。记录保留`execution_completed=0`、返回码1及`reason=operator_interrupt`，表示这次保持被主动中断，不是运行满15秒。

原始退出段有7条成功命令，双臂控制字均为0x06；末条反馈双臂状态掩码均为0x21，原生记录`disable_ack=1`。之后的`after_ctrl_c`完成两秒只读，不发命令。

## C05：确实停止了命令入队

静默边界来自`no_commands/native/events.csv`：

- `command_writes_stopped`：1950988177251 ns。
- `quiet_observation_end_disable`：1953989177937 ns。

按成功入队完成时刻检查`commands.csv`，半开区间内为0条命令。此间3001行反馈帧号各不相同，双臂`Statusword & 0x006f`始终0x27，错误码0；最后`disable_ack=1`。

用户的“直到请求失能才松下来”与上述状态记录相符。本轮支持“停写不等于停止有源控制”这一GUI设计依据；没有用小幅位移或手推数据来判断它。

## C06：是SIGTERM，发送器执行了退出处理

`sender_term/process.json`记录对本次PID12702发送SIGTERM；原生随后进入`operator_interrupt`退出段。退出段同样有7条成功命令、双臂控制字0x06，末反馈双臂状态0x21，`disable_ack=1`。计划终止用例的软件结果为完成；原生运行自身仍保留中断/返回码1，两者含义不同。

独立被动探针有320条快照，但仅2种帧号，318次重复。因此不能把探针三秒快照称为连续320条新反馈。发送器退出后无人出队时，队列可能积满，重复快照本身不证明主站停止或电机持续刷新。

随后`after_sender_term`重新建立唯一只读owner，取得2000行帧号各不相同的主站反馈、0命令，双臂状态均为0x21。复连没有重新prepare/activate，也没有清队列或自动使能。

## 证据位置与尚未开展项

- Orin原目录中的`case.json`、`process.json`、`native/commands.bin/csv`、`feedback.bin/csv`、`events.csv`和`result.txt`；C06另有`probe/`。
- 用户反馈已登记到`after_ctrl_c`、`no_commands`和`sender_term`的`observations.jsonl/md`、`report.md`；汇总原话为根目录`lifecycle_observations_20261114.json`。
- 本机完整回收在`artifacts/shm_lifecycle_20261114/lifecycle_69415435241802/`，流数据核对摘要为同级`readback.json`。

SSH会话断开/重新登录、真实网络中断、SIGKILL及主站死亡都未由本轮测试覆盖；本轮也未新增故障注入或恢复动作。下一项由用户选择后再单独开展，不自动连续执行。
