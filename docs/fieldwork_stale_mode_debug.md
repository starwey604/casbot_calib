# `stale_mode`：发送线程被写盘阻塞

本次错误是模式回调超时，不是关节速度越限。修复手动发送器的同步CSV写盘后，重新打开GUI，继续使用原启动命令即可；无需放宽模式超时。

## 用户缓存显示了什么

来源：`~/.local/state/casbot-calib/fieldwork/sessions/20261112T044434Z_3f356ccf`，私有端点`/casbot_calib_s2_sdk_42b24793f3`，控制`57ebf8d63fb9/runs/8f48cd0db5b4`。

- 发送开始约9.61秒后，发送器报`stale_mode`，模式回调间隔799.183ms，阈值500ms。
- 实际最后一条命令比故障事件早约784ms；发送器自己的反馈CSV也出现约786ms空窗。恢复时先处理到一个关节反馈，模式回调尚未处理，于是表面上报成“模式消息超时”。
- Mock在GUI报错前约683ms已经进入`assumed_watchdog_hold`，即断流保护先发生，GUI随后才收到故障。
- 独立的控制侧原始记录器在故障前约1.9ms仍收到模式消息，此前约0.4秒基本保持20ms间隔。因此不是Mock单独停止发布模式。
- 两个原始记录进程也各自出现不同时间的接收停顿；它们的接收间隔不能直接解释为网络断流。

历史缓存确定了客户端执行停顿；没有保留当时的系统调用栈。源码中控制线程直接写/flush命令、反馈与事件CSV，是一条已找到并可重复触发该故障的阻塞路径。磁盘当前约97%已用，写入停顿也是需要留意的运行条件；本轮未删除用户缓存。

## 用800ms写入延迟复现并对照

只在自有私有localhost Mock中，给`execution/commands.csv`的一次写入注入800ms延迟；不修改模式/反馈阈值、积分步长、控制频率或重力策略。

| 对照 | 修复前 | 修复后 |
| --- | --- | --- |
| 磁盘写入实际停顿 | 800.102ms，发送主线程 | 800.095ms，独立I/O线程 |
| 结果 | 同样出现`stale_mode`，模式回调802.937ms | 发送继续，最后由用户正常停止 |
| 发包情况 | 报错前约800ms没有再发命令 | 整段最大相邻发布间隔4.84ms |
| Mock发送期间断流保护 | 触发 | 未触发 |
| 正常收尾 | 故障终止 | 队列排空、记录未暂停 |

注入前/后记录分别在`artifacts/fieldwork_stale_mode_injected_before_v1`与`fieldwork_stale_mode_injected_after_v1`。测试用系统调用探针及源码在`artifacts/fieldwork_stale_mode_io_v1`，未安装到正常启动环境。原始非注入复测`fieldwork_stale_mode_repro_v1`及I/O计时复测本身没有重现罕见的800ms停顿，不能把注入结果称作历史内核调用的直接测量。

## 手动CSV改为独立、有界写盘

- 新手动工作区的命令、反馈、事件和生效设置CSV复用现有`AsyncCsv`/spdlog线程池，由一个I/O线程顺序写入。回调/发布时刻仍在原发送线程取值；不降采样、不按数值相同去重。
- 只在新手动策略路径启用；H1–H4/S2和旧手动调用保留原同步文件路径。Mock原有诊断的阻塞队列合同不改。
- 队列8192槽，由唯一的发送线程生产。入队前为日志及flush预留两个槽；满后停止接收新记录、明确告警，不覆盖旧记录、不阻塞发送。SDK原始CDR记录仍独立运行。
- 写盘失败后，失败文件不再续写，保留可用前缀。新手动sink使用检查错误的flush，避免当前spdlog1.9默认file helper忽略`fflush`错误。GUI提示停录原因，自动短轨迹取消，手动发送/反馈与停止通道继续。
- 控制结束后排空已接受的行。`result.json`记录`execution_recording_mode`及`recording_paused`；导出的`recording.json`区分`live_prefix`、`finalized_prefix`与`paused_prefix`。
- 设置回执表示发送器已采用该版本；对应归档行异步可见，正常结束会排空。活动保存仍是各文件的已提交前缀，不承诺所有文件同一时刻截取。

真实模式消息停止时仍按原500ms阈值结束控制，不自动恢复。私有固定状态协议夹具停止模式发布、保持关节反馈时，实际在500.040ms触发`stale_mode`；发送结束后只读连接保留。证据：`artifacts/fieldwork_genuine_mode_timeout_v1/verification.json`。

## 检查范围

六项SDK CTest通过，覆盖原异步记录、单生产者队列满时立即返回且保留准确前缀、`/dev/full`写失败与同步兼容。80项相关Python测试通过，另补验停录具体原因在GUI中保留。SDK原始记录器没有在本轮改成异步，仍保留其接收/写盘语义；未使用这些测试授予实机或网络采样资格，未查看窗口外观。

交付索引与对照数值：`artifacts/fieldwork_stale_mode_delivery_v1/verification.json`；旧SDK Mock只读快照实际验证同步兼容路径通过（`fieldwork_async_legacy_v1`）。
