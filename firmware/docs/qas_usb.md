# 用 USB 读写 QAS 千分计

板卡一上电就扫描带 QAS 服务的可连接广播，逐表连接、订阅。三块就绪后停扫；少一块就继续找，已有连接保持。给板卡供电即可，不用打开串口。现场先让表显示数值，再唤醒发射器。

## 查表和读取

从仓库根目录执行，串口替换为实际设备：

```bash
firmware/.venv/bin/python firmware/tools/trical_usb.py status --port /dev/ttyACM0
```

`ready_count` 是可读表数，和 LED 闪数一致。`addresses` 按槽位 0、1、2 排列；空字符串表示该槽尚未就绪，`P:` / `R:` 分别是 public / random。例如 `["P:78:CD:55:45:67:AD","",""]` 表示这块表在槽 0。槽位随接入顺序分配，标定时按地址匹配 X/Y/Z。

```bash
# 目标表当前在槽 0：查询 MOD，请求一次测量，继续收通知 5 秒
firmware/.venv/bin/python firmware/tools/trical_usb.py command \
  --port /dev/ttyACM0 --commands 'cmd 0 wcmd MOD' 'cmd 0 wcmd MEAS' \
  --listen 5 --output artifacts/trical_read_01
```

工具输出 `events.jsonl`：包括完整通知 hex、板端接收时刻和主机接收时刻；相同数值不去重。
`command` 中一条指令的 `result=0` 仅表示 API 接受该指令，需继续查看 `write_done` 和仪表通知。
连接准备完成以 CCC 写入成功后的 `subscribed/state=ready` 为准。
普通桥接与测速固件的板载 LED 每秒一组：零块就绪时熄灭，1／2／3 块分别短闪 1／2／3 下。
灯数表示订阅已就绪；要确认是哪块表、是否真的有数值，看 `status` 的槽位和对应槽的 `DAT0`／`DAT1` 通知。

连接结果、订阅确认和断开事件都带有仪表地址；断开时看 `disconnected.result` 的 HCI 原因码。

关闭串口会结束正在运行的测速实验，蓝牙连接保持。实验期间表掉线，该轮实验停止；重新连上的表可用于下一次实验。

## 明确修改参数

```bash
# 仅在你希望修改仪表持久配置时调用；例：关闭自动发送
firmware/.venv/bin/python firmware/tools/trical_usb.py command \
  --port /dev/ttyACM0 \
  --commands 'cmd 0 wcmd MOD:0' 'cmd 0 wcmd MOD' \
  --output artifacts/trical_config_01
```

`wcmd` 是 Write Without Response，完成回调表示本地蓝牙栈发送完成，不是仪表应用确认；
`wreq` 是 ATT Write Request，其完成回调包含 ATT 应答错误码。特征不支持所选模式时返回 `-ENOTSUP`，不暗中切换。
QAS 参数读取也是把 ASCII 参数名写到 FFB2，再从通知中取参数值，不是直接 GATT Read。

固件允许已知参数名的读取/设值和 `MEAS`，不提供 `CLR`、`RESTORE`、任意特征写入或 OTA。
配置值的语义/取值仍须按仪表版本确认；普通桥接不会自动改 HID、休眠、发送间隔或零点。

## USB 行协议

输入为单行 ASCII：`<id> <操作> ...\n`，id 范围 `1..2147483647`，整行长度小于 128 字节。
输出为 `schema=1` 的 JSON Lines，日志另走 RTT。可用串口终端直接输入：

```text
1 status
2 cmd 0 wcmd MOD
3 cmd 0 wcmd MEAS
```

| 操作 | 含义 |
| --- | --- |
| `status` | 就绪数量、各槽地址（含类型）、状态和连接轮次，应用名、时钟刻度、累计记录损失 |
| `cmd <slot> <wcmd\|wreq> <QAS>` | 例 `MOD`、`MOD:0`、`INT`、`MEAS`；不添加 CR/LF |
| `stop` | 停止当前频率实验；普通桥接为空操作 |
| `rate <slot> <interval_ms> <duration_ms> <drain_ms> <wcmd\|wreq>` | 仅测试 app；详见测试说明 |
| `report` | 重新输出最近一次频率实验报告；结束后的统计冻结 |

主机工具会在 `cmd` 和 `rate` 末尾附上本次查询得到的槽位 `epoch`，槽位换表或重连后旧命令会被拒绝。
一槽最多一条未完成的 GATT 写入，其余返回 `-EBUSY`，不另建积压命令队列。
连接/发现阶段默认 10 秒超时，释放该槽后继续找表。控制事件队列丢失会终止本次固件运行中的连接资格，需重启板卡；普通原始事件丢失计数并使相关频率记录不完整。

## 数据含义

- 支持 QAS 的 `FFB0/FFB2` 及已登记的 `0783…ffb0/ffb2` 变体。CCC 搜索限定在目标特征范围内，不会把另一特征的 CCC 当作数据订阅入口。
- 一个 GATT notification 保留为一个原始包；不能用无结束符数据任意拼接、把 `DAT0/DAT1` 当作 x/y/z，或按相同值推断重发。
- `kind=0/1/2` 分别表示其它、测量、参数；测量解析保留整数 `mantissa`、小数位数 `decimals` 和词法前缀 `unit_tag`（字符 ASCII 码，未标注时为 0）。`numeric=false` 时这些数值字段不得用于测量。
- `m`/`i` 只作为原文单位提示保留；尚未绑定仪表单位、尺度、方向和 mastering，不在固件里默认换算成米。连接轮次 `epoch` 也不是仪表原生帧号。
- `time_us` 是通知进入桥端回调的单调接收时刻。`ticks_per_second` 声明实际系统刻度；微秒单位不意味着微秒精度。
- QAS 没有可用的采样时刻或逐次测量序号。本适配器不把接收时间或本地事件计数冒充它们，不推断物理接触状态。
- 原始包上限 244 字节；超长明确标记 `truncated` 并计入记录损失。USB/事件队列有界，满时不阻塞 BLE 回调、不静默覆盖旧记录；输入溢出丢弃该命令并停止实验，蓝牙连接保持。

协议依据：`~/codings/guanglu-qas-ble/docs/qas_apk_reverse.md`、`tools/gl_qas_config.py` 及 README 实测记录；运行不依赖该外部仓库。
本实现没有打开旧 `0783…5cb8/5cba` 协议，也未连接机器人或替换 K5 的模拟仪表。
