# 用 USB 读写 QAS 千分计

先用普通桥接固件确认一块仪表的连接、参数和通知，再运行[独立频率实验](../tests/qas_rate/README.md)。
开发板启动后不会自行扫描或连接。仪表地址和地址类型由使用者明确输入，不能按名称、MAC 前缀或扫描顺序分配三表轴向。

## 连接和读取

完成板卡烧录及 USB 枚举后，从仓库根目录执行。以下 `/dev/ttyACM0` 和地址均需替换为实际值：

```bash
# 扫描 5 秒，仅发现，不连接；输出地址、地址类型、RSSI 和原始广播
firmware/.venv/bin/python firmware/tools/trical_usb.py scan \
  --port /dev/ttyACM0 --output artifacts/trical_scan_01

# 显式连接槽位 0，查询 MOD，然后请求一次测量，继续收通知 5 秒
firmware/.venv/bin/python firmware/tools/trical_usb.py command \
  --port /dev/ttyACM0 --address 78:CD:55:45:67:AD --address-type public \
  --commands 'cmd 0 wcmd MOD' 'cmd 0 wcmd MEAS' \
  --listen 5 --output artifacts/trical_read_01
```

工具输出 `events.jsonl`：包括完整通知 hex、板端接收时刻和主机接收时刻；相同数值不去重。
`command` 中一条指令的 `result=0` 仅表示 API 接受该指令，需继续查看 `write_done` 和仪表通知。
连接准备完成以 CCC 写入成功后的 `subscribed/state=ready` 为准。

主机工具结束会请求停止测试、断开自己的槽位并关闭串口；USB DTR 撤销后固件也会结束实验并断开所有连接。
不自动重连或接回旧实验。

## 明确修改参数

```bash
# 仅在你希望修改仪表持久配置时调用；例：关闭自动发送
firmware/.venv/bin/python firmware/tools/trical_usb.py command \
  --port /dev/ttyACM0 --address 78:CD:55:45:67:AD --address-type public \
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
2 scan 5000
3 connect 0 78:CD:55:45:67:AD public
4 cmd 0 wcmd MOD
5 cmd 0 wcmd MEAS
6 disconnect 0
```

| 操作 | 含义 |
| --- | --- |
| `status` | 应用名、三槽状态/连接轮次、时钟刻度、累计记录损失 |
| `scan <ms>` | 扫描 100–30000ms；不自动选表 |
| `connect <slot> <addr> <public\|random>` | 槽位 0–2 显式连接，发现 QAS 并订阅 |
| `disconnect <slot>` | 取消连接或断开 |
| `cmd <slot> <wcmd\|wreq> <QAS>` | 例 `MOD`、`MOD:0`、`INT`、`MEAS`；不添加 CR/LF |
| `stop` | 停止当前频率实验；普通桥接为空操作 |
| `rate <slot> <interval_ms> <duration_ms> <drain_ms> <wcmd\|wreq>` | 仅测试 app；详见测试说明 |
| `report` | 重新输出最近一次频率实验报告；结束后的统计冻结 |

一槽最多一条未完成的 GATT 写入，其余返回 `-EBUSY`，不另建积压命令队列。
连接/发现阶段默认 10 秒超时，不做自动恢复。控制事件队列丢失会终止本次固件运行中的连接资格，需重启板卡；普通原始事件丢失计数并使相关频率记录不完整。

## 数据含义

- 支持 QAS 的 `FFB0/FFB2` 及已登记的 `0783…ffb0/ffb2` 变体。CCC 搜索限定在目标特征范围内，不会把另一特征的 CCC 当作数据订阅入口。
- 一个 GATT notification 保留为一个原始包；不能用无结束符数据任意拼接、把 `DAT0/DAT1` 当作 x/y/z，或按相同值推断重发。
- `kind=0/1/2` 分别表示其它、测量、参数；测量解析保留整数 `mantissa`、小数位数 `decimals` 和词法前缀 `unit_tag`（字符 ASCII 码，未标注时为 0）。`numeric=false` 时这些数值字段不得用于测量。
- `m`/`i` 只作为原文单位提示保留；尚未绑定仪表单位、尺度、方向和 mastering，不在固件里默认换算成米。连接轮次 `epoch` 也不是仪表原生帧号。
- `time_us` 是通知进入桥端回调的单调接收时刻。`ticks_per_second` 声明实际系统刻度；微秒单位不意味着微秒精度。
- QAS 没有可用的采样时刻或逐次测量序号。本适配器不把接收时间或本地事件计数冒充它们，不推断物理接触状态。
- 原始包上限 244 字节；超长明确标记 `truncated` 并计入记录损失。USB/事件队列有界，满时不阻塞 BLE 回调、不静默覆盖旧记录；输入溢出丢弃整条命令并断开连接。

协议依据：`~/codings/guanglu-qas-ble/docs/qas_apk_reverse.md`、`tools/gl_qas_config.py` 及 README 实测记录；运行不依赖该外部仓库。
本实现没有打开旧 `0783…5cb8/5cba` 协议，也未连接机器人或替换 K5 的模拟仪表。
