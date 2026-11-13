# 在开发板上测 QAS 写入与通知频率

这个独立 Zephyr app 复用普通桥接的同一套 GATT、协议解析和 USB 实现；测试节拍由 nRF52840 产生，不依赖 PC 每次唤醒的准时程度。
先对单表测量，得到可审计的通知吞吐，再决定三表实验和真实静态配对方案。

## 构建和运行

```bash
# 构建测试 app，输出 firmware/build/qas_rate/zephyr/zephyr.uf2
bash firmware/build.sh --rate-test
```

实际烧录和串口枚举完成后，使用下面的命令；地址、类型、串口需替换为现场值。输出目录必须是新目录。

```bash
firmware/.venv/bin/python firmware/tools/trical_usb.py rate \
  --port /dev/ttyACM0 --address 78:CD:55:45:67:AD --address-type public \
  --intervals 1000 500 250 125 100 50 20 --repeats 3 \
  --duration-ms 30000 --drain-ms 2000 --seed 20261113 \
  --output artifacts/trical_rate_01
```

默认 7 档×3 次，每档 30 秒，另加排空和准备时间。工具按固定种子打乱顺序并保存实际顺序。
如需比较 ATT Write Request，另开一个输出目录，用 `--write-mode wreq` 重复同一配方；默认 `wcmd` 与原工具一致。

测试会先确认 `HIDMOD=0`，读取 MOD 原值；若原值为 1，临时写 `MOD:0` 并回读确认。
每档开始前要求没有未完成写入，且至少 1 秒没有 DAT 通知；未满足时继续读流等待，最多 15 秒。
测试后恢复原 MOD 并回读。**INT、HID、休眠和零点不会被测试改动。**
连接故障不会自动重连；恢复失败记录在结果中，需要人工核对 MOD 原值。硬杀进程/拔线也可能留下 MOD=0。

仅监听已有自动通知、不写 MEAS 时，可以在测试 app 串口中输入：

```text
1 connect 0 78:CD:55:45:67:AD public
2 rate 0 0 30000 2000 wcmd
```

需先等到 `subscribed/state=ready` 再发第二行。`interval_ms=0` 是只监听模式，不要求 MOD=0。
普通固件拒绝 `rate`；查询、扫描、连接和参数读写的完整操作见 [USB 指南](../../docs/qas_usb.md)。

## 看哪些结果

- `request.json`：显式端口、地址、参数、随机种子与运行时间。
- `events.jsonl`：主机指令及板端原始事件，含独立的桥端接收/提交/完成时刻、通知 hex、连接轮次与真实连接间隔。
- `analysis.json`：主机用独立的 Python 字节解析器重解原始通知，核对固件解析值，再重新计算并核对板端计数。

可在不连接开发板的情况下重新分析：

```bash
firmware/.venv/bin/python firmware/tools/trical_usb.py analyze \
  --input artifacts/trical_rate_01/events.jsonl
```

**优先比较每档的 `accepted_write_hz`、两个 `streams[*].delivery_hz`、通知间隔 p50/p95/max，以及 `skipped/busy/errors`。**
`delivery_hz=N/T` 使用固定测量窗；`interarrival_hz=(N-1)/(末次−首次)` 单独报告，不混用分母。
DAT0、DAT1 分开统计，相同读数的多个通知全部保留。没有通知是一个可记录的负结果，不自动归因为丢包。

`write` 的 `result=0` 表示本地 API 接受写入；`write_done` 对 `wcmd` 是本地栈完成，对 `wreq` 是 ATT 应答。
`stack_completion_us` 只衡量这个本地/ATT 完成等待，不是 MEAS 到测量值的 RTT。
`MEAS:OK`、其它参数回执、可解析 DAT 和无效 DAT 分开计数；命令回执不当成测量读数。

## 为什么比旧脚本更适合判断频率

旧 `gl_rate_probe.py` 有几个具体问题：

1. 多个 MEAS 并发时，用通知到达时刻减去最近一次 `last_send`，无法识别通知实际对应哪次请求。
2. 档位之间不排空、不重新确认静默，上一档迟到的通知会混入下一档。
3. DAT0/DAT1 合并，只有请求调用数和通知总数，没有本地写完成、错误/忙或调度漏槽计数。
4. `next_send += interval` 在程序迟到后会补发；原 MOD 没有恢复。

新 app 使用固定时间格点，漏掉的格点计入 `skipped`，只尝试最新到期格点，不追发历史请求。
每次测量窗为 `[start_us, stop_us)`；停止写入后继续记录到 `end_us`，排空阶段的 DAT 单列 `tail_data`，不算入窗内吞吐。
`max_late_us` 是本次尝试相对所选最新格点的迟到，完整发送停顿还要看原始写入间隔及 `accepted_write_gap_us`。

`window_complete` 表示整窗执行完毕，`raw_complete` 表示窗口及排空期间未检测到事件/USB 记录损失。
主机的 `usable` 还要求原始通知、写入及完成计数与板端报告一致，独立解析结果相同且没有截断。
一槽一条在途写入的策略保持可见，结合 `busy` 与实际写入间隔区分发送端受限和通知端受限。
USB 堵塞或队列满时保留数据，结果标为不可用。

1 秒静默和 2 秒排空是可见的实验设置，不能证明设备缓存上界。QAS 缺乏设备帧号/采样时刻，迟到数据仍不能逐次归因；
所以本测试报告**接收通知率和写入传输表现**，不报告独立 ADC 更新率、逐请求成功率、三表同步或实机标定资格。

## 离线回归

```bash
bash firmware/tests/run_native.sh
```

同一生产源码接受纯 C 协议/调度测试、GATT/内核替身状态机测试、UART/ring 替身传输测试和 Python 主机/重分析测试。
C 测试开启 AddressSanitizer/UndefinedBehaviorSanitizer。替身不提供 BLE 空口或真实仪表频率证据；板卡实测结果须另行记录。
