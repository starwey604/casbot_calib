# nice!nano 单表蓝牙实测（2026-11-20）

**建议从 125 ms 请求一次开始。** 新板通过 BLE 发送 QAS `MEAS`，通过 USB CDC 把千分表通知送回主机。60 秒内发出 480 次请求，收到 218 条有效 `DAT0`，平均 **3.63 条/秒**。通知中出现了 `DAT0:m+0.00`、`DAT0:m+5.11`、`DAT0:m+31.48` 等不同数值。这里统计的是收到的读数，不是表头内部采样率。

## 设备和状态

- 新 nice!nano 序列号 `D0682ED4F93F8620`，UF2 bootloader 0.6.0，测速固件 `qas_rate`。刷前复制了下载盘上的 `CURRENT.UF2`。旧板未参与本组测量。
- 广陆单表：`78:CD:55:45:64:04`（public），槽位 0；通过 FFB2 写入并订阅通知。仪表回读 `HIDMOD:0`、`MOD:0`、`INT:125`。本次没有改仪表参数。
- 起初 1 Hz 请求 12 次，只收到 12 条 `MEAS:OK`，没有数值。现场确认液晶显示数值后收到了 `DAT0`。复测先看液晶，再确认原始包里有数值，不能用命令回执代替读数。

## 请求频率与实际读数

下表按完整测量窗计算，排空阶段不计入。有重复测试的档位按实际执行顺序列出。

| 请求频率 | 完整窗口内收到的 DAT0 | 有效读数/秒 |
| --- | --- | --- |
| 1 Hz | 12 秒：12 条 | 1.00 |
| 2 Hz | 两个 12 秒窗：24、23 条 | 2.00、1.92 |
| 4 Hz | 12 秒：34 条 | 2.83 |
| 8 Hz | 两个 12 秒窗：40、37 条；60 秒：218 条 | 3.33、3.08；3.63 |
| 20 Hz | 12 秒：41 条；第二次中途断连 | 3.42（完整窗） |
| 50 Hz | 12 秒内仅收到 7 条，排空时连接超时 | 无完整实验结果 |

8 Hz 长测中，通知间隔中位数 255 ms，P95 为 381 ms，最长 800 ms；480 次写入均完成，板端未记录队列或 USB 丢失，原始通知与板端计数一致。20 Hz 与 50 Hz 的断连记录均为原因码 0x08（连接超时）。加快请求没有提高已验证的吞吐；这组数据不支持把 20 Hz 或 50 Hz 当作可持续读数频率。

## 复测

在仓库根目录执行；保持千分表液晶显示数值。先按[频率测试说明](../../firmware/tests/qas_rate/README.md)烧录测速固件；新版自动连接后，用 `status` 确认地址 `P:78:CD:55:45:64:04` 所在槽位，下例为槽 0。

```bash
firmware/.venv/bin/python firmware/tools/trical_usb.py rate \
  --port /dev/serial/by-id/usb-Zephyr_Project_CASBOT_Gauge_Bridge__development__D0682ED4F93F8620-if00 \
  --slot 0 \
  --intervals 125 --duration-ms 60000 --drain-ms 2000 --repeats 1 \
  --output "artifacts/qas_8hz_$(date +%Y%m%d_%H%M%S)"
```

本机原始数据在 `artifacts/qas_single_20261120/board_8620/`：`rate/` 为最初只有回执的实验，`rate_lcd/` 为多档测试，`rate_50hz/` 为 50 Hz 单独复测，`rate_8hz_long/` 为 60 秒复测。各目录的 `events.jsonl` 是原始事件，`analysis.json` 是逐窗分析。
