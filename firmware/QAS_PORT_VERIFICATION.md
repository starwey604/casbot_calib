# QAS 读写与频率测试移植结果

2026-11-13：普通桥接和独立测试 app 均已为 `promicro_nrf52840/nrf52840/uf2` 编译、链接并生成 UF2；
两次最终构建无编译警告。Zephyr 保持 `v4.5.0`，SDK 保持 `1.0.1`。

| 应用 | Flash | RAM | UF2 大小 |
| --- | --- | --- | --- |
| `firmware/trical_bridge` | 159,588 B / 792 KiB（19.68%） | 89,168 B / 256 KiB（34.01%） | 319,488 B |
| `firmware/tests/qas_rate` | 159,588 B / 792 KiB（19.68%） | 89,168 B / 256 KiB（34.01%） | 319,488 B |

两镜像的入口参数分别启用普通桥接/频率实验，测试入口并非由镜像大小区分。
UF2 头的地址为 `0x26000`，nRF52840 family 为 `0xADA52840`。
日志及配置保存在 `firmware/build/`，机器可读汇总为 `qas-port-verification.json`。

## 复现

```bash
bash firmware/build.sh
bash firmware/build.sh --rate-test
bash firmware/tests/run_native.sh
```

## 离线证据

三组 C 测试用 AddressSanitizer/UndefinedBehaviorSanitizer 运行通过：

- 生产协议/调度源码：实测样例、有符号小数、无效/溢出数据、命令白名单、20,000 组有界二进制输入；固定格点跳过而不补发，窗边界、排空和 DAT0/DAT1 独立计数。
- 生产 GATT 客户端/USB 命令 owner，加确定性 GATT/内核替身：三槽、16/128 位 UUID、目标特征 CCC 范围、订阅应答后才 ready、两种写入、忙/断线、无自动重连、控制队列损失终止、普通 app 拒绝频率命令、最大原始包 JSON 长度。
- 生产 USB transport，加 UART/ring 替身：部分 FIFO 写入保留未发送尾部、完整输入行、队列满、超长/二进制输入整行丢弃及 DTR 清理。

16 项 Python 测试通过：串口碎片重组、早于 USB result 的参数通知、ATT 错误、断线不重连、失败后恢复 MOD；
按原始通知独立解码/重算、解析值不符拒绝、相同读数保留、两流分开、跨窗/排空、缺帧/丢录、取消和倒退时间处理。
同时用主机解析器读取生产 C owner 实际输出的 JSONL 替身记录。

## 留给板卡实测的范围

本轮没有烧录、连接仪表或机器人，未取得真实读写频率。
上述替身检查验证本仓库的状态机和统计路径；USB 枚举、BLE 空口行为、具体 QAS 固件兼容性、参数持久化与恢复需实物验证。
下一步按 [USB 读写指南](docs/qas_usb.md) 先跑单表，再按 [频率实验](tests/qas_rate/README.md) 记录数据。
已有 K5 模拟三表和机器人运动入口未改动。
