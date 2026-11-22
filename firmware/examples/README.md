# nRF52840 新板上电测试

在仓库根目录用现有的 [Zephyr 环境](../README.md#首次准备)构建：

```bash
bash firmware/build.sh --example blinky
bash firmware/build.sh --example usb_cdc
```

1. **Blinky**：刷入后板载 LED 每 500 ms 翻转一次。此例程不启用 USB，启动盘消失且没有串口是正常的。
2. **USB CDC**：刷入后 LED 继续闪烁，USB 产品名变为 `CASBOT USB CDC Example`。关闭串口终端，运行 `firmware/.venv/bin/python firmware/tools/check_usb_cdc.py`；两次连接都显示 `PASS` 即通过。多块板同时连接时，用 `--port` 指定例程串口。

用 USB 数据线连接 nice!nano。快速短接 **RST** 和 **GND** 两次进入 UF2 模式，再挂载 `NICENANO` 磁盘。读取磁盘内的 `INFO_UF2.TXT`，核对 `Model: nice!nano` 和 `Board-ID: nRF52840-nicenano`。将要运行的 `firmware/build/blinky/zephyr/zephyr.uf2` 或 `firmware/build/usb_cdc/zephyr/zephyr.uf2` 复制到磁盘根目录。板卡会自动重启。换刷另一个例程时重新进入 UF2 模式。

需要手动收发时，用 `firmware/.venv/bin/python -m serial.tools.miniterm <例程串口> 115200 --raw`；退出按 `Ctrl+]`。例程在主机置 DTR 后才回显，不输出启动横幅或日志。不要选 bootloader 自带的串口。
