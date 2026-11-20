# nRF52840 新板上电测试

在仓库根目录用现有的 [Zephyr 环境](../README.md#首次准备)构建：

```bash
bash firmware/build.sh --example blinky
bash firmware/build.sh --example usb_cdc
```

1. **Blinky**：刷入后板载 LED 每 500 ms 翻转一次。此例程不启用 USB，启动盘消失且没有串口是正常的。
2. **USB CDC**：刷入后 LED 继续闪烁，USB 产品名变为 `CASBOT USB CDC Example`。关闭串口终端，运行 `firmware/.venv/bin/python firmware/tools/check_usb_cdc.py`；两次连接都显示 `PASS` 即通过。多块板同时连接时，用 `--port` 指定例程串口。

[进入 UF2 模式、复制两个镜像及复测回显的完整命令](../../docs/records/nicenano_usb_bringup_20261119.md)。

需要手动收发时，用 `firmware/.venv/bin/python -m serial.tools.miniterm <例程串口> 115200 --raw`；退出按 `Ctrl+]`。例程在主机置 DTR 后才回显，不输出启动横幅或日志。不要选 bootloader 自带的串口。
