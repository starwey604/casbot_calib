# nice!nano UF2 烧录与 USB CDC 实测（2026-11-19）

Blinky 烧录后，现场确认板载 LED 正常闪烁。随后换刷 USB CDC，Linux 识别到 `/dev/ttyACM0`，两次打开串口均通过文本和二进制回显测试。板上最后运行的是 USB CDC 例程。

## 1. 准备 UF2 镜像

在仓库根目录运行。尚未配置 Zephyr 的机器先按[固件环境准备](../../firmware/README.md#首次准备)初始化。

```bash
bash firmware/build.sh --example blinky
bash firmware/build.sh --example usb_cdc
```

本次两个例程均用 Zephyr `v4.5.0` 构建，目标为 `promicro_nrf52840/nrf52840/uf2`。镜像分别生成在 `firmware/build/blinky/zephyr/zephyr.uf2` 和 `firmware/build/usb_cdc/zephyr/zephyr.uf2`。

## 2. 进入 UF2 模式

用 USB 数据线连接板卡。用金属镊子两尖分别碰 **RST** 和 **GND**，短接一下就松开，马上再短接一下并松开。只碰这两个引脚，避开供电引脚。现场从 Blinky 切换时，红灯由闪烁变为呼吸灯；主机同时识别到 `239a:00b3` 和 `NICENANO` 磁盘。若磁盘已出现，直接挂载即可；若没出现，重新快速短接两次。

```bash
lsusb -d 239a:00b3
# 若桌面已自动挂载，跳过 udisksctl 这一行
udisksctl mount -b /dev/disk/by-label/NICENANO
uf2_mount=$(findmnt -rn -S "$(readlink -f /dev/disk/by-label/NICENANO)" -o TARGET)
cat "$uf2_mount/INFO_UF2.TXT"
```

烧录前核对 `Model: nice!nano` 和 `Board-ID: nRF52840-nicenano`。本次板卡报告 UF2 Bootloader `0.6.0`、SoftDevice `S140 6.1.1`。

## 3. 先刷 Blinky，再换刷 USB CDC

确认板卡后，在同一终端把 Blinky 的 UF2 复制到磁盘根目录：

```bash
cp firmware/build/blinky/zephyr/zephyr.uf2 "$uf2_mount/"
```

复制后板卡自动重启，`NICENANO` 消失。Blinky 的 LED 每 500 ms 翻转一次（约 1 Hz）；本次肉眼确认 LED 正常闪烁。Blinky 运行时不启用 USB，没有串口是正常的。

再按第 2 节双短接、挂载并重新取得 `uf2_mount`，改为复制 USB CDC 镜像：

```bash
cp firmware/build/usb_cdc/zephyr/zephyr.uf2 "$uf2_mount/"
```

重启后，`NICENANO` 消失，应用以 `CASBOT USB CDC Example`（`2fe3:0004`）重新枚举。本次分配到 `/dev/ttyACM0`；复测时按产品名识别，不依赖端口编号，也不选 bootloader 或外接 USB 转串口。

## 4. 复测 USB 串口收发

关闭占用该串口的终端，在仓库根目录运行：

```bash
firmware/.venv/bin/python firmware/tools/check_usb_cdc.py
```

脚本自动识别例程串口，置位 DTR，发送一段文本和 `0x00` 到 `0xff` 共 256 B 二进制数据，逐字节比对回传；随后关闭并重开端口，再测一轮。若同时连接多块例程板，先用 `firmware/.venv/bin/python -m serial.tools.list_ports -v` 确认端口，再给脚本加 `--port` 指定设备。

本次实测输出，两轮均通过：

```text
检查 /dev/ttyACM0
PASS 第 1 次连接：文本与 256 B 二进制逐字节一致
PASS 第 2 次连接：文本与 256 B 二进制逐字节一致
```
