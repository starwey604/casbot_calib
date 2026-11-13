# Pro Micro nRF52840 构建结果

本页保留提交 `362582d` 的首版构建基础记录。当前 QAS 读写和独立频率 app 的结果见 [移植验证](QAS_PORT_VERIFICATION.md)。

2026-11-13，Linux 宿主机完整配置、编译、链接和 UF2 生成通过，构建日志无编译警告。

## 复现

完成 [首次准备](README.md#首次准备) 后，在仓库根目录运行：

```bash
bash firmware/build.sh
```

本轮首次构建使用空的 `firmware/build/trical_bridge/`，不是复用其它工作区的构建缓存。
日志保存在 `firmware/build/trical_bridge-build.log`，输出检查保存在
`firmware/build/verification.json`；两者为忽略提交的本机生成物。

## 目标与工具链

| 项目 | 实际值 |
| --- | --- |
| 应用 | `firmware/trical_bridge` |
| 板级目标 | `promicro_nrf52840/nrf52840/uf2` |
| Zephyr | `v4.5.0`，提交 `41d10b597304` |
| west | `1.5.0` |
| Zephyr SDK | `1.0.1` |
| ARM GCC | `14.3.0` |
| Python | `3.14.9`，`firmware/.venv` |
| CMake / Ninja | `4.5.0` / `1.13.2` |

实际 `.config` 包含 BLE Central、GATT Client、三个连接槽位、新 USB stack、CDC ACM 启动初始化与 RTT 丢弃日志模式；console 与旧 USB stack 均未启用。

## 镜像与内存

- Flash：142,780 B / 792 KiB，17.61%。
- RAM：36,496 B / 256 KiB，13.92%。
- UF2：`firmware/build/trical_bridge/zephyr/zephyr.uf2`，285,696 B。
- 同目录已生成 `zephyr.elf` 与 `zephyr.hex`。
- UF2 首块应用地址 `0x26000`，family ID `0xADA52840`，与 nRF52840 UF2 目标一致。
- `.dts` 保留上游应用、存储和 bootloader 分区，只有一个 CDC ACM 实例。

本轮验证范围为指定目标构建及生成配置/镜像检查。未烧录，USB 枚举、BLE 初始化与真实千分计收数尚未实测。
后续接入 QAS 协议和主机 reader，再验证单表与三表并发。
