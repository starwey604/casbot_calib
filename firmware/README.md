# TriCal BLE–USB 固件

在仓库根目录运行 `bash firmware/build.sh`，构建 Pro Micro nRF52840 的 UF2 固件。
应用源码在 `firmware/trical_bridge/`；Zephyr、模块、虚拟环境和构建结果均由本机生成，不提交。

## 首次准备

需要 Python ≥3.12、Git、CMake、Ninja、设备树编译器 `dtc`，以及包含 `arm-zephyr-eabi` 工具链的 Zephyr SDK。
本机已有 SDK 位于 `~/zephyr-sdk-1.0.1`，构建系统可自动发现；其它安装位置可通过
`ZEPHYR_SDK_INSTALL_DIR` 显式指定。固件环境与主项目的 ROS/Python 环境分开。

从仓库根目录执行（依赖下载需要联网）：

```bash
python3 -m venv firmware/.venv
firmware/.venv/bin/python -m pip install -r firmware/requirements.txt
firmware/.venv/bin/west init -l firmware/trical_bridge

cd firmware
.venv/bin/west update --narrow
.venv/bin/python -m pip install -r zephyr/scripts/requirements-base.txt -r requirements.txt
cd ..

bash firmware/build.sh
```

当前工作区已经完成 `west init` 后，不要重复初始化；日常修改只需再次运行构建命令。
不需要 `west zephyr-export`，构建脚本只为本次进程绑定本地 `ZEPHYR_BASE`，不修改全局 CMake 注册表。

## 构建与输出

默认目标固定为 `promicro_nrf52840/nrf52840/uf2`，保留上游板级的 UF2 分区布局。
Zephyr 固定为 `v4.5.0`；CMSIS、Nordic HAL 和 Segger RTT 的版本由该版本的上游 manifest 固定。
这里采用应用主导的 west 布局，应用与 manifest 由当前 `casbot_calib` Git 仓库管理，不另建嵌套的自有 Git 仓库。

```bash
# 增量构建
bash firmware/build.sh

# 独立 QAS 频率测试 app
bash firmware/build.sh --rate-test

# 配置或依赖改变后清洁重建
bash firmware/build.sh -p always
```

默认输出目录：`firmware/build/trical_bridge/`。

| 文件 | 用途 |
| --- | --- |
| `zephyr/zephyr.uf2` | 匹配 UF2 bootloader 的开发板镜像 |
| `zephyr/zephyr.elf` | 符号、调试和内存分析 |
| `zephyr/zephyr.hex` | Intel HEX 镜像 |
| `zephyr/.config` | 实际生效的 Kconfig 配置 |
| `zephyr/zephyr.dts` | 实际生效的设备树与分区 |

烧录前需核对实际板子的 UF2 bootloader/分区布局。本轮只交付构建，不执行 `west flash` 或拷贝镜像到开发板。
USB VID/PID 暂沿用 Zephyr 的开发默认值 `2fe3:0004`，对外发布固件前须换成获授权的 VID/PID。

## 当前固件行为

- 支持显式扫描、按地址/类型连接三槽 QAS 仪表、GATT 发现和 CCC 确认后订阅。
- 支持 QAS 参数查询/设值和 `MEAS`，两种写入模式明确区分；保留原始通知，不自动清零、恢复出厂或 OTA。
- USB CDC ACM 使用有界非阻塞队列输出 JSONL；日志走 RTT。断开串口会停止实验并断开 BLE，不自动重连。
- 普通应用在 `trical_bridge/`，独立频率测试在 `tests/qas_rate/`，共用 `trical_bridge/lib/`。
- [USB 读写指南](docs/qas_usb.md)提供主机工具和行协议；[频率实验](tests/qas_rate/README.md)提供运行方式、统计含义和旧脚本缺陷分析。

仪表采样时刻、单位/尺度/mastering、三表静态配对尚未实测；本轮未烧录或连接实物，现有 K1–K5/SDK 标定入口保持原行为。
构建不依赖外部协议分析仓库。

## 构建证据

QAS 移植与测试结果见 [QAS_PORT_VERIFICATION.md](QAS_PORT_VERIFICATION.md)；
首版固件构建记录保留在 [BUILD_VERIFICATION.md](BUILD_VERIFICATION.md)。

参考：[Zephyr 应用工作区](https://docs.zephyrproject.org/latest/develop/application/index.html)、
[Pro Micro nRF52840](https://docs.zephyrproject.org/latest/boards/others/promicro_nrf52840/doc/index.html)、
[USB CDC ACM](https://docs.zephyrproject.org/latest/services/connectivity/usb/device_next/cdc_acm.html)。
