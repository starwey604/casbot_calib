# 35 槽安装帧夹具

两个 `.bin` 来自 2026-11-13 Jetson 只读勘察的首次被动快照、槽 0：
`artifacts/jetson_shm_recon_20261113/fixtures/`。只保存协议帧，不包含厂家程序。
采集方法、安装配置和快照一致性边界见 [勘察报告](../../../docs/jetson_shm_interface_recon_20261113.md)。

- 命令：744 字节，frame=869953、count=29、hand_count=2、timestamp=925003、crc=0。
- 状态：696 字节，frame=869944、count=0、hand_count=2、timestamp=937742、crc=65535。
- 预期字段以独立 Python `struct` 小端解码核对后写入 `shm_wire_tests.cpp`。
- 记录中的位置、力矩和模式不是新执行器的默认命令；测试不得重放到实机。

状态 count=0 不被转换为“无轴”，crc 常数不被当有效 CRC。这里测试布局与字节保留，
不测试 IPC 出队、原子一致性、物理采样率或停止行为。
