# 厂家启停脚本现场对照副本

2026-11-14从Orin `/workspace/embed_fusion/bin/` 只读取回，来源为现场安装版本（Orin墙钟1970不作采集日期依据）。未改厂家目录或systemd unit。

- `run_ethercat_station.bash`、`common_function.bash`：原始文件。
- `launcher_stop_excerpt.bash`：原launcher的`process_in_list`、`stop_apps`、`auto_stop`三个完整函数；不包含无关刷机/账户配置，不是完整launcher。

这些文件只供本仓库调试对照，不执行、不自动安装到厂家目录。它们依赖厂家环境、绝对路径和其它脚本，不能直接改个文件名复制运行。

运行入口是仓库自己的`fw prepare` → `scripts/shm/prepare_field.sh` → `casbot_calib.shm_takeover`，实现与现场边界见[接管说明](../../../docs/shm_takeover.md)。继续复用厂家剩余业务停止和主站初始化，避免复制24个模块的生命周期逻辑。

发现：厂家`kill_process`直接SIGKILL，`auto_stop`后台请求systemd停止后仅等待1秒就打印成功。主站二进制的`main`注册SIGINT/SIGTERM/SIGUSR1到`sigint_handler`；本轮只据已找到的SIGTERM退出入口改编排，不修改主站/内核或发SIGUSR1。反汇编证据在本机`artifacts/shm_prepare_hang_20261114/master_signal_path.log`。
