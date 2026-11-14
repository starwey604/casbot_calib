# prepare停机阻塞：由仓库维护接管流程

当前现场旧主站的`ecat_ros_pub`线程卡在`ec_master/ecrt_master_sdo_upload`，systemd已发SIGKILL仍未退出。**先在固定支撑、现场物理停止安排下重启Orin，清除旧线程；不要在这次失败记录上直接activate或start-only。** 正常重启若也受阻，按现场断电流程处理。

新脚本已收口为明确的失败退出；正常SIGTERM路径的实机效果仍待下一次开机验证。

## 新开窗口仍用同一条命令

```bash
cd /home/nvidia/casbot_calib_shm
bash fieldwork/shm/run.sh prepare --execute --output artifacts/本次新目录/setup
```

顺序：

1. 已有服务处于启停中时，保存诊断并结束，不重复接管。
2. 停`zero_calibration.service`和`arm_debug_tool.service`，沿用现场支撑安排。
3. 对旧`ethercat_station`发送SIGTERM，请求其已有退出处理，最多等待15秒；检查全部线程，包括“主线程已Z、其它线程仍D”的情况。超时不升级SIGKILL、不继续停机链或启动新主站。
4. 主站存活线程已退出后，执行厂家`launcher.bash stop`，保留原来的其它业务停止覆盖。厂家若逐项询问，需完成全部停止。该命令最多等120秒；输出直接写日志并转显，后台子进程不会再因持有管道让tee一直等EOF。
5. 核对三个服务已inactive/failed，且无存活`hlorin`/旧主站线程；最多等120秒，每5秒显示等待状态。厂家成功文案不替代实际状态。
6. 通过后才调用厂家主站启动脚本，保存`station.log`。这一步只发起启动，仍由现场检查日志后单独`activate --execute`。

独立服务停止上限100秒；命令超时/中断只结束本次自有调用进程，不重启服务、清驱动故障、reset SHM或卸载内核模块。systemd已接受的停止job可能继续执行，失败后需处理当前状态再新开窗口。

SIGTERM注册已从当前主站二进制确认；它能否避免这次SDO等待，须下一轮现场观察。已证实的脚本缺陷是直接SIGKILL和提前打印成功；内核等待的触发因果仍待厂家源码/数据复核。

## 只读诊断，不停止任何业务

```bash
bash scripts/shm/prepare_field.sh --diagnose artifacts/新的诊断目录
```

只读取服务状态、相关进程/全部线程与可用内核栈，写入本次目录，不访问SHM、不发信号、不启动主站。

## 失败时看哪里

- `takeover/prepare_result.json`：是否完成、是否发起新主站启动、失败原因。
- `takeover/diagnosis.json`、`diagnosis.txt`：服务状态、进程/线程、内核栈。
- `takeover/independent_stop.log`、`stop.log`、`stop.returncode`：各停机步骤的实际结果。
- `takeover/master_exit_confirmed.txt`：调用厂家stop前已无存活旧主站线程。
- `process.json`、`result.json`：fieldwork总结果；Ctrl-C记录为中断，不再以traceback作为正常终端反馈。

`--start-only`保留原入口，只用于已核对的中断后启动步骤，仍核对服务/线程状态；不能用它绕过卡住的旧主站。正常结束不自动恢复ROS业务，接管窗口仍持续到下次Orin重启。

## 原始证据与脚本位置

- 现场用户反馈：执行prepare后“关节全部失力”，厂家输出成功后长期不返回，随后Ctrl-C。只对应这次停业务阶段的观察。
- Orin原记录：`artifacts/shm_fieldwork/site_65837296478673/setup/`，只有stop.log，没有新主站station.log。
- 本机诊断：`artifacts/shm_prepare_hang_20261114/`；D线程TID4523、旧主站PID4507、服务deactivating/stop-sigkill，详见`diagnosis.log`。
- [厂家相关脚本对照副本](../references/vendor/shm_takeover_20261114/README.md)与`python/casbot_calib/shm_takeover.py`。

软件验证使用假服务/假进程状态和自有子进程，覆盖正常顺序、D线程阻塞、后台继承stdout、停止超时、只读诊断及Ctrl-C封存。部署后只执行只读诊断；本次未运行新停机路径、重启、主站启动或运动。

Orin部署后的只读诊断保存在`artifacts/shm_takeover_diagnose_66978184978312/`：服务已转failed，但PID4507仍含D线程4523；`station_start_requested=false`。这次服务状态变化没有消除内核等待，现场恢复安排不变。软件验证日志为本机`artifacts/shm_prepare_hang_20261114/software_tests.log`、`proc_scan_test.log`；进程枚举测试修正了目录顺序假设后单独通过，实际检测仍逐线程判断。
