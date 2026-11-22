# GUI、SHM 接管与开发诊断

调参 GUI 在本机工作站渲染，通过 SSH 连接 Orin 读写 SHM；所有设备操作遵循单 owner 原则，GUI、标定流程（`dynamics.sh`/`kinematics.sh`）与 `calib/dev.sh` 实机指令不能同时占用 `/shm_mc`。

## 本机调参 GUI

首次在本机准备 GUI 依赖与原生执行器（不要在 Orin 安装图形环境）：

```bash
bash scripts/shm/setup_gui.sh
```

启动方式：

```bash
bash calib/gui.sh --mock                      # 本机私有 SHM 物理 Mock
bash calib/gui.sh --host nvidia@172.16.0.10   # 连接 Orin（先在宿主机完成 SSH 认证）
```

- **权限与背景帧**：GUI 连接后默认处于只读监视模式；启用主动控制需满足原生协议 v2，且具备有效的 744 字节手部背景帧（可通过 `--hand-background` 指定，或复用同一开机周期内已缓存的背景帧）。与裸法兰自动标定不同，GUI 下发控制时仍会检查手部背景帧。
- **保存位与录像**：保存的参考位写入 Orin 仓库的 `fieldwork/gui_pose.yaml`。会话录制保存为 `.csv.zst`，支持从 GUI 界面恢复及导出回放视频；切换前旧格式录制请使用切换前版本打开。

## SHM 接管与现场排查（`calib/dev.sh`）

正常执行标定时流程会自动检查并接管 SHM，只有在需要人工逐段检查或接管失败排障时才直接使用 `calib/dev.sh`。其中 `prepare`、`activate`、`hold-body`、`run`、`execute` 默认只作预览，**必须显式加 `--execute` 才会真正作用到设备**：

```bash
bash calib/dev.sh inspect                     # 只读查看 SHM、systemd 服务与进程状态
bash calib/dev.sh prepare --execute           # 停厂家业务与旧主站，拉起 EtherCAT 主站
bash calib/dev.sh activate --execute          # 检查使能就绪状态
bash calib/dev.sh diagnose                    # 打包本机日志与进程诊断到 artifacts/dev/
```

常用开发与调试入口（除 `h1` 外，均使用私有端点、只读串口或离线数据）：

```bash
bash calib/dev.sh h1                                         # 等同 calib/dynamics_h1.sh
bash calib/dev.sh kinematics-mock --mode small               # 私有 Mock 跑通运动学全链
bash calib/dev.sh dynamics-verify-mock                       # 私有 Mock 跑通闭环验收
bash calib/dev.sh trical-probe status --port /dev/ttyACM0    # 探测 TriCal 三表串口与 BLE
bash calib/dev.sh hold-body --execute                        # 单独测试非臂槽位原地保持
bash calib/dev.sh list                                       # 列出底层 C00–C06 fieldwork 用例
bash calib/dev.sh status                                     # 汇总 artifacts/shm_fieldwork 下记录
```

## 双臂 NMPC 遥操作

基于本机 MuJoCo Mocap 交互控制 14 轴裸法兰，叠加标定动力学前馈经 SHM 发送。配置在 [dvt2_teleop.yaml](../configs/nmpc/dvt2_teleop.yaml)，入口为 `scripts/nmpc/setup.sh` 和 `scripts/nmpc/run.sh`（`import` / `build` / `sim` / `mock` / `teleop`），详细步骤与环境要求见[双臂 NMPC 说明](dual_arm_nmpc_plan.md)。
