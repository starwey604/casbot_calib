# 本机 SHM 调参台

在笔记本运行窗口和渲染；Orin 运行原生 1kHz SHM owner 和完整记录。连接只读，点击“开始控制”才使能双臂；“停止并失能”等待原生应答后回到只读。

## 本机准备与私有演练

复用仓库已有 Humble 图形环境（`imgui-bundle 1.5.2`、MuJoCo、PyOpenGL 和数学绑定）：

```bash
cd /home/ww/codings/casbot_calib
bash scripts/shm/setup_gui.sh
bash fieldwork/shm/run.sh gui --mock
```

窗口打开后点击“连接 / 重连只读”，看到反馈后点击“开始控制”。选左侧真实 ID，在右侧调整五字段；滑条和精确输入都可用。开始会重读实际位置，不发送历史位置草稿。Kp/Kd 可在开始前调整；速度/前馈初始化 0，无隐藏补偿。

本机有完整独立图形 Python 环境时，设置 `CASBOT_GUI_PYTHON=/绝对路径/python` 使用它；否则入口通过既有 Humble 容器运行。缺少图形依赖时先按 `scripts/humble/requirements-gui.lock` 补齐现有环境，setup 不自动更换依赖版本。有硬件 GPU 时不加软渲染参数；确需 Mesa 时显式加 `--software-rendering`。

## 连接 Orin

更新源码后在Orin构建/安装新原生程序（不连接真实SHM）：

```bash
cd /home/nvidia/casbot_calib_shm
cmake -S . -B build/shm-python
cmake --build build/shm-python --target shm_gui_bundle --parallel 2
cmake --install build/shm-python --component Shm
```

窗口只在原生协议v2且已给手部背景时开放控制；旧原生版本仍可只读观察。

先保留当前开机窗口手部背景，按 CLI 流程完成 prepare/activate。已经准备好的同一窗口不重复握手。然后在**笔记本**运行：

```bash
bash fieldwork/shm/run.sh gui \
  --host nvidia@172.16.0.10 \
  --remote-repo /home/nvidia/casbot_calib_shm \
  --hand-background artifacts/本次窗口/before/native/command.bin
```

`--hand-background` 是 Orin 上的路径，只取双手命令字段。未提供时仍能只读连接，但原生拒绝开始控制。腿/腰/头在主动会话中失能，现场支撑继续承担载荷。

启动器先在宿主机终端完成控制/显示/文件三条独立SSH认证（密码方式会分别询问；key/agent沿用现有配置），再将自有Unix socket传给图形容器，避免容器没有控制终端时无法询问密码。此时只执行true，仍未连接SHM；窗口退出清理本次自有连接。主机密钥仍按known_hosts校验。GUI 不安装 SSH 密钥、不保存密码，不启停业务/主站，也不自动清故障或恢复运动。

远端私有演练可加 `--mock`：

```bash
bash fieldwork/shm/run.sh gui --host nvidia@172.16.0.10 --mock
```

这只创建本次独占 `/casbot_calib_test_*`，不连接真实 SHM。GUI Mock 初态显式采用已使能 PD/固定初始前馈，保留物理重力；进入手动后前馈从 0 开始，没有运行时模型补偿。默认 SHM Mock 配置未改变。

## 操作和状态

- 左侧 ID12–18 左臂、ID19–25 右臂。其它电机槽与双手在折叠区域显示原始状态。
- 中间模型由实际反馈驱动；非双臂采用固定模型姿态。左拖转相机，滚轮缩放。实时曲线为显示抽样序列，不作为标定数据。
- 右侧分别显示请求值、编码后入队值和实际反馈。入队回执不等于模组采用或到位；反馈 effort 为估算总输出，不是可复制的前馈。
- 当前接口没有 Kp/Kd 回读；默认 20/2，窗口显示发送值。
- 限制设置支持全轴默认和当前轴覆盖，原生确认后观察范围在线生效；输入范围、控制租约和通信超时在下次控制生效。位置反馈越界只告警，速度可选择结束。修改范围不移动目标；非法目标拒收、不夹取。
- Orin 记录默认每工作区 512MiB 软额度，可用 `--cache-limit-mib` 调整；满后停录告警，不堵住控制。
- GUI 主循环每 100ms 参与租约续期，默认期限 1s。界面卡死、显示连接丢失或静默断网后原生请求失能；重连默认只读。网络失联时界面显示失能状态未知，以远端结果/现场观察核对。

本机工作区默认创建在 `$XDG_STATE_HOME/casbot-calib/shm-gui/`，可用 `--output 新目录` 指定。远端记录路径在窗口显示，默认 `artifacts/shm_gui/`。渲染失败查看本机 `renderer.log`，SSH 问题查看 `control_ssh.log` / `display_ssh.log`，原生问题查看远端 `bridge.log` 及 `native/`。

## 保存、恢复和视频

“保存会话”填写本机新目录，后台在Orin生成完整行/原始帧一致的只读前缀，再通过独立SSH/rsync下载。活动控制不用停；显示和控制不会等待下载。失败后用同一目标重试，`.download.json`和`.partial/`保存已封存远端位置及下载进度；远端原件不删除。窗口关闭先请求退出控制，再取消自有文件任务。

恢复只读查看末次记录位置，可导出视频，不连接、不发送：

```bash
bash fieldwork/shm/run.sh gui --recover /本机/已保存会话
```

视频从快照完整反馈流式读取，按接收时刻20fps ZOH生成，模型/编码在本机。界面可以取消，失败不发布最终mp4。显示曲线的200Hz抽样不参与视频或A/H1分析。

## 采集零位 / 回到保存零位

“采集零位”保存当次实际位置，不改编码器。实机权威文件为Orin仓库`fieldwork/zero.yaml`；私有Mock写本次私有工作区，避免覆盖实机零位。SDK旧零位格式不会自动转换，迁移后先重新采集。

“回到保存零位”仅在只读状态启动：读取本次实际起点，使用既有C3七次参考与Coal连续碰撞/URDF校验，前馈0，然后交给同一owner执行。参考速度上限0.3rad/s、在线反馈速度上限1.5rad/s；PD取各轴当前草稿。规划期间可停止取消，实际起点变化超过0.005rad时原生拒绝且零写入，需要重新点击规划。正常末帧后失能，不添加末位误差/等待停稳验收。

手动启动会等待实际速度低于0.02rad/s，可取消，没有统一10秒截止；原生在真正进入前再次核对新鲜位置/速度。

## 当前交付与证据

G1 原生长期 owner、G2 SSH bridge、G3 本机窗口及G4保存/恢复/零位已实现。Humble 私有原生/bridge/GUI 数值与 ImGui draw-data 检查通过；G2 已在 Orin 原生编译安装，并通过两条独立 SSH 的私有物理 Mock 五字段/正常失能/GUI 不续期失能验证（`artifacts/shm_gui_g2_20261114/orin_remote_v2/verification.json`）。

本轮不运行 GUI 渲染外观验收，实际显示、相机和现场调参由用户核对。30fps、尾延迟和整套 GPU/CPU 指标仍待实际图形环境测量，不将非视觉检查写成性能达标。实际视频编码与窗口外观同样由用户核对；软件已验证流式读取、取消/异常不发布最终文件。

### 最终软件与部署记录（2026-11-14）

本机 `setup_gui.sh` 已实际完成依赖导入、聚合目标构建和安装；Orin `03219a3` 原生聚合构建/安装及8项SHM CTest通过。完整代码按G1 `ab0d515`、G2 `70c91f2`、G3 `acb2369`、G4 `38d9002`提交，后续收尾提交补齐部署与前端反馈。

`artifacts/shm_gui_g4_20261114/final_remote_verification.json`及`container_transport/verification.json`记录：

- 本机→Orin私有物理Mock的v2零写入只读、独立五字段及正常失能；
- 活动控制期间封存/独立下载，CSV行与原始bin帧一一对应，未中断控制；
- GUI主循环继续运行、仅暂停本轮自有两条SSH TCP master时，原生租约失能获应答；恢复传输后命令数没有增加；
- 宿主终端正常认证→共享socket→实际Humble容器的控制/显示/文件三条传输，私有控制/退出及rsync保存通过，退出清理本次连接。

最初暂停SSH mux客户端没有阻断数据，因为fd由TCP master持有；该次注入无效，记录保留，不作为断线证据。后续改为暂停本轮自有专用master完成上述验证，未动用户SSH、网络接口或真实主站。所有开发控制均为私有端点；结束时无GUI owner/Mock残留，真实主站PID11737及开机窗口未改变。

回零已验证原生协议夹具及完整bridge流程；bridge回零专项使用显式零重力测试场景，只验证协议和参考执行，默认GUI Mock仍保留物理重力。视频使用编码器/渲染替身验证失败与取消边界，未生成实屏渲染验收结论。
