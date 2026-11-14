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

先保留当前开机窗口手部背景，按 CLI 流程完成 prepare/activate。已经准备好的同一窗口不重复握手。然后在**笔记本**运行：

```bash
bash fieldwork/shm/run.sh gui \
  --host nvidia@172.16.0.10 \
  --remote-repo /home/nvidia/casbot_calib_shm \
  --hand-background artifacts/本次窗口/before/native/command.bin
```

`--hand-background` 是 Orin 上的路径，只取双手命令字段。未提供时仍能只读连接，但原生拒绝开始控制。腿/腰/头在主动会话中失能，现场支撑继续承担载荷。

SSH 采用正常主机密钥/密码或 key/agent 认证；密码认证可能在启动终端分别询问两条连接。不要关闭 known_hosts 校验。GUI 不安装 SSH 密钥、不保存密码，不启停业务/主站，也不自动清故障或恢复运动。

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
- 限制设置首版在只读时应用并等待原生确认；位置反馈越界只告警，速度可选择结束。修改输入范围不移动目标；非法目标拒收、不夹取。
- Orin 记录默认每工作区 512MiB 软额度，可用 `--cache-limit-mib` 调整；满后停录告警，不堵住控制。
- GUI 主循环每 100ms 参与租约续期，默认期限 1s。界面卡死、显示连接丢失或静默断网后原生请求失能；重连默认只读。网络失联时界面显示失能状态未知，以远端结果/现场观察核对。

本机工作区默认创建在 `$XDG_STATE_HOME/casbot-calib/shm-gui/`，可用 `--output 新目录` 指定。远端记录路径在窗口显示，默认 `artifacts/shm_gui/`。渲染失败查看本机 `renderer.log`，SSH 问题查看 `control_ssh.log` / `display_ssh.log`，原生问题查看远端 `bridge.log` 及 `native/`。

## 当前交付与证据

G1 原生长期 owner、G2 SSH bridge、G3 本机窗口已实现。Humble 私有原生/bridge/GUI 数值与 ImGui draw-data 检查通过；G2 已在 Orin 原生编译安装，并通过两条独立 SSH 的私有物理 Mock 五字段/正常失能/GUI 不续期失能验证（`artifacts/shm_gui_g2_20261114/orin_remote_v2/verification.json`）。

本轮不运行 GUI 渲染外观验收，实际显示、相机和现场调参由用户核对。30fps、尾延迟和整套 GPU/CPU 指标仍待实际图形环境测量，不将非视觉检查写成性能达标。保存下载/恢复/视频与零位入口按 G4 接续。
