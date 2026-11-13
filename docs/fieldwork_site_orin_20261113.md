# 现场记录：Orin 实机只读接入（2026-11-13）

**状态**：一次**只读**现场会话的事后记录。全程未发送运动指令、未切换任何模式、未修改 Orin 上的任何文件或配置；SSH 只做了读操作。记录中的数值是当次观测，未重复验证，**不构成任何验收资格**（不改变 S1/S3/S4 与 H1–H4 的既有门槛）。

会话结束时现场连接已断开。Orin 侧信息按本次会话的 `ps`、`/proc/<pid>/environ`、`ros2` 查询与现场口述整理；标「未确认」的项不要当作结论。

---

## 1. 网络与接入方式

| 项 | 值 |
| :-- | :-- |
| Orin 地址 | `172.16.0.10/24`，接口 `enP1p1s0` |
| 本机地址 | `172.16.0.20/24`，接口 `enp0s20f0u1u4c2` |
| 连通性 | ICMP 0.28 ms（同交换网段） |
| 登录 | `ssh nvidia@172.16.0.10`（密码方式；凭据不入库） |
| 本机运行环境 | distrobox 容器 `ros2-humble`，`NetworkMode=host`，`ROS_LOCALHOST_ONLY=0` |

现场口述「没有 domain id、可能是 SDK 版本问题」。实测见第 3 节：**Orin 侧所有 ROS 进程都没有设置 `ROS_DOMAIN_ID`，即默认域 0**。

---

## 2. Orin 主机

| 项 | 观测 |
| :-- | :-- |
| 主机名 | `tegra-ubuntu` |
| 内核 | `Linux 5.15.148-rt-tegra #13 SMP PREEMPT_RT Jul 3 2025 aarch64` |
| 用户 | `nvidia`（uid 1000，`sudo` 组）；**机器人全部 ROS 进程以 root 运行** |
| 部署形态 | 无容器（`docker` 命令不存在）：系统 `/opt/ros/humble` + `/workspace` 下多个 overlay |
| `/workspace` 主要目录 | `embed_fusion`、`hl_motion`（含 `hl_motion_pkg`）、`prod_casbot02_basic`、`arm_debug_tool`、`casbot_grasp`、`action_data`、`global_robot.yaml`、`backup_before_wuji_*` |
| 家目录安装包（版本线索） | `prod_casbot02_basic_aarch64_2.3.3_20260828_1829_04aa42.deb`、`embed_fusion_aarch64_2.5.0_20261012_1522_61af59.deb`、`casbot-motion_2.2.36_arm64.deb` |
| `/dev/shm` | 31 G，存在 `fastrtps_*` 段（说明本机参与者使用 SHM 传输） |
| 防火墙 | `iptables` 仅对 `192.168.1.102` 有 DROP（不在本网段）；`nft` filter 默认 accept；未观察到针对 DDS 的限制 |

> ⚠️ 上面的 deb 版本号**只是软件包线索，不是设备序列号/固件指纹**。按既有约定，设备身份门禁仍处于解除状态，不得把版本号或运行时间戳当作设备身份。

---

## 3. ROS 2 环境（Orin 侧）

| 项 | 观测 |
| :-- | :-- |
| 发行版 | Humble（`ROS_DISTRO=humble`，`ROS_VERSION=2`） |
| `ROS_DOMAIN_ID` | **未设置**（抽查 `hlorin`、`casbot02_basic` launch、相机节点、`ota_station` 的 `/proc/<pid>/environ` 均无该键）⇒ 默认 **域 0** |
| `ROS_LOCALHOST_ONLY` | `0` |
| RMW | `/opt/ros/humble/lib` 下只有 `librmw_fastrtps_cpp.so` 等，**没有 CycloneDDS**；两端都是 FastRTPS |
| 发现 | RTPS 组播 `239.255.0.1:7400`，端口基址 7400（域 0），本机探针当时看到约 133 条话题 |

---

## 4. 运行中的进程（会话内 `ps` 快照，启动约 22 分钟前）

| 进程 | 归属/说明 |
| :-- | :-- |
| `/workspace/hl_motion/bin/hlorin` | 运控主进程（`run_hlmotion.bash` 启动） |
| `embed_station`、`ethercat_station`、`ota_station` | `embed_fusion` 的驱动/OTA 站点 |
| `bt_executor` ×2、`resource_library_server`、`shake_hand_node`、`hand_gesture_server`、`casbot_human_perception_node`、`joystick_cmd_sub`、`humanoid_nav_bridge_node`、`crb_collector` | `prod_casbot02_basic`（行为树/应用层） |
| `stereo_camera_node`、`ob_camera_node` ×2、`rslidar_sdk_node` | 相机/雷达 |
| `casbot_arm_calibration_web`、`calibration_web_ui`（`zero_offset_calibration`） | `arm_debug_tool` 网页标定工具 |
| `joystick_node`、`diag_station` | 手柄与诊断 |

以上仅是**进程存在性快照**，没有逐条核对健康状态与配置文件。

---

## 5. `/joint_states` 轴结构与实测频率

- 类型 `sensor_msgs/msg/JointState`，发布者节点名 `hl_orin_core_msg`，`header.frame_id = joint`，**49 轴**：

| 分组 | 数量 | 名称 |
| :-- | :-- | :-- |
| 腿 | 12 | `LJ0`–`LJ5`、`RJ6`–`RJ11` |
| 双臂 | 14 | `left_shoulder_pitch`、`left_shoulder_roll`、`left_shoulder_yaw`、`left_elbow_pitch`、`left_wrist_yaw`、`left_wrist_pitch`、`left_wrist_roll`、`right_shoulder_pitch`、`right_shoulder_roll`、`right_shoulder_yaw`、`right_elbow_pitch`、`right_wrist_yaw`、`right_wrist_pitch`、`right_wrist_roll` |
| 头/腰 | 3 | `head_pitch`、`head_yaw`、`waist_yaw` |
| 手 | 20 | `left/right_thumb_metacarpal`、`thumb_proximal`、`index_proximal/distal`、`middle_*`、`ring_*`、`pinky_*` |

**注意：机器人侧关节名没有 `_joint` 后缀。**

实测接收节奏（`preflight`，约 4.6 s，919 帧）：

| 指标 | 值 |
| :-- | :-- |
| 平均接收频率 | **201.7 Hz**（另一次 6 s 探针 ≈185 Hz） |
| 间隔 p50 / p99 / max | 4.99 ms / 10.55 ms / 22.62 ms |
| 无效位置记录 | 0 |

这与仓库配置里期望的 500 Hz 不一致（见第 8 节待办）。接收频率是**主机回调频率**，不是电机采样率。

---

## 6. 其它已验证话题（同一次会话、加 FastDDS LAN 绑定后）

| 话题 | 类型 | 发布者节点 | 实测 |
| :-- | :-- | :-- | :-- |
| `/joint_states` | `sensor_msgs/msg/JointState` | `hl_orin_core_msg` | ~185–202 Hz |
| `/imu` | `sensor_msgs/msg/Imu` | `hl_orin` | ~174 Hz |
| `/motion/status` | `std_msgs/msg/String` | `hl_orin_motion_node` | ~10 Hz，内容 `mode: READY, state: READY` |
| `/motion/robot_state` | `std_msgs/msg/String`（**不是 crb 类型**） | — | `State: READY, Mode: READY, Remote Control State: Exit, Action Player State: Idle, Teleoperation State: Exit` |
| `/motion/debug/joint_state` | `crb_ros_msg/msg/JointStateData` | `rl_agent_node` | **0 条**（需 `WHOLE_BODY_DEBUG`） |
| `/camera_belly/color/image_raw_mini` | `sensor_msgs/msg/Image` | `/camera_belly/ob_camera_node` | 8.7 Hz |

仅见于 `ros2 topic list`、未订阅验证：`/motion/joint_state`、`/motion/joint_cmd`、`/motion/debug/joint_cmd`、`/upper_body_debug/joint_cmd`、`/upper_body_debug/cmd_vel`、`/joint_data`、`/joint_control`、`/motor_temp`、`/robot/joint_monitor/real_time`、`/robot_status_report`、`/w2_passthrough/*`、`/hru/*`、`/robot_interface/grpc_connection_status`、`/tf`、`/tf_static`。

可见服务：`/get_robot_mode` [`crb_ros_msg/srv/GetRobotMode`]、`/get_robot_state_srv_hl`、`/set_robot_mode`、`/motion/switch_mode`、`/motion/switch_nav_mode`、`/motion/upper_body_debug` [`std_srvs/srv/SetBool`]、`/motion/whole_body_debug`、`/motion/nav_upper_body_debug`、`/motion/update_ee_offset` [`std_srvs/srv/SetBool`]、`/motion_library/sync`。

> 从 Orin 本机调 `/get_robot_mode` 12 秒内未返回应答，原因未确认；**模式判断改用 `/motion/robot_state` 的文本内容**，不作为“服务不可用”的结论。

---

## 7. 关节命名差异（现场“SDK 版本差异”的实际表现）

- Orin 侧 `/joint_states`：`left_shoulder_pitch` 等，**无 `_joint`**。
- 仓库 `configs/vendor_s1.yaml`、URDF 与 SDK 文档示例：`left_shoulder_pitch_joint` 等，**有 `_joint`**。
- 后果：直接用仓库默认 S1 配置跑 `preflight`，937 帧全部被判 `invalid_position_feedback`（`missing_axes`），报告 `failed`。
- 用**站点名**做的临时配置（不改仓库默认）后：`preflight` **7/7 passed**（`artifacts/fieldwork/site_preflight_v5`）。
- **命令侧命名尚未验证**：`/upper_body_debug/joint_cmd` 的 `UpperJointData` 按 SDK 文档使用带 `_joint` 的名字。**发送任何指令前必须先在现场确认命令侧的命名与顺序**，不得照搬反馈侧名字。

---

## 8. 客户端侧根因（本机问题，不是 Orin 问题）

最初的症状是“能发现、收不到任何数据”，一度误判为 Orin 不发布。实际原因在本机：

- 本机有 5 个地址：`lo`、`docker0 172.17.0.1`、`wlan0 192.168.8.3`、`tailscale 100.108.181.104`、`enp 172.16.0.20`。FastDDS 默认把所有接口的 locator 都广播出去。
- 抓包（13 s、20733 个 UDP）显示：**只有内置 SEDP 发现 DATA，没有任何用户数据 DATA，也没有用户 writer 的 HEARTBEAT**；`ros2 topic info -v` 里的节点名全是 `_NODE_NAME_UNKNOWN_`。
- 反向测试：本机以 5 Hz 发布 `/calib_hop_test`，Orin 侧 18 s 收到 **86 条** ⇒ 本机→Orin 方向正常，Orin→本机方向不通。
- 修复：用一个只保留 LAN 地址的 FastDDS XML profile 启动本机节点，用户数据立即恢复，节点名也恢复正常：

| 话题 | 修复前 | 修复后 |
| :-- | :-- | :-- |
| `/camera_belly/color/image_raw_mini` | 0 | 87 条 / 10 s |
| `/joint_states` | 0 | 1114 条 / 6 s |
| `/imu` | 0 | 1044 条 / 6 s |
| `/motion/status` | 0 | 58 条 / 6 s |

**结论：Orin 侧的发布、DDS 域与 QoS 都正常；问题出在本机多网卡导致的 locator 选择。** 因此“改机器人配置/改 domain id”不是正确方向。

### 本次使用的临时文件（未入库，`artifacts/` 被 Git 忽略）

- `artifacts/tmp_hop/lan_only.xml`：FastDDS profile，`UDPv4` + `interfaceWhiteList=172.16.0.20` + `defaultUnicastLocatorList=172.16.0.20`。
- `artifacts/tmp_hop/vendor_s1_site.yaml`：把 `configs/vendor_s1.yaml` 的 14 个轴名去掉 `_joint` 后缀。
- `artifacts/sdk_iface_ws/`：从 `../casbot2-ros2-sdk` 编出的 `crb_ros_msg`（`install/vendor` 里也有一份 10:44 编的）。

复现命令（只读）：

```bash
bash scripts/humble/container.sh env ROS_DOMAIN_ID=0 \
  FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/artifacts/tmp_hop/lan_only.xml \
  bash -c 'source install/vendor/local_setup.bash;
           exec python -m casbot_calib.fieldwork_cli preflight \
             --config artifacts/tmp_hop/vendor_s1_site.yaml \
             --domain-id 0 --scope hardware_read_only \
             --output artifacts/fieldwork/<新目录>'
```

> 注意 `crb_ros_msg` 必须在环境里（`install/vendor/local_setup.bash` 或上面那个 workspace），否则 `state_recorder` 会报 `libcrb_ros_msg__rosidl_typesupport_cpp.so: cannot open shared object file`。用 venv 编 `crb_ros_msg` 会因缺 `empy` 失败，需走系统 Python（`exec.sh --no-venv` + `-DPython3_EXECUTABLE=/usr/bin/python3`）。

---

## 9. 模式与“要不要切模式”

- **收位置/速度反馈不需要切模式**：当前 READY 模式下 `/joint_states` 正常以约 200 Hz 发布。
- **读回生效 Kp/Kd 需要调试模式**：`/motion/debug/joint_state` 当时 0 条，SDK 文档说明该话题需要 `WHOLE_BODY_DEBUG`；GUI 的“Kp/Kd 首帧回读”依赖它，进不去时只能退回配置默认值（`20/2`）。
- 发送五字段指令本身还要申请 `/motion/upper_body_debug`，属于**运动授权**，本次会话未做。

---

## 10. 后续待办（按优先级）

1. **把 FastDDS LAN-only profile 做成一等公民**：新增 `configs/fastdds_lan_only.xml`（或按 `--interface-ip` 生成），并让 `fieldwork/run.sh`/CLI 自动注入 `FASTRTPS_DEFAULT_PROFILES_FILE`，避免每次手写环境变量。
2. **新增站点 S1 配置**（14 个无 `_joint` 轴名），不替换仓库默认；同步说明它与 URDF 命名的映射关系。
3. 跑一次 30 s `observe` 留只读证据；确认约 200 Hz 与间隔分位数。
4. 在**取得现场运动授权后**再验证：调试模式下 `/motion/debug/joint_state` 是否出数据（Kp/Kd 回读）、命令侧 `UpperJointData` 的命名与轴序、以及模式切换后首帧行为。

---

## 11. 明确未验证 / 不得推断

- 实测约 **200 Hz**，与配置里期望的 500 Hz 不一致；**发布/接收时刻不是采样时刻**，仍不满足 S3 采样资格条件。
- effort 尺度与来源（模组估算 Nm）、PD 环实际频率、位置插值细节、腰部固定条件、工具负载、自由测杆包络等仍未在设备上验证；这些字段本次**没有采集**。
- 命令侧关节命名、`/motion/debug/joint_state` 在调试模式下是否可用、模式切换后首帧保持行为，**均未验证**。
- 现场曾观察到 `/joint_states` 的 publisher count 在 0/1 之间波动、节点名显示 `UNKNOWN`：该现象随本机 LAN 绑定后消失；**Orin 本机查询时的波动原因仍未确认**，不据此判断节点健康度。
- 本记录不改变任何既有门槛、协议或现场策略；不把“只读预检 passed”写成实机标定或 S3/S4 资格。

## 12. 同日后续 SHM 只读勘察

后续已通过新网卡 `enp0s20f0u2u2c2`（`172.16.0.20/24`）重新 SSH 接入，核对安装 ELF、实际配置、启动日志及被动共享内存字节快照。35/40槽 ABI 差异、方向/非零零偏、队列初始化副作用、主站墙钟毫秒时间和 CRC 常数等结论见[共享内存接口勘察](jetson_shm_interface_recon_20261113.md)。后续会话的安装包版本与时钟状态单独记录，不回写本页前一次会话的观测。
