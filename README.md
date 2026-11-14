# CASBOT Calibration

C++动力学/碰撞/模拟核心，Python规划与辨识，以及Orin本机EtherCAT共享内存执行。

## 先选入口

- **[SHM现场工具](fieldwork/shm/README.md)**：C00–C06逐例观测、接管、双臂动作和停止专项，原始帧、时钟与人工观察一起留证。
- **[SHM构建与Git部署](docs/shm_build.md)**：本机Humble与Orin使用同一份源码分别原生编译。
- **[SHM迁移与ROS退役](docs/shm_migration_plan.md)**：当前实现范围及下一步。
- [动力学原理](docs/dual_arm_dynamics_plan.md)、[运动学K1–K5](docs/kinematics_trical_plan.md)、[TriCal固件](firmware/README.md)。

```bash
bash fieldwork/shm/run.sh list
bash fieldwork/shm/run.sh run C03 --joint-id 12  # 仅预览，不连接/发送
```

真实动作需要显式`--execute`，先按现场手册完成支撑、停业务、仅主站和首次握手。当前双臂逐轴用例按用户授权只查URDF位置/参考速度，不查碰撞；H1–H4/K5的原规划规则独立。

## 构建与软件测试

```bash
# Orin或已具备算法依赖的Linux：不需要ROS通信、colcon或Python绑定。
bash scripts/shm/build.sh --with-mock --build-dir build/shm-core \
  --install-prefix "$PWD/install/shm-core" --dependency-prefix /opt/ros/humble

# 本机复用已有Humble/distrobox工具链。
bash scripts/humble/container.sh cmake --preset humble -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=ON
bash scripts/humble/container.sh cmake --build --preset humble
bash scripts/humble/container.sh ctest --test-dir build/humble -L shm --output-on-failure
```

编译安装后可一条命令跑私有物理闭环，不连接机器人：

```bash
bash fieldwork/shm/run.sh run C03 --joint-id 12 --amplitude .01 --segment-s 1 --mock --execute
```

通用参考用`plan --relative`生成、`execute FILE --mock --execute`运行；独立五字段用`fields`生成。详见现场工具手册。当前Mock/通用参考已通过本机Humble与Orin原生编译、安装及私有闭环验证。

Pinocchio/Coal装在`/opt/ros/humble`不意味着SHM程序依赖ROS通信。C++17/CMake3.22为共同基线，x86_64二进制不能直接复制到Orin。

## 离线算法与回放

Pinocchio负责刚体力学与导数，Coal负责碰撞/距离，Boost.Odeint负责积分；MuJoCo只显示几何。

```bash
bash scripts/humble/container.sh casbot-calib --help
bash scripts/humble/container.sh casbot-calib demo \
  --config configs/workflow_demo.yaml --output artifacts/my_demo
bash scripts/humble/container.sh casbot-calib staged-suite \
  --config configs/staged_dual_arm.yaml --output artifacts/my_staged
bash scripts/humble/container.sh casbot-calib render-staged \
  artifacts/my_staged --output artifacts/my_staged_video
```

当前模型为`casbot_hl12p1_dvt2_bare_flange_1105_03_gauge_v2`。模型资产、辨识内核、连续参考/碰撞检查及正式S3/S4/P0数据合同保留。H1–H4与GUI尚在迁移，不能把旧ROS发布stamp或新主站stamp当作驱动采样时刻。

## ROS退役范围

旧ROS stream benchmark、`ros-view`和legacy S2执行链已从main移除；原用法及历史验收在`ros`分支。

剩余SDK/ROS的H1–H4、GUI及TriCal消费者暂留[过渡入口](fieldwork/ros/README.md)，迁到SHM后删除。`fieldwork/run.sh`目前仍转发旧ROS工具，不是SHM入口。不要求每个旧ROS实验都重做SHM版本。

## 目录

- `cpp/`：算法核心、SHM协议/执行、测试。
- `python/casbot_calib/`：规划、辨识、数据与现场编排。
- `fieldwork/shm/`：新现场入口；`fieldwork/ros/`：待退役旧入口。
- `configs/`、`docs/`：冻结配方、模型/场景、使用与证据。
- `scripts/shm/`：原生构建、Git部署、私有联跑。
- `scripts/humble/`：现有本机工具链和离线验证。
- `ros/`：尚有消费者的SDK包与包装，按迁移清单逐批移除。
- `firmware/`：独立TriCal BLE–USB固件工作区。

生成的`artifacts/`、`build/`、`install/`不入Git。离线环境及跨平台用法见[环境说明](docs/humble_environment.md)。
