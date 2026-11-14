# CASBOT Calibration

CASBOT双臂运动学/动力学标定。C++负责动力学、碰撞与1kHz SHM执行；Python负责规划、辨识与现场编排。硬件接口为Orin本机EtherCAT中间件共享内存（SHM）。

## 入口

- [SHM现场工具](fieldwork/shm/README.md)：接管、C系列逐例确认、双臂动作。
- [H1静态重力](docs/shm_h1_a.md)：准备位、碰撞检查参考、重力前馈、静态辨识。
- [调参GUI](docs/shm_gui.md)：本机渲染，经SSH控制Orin。
- [构建与部署](docs/shm_build.md)：本机Humble容器与Orin使用同一份源码分别原生编译。

```bash
bash fieldwork/shm/run.sh list
```

## 构建与测试

```bash
# Orin
bash scripts/shm/build.sh --with-mock --build-dir build/shm-core \
  --install-prefix "$PWD/install/shm-core" --dependency-prefix /opt/ros/humble

# 本机（Humble容器只用于提供Pinocchio/Coal工具链）
bash scripts/humble/container.sh bash scripts/humble/validate.sh
```

## 目录

- `cpp/`：动力学/碰撞核心、SHM协议与执行器、测试。
- `python/casbot_calib/`：规划、辨识、现场编排、GUI。
- `fieldwork/shm/`：现场入口。
- `configs/`：模型场景与现场配方。
- `scripts/shm/`：Orin构建、部署、私有Mock联跑。
- `scripts/humble/`：本机容器工具链。
- `firmware/`：TriCal BLE–USB固件（独立工作区）。

旧ROS/SDK实现及离线研究代码见`ros`分支。`artifacts/`、`build/`、`install/`不入Git。
