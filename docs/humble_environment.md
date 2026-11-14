# Humble主开发环境

整体轮次见迁移和标定计划。本入口使用已有Docker后端distrobox `ros2-humble`，与宿主共享仓库绝对路径；不会继承宿主conda环境。原conda环境暂保留作迁移回归。

本机M1已完成，见验收记录。完整自动验证入口：`bash scripts/humble/container.sh bash scripts/humble/validate.sh artifacts/my_humble_validation`，输出目录必须不存在；可追加冻结基线目录作为第二个参数做数值对照。环境准备统一使用下述 `setup.sh`，验证脚本本身不改。

## 安装和构建

**推荐单一入口**，在宿主仓库根目录执行：

```bash
bash scripts/humble/setup.sh --dry-run   # 只打印将执行的命令，不接触容器
bash scripts/humble/setup.sh             # 前置检查 → 系统依赖 → venv → 构建 → 冒烟验证
bash scripts/humble/setup.sh --doctor    # 只体检并输出 JSON，不安装
```

`setup.sh` 按 `preflight → system → python → build → verify` 顺序执行。系统依赖以 root 经 `exec.sh --no-venv` 安装，`.venv-humble` 以当前用户经同一入口创建；只有 `build`/`verify` 才经过要求 venv 的 `run.sh`。因此不再出现「建 venv 的入口先要求 venv 已存在」的自举死锁，`run.sh` 在 venv 缺失时也会直接提示运行 `setup.sh`。

阶段状态保存在 `build/humble-setup/`（`build/` 已被 gitignore）：`stamps/<stage>.ok` 是完成标记，`state.json` 是汇总报告，已完成阶段自动跳过。检测到既有 `.venv-humble` 时 `system`/`python` 会被推断为已完成，**不会重建已有 venv**；要强制重装用 `--force`。

```bash
bash scripts/humble/setup.sh --stage python    # 只装环境，不构建
bash scripts/humble/setup.sh --only build     # 只重跑构建
bash scripts/humble/setup.sh --force system   # 从系统依赖起重跑，并使其后阶段失效
bash scripts/humble/setup.sh --no-skip        # 全部阶段重跑
```

`exec.sh` 是唯一与容器运行时打交道的入口，默认自动探测 `podman` / `docker`（`CALIB_CONTAINER_RUNTIME` 可指定），使用 podman-docker 兼容层时不再输出 `Emulate Docker CLI using podman`；容器名仍由 `CALIB_HUMBLE_CONTAINER` 控制，默认 `ros2-humble`。`container.sh` 已改为它的薄封装，现有 `bash scripts/humble/container.sh <命令>` 调用方式不变。容器创建不在本脚本范围内（Phase 2）；容器不存在时 `preflight` 会打印一次性创建提示并停止。

### 手动分步（等价写法，调试用）

```bash
docker start ros2-humble
docker exec --user root --workdir "$PWD" ros2-humble \
  bash scripts/humble/install_system.sh
docker exec --user "$(id -u):$(id -g)" --workdir "$PWD" ros2-humble \
  bash scripts/humble/setup_python.sh
bash scripts/humble/container.sh python -m pip install -e . --no-deps --no-build-isolation \
  --config-settings=build-dir=build/humble --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON
bash scripts/humble/container.sh cmake --preset humble -DCALIB_BUILD_PYTHON=ON
bash scripts/humble/container.sh cmake --build --preset humble
bash scripts/humble/container.sh ctest --preset humble
bash scripts/humble/container.sh python -m pytest -q
```

前两步**必须**用裸 `docker exec`（或 `exec.sh --no-venv`）绕过 `run.sh`；`install_system.sh` 必须以 root 运行，否则脚本内部退化到 `sudo`，精简镜像里可能没有 sudo。podman 后端可把 `docker` 换成 `podman`，或直接用 `exec.sh` 自动探测。

Python使用`.venv-humble`（系统Python 3.10创建，不启用system-site-packages）；ROS underlay通过显式PYTHONPATH提供rclpy/Pinocchio/Coal，其余Python包在venv内统一安装。不暴露全部系统Python包，避免旧Matplotlib namespace启动文件覆盖venv包。NumPy暂锁定1.x以匹配ROS原生Python扩展；确切版本见 `scripts/humble/requirements.lock`，首次解析来源见 `requirements.in`。原生依赖版本另存验证产物，不通过pip重复安装Pinocchio/Coal。

`CMakePresets.json`兼容Ubuntu自带CMake 3.22。构建不再根据CONDA_PREFIX偷偷插入搜索路径；Humble入口显式使用ROS underlay及独立构建目录。`dev`用于通用原生环境；旧conda对照必须使用`conda` preset显式选择依赖前缀，防止宿主Boost混入。跨工具链禁止共用缓存。

## 独立C++核心与ROS包

```bash
bash scripts/humble/container.sh cmake --install build/humble --component Core
bash scripts/humble/container.sh cmake -S tests/installation -B build/humble-consumer \
  -DCMAKE_PREFIX_PATH="$PWD/install/humble-native;/opt/ros/humble"
bash scripts/humble/container.sh cmake --build build/humble-consumer
bash scripts/humble/container.sh build/humble-consumer/calib_consumer
bash scripts/humble/container.sh bash scripts/humble/build_ros.sh
```

colcon入口明确扫描`ros/`，避免根目录独立CMake项目影响嵌套包发现。ament/rosidl构建辅助脚本显式使用系统Python，以匹配apt安装的catkin_pkg等构建依赖；离线求解/绑定使用同版本系统Python创建的venv。`casbot_calib_core`是ament安装薄封装，编译同一份`cpp/`源码。独立消费者使用 `find_package(casbot_calib CONFIG REQUIRED)`；ROS消费者使用 `find_package(casbot_calib_core REQUIRED)`，均链接 `casbot_calib::core`，不复制算法。Python wheel只安装Python组件，C++开发文件通过Core组件/ament安装。

ROS包包括核心ament薄封装、消息接口与C++双端Mock。Mock端点使用wall timer，尚非ros2_control硬件插件；甲方SDK/topic适配仍待确认。package.xml维护者取自本机Git配置，仓库未声明许可证，发布前应明确许可元数据。

## 实验

```bash
bash scripts/humble/container.sh casbot-calib suite --configs configs --output artifacts/my_humble
bash scripts/humble/container.sh casbot-calib arm-check --output artifacts/my_humble/R1
bash scripts/humble/container.sh casbot-calib dual-calibrate --output artifacts/my_humble/D14 --render
```

原始观测格式、CLI和真值隔离继续复用。命令/观测不通过MuJoCo物理模型。完整EGL测试使用 `bash scripts/humble/container.sh env CASBOT_TEST_RENDER=1 python -m pytest -q`；跨机网络时序、碰撞和新分阶段辨识的验收不能由本轮离线实验代替。
