# Humble主开发环境

整体轮次见[迁移和标定计划](humble_migration_plan.md)。本入口使用已有Docker后端distrobox `ros2-humble`，与宿主共享仓库绝对路径；不会继承宿主conda环境。原conda环境暂保留作迁移回归。

本机M1已完成，见[验收记录](humble_m1_validation.md)。完整自动验证入口：`bash scripts/humble/container.sh bash scripts/humble/validate.sh artifacts/my_humble_validation`，输出目录必须不存在；可追加冻结基线目录作为第二个参数做数值对照。

## 安装和构建

以下在宿主仓库根目录执行，原生依赖安装在容器内：

```bash
docker start ros2-humble
docker exec --workdir "$PWD" ros2-humble bash scripts/humble/install_system.sh
docker exec --user "$(id -u):$(id -g)" --workdir "$PWD" ros2-humble \
  bash scripts/humble/setup_python.sh
bash scripts/humble/container.sh python -m pip install -e . --no-deps --no-build-isolation \
  --config-settings=build-dir=build/humble --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON
bash scripts/humble/container.sh cmake --preset humble
bash scripts/humble/container.sh cmake --build --preset humble
bash scripts/humble/container.sh ctest --preset humble
bash scripts/humble/container.sh python -m pytest -q
```

Python使用`.venv-humble`（系统Python 3.10创建，不启用system-site-packages）；ROS underlay通过显式PYTHONPATH提供rclpy/Pinocchio/Coal，其余Python包在venv内统一安装。不暴露全部系统Python包，避免旧Matplotlib namespace启动文件覆盖venv包。NumPy暂锁定1.x以匹配ROS原生Python扩展；确切版本见 `scripts/humble/requirements.lock`，首次解析来源见 `requirements.in`。原生依赖版本另存验证产物，不通过pip重复安装Pinocchio/Coal。

`CMakePresets.json`兼容Ubuntu自带CMake 3.22。构建不再根据CONDA_PREFIX偷偷插入搜索路径；Humble入口显式使用ROS underlay及独立构建目录。`dev`用于通用原生环境；旧conda对照必须使用`conda` preset显式选择依赖前缀，防止宿主Boost混入。跨工具链禁止共用缓存。

## 独立C++核心与ROS包

```bash
bash scripts/humble/container.sh cmake --install build/humble --component Core
bash scripts/humble/container.sh cmake -S tests/installation -B build/humble-consumer \
  -DCMAKE_PREFIX_PATH="$PWD/install/humble-native;/opt/ros/humble"
bash scripts/humble/container.sh cmake --build build/humble-consumer
bash scripts/humble/container.sh build/humble-consumer/calib_consumer
bash scripts/humble/container.sh colcon --log-base log/humble build --base-paths ros \
  --build-base build/humble-ros --install-base install/humble-ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

colcon入口明确扫描`ros/`，避免根目录独立CMake项目影响嵌套包发现。ament/rosidl构建辅助脚本显式使用系统Python，以匹配apt安装的catkin_pkg等构建依赖；离线求解/绑定使用同版本系统Python创建的venv。`casbot_calib_core`是ament安装薄封装，编译同一份`cpp/`源码。独立消费者使用 `find_package(casbot_calib CONFIG REQUIRED)`；ROS消费者使用 `find_package(casbot_calib_core REQUIRED)`，均链接 `casbot_calib::core`，不复制算法。Python wheel只安装Python组件，C++开发文件通过Core组件/ament安装。

本轮ROS包只安装核心，不是硬件驱动或标定控制器；甲方SDK和topic接口在M2实现。package.xml维护者取自本机Git配置，仓库未声明许可证，发布前应明确许可元数据。

## 实验

```bash
bash scripts/humble/container.sh casbot-calib suite --configs configs --output artifacts/my_humble
bash scripts/humble/container.sh casbot-calib arm-check --output artifacts/my_humble/R1
bash scripts/humble/container.sh casbot-calib dual-calibrate --output artifacts/my_humble/D14 --render
```

原始观测格式、CLI和真值隔离继续复用。命令/观测不通过MuJoCo物理模型。完整EGL测试使用 `bash scripts/humble/container.sh env CASBOT_TEST_RENDER=1 python -m pytest -q`；跨机网络时序、碰撞和新分阶段辨识的验收不能由本轮离线实验代替。
