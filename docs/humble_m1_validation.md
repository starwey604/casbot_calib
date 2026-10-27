# M1：Humble迁移完成记录

2026-10-27。本机已有`ros2-humble` distrobox内完成，Ubuntu 22.04.5、Python 3.10.12、GCC 11.4、CMake 3.22.1。主开发入口见[环境说明](humble_environment.md)，下一轮见[实施计划](humble_migration_plan.md)。

## 已完成

- apt原生Pinocchio **4.0.0**、Coal **3.0.3**、Boost **1.74.0**，没有使用conda编译产物或动态库。系统和Python精确版本、扩展库动态链接清单、基础镜像摘要均记录在产物中。
- 系统Python创建的隔离venv，显式ROS underlay提供rclpy/Pinocchio/Coal。Python额外依赖39项固定在`requirements.lock`；NumPy1.26.4、SciPy1.15.3、pybind11 2.13.6。
- 独立CMake提供头文件、静态库和`casbot_calib::core`导出目标。ament薄封装`casbot_calib_core`编译同一份源码，两种独立消费者均成功构建、链接和运行。
- Python wheel与Core开发组件分开。非editable wheel解包后屏蔽源码重定向及Python Pinocchio/Coal/MuJoCo，仍可加载随包URDF并运行14轴原生重力保持Mock。
- CMake/Humble/conda分别使用显式preset及独立缓存；Humble运行脚本不继承宿主conda。旧conda/Windows入口保留为对照。

## 验收证据

完整产物：[M1验收目录](../artifacts/humble_migration/validation/README.md)、[环境和动态链接](../artifacts/humble_migration/validation/environment.json)、[机器记录及源码摘要](../artifacts/humble_migration/validation/verification.json)、[wheel隔离检查](../artifacts/humble_migration/validation/wheel/verification.json)、[冻结基线数值比较](../artifacts/humble_migration/validation/baseline-comparison.txt)。

| 检查 | 本机结果 |
| --- | --- |
| pytest，包括EGL几何渲染 | 87/87通过、无跳过 |
| CTest | 3/3通过 |
| A–F / R1 / 完整双臂流程 | 全部通过 |
| 与conda冻结基线比较 | 114项数值按已有容差一致，排除墙上耗时 |
| CMake / ament独立消费者 | 均通过 |
| ROS原生模块及动态库隔离 | 通过，未发现conda路径 |
| Coal基础距离查询 | 两个半径0.25m球体、中心距1m，间隙0.5m |
| 新环境双臂中文视频 | 1280×720、12fps、840帧、70秒 |
| 视频关节FK最大分量误差 | 1.11e-15 |

[A–F](../artifacts/humble_migration/validation/experiments/README.md)、[R1](../artifacts/humble_migration/validation/experiments/R1/report.md)、[双臂辨识](../artifacts/humble_migration/validation/experiments/D14/identification/report.md)、[中文视频](../artifacts/humble_migration/validation/experiments/D14/presentation/dual_calibration_zh.mp4)。双臂/仅左/仅右独立预测最大逐轴力矩NRMSE分别0.8737%、0.9130%、0.3564%，与旧合成基准一致。MuJoCo仍只做几何显示。

## 迁移中修复的问题

SciPy1.15的鲁棒最小二乘会原地缩放回调返回的Jacobian。原双臂优化器把残差使用的同一矩阵直接返回，导致残差矩阵被改写；现在返回独立副本。两个独立线性真值测试覆盖全参数及部分参数固定情形，不依赖Mock物理模型。相同失败观测重新辨识通过；原验收门槛未放宽。旧SciPy环境中新增两项测试也通过。

venv不启用system-site-packages，避免系统Matplotlib的namespace启动文件覆盖venv内版本；ROS包通过underlay路径提供。ament构建辅助脚本显式使用系统Python，与apt提供的catkin_pkg等工具匹配。Python绑定开关和pybind11路径明确，防止复用C++专用缓存后遗漏绑定。

CI已新增Humble完整运行及其与原Linux/Windows结果的114项数值比较；该CI修改尚未在远端执行。本轮本机只验证Linux/amd64，不据此宣称Windows、ARM或实机实时性能通过。apt清单/镜像摘要记录本次环境，不等于长期软件包镜像归档。

## 下一轮

M2实现ROS命令/观测契约、双端Mock和频率/延迟实验。甲方整段轨迹能力仍未知，持续topic发送作为预期接入方式，时间戳及指令采用回执按实际能力配置。M3碰撞门禁、M4–M6重力/运动摩擦/起动摩擦/惯量新流程尚待实施；Coal基础距离检查不代表完整机器人碰撞检查已完成。
