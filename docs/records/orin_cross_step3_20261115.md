# Orin 交叉构建 Step 3（2026-11-15）

## 运行入口

```bash
bash scripts/shm/cross_build_orin.sh
bash scripts/shm/verify_orin_qemu.sh --pytest
```

一键构建默认运行完整数值冒烟；`--pytest` 在冒烟后继续非实时回归。
ARM64 环境保存在 `build/orin-qemu-venv`，待测 wheel 每轮重新安装；C++ tar
解压到独立临时目录。日志与数值结果自动写入 `artifacts/orin-qemu/<时间>/`。

数值依赖复用 `scripts/shm/requirements-shm.txt`；`scripts/cross/requirements-qemu.txt`
补充 pytest 8.4.2。本轮离线 wheelhouse 共 32 个 wheel。`link_math_bindings.py`
链接同一 Humble prefix 的数学绑定，验证时清空 `LD_LIBRARY_PATH`、`PYTHONPATH`。

## 完整数值栈与安装路径

ARM64/QEMU 实际加载：NumPy 1.26.4、SciPy 1.15.3、PyYAML 6.0.3、orjson 3.13.0、
Pinocchio 4.0.0、Coal 3.0.3、eigenpy 3.13.0、Clarabel 0.11.1、cvxpy 1.7.5、
cma 4.5.0、matplotlib 3.10.0。SciPy 线性求解和 CVXPY/CLARABEL 约束最小二乘通过。

标准 venv 安装暴露了配置根路径问题：原先按包文件的父目录定位仓库，会查找
`venv/lib/python3.10/configs/robot/dvt2.yaml`。新增共享 `project_paths.ROOT`，
由验证入口设置 `CASBOT_PROJECT_ROOT`；源码执行沿用原默认根路径。
模型资源仍从安装包读取。容器内将源码包路径覆盖挂载为已安装包，使 shell
入口设置 `PYTHONPATH` 后也执行相同 ARM64 wheel。

## RNEA、碰撞与线程

使用默认双臂模型，固定随机种子 20261115，32 行、14 轴、502 个碰撞对。
关节位置取 home 加均匀扰动 ±0.2 rad；速度 ±0.3 rad/s， 加速度 ±0.8 rad/s²。
四个独立进程覆盖 1/4 线程与 `_core` 优先/数学库优先两种导入顺序。
RNEA 对照 Python Pinocchio；碰撞距离对照 Python Pinocchio FK + Coal Convex，
同时对照 native 单行调用，最差碰撞对索引要求完全一致。

| 检查 | 最大绝对误差 |
| --- | ---: |
| RNEA 对照 Python Pinocchio | 4.440892098500626e-15 |
| 碰撞距离对照 Python Coal | 0 m |
| RNEA 单/多线程、导入顺序 | 0 |
| 碰撞单/多线程、导入顺序 | 0 m |

全部满足 `< 1e-12`。4 线程进程新增 3 个工作线程，1 线程新增 0 个；
分别让 RNEA 和碰撞首先执行，检查各自启动线程池。C++ OpenMP 探针分别观察到
1/4 个 TID，reduction 均为 499500。

`/proc/self/maps` 中 libomp 唯一路径为 wheel 的 `.libs/libomp.so.5`，同时加载
NumPy/SciPy 的 OpenBLAS。C++ 程序从 tar 的 `lib/libomp.so.5` 加载运行库。
ELF 架构、RUNPATH、SONAME、最高 GLIBC 2.34、许可及 CMake 导出检查通过。

### 消除求解历史差异，保留 Coal 默认收敛设置

初始复用 `ComputeDistance` 会携带求解器内部状态，实测与独立 Python 参考差
1.533e-6 m，单/多线程差 3.378e-7 m。尝试将 GJK/EPA 容差收紧到 1e-12 后，
32 行冒烟通过，但 H3 完整规划暴露反例：基座与左肩 roll 几何相距约 0.255194 m，
该设置返回零距离，原生和 ARM64 均触发碰撞门禁。

最终保留 Coal 的默认 GJK/EPA 容差，每个距离查询使用独立求解器，去除跨姿态、
跨线程的求解状态。原有形状与 FK 资源仍复用。OpenMP 回归中跨求解历史的距离
断言收紧为 `< 1e-12`；新增 H3 反例回归，检查正距离与单/批调用一致性。

## 离线检查与回归

- 7 个 C++ 数值/协议测试全部通过；SHM ABI 为命令 744、状态 696 字节。
- 安装后的 `shm_executor`、`shm_mock_server` 依赖完整，加载 Pinocchio 4.0.0 / Coal 3.0.3。
- `python -m casbot_calib.pipeline.preflight --offline` 通过。
- 原生 Humble 重建核心与 Python 绑定后，CTest 13 项首轮通过；并行构建期间 `shm_physics_cli` 未完成执行，构建结束后单独复测通过（4.90 秒）。14 项均已通过。动力学、碰撞、运动门禁、OpenMP 及 H3 完整规划 Python 回归共 21 项通过（25.89 秒）。
- 宿主脚本与项目根路径单元测试 26 项通过。

QEMU pytest 按用例与 fixture 排除实时 SHM/物理 Mock 调度、6 个原生墙钟阈值
用例、宿主 Docker 编排和依赖 imgui_bundle 的宿主 GUI 绘制；每项原因保存在 `pytest-selection.json`。
最终结果为 **241 passed、81 deselected、1 warning**，耗时 **2155.79 秒（35 分 55 秒）**。
排除明细：50 个实时 SHM/物理 Mock、24 个宿主编排、6 个墙钟阈值、1 个 GUI 绘制。
CVXPY 在 H3 合成惯量辨识中报告一次求解精度 warning，该用例的数值验收断言通过。
原生测试入口和其余断言保持原样。

## 耗时与证据

单独验证时完整数值冒烟耗时 **36.613 秒**；一键构建与原生回归并行时为 45.004 秒。
两轮均高于计划的热环境 8 秒目标。
四个独立进程均计算完整 Python Coal 参考；构建/验证耗时优化在 Step 4 继续处理。

最终数值与 pytest 结果：`artifacts/orin-qemu/20261115T084023.618679Z/`。
一键打包的冒烟结果：`artifacts/orin-qemu/20261115T083827.033240Z/`。
最终一键构建退出码 0，manifest 的 `verification=passed`；总耗时 89.942 秒，
其中验证准备与 QEMU 为 57.827 秒，wheel 构建/打包 18.978 秒。
本轮构建与原生数值回归同时执行。
本轮诊断、构建和原生回归日志：`artifacts/orin_cross_step3/`，包括
`diagnosis-*.log`、`precise-*.log`、`rebuild.log`、`qemu-full.log`、
`qemu-final.log`、`final-package.log`、`native-final.log`、`native-physics-recheck.log`、`native-final-python.log`、`host-tests-final.log`。
