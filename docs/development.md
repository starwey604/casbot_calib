# 开发、复现与设备接入

## 环境和构建

本机 Miniforge 根目录为 `/opt/miniforge`，base 只读；本项目环境位于 `~/.conda/envs/casbot_calib`。没有修改 bashrc、系统编译器或现有项目环境。交互激活需 `source /opt/miniforge/etc/profile.d/conda.sh`；脚本可始终使用绝对路径 `conda run --no-capture-output -n casbot_calib ...`。CMake 编译命令通过 `conda run` 执行时会自动取得激活后的 conda 编译工具链。

`environment.yml` 描述可维护依赖范围，`conda-lock.yml` 固定 linux-64/win-64 的精确版本、下载地址和哈希。Linux `cxx-compiler` 仅通过平台选择器启用；Windows 使用外部 MSVC。更新依赖：

```bash
conda-lock lock -f environment.yml -p linux-64 -p win-64
conda-lock install -n casbot_calib conda-lock.yml
python -m pip install -e . --no-build-isolation
```

Python 代码修改立即生效；**C++ 修改后重新运行 pip install**。`cmake --build --preset dev` 只更新独立 CMake 构建树，不会自动替换 Python 已安装扩展。发布包用 `python -m pip wheel . --no-build-isolation --no-deps`。无 Python 的核心构建：`cmake -S . -B build/core -G Ninja -DCALIB_BUILD_PYTHON=OFF`。

pybind11 在调用前复制整段命令数组到原生结构，运行期间释放 GIL，返回批量 NumPy 数组。不在每个积分步回调 Python，不宣称当前离线模拟满足硬实时。高频实验推荐固定 BLAS 为单线程，避免小矩阵的线程调度成本；科学结果不应依赖线程数。

## C++ 设备契约

`JointDevice` 只有 `capabilities()`、`write(JointCommand)`、`read(JointSample&)`。`MockJointDevice` 缓冲一个实验批次的指令，由 `SimulationRunner` 在虚拟时间内执行后提供反馈，下一实验重置初态；不是逐墙钟实时驱动。Python 批量绑定直接调用相同模拟核心，避免逐条跨语言调用。

后续 SDK 实现 read/write 即可；SDK 自己不提供 `advance()`。真正的实时线程、EtherCAT board／SOEM 适配和无阻塞日志应位于独立设备适配层。本仓库当前没有 SOEM、网络报文或电机使能操作。

接入时需确认并写入能力配置：

- 当前型号、减速比、输出／电机侧单位、绝对零位和编码器量化。`g` 已折算输出侧；不要再次乘速比。
- 转矩指令是 Nm、额定比例还是电流换算，Kp/Kd 量纲、合法范围和固件约束。默认不假定 Ki 或内部 PI 开放。
- 反馈是否有设备采样时间、哪个时钟域、到主机时钟的映射，位置／速度／电流是否同时采样、滤波是否可知。
- 实际工作模式、指令保持策略、更新周期、饱和／错误状态、丢帧与过期指令处理。
- 双编码器开放情况；电流换算力矩始终标记为派生观测。千分表位移可加可选列，现有最小观测模型不依赖它。

接入第一步只录制统一格式的数据，检查单位和时间；第二步验证电流响应及负载尺度；第三步恢复机械参数并做独立轨迹预测；最后才将辨识模型用于前馈与 PD 选择。EtherCAT DC 不是从指令到实际关节运动的完整延迟保证。

## 后续运动学整合

首版 `model_version=output_joint_v1` 明确为合成单关节。未来扩展共享 run_id、joint_names、robot_model_id、时间／单位、工具／负载和观测格式，动力学参数与几何参数分别保存。evt6 TriCal 与 CASBOT 20251015 吉他模型必须使用不同 robot_model_id，不能直接混用标定结果。

先添加多轴数组和真实模型适配，再决定是否引入 Pinocchio 刚体回归矩阵。转子折算惯量、摩擦和驱动增益仍需单独处理，不因为用了 URDF 就认为这些参数可靠。对于可见柔顺性，沿 TriCal 路线使用位移与已知载荷，不需要细化或调整模组内部齿轮。
