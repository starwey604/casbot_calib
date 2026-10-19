# casbot_calib

单关节输出侧动力学校准实验：**C++ 模拟采集 → Python 时间处理与辨识 → 独立验证 → PD／前馈对照 → 报告**。首版仅使用合成设备，无实机控制入口。

模拟器包含等效惯量、摩擦、外部夹具弹簧、一阶电流响应、通信时序和简化单弦接触。双惯量只生成固定模型失配，不辨识减速器内部参数。`Kp/Kd` 控制增益与物理刚度 `spring` 严格区分。默认数值均不是 ENCOS 实测或厂家保证值。

## 本机运行（bash）

本机已发现 `/opt/miniforge/condabin/conda`，环境名 `casbot_calib`。交互 bash 可这样接入：

```bash
source /opt/miniforge/etc/profile.d/conda.sh
conda activate casbot_calib
cd ~/codings/casbot_calib
python -m pip install -e . --no-build-isolation
cmake --preset dev
cmake --build --preset dev
ctest --preset dev
python -m pytest -q
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 casbot-calib suite --configs configs --output artifacts/my_validation
```

不依赖 shell 初始化的入口：

```bash
/opt/miniforge/condabin/conda run --no-capture-output -n casbot_calib casbot-calib run configs/A_basic.yaml --output artifacts/my_A
```

输出目录必须不存在，防止覆盖原始数据。完整实验通常为几十秒，实际耗时写入报告，不作为验收阈值。

## 新机器与 Windows

```bash
conda install -n base -c conda-forge conda-lock
conda-lock install -n casbot_calib conda-lock.yml
conda activate casbot_calib
python -m pip install -e . --no-build-isolation
cmake --preset dev
cmake --build --preset dev
ctest --preset dev
python -m pytest -q
casbot-calib suite --configs configs --output artifacts/validation
```

Windows 先安装 **Visual Studio 2022 Build Tools 的 C++ 工作负载与 Windows SDK**，在 Developer PowerShell／Developer Command Prompt 中激活 conda 后执行上述命令。conda 不打包 MSVC 编译器。Linux 编译器由锁文件提供；CMake 自动查找 conda 根目录和 Windows 的 `Library` 子目录。也可用 `conda env create -f environment.yml` 开发，但复现实验优先使用已提交的双平台锁文件。

CI 为 Linux、Windows 分别构建、执行 CTest、pytest、A–F 全套实验，并对关键数值按容差比较。**本机只能验证 Linux，Windows 实际结果以 CI 为准**。

## 六组实验

| 配置 | 验证内容 |
| --- | --- |
| [A_basic.yaml](configs/A_basic.yaml) | J/b/k/摩擦、多频、单频秩亏、十个噪声种子、自由关节、冷／热摩擦 |
| [B_gain.yaml](configs/B_gain.yaml) | 空载＋已知附加惯量联合辨识驱动增益；无尺度锚点拒绝恢复绝对参数 |
| [C_timing.yaml](configs/C_timing.yaml) | 1/2/4 ms 周期、固定／随动反馈、指令延迟、电流通道偏移、滤波失配、驱动响应、丢包／重复／抖动 |
| [D_mismatch.yaml](configs/D_mismatch.yaml) | 固定双惯量由单惯量模型拟合，独立低／高频残差 |
| [E_control.yaml](configs/E_control.yaml) | 原模型／辨识模型 × 基准／所选 PD，25 档网格，独立轨迹验收 |
| [F_string.yaml](configs/F_string.yaml) | 空扫、正反拨弦、深度变化、连续再拨；冻结空扫模型分析电流残差 |

单实验：`casbot-calib run configs/F_string.yaml --output artifacts/string_trial`。

每组输出 `report.json`、`report.md`、`metrics.csv` 和 PNG。每次采集独立保存 `observations.npz`、`commands.npz`、`observation_schema.json`、`metadata.json`、`evaluation_truth.npz`；原始数据不被预处理覆盖。单位通过独立 schema 验证，辨识函数和数据加载器不会读取真值文件或含真值的 metadata。

仅从观测重新辨识（删除真值仍可运行）：

```bash
casbot-calib identify artifacts/validation/A/friction --config configs/A_basic.yaml --output artifacts/reidentified.json
```

频率／延迟结果需结合采样带宽解释：快速电流响应配合稀疏同步采样，会产生有偏的表观偏移。C 实验同时报告充分采样基线与快速驱动压力场景，不能把所有相位误差解释为链路延迟。

## 代码与后续接入

| 位置 | 职责 |
| --- | --- |
| `cpp/include/casbot_calib/simulator.hpp` | 设备协议、能力、批量 Mock、模拟配置 |
| `cpp/src/simulator.cpp` | 虚拟时钟、RK4、驱动／传感器／链路／接触 |
| `cpp/src/bindings.cpp` | 整段 NumPy 输入输出；模拟期间释放 GIL |
| `python/casbot_calib/identification.py` | 傅里叶解析求导、频带投影、SVD、约束鲁棒拟合、时序与增益辨识 |
| `python/casbot_calib/experiments.py` | A–F 编排、独立验证、控制选择 |
| `python/casbot_calib/data.py` | NPZ、JSON、SI 单位、原始数据摘要 |
| `scripts/compare_reports.py` | 跨平台关键数值比较，排除墙钟耗时 |

后续 SDK 只实现 C++ `JointDevice` 的 `write/read/capabilities`。`SimulationRunner` 持有虚拟时间推进能力；真实 SDK 不必模拟时间。先保存相同观测格式，再接现有 Python 流程。默认只有输出侧位置／速度／电流；由电流换算的力矩不是独立传感器。内环 PI 可调性和实际增益范围等待型号／固件资料确认。

详细见 [模型与参数](docs/model_and_parameters.md)、[实验协议与验收](docs/experiments.md)、[资料来源](docs/references.md)、[开发与 SDK 接入](docs/development.md)。本轮实测结果见 [验证记录](docs/validation.md)。
