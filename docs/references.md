# 资料来源与首版落实

原资料保留在相邻 `casbot_tasks/references/dynamics_identification_2026-10-19/`；新仓库不重复复制四十余 MB PDF。CSV 索引见 [literature.csv](sources/literature.csv)，其中的 `local_pdf` 相对于原资料目录。Robotica 2024 那篇仍需机构订阅，不阻碍首版实验。

| 资料 | 在本项目中的使用 |
| --- | --- |
| [Swevers 1997](https://doi.org/10.1109/70.631234) | 周期多频、重复周期、独立实验；未实现论文完整激励不确定性优化 |
| [Gautier/Briot 2012](https://pagesperso.ls2n.fr/~briot-s/PubliConf/gm_sb_icra12_100212.pdf) | 已知附加惯量提供尺度，单关节联合回归特例 |
| [Panda 2019](https://iris.uniroma1.it/handle/11573/1356777)、[作者代码](https://github.com/marcocognetti/FrankaEmikaPandaDynModel) | 惯性／摩擦参数、物理边界和独立验证；不冒充整臂复现 |
| [辨识综述 2021](https://www.mdpi.com/2076-3417/11/9/4303) | 可辨识性、滤波、偏差；约束／鲁棒损失不能自动消除所有闭环偏差 |
| [摩擦重复性 2021](https://www.mdpi.com/2218-6581/10/1/49) | 冷／热两组摩擦重复实验，无热传导模型 |
| [TriCal 2020](https://doi.org/10.3390/s20205919) | 未来几何＋整体柔顺性；已有 task 版本仅做几何，未实现论文刚度部分 |
| [再激励 2022](https://www.mdpi.com/2076-3417/12/3/1659) | 弦振动状态在重复拨动间持续，接触改变后续响应 |
| [微轨迹 2025](https://vibsys.put.poznan.pl/_journal/2025-36-2/articles/vps_2025205.pdf) | 深度、材料作为条件，不把论文 192 µm 步长当机器人精度要求 |
| [物理一致性 2018](https://arxiv.org/abs/1701.04395) | 多轴后续方向；首版仅正惯量／非负耗散，无 SDP/LMI |

## 当前 task 的来源记录

- `scripts/trical_sphere_calibration/README.md`：evt6 右臂、21 个 joint origin 平移参数；位置约束有规范自由度，仿真 10 µm 噪声不是实机精度。
- `scripts/casbot_dual_arm_model/guitar_strumming_config.yaml`：20251015 模型、500 Hz、100 mm／1 m/s／50 ms；本项目只复用动作规格和代理弦参数。原配置快照见 [legacy_guitar_strumming.yaml](sources/legacy_guitar_strumming.yaml)。
- `scripts/casbot_dual_arm_model/output/source_model_issues.md` 和 `benchmark_model_audit.md`：旧 MJCF 的统一 armature=0.2、damping=0.5 来源未确认，不能当作本项目真值。后来的手部质量修正仍不是实测负载；两版本手臂模型不可混用。

## ENCOS 能力依据与限制

- [厂家下载中心](https://www.encos.cn/xzzx)：已检索到 2026 年更新的调试／第二代关节手册目录，但本轮没有取得完整最新版手册。因此不写入任何官方内部 PI 可调范围。
- [示例驱动源码](https://github.com/THMOS2025/encos-driver)：可核验实现包含 Kp、Kd、目标位置、目标速度、前馈力矩。它是该驱动实现的第一手资料，不能替代具体型号与固件的厂家说明。
- [官方 EtherCAT To CAN 产品页](https://www.encos.cn/productinfo/55395.html?templateId=6217)：说明存在桥接拓扑；不能据此推断每台模组都是原生 EtherCAT，或所有反馈同步可达相同频率。

默认按“输出位置、速度、电流可读，转矩／混合 PD 可发”建接口；内环 PI 和两侧编码器读取能力均不预设。参数辨识的用途是软件前馈、控制选择和有效范围判断，不要求用户能改减速器内部机械参数。

## 构建资料

[pybind11 NumPy 接口](https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html)、[conda-lock](https://conda.github.io/conda-lock/)、[CMake Presets](https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html)、[SciPy least_squares](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)。

## 新增产品研发需求

[机械臂研发需求](<../references/机械臂研发需求(1).pdf>)：七轴S-R-S机械臂的KIN/DYN/PLAN/CTRL需求、阶段与测量口径。逐项来源页、原优先级、验收目标与当前实验的对应关系见 [需求对应表](requirements_matrix.md) 和 [CSV](sources/requirements_matrix.csv)。源文件摘要记录在对应表中；待定产品目标不采用现有合成门槛代填。

## R1刚体计算后端

[Pinocchio官方项目](https://github.com/stack-of-tasks/pinocchio)：本轮按用户指定，采用其C++正/逆动力学、质量矩阵、重力、速度耦合和运动学接口，未保留自实现递归动力学。锁文件固定3.9.0；接口映射、独立二连杆解析校验与七轴几何来源见 [R1说明](arm_r1.md)。

2026-10-27 扩展到 `computeJointTorqueRegressor`、`computeRNEADerivatives`、`computeABADerivatives` 和TCP雅可比；参数顺序/原点惯量与摩擦/转子分离约定见同一说明。有限差分仅用于验证解析导数，后续辨识不再手写整臂刚体回归展开。

[Boost.Odeint官方实现](https://github.com/boostorg/odeint/tree/boost-1.88.0)：统一固定步长RK4与官方Eigen适配，版本来自双平台现有Boost1.88锁记录。事件调度仍由项目维护，后端与验证说明见 [统一积分接口](integration_backend.md)。
