# 本机验证记录

最新主环境验收见 [M1：Humble迁移完成记录](humble_m1_validation.md)：87项Python测试、3项C++测试、114项冻结数值对照通过。以下保留原环境与各轮实现的历史记录。

2026-10-19，Linux、Python 3.12.15、NumPy 2.5.4、SciPy 1.18.1。以下均为本仓库合成参数下的结果，不代表 ENCOS 实机或乐器演奏性能。

完整输出位于本机 [artifacts/release_validation](../artifacts/release_validation/README.md)，含所有原始采集、真值、配置、JSON／CSV 报告和 22 张 PNG。新机器请按 README 重跑；大体积实验输出通过 `.gitignore` 排除。

## 验收结果

| 检查 | 结果 |
| --- | --- |
| CMake / CTest | 编译通过，1 个 C++ 测试程序通过 |
| pytest | 12 项通过，包括独立解析响应、接触能量、单位、时间和真值隔离 |
| A–F 实验 | 六组全部通过；本机总耗时约 18 s，仅供参考 |
| 依赖复现 | Linux 全部 203 个 conda 包版本与双平台锁文件一致 |
| 可安装 wheel | 构建成功；禁用 editable 导入后，隔离导入及 C++ 批量模拟通过 |
| Windows | 已生成 win-64 锁文件与完整 CI，尚未实际执行；不能声称双平台已全部通过 |

## 参数与尺度

| 参数 | 合成真值 | 含摩擦辨识 |
| --- | --- | --- |
| J / kg m² | 0.08 | 0.07996456 |
| b / Nm s/rad | 0.12 | 0.12070481 |
| k / Nm/rad | 3.0 | 2.99802371 |
| fc / Nm | 0.06 | 0.05965531 |

主要参数最大误差 0.587%。公布噪声配置下十个种子全部通过，最差非零参数误差 0.597%。单频负例正确报告秩 2；缺少尺度锚点正确拒绝绝对参数恢复。已知附加惯量的联合拟合得到 `g=2.002107 Nm/A`，真值为 2。

## 时间与模型边界

纯延迟充分采样基线的 12 组组合中，1.7 ms 相对偏移的最大恢复误差为 0.0194 ms。该基线显式采用 20 ms 驱动时间常数，不是 ENCOS 性能断言。

快速驱动时间常数改为 1 ms、反馈随控制周期降低时：

| 反馈周期 | 偏移估计误差 |
| --- | --- |
| 1 ms | 0.0258 ms |
| 2 ms | 0.1460 ms |
| 4 ms | 0.3554 ms |

因此不能只固定通信周期后就默认完成了延迟标定，还要控制采样带宽、通道时间、驱动响应和采样相位。电流测量滤波单独报告，表观相位偏移不等于真实传输延迟。

固定双惯量用同一个简化模型预测，低频／高频独立转矩残差 RMSE 分别为 0.00660 / 0.24122 Nm，展示了有效频带的限制。

## 控制与接触

四组独立轨迹结果：

| 模型 | PD | 跟踪 RMSE / mrad |
| --- | --- | --- |
| 原模型 | 基准 | 6.1267 |
| 原模型 | 网格选择 | 4.4075 |
| 辨识模型 | 基准 | 0.3164 |
| 辨识模型 | 网格选择 | 0.2642 |

固定 PD 时，本合成基准的前馈改善约 94.8%；所有组都满足同一力矩／速度／饱和约束。原模型故意有明显失配，该改善比例不能外推到实机。

单弦任务保留原始电流残差，并通过独立匹配空扫构造附加残差；事件检测不读取力或加速度真值。默认正反向各检测 1 次，重复轨迹检测 6 次，与真值事件数一致。几何接触时段的采样点召回率仍可能只有约 50%，它适合作为实验事件代理，不是精确接触起止或测力算法。接触步长减半后轨迹与峰值指标变化满足 1% 门槛。

可查看 [四组控制图](../artifacts/release_validation/E/four_way.png)、[时间偏移图](../artifacts/release_validation/C/offset_recovery.png)、[接触附加残差图](../artifacts/release_validation/F/contact_residuals.png)。来源哈希、锁文件与版本清单见输出目录 `build_manifest.json` 和 `conda_packages.json`。

## 2026-10-20：中文演示与流程收紧

基线演示代码已先提交为 `8121b29`，后续改动在此基础上验证。最新版产物位于 [calibration_zh_release](../artifacts/calibration_zh_release/README.md)，本轮 A–F 回归位于 [tightened_validation](../artifacts/tightened_validation/README.md)。

- CTest 1/1、pytest 35/35、A–F 六组全部通过。新增拒绝驱动漂移／未声明工况、验收失败保留记录、带符号重力、独立脉冲预测、中文字体／换行检查。
- 十个含噪批次全部通过；1／2／4 ms 控制周期由同一个冻结模型验证。重复性不等于绝对准确度，经验分位数不包含共同尺度误差。
- 中文 H.264 视频为 1280×720、24 fps、984 帧、41 s、2,522,403 字节。八张截图已检查中文显示和布局，渲染前后数据与配置摘要一致。
- Windows 尚未在本机执行；已有双平台构建／实验 CI，Linux viewer CI 增加了 CJK 字体依赖。

| 项目 | 当前合成结果 |
| --- | --- |
| 输出侧增益 | 2.000602 Nm/A |
| 电流时间常数 | 1.998861 ms |
| 指令延迟 | 1.300650 ms |
| 独立脉冲电流 RMSE | 0.200015 mA |
| 冻结机械模型独立残差 | 0.201198 mNm |
| 重力模型独立残差 | 0.192751 mNm |
| 省略重力时独立残差 | 0.156625 Nm |
| 固定 PD 前馈改善 | 85.98% |

重力结果仅用于其他关节固定的单关节工况。实机噪声、固件能力、时钟映射、负载误差与多轴耦合仍需设备与实测数据；此轮结果不授权实机控制。

## R1：Pinocchio七轴统一模型（2026-10-20）

实现与运行入口见 [R1说明](arm_r1.md)，本机产物见 [数值报告](../artifacts/r1_pinocchio/report.json)、[中文快照](../artifacts/r1_pinocchio/arm_geometry.png) 和 [原A–F回归](../artifacts/r1_legacy_validation/README.md)。

- 后端为C++ Pinocchio 3.9.0；RNEA、ABA、CRBA、重力/速度耦合和FK均调用Pinocchio，无自实现递归刚体计算。
- CTest 2/2、pytest 59/59、A–F六组通过；原57项数值与上一版按既定容差一致。Linux/Windows锁文件均已包含Pinocchio；双平台CI执行R1报告并增加4项跨平台数值比较，本机未执行Windows。
- Linux实际260个conda包的精确构建与viewer锁文件一致。磁盘满回滚后缺失的conda history从完好的203条原包记录重建，安装前记录保存在忽略目录 `artifacts/r1_environment_recovery/`；先前历史事件无法据此恢复。
- wheel已构建；屏蔽editable导入后从独立解包路径加载，原生Pinocchio后端、七轴配置和FK参考完整，计算不依赖Python Pinocchio绑定。
- 无图形依赖的核心报告通过；可选渲染按只读几何方式与Pinocchio FK核对。反射转子项/工具只计一次、模型快照、并发读取、错误输入和渲染失败保留证据均有测试。

| 项目 | 本机合成检查结果 |
| --- | --- |
| 二连杆质量矩阵最大绝对误差 | 3.33066907e-16 |
| 二连杆逆动力学最大绝对误差 | 3.55271368e-15 |
| 二连杆解析力矩下ABA加速度最大绝对误差 | 2.97539771e-14 |
| 源URDF几何FK最大分量误差 | 2.77555756e-17 |
| 七轴最小质量矩阵特征值 | 0.000261932662 |
| 七轴最大非对角质量分量 | 0.136187509 |
| 七轴最大速度耦合力矩/Nm | 0.216760416 |
| 微分功—能最大误差/W | 3.67381403e-09 |

七轴质量矩阵最大条件数约1433，是此合成模型的M矩阵性质，不是几何标定辨识条件数，不套用KIN-1的≤200门槛。批量平均计算耗时写入JSON，仅供本机性能观察，不证明DYN-4硬实时。R1尚未采集七轴观测、辨识参数或验收实机性能。

## Pinocchio 使用范围扩展（2026-10-27）

运行入口仍为 `casbot-calib arm-check`。本轮 [完整报告](../artifacts/pinocchio_expansion/report.md)、[参数契约](../artifacts/pinocchio_expansion/regressor_contract.json)、[只读几何快照](../artifacts/pinocchio_expansion/arm_geometry.png)、[性能记录](../artifacts/pinocchio_expansion_benchmark/benchmark.json)、[A–F回归](../artifacts/pinocchio_expansion_legacy_validation/README.md) 均已在本机生成，原产物保留。

- 新增 Pinocchio 刚体力矩回归矩阵、RNEA/ABA解析导数、TCP世界方向雅可比。总模型导数包含显式摩擦及其链式法则，非零转子、偏置工具和旋转基座均有检查。
- 工作区每实例独立持有Data，批内复用；验证交错调用、原引擎销毁后的工作区寿命、多线程批次隔离和模型不可变性。结果仍可能分配内存，不是无分配实时接口。
- Linux构建/安装通过，pytest **70/70**（含EGL渲染）、CTest **2/2**、A–F六组全部通过；原57项数值按既定容差一致。无运行时Python Pinocchio/MuJoCo的核心报告路径仍通过。
- 新增报告检查并接入既有Linux/Windows CI；跨平台比较包含回归恒等式、TCP雅可比、工作区结果和探针矩阵秩等字段。本轮未执行Windows，不将Linux结果视作双平台通过。

| 项目 | 本机合成检查结果 |
| --- | --- |
| 回归矩阵重构力矩最大绝对误差 | 3.55271368e-15 Nm |
| TCP雅可比与FK有限差分最大分量误差 | 1.44627421e-10 |
| 正逆动力学解析导数最大按块归一化误差 | 1.50201127e-9，归一化定义见JSON；不是物理性能指标 |
| 工作区复用后分项力矩最大绝对误差 | 1.77635684e-15 Nm |
| 96个合成探针下缩放回归矩阵秩 | 45 / 70；不代表完整结构可辨识性或已完成标定 |

修改前先冻结4096个输入探针及其六组输出，前后各计时15轮（预热后）。`evaluate_batch`折算每点中位耗时 **5.642 → 1.732 µs**，本次比值约3.26；六组冻结输出逐项完全一致。计时包括Python批量入口与输出分配，属于同机桌面观察，不能推断最坏控制延迟或实机1kHz保证。复查命令：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python scripts/benchmark_arm_workspace.py \
  --baseline artifacts/pinocchio_expansion_baseline --output artifacts/my_workspace_comparison
```

本轮没有推进R2采集或R3参数求解；七轴驱动尺度、基参数恢复、独立观测验证仍按原计划实施。MuJoCo继续仅更新/渲染几何。

## Boost.Odeint统一积分后端（2026-10-27）

接口、时间语义与复用示例见 [积分后端说明](integration_backend.md)，本机 [完整验证](../artifacts/boost_integration_report/report.md)、[A–F结果](../artifacts/boost_integration_validation/README.md)、[整臂内核/几何核对](../artifacts/boost_integration_validation/R1/report.md) 和 [旧积分器冻结数据对照](../artifacts/boost_integration_comparison/report.json) 已生成。

- 原手写RK4四阶段计算已移除，单惯量、双惯量、接触及电流/滤波状态共用 `FixedStepIntegrator`。CMake显式使用Boost CONFIG与 `Boost::headers`；不增加Odeint二进制库。
- Linux安装/构建通过；**pytest 73/73**（含EGL渲染）、**CTest 3/3**、**A–F全部通过**；与Pinocchio扩展版原57项数值按既定容差一致。Windows CI自动执行同一新增测试，本轮未在Windows运行。
- 新增独立指数/振子解析解、RK4四阶收敛、非自治系统阶段时间/截断步长、非法区间/维度、分段恒力矩解析运动、指令/采样同刻顺序及双惯量矩阵指数检查。
- 动态长度Eigen状态通过同一后端与Pinocchio二连杆正动力学组合，验证能量守恒及2ms/1ms步长一致性；这不代表R2七轴采集已实现。
- 四组修改前冻结数据覆盖机械、非整网格时序/噪声/丢包、双惯量和接触。整数/布尔列完全一致，全部浮点列在 `rtol=1e-10, atol=1e-11` 内，最大绝对差异约 **1.45e-13**。未以放宽既有实验门槛通过迁移。
- 每次采集记录实际后端 `boost.odeint.rk4/1.88.0`、最大积分步长和整数事件截断策略；现有NPZ观测列兼容。两套锁文件只更新新直接依赖对应的输入哈希，原468/484条包记录逐项不变。

本轮目标是统一接口与数值可维护性，没有以换库宣称计算加速或硬实时保证。原纳秒调度、驱动/观测模型与MuJoCo几何显示职责保持明确。

## nominal双臂Mock标定流程（2026-10-27）

使用用户指定的nominal双臂版本，腰部按图示前方开口位置抱紧；精确夹具变换未测量，URDF继续采用原固定安装。设备限制和实机动力学先验确认按用户要求暂缓，参数/限制来源均明确为合成配置。实现及命令见 [双臂流程](dual_arm_workflow.md)。

- 已实现C++ Pinocchio树形URDF解析、14轴Boost积分/混合PD/一阶电流驱动/采样与固定延迟、schema 2原始记录、相位候选激励、重力/摩擦/动态辨识及三条独立验证。
- 候选为14个正质量/惯量比例和28个黏性/平滑库仑摩擦参数，COM/惯量形状固定。完整140列回归矩阵只作诊断；不是全惯量参数恢复，不包括运动学或真实SDK。
- **pytest 82/82**（含EGL渲染）、**CTest 3/3**、**A–F全部通过**；原A–F加R1共66项数值按既定容差一致。
- 覆盖原串联接口回归、树形与完整Pinocchio parser一致、理想跨臂块为零、14轴电流响应、能量/步长、缺帧傅里叶导数、删除真值重辨识、未知尺度拒绝、仅一臂训练拒绝全14轴通过、关节顺序/数据篡改拒绝以及MuJoCo禁止动力学API。
- Linux wheel已生成并独立解包，移除editable finder后从wheel加载；屏蔽Python Pinocchio/MuJoCo仍可解析随包URDF并运行14轴重力保持。包不依赖相邻task的mesh或文件布局。记录见 [wheel检查](../artifacts/dual_arm_packages_final/verification.json)。
- 双平台CI已加入双臂完整运行与数值比较；本机只验证Linux，Windows结果尚待CI实际执行。

本机 [完整运行](../artifacts/dual_arm_nominal_release/README.md)、[辨识报告](../artifacts/dual_arm_nominal_release/identification/report.md)、[中文视频](../artifacts/dual_arm_nominal_release/presentation/dual_calibration_zh.mp4)、[回放几何/数据不变性检查](../artifacts/dual_arm_nominal_release/presentation/presentation.json)、[原实验回归](../artifacts/dual_arm_legacy_validation/README.md)。

| 项目 | 公布合成配置结果 |
| --- | --- |
| 完整刚体回归探针秩 | 90 / 140 |
| 当前受限物理参数空间秩 | 42 / 42 |
| 独立双臂轨迹逐轴最大力矩NRMSE | 0.5602% |
| 仅左臂运动验证逐轴最大力矩NRMSE | 0.3293% |
| 仅右臂运动验证逐轴最大力矩NRMSE | 0.4204% |
| 固定PD下nominal完整前馈汇总位置RMSE | 0.00561019 rad |
| 固定PD下辨识完整前馈汇总位置RMSE | 0.000601550 rad |
| 视频192帧的关节FK最大分量误差 | 8.8817842e-16 |

力矩NRMSE为逐轴RMSE除以 `max(rms(电流派生力矩),0.2 Nm)`，所有训练/验证轨迹分批；位置汇总为14轴等权RMS，逐轴结果另存。原始观测未改写；收紧秩/激励门槛后只重新生成派生辨识/渲染，原派生产物保留为 `_initial`。默认基准的Mock物理参数属于当前受限模型族，结果用于验证流程，不证明未知实机COM/惯量形状误差下的性能，也不构成真实设备限制/碰撞/夹具反力验收。

## 双臂方向、时序与轨迹展示补齐（2026-10-27）

新版[完整采集](../artifacts/dual_arm_workflow_v2_release/README.md)、[70秒中文视频](../artifacts/dual_arm_workflow_v2_release/presentation_final/dual_calibration_zh.mp4)、[逐轴行程](../artifacts/dual_arm_workflow_v2_release/motion_coverage.json)、[驱动时序](../artifacts/dual_arm_workflow_v2_release/driver_timing.json)均重新生成，旧版保留。

- Linux全量pytest **85/85**（含EGL渲染）、CTest **3/3**通过。新增独立解析电流响应、14组不同时间常数/延迟、冻结参数跨周期预测、错误方向/零位映射拒绝、C3点动及大幅激励导数检查。
- 动态训练实际逐轴跨度 **25.09°～49.76°**，逐轴点动最小正反转跨度 **50.27°**；全部使用采集角度，未放大渲染。视频为1280×720、12fps、840帧，八阶段、24段记录，动态阶段展示完整6秒周期。
- MuJoCo仍只调用几何更新/相机接口；渲染输入哈希前后一致、不读取评估真值，840帧关节FK最大分量误差 **1.11e-15**。
- 未放宽原有5%独立预测门槛；双臂、仅左、仅右独立验证最大逐轴力矩NRMSE分别为 **0.8737%、0.9130%、0.3564%**。固定PD下前馈对照汇总位置RMSE为 **0.00737468 → 0.00127036rad**，均为合成模型结果。

驱动训练使用1ms命令/反馈周期，14轴辨识电流响应时间常数 **0.9919～1.0013ms**、指令延迟 **1.3474～1.3537ms**；反馈采样至接收时间差 **0.700ms**另报。独立2/4ms命令周期保持1ms反馈，冻结参数预测最大电流RMSE **0.2130mA**，不重拟合。时序估计只使用已记录命令/电流、已知采样时间、声明增益及宽搜索边界，不读取真值时间常数/延迟作为拟合先验。

方向/零位阶段检查独立合成编码器的符号、软件声明零位及回参考动作；物理方向/机械零位仍标记待独立确认。本轮未新增实机驱动或运动学标定，也未执行Windows验证。
