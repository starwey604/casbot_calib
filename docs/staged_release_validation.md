# Humble与双臂分阶段动力学：本机完整交付

2026-10-28。本机合成计划M1–M6已实现并通过验收；未连接实机，未完成物理以太网、甲方SDK或ros2_control硬件插件验收。默认命令/控制/ROS反馈发布500Hz，原始设备采样独立配置。

## 本机结果

| 检查 | 结果 |
| --- | --- |
| 原实验回归 | 111项Python、3项C++通过；A–F/R1/旧双臂114项冻结数值按原容差一致 |
| 安装与环境 | Humble/Python3.10、Pinocchio4.0.0、Coal3.0.3、Boost1.74；独立CMake/ament消费者和wheel隔离导入通过 |
| ROS双端Mock | 13组真实本机ROS实验通过协议/冻结模型/故障/录包验收；无真值重分析一致 |
| 默认ROS批次实测 | 发布498.8Hz、采用496.3Hz、接收497.6Hz；指令传输P99 0.2844ms；仅该本机配置筛选 |
| 完整规划/采集 | 764条，全部参考与观测插值碰撞复核通过 |
| 重力 | 可观测秩28/140；独立配对最大RMSE 0.000226915Nm |
| 运动摩擦阶段 | 黏滞/库仑最大合成相对误差1.387%/0.035%；4kHz与8kHz采样收敛通过 |
| 起动摩擦 | 448组；对称等效阈值最大合成相对误差1.129% |
| 惯性 | 刚体回归秩90/140；独立双臂/左臂/右臂最大运动力矩NRMSE 1.883% |
| 腕部专项 | 12条持续同向加速度扫描；独立回归与模型惯量最大差异2.676% |
| M(q)合成真值评估 | 验证范围内对角项最大相对误差3.686%；矩阵最大相对Frobenius误差0.216% |
| M6小范围修正后摩擦 | 黏滞/库仑最大合成相对误差3.154%/0.927%；前序独立验收仍通过 |
| 控制 | 固定PD下前馈降低独立RMSE 77.005%；原模型仅调PD降低26.765%；辨识模型再调PD收益0.414% |
| 积分与能量 | 六类轨迹分别比较最大步长/半步；辨识模型与声明的合成设备模型均低于1%；零输入逐积分步动能耗散通过 |
| 真值隔离 | evaluation目录物理移出会话，新Python进程重分析；算子最大差异0.0，原始数据摘要不变 |
| 只读中文视频 | 880帧，73.33秒，1280×720/12fps/H.264；FK最大差异8.88e-16 |

合成真值只在独立评估程序使用。质量、一阶矩和惯量有不可观测自由度；输出是物理一致代表，不声称逐连杆真实参数唯一恢复。静止反力按已辨识区间验收，不强行解释成唯一零速力矩。全部百分比均为公布配置下的合成结果，不替代产品指标。

## 证据与入口

- [完整机器验收](../artifacts/staged_release/delivery.json)、[交付摘要](../artifacts/staged_release/delivery.md)、[综合复核](../artifacts/staged_release/comprehensive/verification.json)、[统一M1–M6摘要](../artifacts/staged_release/release.json)。
- [73秒中文演示](../artifacts/staged_release/presentation/staged_calibration_zh.mp4)、[九阶段快照及来源摘要](../artifacts/staged_release/presentation/presentation.json)。MuJoCo仅调用几何更新/渲染，不积分、不算动力学或碰撞；角度不放大。
- [物理模型参数包](../artifacts/staged_release/identification_inertia/candidate.json)、[独立预测图](../artifacts/staged_release/identification_inertia/independent_prediction.png)、[惯性报告](../artifacts/staged_release/identification_inertia/report.md)、[四组控制曲线](../artifacts/staged_release/control_comparison.png)。
- [重力](../artifacts/staged_release/identification_gravity/report.json)、[运动摩擦](../artifacts/staged_release/identification_friction/report.json)、[起动阈值分布](../artifacts/staged_release/identification_breakaway/thresholds.csv)、[真值隔离后的独立分析](../artifacts/staged_release/comprehensive/without_evaluation/analysis.json)。
- [原实验回归](../artifacts/staged_release_regression/verification.json)、[ROS报告](../artifacts/staged_release_ros/analysis/README.md)、[ROS机器验收](../artifacts/staged_release_ros/verification.json)、[可安装Linux wheel](../artifacts/staged_release/packages/casbot_calib-0.1.0-cp310-cp310-linux_x86_64.whl)、[wheel独立加载](../artifacts/staged_release/wheel_final/verification.json)。

全流程与单阶段命令见[运行指南](collision_staged_workflow.md)。新输出可直接运行：

```bash
bash scripts/humble/container.sh casbot-calib staged-suite --output artifacts/my_staged --fps 12
```

如需在视频显示实测ROS结果，先用`ros-benchmark`取得本机数据，再加`--ros-report 对应目录/analysis/report.json`。同机回环和虚拟驱动测试在画面与报告中分别标注。

## 数据来源与收紧记录

最终会话为`artifacts/staged_release`。完整新采集先在`staged_release_v1`执行，均使用`coupled_active_set_mass_projection_v2`内核；最终会话仅复用761条条件、参考系数、单位、原始数据与执行指令均校验一致的记录。三个时序试验因积分收敛需要重新以25µs采集，其他机械试验最大步长50µs。复用摘要及源路径保存在`reused_trials.json`，所有阶段重新辨识、验收，不改原始数组。

1kHz电流采样原型因黏滞摩擦混叠被拒绝；现用4kHz原始采样、8kHz对照。曾经误缩小的动态轨迹在规划审计时被隔离，未进入该批采集；现在有最小行程及实际速度/加速度RMS门禁。起动保护读取已送达反馈、保存实际执行指令；耦合停止使用质量矩阵度量的耗散投影。拒绝的碰撞候选和中断的不完整记录均保留。

参考与观测路径均使用Coal保守距离界；参考包括全部进出/过渡/制动/回位，起动阶段还检查位置管道。最小观测采样距离约11.141mm；连续插值的通过条件为保守下界超过10mm，不能把采样最小值当成连续真实运动的下界。

## 接实机仍需确认

- 实际URDF/关节顺序、工具、输出电流—力矩尺度、方向和机械零位；当前软件零位检查不能代替机械/运动学标定。
- 夹具真实几何、固定变换、安装相对重力方向、设备限制及停止策略。当前腰部盒体是未测量代理，未覆盖线缆与整机所有外部环境。
- ROS接口、主机采用时刻、原始采样时钟和电流带宽。4kHz/8kHz原始记录能力未确认；500Hz topic瞬时电流不能自动等价于高带宽原始记录，可另接主机记录/批量回传或独立验证的周期平均值。
- 甲方ros2_control硬件适配、物理以太网频率/抖动、重复批次与独立实机验证。当前Mock节点是wall timer端点；本机单批频率通过不等于硬实时或跨机保证。

Windows保留原离线入口与CI配置；本轮本机仅验证Linux/Humble。论文与模型落实见[参考索引](references.md)和[分阶段算法说明](collision_staged_workflow.md)。
