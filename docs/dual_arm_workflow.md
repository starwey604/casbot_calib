# nominal双臂动力学标定：首个可运行Mock流程

这是保留的历史回归流程。当前主入口、碰撞门禁、完整物理惯性参数及分阶段验收见[新版流程](collision_staged_workflow.md)；本文件中的固定COM/形状、无碰撞检查和旧频率配置仅对应`dual-calibrate`历史基线。

使用 `casbot_dual_arm_benchmark_nominal.urdf`，腰部固定、左右各七轴。用户图示的抱紧位置记录为腰部前方圆形开口处；精确夹具变换未测量，暂不改原URDF，也不从截图猜坐标。自由空间固定基座关节动力学不依赖固定原点的平移位置，安装相对重力的方向暂沿用nominal。

本轮先跳过实机动力学先验确认与设备限制核对，采用明确的合成设备参数和实验包络。**没有SDK、设备连接或电机下发入口。运动学不标定。**

## 运行

```bash
source /opt/miniforge/etc/profile.d/conda.sh
conda activate casbot_calib
cd ~/codings/casbot_calib
CMAKE_BUILD_PARALLEL_LEVEL=1 python -m pip install -e '.[viewer]' --no-build-isolation --no-deps

# 采集、辨识、独立验证和固定PD前馈对照；不需要MuJoCo。
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 casbot-calib dual-calibrate \
  --config configs/dual_arm_dynamics.yaml --output artifacts/my_dual

# 只读中文视频；不会重新模拟、辨识或读取evaluation真值。
MUJOCO_GL=egl casbot-calib render-dual artifacts/my_dual --fps 8

# 从已保存的观测重新辨识；可删除evaluation/及各批次evaluation_truth.npz。
casbot-calib dual-identify artifacts/my_dual --output artifacts/my_dual_reidentified
```

也可在 `dual-calibrate` 加 `--render` 一次完成，或 `render-dual --snapshots-only` 只生成阶段快照。输出目录必须不存在。Windows使用相同conda锁文件与MSVC，桌面渲染不设置EGL；实际跨平台结果以CI为准。

## 当前实际执行的流程

| 阶段 | 实现 |
| --- | --- |
| 模型 | C++ Pinocchio直接解析随包nominal URDF，保留左右分支和已合并的固定手部惯性；明确的 `tree_rigid_v1` 父索引，不能拼成一条14轴串联链 |
| 方向/软件零位 | 14轴依次正向、反向、回位，C3连续参考；独立合成编码器生产端与待检查映射分开，错误方向或零位声明会导致前置检查失败 |
| 驱动时序 | 14轴纯力矩脉冲（零PD）隔离驱动响应，使用原单轴共享的精确ZOH+一阶模型分别估计时间常数/指令延迟；冻结模型验证2/4ms命令周期，反馈保持1ms |
| 激励设计 | 五组有限傅里叶多频相位候选，比较受限参数空间的缩放回归秩/信息量；解析生成q/dq/ddq/jerk，检查合成速度/加速度/jerk包络 |
| 重力 | 8组不同构型，五秒保持、平滑过渡；仅使用末段且实际速度足够小的观测 |
| 摩擦 | 8秒周期的双向低速运动，固定重力阶段结果先估计黏性/平滑库仑摩擦 |
| 动态 | 6秒周期、六周期训练，舍弃前两周期；每轴总幅值参数0.4～0.6rad，实际行程另报；联合细化惯性比例与摩擦 |
| 验证 | 新相位/谐波的双臂运动、仅左臂运动、仅右臂运动三条独立轨迹；保持同一冻结参数 |
| 控制 | 同一设备、轨迹、初态、PD、驱动配置和随机种子，比较nominal/辨识完整前馈；不调PD、不把PD收益混入模型收益 |
| 交付 | 原始NPZ、命令及解析参考、阶段结果/奇异值、参数JSON/CSV、曲线、独立残差、中文几何回放 |

`ArmSimulationRunner` 在C++内用统一Boost.Odeint推进 `[q14,dq14,i14]`；力学调用独立Pinocchio工作区的ABA，加入一阶电流响应、混合PD与前馈、合成力矩限幅、采样噪声、固定指令/反馈延迟以及可选整帧丢失/重复。整数纳秒积分步会在送达/采样事件处截断，不逐步回调Python。PD视为驱动内部理想状态反馈；观测噪声添加在采样层。

这是批量离线Mock，不是14轴实时驱动。当前多轴时序仅支持声明的共同时钟和固定延迟；未知通道偏移、独立SDK时钟映射/抖动不因单轴版本已有实验就算多轴已验证。

### 对照原calibration_workflow补齐的前置阶段

`direction_jog`逐轴执行“正转→反转→回位”。参考使用七次平滑多项式，q/dq/ddq/jerk在拼接处连续；其他轴保持。示例编码器生产端独立使用131072计数/转、交替方向及不同零位，待检查映射来自配置，不能用错误映射反向生成数据而相互抵消。报告检查归一化回差、方向相关性、正反转实际跨度和回参考位置偏差；**不把软件声明零位或回位动作称为机械零位已标定**。

`driver_timing_train`使用1ms命令周期；所有轴PD为零，输入是已记录的力矩请求，可用于当前响应模型。基线来自nominal重力，叠加平衡正负脉冲；实际机械参数不参与时序估计。随后两条新脉冲序列分别采用2/4ms命令周期，固定采样周期，沿用同一组拟合参数预测电流，不重新拟合。反馈传输时间差由已知采样/接收时钟直接统计，和命令纯延迟、电流响应常数分开报告。

两个前置阶段均有真实C++采集与观测验收，不只是视频标题。软件映射或驱动验证失败会停止后续采集；重辨识和回放会校验前置证据摘要及驱动/映射声明，禁止悄悄改动会话条件。这里的“通过”仅指合成软件检查，实机物理方向/机械零位仍需要独立确认。

### 轨迹与展示调整

旧版幅值参数0.12rad、每阶段只播放3秒，容易看不出完整动作；轴对称末端也使滚转不显眼。新版实际多频轨迹扩大为分轴幅值、周期延长至6秒，保持q/dq/ddq/jerk解析一致及合成包络检查；不是在viewer中放大角度。`motion_coverage.json`记录实际反馈的逐轴跨度。

`storyboard.json`定义八个逻辑阶段、24个数据片段：14轴点动分别展示，驱动1/2/4ms各展示一段，动态/验证覆盖完整周期。默认视频约70秒，各片段显示实际回放倍率；驱动段在几十毫秒窗口慢放，同时画出指令换算电流、采集电流与拟合响应。其余阶段显示目标/采集角度，控制对照显示两组真实位置误差。相机视角按关节选择，半透明躯干、局部轴和不对称末端用于减少遮挡/使滚转可见，不改变关节运动。

## 辨识范围：物理可行的受限模型

完整双臂刚体回归矩阵为 `Y[N,14,140]`，由Pinocchio生成，用于列缩放/SVD和秩诊断。**首版候选没有尝试恢复全部140个原始惯性参数**，实际估计42个量：

- 14个正比例 `s_j`：各运动刚体的质量和COM惯量同时乘以 `s_j`，质心位置与惯量形状固定为nominal。固定手部已包含在对应末端刚体中，仅计一次。
- 每轴黏性摩擦 `b_j>=0` 和平滑库仑摩擦 `fc_j>=0`，共28个。平滑速度固定；本轮不估计Stribeck、温漂或减速器内部参数。

由于质量/惯量按正比例变化，候选天然保留正定性及主惯量三角关系，并由C++边界再次校验。这是一种明确依赖名义COM/形状的限制，不等同于“没有动力学先验”或完整惯性张量标定。nominal仅作开发基线，不冒充实测真值。

先从实际位置和电流按采样时刻拟合周期傅里叶，再解析求速度/加速度；回归特征与电流采用相同频带投影。缺帧通过时间戳回归处理，不跨缺口差分，也不读取模拟ddq或把指令加速度当观测。经过列缩放/SVD后，采用有界鲁棒最小二乘，分阶段输出结果，再用独立整条轨迹验收。

结果同时报告140列原始回归矩阵的秩和42列受限空间的秩；若受限空间秩不足，则标为预测候选，不能宣称每个参数都独立可辨识。包内电流—力矩尺度明确为合成已知增益；改成 `unanchored` 会拒绝绝对参数辨识。本轮没有实现实机14轴增益标定或电流环参数辨识。

通过还要求训练中14轴均达到公布的实际速度/加速度RMS下限。仅一只臂运动的反例会失败，避免列缩放把微小数值波动放大后误报充分激励；这些下限是合成实验验收条件，不是设备性能限制。

## 文件和数据隔离

| 路径 | 内容 |
| --- | --- |
| `session.json` / `source.json` | 观测处理与已知条件、关节顺序、原URDF SHA256和夹具状态；不含模拟机械参数 |
| `direction_jog/raw_telemetry.npz` / `encoder_mapping.json` | 独立生产的合成原始编码器与待检查映射；不是ENCOS编码器规格 |
| `direction_check.json` / `driver_timing.json` / `preflight.json` | 14轴方向/软件零位与时序验收，含输入摘要和会话声明指纹 |
| `driver_timing_*/current_prediction.npz` | 使用同一训练参数得到的电流响应预测，供只读曲线回放 |
| `storyboard.json` / `motion_coverage.json` | 真实记录选段、回放时长/倍率、逐轴实际运动跨度 |
| `excitation_design.json` | 候选相位、信息量、缩放秩、包络峰值；明确未检查碰撞/夹具反力，不标为实机可执行 |
| `*/observations.npz` | 14轴q/dq/current、逐轴状态、序号和采样/接收时间 |
| `*/observation_schema.json` | schema 2、SI单位和同步时钟声明；旧单轴schema 1保持兼容 |
| `*/commands.npz` | 命令N×14×5（q/dq/前馈/Kp/Kd）及解析q/dq/ddq/jerk参考 |
| `*/reference_summary.json` | 逐轴参考速度/加速度/jerk/前馈峰值，明确不是传感器观测 |
| `*/metadata.json` | 原始数据摘要、控制/采样/延迟配置与模拟耗时；读取时核对数据摘要和关节顺序 |
| `evaluation/`、`*/evaluation_truth.npz` | 单独的Mock机械参数及状态/加速度真值，只供评估，可删除 |
| `identification/` | 阶段结果、验证NRMSE、参数包、CSV及曲线 |
| `control_comparison.json` | 逐轴/汇总位置RMSE，参考为共同采样时刻的规划轨迹，两组含相同指令延迟 |
| `presentation/` | H.264中文视频、八阶段PNG、场景/映射和输入不变性/几何核对证据 |

`dual-identify`核对前置验收证据后，只以session、随包nominal和各机械训练/验证观测进行力学辨识，不把命令加速度或evaluation作为力学观测；测试会删除真值并禁止访问evaluation路径，重新辨识必须给出同一结果。驱动时序辨识单独使用纯力矩命令日志与电流采样时间。

显示复用 `StateFrame` / `TelemetryAdapter` / `JointView`，以后可接实机归一化观测。当前画面为nominal骨架的简化几何，夹具仅示意；MuJoCo只调用 `mj_kinematics` / `mj_camlight`，不调用动力学API。生成视频前后校验输入摘要，逐帧与Pinocchio关节FK比较。

树模型的 `fk(q)`返回14个运动关节位姿，不附带虚构的单一TCP；旧串联模型仍保留其工具位姿。树模型不开放旧的单TCP雅可比接口，左右具名TCP与运动学流程后续再接入。

## 当前结果及下一步

本机完整产物见 [双臂运行报告](../artifacts/dual_arm_nominal_release/README.md)、[辨识报告](../artifacts/dual_arm_nominal_release/identification/report.md)、[中文视频](../artifacts/dual_arm_nominal_release/presentation/dual_calibration_zh.mp4)。数值属于公布合成配置，不能转成实机性能承诺。

补齐前置阶段及大幅轨迹的新版产物见 [v2流程](../artifacts/dual_arm_workflow_v2_release/README.md)、[v2驱动时序](../artifacts/dual_arm_workflow_v2_release/driver_timing.json)、[v2实际运动跨度](../artifacts/dual_arm_workflow_v2_release/motion_coverage.json) 和 [v2中文视频](../artifacts/dual_arm_workflow_v2_release/presentation_final/dual_calibration_zh.mp4)。旧产物保留以供对照。

后续可在这一采集/辨识边界接入更一般的惯性基参数与物理可行恢复、实际SDK/尺度与时钟、真实设备限制、碰撞和夹具反力评估。按用户当前要求，这些不作为本轮Mock流程的阻断条件。当前所有结果均标记 `hardware_executable=false`。
