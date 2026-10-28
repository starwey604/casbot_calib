# K4：重复批次验收与中文只读演示

承接[K3](kinematics_trical_k3.md)。本轮交付同一nominal 14轴双臂、固定腰部、同一合成六球工装下的重复采集/辨识、留出位置约束验收、独立计量输入契约和中文MuJoCo视频。K2/K3已先提交为`cedac82`；K4作为后续独立提交，生成数据不提交。

## 批次与验收口径

- 基准批次是已验收K3会话的24个训练点。K4另启动两个独立的C++设备会话，各重新执行24点完整接近、读表、对中、稳定采样和退出；保持相同物理几何/工装，改变设备/传感器噪声种子。
- 共用K3已有的12点独立设备验证会话。它与三个训练批次都分开，但不是K4新增的未知构型或外测；不宣称验证覆盖整个工作空间。总计三个训练会话72点、一个验证会话12点。
- `prepare`在新采集前写入不可改的`protocol.json`及摘要，冻结K3训练选出的44列、参数规范、源文件摘要和合成阈值。复测使用同一规范和同一参数子集，避免把不同规范/不同冻结先验的原始参数差异误当成漂移。
- 每批仍须通过缩放SVD、边界、独立球心约束等K3门禁。当前合成重复性门槛为：选中参数的无量纲样本标准差最大值≤0.15，公共验证姿态的TCP预测标准差最大值≤20µm；每批位置约束RMSE≤20µm。这些门槛在采集前冻结，不是产品AP/RP或厂家规格。
- 参数离散度采用三个估计的样本标准差（`ddof=1`，仅2个自由度），转换回各自的m/rad后输出CSV。冻结先验参数不记作“测得零方差”。同时在相同观测q下比较三组模型的TCP预测，区分参数表变化和实际预测变化。

该实验只估计**同工装、同物理参数、同温态假设下的条件重复性**，没有拆装、间隔复标或热漂。需求“间隔变化≤2×重复性”需要另外独立的间隔复标数据及确认后的度量，不能把已用于计算标准差的三个样本再充当间隔验收。本轮明确输出`interval_recalibration.status=not_measured`。

## 原始门禁与真值隔离

K4重新核对基准训练、公共验证和两个复测会话的完整原始稳定窗口/参考与观测路径证书，不只读取历史通过标志。门禁报告绑定当前原始manifest摘要；辨识再次验证原始文件SHA256、关节顺序、工装/工具、批次ID与几何身份。重复的批次、不同规范和过期门禁均拒绝。

`finish`将相关`evaluation/`和原K3独立真值评分目录移开，在新的Python进程中运行`kinematic-verify`，随后恢复评估目录。选点、求解、重复性和视频都不读取隐藏几何/编码器偏置；只有设备Mock工厂读取这些参数。采集使用同一持久化`ArmSimulationSession`，未新增机械内核或实机控制循环。

## 外部位置计量接口

当前没有实物外测，默认`external_metrology.status=not_measured`，`product_acceptance_passed=false`。球架计量、工具安装、柔顺/温度等共享系统误差仍不由三批复测揭示。

`kinematic_metrology.py`提供可选`kinematic_external_position_v1`，可接未来独立位置计量结果。目录含只写一次的`observations.npz`、`metadata.json`及双文件SHA256 manifest；保存/读取用`save_meter`/`load_meter`。没有实现真实计量SDK。

| 原始数组 | 形状与单位 |
| --- | --- |
| `q_rad`, `dq_rad_s` | N×14，rad / rad/s，声明关节顺序 |
| `position_meter_m` | N×3，计量设备坐标系下的米 |
| `tool_frame`, `point_id` | N个Unicode，工具与重复测点身份 |
| `joint_sample_ns`, `joint_receive_ns`, `meter_sample_ns`, `meter_receive_ns` | N个int64纳秒，同一已知时钟域 |
| `sequence`, `status` | N个递增int64序号、有效性状态 |

元数据必须含完整SI单位表、独立批次ID、源几何摘要、工具/关节身份、测量标准差和已独立冻结的`robot_from_meter`刚体变换及其来源。禁止用待验收数据重新配准或拟合坐标变换来降低误差；训练批次不能伪装成外测批次。首版只接受已知公共时钟、样本偏移≤1ms和速度≤1mrad/s的静态测量，不对未知异步数据猜测对齐。

每臂至少三个不同姿态、每点至少三次观测。输出位置RMSE/均值/最大值/p95，以及同测点预测残差的样本标准差。默认独立位置RMSE门槛0.4mm、残差重复性0.1mm只是显式软件判据，尚不构成ISO AP/RP试验规程或产品承诺；姿态精度未测量。合成测试覆盖非单位坐标变换、独立噪声、偏置失效和错误单位/时钟/配准的拒绝。

## 中文MuJoCo演示

`kinematic_presentation.py`只读原始关节包和仪表包，按接收时刻取已经送达的样本，保留缺失/无接触/超量程状态。不插值造出采样、不使用指令角冒充观测、不放大角度或几何误差；起点没有反馈的时间明确裁去。

视频包含六球工装与左右臂接近、三表接触、修正与稳定采样、分层辨识结果、独立验证、两批复测和验收边界。微米误差只显示数字/图表，插入的近景属于摄像机缩放；后处理指标明确标注为离线结果。名义采集几何用于显示，微小标定差异不通过夸张网格展示。

复用既有MuJoCo `JointView`和具名关节状态接口，场景包含机器人CAD凸包、固定夹具代理、六球/支杆、左右表架/测头和TCP site。每个视频帧以Pinocchio核对左右TCP位姿，检查未计算`qacc`，最后重新核对全部原始文件摘要。只调用`mj_kinematics`、`mj_camlight`和渲染；不调用`mj_step/mj_forward/mj_inverse/mj_collision`。

## 运行入口

主环境仍为Humble，无新增依赖。输出目录须不存在：

```bash
# 使用已接受的K3会话作为基准，新采两批并复核、隔离真值、输出中文视频
bash scripts/humble/container.sh casbot-calib kinematic-acceptance \
  --reference artifacts/kinematics_k3_release_v2 --output artifacts/my_k4 --fps 12

# 仅从已完成采集及门禁的K4目录重分析；无需真值，不能写入既有analysis
bash scripts/humble/container.sh casbot-calib kinematic-verify artifacts/my_k4 \
  --output artifacts/my_k4_reanalysis

# 只读重新渲染，结果写新目录；可加 --snapshots-only
bash scripts/humble/container.sh casbot-calib render-kinematics artifacts/my_k4 \
  --analysis artifacts/my_k4/analysis --output artifacts/my_k4_video --fps 12
```

可选外测用`--external-meter METER_DIRECTORY`传入。`kinematic-repeat`和`kinematic-reference-gates`是独立进程执行入口，依赖已经冻结的protocol；不自行创建另一套条件。默认完整命令用两个独立进程采复测数据，另一个进程复核基准；目录和实例独占，不并发重建本机绑定或写同一会话。不支持覆盖/自动续跑，失败原始数据和日志保留。

输出包括`protocol.json`、`batches/`、`gates/`、三份完整辨识/候选URDF、`parameter_repeatability.csv`、`validation_residuals.csv`、`repeatability_zh.png`、`presentation/kinematic_workflow.mp4`及阶段快照、`presentation.json`和`delivery.json`。实机适配、实际表杆全行程/负载几何、球架/工具计量及更换几何后的碰撞/动力学重验仍归K5。

## 本机合成结果

本机会话`artifacts/kinematics_k4_release`复用K3原始基准与公共验证，并实际新采两批各24点；三个训练会话共72点、公共验证12点，全部稳定窗口与运动门禁复核通过。三个模型始终使用预先冻结的44列，最终缩放条件数约176.76。

| 批次 | 公共验证位置约束RMSE |
| --- | --- |
| K3基准 | 1.6266µm |
| 新复测一 | 1.5875µm |
| 新复测二 | 1.7502µm |

选中参数最大无量纲样本标准差`0.01827`（门槛0.15）；公共12点的TCP预测样本标准差最大`1.2651µm`（门槛20µm）。原始参数的逐项单位/均值/标准差见`analysis/parameter_repeatability.csv`，冻结先验项不填测得标准差；分批逐点残差见`analysis/validation_residuals.csv`。这些数值仍然是同工装条件重复性，外测、间隔复标与产品验收标志保持未测量/false。

在真值目录移开的新进程中完成三批辨识；回放后全部输入摘要不变。最终中文视频为`presentation_final/kinematic_workflow.mp4`，53s、12fps、636帧，逐帧左右TCP对照Pinocchio的最大矩阵元素误差`1.33e-15`。已查看接近、修正近景、分层辨识与批次重复快照，且核对实际编码帧数/时长；MuJoCo未推进机械计算。初版`presentation/`保留，最终入口使用`delivery_final.json`。

这些目录为被Git忽略的本地产物。K4引用原K3会话而不重复拷贝数据，归档时须保留protocol记录的源会话及摘要；新检出应先运行K3再运行K4，不把文档中的历史数值当成新运行的通过证据。

最终Humble验证记录为`artifacts/kinematics_k4_regression/verification.json`：179项Python测试（含15项K4专项）、4项CTest、原A–F/R1/旧双臂114项冻结数值对照通过，独立CMake/ament消费者与禁用editable及Python物理库导入的wheel检查通过。wheel验证脚本中的函数/局部变量重名已修复，保留`wheel_failed/`与`packages_failed/`，重建wheel后完成其余检查；没有修改原数值容差。本轮未重跑新版M2–M6全部ROS/分阶段采集、物理网络、Windows或实机验收。
