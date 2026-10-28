# K3：双臂几何参数辨识

K3在K2三表测量闭环上增加候选姿态设计、可辨识性检查、MDH/Hayati参数化和分层求解。模型仍是完整装配、腰部固定的nominal双臂14轴；左右顺序运动，另一臂保持并记录。当前为合成Mock，不连接实机。完整重复性、外部计量验收和中文MuJoCo视频留在K4，实际仪表/机器人适配留在K5。

## 测量条件与六球工装

K2的四球工装每臂只有两个目标。即使充分改变肘部构型，整条臂绕两球连线的小转动仍不改变球心位置约束。该设计可用于测量状态机，不能据此接受完整几何标定。K3合成工装改为每臂三个非共线球心，共六球；规划和求解均检查目标数及几何秩，不满足时返回`insufficient_noncollinear_metrology_targets`。

六球位置、球架到机器人基座的变换、左右工具安装、三表轴向/符号/零点均被视为**独立测得且冻结的计量输入**。当前输入是明确标记的合成值，不是实际制造或测量结果。若实机只有球间距离、没有球架安装位姿，不能沿用这些锚点；需要重新设计安装规范与相应不可观测方向。三点非共线只是必要的锚点条件，不保证整个机器人参数充分可辨识。

默认每球训练4个构型、验证2个构型，共24训练点和12独立批次验证点。每球先生成12个训练候选与5个验证候选；训练使用带配额的D-optimal选择，验证使用不同随机种子、独立候选池。近重复姿态被拒绝。IK及其雅可比由Pinocchio提供；Coal检查接近、插入、退出、回位及测点间过渡，所有拒绝候选保留。并非直接给求解器生成理想FK标签。

实际采集调用K2同一持久化C++模拟器：500Hz命令、独立1kHz关节采样和50Hz异步三表，经过驱动响应、反馈延迟、量化、噪声、自动对中与稳定窗口后才接受测点。训练、验证是两个独立设备会话，观测噪声种子不同。求解使用原始观测复核后的平均关节位置和三表偏移，**不使用指令位置、不假定读数恰好为零**：

```text
d_sensor = solve(A, mean(z) - z_center)
r = p_world_sensor(q, x) + R_world_sensor(q, x) d_sensor - p_ball_world
```

该残差验证有尺度基准球的位置约束，不测末端姿态精度，也不等同于全空间绝对位置精度。留出批次仍共享工具/球架等系统误差。

## 参数化与Pinocchio边界

采用[TriCal原论文](https://doi.org/10.3390/s20205919)式(1)的顺序：

```text
Rx(alpha) Tx(a) Ry(beta) Rz(theta + q) Tz(d)
```

常规相邻轴使用MDH，接近平行轴使用连续的Hayati扩展，避免公法线造成极大轴向偏移。当前CASBOT相邻轴没有触发Hayati分支；平行和近平行的独立解析模型覆盖该分支，不人为增加当前机器人的beta参数。

`kinematic_parameters.py`只组装常数坐标变换及其导数，将表参数映射回原生关节/body坐标。所有机器人FK、点雅可比及关节放置回归量均调用C++ Pinocchio；不另写Python机器人FK递推。`point_geometry_batch`使用Pinocchio的`computeJointKinematicRegressor`，将观测非零偏移也计入杠杆。批量调用复制输入、释放GIL、复用独占workspace。

固定原始基座及工具规范后，每臂30个候选：安装变换6个，加后六行各`alpha,a,theta,d`。首行固定，其有效首轴相位仅由安装`rz`表示一次。先开放双臂14个有效零位，再考虑其余46个几何候选。这里的“有效零位”属于冻结工具/球架规范下的模型修正，不是已单独测得驱动机械零位。

角参数归一化尺度为1mrad，长度参数为0.2mm；优化变量无量纲，边界±3。条件数仅对这套明确缩放与训练选择后的矩阵有意义。完整候选SVD、数值秩、条件数200对应的实用秩、选中列和冻结先验列分别保存。不把冻结参数标为测量值，不宣称60个物理参数均独立恢复。

提供等价MDH坐标系的Pinocchio模型，连同质心、惯性和固定frame一起换坐标，并检查FK与质量矩阵等价。实际候选几何仍保留原生body坐标及惯性内容，以便后续动力学重验。

## 求解、验收和数据隔离

1. 核对模型、工装、源文件SHA256、原始稳定窗口及训练/验证批次；重分析不得改写采集文件。
2. 训练零位层SVD必须满秩且条件数≤200，否则拒绝。零位层投影后对其余列做带主元QR，逐列保留仍满足条件数的方向。
3. 有界线性最小二乘建立零位基线，再用SciPy有界TRF、`soft_l1`损失依次拟合零位层和几何层，使用解析导数。需求中“LM”的无界实现不能直接满足这里的边界与鲁棒损失，当前明确记录为TRF，尚需与甲方确认产品算法要求。
4. 参数选择与优化只读取训练数据。验证仅用于最终接受/拒绝；改变验证标签不会改变估计参数。默认声明观测尺度2µm、留出位置约束RMSE≤20µm，明显偏离名义模型时还要求至少50%改善；无边界命中、选中矩阵条件数≤200。
5. 这些是合成实验门槛，不是需求0.4/0.7mm的实机验收。报告的条件标准误只考虑声明的观测噪声与所选参数，不包含工装CMM、工具误差、热漂和鲁棒估计的完整不确定度。

`collect_and_identify`采集后将规划和两个采集会话的`evaluation/`移出原位置，在真值不可用时调用辨识，再恢复评估数据并重新核对原始文件摘要。独立`kinematic-identify`入口只需公开配置/观测，没有读取评估目录的路径。隐藏几何和编码器偏置仅进入Mock工厂；真值评分是另一个模块。

为防止隐含几何误差破坏名义路径的碰撞认证，Mock工厂先验证隐藏几何处于声明的全构型刚体位移预算内。递推上界使用每个origin的平移变化与`2 R sin(angle/2)`旋转位移界；每对碰撞体的阈值增加相应位移预算之和。默认每个活动碰撞体0.7mm，另有0.25mrad编码器/观测角包络。该方法适用于当前固定局部形状、只改变刚体放置的族；实际形状改变需要新模型与新认证。K2有限测头代理、接触行程和忽略表杆载荷的局限继续存在。

## 入口与导出

在仓库根目录、Humble环境运行，输出目录须不存在：

```bash
# 全套：规划 → 两个独立K2设备会话 → 辨识 → 单独合成评分/中文图
bash scripts/humble/container.sh casbot-calib kinematic-suite \
  --config configs/kinematics_geometry.yaml --output artifacts/my_k3

# 也可分别执行，便于审阅冻结规划后再采集
bash scripts/humble/container.sh casbot-calib plan-kinematics \
  --config configs/kinematics_geometry.yaml --output artifacts/my_k3_plan
bash scripts/humble/container.sh casbot-calib kinematic-calibrate \
  --plan artifacts/my_k3_plan --output artifacts/my_k3_capture

# 不读取真值的独立重分析
bash scripts/humble/container.sh casbot-calib kinematic-identify \
  --train artifacts/my_k3_capture/train --validation artifacts/my_k3_capture/validation \
  --output artifacts/my_k3_reanalysis
```

输出包括公开规划、候选/拒绝/过渡审计、原始NPZ、不可变采集摘要、参数尺度/状态、训练/验证分层残差、SVD、MDH JSON/CSV、`mount_table.csv`安装修正、候选几何和`candidate.urdf`。首轴相位属于安装表，复用时还需JSON中冻结的基座前缀与frame转换；不能只用14行MDH CSV丢掉安装修正。URDF保留原关节轴、惯性、限位与网格引用，添加无质量TCP；重新导入后对随机姿态核对工具FK与质量矩阵。不复制网格，查看时仍需原模型资源。

**发布几何后必须重新验收碰撞缓存和动力学预测**。导出文件显式标记不可直接执行实机。零位已写入origin，不能再叠加到观测q或重复下发驱动零位。候选模型不自动替换当前默认模型。

## 验证范围

单元验证包含独立平面7R位置解析基准、近平行轴连续性、全部候选几何导数与有限差分（含非零三表偏移）、MDH/原生坐标惯性等价、两球门禁、重复构型秩亏、训练与验证隔离、冻结规划摘要及URDF往返。原A–F/R1/旧双臂回归另行执行；不将历史M2–M6、Windows或实机结果作为本轮验收。

参考实现入口：[参数化](../python/casbot_calib/kinematic_parameters.py)、[选点](../python/casbot_calib/kinematic_design.py)、[辨识](../python/casbot_calib/kinematic_identification.py)、[编排](../python/casbot_calib/kinematic_workflow.py)、[导出](../python/casbot_calib/kinematic_export.py)。

## 本机六球实验记录

本机规划为`artifacts/kinematics_k3_six_ball_plan`，采集/辨识为`artifacts/kinematics_k3_release_v2`。这些目录被Git忽略，新检出不保证存在。102个有效候选、3个拒绝记录中选取24个训练测点和12个独立验证测点；60列候选矩阵数值秩60，但完整条件数约1809.76，因此仅拟合训练选出的44列。最终选中矩阵条件数176.76，未命中边界，余下16个参数固定先验。

| 模型 | 训练位置约束RMSE | 独立验证位置约束RMSE |
| --- | --- | --- |
| 名义模型 | 56.20µm | 58.23µm |
| 仅零位层 | 7.68µm | 7.79µm |
| 几何层 | 0.775µm | 1.627µm |

训练/验证全部36个测点通过K2稳定窗口复核，202段参考/观测插值门禁通过；记录命令频率均500Hz。两会话虚拟运动时间合计423.478s；采集、复核与求解墙上时间约806.43s，其中几何求解约0.10s。规划及渲染另计；虚拟时间不代表实机节拍。

新进程将规划、两个会话与独立评分的真值目录全部移开后，再运行`kinematic-identify`，估计参数与首轮逐位相同，原始文件摘要未改变。证据在`reanalysis_verification.json`，重分析为`artifacts/kinematics_k3_reanalysis`。导出URDF重新导入后，工具位姿最大元素误差`1.22e-15`、质量矩阵最大元素误差`5.00e-16`，属于数值一致性。

独立隐藏真值模块还做了128次离设计空间的静态TCP查询：名义/修正位置RMSE约61.43/20.23µm。它只作外推诊断，没有对应的无碰撞运动证书；比留出球心约束误差更大，也说明不能把1.627µm解释为全空间精度。此结果没有用于重新挑选姿态、参数或验收阈值。

前期`artifacts/kinematics_k3_release`是四球原型，其局部留出残差不能消除两球连线规范歧义；保留作边界记录，最终证据使用上述六球`release_v2`，不覆盖旧数据。

六球会话的原始公开profile保留了采集时继承的K1 `parameter_policy.fit_enabled=false`及旧映射描述；这是当时的元数据缺陷，K3是否执行辨识以独立辨识报告及导出为准。后续K3配置已修正为明确启用几何辨识，K1仍禁止拟合；两种标志下的机器人几何摘要相同。原始会话不为修正文案而重写，已在无真值的新进程中重分析核对参数一致。

最终Humble回归为`artifacts/kinematics_k3_regression_v2`：164项Python测试（含14项K3专项）、4项CTest、原A–F/R1/旧双臂114项冻结数值对照全部通过；独立CMake与ament消费者、禁用editable/Python物理库导入的wheel检查通过。该次运行包括原viewer回归，没有新增K3视频。第一次回归在旧单关节CLI发现同名局部导入冲突，已修复并完整重跑，失败记录保留在`artifacts/kinematics_k3_regression`。本轮没有重跑新版M2–M6完整ROS/阶段采集、Windows、物理网络或实机验收。
