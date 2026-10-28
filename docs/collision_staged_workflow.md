# 分阶段双臂流程与碰撞门禁

本轮入口为 `configs/staged_dual_arm.yaml`。默认命令500Hz、原始采样1kHz、机械积分最大50µs；时序脉冲因粘滑转换的步长收敛需要，单独使用25µs。运动摩擦明确使用4kHz瞬时电流原始采样，并另做8kHz收敛对照，惯量阶段原始采样4kHz，ROS反馈发布默认仍500Hz。旧 `dual-calibrate` 保留为历史回归基线；其无碰撞门禁的轨迹不作为新版执行计划。

## 规划与执行

```bash
bash scripts/humble/container.sh casbot-calib plan-staged --output artifacts/my_plan
bash scripts/humble/container.sh casbot-calib staged-calibrate --output artifacts/my_plan --resume --through gravity
bash scripts/humble/container.sh casbot-calib staged-calibrate --output artifacts/my_plan --resume --through friction
```

也可直接使用 `staged-calibrate --output 新目录`，先规划再采集。`staged-calibrate`依次执行方向/软件零位、驱动时序、重力、运动摩擦、起动摩擦、惯量和四组控制验证。`checkpoint.json`只描述采集/辨识检查点；完整交付以`staged-suite`生成的`delivery.json`为准，其中还要求数值收敛、所有轨迹碰撞复核、删除真值后的新进程重分析和中文视频全部通过。参考轨迹通过不等于全部观测轨迹通过。

每次试验都从共同姿态静止开始，包含接近、测量、制动、回位和结束保持。真实适配器将来必须独立确认开始姿态，不能照搬Mock的状态初始化。前一阶段验收失败会阻止后一阶段。原始NPZ保留，重分析另建目录；合成物理真值单独放在 `evaluation/`，辨识只读 `session.json`、计划和观测。

## 几何与连续运动检查

nominal URDF的45个CAD碰撞网格由Pinocchio导入，SciPy/Qhull生成16个凸外包络；固定躯干/骨盆分别保留，固定手指合并至相应关节。缓存记录URDF和全部网格SHA、几何附着关系、原始顶点到凸包面的最大越界量。凸包包含全部原始顶点，因此也包含它们连接的三角形；运行时无需源STL或Python Pinocchio/Coal绑定。

C++复用Pinocchio Data更新位姿，以Coal计算有符号距离/最近点；释放GIL批量查询。同臂、跨臂、躯干和夹具代理均进入检查。排除表只保留同一刚体、相邻装配界面及源benchmark明确排除的两处腕部壳体重叠，不照搬SRDF所有Never项。相邻装配界面仍依赖正确的关节行程范围，规划通过Bernstein系数包络检查nominal URDF角度范围。

新版不以稀疏采样点“全绿”认定整段无碰撞：

- 参考用SciPy的七次Hermite/Bernstein多项式，导数系数凸包提供全段关节速度上界。
- 每对几何用祖先链长度和外包络半径计算点运动界，共同祖先的刚体运动抵消。
- 以区间中点距离减去两形体运动界作为距离下界。只有下界超过10mm才通过；否则细分，达到深度仍不能证明则拒绝。
- 观测先验收序号/时间/缺失，再检查完整的关节空间分段线性插值。分块包含每个内部观测点的坐标极值，自适应细分不会遗漏块内折返；超过采样间隙限制直接拒绝。

观测检查覆盖的是声明的插值，不证明未观测的实机运动。当前夹具是明确标注的未测量盒体代理，未覆盖URDF以外的头、腿、线缆和环境。因此所有入口均无实机下发权限。

实际规划曾拒绝 `gravity_train_05` 的两种接近：腕部进入腰部代理。默认从确定性候选池补选整组正反接近，保留拒绝报告，不放宽间隙，也不取消这对碰撞检查。当前参考规划764条：14方向、3时序、64重力、210运动摩擦、448起动摩擦、12腕部加速度探针、4整臂/逐臂惯量、5增益选择、4控制验证。所有区间都需有大于10mm的保守下界才通过。

## 新的参数空间

质量、一阶矩和关于关节原点的惯性张量使用Pinocchio标准140参数；不再只调14个固定COM/形状的质量比例。Clarabel求解带4×4伪惯性PSD约束的二次问题，SVD区分可观测子空间与先验选择的零空间；候选再由C++独立检查COM惯量及主惯量三角不等式。静态合成实验为28个可观测组合，不能宣称逐连杆唯一质量/COM恢复。

重力用相反接近的平均回归量和平均电流力矩拟合，报告独立姿态配对误差及单次保持残差、配对半差。运动摩擦冻结重力，在多构型、三档正反向匀速平台按实际位置配对，分离黏滞和库仑项，剔除起停/换向/加速度污染。两种阶段都使用独立整条试验验收。

1kHz瞬时电流原型揭示了一个额外门禁：500Hz零阶保持位置指令的周期内惯性纹波会因固定采样相位混入黏滞系数。即使同采样条件的预测误差很小，系数仍可能有约10%系统偏差。控制周期平均和低通能降低散布，却不能消除已混叠到直流的成分。因此保留并拒绝该批次，新增4kHz主采样和8kHz对照；每轴在同一构型、三档速度分别拟合，两种采样率的参数需在5%内一致，原始瞬时采样还需满足合成驱动的带宽条件。独立1DOF匀速力矩平衡测试已复现1kHz偏差并验证加密采样收敛。这个规则只针对本Mock的瞬时电流传感器，实机必须确认电流滤波、带宽和时间含义；不预设甲方topic能提供4kHz，可由主机原始记录或经独立验证的周期平均观测解决。

起动摩擦Mock已经具有显式粘住/脱离约束，耦合反力由Pinocchio质量矩阵和线性约束求解；不同于零速为零的tanh摩擦。机械冻结且保持指令时，如果整段电流指数响应对应反力均在静摩擦界内，使用其精确静止解跳到下一事件，避免无意义积分；其余运动继续同一Boost.RK4后端。该功能默认关闭，保留旧回归结果。起动力矩斜坡由C++策略读取已送达的位置/速度反馈；持续判据满足后立即停止增加力矩并恢复PD保持，记录计划指令、实际执行指令、确认和控制决策时刻。陈旧反馈、越界、未检测起动等均拒绝该试验。停止速度采用质量矩阵度量下的最小动能投影，避免耦合系统逐轴截断速度引入能量。

物理一致性约束依据 Wensing、Kim、Slotine 的 [LMI惯性参数辨识论文](https://arxiv.org/abs/1701.04395)（IEEE RA-L 2018，DOI 10.1109/LRA.2017.2729659）：使用质量分布二阶矩构成的4×4伪惯性矩阵，而不仅检查惯量正定。本轮未采用论文中的额外CAD椭球支撑约束，也不辨识传动内部参数。

求解器接口参考 [Clarabel问题形式](https://clarabel.org/stable/python/getting_started_py/)、[半正定约束示例](https://clarabel.org/stable/examples/py/example_sdp/)。这里仅组装参数与约束，机器人动力学、回归量和运动学均由Pinocchio计算。


## 起动摩擦与惯量

两种构型 × 14轴 × 正反方向 × 两种静置时间 × 两种力矩斜率 × 两次独立噪声批次，形成448条起动实验。后处理回溯确认前的最后静止/首次运动区间，报告区间、斜率/构型/方向差异与独立重复误差。较晚的持续确认力矩不作为起动阈值。它是当前实验条件下的等效起动阈值，不等同于减速器材料参数。

前序独立验收通过后才允许加速度激励。整臂多频候选按缩放回归矩阵奇异值评分，运动范围、速度/加速度RMS、峰值速度/加速度/jerk及全段碰撞共同门禁；不能靠缩小到几乎不运动而获得安全结果。训练与三条双臂/左臂/右臂验证轨迹种子分离。

腕偏航小惯量另设12条5Hz、0.004rad振幅、0.18rad/s单向偏置扫描，避免零速静摩擦掩盖加速度力矩。不同腕姿态作为训练/验证条件；积分、碰撞和jerk单独按显式合成包络验收。独立验证用实测力矩、加速度、速度及慢变干扰项做单轴回归，不能仅凭整机总力矩误差很小就声称腕部惯量准确。

周期数据对实际位置做40阶傅里叶拟合并解析求导，电流与模型项使用一致投影；非线性摩擦特征先作用于原始速度、再做时间投影。腕部非周期扫描对位置、速度和电流做一致30Hz零相位滤波，再用SciPy样条对测得速度求导。静止轴按已辨识静摩擦区间验收，不能给零速反力强行指定唯一值。

M6冻结重力可观测组合，先固定摩擦拟合惯性，再允许黏滞/库仑参数在预设±5%带内进行一次联合修正；静摩擦保持冻结，随后重新检查重力、运动摩擦与起动重力误差。Clarabel使用QR压缩的显式残差变量和物理尺度变换，避免正规方程平方条件数；PSD约束保留原物理意义。M(q)、重力和速度耦合项始终由Pinocchio生成，不拟合常数惯量矩阵。

控制验证使用原模型/辨识模型 × 原增益/优化增益四组。增益只在另一条轨迹的五档网格中选择，两种模型共享同一所选增益，独立轨迹不参与选择。固定PD的前馈收益与调PD收益分别报告。

## 完整运行与只读回放

```bash
bash scripts/humble/container.sh casbot-calib staged-suite --output artifacts/my_staged --fps 12
# 中断后显式继续；缺少完成metadata的部分记录被保留到interrupted_records，不覆盖原始数据
bash scripts/humble/container.sh casbot-calib staged-suite --output artifacts/my_staged --resume --fps 12
# 独立重分析不读取evaluation/，新输出目录必须不存在
bash scripts/humble/container.sh casbot-calib staged-identify artifacts/my_staged --output artifacts/my_reanalysis
bash scripts/humble/container.sh casbot-calib staged-verify artifacts/my_staged --output artifacts/my_staged/new_verification
bash scripts/humble/container.sh casbot-calib render-staged artifacts/my_staged --output artifacts/my_staged/new_presentation
```

综合验证包含六类轨迹的积分步减半、逐积分步耗散检查、Pinocchio分量加和/质量矩阵正定、全部764条参考与观测插值碰撞复核，以及将整个`evaluation/`物理移走后在新Python进程重新辨识。原始数据、控制命令及前馈模型均校验摘要；不跨丢帧求导。

中文视频只回放记录的实际角度，不放大运动、不参与积分；同帧比较MuJoCo几何位姿与Pinocchio FK，并检查输入摘要未改变。显示未测量夹具代理，区分软件零位与尚待实测的机械零位。完整报告与最终实测数值见[本轮交付记录](staged_release_validation.md)。

## 后续三个P0

[双臂动力学三个P0实施计划](staged_dynamics_p0_plan.md)规定多构型/批次验证、算子稳定性与频段/分量归因。[R0契约](staged_dynamics_p0_r0.md)及[R1开发流程](staged_dynamics_p0_r1.md)已实现：新配置默认规划R1七中心完整参考，并采集train/selection；`--contract-only`仅冻结R0身份。[R2算子稳定性](staged_dynamics_p0_r2.md)、[R3频段归因/选参](staged_dynamics_p0_r3.md)及[R4正式验收](staged_dynamics_p0_r4.md)已实现，本机新正式test、四组控制和中文回放通过；结论只适用于声明的合成范围。
