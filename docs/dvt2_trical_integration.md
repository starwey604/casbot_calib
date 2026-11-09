# 用实际四球装配运行K2–K4

默认K2–K4或使用`configs/dvt2/kinematics.yaml`可以在同一套三表采集、几何辨识和复测代码中选择1105_03装配。球架位置和平台高度沿用N2冻结方案；同一工具先装左腕（相对STEP +90°），双臂回位停稳后再人工换装右腕（0°），另一臂始终裸法兰停放。

本入口是原生Mock实验。球位、工具安装和测量轴使用CAD注册，仪表零位/量程为合成配置；真实mastering、工具负载与自由测杆全行程外形还需现场核验。

## 先规划，再采集和辨识

在仓库根目录执行，输出目录必须不存在：

```bash
# K2：先演练第一个测点，检查接近、读表和退出
bash scripts/humble/container.sh casbot-calib trical-mock \
  --config configs/dvt2/kinematics.yaml --first-only --output artifacts/my_dvt2_k2
bash scripts/humble/container.sh casbot-calib trical-analyse \
  artifacts/my_dvt2_k2 --output artifacts/my_dvt2_k2_check

# K3：四球×左右工具，选择不同训练/验证姿态
bash scripts/humble/container.sh casbot-calib plan-kinematics \
  --config configs/dvt2/kinematics.yaml --output artifacts/my_dvt2_kin_plan
bash scripts/humble/container.sh casbot-calib kinematic-calibrate \
  --plan artifacts/my_dvt2_kin_plan --output artifacts/my_dvt2_kin_calibration

# K4：冻结参数子集，另采两批，与基准批次共做三批复测
bash scripts/humble/container.sh casbot-calib kinematic-acceptance \
  --reference artifacts/my_dvt2_kin_calibration --output artifacts/my_dvt2_kin_acceptance
```

K3默认每个工具/球组合保留2个训练姿态、1个独立验证姿态，共16个训练点、8个验证点。训练池每组合4点；信息选点按**工具和球的组合**分组，不能因为左右共用A/B/C/D而合并两臂配额。验证姿态由独立随机种子生成，设计分离距离为0.012rad；辨识仍保留原先0.01rad的最近验证姿态门槛、200的缩放条件数门槛与20µm独立位置约束门槛。

新源模型仍使用60维MDH/Hayati参数图。先检查14维有效零位层，再由训练数据选择满足条件数的几何列；未选方向保留先验。四球CAD位置是本次合成计量锚点，不能将这些条件下的结果当成实机绝对/姿态精度。

## 接触前后的路线有不同规则

路线来自`configs/trical_1105_release_turn_reference.json`，随公共profile冻结；执行和复核不依赖旧artifacts、下载目录或FreeCAD。

1. 沿已冻结的自由空间转移到接近位，保持普通10mm余量。
2. 逆向进入局部路线；仅当前球对应的三个测头可使用有界5mm行程，列明的近接对保持2mm，其余仍是10mm。
3. 接触区保持相对该测点参考朝向的1°包络，包含每轴±0.00105rad不确定性。选定测量姿态及每次读表修正都要预先证明能退回原接触种子。
4. 左D、右C等转向路线在三个测头均有至少0.5mm正间隙后才进入转向段，随后回到全部10mm自由空间规则。无需转向的直退路线保持接触朝向直到自由空间接近位。
5. 每次访问都回到共同home。左右换装前检查已送达关节位置、速度、状态和新鲜度；Mock记录一个人工换装边界，不模拟拆装时间或装配误差。

新配置把参考管的±0.00105rad分为编码器0.00005rad与跟踪0.001rad：离线证明覆盖两者之和，执行与观测重分析可复用身份和端点匹配的封存参考证明；在线检查已送达位置相对参考的跟踪误差、朝向、新鲜度、速度与状态，不在控制循环内重跑CAD证明树；完整原始观测折线仍独立复核并只再加入编码器误差，避免重复叠加跟踪误差。冻结旧批次不补写该分配，继续按其原始较保守预算复核。

CAD几何的刚体误差族另加0.1mm位移预算，并由Mock工厂在左右装配上检查。当前合成真值只使用小幅安装平移和编码器偏置；没有声明额外刚体旋转误差预算。仪表噪声和量化配置随profile保存。

故障后保留原始记录，最多在当前有效相位内保持；不会借用旧K2的直线退出去穿过新台架，也不自动换装或返回home。后续实机薄适配须明确故障处置与人工确认。

## 只增加路线策略，不复制标定内核

- `dvt2_trical.py`提供实际装配配置、相位profile、路线策略与候选姿态设计。
- `MeasurementController`继续统一五字段下发、反馈检查、三表新鲜度、自动对中和稳定窗口；新旧场景复用同一个`center()`。C++仍只有既有`TricalMockSession`及其`ArmSimulationSession`推进物理。
- 动态与运动学使用同一个DVT2源、关节顺序和TCP。新球位配置若偏离冻结CAD碰撞场景会被拒绝，不能只改观测目标而继续用旧碰撞物体。
- K3拟合、SVD/选列、物理参数图和候选URDF导出复用既有模块。几何候选导出后仍要求重建碰撞资产及重验动力学，当前不自动部署。
- K4复用三批采集、冻结子集重拟合、真值隔离和中文回放。显示按已记录的工具侧切换左右两份CAD场景，每次只出现一个三表工具；MuJoCo不参与物理或碰撞。

### 连续门禁的计算方式

跨侧球附近有些运动未能用整段的保守运动界在预算内证明。新入口在接触弦段上分小段重算N2已有的局部运动上界，仍由Coal与同一关节盒证明器覆盖所有检查对；不减余量、不把预算耗尽当成通过。

观测复核保留每个原始关节点。每块用端点弦段加上**全部内部点**的偏差包络覆盖完整观测折线，再加编码器不确定性；几何弦段参数与物理时间分开，因此不会因C3加减速把一条直线路径误认为很宽的曲线。接触朝向也逐个原始样本检查。证明对应声明的观测折线与误差族，实机两帧间的运动仍需独立采样/控制资格。

## 当前验证记录

有效K3记录为`artifacts/dvt2_n3_k3_calibration_v3/report.json`：16训练/8验证、31/60选中参数，缩放条件数190.37；独立位置约束RMSE为名义40.25µm、零位层12.25µm、几何层0.879µm。最近训练/验证姿态距离0.02067rad；候选URDF重加载位姿差低于8e-16，质量矩阵差低于2e-16。辨识时移走evaluation，原始记录保持封存。

K4三批复测及新进程真值隔离通过，公共验证TCP预测标准差最大0.856µm。66秒中文只读回放位于`artifacts/dvt2_n3_k4_v1/presentation/kinematic_workflow.mp4`，展示左D/右C释放后转向和同一工具回位换装；已解码查看。汇总为`artifacts/dvt2_n3_kinematics_delivery_v1/delivery.json`。

完整本轮结果与限制汇总在[N3接入进度](dvt2_n3_integration.md)。默认CLI已整体切换为同一模型与实际装配；无需`--config`也会使用四球和冻结路线。N4及P0后续进展见[N4收尾](dvt2_n4_closeout.md)。
