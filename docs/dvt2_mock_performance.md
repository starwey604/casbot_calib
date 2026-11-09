# SDK Mock物理推进耗时与修复

500 Hz发送、50 µs积分和原跟踪门槛不变。本轮修复共享模拟设备中的重复动力学计算，以及SDK适配层的晚到调度；没有新增简化plant。

## 两个不同的问题

1. 物理线程某次推进超时后，旧调度器还会再等待完整1 ms。这会放大积压；现在超时后立即推进，追上后恢复正常节拍，不跳过任何虚拟事件。
2. 平均计算量低于实时预算，不等于每段运动都够快。H4 v3的279.59 s完整原生回放平均耗时130.48 s，但两个1 s虚拟时间段各自耗时超过1 s，最慢段约1.281 s。静摩擦约束解在部分轴运动、部分轴粘住时频繁计算`gravity + velocity_bias`，而后者内部又计算一次重力。

`ArmWorkspace.nonlinear_effects(q,dq)`现在直接调用Pinocchio `nonLinearEffects`，一次取得`C(q,dq)dq + g(q)`，不包含摩擦或加速度对应的armature项。静摩擦残差改用此项；原有gravity、velocity_bias、inverse和forward API及其含义不变。物理方程、摩擦/接触规则、积分器、采样与指令事件顺序不变。

## 如何重现原生剖析

```bash
bash scripts/humble/container.sh cmake --preset humble -DCALIB_BUILD_TOOLS=ON
bash scripts/humble/container.sh cmake --build build/humble --target profile_device
bash scripts/humble/container.sh python scripts/humble/profile_sdk_device.py \
  artifacts/dvt2_n4_h4_sdk_v3 --output artifacts/my_native_profile
```

剖析工具只用于评估：读取声明的Mock参数和SDK实际采用的五字段命令，直接回放同一个`SimulatedArmDevice`。1 ms推进区间不包含ROS、日志格式化或磁盘写入；结果给出每步耗时、每秒计算预算和完整样本。它不提供辨识输入，不把私有采样时间传给SDK客户端，也不授予硬实时资格。目标不安装进发布包；日常构建默认关闭。

## 已取得的对照

| 相同的279.59 s回放 | 修复前 | 单次非线性项计算后 |
| --- | --- | --- |
| 总耗时 | 130.48 s | 90.18 s |
| 耗时／虚拟时长 | 0.467 | 0.323 |
| 最慢1 s区间的计算耗时 | 1.281 s | 0.928 s |
| 超过1 s预算的整秒区间 | 2 | 0 |

修复前后279,586个样本按相同采样时刻比较：q最大差`4.15e-14 rad`，dq最大差`1.60e-11 rad/s`，电流最大差`6.47e-13 A`。原始结果在`artifacts/dvt2_n4_perf_full_v1`和`dvt2_n4_perf_nonlinear_v1`，对照为后者的`numerical_comparison.json`。5项核心CTest、4项SDK CTest及66项相关Python数值测试通过；1项可选渲染测试未启用。修改前核心源码和剖析二进制留在`dvt2_n4_perf_source_v1`。

每步耗时还受机器负载和操作系统调度影响，实时采用频率须在目标机上测量。

## 完整SDK结果

`dvt2_n4_h4_sdk_v4`的12条执行已走通，但弱惯量信号下真值误差14.413%，整体验证拒绝。随后按新的参考设计与独立种子执行`dvt2_n4_h4_sdk_v6`：12条记录及十项验证全部通过，所选左肩pitch等效惯量误差5.376%。前序复核、独立预测、无真值新进程重拟合与50/25µs回放通过。v6轨迹和峰值指标通过，粘滑瞬态逐点速度归一化差约6.65%仍单列为诊断。

这次代表验证没有展示稳定控制收益，完整负结果和新夹具命令见[N4收尾](dvt2_n4_closeout.md)。公开缓存语义、未知采样资格与故障整批终止规则未变。
