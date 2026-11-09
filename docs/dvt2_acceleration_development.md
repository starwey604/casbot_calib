# DVT2腕部加速度开发实验

**004的R1与独立复核已通过，但R2稳定性拒绝，暂不升为默认方案。** 这里保留本轮设计和复现入口，后续按弱方向重新预注册实验。`configs/dvt2/staged_dual_arm_p0_v4.yaml`是新的开发配置；旧003协议、60条观测及R2拒绝记录保留，默认配置暂不替换。

## 为什么改激励，而不是改门槛

003的R2训练预测通过，但删构型后小腕轴对角惯量变化超过10%。`artifacts/dvt2_n4_wrist_information_v1/report.json`只读取原train与已完成R2解：右腕pitch的三个训练中心约为0.021、0.092、0.113 rad，覆盖窄且同号。删去不同中心时，最敏感方向分别落在右腕roll、pitch、yaw；刚体项与摩擦项的变化没有明显相互抵消。这提示应补充腕部加速度和子关节姿态覆盖，不能靠总体质量矩阵误差较小来忽略它。

## 004增加什么

- 保留原大幅低频多频正弦，不缩小运动、不改摩擦权重、ridge或验收阈值。
- 六个腕轴额外叠加0.010 rad正弦：train为2 Hz，selection为13/6 Hz，test为14/6 Hz，均在原40阶处理频带内。新项与原多频项使用独立相位种子。
- 在三个train中心，左右腕pitch/roll相对home分别固定为`(-0.3,-0.3)`、`(+0.3,-0.3)`、`(0,+0.3)` rad；其他轴仍从随机池中选择。selection/test中心不套用这些固定值。
- 采用已有C3幅值包络进入和退出激励，静态中心转移、回位和未激活侧保持原逻辑。参考端解析给出q/dq/ddq/jerk，不引入新的测量加速度通道。
- 全部角色使用新采集、参考及排序种子。原有56条正反向腕yaw探针不变；正式test只有参考规划，不在开发阶段采集。

0.015 rad初始参考草案在test参考的连续jerk界超过60 rad/s³，采集前已将**新增项**定为0.010 rad；原低频幅值和门槛未变。最终全路径仍须通过Coal连续碰撞门禁，不能用这项解析检查替代。

## 从新目录运行

```bash
bash scripts/humble/container.sh casbot-calib plan-staged \
  --config configs/dvt2/staged_dual_arm_p0_v4.yaml \
  --development-reference artifacts/dvt2_n3_p0_predecessors_v2 \
  --output artifacts/my_dvt2_004

bash scripts/humble/container.sh casbot-calib staged-calibrate \
  --config configs/dvt2/staged_dual_arm_p0_v4.yaml \
  --output artifacts/my_dvt2_004 --resume --through inertia

bash scripts/humble/container.sh casbot-calib staged-identify \
  artifacts/my_dvt2_004 --through stability --output artifacts/my_dvt2_004_r2

bash scripts/humble/container.sh casbot-calib staged-verify \
  artifacts/my_dvt2_004 --stability-analysis artifacts/my_dvt2_004_r2 \
  --output artifacts/my_dvt2_004_r2_checked
```

前序引用仍是声明的**开发参考**，不是新的独立重力/摩擦留出数据。只有R1原始观测与前序门禁通过后，才进入R2；稳定性失败则保留失败，不读取正式test调参。R3选参冻结、R4新独立实验和K5按后续轮次推进。

## 本轮已取得的R1结果

正式开发目录为`artifacts/dvt2_n4_p0_v4_development_v3`：636条参考连续门禁通过，36 train/24 selection新记录全部采完。R1五项检查通过；主要轨迹最大逐轴NRMSE为2.946%，原始静止区间超界最大0.000739Nm，独立腕部探针惯量差最大4.879%、预测RMSE最大0.000133Nm。它们分别低于原5%、0.01Nm、10%及0.002Nm门槛，权重和门槛均未改。R2结果另见[N4收尾](dvt2_n4_closeout.md)，不能仅凭R1通过就进入正式test。

原型`v1/v2`只用于参考规划，未采集标定记录；v2在重加载时暴露整数索引字典经JSON变成字符串键的问题，采集前已停止。固定姿态配置改用规范字符串索引，并新增完整预注册→JSON落盘→重加载测试。v3从最终绑定重新规划、注册和采集，不修改原型封存。v3/R2执行时的H4验证脚本及`staged_acceleration.py`在`artifacts/dvt2_n4_p0_v4_source_v1`。收尾仅补了旧R0不要求R1中心设计字段的兼容性回归（最终55项通过），004已有配置的参考和数值不变；后续独立H4动态夹具不参与P0采集/辨识。

## R2的最终结论

30个变体、33次拟合及独立复核均完成，三档ridge没有合格项。删构型后最大对角惯量差分别24.017%、23.971%、23.802%，均超10%；最差方向为左腕yaw。删组留出的腕部探针惯量差最大约23.54%，而完整train、删批次和先验扰动通过。分析与复核进程禁止读取evaluation，正式test未采集。

因此004保留为未通过稳定性的开发方案，不进入R3/R4。下一步基于现有train的Pinocchio回归量定位该姿态相关算子的弱方向，再设计轴分离/分频或有效姿态补充实验；不再仅凭更大加速度或更宽中心宣布信息充分。完整证据与责任见[N4收尾](dvt2_n4_closeout.md)。
