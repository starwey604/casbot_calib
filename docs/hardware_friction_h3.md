# 🦾 H3：运动摩擦与起动摩擦现场流程

H3在完整双臂、腰部固定条件下，冻结前序重力候选，然后依次完成：**多姿态正反匀速 → 独立姿态验收 → 冻结摩擦候选 → 零PD静置与力矩斜坡 → 独立重复验收**。H1/H2/H3共用同一个SDK客户端、原始记录、批次封存、受控续采和只读回放，不另建Mock版标定算法。

本轮用户确认：CASBOT主机将position、velocity、kp、kd、effort五项直接转发给模组，由模组本地计算PD；command effort按Nm前馈叠加，kp=kd=0会生效，主机不自动加入重力或摩擦补偿。该转发与PD执行位置固定为接口约定，模组PD环具体频率仍未知；主机position的1000Hz插值约定保持。H3因此由客户端下发冻结重力模型的前馈；标定期间不加摩擦前馈。上述确认仍需绑定机型/固件实测，不代表已取得合格采样时间或独立力矩传感器。

## 🧭 一、先准备输入

需要[H2](hardware_batch_h2.md)完成的重力候选及其联调验证；也可显式提供H1已封存、验证通过的重力分析目录。H3复制并绑定参数、URDF、重加载报告和原摘要，核对模型、关节顺序、安装重力、effort增益和零偏；改变这些条件必须新建批次。

默认配置为[hardware_friction.yaml](../configs/hardware_friction.yaml)。默认覆盖14轴，其他轴保持；每轴有两个训练姿态、一个独立验证姿态、三档训练速度和两档验证速度，共112条匀速记录。起动实验包含两姿态、正反方向、两种静置时间、两种斜率及两次独立重复，共448条。完整默认批次共560个单元；现场可先把joints改为一个具名轴做联调，新建批次逐步扩大。

home_rad是规划中心，不是机体已处于该姿态的声明。每次尝试都重新读实际起姿，再检查接近、动作、制动及回位。起动实验还检查目标轴允许偏移和其余轴保持范围，不能靠删除碰撞余量放行。

## ▶️ 二、运行

先准备Humble和SDK overlay，见[启动指南](dynamics_getting_started.md)。所有新输出目录必须不存在。

~~~bash
cd ~/codings/casbot_calib
read -r -p "现场 ROS_DOMAIN_ID: " calib_domain_id

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-plan-friction \
  --config configs/hardware_friction.yaml \
  --profile configs/hardware_field.yaml \
  --gravity artifacts/site_batch_v1 \
  --domain-id "$calib_domain_id" --output artifacts/site_h3_v1
~~~

--gravity可以是已完成H2的批次根目录，也可直接指向其最新报告中的gravity目录。规划不连接机器人、不下发指令。现场模型、连接和限制应使用核对后的profile，不能拿另一台设备的候选冒充本机实测。

执行继续使用H2的同一个命令；该命令会真实发送SDK指令：

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h3_v1 --max-jobs 1

bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  casbot-calib hardware-run-batch artifacts/site_h3_v1 --resume
~~~

Mock演练先按[统一SDK Mock指南](sdk_mock_unification.md)启动设备，用生成的client_profile.yaml替换--profile。为了验证非零摩擦，可在SDK Mock的plant配置中显式设置14轴friction_viscous、friction_coulomb、friction_static，并启用stiction_enabled；这些只是合成真值，采集和辨识程序不能读取它们。

## ⚙️ 三、运动摩擦怎样处理

轨迹复用已有C3速度过渡、真正匀速段、制动和反向流程。保持增益由名义模型及冻结配置生成，带明确下限，避免低惯量腕部的保持过软；它们不是优化出来的摩擦参数。实际发送的PD与effort记录在commands.csv。

从SDK实际接收的q、dq、估算Nm提取较长匀速窗口，保留所有原始缓存发布。处理检查窗口覆盖、回调缺口、位置趋势与报告速度一致性、速度趋势和名义惯量污染；不对发布stamp逐帧差分，不跨缺口求导。短窗口的趋势只是准静态接收观测诊断，不冒充真实采样加速度。

扣除冻结重力及名义速度耦合项后，按相同位置配对正反方向，利用半差降低静态重力误差。空间配对和非负鲁棒求解复用原有staged_identification函数，拟合b与fc：

\[
\tau_f=b\dot q+f_c\tanh(\dot q/0.001)
\]

训练和验证按整个姿态/轨迹分离，SVD检查多速度是否足以区分两项；只有独立验证通过，才封存friction_checkpoint并允许后续起动实验。不能用重新拟合验证记录来掩盖失败。

## 🧪 四、起动实验怎样执行

1. 位置控制到指定姿态，稳定等待，再按配置静置。
2. 目标轴kp=kd=0，继续外部补重力，检查零PD状态确实静止；其他轴保持。
3. 正向或反向逐渐增加effort，不超过冻结的斜率、时间、增量及位移/速度范围。
4. 实际接收反馈满足位移、速度和持续时间条件后，立即撤掉斜坡，恢复PD保持。回位参考从实际确认偏移连续衰减到0，随后返回本次实测起姿。
5. 从完整记录回溯最后静止、首次可分辨运动及较晚确认时刻，分别保存操作性区间和确认力矩；不把较晚确认力矩当成静摩擦常数。

区间减去冻结重力，保留噪声、原重力验证残差及正反接近差异。训练重复和独立验证重复分开；不同方向、姿态、静置时间和斜率的原始结果保留，不强行拟合材料级静摩擦规律。

SDK缓存年龄、字段同步和滤波仍未知，所以报告明确physical_onset_interval_known=false。区间是当前观测分辨率下的操作性证据，不是精确物理起动时间或实机静摩擦的高精度承诺。

## ⏯️ 五、中断、故障与报告

正常检查点或受控中断可按H2规则续采，跳过已接受记录并新读起姿。SDK/通信/模式故障、运动限值、未确认起动或零PD不静止会结束整批；clear_fault只log，不清故障、不恢复、不允许故障会话resume。匀速窗口不合格也不自动扩大范围或增加重采预算。

~~~bash
bash scripts/humble/container.sh casbot-calib hardware-analyse-batch \
  artifacts/site_h3_v1 --output artifacts/site_h3_analysis_v1

bash scripts/humble/container.sh casbot-calib hardware-render-batch \
  artifacts/site_h3_v1 --output artifacts/site_h3_video_v1 --fps 12
~~~

批次仍采用protocol、events、attempts、reports及不可变摘要。报告包含moving_friction.json、breakaway.json、friction_parameters.csv、onset_thresholds.csv及runtime_friction.json、独立验证、全部来源及频率/控制配置；运行时候选列出未辨识轴，auto_deploy=false，不自动部署。回放仅显示SDK接收位置，MuJoCo不参与机械计算。

H3输出是现场候选，formal_hardware_calibration_accepted始终false。该SDK窗口协议不降低旧S3/S4的4kHz/独立更密采样要求，也不把未知缓存变成已知采样。后续H4接入惯量激励与前序复核，H5再闭合真实尺度、模型/限制和独立设备验收。

## 🔬 六、本机演练结果与边界

本轮完整回归441项Python、4项核心C++及2项SDK检查通过；H1/H2统一SDK回归23项通过，旧S2的500Hz往返通过。证据分别在hardware_friction_tests_final_v1/full.xml、hardware_friction_h12_regression_v1及hardware_friction_s2_regression_v1，汇总见hardware_friction_delivery_v1/delivery.json。历史114项冻结数值交付本轮未重跑。

本轮实际运行统一ROS SDK Mock，客户端500Hz、主机参考/公开反馈1000Hz，缓存刷新为合成3ms。14轴机械模型均参与，但只对左肩pitch执行3条不同速度/独立姿态匀速记录及4条双向起动斜坡；支持检查点续采、故障终止和新进程禁止读取evaluation后的重分析。完整默认560单元未执行，也未连接物理机器人。

| 指标 | 合成设定 | SDK观测估计 |
| --- | --- | --- |
| 黏性摩擦b，Nm·s/rad | 0.08 | 0.0805900 |
| 库仑摩擦fc，Nm | 0.03 | 0.0301431 |
| 负向起动阈值，Nm | 0.06 | 0.0628951 |
| 正向起动阈值，Nm | 0.06 | 0.0640977 |
| 运动摩擦独立验证RMSE，Nm | — | 0.0009714 |

证据为本地artifacts/hardware_friction_ros_v9/verification.json、batch/reports/report_0002及中文replay；新增CSV/运行时导出在hardware_friction_readonly_v1。以上是本机合成结果，不保证其它关节、全部默认条件或实机精度。原始起动区间与较晚确认力矩分别记录。

该ROS演练显式使用100µs积分、1ms内部采样，避免墙上时间计算积压；生产默认积分仍为50µs。独立回放已记录指令、关闭两组观测噪声，比较100/50µs：位置轨迹最大差约0.000055rad，占行程0.0231%；q、dq、电流峰值相对变化均低于1%。粘住/脱离瞬态的逐点速度最大差为峰值的2.03%，另保留为诊断，不声称所有逐点状态差都低于1%。报告位于step_convergence/report.json，首次更严格逐点判据的失败也保留在step_convergence_pointwise_diagnostic。该数值检查不赋予SDK采样时间或旧S3/S4资格。

早期v1–v8演练中的首帧空窗、弱保持增益、ROS反馈闭PD和计算积压失败均保留。改为已确认的首帧保持、有下限的声明增益以及本地PD后重建新批次；没有放宽原反馈年龄、跟踪或碰撞门槛来掩盖失败。厂家模式/公开驱动对本地PD的支持证据见[SDK说明](sdk_hardware_prerequisites.md#因克斯模组本地pd能力证据)，CASBOT主机五项转发/模组本地PD已由用户明确确认；模组PD环具体频率仍待核验。

再次执行同类小范围演练，可使用以下命令；--gravity需要已有通过验证的H1/H2重力候选：

~~~bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  python scripts/humble/verify_hardware_friction.py \
  --gravity artifacts/site_mock_h2 --output artifacts/site_mock_h3_new --render
~~~

脚本只在localhost私有域测试，不能替代现场hardware-*操作；后续运行会同时执行记录指令的步长减半检查。
