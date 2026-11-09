# DVT2默认切换与N4收尾

默认14轴模型现已统一为`casbot_hl12p1_dvt2_bare_flange_1105_03_gauge_v2`。原来的命令、配置入口和SDK五字段接口继续使用；不再需要为新模型额外指定一套入口。

**本轮结果（2026-11-07）：SDK H4代表性验证通过；新004实验的R1及独立复核通过，R2仍拒绝。** 当前剩余重点是左腕yaw的跨构型惯量稳定性，未进入R3/R4或K5。汇总为`artifacts/dvt2_n4_round2_delivery_v1/delivery.json`，整体`passed=false`；旧交付不回写。

## 默认入口现在选什么

- 动力学：腰部固定、完整裸法兰整机与地面；不把未知的三表工具负载算成已知惯性。
- 运动学：实际1105_03四球台架，同一个三表工具左右依次换装，沿冻结的八组释放/转向路线。
- 设备与显示：SDK Mock、H1–H4、Pinocchio、Coal、导出和MuJoCo viewer共用同一源身份。viewer不计算机械运动或碰撞。
- `configs/staged_dual_arm*.yaml`、`configs/sdk_mock.yaml`、`configs/hardware_*.yaml`及`configs/kinematics_*.yaml`保留原入口名，内容更新为同源DVT2条件；`configs/dvt2/`显式入口仍可使用。
- wheel携带默认运动学所需的冻结路线，无需下载目录、仓库根目录或FreeCAD。`nominal_dual_arm()`、`nominal_kinematics()`、`nominal_profile()`等原函数名保留。

旧14轴URDF、旧碰撞缓存及旧合成三表配置已移出运行包。仅在`tests/fixtures/legacy_nominal/`保留拒绝跨代输入、核对原协议门槛所需的历史夹具；没有旧模型运行开关或自动fallback。原单关节/evt6独立解析实验不改绑为DVT2。

**历史会话不能直接续采或改身份。** 源模型、场景或参数身份不匹配时拒绝加载；复现旧实验使用相应Git版本和原始记录。旧nominal的114项历史数值及P0正式test不作为新模型证据。

## 修正后的R1如何接续

[物理静止门禁](dvt2_p0_integration.md#修复用原始静止窗口检验物理区间)已在同60条开发数据、同冻结参数上重验通过，旧拒绝报告不回写。R2通过`--development-verification`显式消费新的独立R1复核：

```bash
bash scripts/humble/container.sh casbot-calib staged-identify \
  artifacts/dvt2_n3_p0_development_v1 --through stability \
  --development-verification artifacts/dvt2_static_gate_verify_v1 \
  --output artifacts/my_dvt2_r2
```

新R2封存该复核及所对应的分析/输入身份。原已通过R1的调用不需要增加参数；原拒绝会话没有显式证据则继续拒绝。R2的三档ridge、30变体、固定比较尺度及10%腕部惯量稳定性门槛不变。

## 已取得的默认路径证据

| 证据 | 覆盖范围 |
| --- | --- |
| `artifacts/dvt2_n4_install_v1.log` | 5项CTest、独立Core/Models安装；首次wheel验证暴露旧的测量轴假设，随后修正验证器 |
| `artifacts/dvt2_n4_install_final_v2/wheel/verification.json` | 非editable wheel、五套源、默认14轴、四球/默认路线与原生Mock；禁止Python Pinocchio/Coal及MuJoCo导入 |
| `artifacts/dvt2_n4_consumers_v2.log`、`dvt2_n4_ament_install_v3.log` | 独立C++与ament消费者加载新默认URDF及左右CAD场景；ament原安装缺少新派生资产，重装Core后通过 |
| `artifacts/dvt2_n4_sdk_v1/report.json` | 真实ROS SDK消息：左右肩100/250/500Hz、左右腕roll 500Hz共8条；同侧跨频率参考一致，反馈采样时刻仍未知 |
| `artifacts/dvt2_n4_default_render_v1/report.json` | 默认工厂的原生Mock、两类URDF重加载、155个frame与viewer一致；已查看实际PNG，禁止MuJoCo机械接口 |
| `artifacts/dvt2_static_gate_verify_v1/verification.json` | 同一新模型R1的60条路径、8组步长减半及无真值新进程重评分；模型/输入未变，保留原证据 |

新默认K1检查为`artifacts/dvt2_n4_k1_v2`。新模型测量轴来自CAD，解析仪表检查按实际法向投影，不能继续假定三个通道就是世界坐标轴；数值门槛不变。

## 前一轮拒绝与本轮处理

### R2：腕部惯量对训练构型仍敏感

`artifacts/dvt2_n4_stability_v1`已完成30个变体、33次拟合；`dvt2_n4_stability_verified_v1`独立复核得到相同拒绝，原生算子差0。三档ridge都没有稳定性资格：删构型后的最大对角惯量差分别为11.014%、10.986%、10.780%，高于原10%门槛，最差方向为右腕yaw。各变体训练预测仍通过，质量矩阵整体差异与逆动力学差异也通过，失败集中在小腕轴的对角惯量稳定性。图`operator_stability.png`已查看。

当前不进入R3/R4、不冻结候选、不采正式test。本轮已完成train信息诊断，并建立[004加速度/中心覆盖开发实验](dvt2_acceleration_development.md)。新配置单独采集，原60条记录和拒绝结果保留；不增加ridge或放宽10%门槛来宣布惯量已标定。

R2完成之后补了新配置/旧快照的身份绑定：新配置在采集前绑定DVT2，旧封存快照缺身份则拒绝，不自动贴新标签。原版的三个绑定模块，以及本轮改动前的`staged_protocol.py`、`staged_multiconfiguration.py`，保存在`artifacts/dvt2_n4_stability_source_v1`；原数值辨识和比较算子未改。

#### 004：R1通过，扩大覆盖仍未解决R2

`dvt2_n4_p0_v4_development_v3`完成636条参考规划及36 train/24 selection新采集。`dvt2_n4_p0_v4_verified_v1`独立重验60条参考/观测路径、8组步长减半及无真值新进程分析通过，算子差0。R1主要轨迹NRMSE最大2.946%、物理静止超界0.000739Nm、腕部探针惯量差4.879%，均满足原门槛。

`dvt2_n4_p0_v4_stability_v1`完成30变体、33次拟合，约767.12s；`dvt2_n4_p0_v4_stability_verified_v1`独立复核得到相同拒绝，原生算子差0。分析与复核均在禁止打开evaluation的进程中执行，正式test未消费。三档ridge的三个删构型变体均拒绝，完整train、删批次及先验扰动通过。

| ridge | 最大对角惯量差（门槛10%） | 最大删组留出腕部探针惯量差（门槛10%） |
| --- | --- | --- |
| 1e-5 | 24.017% | 23.537% |
| 1e-4 | 23.971% | 23.535% |
| 1e-3 | 23.802% | 23.511% |

最敏感方向转为**左腕yaw（索引4）**。`dvt2_n4_p0_v4_wrist_information_v1`只读train及已封存R2解，刚体/摩擦变化的加权余弦约−0.015至0.021，没有明显抵消。004考察的姿态域也更宽；这套加速度/中心设计未解决缺构型外推问题，不能升级为默认P0方案。

**下一步先做针对性可辨识性设计**：用现有train、固定比较尺度和Pinocchio回归量，定位左腕yaw姿态相关惯量的弱方向；据此预注册轴分离/分频或补充有效姿态的最小增量实验。仍由calib实现负责，待新实验通过原R1/R2后再进入R3/R4；不放宽10%、不继续靠增加ridge或读取正式test选参。

### SDK H4：推进瓶颈修复，代表性验证通过

- `artifacts/dvt2_n4_h3_sdk_v2/verification.json`：H3代表性7条SDK记录、匀速摩擦/双向起动、无内部真值新进程重分析和50/25µs回放通过。只选择左肩pitch，使用声明的名义重力先验夹具，不声称完成实测H2或全14轴采集。0.06Nm合成静摩擦下，原保持增益导致回位误差超5mrad；验证夹具将保持Kp下界设为30，目标起动仍零PD，实机H3默认/门槛不改。
- H4首轮`dvt2_n4_h4_sdk_v1`因旧Mock在内部物理落后时停发缓存，出现111ms公开反馈空窗并终止。现已按用户确认的SDK语义修正：正常控制五字段转发不等待主机反馈；公开话题持续发最新缓存，stamp不变成采样时间。合成保护保持仍需要新鲜内部位姿。见[SDK收敛说明](sdk_mock_unification.md#2026-11-06缓存发布与五字段转发不依赖主机反馈刷新)。
- 修正后的`dvt2_n4_sdk_cache_contract_v1/verification.json`通过17项SDK实际往返/模式/断流检查；显式关闭反馈后仍停止发送并触发看门狗。四项SDK CTest通过，客户端100ms失联门槛不变。
- `dvt2_n4_h4_sdk_v2`第一条惯量记录接受，第二条因未对齐跟踪误差**20.093mrad > 20mrad**终止。`timing_tracking_diagnostic.json`显示该段内部缓存采样年龄最大338.85ms。持续1000Hz发布没有把落后的物理状态变成新采样；守卫仍拒绝，未续采故障会话。

本轮已修复晚到调度，并以一次Pinocchio非线性项计算替代静摩擦约束中的重复重力计算；[完整剖析与数值对照](dvt2_mock_performance.md)记录耗时和样本差异。仍为同一个`SimulatedArmDevice`、500Hz发送、50µs积分及原跟踪/断流门槛。

`dvt2_n4_h4_sdk_v4`已完成3条惯量、5档调参与4组控制，无真值新进程重新处理/拟合、候选一致性和50/25µs回放通过。独立真值核对所选轴惯量为14.413%误差，超过原10%门槛，整体验证仍为false。其独立预测误差虽低，但train名义惯性力矩RMS仅0.0053–0.0070Nm，约占2.3Nm总力矩的0.23–0.30%；总力矩NRMSE低不能代替惯量被准确恢复。固定PD前馈RMSE改善约0.35%，不作稳定控制收益结论。

因此新增`verify_hardware_inertia.py --dynamic-fixture`：使用新train、selection及validation种子，5s周期、0.10rad幅值，仅改变合成验证夹具。全部预先声明参考的最大加速度约1.286rad/s²、jerk约7.168rad/s³，保持原1.5/8门槛，实机H4默认不改。4s/0.12rad草案在`dvt2_n4_h4_sdk_v5`被原连续参考门禁拒绝，只有只读起姿快照，未发送运动，不续采该故障会话；正式新采集为`dvt2_n4_h4_sdk_v6`。

**v6代表性SDK验证通过**：`artifacts/dvt2_n4_h4_sdk_v6/verification.json`十项检查全部通过。12条记录完整采集，前序复核、独立预测、无真值新进程重处理/重拟合及50/25µs回放均通过。所选左肩pitch的等效惯量为`0.064353kg·m²`，独立plant参考为`0.061070kg·m²`，相对误差**5.376% < 10%**。只核对该轴，不声称恢复了全14轴或每个连杆的唯一惯性。

PD选择为1.5档。独立控制中固定PD的辨识前馈RMSE仅改善约0.374%；先验/辨识模型下调参后的RMSE分别反而升高23.51%/39.53%。完整四组结果保留，不作稳定调参收益结论。采样时间/新帧率/缓存年龄仍未获得实机资格，`raw_sampling_qualified=false`。

可复现本轮新夹具：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh python \
  scripts/humble/verify_hardware_inertia.py --dynamic-fixture \
  --prior artifacts/dvt2_n4_h3_sdk_v2/batch --output artifacts/my_dvt2_h4_dynamic
```

## 交付状态与K5

**N3默认模型切换完成；N4整体验收未通过。** 安装、默认工厂、K1–K4、显示与SDK接口链已有有效证据；SDK H4代表性验证和004 R1独立复核已通过；004 R2仍拒绝，尚不进入R3/R4。K5真实仪表/ROS测量薄适配尚未开始。

最新汇总见`artifacts/dvt2_n4_round2_delivery_v1/delivery.json`，前一轮保留在`artifacts/dvt2_n4_delivery_v1/delivery.json`。本机软件/合成证据不授予实机标定资格；实物工具负载、安装计量、自由测杆外形和SDK采样资格仍按原清单核验。正式P0 test未消费。

本轮最终代码的5项核心CTest、4项SDK CTest、相关Python分组、native/ament消费者与独立wheel通过；完整计数/交叠范围见最新汇总。最终wheel为`artifacts/dvt2_n4_nonlinear_install_final_v4`，包含一次非线性项API及004规范索引，已在禁用editable导入的新进程验证。未执行Windows或旧114项历史交付。
