# 1105_03：肩腰包络与跨侧访问核对

2026-11-06。

肩腰包络不需要为了通过检查而缩小；真正需要细化的是三表工具的表体包络。右C原姿态的肩腰间距确实不足10 mm，左D则同时存在表体单凸包过度保守和保持姿态余量不足的问题。本文记录前序原姿态尚未放行的原因；后续已在同一工作状态模型下完成[8/8组分段进退参考](trical_1105_release_then_turn.md)。

## 两类问题分开处理

使用仓库DVT2源碰撞网格与实际1105_03工具STL，对原review测量姿态进行对照：

| 位置/零件对 | 原单凸包距离 | 源网格表面距离 | 判断 |
|---|---:|---:|---|
| 左D：肩滚转壳体—腰部 | 10.920 mm | 10.920 mm | 名义点超过10 mm，左D不能笼统归因为肩腰碰撞 |
| 右C：肩滚转壳体—腰部 | 4.849 mm | 4.849 mm | 是真实的门槛不足，不是这处凸包误判；仍为正间距 |
| 左D：表体5160—球柱 | 5.001 mm | 12.040 mm | 单凸包填平了细杆到圆表体之间的空隙 |
| 左D：表体5156—球柱 | 6.604 mm | 13.350 mm | 同类过度保守 |
| 右C：表体5140—球柱 | 8.033 mm | 17.540 mm | 表体有误判，但修正后仍须另解肩腰问题 |

肩腰另外按每臂837组肩俯仰/向外肩滚转角做网格审计。在表面分离区域，两种距离的最大差小于0.002 mm；本次网格没有出现“源表面达到10 mm而凸包不足10 mm”的点。因此保留现有肩腰凸包和检查对，不导入原review的排除项。

源三角网格的距离用于定位保守性，不能直接替代实心碰撞检查，也不生成连续轨迹证书。

## 已修正六个表体的空隙包络

新增独立候选：

- 工具资产：`python/casbot_calib/assets/models/trical_1105_03_gauge_cover_v2/`。
- 模型包：`casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2/`。
- 配置：`configs/dvt2_trical_1105_03_gauge_cover_v2.yaml`。

每个表体先使用与原导入器共用的工作行程切割，再沿实际测杆轴、按CAD面边界分成6个实体区间；逐区间做外凸包并加0.2 mm余量。六个表体共36段，BREP分割并集相对原工作状态实体的缺失/多余体积均为0。源STL顶点和三角形中心也检查落在分段外包并集中。

左D的5160表体—柱距离因此由约5.00 mm恢复到约11.84 mm，接近源表面12.04 mm。原STL、旧缓存和旧报告保留；新包另有模型/几何身份。新资产复用旧原始显示STL，不复制或修改源CAD。URDF裸机器人FK、质量矩阵和重力与原候选保持一致，工具未知质量没有补成已知负载。

普通10 mm、选定近接对2 mm、三对测头接触5 mm及每轴±0.00105 rad位置误差包络均保持不变。旧模型包没有新资产字段时，仍加载原工具资产；没有切换现有标定默认入口。

## 原姿态不能直接沿用

修正表体包络后，原review的八组参考仍只有左A/B/C、右B/D共5组通过整条连续门禁。右C仍因肩腰4.849 mm被拒绝。

对左D和右A各枚举活动臂7轴误差盒的128个角点，找到明确的余量反例：

- 左D：固定连接零件5147—球柱，距离降到 **9.635 mm**。
- 右A：表体5136的分段包络—另一颗D球，距离降到 **9.856 mm**。

这两例均在原先声明的±0.00105 rad范围内，数字为当前保守包络距离。另用源表面复查：左D仍为9.635 mm；右A源表面约10.186 mm，说明右A的反例涉及当前分段外包及0.2 mm外扩，不能将9.856 mm当成实物已接触或穿模。此前“保持包络未获证明”不能仅靠增加细分预算解决；需要换姿态或路径。另做20层→28层细分诊断仍未放行；默认细分深度保持20，新增具体零件对、采样距离和下界诊断，便于定位失败。

## 跨侧姿态搜索的含义

`trical_access.py`固定球心位置，释放末端朝向及活动臂7轴，用原生FK/Coal距离和SciPy SLSQP做多起点最大化最小余量。后续逐项检查：

1. 接触姿态及声明关节误差盒。
2. 保持朝向的直线退入路径；原左D40 mm、右C25 mm。
3. 退出后用普通10 mm规则检查到停放位的完整过渡。

另支持接触/退出两个端点联合优化；在恢复的位置附近使用±0.35 rad搜索盒，避免大幅跳解进入深度穿模。优化器返回成功、名义接触点变好或局部粗筛通过，都不等于完整放行。完整结论以搜索目录的`report.json`及连续证书为准。

原包已做48次单端点尝试，新包32次；新包能找到满足球心且名义最小余量约1 mm的接触姿态，但接近/退出仍可能损失肩腰余量。联合端点另完成8次尝试，仍无完整放行；对已得接触姿态试查20/25/30/40 mm撤回长度，也未找到同时满足局部名义余量与普通退出余量的候选。记录见下列证据目录。有限搜索失败不构成不存在可行姿态的证明。

**该审计确定的后续方向已完成：[脱离后转向参考](trical_1105_release_then_turn.md)在当前工作状态模型下8/8通过。** 原建议是改跨侧访问姿态和退入路径，而不是缩小肩腰包络或继续盲扫平台高度。 可检查是否在脱离球后转向、改变撤回方向或采用更短且已达到普通余量的退出位置；这些新路径仍须通过相同连续门禁，不能只放行接触点。

## 查看与重现

本机证据：

- `artifacts/trical_shoulder_audit_v2/`：八个接触姿态、每臂837点距离表及中文肩腰分布图。
- `artifacts/trical_fixture_surface_audit_v1/`：原工具与台架源STL最近表面距离及最近点。
- `artifacts/trical_collision_audit_view_v3/`：三幅近景合图及viewer核对；只更新几何，无MuJoCo机械计算。
- `artifacts/trical_gauge_cover_tests_v1.log`：新资产、源样本覆盖、原模型兼容和门禁相关验证。
- `artifacts/trical_gauge_v2_original_plan/`：原八组参考在新工具包上的完整门禁报告。
- `artifacts/trical_contact_corner_audit_v1.json`：两个误差盒反例，含具体关节角；`trical_corner_source_audit_v1/`补充源表面比较。
- `artifacts/trical_cross_access_v1/`、`trical_cross_access_gauge_v2/`：旧/新包单端点搜索。
- `artifacts/trical_cross_access_coupled_v2/`：局部盒内端点联合搜索。前一次宽范围搜索因深度穿模查询过慢，在首个失败结果后停止，保留`trical_cross_access_coupled_v1/interruption.json`，不计为完成批次。
- `artifacts/trical_short_retract_probe_v1.log`：20/25/30/40 mm撤回长度的额外诊断。

目录均为本地生成产物，不随Git提交。源码、配置及LFS资产在仓库。可从同一候选复查：

```bash
BUNDLE=python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2
bash scripts/humble/container.sh python scripts/humble/audit_trical_shoulders.py \
  --bundle "$BUNDLE" --output artifacts/my_shoulder_audit
bash scripts/humble/container.sh python -m casbot_calib.trical_access \
  --bundle "$BUNDLE" --attempts 4 --coupled --output artifacts/my_cross_access
```

CAD作者工具为`scripts/cad/cover_trical_gauges.py`；不指定`--native-checkpoint`时直接读归档STEP，指定时可复用本机FreeCAD缓存。分段后以`build_trical_1105.py --assets <新资产目录> --prepare-assets --config <新配置> --output <新模型包>`构建。实机或MuJoCo不承担这部分几何计算。
