# 1105_03：试选孔位与平台高度

本页保留前一轮孔位/高度试算：当时原位及+50 mm代表方案各5/8组通过。后续核对表体包络、改进保持位并采用[脱离后转向路径](trical_1105_release_then_turn.md)，已在原布局的工作状态CAD模型下8/8通过。下表仍对应原试算输入，不回写为新方案结果。

## 先看结果

本机报告在 `artifacts/trical_hole_height_report_v2/`：

- `hole_height_screening_zh.png`：孔位、侧球高度和平台升降的结果总览。
- `screening.csv`：完整数列；`summary.json`：试验来源及连续门禁结果。
- 平台对比图在 `artifacts/trical_platform_comparison_v3/platform_height_comparison_zh.png`。

这些是本地产物，不随Git提交。重建命令见下文。

本轮共141条试算记录，含基线对照和重复物理布局，不是141种独立装配：

| 试算 | 范围 | 记录数 | 最多找到的名义组合 |
|---|---|---:|---:|
| 整柱换台面孔 | 靠近机器人侧第2–7排、第9–11列，另含原位 | 19 | 6/8 |
| C、D球换柱侧孔 | 各自相对原位−40～+40 mm，步长10 mm | 81 | 6/8 |
| 平台整体升降 | −150～+150 mm，步长25 mm | 13 | 6/8 |
| 扩展测头朝向 | 基线及部分台面/侧孔布局，绕接近轴旋转并保留最多3个候选 | 15 | 6/8 |
| 升降与朝向/侧孔组合 | 部分高度及升降50/100 mm配合C、D球±30 mm | 13 | 6/8 |

“名义组合”指IK、局部路径与采样过渡路径粗筛成功，**不是连续路径通过**。八组为左右两臂各访问A/B/C/D四球；每臂都需要使用同一套锁紧后的球架布局。

原位和平台升高50 mm分别完成整批连续门禁：左A/B/C、右B/D通过；左D和右C在所测候选中未找到名义解；右A测量保持的关节不确定性包络未获证明（`joint_box_unresolved`），不是已证明碰撞。放行参考为500 Hz、C3静止起停曲线，保持每轴±0.00105 rad位置包络。失败项没有生成执行参考。

**建议先核对肩部/腰部碰撞包络及跨侧球的访问姿态，再决定换孔或锁定高度。** 所测升降没有消除左D、右C瓶颈，继续只扫高度收益有限；有限候选搜索未成功不等于实物必然无法运行。

## 平台升降怎样处理

从STEP圆柱面提取出台面209个孔，25 mm间距；柱侧20排孔，10 mm间距。排从机器人向前增加，列从机器人右侧向左增加。

- 台面换孔：球柱、四个球及球柄一起平移；台面、机器人和工具不变。当前假设柱底中心参考对齐台面孔中心，螺栓/转接件兼容性未审核。
- 柱侧换孔：仅相应球和球柄移到现有高度孔，球柱不动。
- 平台升降：台面、随动支架、球柱、全部球与球柄一起升降；固定底座、机器人和地面不动。−150～+150 mm是试算范围，**实际行程未知**。
- 升降只建模锁紧后的静态位置，不模拟调节机构或升降过程。同一双臂批次中不改变平台高度、球位或球架世界变换；改变后需新建布局、重做接近路径与计量登记。

布局写入种子文件、碰撞几何身份和参考协议。viewer读取同一布局，不通过平移机器人或调整重力来凑显示。

## 实际装配与模型

原始STEP归档为 `references/cad/trical_1105_03/1105_03.stp`，使用Git LFS。该目录保存原review的坐标登记、选择集和路径；它们是几何来源，不是当前严格门禁通过证据。运行时只读已导出的STL与碰撞缓存，无需FreeCAD或下载目录。

运行资产为 `python/casbot_calib/assets/models/trical_1105_03/`；候选模型为 `casbot_hl12p1_dvt2/derived/bare_flange_1105_03_v1/`，派生配置是 `configs/dvt2_trical_1105_03.yaml`。

- 从DVT2 `a2_ff`移除普通手子树，保留原裸法兰与源惯性；仅双臂14轴活动。
- 一个实物表架依次装左、右臂，另一臂裸法兰停放。左安装角相对STEP +90°，右0°；本轮旋转的是接近朝向候选，不是暗改该装配登记。
- TCP、表轴方向、球心、工具与台架几何来自已登记的CAD；三个表的零读数仍须调用方提供。
- 原完整裸装配与约简模型对照已覆盖149个源frame，以及质量矩阵、重力和逆动力学；裸源模型总质量49.4481290188 kg。真实三表工具总质量/质心未知，未偷偷加成已知动力学负载。
- 测杆只采用review的工作接触状态；自由伸出全行程尚未量测，不能据此批准所有实机接近动作。

机器人与工装都走现有原生Pinocchio/Coal。碰撞模型保留保守外包；两个凹形联轴器先经FreeCAD分成各8个BREP单元，再逐单元外包并加0.2 mm余量，分割并集的缺失/多余实体体积均为0。没有用不具备实体包含判据的三角网格距离代替实心碰撞。

普通碰撞余量仍为10 mm，只有列明的选定球近接零件对为2 mm，三个测头/选定球为5 mm有界行程。没有导入review中肩部与腰部的排除规则。原review离散检查的“可加工”结论，不能代替当前规则下的整批认证。

连续检查使用距离变化上界：先证明整段关节包络内安全的零件对，再仅对剩余对细分；未证明的对不丢弃，不改变门限。MuJoCo仅几何渲染；显示时将超过20万面的STL拆成多个文件，保留全部三角形，Coal资产不变。原试算后的显示文件修复只改变viewer封存身份，未改变模型、碰撞或夹具；最终两组连续门禁在修复后的包上重新运行。

## 自己再试一组

在仓库根目录执行，输出目录须不存在。环境沿用已安装的Humble，不启动SDK或机器人控制：

```bash
BUNDLE=python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_v1

# 原位加18组台面换孔；--kind side 为81组侧孔，height为13组平台高度。
bash scripts/humble/container.sh python -m casbot_calib.trical_holes \
  --bundle "$BUNDLE" --kind column --output artifacts/my_hole_screen

# 使用本轮保存的组合布局，增加接近轴旋转候选。
bash scripts/humble/container.sh python -m casbot_calib.trical_holes \
  --bundle "$BUNDLE" --layouts configs/trical_1105_platform_pose_trials.json \
  --spin-deg 0 15 -15 30 -30 45 -45 60 -60 90 -90 120 -120 180 \
  --alternatives-per-case 3 --output artifacts/my_height_pose_screen

# 粗筛成功还要过连续门禁；有未获证明项时返回非零并保留完整报告。
bash scripts/humble/container.sh python -m casbot_calib.trical_paths \
  --bundle "$BUNDLE" \
  --seeds artifacts/my_height_pose_screen/platform_Z+050/planning_seeds.json \
  --output artifacts/my_height_plan
```

`planning_seeds.json`包含八个完整角色及缺解占位，`seeds.json`仅包含找到的候选。门禁输出 `report.json`、`cases.json`，放行案例另输出 `motion.json`和`reference.npz`。它们只是位置参考，不含PD/effort执行指令、不采集表值，也未进入K3参数拟合。

本轮试算根目录依次为 `artifacts/trical_hole_sweep_v1`、`trical_side_hole_sweep_v1`、`trical_platform_height_sweep_v1`、`trical_hole_pose_sweep_v1`、`trical_platform_pose_sweep_v1`。用 `scripts/humble/report_trical_holes.py --help`可从这些目录重建总览；用 `scripts/humble/view_trical_heights.py --help`可比较两批已放行的左A测量姿态。

CAD再导出工具在 `scripts/cad/import_trical_1105.py`和`cover_trical_1105.py`，使用主机FreeCAD；`scripts/humble/build_trical_1105.py`在Humble内生成缓存和候选。完整CAD再生成需要Git LFS中的STEP和登记文件；常规wheel安装不包含434 MB原始STEP，也不执行CAD作者工具。

## 还没有完成的事

新默认切换仍属于N3，目前保留旧流程运行入口，不把两套身份混用。实际STEP装配已明确，无需再问是否拆普通手；工作状态模型的N2代表进退路径已闭合；N3尚待所有消费者统一切换，而非等待装配选择。后续还要核对真实工具载荷、表轴/零点、自由测杆外形、球柱计量和平台锁紧登记，再接实机三表采集。当前没有连接实体机器人。
