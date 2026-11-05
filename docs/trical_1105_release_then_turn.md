# 八组球位进退：先脱离，再允许转向

当前1105_03台架原孔位、原高度、左+90°/右0°安装下，**左右臂各访问A/B/C/D的8组参考已全部通过**。左D、右C采用分段退出；右A修正保持姿态后沿释放锥退出，无需额外转向。

本轮闭合的是当前CAD工作状态模型的进退参考，尚未采集千分计数据或执行运动学辨识。工具自由伸出时的完整测杆包络未量测，实机前需补齐并重新验证；不能把工作状态模型中的正间隙直接当作实物自由行程保证。

## 先重建冻结参考，不必重新搜索

在已配置Humble环境的仓库根目录执行。输出目录须不存在；这些命令不连接SDK、不发送控制消息。

```bash
BUNDLE=python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2

bash scripts/humble/container.sh python -m casbot_calib.trical_paths \
  --bundle "$BUNDLE" \
  --seeds configs/trical_1105_release_turn_reference.json \
  --settings configs/trical_1105_release_turn_settings.json \
  --output artifacts/my_trical_reference

bash scripts/humble/container.sh python -m casbot_calib.trical_reference_validation \
  --bundle "$BUNDLE" --plan artifacts/my_trical_reference \
  --seeds configs/trical_1105_release_turn_reference.json \
  --output artifacts/my_trical_verification

bash scripts/humble/container.sh python -m casbot_calib.trical_assembly_view \
  --bundle "$BUNDLE" --plan artifacts/my_trical_reference \
  --output artifacts/my_trical_video
```

第一步生成`report.json`、逐案例`motion.json`和500 Hz `reference.npz`；第二步独立重加载并验证导出参考；第三步生成中文`trical_1105_workflow_zh.mp4`。规划或验证不通过时CLI返回非零。MuJoCo只更新几何，不计算机器人动力学、碰撞或接触力。

冻结参考已包含球位、安装及模型身份，不依赖本机旧搜索产物。换模型、移动球或平台后，不能直接复用它。当前分段搜索入口仅支持原布局，会拒绝非原位布局字段，不自动回退到原位。

## 退出分为三个区域

| 区域 | 条件与动作 |
|---|---|
| 接触区 | 保留普通10 mm、列明近接对2 mm、三测头/选定球5 mm有界行程。先退出释放锥；相对测量姿态的转角包络不超过1°，包含关节误差盒。 |
| 已脱离区 | 三个模型测头均已证明至少有0.5 mm正间隙，才允许改变朝向。之后各段继续维持这项正间隙，不再沿用负行程例外。 |
| 自由空间 | 所有零件对恢复普通10 mm规则后，才接入停放位过渡。反向接近按同一分段逆序检查。 |

新路径不再把原40/25 mm固定直撤长度作为切换条件，而是在整个误差盒满足普通10 mm规则的位置进入自由空间。新`withdrawal`模式中，`contact[].retract_m`及报告的`local_retract_m`表示TCP到球心的径向距离，不是曲线弧长；旧直线种子的轴向撤回距离含义保持。孔位粗筛器仍只接收旧式直线种子，不能将新曲线默认为直线重建。

位置误差盒始终为每轴±0.00105 rad；普通、近接和测头行程规则没有降低。每条参考以静止开始、静止结束回到home，左右臂之间停放后人工换装同一个表架。

实际导出结果：

| 案例 | 切换到正间隙区时TCP距测量点 | 本地退出时间 | 本地退出采样最大转角 |
|---|---:|---:|---:|
| 左D | 4.00 mm | 3.292 s | 7.95° |
| 右C | 4.00 mm | 2.854 s | 6.40° |
| 右A | 12.00 mm | 7.032 s | 约0.00053°，基本不转向 |

这些数值来自参考轨迹，不是实机测量。八条完整参考合计315.256 s、157,636个500 Hz样本，不包含人工换装等现场耗时。速度、加速度、jerk的连续上界分别低于0.35 rad/s、1 rad/s²、8 rad/s³。

## 保持姿态与退出路径一起检查

`trical_withdrawal.py`先复用上一轮接触姿态，针对关节误差盒生成最差角点约束，改进保持余量，再交给原生连续门禁确认。这样不再出现“名义点好看，但静止保持误差盒已经违规”的情况。

初始退出尝试三测头释放锥内的方向。三个模型测头脱离后，使用小范围关节步进优化改变朝向；局部优化只省略已用距离变化上界证明在整次查询范围内安全的零件对。优化器成功或角点抽检通过均不授予路径资格，最终仍检查完整C3曲线与误差盒。

新参考的相位规则在种子文件的`withdrawal`字段中，包含释放节点、正间隙下限和接触区角度上限。`trical_paths.py`统一生成500 Hz参考；旧种子未提供该字段时仍按原规则执行，不要求旧调用补新字段。三阶段共用同一套Pinocchio/Coal几何，不复制Mock、SDK采集或物理内核。

## 局部运动上界减少无效细分

原全局运动半径覆盖整个关节范围，在短退出段上过松，曾耗尽30,000及100,000次查询预算。现在只在明确的局部关节盒中收紧上界，不缩小几何或余量。

设盒中心为`q_c`、半宽为`h`。若关节`j`只影响一个碰撞体，其半径可按以下方式上界：

```text
局部半径(j)
  ≤ 中心姿态下，凸包顶点到该关节轴的最大垂距
    + Σ 严格后代关节(k)的全局位移半径 × h[k]
```

所有位姿和轴线来自原生Pinocchio FK。祖先对该碰撞体和关节轴施加同一刚体变换；关节自身转动不改变垂距；严格后代引起的位移由原全局上界覆盖。两个碰撞体共有的祖先仍为零贡献。取该上界与原全局上界中的较小值，再用原Coal距离细分检查。

对一条关节弦，使用中心`(q_a+q_b)/2`和半宽`abs(q_b-q_a)/2+位置误差盒`，因此上界覆盖整条路径及误差盒。`local_motion_bounds`为显式选项，旧调用默认关闭。本轮冻结设置启用它，并声明100,000次查询、32层关节细分的离线预算；这些是证明计算预算，不是控制器或通信频率。

数值检查覆盖局部盒内原生顶点运动、共享祖先、重新计算不同盒时不复用旧局部上界，以及上一轮真实违规反例仍被拒绝。同一规划运行中，完全相同几何、阈值、端点和设置的证书可复用；接触和已脱离规则使用不同缓存键。

## 独立复验与中文演示

本机交付产物：

- `artifacts/trical_release_turn_frozen_v1/`：从仓库冻结配置直接重建的8/8完整参考及相位证书；与搜索得到的`trical_release_turn_routes_v4/plan`参考数组相同。
- `artifacts/trical_release_turn_frozen_verify_v1/`：新进程重加载冻结配置，禁止优化和模拟器执行，192段连续复验通过；逐点核对导出的q/dq与C3多项式一致，并执行角点抽检。前次独立记录保留在`trical_release_turn_verify_v1`。
- `artifacts/trical_release_turn_corners_v1/`：另用原生完整Coal几何，对三个新案例做3,968个角点采样；最小采样余量约0.540 mm。抽检不替代连续证书。
- `artifacts/trical_release_turn_report_v2/`：TCP路径、姿态变化中文图及CSV。
- `artifacts/trical_release_turn_render_v1/`：中文MP4、关键阶段PNG、viewer核对及实际视频解码结果。
- `artifacts/trical_release_turn_tests_v2.log`：22项相关测试通过；`trical_withdrawal_final_api_tests_v4.log`另覆盖包括旧种子兼容、模型/布局绑定和曲线语义在内的10项最终API检查。

这些本地产物不随Git提交；配置和源码已纳入仓库。前序失败尝试保留在`trical_release_turn_routes_v1/v2/v3`，分别用于定位名义优化忽略误差盒、全局上界查询预算不足等原因；中断记录注明已完成范围，不计作完整通过。

如需重新搜索，而不是复用冻结参考：

```bash
bash scripts/humble/container.sh python -m casbot_calib.trical_withdrawal \
  --bundle "$BUNDLE" --seeds configs/trical_1105_withdrawal_seeds.json \
  --output artifacts/my_hold_search

bash scripts/humble/container.sh python -m casbot_calib.trical_withdrawal \
  --bundle "$BUNDLE" --holds configs/trical_1105_withdrawal_holds.json \
  --output artifacts/my_release_search
```

## 下一轮进入N3，当前不局部切默认

本轮完成N2工作状态模型的代表性八组进退路径闭合。新模型还未整体接管动力学、运动学、SDK Mock和采集入口；N3须按[模型替换计划](dvt2_model_migration_plan.md)一起迁移，不能混用旧默认会话。真实工具自由测杆包络、安装计量和工具负载仍按实物核验；实机权限与SDK前置条件不因8/8参考通过而改变。
