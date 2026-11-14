# 使用DVT2固定腰部双臂候选模型

N2已生成可独立加载的14轴普通手候选。它保留源整机的固定部件、左右实际装配和惯性，用同一模型生成碰撞资源及只读显示场景；这是前序普通手技术候选，当前默认已切换为`bare_flange_1105_03_gauge_v2`，见N4收尾；本页以下保留该候选的原验证条件。

**本页保留前序普通手候选的使用说明。** 2026-11-05的实际装配已由1105_03 STEP明确为拆普通手、裸法兰依次安装同一三表工具；当前候选、孔位与升降结果见1105_03说明。本页的`assembly_confirmed: false`只描述这份普通手历史候选，不代表仍需询问用户。

## 从已安装的包读取候选

先在仓库运行`git lfs pull`，然后使用既有Humble环境。候选随Python wheel和原生`Models`组件安装，不依赖`~/下载`或当前目录。

```python
from casbot_calib.model_sources import model_directory
from casbot_calib.dvt2_bundle import load_bundle, robot_profile, viewer_adapter

folder = model_directory() / 'casbot_hl12p1_dvt2/derived/a2ff_stock_hands_candidate_v1'
engine, descriptor = load_bundle(folder)
q = descriptor['profile']['home_rad']
print(engine.gravity(q))
print(robot_profile(folder).check(q)['distances'].min())
adapter = viewer_adapter(folder)
```

这里仍使用既有C++ Pinocchio内核和Coal，不额外要求Python Pinocchio、Coal或MuJoCo。显示时才需要viewer依赖：

```python
from casbot_calib.viewer import JointView

# frame是同一模型身份、左7/右7顺序的标准StateFrame。
with JointView(adapter, folder / 'scene.xml') as view:
    rgb = view.render(frame)
```

不复制Mock或SDK适配层；以后将该标准观测接到同一viewer即可。`scene.xml`没有执行器/控制器，显示路径只更新几何，未调用MuJoCo的物理推进、动力学或碰撞计算。

C++/ROS使用`${casbot_calib_MODEL_DIRECTORY}/casbot_hl12p1_dvt2/derived/a2ff_stock_hands_candidate_v1/model.urdf`。原生加载器读取URDF；Python上下文另外应用配置的世界重力、碰撞资源和场景。C++消费者若采用其他安装/重力方向，也须显式设置重力，不能仅加载URDF后假定它保存了重力向量。

## 锁定、坐标和负载分别配置

派生规则在`configs/dvt2_model_candidate.yaml`，完整配置也封存在候选的`model.json`中：

- 原始`a2_ff`有29个活动轴；显式锁定双腿12轴、腰1轴和头2轴为零，保留双臂14轴，左7/右7顺序不变。手指沿用源文件的固定装配。
- 首先保留源世界坐标，`world_from_source`另设安装变换，重力单独记为世界向量。当前安装未实测；地面Z=0，新腰部frame Z=1.040582 m，不拿旧场景坐标平移拼接。
- URDF保留全部固定link及其惯性；Pinocchio在运行时聚合固定惯性。左右手从各自源文件读取，不镜像质量/质心，也不再加一遍末端质量。
- 三表TCP为手掌frame沿局部Z方向280 mm的**合成无质量frame**。测头/壳体/连接杆是碰撞代理，没有加入未知实物工具负载；实际TCP、工装质量/质心必须在装配冻结后补齐。
- 源惯性是模型数据，不是此次标定结果。摩擦未标定，原生刚体模型采用显式零摩擦；URDF的原始`dynamics`属性不等于运行时已补偿摩擦。

`model.json`绑定模型来源、安装、重力、锁定姿态、工具、几何、碰撞排除和场景身份。运行时拒绝URDF、缓存、排除关系、配置或显示资产与封存身份不符的组合。文件被修改后应重新生成、验证新候选；不能只重命名为旧批次的模型。

## 新场景怎样检查碰撞

碰撞缓存按源文件的**每个collision几何**生成，而不是将整只手合成一个包络。74个源几何包含网格和16个球体；球体用外包凸多面体，网格用包含所有源顶点的凸包，Coal计算距离。

排除项仅来自最近的碰撞承载父子link，以及源MJCF中另行审查的左右腕yaw/roll壳体两对重叠。没有导入厂家MJCF的大范围排除列表；腕部壳体的例外不扩展到手掌或手指。原生检查器沿用同一刚体内几何不互检的约定，因此固定部件/固定夹具之间的装配干涉不属于动态路径门禁。

场景重新生成腰部抱紧盒体、地面、左右各三球、球杆、双侧三表代理。六球中心由**新模型FK**产生，第三球避开前两球接近通道。球架和夹具都是合成尺寸，不代表已有实物测量。

每球检查home→预接近→接触位→退出→home，包含左右工具和腕部的明显运动，共24段连续路径：

- 非接触余量仍为10 mm；关节位置包络为每轴±0.00105 rad。
- 仅选中球与当前三测头的三对接触可用5 mm有界行程，其他球、壳体和手部不放宽。
- 球包络提高到512个采样点，减少保守外包误差，同时维持上述球体尺寸、位置包络及余量。
- 当前home最小名义距离约11.45 mm，来自腰部壳体与肩部。这个余量不宽裕；N3改轨迹、N4扩大工作范围时仍须重验，不能泛化为全空间无碰撞。

这些是场景与参考路径验证，不是K3完整采集配置；`trical_scene.json`显式标记`scene_only: true`。K1–K4工作流与新默认的接入归N3/N4。

## 重新生成和验证

构建阶段使用Humble中已有的Python Pinocchio与SciPy；运行时不重新生成CAD/凸包。输出目录须不存在，避免覆盖冻结包或原始数据：

```bash
bash scripts/humble/container.sh python -m casbot_calib.dvt2_build \
  --config configs/dvt2_model_candidate.yaml \
  --output artifacts/my_dvt2_candidate

bash scripts/humble/container.sh python -m casbot_calib.dvt2_validate \
  --bundle artifacts/my_dvt2_candidate \
  --output artifacts/my_dvt2_check --scene --render
```

第二条命令生成`verification.json`、`scene.json`、三幅中文PNG；场景门禁通过时再输出`dvt2_candidate_zh.mp4`。视频是已通过门禁的参考路径加速回放，不包含新增标定观测或机械计算。

完整源模型对照不只验证“导出后再读回”：以原29轴URDF锁定非臂坐标，独立计算源frame/TCP、14轴质量矩阵子块、重力和逆动力学，与派生模型比较。测试另覆盖非零腰/头锁定、旋转安装和非竖直世界重力。

本机本轮汇总见`artifacts/dvt2_n2_delivery_v1/delivery.json`：39项相关测试、完整源模型对照及24段场景门禁通过；独立wheel与Models/ament安装通过。最终中文回放为`artifacts/dvt2_n2_render_v3/dvt2_candidate_zh.mp4`，已解码查看；场景原始证据见`artifacts/dvt2_n2_validation_v1/scene.json`。这些本地产物不随Git提交；Linux已执行，Windows实际运行留待CI。旧模型的数值验收仍归旧输入，不移作新模型标定成功的证据。

下一步先确认保留手/拆手和末端安装，再由N3统一切换消费者；N4重跑新模型的动力学、SDK Mock和K1–K4流程。
