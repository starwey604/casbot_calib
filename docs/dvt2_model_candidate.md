# 读取当前 DVT2 裸法兰模型

当前模型为 `bare_flange_1105_03_gauge_v2`：固定腰部、14 轴双臂、拆普通手，使用
`trical_1105_03_gauge_cover_v2` 的三表碰撞资产。旧普通手候选与前版裸法兰包已删除，历史见 Git。

在仓库中物化模型后，用现有 Humble 环境读取：

```python
from casbot_calib.model_context import load_context

context = load_context()
engine = context.engine
print(engine.gravity(context.home))
print(context.collision_profile().check(context.home)['distances'].min())
```

原生与 ARM64 wheel 均使用仓库侧大型资产。预编译安装自动设置 `CASBOT_PROJECT_ROOT`；
自行安装 wheel 时，将它设置为仓库根目录。小型默认 JSON/XML 仍随 wheel 安装。
`bash calib/setup.sh --check` 检查 URDF、碰撞资产和数值依赖。

GUI 从同一上下文生成显示场景，启动时检查显示网格：

```python
adapter = context.write_view_scene('artifacts/view/scene.xml', active_tool='left')
```

C++ 消费者可读取同一 `model.urdf`；重力方向取 `model.json` 的
`profile.gravity_world_m_s2`。开发安装可额外选择 `Models` 组件。
Orin 无头部署只同步模型描述符、URDF、两套碰撞 NPZ，显示资产在本机 GUI 工作区使用。
