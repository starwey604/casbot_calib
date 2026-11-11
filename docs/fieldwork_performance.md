# 调参GUI性能优化

## 保留控制精度，减少显示开销

本轮只改GUI及其几何显示管线。手动Mock的1 kHz控制/反馈、50 µs积分、1 kHz SDK发布，以及原生发送器的500 Hz配置不变；不降采样原始CDR/反馈记录，不修改重力、PD、碰撞规则或启动姿态。

主要热点是软件绘图，而不是Python滑条计算：

- 原窗口关闭空闲限帧，软件OpenGL全窗口多重采样；GUI和3D进程分别建立Mesa工作线程池。
- 3D每帧绘制约129万个三角面，包括完整腿部、机身和传感器细节。即便只是反馈接收时间变化或微小位置噪声，也会重画。
- GUI每帧同步写入并`fsync`心跳；视图邮箱也使用了证据文件的持久化写法。

现在：

- 窗口交互刷新上限30 Hz，空闲刷新目标20 Hz；保留字体/ImGui边缘抗锯齿，关闭全窗口MSAA。实际帧率取决于显示驱动与机器负载。
- Mesa软件光栅工作线程默认每个绘图进程2个；不改DDS、物理推进或BLAS线程配置。显式的`LP_NUM_THREADS`覆盖默认值，包装脚本会转发它。
- 实时视图使用同源轻量网格：约25.4万个三角面，比原网格少约80%。关节轴、原点、层级、缩放、材质和SDK反馈映射不变。**只有显示资源被替换；碰撞、动力学和导出视频继续用原模型。**
- 轻量网格离线构建时检查双向顶点及三角形重心到表面的距离，最大不超过2 mm；每轴包围盒极值变化不超过0.5 mm。此数值用于显示质量，不是碰撞或实物外形证明。实时窗口无需Blender或额外Python依赖。
- 3D刷新上限15 Hz；仅接收时间变化不重画。相对上次实际绘图姿态，各轴变化均不超过`1e-4 rad`时略过重画；慢运动累积越过该值仍会更新，选轴高亮立即使视图待更新。数值反馈和原始记录不取整、不经过这个显示死区。
- 心跳每帧仍走管道，取消/关闭优先于文件I/O；兼容心跳文件每秒更新。可重建的视图/心跳邮箱改为完整文件原子替换，不强制同步磁盘。协议封存、事件与正式记录的持久化规则不改。
- 图像纹理复用已分配空间，不重复分配OpenGL纹理。

## 同机软件渲染对照

私有localhost Mock、Xvfb、1440×980窗口、640×480模型图像，每阶段测量约10秒；启动和首次模型加载不计入阶段。下表统计GUI及其子进程，包括Mock、渲染、只读记录和控制进程；不含Xvfb测试显示服务或其它后台任务。**100%表示一个CPU核**，不是整机百分比。

| 阶段 | 优化前CPU | 优化后CPU |
| --- | ---: | ---: |
| 空闲窗口，Mock仍运行 | 373% | 151% |
| 已连接，只读观察 | 418% | 190% |
| 左肩pitch小幅目标变化、发送中 | 408% | 272% |

发送阶段GUI进程本身从189%降至51%，3D实际更新从约3.5帧/秒提高到6.3帧/秒。未连接/静止时不再反复重画模型。物理仿真和原始高频记录仍有计算成本；CPU计时与机器负载、频率调节有关，以上是本机实测，不是其它设备的预算保证。

对照输入与原始数据：

- 优化前源码：`artifacts/fieldwork_performance_before_v1/source/`。
- 优化前计时：`artifacts/fieldwork_performance_baseline_v3/profile.json`。
- 优化后计时：`artifacts/fieldwork_performance_after_v3/profile.json`。
- 显示网格清单及逐网格偏差：`python/casbot_calib/assets/fieldwork_display/dvt2_v1/manifest.json`。

这些是本机产物路径，计时记录不随源码分发。窗口未作截图或外观验收，外观由用户核对。

78项相关Python检查通过；实际私有Mock中，GUI生成的2秒小幅短轨迹以约29.8 Hz更新目标，正常返回原目标并停止。另将GUI冻结3.2秒，发送器按2秒心跳规则终止、显示超时原因，独立只读反馈继续；未在STOPPING之后发布新指令，CDR保留、前馈为零。汇总：`artifacts/fieldwork_performance_delivery_v1/verification.json`。

## 使用与复测

照常启动即可，无需重编C++：

```bash
bash fieldwork/run.sh gui --mock --domain-id 181 --software-rendering
```

有可用显卡驱动时省略`--software-rendering`；已有独立Mock的连接方式不变。`--mock`会另起本窗口的Mock，不需要同时保留旧的独立服务。

非可视CPU复测（会启动并控制一个自有私有Mock，不连接实机）：

```bash
bash scripts/humble/container.sh bash scripts/humble/vendor.sh \
  env LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s '-screen 0 1600x1100x24' \
  python scripts/humble/profile_fieldwork_gui.py --domain-id 196 \
  --output artifacts/my_gui_profile
```

可加`--short-trajectory --seconds 5`测试GUI短轨迹；`--stall-seconds 3.2`测试界面卡顿后的心跳超时。`--exercise-input`仅注入ImGui鼠标事件，不移动系统指针。各次输出使用新目录，保存CPU计时和原始控制/反馈记录，不生成截图。

轻量网格重建仅由维护者离线执行，使用Blender 5.2.3和冻结CAD来源，输出必须是新目录：

```bash
blender -b -t 2 --factory-startup \
  --python scripts/cad/build_fieldwork_display.py -- \
  --output /tmp/casbot_display_rebuild
```

重建清单通过偏差和库存检查后再替换随包显示资产；不把生成物写回冻结标定模型包。
