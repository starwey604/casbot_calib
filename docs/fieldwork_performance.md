# 调参GUI性能优化

> ROS GUI 历史记录。main 的图形入口已迁到 [SHM 本机调参台](shm_gui.md)；本文旧验收/性能脚本在 ROS 备份分支查看。

## 第一轮记录：保留控制精度，减少显示开销

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

## 使用与复测（第二轮须先构建，见文末）

完成本轮原生构建后，仍按原命令启动：

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

## 第二轮：减少绘图和高频路径成本（2026-11-12）

第二轮保留上面的轻量显示资源，但替换了显示调度和搬运路径：

- GLFW/ImGui输入与监督约60Hz，绘图独立预算：静止2Hz、反馈运动10Hz、输入/短轨迹30Hz；隐藏窗口暂停绘图但继续监督。短轨迹目标在监督循环计算，不依赖绘图帧率。
- 实时RGB改为480×360，使用有锁双缓冲memfd共享内存，不再逐帧写/读`frame.npy`。视频仍用原640×480模型。
- 不动的机身与地面缓存颜色/深度；只绘制实际运动的双臂并深度合成。四个代表姿态与完整绘制逐像素一致（`fieldwork_perf2_scene_comparison.json`）。
- ImGui采用单通道字体纹理、持久帧缓冲与增量损伤区域。63帧与全量绘制逐像素一致（`fieldwork_perf2_scissor_check_v1`）；未截图或查看外观。软件光栅默认串行`LP_NUM_THREADS=0`，显式环境覆盖仍有效。
- 高频数值序列化使用17位round-trip表示；手动邮箱只在文件实际变化后解析。原始消息不按数值去重。
- 仅手动GUI的记录器按20ms成组提交：先提交CDR再发布对应JSON索引。正常结束排空；硬杀可能丢失尚未提交的约20ms尾段。S1默认仍逐回调提交。
- `gui --mock`默认不写私有物理诊断CSV，仍完整保留SDK原始CDR/JSON和实际命令/反馈。需要私有诊断时加`--mock-diagnostics`。独立SDK Mock/标定调用默认仍完整诊断、原定时调度；GUI自有Mock可合并相等周期的三个定时器，频率与50µs积分不变。
- 零干摩擦模型跳过恒零的指数运算/临时量；20,000个实际五字段设备输入的前后q/dq/current差均为0。非零摩擦路径未改。

本轮包含原生代码，更新后先执行`bash scripts/humble/container.sh bash scripts/humble/build_vendor.sh`，并按项目Humble安装流程更新原生核心。启动方式不变。

最终合并定时器标准模式`artifacts/fieldwork_perf2_coalesced_v2/profile.json`：空闲65%、只读87%、发送179%，对第一轮151%/190%/272%；发送阶段实际画面约8fps，窗口约9.9fps。100%=一个核。软件渲染活动整套≤150%的目标尚未达到，不能称为达标。保留物理推进、完整SDK双录制仍是主要固定成本。另有旧独立Mock约占0.9核，不属于本窗口自有进程，未擅自关闭。

正常停止、隐藏恢复及发送中保存通过；保存记录13,429条CDR独立解码与JSON字段一致。短轨迹目标写入实测约26Hz并返回，GUI冻结3.2秒的心跳终止与只读继续通过。102项相关Python、5项SDK和2项核心CTest通过；S1实际DDS契约另见`artifacts/fieldwork_perf2_s1_v1`。这些是本机私有Mock证据，不代表实机资格。
