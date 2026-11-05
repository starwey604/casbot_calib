# DVT2原始模型包

此目录保存用户于2026-11-02提供的`~/下载/casbot_hl12p1_dvt2`模型。`source/`中的五套URDF/MJCF及整个公共assets目录按原文件复制，未执行下载目录中的`.generate_mjcf`。供应商场景和单网格MJCF只是原始参考，不作为当前机械计算或控制入口。

`catalog.json`记录来源、源URDF身份、五套装配、活动关节与惯性缺失清单。URDF单位按其格式约定解释为m/rad/kg，并非实测设备确认。`links_without_inertial`中既可能有无质量frame，也可能有尚未提供惯性的工具；导入阶段不自动补值。

- `a2`：普通双手，活动手指含mimic定义。
- `a2_ff`：普通双手，源文件已经固定手指。
- `b1`：左普通手、右吉他手，含固定乐器装配。
- `b2`：普通双手，另一整机装配配置。
- `b3`：双侧鼓槌，部分末端未给惯性。

实际千分计装配已由1105_03 STEP明确为拆普通手、裸法兰依次安装同一三表工具。`derived/bare_flange_1105_03_v1/`是前序N2实际装配候选；`derived/bare_flange_1105_03_gauge_v2/`细化六个表体的碰撞外包，源机械模型不变；当前工作状态CAD模型的八组分段进退参考已通过，见`docs/trical_1105_release_then_turn.md`，新默认尚未整体切换；参见`docs/trical_1105_hole_trials.md`。`derived/a2ff_stock_hands_candidate_v1/`提供N2生成的普通手14轴候选，保留前序普通手技术验证配置、不是活动默认。源文件的29/40/51个活动关节不能直接当成14轴SDK控制模型使用。

## 拉取与查找

源码检出后先运行`git lfs pull`。312个OBJ及派生碰撞NPZ使用LFS，其他源文件保留文本；不能用LFS指针文件构建或加载网格。源包的原始网格索引按原样保留，不作为本项目独立核验的模型身份。

Python通过`casbot_calib.model_sources.dvt2_source(variant)`取得本目录、URDF、公共assets路径和参考MJCF；`resolve_asset()`解析源URDF中的`visual/...`或`collision/...`路径，不依赖当前工作目录。

C++/ROS消费者使用CMake导出的`casbot_calib_MODEL_DIRECTORY`，其中的`casbot_hl12p1_dvt2/`与本目录结构相同。独立CMake安装需额外安装`Models`组件；ament正常安装包含该组件。Python wheel自带同源数据，不需要另找下载目录。

仓库使用说明：`docs/dvt2_model_sources.md`、`docs/dvt2_model_candidate.md`。派生规则在`configs/dvt2_model_candidate.yaml`；碰撞、viewer与运行配置绑定同一候选身份，不手工修改生成文件。
