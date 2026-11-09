# 读取已导入的DVT2源模型

N1已导入五套源模型和公共资源；Python、独立C++安装和ROS安装读取同一来源。当前默认为`derived/bare_flange_1105_03_gauge_v2`：固定腰部、14轴双臂、普通手拆除；千分计使用1105_03实际四球装配及已闭合的八组进退参考。下述五套源API仍用于来源检查，不是五套默认运行选项。[普通手候选](dvt2_model_candidate.md)保留为前序技术证据；当前状态见[N4收尾](dvt2_n4_closeout.md)。

## 先取得实际网格，不要构建LFS指针

```bash
cd ~/codings/casbot_calib
git lfs pull
```

源包位于`python/casbot_calib/assets/models/casbot_hl12p1_dvt2/`，公共资源625个文件、140,835,049字节（约134.3MiB），其中312个OBJ使用LFS。`.gitignore`只对这个目录放行OBJ，其他目录的编译产物仍忽略。五套源URDF/MJCF及供应商参考场景保持原样；`.generate_mjcf`没有导入或执行。原始数据目录禁用Git换行转换，避免Windows检出改变源URDF身份。

`catalog.json`是装配目录和来源身份；`source/`是原始文件。不在多个模块复制模型，也不从`~/下载`回退读取。供应商场景和MJCF仅作参考，当前仍由Pinocchio/Coal/Boost承担计算，MuJoCo只显示。

## Python显式选择一套源配置

```python
from casbot_calib.model_sources import dvt2_source, inspect_source

source = dvt2_source('a2_ff')
print(source.urdf)
print(source.resolve_asset('collision/CASBOTHL12/base_link.obj'))
report = inspect_source('a2_ff')
print(report['urdf_meshes'])
```

选择配置是必需参数；未知名称不会回退到旧模型。这个API返回源资源，不启动模拟器、SDK或控制循环，也不自动约简关节。五套源URDF的活动关节数分别为51、29、40、51、29；固定腰部14轴候选已另存于`derived/`，该源API仍不隐式返回派生模型。

`inspect_source()`核对源URDF身份、双臂名称映射、URDF/参考MJCF资源及OBJ材质引用。缺文件或只有LFS指针会直接报错。关节完整源列表按URDF声明顺序记录，不冒充SDK或Pinocchio的广义坐标排列；`calibration_joint_names`另列既有左7/右7逻辑顺序，N2按名称建立映射。

可运行完整源资源检查，输出目录须不存在：

```bash
bash scripts/humble/container.sh python scripts/humble/verify_dvt2_sources.py \
  --output artifacts/my_dvt2_sources
```

该检查实际调用既有C++加载器和Pinocchio/Coal，读取五套完整模型的显示/碰撞网格；不执行运动，也不证明新场景路径无碰撞。

## C++安装额外选择Models组件

```bash
bash scripts/humble/container.sh cmake --preset humble
bash scripts/humble/container.sh cmake --build --preset humble
bash scripts/humble/container.sh cmake --install build/humble --component Core
bash scripts/humble/container.sh cmake --install build/humble --component Models
```

CMake中`find_package(casbot_calib CONFIG REQUIRED)`提供`casbot_calib_MODEL_DIRECTORY`。典型URDF路径是`${casbot_calib_MODEL_DIRECTORY}/casbot_hl12p1_dvt2/source/a2_ff/robot_description_formatted.urdf`，公共资源根是同一包下的`source/assets/`。

该变量随安装前缀重定位，不编入用户下载路径。Models是独立组件，旧的纯Core消费者不因新资源而新增安装前提；只有读取这些源模型时才需要Models。

ROS使用原有`build_ros.sh`安装；`find_package(casbot_calib_core REQUIRED)`经既有薄封装提供同一变量和资源。不要在ROS包里另拷一份网格。

## N1验证与下一步

本机证据目录为`artifacts/dvt2_n1_sources_v1`、`artifacts/dvt2_n1_tests_v1`、`artifacts/dvt2_n1_install_v1`、`artifacts/dvt2_n1_wheel_v2`及`artifacts/dvt2_n1_sdist_v1`，不随Git提交。17项专项测试通过，源码分发包包含实际网格而非LFS指针。安装检查包含无Models的原Core消费者、附加Models后的重定位、ament消费者和禁用editable/源码重定向的独立wheel。具体结果汇总见`artifacts/dvt2_n1_delivery_v1/delivery.json`。

N2普通手候选和新场景的构建、核验及安装接口见[候选模型指南](dvt2_model_candidate.md)。实际已明确拆普通手、同一三表工具左右换装；N2先补齐路径闭合，N3才统一切换旧nominal与既有场景。
