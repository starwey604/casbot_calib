# 动力学候选URDF导出与重加载验证

已提供`export-dynamics-urdf`和`verify-dynamics-urdf`，将已接受的S4重力/惯量候选导出为独立URDF，并通过Pinocchio重加载核验。原nominal URDF、已封存分析、原始观测和控制参数不回写；没有连接设备或改变实机执行资格。

## 输入与导出内容

输入是S4分析目录，需通过manifest校验，analysis及所请求阶段均已通过。`--stage inertia`另核对candidate与接受的刚体/摩擦拟合及起动摩擦冻结结果一致；拒绝未封存、被修改、阶段失败或模型身份不同的输入。目前支持绑定的nominal新CASBOT固定腰部14轴，不能直接换成evt6或经过几何修正的另一套坐标定义。

两种导出模式：

- `--stage gravity`：导出重力阶段选出的完整物理参数代表；惯量未被静态数据辨识的部分仍来自先验/物理约束，不声称已标定转动惯量。此模式的软件摩擦项未辨识、设为禁用，不代表要清零SDK原有摩擦补偿。
- `--stage inertia`（默认）：导出通过分阶段验收的完整候选，包括惯量阶段允许的有限修正。

S4新分析会记录`source_model_sha256`。旧的完整S4分析可从同一封存会话中已接受的inertia candidate取得模型绑定；旧的仅gravity分析若缺少源模型身份则拒绝导出，需要按新入口在新目录重分析，不猜测或改旧报告。

输出文件：

| 文件 | 内容 |
| --- | --- |
| candidate.urdf | 各运动关节等效体的质量、质心、质心惯量；几何/关节/固定坐标系及限制保留 |
| calibration.json | 源模型/输入摘要、关节顺序、140参数代表、可辨识性、固定体聚合映射及运行时参数 |
| bodies.csv | 14个等效体的质量、质心及“参数代表”状态，不能当作逐个真实零件的称量结果 |
| report.json | 重加载检查项、最大绝对误差、容差、构型种子及适用边界 |
| manifest.json | 上述文件摘要；后续重加载先校验，输出目录必须为新目录 |

## 质量、质心与固定手部怎样写回

当前拟合的是**运动关节坐标系下的等效刚体**，腕部已包含固定手部/工具的惯性。如果直接把等效腕部参数写入腕链接，同时保留原手部惯性，重加载会把手部计入两次。

本版采用明确的聚合表示：

1. 每个运动关节的完整等效惯性写入其直接子链接。
2. 属于同一运动关节的固定后代链接移除原`inertial`，但保留link、joint、visual、collision和固定变换。JSON记录被聚合的链接，不假装恢复了各手部零件的独立质量。
3. 固定根部的base/腰部惯性保留原值；固定基座关节观测没有标定这些参数。
4. 使用Pinocchio的`Inertia.FromDynamicParameters`及空间变换转换到目标链接坐标；URDF的`origin.xyz`写质心、`origin.rpy=0`，张量为在链接坐标轴中表达的**质心惯量**。不把关于关节原点的惯量直接填入URDF，不手写另一套平行轴/刚体聚合算法。

源与候选的基座、关节树、轴、帧、限制必须一致。几何标定后的坐标改变不能隐式混入这次惯性导出；需要先建立新的源模型绑定和验证。

## URDF与运行时参数的边界

拟合的非线性摩擦（含起动/Stribeck相关参数）、输出侧折算转子惯量和世界重力向量在`calibration.json`中显式保存，不追加进link刚体惯量，也不伪装成URDF通用摩擦字段。

`load_exported_dynamics(folder)`先从URDF载入刚体，再从JSON恢复软件模型的摩擦、转子和重力设置；不会用JSON里的刚体参数覆盖URDF载入结果。裸URDF用于刚体模型，完整软件前馈模型需要配套JSON。控制器自己的摩擦/重力补偿仍须按照[SDK实机前置条件](sdk_hardware_prerequisites.md)核实，此入口不会下发参数、关闭补偿或替换主机配置。

全部visual/collision定义保留，但当前nominal网格没有随包复制。动力学重加载通过不代表RViz/MuJoCo网格路径已解决。导出不复用旧URDF摘要下的几何资格，也不开放实机；后续使用须重新核对模型/碰撞与设备条件。

## 验证方法

重加载同时使用直接Pinocchio URDF解析和现有C++适配器，与冻结的候选模型比较：

- 运动关节顺序、140参数、各等效体质量/质心/质心惯量。
- 非惯性XML内容及固定根部惯性保持一致。
- 32个固定种子的限位内数值构型：重力、刚体质量矩阵、速度耦合和逆动力学。
- 所有具名帧的位姿不变。
- 恢复配套参数后，完整软件模型的质量矩阵与逆动力学一致，转子只计一次、摩擦不丢失。

比较使用绝对/相对容差各`1e-10`，具体误差及单位写入报告。随机构型仅作数值校验，不是经碰撞门禁的可执行轨迹。验证的结论是**序列化、坐标及运行时恢复一致**，不代表参数唯一可辨识、真实机械质量正确或新的实机验收。

## 使用

在仓库根目录运行，输入会话必须存在，输出须使用新目录：

```bash
# 完整动力学候选
bash scripts/humble/container.sh casbot-calib export-dynamics-urdf \
  artifacts/vendor_s4_analysis_v1 --stage inertia --output artifacts/my_dynamic_urdf
# 重力候选：适合先检查静态重力补偿模型
bash scripts/humble/container.sh casbot-calib export-dynamics-urdf \
  artifacts/vendor_s4_analysis_v1 --stage gravity --output artifacts/my_gravity_urdf
# 仅依赖导出包与相同版本的已安装源模型，不需要旧会话/原始采集/真值
bash scripts/humble/container.sh casbot-calib verify-dynamics-urdf \
  artifacts/my_dynamic_urdf --output artifacts/my_dynamic_urdf_recheck
```

Python使用：

```python
from casbot_calib.dynamic_export import load_exported_dynamics
engine, metadata = load_exported_dynamics("artifacts/my_dynamic_urdf")
# engine.gravity(q), engine.mass_matrix(q), engine.inverse(q, dq, ddq)
```

本机已从`artifacts/vendor_s4_analysis_v1`分别生成`artifacts/dynamic_urdf_release_v1/gravity`和`inertia`。额外新进程拒绝访问原分析目录和任何evaluation路径后，两套包仍通过`verify-dynamics-urdf`；验证没有重新拟合，也未读取P0正式test或改动S4冻结产物。详细结果见该release目录中的`delivery.json`。

本轮全量375项Python测试通过，含16项新增导出测试；14条已有Matplotlib/Pyparsing弃用警告。反例覆盖重复手部质量、轴/几何改变、未封存/被修改/失败候选，以及转子重复计入。两套实际导出在32个构型上的重力最大绝对差不超过4.45e-15Nm、刚体质量矩阵最大差不超过3.89e-16kg·m²，具名帧位姿差为0。检查只证明导出与候选一致，不宣称原始参数无误。C++力学内核未修改，未重跑历史114项冻结数值对照。
