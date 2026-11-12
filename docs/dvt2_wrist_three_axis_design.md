# 左腕三轴补采：选两构型、四条分频记录

**推荐方案：在 `train_c00` 与 `train_c01` 各做一条三轴分频参考，两批各重复一次，共 4 条。** 固定当前训练记录，按原 R2 删构型归属计分。这个方案把最差线性误差放大从 **1.951× 降到 1.716×**；相同记录预算的分轴方案为 **1.723×**。两者总分接近，分频优先是因为它覆盖三轴，且改善了上一轮最差的 `train_c02` 删除情形，而不是因为总分相差 0.007×。

## 只用当前训练观测比较

输入是 004 左腕 yaw 补采协议的 **44 条训练**。保留 R2 的重力等式、固定尺度、训练行权重和共同可观测空间。目标是原 R2 **128 个冻结名义算子评估位置**上左腕 yaw、pitch、roll 的三个质量矩阵对角项；这些位置是参考几何，不是正式测试观测。没有打开选参或正式测试观测，也没有调用合成采集、拟合或读取 plant 真值。

把候选参考的解析 q/dq/ddq 送入 Pinocchio。匹配原 40 阶处理、1024 相位点和移动阈值；只给激励轴的力矩行计信息，保留粘滞与库仑项为待估干扰量，不拿静止轴的反力充样本。每条记录总权重相同：分频把一条记录的权重分给三轴。另给出「每轴权重相同」对照，以区分记录预算与单轴测量预算。这是局部线性筛选，不包含闭环跟踪、噪声、物理约束重新求解；放大倍数不换算为 R2 的实测百分比。

## 等记录预算下，分频更均衡

| 方案 | 新记录 | 删任一构型后的最差放大 | 删 `train_c02` 后三轴中的最差 |
| --- | ---: | ---: | ---: |
| 当前训练，不补采 | 0 | 1.951× | 1.951× |
| 单构型三轴分频 | 2 | 1.790× | 1.790× |
| 两构型分轴：c00 pitch，c01 yaw | 4 | 1.723× | 1.723× |
| **两构型三轴分频** | **4** | **1.716×** | **1.709×** |
| 三构型三轴分频 | 6 | 1.686× | 1.686× |

最小定义为：在所筛布局内，选等记录权重下距离最优最差项不超过 **2%** 的最少记录方案。2 条不满足，4 条比 6 条少三分之一记录、最差项只差 1.75%。删除任一构型时也删除归属于它的新增轨迹；删除任一批次时连同该批新增记录删除，最差线性指标为 1.413×，当前为 1.414×。

以每轴权重相同单独比较：两构型逐轴各采需 12 条、最差 1.685×；两构型分频需 4 条、最差 1.603×。这两行的有效总行权重和上表不同，不能拿 1.603× 当作推荐方案的预期。原 R2 删构型最差是 15.30%（1e-4 档）；本次线性指标约改善 12%，没有依据宣称会降到 10% 以内。

## 两条冻结参考，测量段只动左腕三轴

| 归属中心 | 左腕 pitch / roll 中心角（rad） | 两批 |
| --- | ---: | --- |
| `train_c00` | **0.00 / +0.30** | 各一条 |
| `train_c01` | **−0.30 / +0.30** | 各一条 |

保持原中心其余角度。6 s 基周期中，yaw/pitch/roll 分别用第 **7 / 11 / 17** 谐波（**1.167 / 1.833 / 2.833 Hz**），幅值 **0.04653 / 0.01884 / 0.00789 rad**，相位 **0 / π/3 / 2π/3**，稳态加速度峰值统一为 2.5 rad/s²。每条含四周期、两端各 2 s 的 C3 渐变与完整进退，总长 31.2 s。两条参考均过连续碰撞、URDF 限位和 2 rad/s、8 rad/s²、60 rad/s³ 参考导数约束；实际最紧导数上界约为 0.656、2.527、44.654，最小采样间隙 11.206 mm（连续门槛 10 mm）。

如果进入下一轮补采，先注册 **train 专用的窄幅三轴周期记录**，只用激励轴的力矩行。不能把它伪装成原有全臂大行程周期记录或非换向单轴探针。新增数据仍需实测信号、跟踪、独立重复及路径复核，再以原 10% 门槛重做 R1/R2；离线筛选没有变更门槛。

## 复现

```bash
run=artifacts/dvt2_004_wrist_supplement_local_v1
bash scripts/humble/container.sh python scripts/humble/design_wrist_three_axis.py \
  "$run/development" --analysis "$run/r2" --output "$run/three_axis_design_recheck"
```

本次 `three_axis_design_v1/report.json` 逐轴保存每个删构型和删批次的得分；`plans/` 保存两条参考及连续门禁证明。


## 补采已完成

本机采集目录：`artifacts/dvt2_004_wrist_supplement_local_v1/three_axis_capture_v1/`。两构型×两批的 4 条 `wrist_periodic` 训练记录均已采齐。独立采集种子为 3210008、3211008、3210108、3211108；控制 500 Hz，原始观测 4 kHz。四条实际观测路径均通过连续门禁，最小采样间隙 11.206 mm；有效周期内的最大位置跟踪误差 0.807 mrad。三轴各自频点的加速度 RMS 为 1.769–1.785 rad/s²；力矩频点含摩擦，不据此判断惯量可辨识性。

`capture_protocol.json` 绑定原开发协议、train 清单、冻结前序和设计；`train_capture_manifest.json` 封存四条新记录，`report.json` 汇总实测信号及路径。原会话不回写。采集和信号检查已完成；拟合接入和复核见下节。正式测试观测未读取或采集。

重新采集到新目录时使用：

```bash
run=artifacts/dvt2_004_wrist_supplement_local_v1
bash scripts/humble/container.sh python scripts/humble/collect_wrist_three_axis.py \
  "$run/development" --design "$run/three_axis_design_v1" \
  --output "$run/three_axis_capture_recheck"
```


## 接入开发拟合

新会话在 `artifacts/dvt2_004_wrist_three_axis_integration_v1/`。原 44 条 train 和 24 条 selection 保留原角色与采集身份；新增 4 条独立绑定为 train 专用 `wrist_periodic`，不重采或改写已有记录。

新增记录用实测 q/dq 和原 40 阶周期处理。拟合仅消费左腕三轴各自的运动行；固定 0.2 Nm 归一化，每条记录的总权重等于一个原探针，三轴均分。原 44 条训练的行权重逐项不变。R2 删中心／删批次沿用原掩码，新行随所属中心或批次一起删除。原全臂行程和静止门禁继续在原全臂数据上检查；新增窄幅记录按三个激励轴的预测误差及批次重复性评分，不把非激励轴力矩当观测。原 R1/R2 阈值未改。

本机 R1 重拟合通过，新增四条记录的最大归一化力矩误差为 **2.423%**（原门槛 5%）。

```bash
old=artifacts/dvt2_004_wrist_supplement_local_v1
out=artifacts/my_wrist_three_axis
bash scripts/humble/container.sh python scripts/humble/integrate_wrist_three_axis.py \
  "$old/development" --capture "$old/three_axis_capture_v1" --output "$out/development"
bash scripts/humble/container.sh casbot-calib staged-verify "$out/development" --output "$out/r1_verified"
bash scripts/humble/container.sh casbot-calib staged-identify "$out/development" --through stability --output "$out/r2"
bash scripts/humble/container.sh casbot-calib staged-verify "$out/development" --stability-analysis "$out/r2" --output "$out/r2_verified"
```

R2 若按门槛拒绝，分析命令返回 1；若已写出 `r2/delivery_manifest.json`，仍运行最后一条独立复核。原正式测试观测不参与。

### R1 通过，R2 删构型反而变差

独立复核覆盖 72 条参考与实际路径、拟合模型和合成系统各 10 条步长减半记录、无真值新进程重分析；全部通过。R2 的 128 个评估点与上轮逐项相同。

| 正则化 | 只补 yaw：删构型最大对角差 | 加四条分频：删构型最大对角差 | 加四条分频：删批次最大对角差 |
| --- | ---: | ---: | ---: |
| 1e-5 | 15.335% | 19.823% | 0.279% |
| 1e-4 | 15.300% | 19.820% | 0.277% |
| 1e-3 | 15.194% | 19.775% | 0.256% |

三档都通过删批次，但每档的三个删构型变体均未过原 10% 门槛。1e-4 档删 `train_c02` 时最差轴由左腕 pitch 变为左腕 roll；该删组的左腕 yaw/pitch/roll 对角差分别为 15.688% / 19.204% / 19.820%。独立 R2 复核完成，原生算子重算差为 0，没有合格档。正式测试观测未读取、未采集。

此结果与离线线性信息排序的改善方向相反。**不把四条分频方案升为默认。** 下一步先用已采四条记录检查匹配频带内的力矩残差，以及完整训练与删 `train_c02` 的解为何向不同方向移动；不继续按线性放大倍数追加采集。

后续[力矩目标反事实](dvt2_wrist_target_swap_diagnosis.md)已完成：保留四条分频的实际轨迹，仅换成 yaw-only 基准预测后，删 c02 最大差从 19.820% 降至 10.204%。优先查 roll 同相力矩残差，不继续按离线信息倍数补采。
