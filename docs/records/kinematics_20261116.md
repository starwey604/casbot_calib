# K1–K3 实施与验证（2026-11-16）

## 已交付

入口为 `bash calib/kinematics.sh`：K1 左臂采集后失能退出，换装后 `--resume` 执行 K2、K3；`--reanalyse` 从二进制反馈重算窗口关节均值及辨识。smoke/small 输出名义残差诊断，full 输出零偏、几何 URDF、汇总与参数包。用法见 `docs/kinematics.md`。

- 新增 `shm_trical_executor`：1 kHz 独立保持、C3 分段、windup 限幅、裸法兰、名义重力及工装重力前馈、故障/信号/EOF 失能。
- USB 三槽一问一答、独立缓存 Mock、关节停稳与读表包络、首点两步入球、连续路线检查及有界粗修正。
- 14 零偏＋5 台架参数、正交补空间几何列选择、两层拟合与逐臂留出比较；URDF 几何与零偏分别导出。
- H 系列构建及入口保留；会话按 dynamics/kinematics 分开续跑。GUI 保存位改为 `gui_pose.yaml`。

## 路线与数值验证

实际通过连续路线检查的 full 设计为 48 点：每臂每球 4 个训练点、2 个留出点。19 列基础回归量满秩，缩放条件数 **131.5563**（`artifacts/kinematics/actual-design.json`）。候选姿态先做接触路线筛选；训练/留出最小关节距离为 0.02 rad。

- 使用现有连续弦段碰撞证明；接触进退及接触姿态转移分别检查。固定 ±20° 准备位到路线起点也做连续碰撞检查。
- 左右 A 球均按实际 FK 距离构造 5 mm、2.5 mm、0 三个入球位置；右臂原有 2 mm 网格已修正。small 物理闭环和双方几何测试覆盖最终路径。
- 修正增量累加到保持指令，并按实测位置限制 windup；再限制实际 FK 笛卡尔步长 ≤1.5 mm。静摩擦用例中腕部保持不动，肩肘完成入量程。
- 合成辨识包含 14 轴隐藏零偏、台架 2 mm / 0.2° 偏移和可观测几何误差，留出 RMSE <10 µm；解析 Jacobian 与有限差分一致，导出 URDF 的 FK、质量矩阵重加载误差 <1e-12。

## 原生验证

证据均在 `artifacts/kinematics/`：

- `native-validation-rest.log`：完整 pytest **343 passed**，原生消费者和非 editable wheel 检查通过。
- `native-validation.log` 的 5 项 CTest 通过；TriCal 时序项首次全程最大间隔为 5.992497 ms。测试改为统计规格要求的 5 s 静默保持区间，同时保留全程间隔。独立重测 `native-quiet-final.log` 覆盖分段、EOF、驱动故障，静默和全程最大间隔均为 **3.513912 ms**。
- 后续修正的定向测试：`final-correction-tests.log` **11 passed**；配方验证提前到硬件查找前，`final-pipeline-tests.log` **12 passed**，含换装退出、继承配方续跑和离线重算。
- 独立 SHM 原生构建、安装通过；物理与 TriCal 测试独立复测见 `native-recheck.log`。最终增益上限与机器人配置一致，重建安装见 `native-final-install.log`。

完整物理 Mock：`mock-full/dvt2/20261116_184633`，左右共 48 点，约 14 min，最大表读数极差 **2.183 µm**。留出 RMSE：名义左 0.2263 mm / 右 0.1701 mm；零偏层约左 0.0002 mm / 右 0.0001 mm。几何层使左臂略变差，交付选择零偏层。`--reanalyse` 重算后的参数包与原包完全一致。

最终双步入球路径的 small 物理 Mock：`mock-small/dvt2/20261116_190412`，8 点通过，只有名义残差报告；离线重算通过。

## ARM64 交付

ARM64 包包含新执行器和随包 libomp；运行资产清单增加接触种子、工装几何资料及 USB 协议工具，依赖增加 pyserial 3.5。运行时压缩包为 **1,169,075 bytes**，wheel 为 **1,107,989 bytes**。新增约 284 KB 的 TriCal 执行器后，将运行时压缩包预算由 1 MiB 调整为 **1.5 MiB**，继续排除测试、头文件、静态库和 CMake 开发文件。

热缓存构建及 QEMU 冒烟共 **28.041 s**，其中 QEMU 23.073 s，见 `dist/orin-aarch64/manifest.json` 和 `cross-build-final.log`。

离线首次安装 **173.504 s**、更新安装 **19.920 s**，随后各自 `setup.sh --check` 通过；H1 路线规划与随包 libomp 加载通过。安装后的 pyserial、USB 协议辅助模块、新执行器及 K1/K2 smoke 路线规划通过。记录：`artifacts/orin-install/20261116T113400Z/`，含 `installed-trical.log` 和 `kinematics-plan.log`。

ARM64 完整非实时回归 **254 passed，1 warning**（H3 合成辨识的 CVXPY 精度警告），pytest 1110.81 s；包含数值冒烟总计 1119.529 s。1/4 线程及两种导入顺序的 RNEA、碰撞结果最大差均为 0，包内 OpenMP 加载通过。新增 TriCal 参数回归、几何导出、测量协议、路线和会话测试均通过；实时物理及墙钟门限仍在原生环境验证。证据：`artifacts/orin-qemu/20261116T113408.941404Z/`。

## 现场后续

板卡到货后测量三表应答节拍、缓存更新延迟，填写串口/槽位/符号、清零值及工装质量/质心；部署时将已有 `fieldwork/zero.yaml` 一次性改名为 `fieldwork/gui_pose.yaml`。本轮执行环境为私有 SHM、仪表 Mock 和 ARM64/QEMU。
