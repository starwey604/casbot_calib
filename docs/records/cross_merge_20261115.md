# Orin 交叉构建合入 main：阶段 A 验收

## 合并结果

以 main `c7389b3` 合入 `orin-cross-build` 的 `0811afc`，保留两侧提交历史。

- H1 保留全臂 `approach_joints`、8° 接近量、观测方向/位移权重、库仑正则解耦和镜像增益；将端点筛选、配对轨迹及回归迁入 `h1_selection.py` 和 `test_wave2a_h1.py`。
- `deploy_orin.sh` 固定为交叉构建、同步、预编译安装；删除 `--jobs` 与过渡入口 `run_orin_tests.sh`。`--check-only` 跳过构建及预编译包同步，检查已有安装；`--no-setup` 只构建并同步。
- `setup.sh --prebuilt` 清理旧 `install/shm-core`、`build/shm-python`、`build/shm-core`。无参数原生安装保留按内存选择 1/2/4 个编译任务。
- 模型上下文测试移出显示网格断言；独立安装项目只复制 8 个无头运行资产，验证模型、碰撞及运动学加载。
- K1–K3 蓝图的原生终验改用 `validate.sh` 重装绑定；后续实施遵循合并计划阶段 B 的工具、路径、资产与测试分层约束。

## 原生验收

日志位于 `artifacts/cross-merge/`：

| 验收 | 结果 |
|---|---|
| `validate.sh` 重装 editable 绑定与 CTest | 5/5，16.60 s |
| 全量 pytest（包含 slow） | 首次 317 通过、2 失败，279.21 s；修正测试配方后两项定向复测通过，39.41 s |
| 原生 C++ consumer、非 editable wheel | 通过；wheel 707,350 B |
| `scripts/shm/build.sh --with-mock` | Humble 容器内构建、安装成功；CTest 5/5，16.38 s |
| 交叉编排与部署替身测试 | 32/32，2.28 s |

两项失败均来自瘦身测试配方与主线有效接近位移门槛的冲突：

- 合成二进制采集测试：接近量 0.2° 小于生产门槛 0.3°，拟合权重全零；改为 0.5°。
- 八窗口物理 Mock：目标 0.1°、接近 0.03°，同样无法形成有效权重；改为目标 1.0°、接近 0.75°。
- 两项均增加左右肩 pitch 各有一对有效验证窗口的断言。生产逻辑与门槛保持原值。

`native.log` 保存完整首轮结果，`h1-repair.log` 保存两项复测；随后按 `validate.sh` 的剩余步骤完成环境记录、consumer 与 wheel 验证，日志为 `native-resume.log`。原生独立构建日志为 `native-shm.log`。

## ARM64/QEMU 与断网安装

交叉构建与首次 QEMU 环境准备共 165.149 s：C++ 56.751 s，wheel 22.289 s，验证 82.027 s；数学/C++ 冒烟主体 13.232 s。本轮在 main 的新构建目录执行并新建 QEMU venv。

- ARM64 wheel：1,082,920 B；C++ 运行包：1,046,137 B。
- QEMU 非实时 pytest：212/212，60.49 s；验证入口总计 67.184 s。
- 1/4 线程、两种导入顺序：RNEA 对 Pinocchio 最大误差 `4.44e-15`；碰撞对 Coal、单/多线程结果差均为 0。
- 实际加载 wheel 内 `casbot_calib/.libs/libomp.so.5`；C++ probe 加载随 C++ 包安装的私有运行库，观察到 1/4 个工作线程。

QEMU 报告：
- `artifacts/orin-qemu/20261115T154604.977532Z/`：打包冒烟。
- `artifacts/orin-qemu/20261115T154621.467365Z/`：完整非实时回归。

断网安装报告：`artifacts/orin-install/20261115T154728Z/`。

| 操作 | 耗时 |
|---|---:|
| 首次安装 | 62.524 s |
| 首次完整 preflight | 8.832 s |
| 更新安装 | 8.770 s |
| 更新完整 preflight | 7.634 s |
| 已安装路径与无头模型检查 | 2.802 s |
| H1 plan-only | 12.055 s |
| OpenMP probe | 0.557 s |

更新前植入旧原生构建目录、旧测试二进制/头文件/静态库；更新后全部清除，操作员配置保留。模型检查禁用 Python Pinocchio/Coal/MuJoCo 导入，使用安装的 `_core` 与项目的无头资产完成 14 轴运动学和碰撞检查。

## 产物与工作树收拢

- 合并产物：`dist/orin-aarch64/`；安装与测试目录均在 main 工作区的 `build/`。
- 原 cross 工作树的 `artifacts/` 和 `dist/` 已归档到 main 的 `artifacts/cross-worktree/`；下载轮子与 ccache 已复制到 main，供后续构建复用。
- 全程使用本机容器、QEMU 与私有 Mock；没有连接 Orin 或 push。
- Git 已注销 cross 工作树并删除 `orin-cross-build` 分支。旧目录删除遇到历史容器生成的 `root:root` 文件，报 `Permission denied`；剩余目录需由用户以管理员权限清理，归档已保存在上述位置。
