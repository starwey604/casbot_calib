# Orin 交叉构建 Step 4（2026-11-15）

## 交付入口

- `scripts/shm/cross_build_orin.sh`：默认 ThinLTO，支持 `--thinlto off`、独立 `--build-root` / `--output-dir` 对照。
- `scripts/shm/deploy_git.sh --with-prebuilt`：通过已有 SSH 复用连接同步 wheel、tar 与 manifest 至 `artifacts/prebuilt_aarch64`。
- `calib/setup.sh --prebuilt [DIR]`：ARM64 Python 3.10 预编译安装；默认目录为 `artifacts/prebuilt_aarch64`，安装后执行 ABI 与离线 preflight。
- `scripts/cross/verify_install.py`：在断网 ARM64/QEMU 容器内创建独立项目，验证首次/更新安装、两次 `--check`、实际 wheel 路径、H1 离线规划及 4 线程 OpenMP。

安装采用标准 pip，复用已有数值依赖，以 `--no-compile` 避免 QEMU 逐文件编译字节码。
预编译 venv 记录项目根路径，CLI 入口选择已安装的 wheel；配置和输出仍属于工作树。
原生安装完成后清除预编译入口标记，恢复原来的源码优先导入。

部署测试在本地临时 Git 仓库中使用 SSH 传输替身，覆盖有空格的目标路径、模型资产、
ARM64 wheelhouse 和预编译包传输。修复了旧资产/wheelhouse 分支中 heredoc 与 tar 管道
争用 SSH stdin 的问题：远端 shell 脚本通过命令参数传递，stdin 完整保留给 tar。

## ThinLTO 编译耗时与体积

继续默认启用 ThinLTO。LLVM/LLD 22.1.8、Jammy GCC 11 sysroot、`-O3`、
`-mcpu=cortex-a78ae`、OpenMP 和 Python 配置相同，只改变 `CALIB_ORIN_THINLTO`。

使用两个全新 build-root，分别保存 ccache 与 ThinLTO 缓存；jobs=2，O3 冷/热、
ThinLTO 冷/热依次执行，无其它构建或测试并发。热轮使用 `--rebuild --no-verify`，
清理目标文件后全量重编。总耗时包含容器启动与元数据记录，QEMU 单独计时。

| 阶段（秒） | O3 冷 | O3 热 | ThinLTO 冷 | ThinLTO 热 |
| --- | ---: | ---: | ---: | ---: |
| CMake 配置 | 2.445 | 0.986 | 1.643 | 1.001 |
| C++ 构建 | 47.404 | 0.841 | 58.696 | 0.997 |
| C++ 安装 | 0.073 | 0.070 | 0.055 | 0.076 |
| wheel 构建/打包 | 19.274 | 3.210 | 24.949 | 3.068 |
| C++ 打包 | 0.367 | 0.321 | 0.286 | 0.269 |
| 总计 | 70.722 | 6.404 | 86.441 | 6.286 |

ThinLTO 冷构建增加 15.719 秒（22.2%），热缓存耗时接近。

```bash
bash scripts/shm/cross_build_orin.sh --thinlto off --build-root build/orin-step4-final-o3 --output-dir artifacts/orin_cross_step4/final-o3 --no-verify
bash scripts/shm/cross_build_orin.sh --thinlto off --build-root build/orin-step4-final-o3 --output-dir artifacts/orin_cross_step4/final-o3 --rebuild --no-verify
bash scripts/shm/cross_build_orin.sh --thinlto on --build-root build/orin-step4-final-thin --output-dir artifacts/orin_cross_step4/final-thin --no-verify
bash scripts/shm/cross_build_orin.sh --thinlto on --build-root build/orin-step4-final-thin --output-dir artifacts/orin_cross_step4/final-thin --rebuild --no-verify
```

| 产物（字节） | O3 | O3 + ThinLTO |
| --- | ---: | ---: |
| `_core` | 1,610,072 | 1,213,472 |
| `shm_executor` | 993,472 | 376,976 |
| 每份 `libomp.so.5` | 1,199,248 | 1,199,248 |

ThinLTO 将扩展减小约 24.6%，执行器减小约 62.1%。O3 与 ThinLTO 均通过 ARM64/QEMU
7 个 C++ 数值/协议测试、ABI、私有 libomp、4 组线程/导入顺序对照与离线 preflight。
RNEA 参考最大误差 4.44e-15，碰撞参考及单/多线程差异均为 0。

## 打包与冒烟优化

交叉构建先让 scikit-build-core 生成代码/元数据 wheel，再由标准 WheelFile 添加模型资产，
生成最终 RECORD。资产采用 ZIP stored；未改变原生 wheel 设置。
完整比对新旧 wheel 的资产文件清单与大小：860 个文件，共 303,545,095 字节。
最终 wheel 为 305,134,254 字节，旧压缩 wheel 为 94,957,401 字节。

本轮优化后热缓存全量重编与打包 5.712 秒，其中 wheel 3.061 秒；44 次编译调用全部
直接命中 ccache（hits 20→64、misses 保持 74）。原方式热全量打包 16.637 秒，wheel 13.651 秒。

QEMU 四组 fresh-process 对照并行执行，与父进程的数值依赖检查及 C++ 测试重叠。
每轮计算一次完整独立 Python Coal 参考，父进程比较四组结果；Python Pinocchio RNEA 仍各自计算。模型、32 行数据、502 个碰撞对、线程 TID、
加载路径、两种导入顺序和数值阈值保持一致。离线 preflight 在同一数值进程调用原 CLI
函数，数学绑定链接也在该进程完成。热环境直接读取目标 dist-info 检查依赖版本。

最终运行冒烟 **13.288 秒**，Step 3 为 36.613 秒，缩短 63.7%；含 wheel 安装与 ELF
审计的完整 QEMU 验证为 **17.539 秒**。最终无源码变更的一键构建、打包与验证共
**22.541 秒**，manifest 的 `verification` 为 `passed`。
数值结果：RNEA 对 Pinocchio 最大误差 4.44e-15；四组碰撞对 Coal 参考、批处理对逐行、
单/多线程及导入顺序差异均为 0。1/4 线程实际 TID 数符合要求，两份 libomp 从各自包内加载。

## 安装证据

首次/更新均在 `--network none` 的 ARM64 runtime 中实际运行 `calib/setup.sh --prebuilt`。
项目仅复制源码、脚本与配置；模型和 `_core` 来自已安装 wheel。更新前放入的操作员配置
保留，安装后 Python 路径在 venv 内，H1 `--plan-only` 通过，C++ 探针执行 4 个线程。

完整安装结果：`artifacts/orin-install/20261115T121251Z/`，首次 74.163 秒、更新
24.068 秒；当时同时运行本机原生构建。`first-check`、`update-check`、`installed-path`、
`h1-plan`、`openmp` 均通过。更新轮包含全部 pip 替换、C++ 解包和离线 preflight。

随后在同一独立项目中，无其它构建并发，使用显式 `--prebuilt DIR` 更新安装，
耗时 **20.002 秒**，ABI 与离线 preflight 通过。
通过 `tests/installation` 的 CMake 消费项目链接该项目实际安装的 Core，ARM64/QEMU
运行成功；清空 `LD_LIBRARY_PATH` / `PYTHONPATH` 后仍能加载安装树运行库。

## 原生回归

本工作树原来的 `.venv-humble` 是主工作树 venv 的链接。为使子进程也加载本工作树代码，
将原链接保留在 `build/humble-shared-links-step4`，建立独立 `.venv-humble`，复用主工作树
site-packages 中的依赖；以 `--ignore-installed` 安装本工作树 editable 包与重建后的绑定。
主工作树源码与环境保持原样。

```bash
bash scripts/humble/container.sh bash scripts/shm/build.sh --with-mock --jobs 2 < /dev/null
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OMP_NUM_THREADS=4 \
  CASBOT_SHM_TEST_BIN="$PWD/build/shm-core/cpp/src/shm" \
  CASBOT_SHM_PYTHON="$PWD/.venv-humble/bin/python" \
  python -m pytest -q -ra --durations=10 --junitxml=artifacts/orin_cross_step4/native-final.xml < /dev/null
```

- C++ 原生构建、安装与 CTest：14/14 通过。
- 完整原生 pytest：326 passed、0 failed、0 skipped，349.19 秒，3 条 CVXPY 警告。
- 新增宿主测试覆盖部署三类 tar 流、带空格路径、Python 入口选择、wheel RECORD/资产、
  ARM64 依赖元数据检查；与项目路径测试合计 30 项通过。
- Step 3 的 ARM64 非实时 pytest 241 项结果继续适用；本轮优化后的完整数值冒烟重新通过。

## 耗时目标与证据位置

热缓存全量重编打包已达到 < 10 秒目标。QEMU 运行冒烟 13.288 秒仍高于 8 秒目标，
更新安装 20.002 秒仍高于 2～3 秒目标；单文件增量 < 3 秒尚未单独测量。
本轮未连接实机。

主要证据：

- `artifacts/orin_cross_step4/final-{o3,thin}-{cold,hot}.json`：顺序冷/热对照。
- `artifacts/orin_cross_step4/compiler-commands.log`、`sizes.json`：实际编译选项与 ELF 体积。
- `artifacts/orin_cross_step4/final-parallel-package.log`：最终一键构建与优化后冒烟。
- `artifacts/orin-qemu/20261115T123834.560011Z/`：四组数值结果、加载路径、最终 smoke JSON。
- `artifacts/orin-install/20261115T121251Z/`：首次/更新安装及入口验收日志。
- `artifacts/orin_cross_step4/update-isolated.{log,json}`：无其它构建并发的更新安装。
- `artifacts/orin_cross_step4/consumer.log`：实际安装 Core 消费。
- `artifacts/orin_cross_step4/native-{shm,final}.log`、`native-final.xml`：原生回归。
- `dist/orin-aarch64/manifest.json`：最终 ThinLTO 产物、工具链与验证状态。
