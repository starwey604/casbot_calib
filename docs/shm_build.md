# SHM 原生构建与 Orin 源码部署

所有构建产物统一安装到仓库内的 `install/shm-core`（跨架构必须在目标机原生编译，不能把 x86_64 二进制复制到 Orin）。

## 1. 通过 SSH 从本机同步到 Orin

直连网络配置（本机 `172.16.0.20/24`，Orin `172.16.0.10`）：

```bash
bash scripts/jetson_network.sh up <本机有线网卡名>
```

在本机仓库根目录将当前已提交分支同步到 Orin（默认 `/home/nvidia/casbot_calib_shm`）：

```bash
bash scripts/shm/deploy_git.sh --host nvidia@172.16.0.10
```

- 通过 SSH 反向隧道与本机临时只读 `git daemon` 克隆或快进更新，不经过外网或 GitHub。
- 若远端默认 DVT2 标定模型包（`collision.npz` 与 `visual_meshes/*.stl`）仍为 LFS 指针且远端已装 `git-lfs`，脚本会自动传输并物化这组模型资产，同时保持远端 `git status` 干净（仅传源码可加 `--no-assets`）。

## 2. Orin 一键环境初始化与编译（`calib/setup.sh`）

登录 Orin 后在 `/home/nvidia/casbot_calib_shm` 执行：

```bash
cd /home/nvidia/casbot_calib_shm
bash calib/setup.sh
```

`calib/setup.sh` 自动完成：
1. 创建/复用 `.venv-shm` 并安装数值依赖（离线时自动探测本地 `artifacts/*wheels*`，也可显式传 `--wheelhouse DIR`；依赖已齐备时不访问外网）。
2. 将 `/opt/ros/humble` 中的 `pinocchio`、`eigenpy`、`coal` 链接进 `.venv-shm`（不 `source` ROS，不污染 `LD_LIBRARY_PATH`）。
3. 编译 Python `_core` 扩展、动力学/碰撞核心、`shm_executor`、`shm_channel_start`、`shm_mock_server`、`shm_gui_owner`，运行 CTest 并安装到 `install/shm-core`。
4. 执行模型 LFS 资产与二进制完整性预检。

随时单独运行预检（不重编）：

```bash
bash calib/setup.sh --check
```

## 3. 单独构建 C++ SHM 组件（`scripts/shm/build.sh`）

仅需构建 C++ 执行器与物理 Mock（不建 Python venv）时：

```bash
# 默认使用 build/shm-core -> install/shm-core，存在 /opt/ros/humble 时自动选用
bash scripts/shm/build.sh --with-mock

# 安装后双进程物理闭环验收（私有 SHM 端点，不连接 /shm_mc）
python3 scripts/shm/verify_physics.py --bin-dir install/shm-core/bin
```

只构建纯 SHM 协议组件（不含 Pinocchio/Coal 核心）可不带 `--with-core` / `--with-mock` 运行 `bash scripts/shm/build.sh`。

## 4. 本机开发与测试（Humble 容器）

```bash
# C++ 构建与测试
bash scripts/humble/container.sh bash -c 'cmake --build --preset humble && ctest --preset humble --output-on-failure' < /dev/null

# Python 测试套件
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q < /dev/null

# 准备本机 GUI 原生程序（安装至 install/shm-core）
bash scripts/shm/setup_gui.sh
```

历次部署与实机编译记录归档于 [shm_orin_build_20261114.md](records/shm_orin_build_20261114.md)。
