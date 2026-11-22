# 构建、测试与 Orin 部署

本机开发与验证在 Linux 容器 `ros2-humble` 中运行，容器与宿主机共享同一个仓库目录。

## 本机环境准备

在宿主机仓库根目录拉取 Git LFS 模型资产并初始化 Humble 环境：

```bash
git lfs pull
bash scripts/humble/setup.sh
bash scripts/humble/setup.sh --doctor
```

`setup.sh` 依次完成 `preflight`、`system`、`python`、`build`、`verify` 五步，状态记录在 `build/humble-setup/`。可用 `--only build` 仅重跑构建，或用 `--force system` 重新执行系统依赖及后续阶段。

## 本机构建与测试

完整交付前校验（含 CTest、全量 pytest、C++ 安装消费和 Python wheel 检查，结果写入 `artifacts/validation_*/`）：

```bash
bash scripts/humble/container.sh bash scripts/humble/validate.sh < /dev/null
```

日常开发中修改了 `cpp/` 代码后，需重新安装 Python 扩展并重编原生执行器，让 pytest 使用当次构建产物（经 `docker exec` 运行时需加 `< /dev/null`）：

```bash
bash scripts/humble/container.sh bash -c '
  cmake --preset humble -DCALIB_BUILD_PYTHON=ON -DCALIB_BUILD_SHM=ON -DCALIB_BUILD_SHM_MOCK=ON &&
  python -m pip install -e . --no-deps --no-build-isolation \
    --config-settings=build-dir=build/humble --config-settings=cmake.define.CALIB_BUILD_PYTHON=ON &&
  cmake --build --preset humble &&
  ctest --preset humble --output-on-failure &&
  CASBOT_SHM_TEST_BIN="$PWD/build/humble/cpp/src/shm" CASBOT_SHM_PYTHON="$VIRTUAL_ENV/bin/python" \
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
' < /dev/null
```

日常快速集默认跳过慢测试；跑完整 pytest 回归时附加 `-o addopts=''`。仅构建并验收 C++ SHM 执行器与物理 Mock（安装到 `install/shm-core`）：

```bash
bash scripts/shm/build.sh --with-mock
.venv-shm/bin/python scripts/shm/verify_physics.py --bin-dir install/shm-core/bin
```

## 部署到 Orin

若使用网线直连（本机 `172.16.0.20/24`，Orin `172.16.0.10`），可先用 `bash scripts/jetson_network.sh up <网卡名>` 配通网卡。

### 方式一：本机交叉编译一键部署（推荐）

先在本机生成 ARM64 预编译包，再通过 SSH loopback 隧道同步源码、LFS 模型、离线 wheel 与预编译产物到 Orin，并在远端完成安装与预检：

```bash
bash scripts/shm/deploy_orin.sh --host nvidia@172.16.0.10
```

> 若修改过 [Dockerfile.orin](../scripts/cross/Dockerfile.orin)，先运行 `bash scripts/cross/ensure_images.sh --rebuild` 重建交叉镜像，再执行交叉构建或部署。Orin 运行预编译产物需系统已安装 `libzstd1`。

也可在本机提前跑 ARM64 QEMU 离线与数值验证：

```bash
bash scripts/shm/verify_orin_qemu.sh --smoke
bash scripts/shm/verify_orin_qemu.sh --pytest
```

### 方式二：同步源码并在 Orin 原生编译

仅将已提交的本地分支同步到 Orin（默认 `/home/nvidia/casbot_calib_shm`；本地另有未提交工作时加 `--committed-only`）：

```bash
bash scripts/shm/deploy_git.sh --host nvidia@172.16.0.10
```

Orin 源码编译需安装系统包 `libzstd-dev`。登录 Orin 在项目目录执行初始化与构建，随时可用 `--check` 复检：

```bash
bash calib/setup.sh
bash calib/setup.sh --check
```

若使用 `--with-prebuilt` 手动同步的预编译包，则在 Orin 运行 `bash calib/setup.sh --prebuilt`。

## 模型资产

默认模型 `bare_flange_1105_03_gauge_v2` 采用固定腰部、14 轴裸法兰与 `1105_03` 三表碰撞资产。Python 通过 `casbot_calib.model_context.load_context()` 统一读取。手动安装独立 wheel 时，需将 `CASBOT_PROJECT_ROOT` 指向已完成 `git lfs pull` 的仓库根目录。源模型与派生参数细节见[资产说明](../python/casbot_calib/assets/models/casbot_hl12p1_dvt2/README.md)。
