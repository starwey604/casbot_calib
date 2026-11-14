# casbot_calib 协作约定

面向用户的说明、报告默认用中文，代码标识符用英文。开始改动前看`git status`，保留用户已有改动。

## 项目是什么

CASBOT双臂（14轴）运动学/动力学标定。唯一硬件路径是Orin本机EtherCAT中间件共享内存（SHM，`/shm_mc`）：C++`shm_executor`跑1kHz执行与完整记录，Python负责规划、碰撞检查、辨识和编排。旧ROS/SDK实现及离线研究代码已从main删除，需要时去`ros`分支查。

目标交付形态：工厂按固定流程运行一个脚本完成标定（`dynamics.sh`全流程，`dynamics_h1.sh`等分阶段）。动力学阶段：**H1静态重力**（含准备位与链路冒烟）→**H2摩擦**→**H3惯量/激励**。运动学（TriCal）暂未在SHM上实现。

## 已实测确认的设备事实

- 双臂SHM ID：12–18左臂、19–25右臂（直接用SHM ID，不用CAN ID或1–14别名）。GUI已实机确认ID、方向、零点与渲染一致。
- 主机直接转发position/velocity/kp/kd/effort五字段到模组，模组用本地反馈算PD；effort按Nm前馈叠加，kp=kd=0生效；主机不自动加重力/摩擦补偿。
- 状态字按`Statusword & 0x006f`解码：0x21就绪、0x27使能。
- 准备位：ID13左肩roll +20°、ID20右肩roll −20°，其余0°（自然下垂贴腿时用）。只是目标角，不改编码器零点。
- 起点停稳速度默认0.2 rad/s（Python`field_defaults.py`、C++`field_defaults.hpp`）。
- 实机零位在Orin`fieldwork/zero.yaml`（不入Git）。
- 未核实：输出力矩尺度（C08未做）；反馈effort是驱动估算值，无独立扭矩传感器。

## 现场流程

- 接管：停`zero_calibration.service`/`arm_debug_tool.service` → SIGTERM旧主站（15秒不退即失败，不SIGKILL）→ 厂家`launcher.bash stop` → 只启EtherCAT主站。由`shm_takeover.py`完成。接管窗口持续到Orin重启，窗口内假设无其它控制程序插队。
- 操作员在Orin上执行（`/home/nvidia/casbot_calib_shm`，`nvidia@172.16.0.10`，本机172.16.0.20/24）。GUI在本机运行，经SSH控制Orin；GUI与CLI不能同时占用SHM。
- 运动前的人工确认（支撑、周围空旷、继电器在手）写在操作文档里，**不在代码里加确认步骤**。
- 实机运动只在用户要求时执行，由用户观察并反馈。开发与测试只连私有端点`/casbot_calib_test_*`，不连真实机器人。

## 代码里必须保留的安全逻辑

这些放在内部自动执行，不要求操作员声明：
- 正式参考发送前做连续碰撞、URDF限位、参考速度检查；准备位前缀只查URDF/速度。
- 起点静止与无驱动故障检查；C++侧起点重读与限位复核；单owner。
- SIGINT/TERM/HUP → 失能并等待应答。
- 驱动/通信/模式故障 → 失能并结束整个会话，不清故障、不自动恢复。

## 写代码与文档的原则

- **不要防御性设计**。不加默认dry-run、必填输出目录、封存/摘要绑定、重复声明代码默认值、`不代表…/未证明…`类免责字段。默认值放代码里，输出目录自动生成，步骤之间的文件自动衔接。
- 参数只暴露真正需要操作员选择的；开发/Mock用的选项放开发入口。
- 文档写“怎么跑、结果怎么看”，简短直接。现场记录放`docs/records/`，不往本文件追加日志。
- 本文件只放长期约定，不记流水账；历史看`git log`。

## 构建与测试

```bash
# 本机：Humble容器只提供Pinocchio/Coal工具链。经docker exec运行时需重定向stdin。
bash scripts/humble/container.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q < /dev/null
bash scripts/humble/container.sh bash -c 'cmake --build --preset humble && ctest --preset humble' < /dev/null
# Orin（完整数值环境 + C++ 执行器安装至 install/shm-core + 预检）：
bash calib/setup.sh
# 仅构建 C++ SHM 执行器与物理 Mock（默认 build/shm-core -> install/shm-core）：
bash scripts/shm/build.sh --with-mock
```

改了`cpp/`核心后要重建Python绑定再跑Python测试。x86_64产物不能复制到Orin，Orin用同一份源码原生编译（`scripts/shm/deploy_git.sh`同步）。

## Git

- Python四空格；C++沿用周边格式。SI单位、整数纳秒。
- 提交聚焦、标题简短行动式，用本机Git身份。不push，除非用户要求。
- 不提交`artifacts/`、`build/`、`install/`；大型源资产用Git LFS。
