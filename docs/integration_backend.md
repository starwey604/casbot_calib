# 统一积分后端：Boost.Odeint

当前统一采用 **Boost.Odeint RK4**，依赖锁定的Boost为1.88.0。单惯量、固定双惯量失配、驱动电流/滤波响应和单弦接触都通过同一个 `FixedStepIntegrator<State>` 推进；原手写四阶段公式已移除。实现见 [integration.hpp](../cpp/include/casbot_calib/integration.hpp)。

## 职责与时间语义

- `SimulationRunner` 决定整数纳秒的下一事件：指令送达、传感器采样、最长积分步及实验终点。Boost仅推进调用者给定的一个区间，不创建时钟、线程或事件队列。
- `integration_ns` 是最大物理步长，默认50µs；接触实验使用10µs。实际步长会因事件截断而缩短。先计算整数 `end_ns-begin_ns`，再转换成秒，不通过浮点累加推进调度时钟。
- 同一区间四个RK阶段使用同一个已送达的 `JointCommand`；混合PD继续根据各阶段状态计算，这是原有控制模型的语义。指令保持与恒定力矩不是同一概念。
- 区间终点若有新指令，在旧指令下完成此前积分，再应用新指令和采样；不把新指令提前用于上一步的最后一个RK阶段。
- 电流时间常数为零时仍走原代数响应逻辑；传感器延迟/噪声/量化、丢包/重复以及接触状态均由既有设备模型处理。
- 积分后端不使用自适应步长、密集输出插值或FSAL缓存，不改变控制周期和反馈采样周期；电流通道历史的既有线性插值仍由观测层处理。接触切换的数值误差继续通过步长减半实验检查；没有新增连续事件求根。
- MuJoCo仍只显示几何，不参与积分或力学计算。

## C++复用

`calib_integration` 是只依赖Eigen和Boost头文件的CMake接口目标，独立积分测试不链接Pinocchio或Python。`calib_core`通过它使用统一后端。环境直接声明 `libboost-headers>=1.88,<1.89`，不再依赖Pinocchio间接提供头文件。

```cpp
#include "casbot_calib/integration.hpp"

Eigen::VectorXd state(2);
state << 1., 0.;
casbot::FixedStepIntegrator<Eigen::VectorXd> integrator(state, 50000);
auto rhs = [](const Eigen::VectorXd& x, Eigen::VectorXd& rate, double time_s) {
  (void)time_s;
  rate[0] = x[1];
  rate[1] = -4. * x[0];
};
integrator.step(rhs, state, 0, 37001);  // 一个被事件截断到37001ns的区间
```

支持固定长度或动态长度的double精度Eigen列向量，使用Boost官方Eigen适配器。构造时分配RK临时向量，单个工作区维度固定；空状态、非正步长、倒序/过长区间或维度变化会被拒绝。RHS遵循Odeint签名，并填满同维度导数向量。

每个运行实例持有独立积分器；调用者不得并发操作同一个实例。后续固定基座串联臂可用 `[q,dq,驱动状态...]` 作为状态，RHS调用独立的 `ArmWorkspace::forward()`。C++测试已用耦合二连杆验证这一组合的能量和步长收敛；R2七轴设备/采集仍按原计划实施。浮动基座/四元数状态需要另外定义流形积分契约，不直接套用这个欧氏向量接口。

## 记录与复现

`SimulationRunner.integration_backend()` 返回实际编译版本，例如 `boost.odeint.rk4/1.88.0`。Python `simulate()` 的第三个返回值以及实验 `metadata.json` 的 `runtime` 会记录：

```json
{
  "integration_backend": "boost.odeint.rk4/1.88.0",
  "integration_step_ns": 50000,
  "integration_schedule": "int64_ns_event_truncated"
}
```

旧NPZ观测列、时间单位和物理模型版本保持兼容；RK4表达式的浮点运算次序可能产生舍入差异，按声明容差复核，不要求逐位相同。

本轮只把已存在且满足约束的Boost头文件提升为直接依赖。使用conda-lock解析/哈希/写出接口更新两套锁文件，验证去掉该新增声明后能复现旧输入摘要，并验证所有包、URL和哈希完全不变，没有重解或升级环境。审计记录见 [锁文件检查](../artifacts/boost_lock_refresh/verification.json)。

## 验证

- CTest `boost_integration`：独立指数/振子解析解、RK4四阶收敛、非自治系统阶段时间、非等长区间及非法输入。
- CTest `arm_rigid_dynamics`：同一后端推进Pinocchio耦合动力学，检查能量与步长减半。
- pytest：非网格指令/采样时刻对应的分段解析运动、指令与采样同刻的因果顺序、双惯量矩阵指数解，以及原有机械/接触收敛和能量检查。
- 原A–F与几何渲染仍执行。详见 [验证记录](validation.md) 和 [冻结数据比较](../artifacts/boost_integration_comparison/report.json)。Windows由既有CI运行相同测试，本机只验证Linux。

参考当前版本的官方实现：[RK4](https://github.com/boostorg/odeint/blob/boost-1.88.0/include/boost/numeric/odeint/stepper/runge_kutta4.hpp)、[Eigen适配](https://github.com/boostorg/odeint/tree/boost-1.88.0/include/boost/numeric/odeint/external/eigen)、[单步接口说明](https://www.boost.org/doc/libs/1_88_0/libs/numeric/odeint/doc/html/boost_numeric_odeint/odeint_in_detail/steppers.html)。
