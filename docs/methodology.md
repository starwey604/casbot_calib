# 标定原理与算法参考

本文档说明 `casbot_calib` 当前代码中实际使用的动力学辨识、TriCal 运动学辨识与闭环验收数学模型、数据口径及对应参考文献；命令行操作与现场流程见 [动力学手册](dynamics.md) 与 [运动学手册](kinematics.md)。

---

## 1. 动力学总体模型与参数化

14 轴双臂关节顺序固定为左臂 SHM ID 12–18、右臂 SHM ID 19–25。逆动力学按“刚体惯量 + 转子反射惯量 + 关节摩擦”线性分离：

$$\tau = \Phi(q, \dot{q}, \ddot{q})\,\theta + J_m\,\ddot{q} + \tau_f(\dot{q})$$

- **刚体回归与参数顺序**（[rigid_identification.py](../python/casbot_calib/rigid_identification.py)）：由 Pinocchio 计算整臂关节力矩回归矩阵 $\Phi(q,\dot{q},\ddot{q}) \in \mathbb{R}^{14 \times 140}$。参数向量 $\theta \in \mathbb{R}^{140}$ 按每个连杆 $j=1,\dots,14$ 在各自连杆原点坐标系下的 10 维惯性参数排列：
  $$\theta_j = \bigl[m,\; m c_x,\; m c_y,\; m c_z,\; I_{xx},\; I_{xy},\; I_{yy},\; I_{xz},\; I_{yz},\; I_{zz}\bigr]^T$$
  质心惯量与原点惯量之间按平行轴定理互相换算：
  $$I_c = I_o - m\bigl((c^T c)I_3 - c c^T\bigr)$$
- **物理一致性 PSD 锥约束**（*Wensing et al., 2018*）：为避免最小二乘回归产生负质量或非物理惯量张量，每个连杆需满足 $4\times 4$ 伪惯性矩阵正定：
  $$J(\theta_j) = \begin{bmatrix} \frac{1}{2}\operatorname{tr}(I_o)I_3 - I_o & h \\ h^T & m \end{bmatrix} \succ 0, \qquad h = m c$$
  求解器在 CVXPY + Clarabel 中通过长度同余缩放 $C=\operatorname{diag}(1/\ell, 1/\ell, 1/\ell, 1)$ 组装为半正定锥约束 `J >> 0`。
- **鲁棒损失与正则**：刚体锥规划目标统一采用带权 Huber 损失加先验正则：
  $$\min_{\theta,\,J_m}\; \frac{1}{2}\sum_k w_k\,\ell_\delta\!\left(\frac{r_k}{\sigma_\tau}\right) + \frac{\lambda}{2}\|D^{-1}(\theta - \theta_{\mathrm{prior}})\|_2^2 + \frac{10^{-6}}{2}\|P_{\mathrm{unobs}}\,D^{-1}(\theta - \theta_{\mathrm{prior}})\|_2^2$$
  其中 $D$ 为按各参数量纲与先验值构造的对角尺度矩阵，$P_{\mathrm{unobs}}$ 为回归矩阵 SVD 零空间投影；支持在留出集上扫描 `ridge_grid` 自动选 $\lambda$。

---

## 2. 动力学三阶段：分阶段解耦与冻结衔接

项目按 **H1 静态重力 → H2 摩擦 → H3 惯量** 顺序辨识，上游结果自动写入会话并逐级传递。

### 2.1 H1：静态重力辨识（[h1_gravity.py](../python/casbot_calib/stages/dynamics/h1_gravity.py)、[h1_selection.py](../python/casbot_calib/stages/dynamics/h1_selection.py)）

- **姿态筛选与 D-最优选择**：从单轴与双轴组合候选姿态池中剔除 URDF 限位外及静态碰撞姿态；计算静力学回归矩阵 $\Phi(q,0,0)$ 经尺度归一化后的单姿态信息块 $G_i = B_i^T B_i$，贪心最大化 $\log\det(M + G_i)$ 分别选出训练集与留出验证集。
- **正反配对逼近（Paired Approach）**：每个目标姿态 $q$ 均从负向 $(q - \Delta q)$ 与正向 $(q + \Delta q)$ 两次缓慢逼近并静置采样，抵消单向静摩擦造成的静态力矩滞回偏置；采样窗口按唯一观测时间戳的梯形积分权（`TimeMean`）计算稳态均值，避免同一毫秒重复上报样本权重失衡。
- **重力回归**：以 $\dot{q}=0,\;\ddot{q}=0$ 组装静力学方程 $\Phi(q,0,0)\theta = \tau_g$，在 14 个 $4\times 4$ PSD 约束下求得物理合法候选 URDF；此时与重力无关的惯量方向靠先验正则给出物理代表值，由 H3 进一步辨识。

### 2.2 H2：关节摩擦辨识（[h2_friction.py](../python/casbot_calib/stages/dynamics/h2_friction.py)）

- **去重力与正反扫掠配对**：在静态姿态附近按对数速度档做连续正、反向匀速往返扫掠，前馈扣除 H1 重力；对同位置区间的正程 $(v^+,\tau^+)$ 与反程 $(v^-,\tau^-)$ 插值配对：
  $$\tau_{f,\mathrm{pair}} = \frac{\tau^+ - \tau^-}{2}, \qquad \tau_{\mathrm{offset}} = \frac{\tau^+ + \tau^-}{2}$$
  差值消去残余重力偏置，均值 $\tau_{\mathrm{offset}}$ 作为零偏诊断保留；低速加/减速过渡区按速度分箱配对，补充 Stribeck 低速斜坡样本。
- **C++ 同构 Stribeck 模型**：Python 拟合与 C++ 运行时前馈使用完全同构的连续模型：
  $$\tau_f(v) = b\,v + \left(f_c + \Delta f_s\,e^{-(v/v_s)^2}\right)\tanh\!\left(\frac{v}{v_0}\right)$$
  其中 $b$ 为粘性系数（$\mathrm{Nm\cdot s/rad}$），$f_c$ 为库仑摩擦（$\mathrm{Nm}$），$f_s = f_c + \Delta f_s$ 为静摩擦（$\mathrm{Nm}$），$v_s$ 为 Stribeck 特征速度（$\mathrm{rad/s}$），$v_0$ 为过零平滑速度（辨识默认 $0.001\;\mathrm{rad/s}$）。拟合先用中高速匀速平台样本以 `soft_l1` 线性求得 $(b, f_c)$ 初值，再联合低速过渡样本用解析雅可比非线性最小二乘精修 $[b, f_c, \Delta f_s, v_s]$。

### 2.3 H3：周期激励与惯量 / 转子惯量联合辨识（[h3_inertia.py](../python/casbot_calib/stages/dynamics/h3_inertia.py)、[dual_excitation.py](../python/casbot_calib/dual_excitation.py)）

- **分层有限傅里叶激励**（*Swevers et al., 1997*）：对第 $j$ 轴设计周期 $T$ 的有限傅里叶多正弦轨迹：
  $$q_j(t) = q_{0,j} + \sum_{k} A_{j,k}\sin(h_{j,k}\,\omega_0 t + \phi_{j,k}), \qquad \omega_0 = \frac{2\pi}{T}$$
  近端大惯量轴与远端腕轴采用分层谐波（默认训练近端 `1,2,3,5`、远端 `2,4,6,9`，留出集错开谐波），并通过 CMA-ES 优化初相 $\phi_{j,k}$ 以改善回归矩阵条件数。$q,\dot{q},\ddot{q}$ 全程采用精确解析导数，不对编码器位置做数值二阶差分。
- **频带投影与去摩擦**：每个激励块连续运行多周期并丢弃首周期瞬态，将实测关节与力矩向激励谐波基做截断傅里叶投影，剔除激励频带外噪声；扣除 H2 辨识出的 Stribeck 摩擦分量 $\tau_{f,\mathrm{band}}$。
- **重力子空间冻结与 140+14 联合拟合**：从 H1 探测姿态集构造 $\Phi(q_g,0,0)$ 经 SVD 提取的等式约束（`ZeroConeT`），保证 H3 优化结果在静态重力子空间内的变化不超过 `1e-4 Nm`；在此前提下附加有界转子惯量 $J_m \in [0, 5.0]^{14}$，按各轴力矩 RMS 归一化后联合求解：
  $$\Phi_{\mathrm{band}}(q,\dot{q},\ddot{q})\,\theta + \operatorname{diag}(\ddot{q}_{\mathrm{band}})\,J_m \approx \tau_{\mathrm{band}} - \tau_{f,\mathrm{band}}$$

---

## 3. 闭环验收评估口径（[verify_dynamics.py](../python/casbot_calib/pipeline/verify_dynamics.py)）

在同一段电子琴 + 架子鼓及转场参考轨迹上依次连续运行 5 组闭环对照（`verify/report.md`）：

| 组别 | 关节 PD 比例 | 前馈动力学模型 | 用途 |
| --- | --- | --- | --- |
| `G0_stiff` | `1.0× Kp`, `1.0× Kd` | 无前馈 | 常规高刚度基准 |
| `G0_soft` | `0.5× Kp`, `1.0× Kd` | 无前馈 | 柔和刚度无补偿基线 |
| `G1_soft` | `0.5× Kp`, `1.0× Kd` | 名义 URDF | 未标定名义前馈基线 |
| `G2_soft` | `0.5× Kp`, `1.0× Kd` | H2 对照模型（有重力与摩擦、无惯量拟合） | 分离惯量辨识收益的消融对照 |
| `G3_soft` | `0.5× Kp`, `1.0× Kd` | H1–H3 完整标定交付 | 最终标定闭环性能验收 |

- **前馈平滑处理**：下发前馈时将静态摩擦偏置对齐到 `coulomb`，并按 `SMOOTHING_FLOOR = 0.10 rad/s` 夹紧 $v_0$ 下限以抑制过零突变；导出推荐配置写至 `verify/analysis/recommended_band_gains.yaml`。
- **判定条件**：要求 `G3_soft` 的全局法兰位置 RMSE 与关节 RMSE 均严格低于 `G0_soft` 与 `G1_soft`，且法兰位置 RMSE 不高于 `G0_stiff`；同时输出相位滞后（互相关搜至 $\pm 80\;\mathrm{ms}$）、高频速度残差 RMS 与 PD 纠偏力矩 RMS。

---

## 4. TriCal 运动学标定 K1–K3（[trical/solver.py](../python/casbot_calib/trical/solver.py)）

- **测量与几何参数化**（*TriCal, Sensors 2020*）：通过工装末端三轴千分表对四颗固定标准球测量位置残差 $p_{\mathrm{tcp}}(q_k) + R(q_k)d_{\mathrm{gauge}} - b_k = 0$。连杆间坐标系采用 MDH / Hayati 五参数乘积图（TriCal Eq. 1）：
  $$T_{i-1,i} = R_x(\alpha)\,T_x(a)\,R_y(\beta)\,R_z(\theta)\,T_z(d)$$
  对几乎平行的相邻关节轴，代码通过向下一轴投影构造平滑基准而不除公垂线距离，避免平行轴构型奇异。
- **两层递进拟合与自动回退**：
  - **Layer 1（19 维）**：14 维关节零偏（$q_{\mathrm{true}} = q_{\mathrm{encoder}} + \mathrm{offset}$）+ 5 维台架相对安装偏差 $(t_x, t_y, t_z, r_x, r_z)$。
  - **Layer 2**：在 Layer 1 基础上对余下几何参数雅可比投影，经列主元 QR 按条件数上限（`condition_limit = 200`）贪心选取可观列并做 `soft_l1` 信任域最小二乘。
  - **层级选择**：若 Layer 2 在任一臂留出球点上的位置 RMSE 劣于 Layer 1，则自动选用 Layer 1（只输出零偏文件 `joint_offsets.yaml` 而不改动名义连杆几何），防止弱可观几何过拟合。

---

## 5. 核心参考文献

| 文献 | 在本项目中的对应部分 |
| --- | --- |
| Wensing, P. M., Kim, S., & Slotine, J.-J. E. (2018). *Linear Matrix Inequalities for Physically Consistent Inertial Parameter Identification: A Statistical Perspective on the Mass Distribution*. IEEE RA-L. [arXiv:1701.04395](https://arxiv.org/abs/1701.04395) | H1 与 H3 的 $4\times 4$ 伪惯性矩阵半正定锥约束（`rigid_identification.py`） |
| Swevers, J., Ganseman, C., Tükel, D. B., De Schutter, J., & Van Brussel, H. (1997). *Optimal Robot Excitation and Identification*. IEEE Transactions on Robotics and Automation. [DOI:10.1109/70.631234](https://doi.org/10.1109/70.631234) | H3 分层有限傅里叶多正弦激励、重复周期收集与频域投影（`dual_excitation.py`、`h3_inertia.py`） |
| TriCal: *Low-Cost 3-Axis Dial Indicator Calibration for Collaborative Robots* (2020). Sensors, 20(20), 5919. [DOI:10.3390/s20205919](https://doi.org/10.3390/s20205919) | K1–K3 三表标准球测量模型与 Eq. 1 MDH/Hayati 五参数乘积图（`trical/solver.py`） |
| Pinocchio Rigid Body Algorithms Library. [stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio) | C++ 逆动力学与整臂回归矩阵 `computeJointTorqueRegressor` 计算后端 |
