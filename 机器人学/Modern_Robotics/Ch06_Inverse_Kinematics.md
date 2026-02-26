## Ch06 Inverse Kinematics（逆运动学）




---

## IK
IK的目的是：

已知目标位姿 $T_{sd}\in SE(3)$（s：space/world，d：desired），希望找到关节变量 $q$ 使：

$$
T_{sb}(q) \approx T_{sd}
$$

这里 $T_{sb}(q)$ 是 FK 给出的“基座到末端（body）”的齐次变换。

然而，可能出现解不唯一/无解析解的情况



---

### 位姿误差

一个常用、稳定的做法是：把误差写成 $se(3)$ 的 twist（6 维）。

#### 末端系误差

定义：

$$
T_{bd}(q) = T_{sb}(q)^{-1}\,T_{sd}
$$

把它取对数映射到 $se(3)$：

$$
[\,\mathcal{V}_b\,] = \log\!\left(T_{bd}(q)\right)
$$

再用 $\vee$ 映射得到 6 维误差向量：

$$
\mathcal{V}_b = \mathrm{Log}\left(T_{sb}(q)^{-1}T_{sd}\right)^\vee
$$

直觉：**站在末端坐标系上**看“我还差多少才能对齐目标”。

#### 空间系误差

同理也可以定义空间系误差：

$$
T_{ds}(q)=T_{sd}\,T_{sb}(q)^{-1},\quad
\mathcal{V}_s=\mathrm{Log}(T_{ds}(q))^\vee
$$

> Space/Body **误差用哪种，Jacobian 就用哪种**（$J_s$ vs $J_b$）

---

## 数值 IK 的基本迭代框架

### 最常用的写法：用 Jacobian 把末端误差映射回关节增量

以 body form 举例（space form 同理）：

$$
\mathcal{V}_b(q_k) = \mathrm{Log}\left(T_{sb}(q_k)^{-1}T_{sd}\right)^\vee
$$

用 Jacobian 近似线性化：

$$
\mathcal{V}_b \approx J_b(q)\,\Delta q
$$

于是更新：

$$
\Delta q = J_b(q_k)^\dagger\,\mathcal{V}_b(q_k),\quad
q_{k+1} = q_k + \Delta q
$$

---

## 到底“怎么算 IK”（可直接照着实现的 recipe）

你可以把一个实用的数值 IK 当成下面这个循环（这里用 **body error + body Jacobian + DLS**，因为工程上最稳、最常见）：

- **输入**
  - 目标位姿：$T_{sd}\in SE(3)$
  - 初值：$q_0$（很重要）
  - FK：$T_{sb}(q)$（你在 Ch04 已经有）
  - Jacobian：$J_b(q)$（可以来自你自己的推导，也可以来自库/MoveIt/KDL）
  - 超参数：阻尼 $\lambda$，最大迭代次数 $K$，步长系数 $\alpha$（可选），关节限位 $q_{\min},q_{\max}$

- **输出**
  - 关节解 $q^*$（或失败：不收敛/不可达）

### 计算步骤（每一轮都算什么）

对 $k=0,1,2,\dots$：

- **算 FK**

$$
T_k = T_{sb}(q_k)
$$

- **算位姿误差（末端系）**

$$
\mathcal{V}_b(q_k)
=
\mathrm{Log}\!\left(T_k^{-1}T_{sd}\right)^\vee
$$

- **停止条件（常用）**
  - 如果 $\|\mathcal{V}_b\| < \varepsilon$：认为收敛
  - 或者更新量很小：$\|\Delta q\| < \varepsilon_q$

- **算 Jacobian（末端系）**

$$
J_k = J_b(q_k)
$$

- **用 DLS 算关节增量（核心一步）**

$$
\Delta q
=
J_k^T\left(J_kJ_k^T+\lambda^2 I\right)^{-1}\mathcal{V}_b
$$

可选：加步长（避免 overshoot）

$$
q_{k+1} = q_k + \alpha\,\Delta q,\quad \alpha\in(0,1]
$$

- **处理关节限位（最低配做法）**

$$
q_{k+1} \leftarrow \mathrm{clip}(q_{k+1},\,q_{\min},\,q_{\max})
$$

### 伪代码（照抄即可）

```text
q = q0
for k in range(K):
    T = FK(q)                       # T_sb(q)
    V = Log( inv(T) * T_sd ).vee    # body error twist (6x1)
    if norm(V) < eps: return q
    J = J_body(q)                   # 6xn
    dq = J.T * inv(J*J.T + lam^2*I) * V   # DLS update
    q  = clip(q + alpha*dq, qmin, qmax)
return FAIL
```

### 最关键的 3 个坑（不处理就会“算不出来”）

- **初值 $q_0$**：IK 是局部迭代，初值差会跑到别的分支或不收敛（工程上通常会多随机/多初值重启）
- **奇异附近**：用伪逆很容易炸；DLS（$\lambda>0$）能显著稳住
- **frame 混用**：你用的是 $\mathcal{V}_b$ 就必须配 $J_b$；用 $\mathcal{V}_s$ 就必须配 $J_s$

### 为什么它像 Newton？

它本质是在最小化一个误差范数（例如 $\|\mathcal{V}\|^2$），用一阶近似把非线性问题变成最小二乘。

---

## 伪逆 vs DLS：稳定性的核心差异

### Moore–Penrose 伪逆（能用，但奇异附近会炸）

当 $J$ 退化/条件数很大时，$J^\dagger$ 会把噪声放大，导致 $\Delta q$ 爆炸。

### Damped Least Squares（DLS，工程最常用）

最常用的稳定版本：

$$
\Delta q
=
J^T\left(JJ^T+\lambda^2 I\right)^{-1}\mathcal{V}
$$

- $\lambda$ 越大：越稳，但收敛更慢、精度可能更差
- $\lambda$ 越小：越像伪逆，但更容易在奇异附近发散

> 实用策略：根据奇异性/条件数自适应调 $\lambda$（靠近奇异就加大阻尼）。

---

## 冗余机械臂（DOF > 6）：nullspace 让你“边 IK 边做别的事”

当 $n>6$ 时，解通常不唯一。一个经典形式：

$$
\Delta q = J^\dagger \mathcal{V} + (I - J^\dagger J)\,z
$$

- 第一项：完成主要任务（到达目标位姿）
- 第二项：在不影响末端任务的前提下，做次要优化（$z$ 可以是梯度下降方向）

常见次目标：

- 远离关节限位（保持“中间姿态”）
- 远离奇异（提高 manipulability）
- 躲避自碰撞/环境碰撞
- 平滑（让 $\Delta q$ 更小、更连续）

---

## 常见失败/不稳定原因（工程排错清单）

- **初值不好**：落到另一支解、或直接不收敛（IK 对初值高度敏感）
- **接近奇异点**：$J$ 条件数爆炸，伪逆更新会非常大（用 DLS/限幅）
- **目标不可达**：位置/姿态要求过强或违反关节限位/碰撞约束
- **误差与 Jacobian frame 混用**：例如用 $\mathcal{V}_b$ 配 $J_s$（必乱）
- **步长过大**：即使方向对，也会 overshoot（加 step size/line search/限幅）

---

## MoveIt / ROS2 里 IK 是怎么落地的？

- **IK 插件**：KDL、TRAC-IK、IKFast 等，本质都在做“给定目标 pose → 求 joint angles”
- **你常见的现象**：
  - “Plan 能算出来但姿态很怪/绕远”：IK 解选择、约束、代价函数 + 冗余自由度导致
  - “某些姿态抖/卡”：奇异附近 + 数值 IK + 限速/限位共同作用

如果你用的是 MoveIt2，后面我们可以把 SO101 的 URDF/关节限位拿来做一个“可复现”的 IK 调试流程（看 TF、看关节 limits、看初值、看收敛）。

---

## 目录链接（学习过程中会逐步更新）

- 现代机器人：力学，规划，控制（chapter1）- Mr.Bo（知乎）：https://zhuanlan.zhihu.com/p/369236960
- Modern Robotics 官方主页：https://hades.mech.northwestern.edu/index.php/Modern_Robotics


