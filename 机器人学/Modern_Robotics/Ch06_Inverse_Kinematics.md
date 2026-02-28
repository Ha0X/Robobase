## Ch06 Inverse Kinematics（逆运动学）




## IK：问题定义

已知目标位姿 $T_{sd}\in SE(3)$（s：space/world，d：desired），希望找到关节变量 $q$ 使：

$$
T_{sb}(q) \approx T_{sd}
$$

这里 $T_{sb}(q)$ 是 FK 给出的“基座到末端（body）”的齐次变换。

IK 常见难点：

- **解不唯一**：同一个末端位姿可能对应多组关节解（多解/分支）
- **可能无解析解**：很多结构无法写出封闭形式（closed-form）的关节解表达式
- **数值解依赖初值**：迭代法本质是局部方法

---

## 解析 IK vs 数值 IK（分类与适用场景）

### 解析 IK（Analytical / Closed-form）

- 给出“代数/几何推导”得到的封闭解（或一组分支解）
- **速度快、确定性强**（同一目标通常能枚举所有分支）
- 但依赖机构结构（比如某些 6-DOF 串联臂满足特定几何条件），**通用性差**

- **IKFast**：给定机械臂模型自动生成解析 IK 代码（常被 MoveIt 作为插件使用）

### 数值 IK（Numerical / Iterative）

- **通用**：只要你能算 FK 和 Jacobian，基本就能做
- **依赖初值**：可能收敛到不同解分支；也可能不收敛/收敛慢
- 需要处理 **奇异性、步长、阻尼、关节限位** 等工程问题

本笔记的主体内容就是：**用 $SE(3)$ 误差 twist + Jacobian（$J_b$ 或 $J_s$）做数值迭代 IK**。

---

### 位姿误差：把 $SE(3)$ 误差写成 $se(3)$ 的 twist（6 维）

一个常用且稳定的做法是：把位姿误差写成 $se(3)$ 中的 twist（6 维向量）。

#### 末端系（body）误差

定义相对变换：

$$
T_{bd}(q) = T_{sb}(q)^{-1}\,T_{sd}
$$

取对数映射到 $se(3)$：

$$
[\,\mathcal{V}_b\,] = \log\!\left(T_{bd}(q)\right)
$$

再用 $\vee$ 映射得到 6 维误差向量：

$$
\mathcal{V}_b = \mathrm{Log}\left(T_{sb}(q)^{-1}T_{sd}\right)^\vee
$$

直觉：**站在末端坐标系上**看“我还差多少才能对齐目标”。

#### 空间系（space）误差

同理也可以定义空间系误差：

$$
T_{ds}(q)=T_{sd}\,T_{sb}(q)^{-1},\quad
\mathcal{V}_s=\mathrm{Log}(T_{ds}(q))^\vee
$$

> 规则：**误差用哪种坐标系，Jacobian 就用哪种**（$\mathcal{V}_b \leftrightarrow J_b$；$\mathcal{V}_s \leftrightarrow J_s$）。

---

## 数值 IK：Jacobian 线性化与迭代更新

以 body form 举例（space form 同理）。

### 线性化

在 $q_k$ 附近，用 Jacobian 做一阶近似：

$$
\mathcal{V}_b(q_k) \approx J_b(q_k)\,\Delta q
$$

### 迭代更新（伪逆版本：能用但不稳）

$$
\Delta q = J_b(q_k)^\dagger\,\mathcal{V}_b(q_k),\quad
q_{k+1} = q_k + \Delta q
$$

---

## 数值 IK 的“可实现 recipe”（工程常用：body error + body Jacobian + DLS）

这是一套最常见、工程上更稳的配置：**body 误差 $\mathcal{V}_b$ + body Jacobian $J_b$ + DLS（阻尼最小二乘）**。

### 输入 / 输出

- **输入**
  - 目标位姿：$T_{sd}\in SE(3)$
  - 初值：$q_0$（很重要）
  - FK：$T_{sb}(q)$（Ch04）
  - Jacobian：$J_b(q)$
  - 超参数：
    - 阻尼 $\lambda$（DLS 稳定性关键）
    - 最大迭代次数 $K$
    - 步长系数 $\alpha$（可选，用来避免 overshoot）
    - 关节限位 $q_{\min},q_{\max}$
- **输出**
  - 关节解 $q^*$（或失败：不收敛/不可达）

### 每轮迭代做什么

对 $k=0,1,2,\dots$：

- **算 FK**

$$
T_k = T_{sb}(q_k)
$$

- **算误差 twist（末端系）**

$$
\mathcal{V}_b(q_k)
=
\mathrm{Log}\!\left(T_k^{-1}T_{sd}\right)^\vee
$$

- **停止条件（常用）**
  - 误差足够小：$\|\mathcal{V}_b\| < \varepsilon$
  - 或更新量足够小：$\|\Delta q\| < \varepsilon_q$

- **算 Jacobian（末端系）**

$$
J_k = J_b(q_k)
$$

- **DLS 更新（核心）**

$$
\Delta q
=
J_k^T\left(J_kJ_k^T+\lambda^2 I\right)^{-1}\mathcal{V}_b
$$

- **可选：步长（避免 overshoot）**

$$
q_{k+1} = q_k + \alpha\,\Delta q,\quad \alpha\in(0,1]
$$

- **关节限位（最低配）**

$$
q_{k+1} \leftarrow \mathrm{clip}(q_{k+1},\,q_{\min},\,q_{\max})
$$


### 最关键的 3 个坑（不处理就会“算不出来”）

- **初值 $q_0$**：IK 是局部迭代，初值差会跑到别的分支或不收敛（工程上常用多初值重启）
- **奇异附近**：用伪逆很容易炸；DLS（$\lambda>0$）能显著稳住
- **frame 混用**：$\mathcal{V}_b$ 必须配 $J_b$；$\mathcal{V}_s$ 必须配 $J_s$


---

## 伪逆 vs DLS：稳定性的核心差异（为什么 DLS 更稳）

### Moore–Penrose 伪逆（能用，但奇异附近会炸）

- **问题**：当 $J$ 退化/条件数很大时，$J^\dagger$ 会把噪声放大，导致 $\Delta q$ 爆炸。
- **现象**：靠近奇异点时，更新量会突然变得非常大，迭代容易发散/抖动。

### Damped Least Squares（DLS，工程最常用）

思路：把“解线性方程”变成“带正则的最小二乘”，在奇异附近也能稳住数值。

最常用的更新形式：

$$
\Delta q
=
J^T\left(JJ^T+\lambda^2 I\right)^{-1}\mathcal{V}
$$

- **$\lambda$ 越大**：越稳，但收敛更慢、精度可能更差
- **$\lambda$ 越小**：越像伪逆，但更容易在奇异附近发散

> 实用策略：根据奇异性/条件数自适应调 $\lambda$（靠近奇异就加大阻尼）。

---

## 冗余机械臂（DOF > 6）

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

## 常见失败/不稳定原因

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



---



