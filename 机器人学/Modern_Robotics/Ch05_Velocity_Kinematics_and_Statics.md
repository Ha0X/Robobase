## Ch05 Velocity Kinematics and Statics（速度运动学与静力学）


本章要解决两个核心问题：

- **速度运动学**：已知关节速度 $\dot q$，末端执行器的速度/角速度（twist）是什么？
- **静力学**：已知末端外力/外力矩（wrench），关节需要输出的力矩 $\tau$ 是多少？

---

## Twist：用 6 维向量描述“转 + 平移”的瞬时运动

刚体在空间中的瞬时运动可用 twist 表示：

$$
\mathcal{V} =
\begin{bmatrix}
\omega\\
v
\end{bmatrix}
\in \mathbb{R}^6
$$

- $\omega$：角速度（3 维）
- $v$：线速度（3 维）


---

## 2. Jacobian：把关节速度映射到末端 twist

设关节变量为 $q=[q_1,\dots,q_n]^T$，其速度为 $\dot q$。末端 twist 与关节速度的关系为：

$$
\mathcal{V} = J(q)\,\dot q
$$

这里的 $J(q)\in\mathbb{R}^{6\times n}$ 称为 Jacobian（雅可比矩阵），它是 $q$ 的函数。

### 2.1 “链式法则”的直觉（为什么会有 Jacobian）

如果你用某种最小坐标 $x$ 描述末端位姿/配置（例如平面 2R 用 $x=[x,y]^T$），那么：

$$
x = f(q)
\quad\Rightarrow\quad
\dot x = \frac{\partial f}{\partial q}\,\dot q
$$

这就是 Jacobian 的最基本来源：**FK 对 $q$ 求导**。

> 区别：$\dot x$ 是“坐标速度”，而 twist 是“刚体速度”（本质不同，但常常可以互相转换）。

---

## 3. 一个 2R 平面开链的例子：奇异与 Jacobian 的几何意义

考虑一个 2R 平面机械臂（两转动关节，连杆长度 $L_1,L_2$），末端位置可写为：

$$
\begin{aligned}
x &= L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2)\\
y &= L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2)
\end{aligned}
$$

对时间求导：

$$
\begin{bmatrix}\dot x\\ \dot y\end{bmatrix}
=
J(\theta)
\begin{bmatrix}\dot\theta_1\\ \dot\theta_2\end{bmatrix}
$$

其中一个常见形式的 Jacobian 为：

$$
J(\theta)=
\begin{bmatrix}
-L_1\sin\theta_1 - L_2\sin(\theta_1+\theta_2) & -L_2\sin(\theta_1+\theta_2)\\
\ \ L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2) & \ \ L_2\cos(\theta_1+\theta_2)
\end{bmatrix}
$$

### 3.1 奇异（singularity）是什么？

当 $J$ 的两列向量共线（或矩阵秩下降）时，末端在某些方向上**无法产生速度**，或者需要**非常大的关节速度**才能换来很小的末端速度。

对 2R 来说，典型奇异发生在：

- **两连杆共线**：$\theta_2=0$ 或 $\theta_2=\pi$（伸直/折叠）

这时 Jacobian 退化，末端速度可达方向“降维”，也就是你在工程里常说的“到奇点了”。

---

## 4. 可操纵性椭球（manipulability ellipsoid）：把“关节速度单位圆”映射到“末端速度椭圆”

把关节速度限制为单位球（2R 时就是单位圆）：

$$
\|\dot q\|\le 1
$$

通过 $\dot x = J\dot q$ 映射到末端速度空间，会得到一个椭圆（高维时是椭球）：

- **离奇异越远**：椭圆越“接近圆”（各方向能力更均衡）
- **靠近奇异**：椭圆会被压扁（某些方向速度能力趋近 0）

一个常用的量化指标是 Yoshikawa manipulability：

$$
w(q)=\sqrt{\det(JJ^T)}
$$

（对非方阵/不同定义的 $J$ 会有细节差异，但直觉一致：**越小越接近奇异**。）

---

## 5. 静力学（Statics）：末端 wrench 与关节力矩的映射

设末端受外力/外力矩（wrench）为：

$$
\mathcal{F}=
\begin{bmatrix}
\tau_{\text{ext}}\\
f_{\text{ext}}
\end{bmatrix}
\in\mathbb{R}^6
$$

关节力矩为 $\tau\in\mathbb{R}^n$。在理想无损情况下，用“功率守恒”的观点：

$$
\mathcal{F}^T\mathcal{V}=\tau^T\dot q
$$

代入 $\mathcal{V}=J\dot q$，并利用“对任意 $\dot q$ 都成立”，得到本章最常用的静力学关系：

$$
\tau = J(q)^T\,\mathcal{F}
$$

### 5.1 反过来：给定关节力矩，末端能“等效输出”多大 wrench？

当 $J$ 在该构型下可逆/满秩（方阵时）：

$$
\mathcal{F} = J(q)^{-T}\,\tau
$$

靠近奇异时，$J^{-T}$ 会变得病态：会出现“某些方向的力放大/衰减非常夸张”的现象。

---

## 6. Space Jacobian 与 Body Jacobian（定系/动系）

和第 4 章的 “space form / body form” 类似，twist 也有两种常用表达坐标系：

- **Space（定系）twist**：站在基座/世界坐标系看末端怎么动
- **Body（动系）twist**：站在末端坐标系上看自己怎么动

对应 Jacobian 也有两种：

$$
\mathcal{V}_s = J_s(q)\dot q,
\quad
\mathcal{V}_b = J_b(q)\dot q
$$

二者关系可用伴随变换（Adjoint）写出（第 3 章内容）：

$$
\mathcal{V}_s = \mathrm{Ad}_{T(q)}\,\mathcal{V}_b
\quad\Rightarrow\quad
J_s(q)=\mathrm{Ad}_{T(q)}\,J_b(q)
$$

> 直觉：同一个“物理运动”不变，换个坐标系描述就要乘一个坐标变换。

---

## 7. Analytic Jacobian vs Geometric Jacobian（坐标速度 vs 刚体速度）

如果你用“某种参数化坐标” $x$ 描述末端配置（例如欧拉角/轴角/四元数参数等），那么会出现两类 Jacobian：

- **Geometric Jacobian**：把 $\dot q$ 映射到 twist（更接近刚体运动本质）
- **Analytic Jacobian**：把 $\dot q$ 映射到坐标速度 $\dot x$（依赖你怎么选 $x$）

它们之间通常满足一个“转换矩阵”关系：

$$
\dot x = J_a(q)\dot q,\quad
\mathcal{V} = J_g(q)\dot q,\quad
\dot x = A(x)\,\mathcal{V}
$$

其中 $A(x)$ 由姿态参数化决定（例如欧拉角在某些角度会出现奇异，这是“参数化奇异”，和机械臂几何奇异是两回事，但工程里会一起导致数值不稳）。

---

## 8. 工程直觉与常见坑（强烈建议记住）

- **别混用 frame**：你写的是 $\mathcal{V}_s$ 还是 $\mathcal{V}_b$？对应的是 $J_s$ 还是 $J_b$？
- **奇异附近别硬求逆**：$J^{-1}$ / $J^{-T}$ 数值会爆；用伪逆/阻尼最小二乘（DLS）更稳。
- **“末端速度”到底指什么**：是 $\dot x$（坐标速度）还是 $\mathcal{V}$（twist）？别用同一个符号混写。
- **可操纵性椭球 vs 力椭球**：主轴方向对齐；力椭球的半轴长度与可操纵性椭球“互为倒数”（直觉：速度强的方向通常力弱，反之亦然）。

---

## 目录链接（学习过程中会逐步更新）

- 现代机器人：力学，规划，控制（chapter1）- Mr.Bo（知乎）：https://zhuanlan.zhihu.com/p/369236960
- 现代机器人：力学，规划，控制（chapter4）- Mr.Bo（知乎）：https://zhuanlan.zhihu.com/p/369819171
- Modern Robotics 官方主页：https://hades.mech.northwestern.edu/index.php/Modern_Robotics
