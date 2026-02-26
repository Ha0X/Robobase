## Ch03 Rigid-Body Motions（刚体运动：如何表示“位置+方向”）

> 本章核心：把“刚体的位姿（position + orientation）”用统一的数学对象表示出来，并能在不同坐标系之间做变换与连乘。

- 刚体位姿至少需要 6 个数字来指定一个刚体在三维物理空间中的位置和方向


- 动系相对于固定参考系被表示为 4×4 的实矩阵，这个矩阵就是 C 空间的隐式表示，将 10 个约束条件应用于 4×4 实矩阵的 16 维空间，就可以得到描述刚体的 6 维空间。



- 用齐次变换矩阵表示位姿：

  $$
  T =
  \begin{bmatrix}
  R & p \\\\
  0 & 1
  \end{bmatrix}
  $$

  - $R$ 是 3×3 旋转矩阵（属于 $SO(3)$）
  - $p$ 是 3×1 平移向量

- 为什么说“16 维 + 约束 → 6 维”？

  4×4 实矩阵有 16 个元素，但只有满足以下约束才是合法位姿矩阵：
  - 最后一行固定为 $[0\ 0\ 0\ 1]$（4 个约束）
  - $R \in SO(3)$：$R^TR=I$（6 个独立约束）且 $\det(R)=1$（1 个约束；但常与前者一起理解）



### 平面的例子：定系 {s} 与动系 {b}


- 考虑物体在平面内运动，定系为 {s}，动系为 {b}
- 定系单位轴向量为 $\hat{x}_s,\hat{y}_s$，动系单位轴向量为 $\hat{x}_b,\hat{y}_b$


#### 动系原点 p 在定系中的表达

以定系 {s} 为参考系，动系 {b} 的原点可以用向量表示：

$$
p=
\begin{bmatrix}
p_x \\
p_y
\end{bmatrix}
$$

#### 定系与动系的关系（旋转 + 平移）

动系 {b} 的单位轴向量（在 {s} 中的表达）可以组成一个矩阵：

$$
P=
\begin{bmatrix}
\hat{x}_b & \hat{y}_b
\end{bmatrix}
$$

其中 $P$ 称为旋转矩阵（rotation matrix），并满足（二维情形下）：

$$
P^T P = I,\quad \det(P)=1
$$

如果 {b} 相对 {s} 旋转角为 $\theta$，则：

$$
P=
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
$$

也可以把旋转+平移合并成二维齐次变换（$SE(2)$）


### 引入第三个坐标系 {c}：变换的连乘

现在我们再引入一个动系 {c}

同理，如果用定系 {s} 来描述动系 {c}，可以写成一对（平移向量 + 旋转矩阵）：

$$
r=
\begin{bmatrix}
r_x \\
r_y
\end{bmatrix},
\quad
R=
\begin{bmatrix}
\cos\phi & -\sin\phi \\
\sin\phi & \cos\phi
\end{bmatrix}
$$

当然，我们也可以用动系 {b} 为参考系来描述动系 {c}，则动系 {c} 相对于 {b} 的表达可写作：

$$
q=
\begin{bmatrix}
q_x \\
q_y
\end{bmatrix},
\quad
Q=
\begin{bmatrix}
\cos\psi & -\sin\psi \\
\sin\psi & \cos\psi
\end{bmatrix}
$$

如果已知 {c} 相对 {b} 的关系（$q,Q$）以及 {b} 相对 {s} 的关系（$p,P$），那么 {c} 相对 {s} 的关系就可以这样计算：

$$
R = P Q
$$

$$
r = P q + p
$$

> 直觉：先把 {c} 在 {b} 下的“位移/方向”转到 {s} 下（乘 $P$），再叠加 {b} 在 {s} 下的平移 $p$。

用齐次矩阵写就是连乘（二维 $SE(2)$）：

$$
T_{sc} = T_{sb} T_{bc}
$$

（对应到 3D 就是 $SE(3)$ 的同样结构：$T_{ac}=T_{ab}T_{bc}$。）



### 旋转矩阵的下标

用下标明确表达“从哪个系到哪个系”：

$R_{ab}$：把 **b 系坐标**变到 **a 系坐标**（也常写成“b 相对 a”）

于是有：

逆：  
  $$
  R_{ba} = R_{ab}^{-1} = R_{ab}^T
  $$
复合（下标消去）：  
  $$
  R_{ac} = R_{ab}\,R_{bc}
  $$

同样，向量坐标也遵循同样的“消去”直觉：

$$
p_a = R_{ab}\,p_b
$$

### 角速度、$so(3)$ 与 $[\omega]$

这一小节要解决的是：

- 我们经常知道/估计的是姿态 $R(t)$（随时间变化的旋转矩阵）
- 但控制/动力学/雅可比里需要的是角速度 $\omega(t)$
- 问题：**角速度是向量**，而 $R(t)$ 是矩阵——二者怎么严格联系起来？


1. **hat 映射（$\wedge$ 映射）** 把向量变成反对称矩阵：$\omega\in\mathbb{R}^3 \leftrightarrow \omega^\wedge=[\omega]\in so(3)$（反过来用 vee 映射 $\vee$：$[\omega]^\vee=\omega$）  
2. **从 $R(t)$ 求角速度矩阵**（空间/本体两种看法）：
   $$
   [\omega_s] = \dot R R^T,\qquad [\omega_b]=R^T\dot R
   $$
3. **同一个物理角速度在不同坐标系下的坐标关系**：
   $$
   \omega_s = R\,\omega_b
   $$

下面的推导说明了“一定会出现 $so(3)$ 的反对称矩阵”


若 $R(t)\in SO(3)$，恒有：

$$
R(t)R(t)^T = I
$$

两边对时间求导：

$$
\dot R R^T + R\dot R^T = 0
$$

移项：

$$
\dot R R^T = -(\dot R R^T)^T
$$

因此 $\dot R R^T$ 是反对称矩阵（属于 $so(3)$）。



对任意 $\omega=[\omega_x,\omega_y,\omega_z]^T$，定义：

$$
[\omega] =
\begin{bmatrix}
0 & -\omega_z & \omega_y \\
\omega_z & 0 & -\omega_x \\
-\omega_y & \omega_x & 0
\end{bmatrix}
$$

它满足：$[\omega]^T = -[\omega]$。

同时它还有一个非常关键的几何意义：

$$
[\omega]\,v = \omega \times v
$$



### 空间角速度与本体角速度

令 $R(t)$ 表示“动系 b 相对定系 s 的旋转”，则有：

- **空间角速度（spatial / space-frame）**：
  $$
  [\omega_s] = \dot R R^{-1} = \dot R R^T
  $$
- **本体角速度（body-frame）**：
  $$
  [\omega_b] = R^{-1}\dot R = R^T \dot R
  $$

两者的关系（同一个物理角速度，不同坐标系表达）：

$$
\omega_s = R\,\omega_b
$$

> 左乘/右乘对应“在哪个坐标系里看”（space vs body）。

#### 一个例子：绕定系 $z$ 轴旋转

设 $R(t)=R_z(\theta(t))$：

$$
R(t)=
\begin{bmatrix}
\cos\theta & -\sin\theta & 0\\
\sin\theta & \cos\theta & 0\\
0 & 0 & 1
\end{bmatrix}
$$

对时间求导并代入 $[\omega_s]=\dot R R^T$，你会得到：

$$
[\omega_s] =
\begin{bmatrix}
0 & -\dot\theta & 0\\
\dot\theta & 0 & 0\\
0 & 0 & 0
\end{bmatrix}
\quad\Longleftrightarrow\quad
\omega_s=
\begin{bmatrix}
0\\0\\\dot\theta
\end{bmatrix}
$$

这正是常识：绕 $z$ 轴转，角速度方向沿 $z$，大小是 $\dot\theta$。

---

## 运动旋量（Twist）与力旋量（Wrench）

这一节的目标是把“刚体的**角速度/线速度**”升级成一个统一对象，并解释它为什么是后续 **Jacobian（Ch05）** 和 **动力学/力（Ch08–Ch09）** 的通用语言。

### 1) 运动旋量 Twist：把角速度和线速度打包成 6 维向量

刚体在 3D 的瞬时运动可以用两部分描述：

- 角速度：$\omega \in \mathbb{R}^3$
- 线速度：$v \in \mathbb{R}^3$

把它们打包成一个 6 维向量（常见约定之一）：

$$
V =
\begin{bmatrix}
\omega\\
v
\end{bmatrix}
\in \mathbb{R}^6
$$

这里 $V$ 就是**运动旋量（twist）**。  
直觉：它描述了刚体“此刻的螺旋运动趋势”（一般运动 = 绕某条轴旋转 + 沿该轴平移）。

#### (a) space twist vs body twist（同一物理运动，不同坐标表达）

- **空间旋量**：$V_s = \begin{bmatrix}\omega_s\\ v_s\end{bmatrix}$（在定系/space frame 中表达）
- **本体旋量**：$V_b = \begin{bmatrix}\omega_b\\ v_b\end{bmatrix}$（在动系/body frame 中表达）

它们之间通过 **Adjoint（伴随）变换**联系（下面给公式）。

#### (b) twist 的 hat 映射：从 6D 向量到 $se(3)$ 矩阵

和 $so(3)$ 一样，$se(3)$ 也用矩阵来表示（这样乘法和指数映射更自然）：

$$
V^\wedge =
\begin{bmatrix}
\omega^\wedge & v\\
0^T & 0
\end{bmatrix}
\in se(3)
$$

其中 $\omega^\wedge$ 是 $so(3)$ 的反对称矩阵（你前面已经见过 $[\omega]$）。

### 2) 从位姿 $T(t)\in SE(3)$ 定义 twist（和上一节完全平行）

令刚体位姿（齐次变换）为：

$$
T(t)=
\begin{bmatrix}
R(t) & p(t)\\
0^T & 1
\end{bmatrix}
$$

则定义：

- **空间旋量（space twist）**：
  $$
  V_s^\wedge = \dot T\,T^{-1}
  $$
- **本体旋量（body twist）**：
  $$
  V_b^\wedge = T^{-1}\dot T
  $$

这和上一节的 $[\omega_s]=\dot R R^{-1}$、$[\omega_b]=R^{-1}\dot R$ 是同一个套路：  
**左乘/右乘决定你在 space 还是 body 坐标系里表达速度。**

### 3) 旋量的坐标变换：Adjoint（伴随）变换

若

$$
T=
\begin{bmatrix}
R & p\\
0^T & 1
\end{bmatrix}\in SE(3)
$$

则其 Adjoint 定义为：

$$
\mathrm{Ad}_T=
\begin{bmatrix}
R & 0\\
p^\wedge R & R
\end{bmatrix}
$$

它把同一个 twist 在不同坐标系下的坐标联系起来（一个常用关系）：

$$
V_s = \mathrm{Ad}_T\,V_b
$$

> 这条式子非常重要：后面推 Jacobian（space Jacobian / body Jacobian）基本都在用它。

### 4) 力旋量 Wrench：把力和力矩打包成 6 维向量

刚体受到的“广义力”也可以打包成 6 维：

- 力：$f\in\mathbb{R}^3$
- 力矩（扭矩）：$\tau\in\mathbb{R}^3$

常见约定：

$$
F =
\begin{bmatrix}
\tau\\
f
\end{bmatrix}
\in \mathbb{R}^6
$$

这里 $F$ 就是**力旋量（wrench）**。

### 5) twist 与 wrench 为什么要成对出现：瞬时功率（power）

一个关键的不变标量是瞬时功率：

$$
P = F^T V
$$

它表示“这组力在这个瞬时运动上做功的速率”。  
因此 twist/wrench 的坐标变换必须相互匹配，才能保证 $P$ 不变。

对应的 wrench 坐标变换是（与 twist 对偶）：

$$
F_s = \mathrm{Ad}_T^{-T}\,F_b
$$

### 6) 三个最小例子（建立直觉）

- **纯旋转**：$v=0$，$V=[\omega;0]$（绕某轴转，不发生平移）
- **纯平移**：$\omega=0$，$V=[0;v]$（不转，只平移）
- **一般螺旋**：$\omega\neq 0$ 且 $v\neq 0$（绕轴转，同时沿轴滑动）





