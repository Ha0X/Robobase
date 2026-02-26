## 《Modern Robotics》笔记

本书重点是：**力学（mechanics）/规划（planning）/控制（control）**


### 数学基础

- **矩阵符号速查**：
  - $I$：单位矩阵（identity）
  - $R^T$：转置（transpose）
  - $\det(R)$：行列式（determinant）。对旋转矩阵，$\det(R)=1$ 表示“纯旋转”（不含镜像翻转）。
  - $\mathrm{tr}(R)$：迹（trace），等于对角线元素之和

#### 1) 从“基向量 + 坐标表示”理解旋转矩阵

假设一个 3D 坐标系的单位正交基为 $[e_1,e_2,e_3]$

空间中任意向量（或从原点指向某点的位置向量）都可以写成：

$$
x = [e_1,e_2,e_3]\,
\begin{bmatrix}
a_1\\a_2\\a_3
\end{bmatrix}
$$

如果我们换到另一个坐标系，其基为 $[e'_1,e'_2,e'_3]$，同一个向量也可以写成：

$$
x = [e'_1,e'_2,e'_3]\,
\begin{bmatrix}
a'_1\\a'_2\\a'_3
\end{bmatrix}
$$

当两坐标系**原点相同**（只有旋转，没有平移）时，就存在一个旋转矩阵 \(R\)，把新坐标值变到旧坐标值：

$$
a = R a'
$$

其中

$$
R=[e_1,e_2,e_3]^T [e'_1,e'_2,e'_3]
$$

旋转矩阵的关键性质：

$$
R^TR=I,\quad \det(R)=1,\quad R^{-1}=R^T
$$

这类矩阵组成的集合，就是 **特殊正交群**：

$$
SO(3)=\{R\in \mathbb{R}^{3\times 3}\mid R^TR=I,\ \det(R)=1\}
$$

#### 2) 加上平移：齐次坐标与变换矩阵（Transform matrix）

如果两个坐标系不仅有旋转，还有平移 \(t\)，则点/向量的坐标变换会出现“加法”：

$$
b = R a + t
$$

多次变换会很难看。解决办法是引入**齐次坐标**，把平移也写进一个矩阵里：

$$
\bar a=
\begin{bmatrix}
a\\1
\end{bmatrix},
\quad
T=
\begin{bmatrix}
R & t\\
0^T & 1
\end{bmatrix},
\quad
\bar b = T\bar a
$$

这样复合变换就变成纯乘法：

$$
T_{ac} = T_{ab}T_{bc}
$$

所有这种“旋转+平移”的齐次矩阵构成 **特殊欧式群**：

$$
SE(3)=\left\{
\begin{bmatrix}
R & t\\
0^T & 1
\end{bmatrix}
\middle|\ R\in SO(3),\ t\in \mathbb{R}^3
\right\}
$$

其逆（表示“反向变换”）是：

$$
T^{-1}=
\begin{bmatrix}
R^T & -R^T t\\
0^T & 1
\end{bmatrix}
$$

#### 3) 旋转矩阵更紧凑的表示：轴角（Axis–Angle）

旋转只有 3 个自由度，但旋转矩阵用了 9 个数（还带约束），所以是冗余表示。  
一个常用的紧凑表示是**轴角**：用一个向量表示旋转

- 方向：旋转轴（单位向量 $n$）
- 长度：旋转角度（$\theta$）

也常写成旋转向量：$\phi = n\theta$。

对应的 Rodrigues 公式：

$$
R = \cos\theta\,I + (1-\cos\theta)\,nn^T + \sin\theta\,n^\wedge
$$

其中 $n^\wedge$ 是把向量变成反对称矩阵的算子

另外一个常用关系（从旋转矩阵反求角度）：

$$
\mathrm{tr}(R)=1+2\cos\theta
\quad\Rightarrow\quad
\theta=\arccos\left(\frac{\mathrm{tr}(R)-1}{2}\right)
$$

#### 4) 为什么要引入李群/李代数

很多问题最后会变成最小二乘（比如逆运动学求解，位姿估计等）：

$$
\min_x \frac{1}{2}\|f(x)\|_2^2
$$

常见求解是迭代式更新：$x_{k+1}=x_k+\Delta x$。  
但对旋转矩阵来说，“直接相加”一般会跑出 $SO(3)$（不封闭），因此需要在**群的局部**找到能做加法的空间——这就是李代数/切空间。

#### 5) 微分流形
$SO(3)$, $SE(3)$ 不是平直的 $\mathbb{R}^n$，不能求导、梯度、雅可比。

直观理解：整体不是向量空间，但局部可以用向量近似。

#### 5) 李群的局部（切空间）

如果旋转随时间变化 $R(t)\in SO(3)$，恒有：

$$
R(t)R(t)^T=I
$$

两边对时间求导：

$$
\dot R R^T + R\dot R^T = 0
$$

移项可得：

$$
\dot R R^T = -(\dot R R^T)^T
$$

也就是说 $\dot R R^T$ 是**反对称矩阵**。反对称矩阵和一个 3 维向量一一对应，这个对应关系写成“ hat 算子”：

$$
a=
\begin{bmatrix}
a_1\\a_2\\a_3
\end{bmatrix},
\quad
a^\wedge=
\begin{bmatrix}
0 & -a_3 & a_2\\
a_3 & 0 & -a_1\\
-a_2 & a_1 & 0
\end{bmatrix}
$$

于是我们可以用角速度向量 \(\omega(t)\) 表示：

$$
\dot R R^T = \omega(t)^\wedge
\quad\Rightarrow\quad
\dot R = \omega^\wedge R
$$

这些反对称矩阵的集合就是 **\(so(3)\)**（它是 \(SO(3)\) 在单位元附近的切空间/李代数）。

在这个切空间里：能够把增量当成普通的向量 $\delta$


#### 6) 指数映射：用 $\exp$ 把“加法更新”变回群上

在李代数里做“增量”最方便，但我们最终要回到$SO(3)$/$SE(3)$。  
用指数映射即可把反对称矩阵（或 twist）变成旋转（或位姿）：

$$
R = \exp(\omega^\wedge \theta)
$$

这也是后面 PoE（指数积）公式的基础。


### 基本概念

- **运动链（kinematic chain）**：通过运动副连接而构成的可相对运动系统。
- **构件（link）**：由许多零件组成；刚性连接在一起、共同构成一个独立运动单元的部分称为构件。
- **运动副（kinematic pair / joint）**：由两个构件直接接触而组成的可动连接。
- **机构（mechanism）**：在运动链中，如果将其中某一构件固定为机架（fixed link），该运动链就形成了机构。

### 机器人机构：开链与闭链（串联与并联）

对于机器人机构来说，连杆可以串联排列（常见开链臂），也可以形成闭环链接（如 Stewart–Gough 平台）。

- **开链**：通常所有关节都被驱动。
- **闭链**：通常只有一个子集的关节可以被驱动（其余可能为被动关节）。

### 执行器、传动与传感

- **执行器**：连杆由执行器移动，常见电驱（直流/交流电机、步进电机、形状记忆合金），也可能由气动或液压缸驱动。
- **传动设计**：电机转速高 → 需要合适传动（齿轮/带传动/滑轮/链传动等），同时要满足“保持静止/快速停止”的需求。
- **传感器**：用于测量关节运动（位移/速度/力矩等：编码器、电位器、测速计、力-力矩传感器）。额外传感器可用于定位物体或机器人本体（视觉相机、RGB-D、激光测距仪、声学传感器等）。

### 基本假设

本书的章节基本都把机器人当做刚体系统来讨论，这是一个基础假设。

## 后续章节概览

> 下面是“每章在解决什么问题”的快速导航，详细内容见各章笔记文件。

- **Chapter 2: Configuration Space**：机器人系统的配置、自由度计算、C 空间拓扑结构、工作空间定义等。
- **Chapter 3: Rigid-Body Motions**：如何用数学方法描述刚体在三维物理空间中的运动。
- **Chapter 4: Forward Kinematics**：给定关节位置求末端位姿；PoE 公式；附录讨论 D-H 表示以及 PoE 与 D-H 的转换。
- **Chapter 5: Velocity Kinematics and Statics**：关节速度与末端速度关系（核心：雅可比矩阵），以及静力学关系。
- **Chapter 6: Inverse Kinematics**：确定一组关节位置，实现期望的末端执行器配置。
- **Chapter 7: Kinematics of Closed Chains**：闭链运动分析（多解、驱动/被动关节、奇异性分析与开链不同）。以平面五杆机构与 Stewart–Gough 平台为例，推广到一般闭链。
- **Chapter 8: Dynamics of Open Chains**：开链动力学：欧拉-拉格朗日方程与牛顿-欧拉法。
- **Chapter 9: Trajectory Generation**：如何生成机器人的运动轨迹。
- **Chapter 10: Motion Planning**：复杂工作空间中寻找无碰撞运动，同时满足关节限制、执行器限制和其他物理约束（核心：路径规划）。
- **Chapter 11: Robot Control**：运动（位置）控制、力控制、混合运动力控制、阻抗控制等。
- **Chapter 12: Grasping and Manipulation**：接触建模；形状闭合/力闭合抓取；推动物体、动态携带、稳定性测试等。
- **Chapter 13: Wheeled Mobile Robots**：轮式移动机器人及移动操作平台的运动学、运动规划与控制。

## 参考链接

- Modern Robotics 官方主页：https://hades.mech.northwestern.edu/index.php/Modern_Robotics