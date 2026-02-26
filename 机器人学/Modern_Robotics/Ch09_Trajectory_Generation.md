## Ch09 Trajectory Generation（轨迹生成）

### 你要会什么

- 区分路径（geometry）与轨迹（time）
- 会用基本的时间标定/插值让轨迹满足速度、加速度连续
- 理解为什么“轨迹越平滑，越容易控制”

### 核心概念

- **Path**：$q(s)$（无时间）
- **Trajectory**：$q(t)$（含速度/加速度）
- 关节空间轨迹 vs 任务空间轨迹（末端位姿轨迹）

### 工程常用原则

- 尽量保证 $q,\\dot q,\\ddot q$ 连续（至少 $C^2$），减少抖动与冲击
- 时间标定要考虑最大速度/加速度限制（尤其真实舵机/电机）

### 与 ROS2 / MoveIt 的关联

- MoveIt 规划输出通常会做时间参数化（time parameterization），以满足速度/加速度限制
- 如果你执行时抖动明显，先检查：\n  - 轨迹是否过于尖锐（角点）\n  - 限速/加速度是否合理 \n  - 控制器带宽是否够

### 建议练习

- 关节空间两点之间：对比线性插值 vs 多项式/样条插值（看速度/加速度曲线）
- 任务空间直线 vs 关节空间插值的差别（末端路径往往不是直线）

### 官方资源入口

- Modern Robotics 官方主页：https://hades.mech.northwestern.edu/index.php/Modern_Robotics


