## Ch10 Motion Planning（运动规划）

### 你要会什么

- 把规划问题表述成：在 C-space 的可行域中找一条从 start 到 goal 的路径
- 理解采样规划的核心直觉：PRM / RRT（为什么“采样”能在高维里工作）
- 理解规划与优化的差别（可行性 vs 最优性）

### 核心概念（工程视角）

- **碰撞检测**是瓶颈：规划算法很多时候在“尽量少调用碰撞检测”
- **约束规划**：姿态约束、末端约束、关节耦合约束会让可行域变成复杂流形
- **后处理**：shortcut/smoothing/time-parameterization 往往比“规划算法本身”更影响执行效果

### 与 ROS2 / MoveIt 的关联

- MoveIt 的 OMPL 规划器家族基本都属于采样规划（RRTConnect/PRM 等）
- 你在 RViz 里做 Plan：\n  - 约束越多越慢\n  - 环境越复杂越容易失败\n  - 初值/IK/碰撞模型会强烈影响结果

### 建议练习

- 2D/低维 toy problem：实现一个最小 RRT（只要能从 start 连到 goal）建立直觉
- MoveIt 实践：同一个目标位姿，改变关节限制/碰撞膨胀参数，看成功率与路径变化

### 官方资源入口

- Modern Robotics 官方主页：https://hades.mech.northwestern.edu/index.php/Modern_Robotics


