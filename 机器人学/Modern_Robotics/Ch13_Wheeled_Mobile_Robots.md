## Ch13 Wheeled Mobile Robots（轮式移动机器人）

### 你要会什么

- 理解轮式机器人常见约束：非完整约束（nonholonomic）
- 能写出差速/汽车模型的基本运动学
- 理解“控制/规划/状态估计（SLAM）”在移动机器人里如何拼起来

### 核心概念

- **非完整约束**：不能直接在所有方向产生速度（例如轮子不能侧滑）
- **模型**：差速（unicycle/differential drive）、Ackermann 等
- **里程计**：轮速积分得到位姿（误差会漂）

### 与 SLAM / ROS2 的关联（工程闭环）

- 移动机器人常见闭环：\n  - 里程计/IMU/视觉/雷达 → 状态估计（定位）\n  - 规划（全局+局部）\n  - 控制（跟踪轨迹）
- ROS2 Nav2 就是把这些模块工程化（后续可单独开一章）

### 建议练习

- 写一个差速模型的离散积分（给 $v,\\omega$ 推进位姿）
- 做一次“里程计漂移”模拟：\n  - 加一点轮速噪声\n  - 看轨迹如何偏离

### 官方资源入口

- Modern Robotics 官方主页：https://hades.mech.northwestern.edu/index.php/Modern_Robotics


