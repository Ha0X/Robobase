## Ch11 Robot Control（机器人控制）

### 你要会什么

- 理解控制的目标：稳定、可预测、可调试
- 会用最常见的跟踪框架（关节空间 PD/PID、加前馈/补偿）
- 理解“模型补偿为什么有效、什么时候会翻车”

### 核心框架（工程常用）

1) 参考轨迹：$q_d(t), \\dot q_d(t), \\ddot q_d(t)$  
2) 误差：$e=q_d-q$，$\\dot e=\\dot q_d-\\dot q$  
3) 关节空间控制（从简单到强）：\n
- **PD/PID（位置）**：实现简单、调参直观，但高速/强耦合时跟踪差
- **Computed torque（模型补偿）**：
  $$\\tau = M(q)\\,\\ddot q_d + C(q,\\dot q)\\dot q + g(q) + K_p e + K_d \\dot e$$
  直觉：先用模型把“系统变线性”，再用 PD 稳定跟踪

### 关键工程点

- 轨迹越平滑，控制越轻松（Ch09）
- 奇异性/IK 抖动会直接体现在控制层（Ch05/Ch06）
- 模型不准时：补偿可能“帮倒忙”，需要限幅/滤波/鲁棒策略

### 与 ROS2 / MoveIt 的关联

- MoveIt 负责给可行轨迹；控制器负责让硬件跟上
- `ros2_control` 的控制器链路：\n  - joint_state_broadcaster 提供反馈\n  - position/velocity/trajectory controller 输出命令\n  - 硬件接口驱动电机/舵机

### 建议练习

- 用你自己的机械臂（或 fake hardware）：对比\n  - 仅位置控制\n  - 位置控制 + 重力补偿\n  - 轨迹更平滑 vs 更尖锐\n  的跟踪误差与抖动差异

### 官方资源入口

- Modern Robotics 官方主页：https://hades.mech.northwestern.edu/index.php/Modern_Robotics


