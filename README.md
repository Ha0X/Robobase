# Robobase

Personal robotics study repository. Covers physical robot bring-up, ROS 2 fundamentals, and robotics theory (Modern Robotics textbook).

---

## Contents

| Path | Description |
|---|---|
| [`soarm101/`](./soarm101/) | SO-ARM101 low-cost arm: ROS 2 + MoveIt setup, LeRobot teleoperation, ACT/SmolVLA training |
| [`ros2/my_ros2_examples/`](./ros2/my_ros2_examples/) | ROS 2 node examples: pub/sub, services, parameters, launch files |
| [`机器人学/Modern_Robotics/`](./机器人学/Modern_Robotics/) | Study notes for Lynch & Park *Modern Robotics* (Ch 1–6, 9–10) |
| [`机器人学/Control/`](./机器人学/Control/) | Control theory notes: PID, Kalman filter, MPC |

---

## SO-ARM101

A 6-DOF low-cost robotic arm (leader + follower) based on the [SO-ARM101 / LeRobot](https://wiki.seeedstudio.com/cn/lerobot_so100m_new/) platform.

Work covered:

1. **ROS 2 + MoveIt + RViz** — URDF/mesh import, MoveIt config generation, TF debugging, motion planning visualization
2. **Teleoperation & data collection** — LeRobot toolchain for leader-follower teleoperation and camera capture
3. **Imitation learning** — ACT (Action Chunking Transformer) and SmolVLA fine-tuning via LeRobot training scripts
4. **Servo calibration** — PWM center-point tuning (1500 µs = neutral) to avoid joint limit collisions on power-up

See [`soarm101/README.md`](./soarm101/README.md) for details.

---

## ROS 2 Examples

[`ros2/my_ros2_examples/`](./ros2/my_ros2_examples/) — a minimal ROS 2 Python package demonstrating:

- Publisher / subscriber nodes
- Synchronous service server + client
- Parameter declaration and dynamic reconfiguration
- Launch file composition

---

## Robotics Theory Notes

**Modern Robotics** (Lynch & Park, Northwestern):

| Chapter | Topic |
|---|---|
| Ch 1 | Preview — degrees of freedom, configuration space |
| Ch 2 | Configuration space, constraints, task space |
| Ch 3 | Rigid-body motions, SO(3), SE(3), exponential coordinates |
| Ch 4 | Forward kinematics via product of exponentials |
| Ch 5 | Velocity kinematics, space/body Jacobian, statics |
| Ch 6 | Inverse kinematics (numerical, Newton-Raphson) |
| Ch 9 | Trajectory generation |
| Ch 10 | Motion planning |

**Control**: PID, discrete Kalman filter, Model Predictive Control (MPC).

---

## References

- [Modern Robotics — Northwestern](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- [SO-ARM101 / LeRobot Wiki](https://wiki.seeedstudio.com/cn/lerobot_so100m_new/)
