# MoveIt / RViz 




## 目标

- 在 RViz 里看到 SO-101 模型
- MotionPlanning 插件能读到 `robot_description` + `robot_description_semantic`
- 能在 RViz 中进行规划（plan）并看到轨迹可视化（display_planned_path）

## 关键组成

### `robot_state_publisher`：负责持续发布 TF

- **为什么要它**：RViz 展示 RobotModel、MoveIt 展示规划结果，本质上都依赖 TF tree。
- **没有它的典型现象**：RViz 状态栏或 Displays 里反复提示 **“No tf data”**。
- 需要在 `demo.launch.py` 里显式启动`robot_state_publisher`，并把 URDF 作为参数传进去。

### `joint_state_publisher(_gui)`：负责提供 joint_states

- **为什么要它**：`robot_state_publisher` 需要 joint angle 才能把 URDF 的关节链“算成 TF”。
- *`notes/launch_debug_notes.txt` 里写了两种修复方式：
手动启动 `joint_state_publisher_gui`/ 把它写进 launch

一个最小化 display 启动：`so101_ws/src/soarm101_description/launch/display.launch.py`

### `move_group`：MoveIt 的核心节点（规划 + 发布规划场景）

- **作用**：提供规划服务、维护 planning scene、发布可视化需要的 topic。
在 `demo.launch.py` 里启动了 `moveit_ros_move_group/move_group`，并特别打开了：
  - `publish_robot_description: True`
  - `publish_robot_description_semantic: True`

这两个开关的意义是：让 RViz/其它节点能通过 topic 拿到描述信息。

## RViz 配置

在 `so101_ws/src/soarm101_moveit_config/config/moveit.rviz` 里能看到几个关键设置：

- **Fixed Frame**：`base_link`
- **MotionPlanning 插件**
  - `Robot Description`: `robot_description`
  - `Planning Scene Topic`: `monitored_planning_scene`
  - `Trajectory Topic`: `display_planned_path`


## 规划组（Planning Group）

在 SRDF（`so101.srdf`）中定义了：

- `group arm`：`shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`
- `group gripper`：`gripper`
- `end_effector gripper`：挂在 `wrist_link` 上
- `virtual_joint world_joint`：`world -> base_link` 固定

这是 MoveIt “知道要对哪些关节做规划”的前提。

## 遇到的问题+处理

### 1：RViz “No tf data”

- **原因**：没有 TF 发布（缺 `robot_state_publisher`），或没有 joint_states（缺 `joint_state_publisher`/真实硬件驱动）。
- **修复**：
  - `demo.launch.py` 里显式启动 `robot_state_publisher`
  - 在笔记里记录了 `joint_state_publisher_gui` 的启动方式

### 2：RViz/MoveIt 启动顺序问题（先开 RViz 再开 move_group）

在 `demo.launch.py` 里把 RViz 延迟了 2 秒：

- 目的：避免 RViz 过早启动导致 MotionPlanning 插件初始化时订阅不到 `robot_description` / planning_scene。


### 3：TF 导出“只剩一个 frame”

`tf/frames_2025-10-08_22.57.18.gv` 里只显示了 `gripper_link -> gripper_frame_link`。

这意味着当时 TF tree **没有完整发布**

导出文件：`tf/frames_2025-10-08_22.57.18.gv`

## 补充

### URDF vs SRDF

- **URDF**：更偏“物理结构与几何”
  - link/joint 的拓扑、关节轴、限位（有些也能在 MoveIt 里被 joint_limits.yaml 覆盖）、惯性、可视/碰撞 mesh 等。
- **SRDF**：更偏“给 MoveIt 的语义标注”
  - 哪些关节组成一个规划组（planning group）
  - end effector 挂在哪个 link 上
  - world/base 的虚拟关节（比如 `world -> base_link` 固定）
  - 禁碰对（让规划更稳定/更快）


### TF / joint_states / robot_state_publisher：三者关系

- `joint_states`：关节角度的“原始状态输入”
- `robot_state_publisher`：读取 URDF + joint_states，计算并持续发布 TF（把“关节角”变成“各 link 的位姿关系”）
- RViz/MoveIt：本质上是消费 TF（再叠加 planning scene、轨迹等可视化）

所以排错顺序一般是：
1) 先确认有没有 `joint_states`
2) 再确认 TF tree 是否完整、Fixed Frame 是否选对
3) 最后再看 MoveIt 的 planning scene / robot_description 有没有跟上

### RViz MotionPlanning 插件在“等”哪些东西

MotionPlanning 插件通常会用到：
- `robot_description` / `robot_description_semantic`：模型与语义（规划组、末端等）
- `monitored_planning_scene`：规划场景（自碰撞、障碍物、当前状态等）
- `display_planned_path`：规划出来的轨迹显示

你在 `moveit.rviz` 里设置的 **Fixed Frame = base_link** 也很关键：Fixed Frame 不对，很多显示会“漂”或者直接不显示。

### plan vs execute：为什么“能规划”不等于“能动起来”

- **Plan**：只要 MoveIt 拿到了模型 + 当前状态（哪怕来自 `joint_state_publisher_gui`），通常就能算路径并显示。
- **Execute**：需要控制器/硬件驱动（action server）真的能接收 `FollowJointTrajectory` 或 gripper command。

你这份工程里 controller 配置是有的（MoveIt simple controller manager / FollowJointTrajectory / GripperCommand），但如果没接真机或没起 ros2_control，那通常就是“能 plan、不能 execute”——这在探索阶段完全正常。

### 常用排查命令

- **看有哪些 topic**：

```bash
ros2 topic list
```

- **看 TF 有没有在刷**：

```bash
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
```

- **看有没有 joint_states**：

```bash
ros2 topic echo /joint_states --once
```

- **导出 TF tree**（你留下的 gv/pdf 就是这类工具的产物）：

```bash
ros2 run tf2_tools view_frames
```

## 快速跳转（相关文件一览）

- **MoveIt demo 启动链路**：`so101_ws/src/soarm101_moveit_config/launch/demo.launch.py`
- **MoveIt SRDF / 规划组**：`so101_ws/src/soarm101_moveit_config/config/so101.srdf`
- **RViz 配置**：`so101_ws/src/soarm101_moveit_config/config/moveit.rviz`
- **URDF（标定版）**：`so101_ws/src/soarm101_description/urdf/so101_new_calib.urdf`
- **排错碎片笔记**：`notes/launch_debug_notes.txt`
- **TF 导出快照**：`tf/frames_2025-10-08_22.57.18.gv` / `tf/frames_2025-10-08_22.57.18.pdf`


