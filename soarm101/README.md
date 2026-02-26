
## soarm101（个人记录）



## TL;DR


1. **SO-101（ROS 2 + MoveIt + RViz）**
   - 搭建 ROS2 工作区，导入/维护机器人描述（URDF+mesh），生成 MoveIt 配置并在 RViz 里完成模型加载与路径规划可视化排错（TF / robot_state_publisher / joint_state_publisher）。
   - 相关内容主要在：`so101_ws/src/soarm101_description/`、`so101_ws/src/soarm101_moveit_config/`、`tf/`、`notes/launch_debug_notes.txt`
2. **使用 LeRobot 的现成遥操作做数采**
   - 复用 LeRobot 的工具链进行遥操作与相机采集验证，为后续数据集做准备。
   - 相关内容主要在：`lerobot/outputs/captured_images/`
3. **ACT / SmolVLA**
   - 按照 LeRobot 的训练入口运行 ACT/SmolVLA
4. **VLA 微调**


## Keys

### 机械臂与标定

- **自由度**：Base/Shoulder Pan、Shoulder Lift、Elbow Flex、Wrist Flex、Wrist Roll、Gripper
- **主臂/从臂**：leader arm 与 follower arm 的 teleop 设定
- **舵机中位校准**：PWM 周期 20ms，高电平 1–2ms；1.5ms（1500µs）代表中位；不校准可能上电撞限位

### 数采

- **双摄像头建议**：top view（俯视）+ side view（侧视）；桌面空间/画质会影响数据质量

### MoveIt + RViz（路径规划/可视化）

- **目标**：MoveIt + RViz 能正常加载 SO-101，在 RViz 里可 plan/可视化
（详见 `notes/moveit_rviz.md` 和 `notes/launch_debug_notes.txt`）

### 模仿学习：ACT / SmolVLA


ACT（Action Chunking Transformer）：CVAE + Transformer；每步预测未来动作块并融合重叠段。

```bash
cd lerobot && python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/il_gym0 \
  --policy.type=act \
  --output_dir=outputs/train/il_sim_test0 \
  --job_name=il_sim_test \
  --policy.device=cuda \
  --wandb.enable=true
```

SmolVLA：更小模型，推理更快；action chunk / 异步

```bash
cd lerobot && python lerobot/scripts/train.py \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=${HF_USER}/mydataset \
  --batch_size=64 \
  --steps=20000 \
  --output_dir=outputs/train/my_smolvla \
  --job_name=my_smolvla_training \
  --policy.device=cuda \
  --wandb.enable=true
```
