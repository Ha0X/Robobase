# pi × SOARM101微调




### 环境
```
sudo apt-get install cmake build-essential python3-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev
conda create -y -n lerobot python=3.10
conda activate lerobot
conda install ffmpeg -c conda-forge

```
```
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e .
pip install lerobot
pip install 'lerobot[all]' 
pip install -e ".[pi]"
policy.type=pi05
```

### 命令
```
python src/lerobot/scripts/lerobot_train.py\
    --dataset.repo_id=your_dataset \
    --policy.type=pi05 \
    --output_dir=./outputs/pi05_training \
    --job_name=pi05_training \
    --policy.repo_id=your_repo_id \
    --policy.pretrained_path=lerobot/pi05_base \
    --policy.compile_model=true \
    --policy.gradient_checkpointing=true \
    --wandb.enable=true \
    --policy.dtype=bfloat16 \
    --policy.freeze_vision_encoder=false \
    --policy.train_expert_only=false \
    --steps=3000 \
    --policy.device=cuda \
    --batch_size=32
```

### 数据集
```
lerobot-record \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=my_awesome_follower_arm \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"}, side: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30, fourcc: "MJPG"}}" \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=my_awesome_leader_arm \
    --display_data=true \
    --dataset.repo_id=seeedstudio123/test \
    --dataset.num_episodes=5 \
    --dataset.single_task="Grab the black cube" \
    --dataset.push_to_hub=false \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=30 
```



### 开环评估（open-loop）
只在离线数据上“看模型会怎么做”，但不把模型的输出动作再喂回环境/机器人去产生下一帧观测。
- `scripts/eval_policy.py` 在数据集上做离线可视化评估
- 传 `--embodiment_tag new_embodiment`、`--model_path`、`--dataset_path`、`--data_config` 等参数

### 部署到真机

```
lerobot-record  \
  --robot.type=so100_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.cameras="{ up: {type: opencv, index_or_path: /dev/video10, width: 640, height: 480, fps: 30}, side: {type: intelrealsense, serial_number_or_name: 233522074606, width: 640, height: 480, fps: 30}}" \
  --robot.id=my_awesome_follower_arm \
  --display_data=false \
  --dataset.repo_id=${HF_USER}/eval_so100 \
  --dataset.single_task="Put lego brick into the transparent box" \
  # <- Teleop optional if you want to teleoperate in between episodes \
  # --teleop.type=so100_leader \
  # --teleop.port=/dev/ttyACM0 \
  # --teleop.id=my_awesome_leader_arm \
  --policy.path=${HF_USER}/my_policy
```




