## 🤖 ROS 2 (Humble) Python 学习示例：Publisher / Subscriber / Service / Parameters

这个仓库是我在 **Ubuntu 22.04 + ROS 2 Humble** 下做的学习笔记工程，包含：

- 发布者 / 订阅者（`std_msgs/String`）
- 服务端 / 客户端（`example_interfaces/AddTwoInts`）
- 参数读取示例（`--ros-args -p ...`）
- 一键启动 `launch/`（talker+listener、server+client）

## 📁 目录结构（该包位于知识库仓库的 ros2/ 下）

`my_ros2_examples` 是本知识库仓库中的一个 ROS2 包，位于：`ros2/my_ros2_examples/`。
推荐把整个知识库仓库 clone 到你的 colcon 工作区 `src/` 下，然后在工作区根目录 `colcon build`。

```
ros2_ws/
└── src/
    └── robobase/                        # git clone 的知识库仓库
        ├── README.md
        ├── ros2/
        │   └── my_ros2_examples/        # 本 ROS2 包目录（package.xml/setup.py/...）
        └── 机器人学/                     # Markdown 笔记
```

## 🧰 环境准备（Ubuntu 22.04 + Humble）

确保你已经安装 ROS 2 Humble，并在新终端里先 source：

```bash
source /opt/ros/humble/setup.bash
```

（可选）安装依赖（在工作区根目录执行 `rosdep` 更常见）：

```bash
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update
```

## 🛠️ 构建（推荐用工作区）

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your_repo_url_or_path> robotics_basics

cd ~/ros2_ws
rosdep install -r --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## ▶️ 运行（ros2 run）

### 1) 发布者 / 订阅者

终端 A：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run my_ros2_examples talker
```

终端 B：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run my_ros2_examples listener
```

可配置参数（示例）：

```bash
ros2 run my_ros2_examples talker --ros-args -p topic:=chatter -p publish_hz:=2.0 -p message_prefix:="Hi"
ros2 run my_ros2_examples listener --ros-args -p topic:=chatter
```

### 2) 服务端 / 客户端

终端 A：

```bash
ros2 run my_ros2_examples add_server
```

终端 B：

```bash
ros2 run my_ros2_examples add_client
```

可配置参数（示例）：

```bash
ros2 run my_ros2_examples add_server --ros-args -p service_name:=add_two_ints
ros2 run my_ros2_examples add_client --ros-args -p service_name:=add_two_ints -p a:=10 -p b:=20
```

### 3) 参数读取示例

```bash
ros2 run my_ros2_examples param_node --ros-args -p message:="你好，ROS 2"
```

## 🚀 一键启动（ros2 launch）

```bash
ros2 launch my_ros2_examples talker_listener.launch.py
ros2 launch my_ros2_examples add_two_ints.launch.py
```

也支持传参（示例）：

```bash
ros2 launch my_ros2_examples talker_listener.launch.py topic:=chatter publish_hz:=5.0 message_prefix:="Hello"
ros2 launch my_ros2_examples add_two_ints.launch.py service_name:=add_two_ints a:=3 b:=4
```

## 🧩 常见问题（FAQ）

- **找不到包 / `ros2 run` 报错**：通常是忘了 `source ~/ros2_ws/install/setup.bash`。\n+- **`colcon build` 找不到包**：请在工作区根目录（例如 `~/ros2_ws`）执行 `colcon build`。\n+- **依赖缺失**：先执行 `rosdep install -r --from-paths src --ignore-src -y`。\n+
