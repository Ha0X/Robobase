# 🤖 ROS 2 示例工程 - Python 节点演示

本项目是一个完整的 ROS 2 Python 包示例，包含以下核心功能：

- 📢 发布者（Publisher）
- 📥 订阅者（Subscriber）
- 💬 服务端（Service Server）
- 📩 客户端（Service Client）
- ⚙️ 参数读取与设置（Parameters）

---

## 📁 工程结构

ros2_examples_ws/
├── src/
│ └── my_ros2_examples/
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ └── my_ros2_examples/
│ ├── publisher_node.py
│ ├── subscriber_node.py
│ ├── service_server.py
│ ├── service_client.py
│ ├── param_node.py
│ └── init.py
├── .gitignore
└── README.md


---

## 🛠️ 构建步骤

```bash
cd ~/ros2_examples_ws
colcon build
source install/setup.bash

运行节点
发布者 / 订阅者：
ros2 run my_ros2_examples talker
ros2 run my_ros2_examples listener
服务端 / 客户端：
ros2 run my_ros2_examples add_server
ros2 run my_ros2_examples add_client
参数节点：
ros2 run my_ros2_examples param_node --ros-args -p message:="你好，ROS 2"


---

### ✅ 如何保存

在工作区根目录下执行：

```bash
cd ~/ros2_examples_ws
nano README.md
