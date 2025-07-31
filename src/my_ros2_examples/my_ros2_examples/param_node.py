import rclpy
from rclpy.node import Node

class ParamDemo(Node):
    def __init__(self):
        super().__init__('param_demo')
        self.declare_parameter('message', 'Hello Param')
        msg = self.get_parameter('message').get_parameter_value().string_value
        self.get_logger().info(f'Parameter value: {msg}')

def main():
    rclpy.init()
    node = ParamDemo()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


# 运行时：ros2 run my_ros2_examples param_node --ros-args -p message:="你好世界"

