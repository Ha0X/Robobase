import rclpy
from rclpy.node import Node

class ParamDemo(Node):
    def __init__(self):
        super().__init__('param_demo')
        self.declare_parameter('message', 'Hello Param')
        msg = self.get_parameter('message').get_parameter_value().string_value
        self.get_logger().info(f'Parameter value: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = ParamDemo()
    try:
        # This demo only needs to read and print once.
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# 运行时：ros2 run my_ros2_examples param_node --ros-args -p message:="你好世界"

