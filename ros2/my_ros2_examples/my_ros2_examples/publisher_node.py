import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.declare_parameter("topic", "chatter")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("message_prefix", "Hello ROS 2!")

        self.topic = str(self.get_parameter("topic").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.message_prefix = str(self.get_parameter("message_prefix").value)

        self.publisher = self.create_publisher(String, self.topic, 10)

        hz = self.publish_hz if self.publish_hz > 0.0 else 1.0
        timer_period = 1.0 / hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.get_logger().info(
            f"Publishing to '{self.topic}' @ {hz:.3f} Hz, prefix='{self.message_prefix}'"
        )

    def timer_callback(self):
        msg = String()
        msg.data = f"{self.message_prefix} Count: {self.count}"
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
