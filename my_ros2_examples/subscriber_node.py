import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.declare_parameter("topic", "chatter")
        self.topic = str(self.get_parameter("topic").value)

        self.subscription = self.create_subscription(
            String,
            self.topic,
            self.listener_callback,
            10)
        self.get_logger().info(f"Listening on '{self.topic}'")

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
