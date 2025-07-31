import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.request = AddTwoInts.Request()
        self.request.a = 10
        self.request.b = 20

        self.future = self.client.call_async(self.request)

def main():
    rclpy.init()
    node = AddClient()
    rclpy.spin_until_future_complete(node, node.future)
    if node.future.result():
        node.get_logger().info(f"Result = {node.future.result().sum}")
    else:
        node.get_logger().error("Service call failed")
    rclpy.shutdown()
