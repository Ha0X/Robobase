import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.declare_parameter("service_name", "add_two_ints")
        self.declare_parameter("a", 10)
        self.declare_parameter("b", 20)

        self.service_name = str(self.get_parameter("service_name").value)
        self.a = int(self.get_parameter("a").value)
        self.b = int(self.get_parameter("b").value)

        self.client = self.create_client(AddTwoInts, self.service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service '{self.service_name}' ...")

        self.request = AddTwoInts.Request()
        self.request.a = int(self.a)
        self.request.b = int(self.b)

        self.get_logger().info(
            f"Calling '{self.service_name}' with a={self.request.a}, b={self.request.b}"
        )
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    node = AddClient()
    try:
        rclpy.spin_until_future_complete(node, node.future)
        if node.future.result():
            node.get_logger().info(f"Result = {node.future.result().sum}")
        else:
            node.get_logger().error("Service call failed")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
