import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddServer(Node):
    def __init__(self):
        super().__init__('add_server')
        self.declare_parameter("service_name", "add_two_ints")
        self.service_name = str(self.get_parameter("service_name").value)
        self.srv = self.create_service(AddTwoInts, self.service_name, self.callback)
        self.get_logger().info(f"Service ready: '{self.service_name}'")

    def callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"Adding {request.a} + {request.b} = {response.sum}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
