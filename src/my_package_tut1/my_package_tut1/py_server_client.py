import rclpy
from  rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client")

    client = node.create_client(AddTwoInts, 'add_two_ints')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    req = AddTwoInts.Request()
    req.a = 41
    req.b = 1
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    future = client.call_async(req) #future object here doesn't return anything but the result of the request
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    node.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (req.a, req.b, result.sum))

    node.destroy_node()
    rclpy.shutdown()



