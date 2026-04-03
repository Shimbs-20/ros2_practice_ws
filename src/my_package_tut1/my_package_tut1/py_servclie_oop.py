import rclpy
from  rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial


class Addtwocallback(Node):

    def __init__(self):
        super().__init__('Addtwocallback')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

    def add_two_ints_callback(self, a, b):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self._logger.info('service not available, waiting again...')

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        """ When you give the cashier your order (your req), they don't make you stand at the register while the food cooks. They give you a buzzer/pager.
        In Python, that pager is the future object.
        call_async() means "Call the server asynchronously." It sends the request over the Zenoh network and instantly moves to the next line of code without freezing your computer.
        The future object is basically an empty box. It promises: "I don't have your data right now, but the second the server replies, I will put the result inside this box."""
        future = self.client.call_async(request) #future object here doesn't return anything but the result of the request

        future.add_done_callback(partial(self.callback_call_add_two_clients, request=request))
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

    def callback_call_add_two_clients(self,future, request):
        response = future.result()
        self.get_logger().info(
            'Result of add_two_ints: for %d + %d = %d' %
            (request.a, request.b, response.sum)
        )

def main(args = None):
    rclpy.init(args=args)

    node = Addtwocallback()
    node.add_two_ints_callback(41,1)

    rclpy.spin(node)

    rclpy.shutdown()
