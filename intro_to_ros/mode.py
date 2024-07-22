import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode


class ModeNode(Node):
    def __init__(self):
        super().__init__('set_mode_client')
        self.client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /mavros/set_mode not available, waiting again...')
        self.request = SetMode.Request()

    def set_mode(self):
        self.request.custom_mode = 'MANUAL'
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('Mode set to MANUAL successfully')
            else:
                self.get_logger().info('Failed to set mode to MANUAL')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = ModeNode()
    try:
        node.set_mode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
