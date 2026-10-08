import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    """Prints every message that arrives on /chatter_talk."""

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(String, '/chatter_talk', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main():
    rclpy.init()
    node = Listener()
    try:
        rclpy.spin(node)             # waits for messages until Ctrl+C
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
