import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    """Publishes a counted greeting on /chatter_talk twice a second."""

    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, '/chatter_talk', 10)   # type, topic, queue depth
        self.i = 0
        self.create_timer(0.5, self.timer_callback)                            # seconds, function

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main():
    rclpy.init()
    node = Talker()
    try:
        rclpy.spin(node)             # runs the timer until Ctrl+C
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
