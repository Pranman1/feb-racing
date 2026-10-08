import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtle_patrol_interface.srv import Patrol


class PatrolServer(Node):
    """Offers /turtle1/patrol. A request sets the speed and turn rate; a timer then publishes
    that Twist on /turtle1/cmd_vel ten times a second, so the turtle drives in a circle."""

    def __init__(self):
        super().__init__('patrol_server')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cmd = Twist()                                            # zero until the first request
        self.create_service(Patrol, '/turtle1/patrol', self.patrol_callback)
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('patrol server ready: ros2 service call /turtle1/patrol ...')

    def patrol_callback(self, request, response):
        self.cmd = Twist()
        self.cmd.linear.x = request.vel
        self.cmd.angular.z = request.omega
        self.get_logger().info('patrol: vel %.2f omega %.2f' % (request.vel, request.omega))
        response.cmd = self.cmd                                       # the response is the Twist we will publish
        return response

    def timer_callback(self):
        self.publisher.publish(self.cmd)


def main():
    rclpy.init()
    node = PatrolServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
