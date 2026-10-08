import sys

import rclpy
from rclpy.node import Node
from turtle_patrol_interface.srv import Patrol


class PatrolClient(Node):
    """Calls /turtle1/patrol once and prints the response.
    ros2 run turtle_patrol patrol_client [vel] [omega]"""

    def __init__(self):
        super().__init__('patrol_client')
        self.client = self.create_client(Patrol, '/turtle1/patrol')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /turtle1/patrol ...')

    def send_request(self, vel, omega):
        request = Patrol.Request()
        request.vel = vel
        request.omega = omega
        future = self.client.call_async(request)                     # send, then wait for the answer
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)                  # drop --ros-args etc.
    vel = float(args[1]) if len(args) > 1 else 2.0
    omega = float(args[2]) if len(args) > 2 else 1.0
    rclpy.init()
    node = PatrolClient()
    response = node.send_request(vel, omega)
    node.get_logger().info('response: %s' % response)
    rclpy.shutdown()
