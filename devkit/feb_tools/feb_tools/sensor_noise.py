"""Make the simulator's perfect sensors look like real ones.

The bridge is launched with its lidar and IMU remapped to /autodrive/raw/...; this node
republishes them on the standard topics with Gaussian range noise, per-beam dropouts,
whole-scan drops and an extra delay. Parameters:

    range_sigma   m, standard deviation added to every lidar range     (default 0.02)
    beam_dropout  probability a beam becomes NaN                        (default 0.02)
    scan_dropout  probability a whole scan is lost                      (default 0.0)
    latency       s, extra delay before republishing                    (default 0.0)
    imu_sigma     rad/s and m/s^2, noise on angular velocity / acceleration (default 0.02)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class SensorNoise(Node):
    def __init__(self):
        super().__init__("sensor_noise")
        for key, value in dict(range_sigma=0.02, beam_dropout=0.02, scan_dropout=0.0, latency=0.0, imu_sigma=0.02).items():
            self.declare_parameter(key, value)
        self.p = lambda k: self.get_parameter(k).value
        self.rng = np.random.default_rng()
        self.pub_scan = self.create_publisher(LaserScan, "/autodrive/roboracer_1/lidar", QOS)
        self.pub_imu = self.create_publisher(Imu, "/autodrive/roboracer_1/imu", QOS)
        self.create_subscription(LaserScan, "/autodrive/raw/lidar", self.on_scan, QOS)
        self.create_subscription(Imu, "/autodrive/raw/imu", self.on_imu, QOS)
        self.get_logger().info("sensor noise: sigma %.3f m, beam dropout %.2f, scan dropout %.2f, latency %.3f s"
                               % (self.p("range_sigma"), self.p("beam_dropout"), self.p("scan_dropout"), self.p("latency")))

    def on_scan(self, msg):
        if self.rng.random() < self.p("scan_dropout"):
            return
        r = np.asarray(msg.ranges, dtype=np.float32)
        r += self.rng.normal(0.0, self.p("range_sigma"), r.shape).astype(np.float32)
        r[self.rng.random(r.shape) < self.p("beam_dropout")] = np.nan
        msg.ranges = r.tolist()
        self.later(lambda: self.pub_scan.publish(msg))

    def on_imu(self, msg):
        s = self.p("imu_sigma")
        for v in (msg.angular_velocity, msg.linear_acceleration):
            v.x += self.rng.normal(0.0, s)
            v.y += self.rng.normal(0.0, s)
            v.z += self.rng.normal(0.0, s)
        self.later(lambda: self.pub_imu.publish(msg))

    def later(self, fn):
        delay = self.p("latency")
        if delay <= 0.0:
            fn()
            return
        timer = None

        def fire():
            timer.cancel()
            fn()
        timer = self.create_timer(delay, fire)


def main():
    rclpy.init()
    node = SensorNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
