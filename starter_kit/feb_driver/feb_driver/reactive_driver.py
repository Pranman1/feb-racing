"""Reactive "follow the gap" driver: the starting point for every member.

Per lidar scan:
  1. clean the scan (inf -> max range, clip)
  2. zero a safety bubble around the nearest return so no target is chosen
     through a gap that passes too close to a wall
  3. aim at the middle of the widest remaining gap
  4. pick a speed from the room ahead and from how hard we are steering
  5. turn the speed target into a throttle with a feed-forward + PI loop

Inputs: lidar and wheel encoders only. ips/odom/tf are barred at race time.
Steering command is normalised [-1, 1] = [-0.5236, 0.5236] rad. Throttle 0 brakes hard.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float32

NS = "/autodrive/roboracer_1/"
QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=10)
WHEEL_RADIUS = 0.059   # m
MAX_STEER = 0.5236     # rad, the simulator's steering limit


class ReactiveDriver(Node):
    def __init__(self, name="reactive_driver"):
        super().__init__(name)
        defaults = dict(bubble=0.40, gap_fov=1.60, range_cap=6.0, steer_gain=0.85, max_speed=2.5, min_speed=0.9,
                        lat_accel=6.0, brake_margin=0.40, decel_limit=6.0, speed_per_throttle=23.0,
                        throttle_kp=0.06, throttle_ki=0.10)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        self.speed = 0.0            # m/s from the encoders
        self.integral = 0.0
        self.last_scan_t = None
        self.encoder_hist = {}      # frame_id -> (t, angle)

        self.pub_throttle = self.create_publisher(Float32, NS + "throttle_command", QOS)
        self.pub_steering = self.create_publisher(Float32, NS + "steering_command", QOS)
        self.create_subscription(LaserScan, NS + "lidar", self.on_scan, QOS)
        for side in ("left", "right"):
            self.create_subscription(JointState, NS + side + "_encoder", self.on_encoder, QOS)
        self.get_logger().info("reactive driver up")

    # ---------------------------------------------------------------- sensing

    def on_encoder(self, msg):
        """Wheel speed from the cumulative wheel angle (rad). Resets on teleport."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        angle = float(msg.position[0])
        last = self.encoder_hist.get(msg.header.frame_id)
        self.encoder_hist[msg.header.frame_id] = (t, angle)
        if last is None or angle < last[1] or t - last[0] < 1e-3:
            return
        v = (angle - last[1]) / (t - last[0]) * WHEEL_RADIUS
        if 0.0 <= v < 25.0:
            self.speed = 0.7 * self.speed + 0.3 * v

    # ---------------------------------------------------------------- driving

    def on_scan(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:  # unstamped scan (replays, tests): fall back to the node clock
            t = self.get_clock().now().nanoseconds * 1e-9
        dt = t - self.last_scan_t if self.last_scan_t else 0.0
        self.last_scan_t = t
        if not 0.0 < dt < 1.0:
            return

        ranges = np.asarray(msg.ranges, dtype=float)
        ranges[~np.isfinite(ranges)] = msg.range_max
        ranges = np.clip(ranges, msg.range_min, self.p["range_cap"])
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        target_angle, clearance = self.choose_gap(ranges, angles)
        steer = float(np.clip(self.p["steer_gain"] * target_angle, -MAX_STEER, MAX_STEER))
        speed_target = self.speed_target(steer, clearance)
        throttle = self.throttle_for(speed_target, dt)
        self.publish(throttle, steer)

    def choose_gap(self, ranges, angles):
        """Angle to aim at and the range straight ahead after the safety bubble."""
        fov = np.abs(angles) <= self.p["gap_fov"]
        nearest = int(np.argmin(np.where(fov, ranges, np.inf)))
        half_angle = math.atan2(self.p["bubble"], max(ranges[nearest], 0.05))
        free = np.where(fov & (np.abs(angles - angles[nearest]) >= half_angle), ranges, 0.0)

        good = np.flatnonzero(free > 0.0)
        if good.size == 0:
            return 0.0, 0.0
        groups = np.split(good, np.flatnonzero(np.diff(good) > 1) + 1)
        gap = max(groups, key=len)
        target = 0.5 * (angles[gap[0]] + angles[gap[-1]])
        ahead = ranges[np.abs(angles) < 0.1]
        return float(target), float(ahead.min()) if ahead.size else 0.0

    def speed_target(self, steer, clearance):
        """Slower when turning hard (lateral acceleration) and when the wall is close."""
        curvature = abs(math.tan(steer)) / 0.324
        v_corner = math.sqrt(self.p["lat_accel"] / curvature) if curvature > 1e-3 else self.p["max_speed"]
        stop_dist = max(clearance - self.p["brake_margin"], 0.0)
        v_clear = math.sqrt(2.0 * self.p["decel_limit"] * stop_dist)
        return float(np.clip(min(v_corner, v_clear), self.p["min_speed"], self.p["max_speed"]))

    def throttle_for(self, v_target, dt):
        """Feed-forward from the steady-state gain plus a PI correction on measured speed."""
        error = v_target - self.speed
        self.integral = float(np.clip(self.integral + error * dt, -0.8, 0.8))
        throttle = v_target / self.p["speed_per_throttle"] + self.p["throttle_kp"] * error + self.p["throttle_ki"] * self.integral
        return float(np.clip(throttle, 0.0, 1.0))

    def publish(self, throttle, steer):
        self.pub_throttle.publish(Float32(data=throttle))
        self.pub_steering.publish(Float32(data=steer / MAX_STEER))


def spin(node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()


def main():
    rclpy.init()
    spin(ReactiveDriver())


if __name__ == "__main__":
    main()
