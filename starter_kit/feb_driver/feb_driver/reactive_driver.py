"""Reactive "follow the gap" driver: the starting point for every member.

Per lidar scan:
  1. clean the scan (inf -> max range, clip)
  2. zero a safety bubble around the nearest return so no target is chosen
     through a gap that passes too close to a wall
  3. aim at the middle of the widest remaining gap
  4. pick a speed from the room ahead and from how hard we are steering
  5. turn the speed target into a throttle: feed-forward plus a bounded trim

Throttle 0 is a hard brake in this simulator, so a plain PI speed loop that cuts to zero
surges and stops in a 0.8 s cycle. The command is feed-forward (v / speed_per_throttle)
plus a trim clipped to half of it, so it never reaches zero while speed is wanted; the
clearance, steering and speed target are low-passed and the throttle is slew-limited.

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
                        throttle_kp=0.02, throttle_ki=0.03, steer_tau=0.25, target_tau=0.30, throttle_slew=0.8)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        self.speed = 0.0            # m/s from the encoders
        self.integral = 0.0
        self.steer = 0.0            # low-passed steering command
        self.clearance = 0.0        # low-passed range ahead
        self.v_target = 0.0         # low-passed speed target
        self.throttle = 0.0
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
        a = min(1.0, dt / self.p["steer_tau"])
        self.steer += (float(np.clip(self.p["steer_gain"] * target_angle, -MAX_STEER, MAX_STEER)) - self.steer) * a
        self.clearance += (clearance - self.clearance) * min(1.0, dt / self.p["target_tau"])
        self.v_target += (self.speed_target(self.steer, self.clearance) - self.v_target) * min(1.0, dt / self.p["target_tau"])
        self.throttle = self.throttle_for(self.v_target, dt)
        self.publish(self.throttle, self.steer)

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
        """Feed-forward from the steady-state gain plus a trim bounded to half of it, so the
        command never drops to zero (a hard brake) while speed is wanted; slew-limited."""
        ff = v_target / self.p["speed_per_throttle"]
        error = v_target - self.speed
        self.integral = float(np.clip(self.integral + error * dt, -0.8, 0.8))
        trim = float(np.clip(self.p["throttle_kp"] * error + self.p["throttle_ki"] * self.integral, -0.5 * ff, 0.5 * ff))
        wanted = float(np.clip(ff + trim, 0.0, 1.0))
        slew = self.p["throttle_slew"] * dt
        return float(np.clip(wanted, self.throttle - slew, self.throttle + slew)) if self.throttle > 0.0 else wanted

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
