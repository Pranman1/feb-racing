"""Reactive "follow the gap" driver: the starting point for every member.

Per lidar scan:
  1. clean the scan (inf -> max range, clip)
  2. zero a safety bubble around the nearest return so no target is chosen
     through a gap that passes too close to a wall
  3. aim into the widest remaining gap: a blend of its deepest beam and its middle
  4. pick a speed from the room ahead (a low percentile of the ranges inside the car's
     own corridor) and from how hard we are steering (a slow steering estimate)
  5. turn the speed target into a throttle: feed-forward plus a bounded trim

Everything that can jump beam-to-beam is rate-limited or low-passed, because the bridge
can run as slow as 10 Hz and the simulator brakes hard at throttle 0: a plain PI speed
loop surges and stops in a 0.8 s cycle, and raw gap targets weave full lock. The
command is feed-forward (v / speed_per_throttle) plus a trim clipped to half of it,
so it never reaches zero while speed is wanted.

Inputs: lidar and wheel encoders only. ips/odom/tf are barred at race time.
Steering command is normalised [-1, 1] = [-0.5236, 0.5236] rad. Throttle 0 is a hard brake.
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
WHEELBASE = 0.324      # m
HALF_WIDTH = 0.135     # m
WHEEL_RADIUS = 0.059   # m
MAX_STEER = 0.5236     # rad, the simulator's steering limit
MAX_STEER_RATE = 3.2   # rad/s, the simulator's steering actuator


class ReactiveDriver(Node):
    def __init__(self, name="reactive_driver"):
        super().__init__(name)
        defaults = dict(bubble=0.40, gap_fov=1.60, range_cap=6.0, steer_gain=0.85, deep_weight=0.6,
                        max_speed=2.5, min_speed=0.9, accel_limit=4.0, decel_limit=6.0, lat_accel=6.0,
                        brake_margin=0.40, clear_pct=3.0, clear_tau=0.30, curv_tau=0.35, goal_tau=0.30,
                        speed_per_throttle=23.0, throttle_kp=0.02, throttle_ki=0.03, throttle_slew=0.8,
                        speed_window=0.25)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        self.speed = 0.0            # m/s from the encoders
        self.encoders = {}          # frame_id -> [(t, angle), ...] over the last speed_window
        self.steer = 0.0            # steering command, rad
        self.steer_slow = 0.0       # slow steering estimate for the cornering speed
        self.clearance = None       # low-passed range ahead
        self.v_goal = 0.0           # low-passed speed goal
        self.speed_cmd = 0.0        # accel/decel-limited speed target
        self.integral = 0.0
        self.throttle = 0.0
        self.last_scan_t = None

        self.pub_throttle = self.create_publisher(Float32, NS + "throttle_command", QOS)
        self.pub_steering = self.create_publisher(Float32, NS + "steering_command", QOS)
        self.create_subscription(LaserScan, NS + "lidar", self.on_scan, QOS)
        for side in ("left", "right"):
            self.create_subscription(JointState, NS + side + "_encoder", self.on_encoder, QOS)
        self.get_logger().info("reactive driver up")

    # ---------------------------------------------------------------- sensing

    def on_encoder(self, msg):
        """Wheel speed over a short window of samples (works at 10 Hz); the history is
        dropped when the angle jumps back, which happens on a collision respawn."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        angle = float(msg.position[0])
        hist = self.encoders.setdefault(msg.header.frame_id, [])
        if hist and angle < hist[-1][1]:
            hist.clear()
            self.integral = 0.0
        hist.append((t, angle))
        while len(hist) > 2 and t - hist[0][0] > self.p["speed_window"]:
            hist.pop(0)
        if len(hist) < 2 or t - hist[0][0] < 1e-3:
            return
        v = (angle - hist[0][1]) / (t - hist[0][0]) * WHEEL_RADIUS
        if 0.0 <= v < 25.0:
            self.speed = 0.5 * self.speed + 0.5 * v

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
        ranges = np.clip(ranges, msg.range_min, msg.range_max)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        target = self.gap_target(np.minimum(ranges, self.p["range_cap"]), angles)
        wanted = float(np.clip(self.p["steer_gain"] * target, -MAX_STEER, MAX_STEER))
        self.steer += float(np.clip(wanted - self.steer, -MAX_STEER_RATE * dt, MAX_STEER_RATE * dt))

        ahead = self.room_ahead(ranges, angles, dt)
        self.steer_slow += (self.steer - self.steer_slow) * lowpass(dt, self.p["curv_tau"])
        curvature = abs(math.tan(self.steer_slow)) / WHEELBASE
        v_corner = math.sqrt(self.p["lat_accel"] / curvature) if curvature > 1e-3 else self.p["max_speed"]
        v_brake = math.sqrt(2.0 * self.p["decel_limit"] * max(0.0, ahead - self.p["brake_margin"]))
        goal = max(self.p["min_speed"], min(self.p["max_speed"], v_corner, v_brake))
        self.v_goal += (goal - self.v_goal) * lowpass(dt, self.p["goal_tau"])
        self.speed_cmd += float(np.clip(self.v_goal - self.speed_cmd, -self.p["decel_limit"] * dt, self.p["accel_limit"] * dt))

        self.throttle = self.throttle_for(self.speed_cmd, dt)
        self.publish(self.throttle, self.steer)

    def gap_target(self, ranges, angles):
        """Bearing to aim at: inside the widest gap left after the safety bubble, between
        its deepest beam (deep_weight) and its middle."""
        fov = np.abs(angles) <= self.p["gap_fov"]
        nearest = int(np.argmin(np.where(fov, ranges, np.inf)))
        half_angle = math.atan2(self.p["bubble"], max(ranges[nearest], 0.05))
        free = np.where(fov & (np.abs(angles - angles[nearest]) >= half_angle), ranges, 0.0)
        good = np.flatnonzero(free > 0.0)
        if good.size == 0:
            return 0.0
        groups = np.split(good, np.flatnonzero(np.diff(good) > 1) + 1)
        gap = max(groups, key=len)
        deep = gap[int(np.argmax(free[gap]))]
        mid = gap[len(gap) // 2]
        return self.p["deep_weight"] * angles[deep] + (1.0 - self.p["deep_weight"]) * angles[mid]

    def room_ahead(self, ranges, angles, dt):
        """Low percentile of the forward range inside the car's own corridor, low-passed;
        a genuinely close obstacle passes straight through so braking is immediate."""
        x, y = ranges * np.cos(angles), ranges * np.sin(angles)
        corridor = (np.abs(y) < HALF_WIDTH + 0.10) & (x > 0.0)
        raw = float(np.percentile(x[corridor], self.p["clear_pct"])) if corridor.any() else self.p["range_cap"]
        self.clearance = raw if self.clearance is None else self.clearance + (raw - self.clearance) * lowpass(dt, self.p["clear_tau"])
        return min(raw, self.clearance) if raw < self.p["brake_margin"] + 0.3 else self.clearance

    def throttle_for(self, v_target, dt):
        """Feed-forward from the steady-state gain plus a trim bounded to half of it, so the
        command never drops to zero (a hard brake) while speed is wanted; slew-limited."""
        ff = v_target / self.p["speed_per_throttle"]
        error = v_target - self.speed
        self.integral = float(np.clip(self.integral + error * dt, -0.8, 0.8))
        trim = float(np.clip(self.p["throttle_kp"] * error + self.p["throttle_ki"] * self.integral, -0.5 * ff, 0.5 * ff))
        wanted = float(np.clip(ff + trim, 0.0, 1.0))
        slew = self.p["throttle_slew"] * dt
        return float(np.clip(wanted, self.throttle - slew, self.throttle + slew))

    def publish(self, throttle, steer):
        self.pub_throttle.publish(Float32(data=throttle))
        self.pub_steering.publish(Float32(data=steer / MAX_STEER))


def lowpass(dt, tau):
    """Gain of a first-order low-pass with time constant tau at step dt."""
    return 1.0 - math.exp(-dt / max(tau, 1e-3))


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
