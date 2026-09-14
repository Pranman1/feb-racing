"""The house driver: a competent, predictable opponent that follows the track's centreline.

It knows the map (track.json) and uses the simulator's position and orientation, which
competitors may not do at race time. That is the point: it is the rabbit, not an entry.

Steering: MAP-style pursuit (L1 guidance plus a curvature feed-forward, capped at the tyre's
slip peak, lightly low-passed), from the IROS 2026 league stack in ~/roboracer.
Speed: a profile along the path from curvature with a braking backward pass, tracked by
feed-forward plus a trim bounded to half of it and slew-limited. Throttle 0 is a hard brake
in this simulator and the bridge can run as slow as 10 Hz, so the command must never fall
to zero while speed is wanted: no PI loop that can cut out, no overspeed brake regime.

    ros2 run feb_tools house_driver --ros-args -p track:=/path/to/track/folder -p max_speed:=2.5
"""
import json
import math
import pathlib

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float32

NS = "/autodrive/roboracer_1/"
QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=10)
WHEELBASE = 0.324
WHEEL_RADIUS = 0.059
MAX_STEER = 0.5236
SLIP_PEAK = 0.30          # rad; the tyre's lateral grip peaks here, steering past it turns less
STEER_DELAY = 0.25        # s; command-to-wheel lag measured on this car


class HouseDriver(Node):
    def __init__(self):
        super().__init__("house_driver")
        defaults = dict(track="", max_speed=2.5, min_speed=0.8, lat_accel=4.5, decel=5.0, accel=4.0,
                        speed_per_throttle=23.0, throttle_kp=0.02, throttle_ki=0.03, speed_window=0.25)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        track = json.loads((pathlib.Path(self.p["track"]) / "track.json").read_text())
        self.path = np.asarray(track["centreline"], dtype=float).reshape(-1, 2)
        self.ds = float(track["length"]) / len(self.path)
        self.kappa = self.signed_curvature()
        self.v_ref = self.speed_profile()

        self.position = None
        self.yaw = None
        self.yaw_rate = 0.0
        self.speed = 0.0
        self.ahead = 10.0           # m, nearest lidar return in the cone ahead (another car, mostly)
        self.encoders = {}          # frame_id -> [(t, angle), ...] over the last speed_window
        self.v_target = 0.0
        self.integral = 0.0
        self.throttle = 0.0
        self.steer = 0.0
        self.last_t = None

        self.pub_throttle = self.create_publisher(Float32, NS + "throttle_command", QOS)
        self.pub_steering = self.create_publisher(Float32, NS + "steering_command", QOS)
        self.create_subscription(Point, NS + "ips", self.on_position, QOS)
        self.create_subscription(Imu, NS + "imu", self.on_imu, QOS)
        self.create_subscription(LaserScan, NS + "lidar", self.on_scan, QOS)
        for side in ("left", "right"):
            self.create_subscription(JointState, NS + side + "_encoder", self.on_encoder, QOS)
        self.get_logger().info(f"house driver on '{track['name']}' at {self.p['max_speed']} m/s "
                               f"(profile mean {self.v_ref.mean():.2f} m/s)")

    # ---------------------------------------------------------------- path model

    def signed_curvature(self):
        """Curvature along the closed centreline (positive = left turn), smoothed over ~0.5 m."""
        d = np.roll(self.path, -1, axis=0) - np.roll(self.path, 1, axis=0)
        heading = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        kappa = np.gradient(heading) / self.ds
        w = max(1, int(0.5 / self.ds))
        padded = np.concatenate([kappa[-w:], kappa, kappa[:w]])
        return np.convolve(padded, np.ones(w) / w, mode="same")[w:-w]

    def speed_profile(self):
        """Cornering limit from curvature, then a backward pass (braking) and a forward pass
        (acceleration) so the profile is reachable; two laps of passes close the loop."""
        v = np.sqrt(self.p["lat_accel"] / np.maximum(np.abs(self.kappa), 1e-3))
        v = np.clip(v, self.p["min_speed"], self.p["max_speed"])
        n = len(v)
        for _ in range(2):
            for i in range(n - 1, -1, -1):
                v[i] = min(v[i], math.sqrt(v[(i + 1) % n] ** 2 + 2.0 * self.p["decel"] * self.ds))
            for i in range(n):
                v[(i + 1) % n] = min(v[(i + 1) % n], math.sqrt(v[i] ** 2 + 2.0 * self.p["accel"] * self.ds))
        return v

    # ---------------------------------------------------------------- sensing

    def on_position(self, msg):
        """One control step per new pose: the bridge may run at 10 Hz, and stepping faster than
        the data arrives only integrates stale errors into the steering."""
        self.position = np.array([msg.x, msg.y])
        now = self.get_clock().now().nanoseconds * 1e-9
        self.dt = min(max(now - self.last_t, 0.01), 0.3) if self.last_t else 0.05
        self.last_t = now
        self.step()

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.yaw_rate = msg.angular_velocity.z

    def on_scan(self, msg):
        """Nearest return within +-15 degrees ahead: the walls are far on the centreline, so
        anything close here is another car."""
        n = len(msg.ranges)
        centre = int((0.0 - msg.angle_min) / msg.angle_increment)
        half = int(math.radians(15.0) / msg.angle_increment)
        cone = np.asarray(msg.ranges[max(0, centre - half):min(n, centre + half)], dtype=float)
        cone = cone[np.isfinite(cone)]
        self.ahead = float(cone.min()) if cone.size else 10.0

    def on_encoder(self, msg):
        """Wheel speed over a short window of samples; robust to a 10 Hz bridge and to the
        encoder reset that comes with a collision respawn."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        angle = float(msg.position[0])
        hist = self.encoders.setdefault(msg.header.frame_id, [])
        if hist and angle < hist[-1][1]:
            hist.clear()
        hist.append((t, angle))
        while len(hist) > 2 and t - hist[0][0] > self.p["speed_window"]:
            hist.pop(0)
        if len(hist) < 2 or t - hist[0][0] < 1e-3:
            return
        v = (angle - hist[0][1]) / (t - hist[0][0]) * WHEEL_RADIUS
        if 0.0 <= v < 25.0:
            self.speed = 0.5 * self.speed + 0.5 * v

    # ---------------------------------------------------------------- control

    def step(self):
        if self.position is None or self.yaw is None:
            return
        v = max(self.speed, 0.3)
        # act where the command will land: predict the pose one steering delay ahead
        yaw = self.yaw + self.yaw_rate * STEER_DELAY
        pos = self.position + v * STEER_DELAY * np.array([math.cos(self.yaw), math.sin(self.yaw)])
        i0 = int(np.argmin(np.linalg.norm(self.path - pos, axis=1)))
        n = len(self.path)

        # --- steering: L1 pursuit + curvature feed-forward, inverted through the bicycle model
        ld = 0.5 + 0.3 * max(v, 1.0) + v * self.dt          # one data interval further at slow bridges
        j = (i0 + max(1, int(ld / self.ds))) % n
        dx, dy = self.path[j] - pos
        eta = math.atan2(dy, dx) - yaw
        eta = math.atan2(math.sin(eta), math.cos(eta))
        kappa_fb = float(np.clip(2.0 * math.sin(eta) / max(math.hypot(dx, dy), 0.3), -0.8, 0.8))
        kappa_ff = 0.6 * float(self.kappa[(i0 + max(1, int(0.15 * v / self.ds))) % n])
        delta = float(np.clip(math.atan((kappa_fb + kappa_ff) * WHEELBASE), -SLIP_PEAK, SLIP_PEAK))
        self.steer += float(np.clip(delta - self.steer, -3.2 * self.dt, 3.2 * self.dt))   # the actuator's own rate

        # --- speed: lowest profile speed over the next 0.3 s, low-passed
        window = [(i0 + k) % n for k in range(max(2, int(0.3 * v / self.ds)))]
        target = float(self.v_ref[window].min())
        if self.ahead < 1.5:   # a car ahead: follow it instead of ramming it
            target = min(target, max(0.6, 2.0 * (self.ahead - 0.5)))
        self.v_target += (target - self.v_target) * min(1.0, self.dt / 0.3)
        self.throttle = self.throttle_law(self.speed, self.v_target)

        self.pub_throttle.publish(Float32(data=self.throttle))
        self.pub_steering.publish(Float32(data=self.steer / MAX_STEER))

    def throttle_law(self, v, v_t):
        """Feed-forward plus a trim bounded to half of it: the command never falls to zero
        (a hard brake) while speed is wanted, so the car cannot surge and stop."""
        ff = v_t / self.p["speed_per_throttle"]
        err = v_t - v
        self.integral = float(np.clip(self.integral + err * self.dt, -0.8, 0.8))
        trim = float(np.clip(self.p["throttle_kp"] * err + self.p["throttle_ki"] * self.integral, -0.5 * ff, 0.5 * ff))
        wanted = float(np.clip(ff + trim, 0.0, 1.0))
        slew = 0.8 * self.dt
        return float(np.clip(wanted, self.throttle - slew, self.throttle + slew)) if self.throttle > 0.0 else wanted


def main():
    rclpy.init()
    node = HouseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_throttle.publish(Float32(data=0.0))
        node.destroy_node()


if __name__ == "__main__":
    main()
