"""The house driver: a competent, predictable opponent that follows the track's centreline.

It knows the map (track.json) and uses the simulator's position and orientation, which
competitors may not do at race time. That is the point: it is the rabbit, not an entry.

Steering: MAP-style pursuit (L1 guidance plus a curvature feed-forward, capped at the tyre's
slip peak, lightly low-passed), from the IROS 2026 league stack in ~/roboracer.
Speed: a profile along the path from curvature with a braking backward pass, tracked by
feed-forward plus a bounded trim (throttle 0 is a hard brake in this simulator, so a plain
PI loop limit-cycles), slew-limited, with a hysteretic brake regime for real overspeed.

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
from sensor_msgs.msg import Imu, JointState
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
                        speed_per_throttle=23.0, throttle_kp=0.05, throttle_ki=0.03, rate=40.0)
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
        self.encoders = {}
        self.v_target = 0.0
        self.integral = 0.0
        self.throttle = 0.0
        self.steer = 0.0
        self.braking = False

        self.pub_throttle = self.create_publisher(Float32, NS + "throttle_command", QOS)
        self.pub_steering = self.create_publisher(Float32, NS + "steering_command", QOS)
        self.create_subscription(Point, NS + "ips", self.on_position, QOS)
        self.create_subscription(Imu, NS + "imu", self.on_imu, QOS)
        for side in ("left", "right"):
            self.create_subscription(JointState, NS + side + "_encoder", self.on_encoder, QOS)
        self.dt = 1.0 / self.p["rate"]
        self.create_timer(self.dt, self.step)
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
        self.position = np.array([msg.x, msg.y])

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.yaw_rate = msg.angular_velocity.z

    def on_encoder(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        angle = float(msg.position[0])
        last = self.encoders.get(msg.header.frame_id)
        self.encoders[msg.header.frame_id] = (t, angle)
        if last and angle >= last[1] and t - last[0] > 1e-3:
            v = (angle - last[1]) / (t - last[0]) * WHEEL_RADIUS
            if v < 25.0:
                self.speed = 0.7 * self.speed + 0.3 * v

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
        ld = 0.35 + 0.22 * v
        j = (i0 + max(1, int(ld / self.ds))) % n
        dx, dy = self.path[j] - pos
        eta = math.atan2(dy, dx) - yaw
        eta = math.atan2(math.sin(eta), math.cos(eta))
        kappa_fb = 2.0 * math.sin(eta) / max(math.hypot(dx, dy), 0.3)
        kappa_ff = 0.6 * float(self.kappa[(i0 + max(1, int(0.15 * v / self.ds))) % n])
        delta = float(np.clip(math.atan((kappa_fb + kappa_ff) * WHEELBASE), -SLIP_PEAK, SLIP_PEAK))
        self.steer += (delta - self.steer) * min(1.0, self.dt / 0.06)

        # --- speed: lowest profile speed over the next 0.3 s, low-passed
        window = [(i0 + k) % n for k in range(max(2, int(0.3 * v / self.ds)))]
        target = float(self.v_ref[window].min())
        self.v_target += (target - self.v_target) * min(1.0, self.dt / 0.3)
        self.throttle = self.throttle_law(self.speed, self.v_target)

        self.pub_throttle.publish(Float32(data=self.throttle))
        self.pub_steering.publish(Float32(data=self.steer / MAX_STEER))

    def throttle_law(self, v, v_t):
        """Feed-forward plus a trim bounded to half of it: the command never falls to zero
        (a hard brake) while speed is wanted. Real overspeed uses the brake with hysteresis."""
        if self.braking:
            self.braking = v > v_t + 0.2
        elif v > v_t + 0.6:
            self.braking = True
        if self.braking:
            self.integral = 0.0
            return 0.0
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
