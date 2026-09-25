"""Cone-track racer: map the track on lap 1, then race a raceline with nonlinear MPC.

Lap 1 (MAPPING): the simple cone follower drives (perception -> local centreline -> pure
pursuit, slowly) while GraphSLAM builds a map of the cones from odometry (wheel speed + IMU
heading) and the coloured cones seen each scan. When the car is back at its start the map is
frozen, the boundaries ordered, the centreline sampled, a minimum-curvature raceline and a
speed profile computed.
Laps 2+ (RACING): the car localises against the frozen map (ICP of the cones it sees) and a
nonlinear MPC on a dynamic bicycle model tracks the raceline at the profile's speed. If the
solver ever fails, pure pursuit on the raceline takes over for that scan.

Inputs: lidar, front_camera, imu, wheel encoders, steering feedback. No ground truth.
Outputs: steering_command, throttle_command. Debug topics under /feb/.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from std_msgs.msg import Float32

from .graphslam import GraphSLAM
from .perception import BLUE, ORANGE, YELLOW, Perception
from .raceline import heading_along, min_curvature, speed_profile
from .track import build_track

NS = "/autodrive/roboracer_1/"
QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=10)
WHEELBASE = 0.324
WHEEL_RADIUS = 0.059
MAX_STEER = 0.5236
MAX_STEER_RATE = 3.2
STEER_DELAY = 0.19          # s, steering command to wheel, sysid

DEFAULTS = dict(
    # perception (as feb_cone_driver)
    max_range=6.0, cluster_gap=0.15, cone_max_width=0.35, cone_max_points=80, match_px_frac=0.04,
    min_blob_px=3, camera_hfov_deg=57.1, track_gate=0.4, track_memory=40, side_guess_range=2.5, track_width=2.2, side_override_x=1.5, side_override_y=0.6,
    camera_ahead=-0.66, camera_lateral=-0.033, camera_col_bias=-1.0, camera_lag_s=0.073,   # camera vs lidar, fitted on this car
    hsv_blue=[105, 220, 8, 135, 255, 255], hsv_yellow=[18, 150, 8, 40, 255, 255], hsv_orange=[0, 150, 15, 15, 255, 255],
    band_top=0.20, band_bottom=0.36, band_floor=0.21, orange_min_px=35,
    # lap-1 follower
    map_speed=1.2, map_min_speed=0.8, lookahead=1.0, chain_step=1.8, avoid_range=0.9,
    speed_per_throttle=23.0, throttle_kp=0.02, throttle_ki=0.03, throttle_slew=0.8, speed_window=0.25, steer_tau=0.15,
    # slam
    keyframe_dist=0.4, slam_range=5.0, dx_weight=2.0, z_weight=1.0, new_landmark_dist=0.6, icp_gate=1.5,
    solve_every=3, min_lap_length=15.0, lap_close_dist=2.0, min_seen=2, icp_min_matches=5, snap_radius=10.0, snap_gate=6.0,
    # raceline
    sample_step=0.25, car_half_width=0.135, margin=0.40, curvature_reg=0.0002,
    v_max=4.0, a_lat=3.5, a_acc=2.5, a_brake=3.0,
    # mpc: model
    mass=3.9, inertia_z=0.10, lf=0.175, lr=0.175, tyre_B=8.0, tyre_C=1.4, tyre_D=18.0,   # L 0.35 m effective (yaw-rate fit)
    long_a=74.4, long_b=2.54, idle_brake=0.71,           # sysid fit on this car (feb_driver/tools/sysid_fit.py)
    max_steer=MAX_STEER, max_steer_rate=MAX_STEER_RATE, tau_max=0.14, v_cap=5.0,
    # mpc: problem
    mpc_horizon=12, mpc_dt=0.1, mpc_max_iter=60, mpc_steer_tau=0.15, mpc_substeps=3,
    w_pos=8.0, w_head=2.0, w_speed=0.6, w_vy=0.2, w_dsteer=0.01, w_dtau=3.0, w_tau=0.3, dtau_max=0.4,
    lost_after=1.5, loc_lost_after=3.0,
    pursuit_lookahead=1.2, race_speed_scale=0.6,
)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Racer(Node):
    def __init__(self):
        super().__init__("racer")
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in DEFAULTS}
        p = self.p
        self.perception = Perception(p)
        self.slam = GraphSLAM((0.0, 0.0), p["dx_weight"], p["z_weight"], p["new_landmark_dist"], p["icp_gate"],
                              icp_min_matches=int(p["icp_min_matches"]))
        try:
            from .mpc import BicycleMPC
            self.mpc = BicycleMPC(p)
            if not self.mpc.ok:
                self.mpc = None
        except Exception as e:                      # noqa: BLE001
            self.get_logger().warn("MPC unavailable (%s); racing with pure pursuit. Run install_deps.sh." % e)
            self.mpc = None
        if self.mpc is None:
            self.get_logger().warn("CasADi not importable: mapping and raceline still run, control falls back to pure pursuit")

        # vehicle state
        self.yaw = None
        self.yaw_rate = 0.0
        self.speed = 0.0
        self.encoders = {}
        self.delta = 0.0                 # steering angle at the wheels, rad, left positive
        self.pose = np.zeros(2)          # x, y in the map frame (starts at the origin)
        self.start = None                # (pose, yaw) at the first keyframe
        self.travelled = 0.0
        self.since_key = np.zeros(2)
        self.since_key_dist = 0.0
        self.keyframes = 0
        self.mode = "MAPPING"
        self.track = None
        self.raceline = None             # (N,2)
        self.race_v = None
        self.race_psi = None
        self.race_s = None
        self.race_idx = 0
        # control state
        self.steer = 0.0
        self.throttle = 0.0
        self.integral = 0.0
        self.last_t = None
        self.uprev = np.zeros(2)
        self.lap_times = []
        self.lap_start_t = None
        self.solve_ms = 0.0
        self.prev_clusters = None        # last scan's cone clusters, to tell whether the world moves past us
        self.clusters_now = []
        self.last_dt = 0.1
        self.odo_speed = 0.0
        self.loc_log_t = 0.0
        self.last_seen_t = 0.0
        self.good_loc_t = 0.0
        self.local_mode = False
        self.stalled_since = None
        self.reverse_until = None

        self.pub_throttle = self.create_publisher(Float32, NS + "throttle_command", QOS)
        self.pub_steering = self.create_publisher(Float32, NS + "steering_command", QOS)
        self.pub_cones = self.create_publisher(PoseArray, "/feb/cones", 10)
        self.pub_map = self.create_publisher(PoseArray, "/feb/map", 10)
        self.pub_line = self.create_publisher(Path, "/feb/raceline", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/feb/pose", 10)
        self.pub_pred = self.create_publisher(Path, "/feb/mpc_prediction", 10)
        self.create_subscription(LaserScan, NS + "lidar", self.on_scan, QOS)
        self.create_subscription(Image, NS + "front_camera", self.perception.on_image, QOS)
        self.create_subscription(Imu, NS + "imu", self.on_imu, QOS)
        self.create_subscription(Float32, NS + "steering", self.on_steer_fb, QOS)
        for side in ("left", "right"):
            self.create_subscription(JointState, NS + side + "_encoder", self.on_encoder, QOS)
        self.get_logger().info("racer up: mapping lap first, then %s" % ("MPC" if self.mpc else "pure pursuit"))

    # ------------------------------------------------------------ sensors

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.yaw_rate = msg.angular_velocity.z

    def on_steer_fb(self, msg):
        self.delta = float(msg.data)      # radians, left positive: same sign as the command (fitted on bags)

    def on_encoder(self, msg):
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

    # ------------------------------------------------------------ main loop, per scan

    def on_scan(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = min(max(t - self.last_t, 0.01), 0.5) if self.last_t else 0.1
        self.last_t = t
        if self.yaw is None:
            return
        clusters = self.perception.clusters(msg)
        self.clusters_now, self.last_dt = clusters, dt
        cones = self.perception.fuse(clusters, self.speed, self.yaw, dt, self.yaw_rate)
        self.publish_cones(msg.header, cones)

        # dead reckoning: wheel speed, slew-limited to what the car can physically do (spinning
        # wheels read high), and only while the lidar scene actually moves past us
        self.odo_speed = float(np.clip(self.speed, self.odo_speed - 6.0 * dt, self.odo_speed + 4.0 * dt))
        moving_now = self.prev_clusters is None or self.scene_moving(msg, peek=True)
        ds = self.odo_speed * dt if moving_now else 0.0
        dx = np.array([ds * math.cos(self.yaw), ds * math.sin(self.yaw)])
        self.pose = self.pose + dx
        self.travelled += ds
        self.since_key += dx
        self.since_key_dist += ds

        # cones for the map: every coloured cone within range gives a position edge; only a
        # colour the camera confirmed this scan carries a full vote for the landmark's colour
        obs = [(x, y, c, w) for x, y, c, w in cones if math.hypot(x, y) < self.p["slam_range"]]
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        z_rel = np.array([[cy * x - sy * y, sy * x + cy * y] for x, y, c, w in obs]).reshape(-1, 2)
        colours = np.array([c for _, _, c, _ in obs], dtype=int)
        weights = np.array([w for _, _, _, w in obs], dtype=float)

        moving = self.scene_moving(msg)
        if self.mode == "MAPPING":
            self.mapping_step(z_rel, colours, weights)
            steer, throttle = self.follow_local(clusters, cones, dt)
        else:
            matched = self.localise(z_rel, colours)
            if len(cones) >= 2:
                self.last_seen_t = t
            # localisation health: after a few seconds without matches the map position is not
            # to be trusted, so the local follower (the mapping-lap driver) takes over until the
            # map is matched again; with nothing in view at all, stop rather than bolt
            if matched >= 3 or len(z_rel) < 3:
                self.good_loc_t = t                 # matched, or too little in view to judge
            if t - self.last_seen_t > self.p["lost_after"]:
                # nothing in view: the car has left the corridor. Its dead-reckoned position is
                # still roughly right, so crawl back towards the raceline by pure pursuit; if that
                # brings no cones into view for a while, back up instead
                self.mpc_warm_reset()
                if t - self.last_seen_t > 20.0:
                    steer, throttle = 0.0, 0.0             # lost for good: stop, do not wander
                else:
                    if t - self.last_seen_t > 8.0 and self.reverse_until is None and self.stalled_since is None:
                        self.get_logger().warn("no cones in view for long: backing up")
                        self.reverse_until = t + 1.5
                        self.stalled_since = t
                    steer, throttle = self.pursuit_step(dt, speed=0.7)
            elif t - self.good_loc_t > self.p["loc_lost_after"]:
                if not self.local_mode:
                    self.get_logger().warn("localisation lost: local follower until the map is matched again")
                    self.local_mode = True
                steer, throttle = self.follow_local(clusters, cones, dt)
                self.mpc_warm_reset()
            else:
                if self.local_mode:
                    self.get_logger().info("localisation back: MPC on the raceline")
                    self.local_mode = False
                    self.race_idx = self.nearest_index()
                steer, throttle = self.race_step(dt)
        # stuck on a cone (wheels may spin, so watch the lidar scene): back away, then carry on
        if self.reverse_until is not None:
            if t < self.reverse_until:
                self.pub_throttle.publish(Float32(data=-0.12))
                self.pub_steering.publish(Float32(data=-self.steer / MAX_STEER))
                return
            self.reverse_until, self.stalled_since = None, None
            self.throttle, self.integral = 0.0, 0.0
            self.mpc_warm_reset()
        if moving:
            self.stalled_since = None
        elif self.throttle > 0.02 or throttle > 0.02:
            self.stalled_since = self.stalled_since or t      # sticky: a throttle dip does not reset it
            if t - self.stalled_since > 2.0:
                self.get_logger().warn("stuck, backing up")
                self.reverse_until = t + 1.2
        self.steer, self.throttle = steer, throttle
        self.pub_throttle.publish(Float32(data=float(throttle)))
        self.pub_steering.publish(Float32(data=float(steer / MAX_STEER)))
        self.publish_pose(msg.header)

    def scene_moving(self, msg, peek=False):
        """True when the world moves past the car. Cone clusters are matched to last scan's by
        nearest neighbour and their median displacement along the car's axis is compared with
        what the wheels claim (or, if the wheels are blocked, with a small minimum while the
        throttle is on). Stuck on a cone, the steering rocks the body but nothing moves forward."""
        cur = np.array(self.clusters_now, float).reshape(-1, 2)
        prev = self.prev_clusters
        if not peek:
            self.prev_clusters = cur
        if prev is None or len(prev) < 3 or len(cur) < 3:
            return True
        d = np.linalg.norm(cur[:, None, :] - prev[None, :, :], axis=2)
        j = d.argmin(axis=1)
        near = d.min(axis=1) < 0.5
        if near.sum() < 3:
            return True
        forward = np.median(np.abs(cur[near, 0] - prev[j[near], 0]))
        expected = max(self.speed * self.last_dt, 0.03 if self.throttle > 0.04 else 0.0)
        if expected <= 0.0:
            return True
        return bool(forward > 0.4 * expected)

    # ------------------------------------------------------------ mapping
    # ------------------------------------------------------------ mapping
    # ------------------------------------------------------------ mapping

    def mapping_step(self, z_rel, colours, weights):
        p = self.p
        if self.start is None:
            self.start = (self.pose.copy(), self.yaw)
            self.lap_start_t = self.last_t
        if self.since_key_dist < p["keyframe_dist"] and self.keyframes > 0:
            return
        # back near the start with the orange gate in view: close the loop with a wide net, so
        # odometry drift over a long lap cannot leave the start cones unmatched
        if self.travelled > p["min_lap_length"] and np.any(colours == ORANGE) and np.linalg.norm(self.pose - self.start[0]) < p["snap_radius"]:
            zw = z_rel + self.pose + self.since_key
            t_snap, R_snap = self.slam.snap(zw, colours, self.start[0], p["snap_radius"], p["snap_gate"])
            if np.any(t_snap):
                self.since_key = self.since_key + t_snap
                self.get_logger().info("loop closure at the start gate: pose corrected by %.2f m" % np.linalg.norm(t_snap))
        self.slam.add(self.since_key, z_rel, colours, weights)
        self.keyframes += 1
        self.since_key = np.zeros(2)
        self.since_key_dist = 0.0
        if self.keyframes % int(p["solve_every"]) == 0:
            try:
                self.slam.solve()
            except Exception as e:                    # noqa: BLE001
                self.get_logger().warn("slam solve failed: %s" % e)
        self.pose = self.slam.xhat[-1].copy()
        # back at the start?
        d0 = np.linalg.norm(self.pose - self.start[0])
        if self.travelled > p["min_lap_length"] and d0 < p["lap_close_dist"] and math.cos(self.yaw - self.start[1]) > 0.5:
            self.finish_mapping()
        elif self.keyframes % 10 == 0:
            self.get_logger().info("mapping: %d keyframes, %d landmarks, travelled %.1f m" % (self.keyframes, len(self.slam.lhat), self.travelled))

    def finish_mapping(self):
        p = self.p
        t0 = time.time()
        self.slam.freeze(int(p["min_seen"]))
        lap = self.last_t - self.lap_start_t
        self.lap_times.append(lap)
        track = build_track(self.slam.lhat, self.slam.colour, self.start[0], (math.cos(self.start[1]), math.sin(self.start[1])), p["sample_step"])
        if track is None:
            self.get_logger().error("map has too few cones of a colour; keep following the local line")
            self.slam.frozen = False
            return
        self.slam.colour_override = track["colour"]
        line, _ = min_curvature(track["centre"], track["normal"], track["half_width"], p["car_half_width"], p["margin"], p["curvature_reg"])
        v, kappa = speed_profile(line, p["v_max"], p["a_lat"], p["a_acc"], p["a_brake"])
        self.track, self.raceline, self.race_v = track, line, v * p["race_speed_scale"]
        self.race_psi = heading_along(line)
        seg = np.linalg.norm(np.roll(line, -1, axis=0) - line, axis=1)
        self.race_s = np.concatenate([[0.0], np.cumsum(seg)])
        self.race_idx = int(np.argmin(np.linalg.norm(line - self.pose, axis=1)))
        self.mode = "RACING"
        self.local_mode = np.linalg.norm(line[self.race_idx] - self.pose) > 1.0   # off the line: follower first
        self.lap_start_t = self.last_t
        self.good_loc_t = self.last_t
        self.get_logger().info("map closed after %.1f s: %d cones, raceline %.1f m, %d points, v %.1f..%.1f m/s (%.0f ms) -> RACING with %s"
                               % (lap, len(self.slam.lhat), self.race_s[-1], len(line), v.min(), v.max(), 1000 * (time.time() - t0), "MPC" if self.mpc else "pure pursuit"))
        self.publish_map()

    # ------------------------------------------------------------ lap-1 follower (the simple driver)

    def chain(self, cones, max_step):
        if not cones:
            return []
        rest = sorted(cones, key=lambda c: math.hypot(c[0], c[1]))
        out = [rest.pop(0)]
        heading = (1.0, 0.0)
        while rest and len(out) < 8:
            last = out[-1]
            best, best_score = None, -1.0
            for c in rest:
                dx, dy = c[0] - last[0], c[1] - last[1]
                d = math.hypot(dx, dy)
                if d > max_step or d < 0.05:
                    continue
                score = (dx * heading[0] + dy * heading[1]) / d
                if score > best_score:
                    best, best_score = c, score
            if best is None or best_score < 0.0:
                break
            dx, dy = best[0] - last[0], best[1] - last[1]
            d = math.hypot(dx, dy)
            heading = (dx / d, dy / d)
            out.append(best)
            rest.remove(best)
        return out

    def local_centreline(self, cones):
        """Midpoints between the ordered chains. A blue chain cone pairs with the yellow cone that
        lies across the track to its RIGHT (right of the chain's direction of travel), never the
        nearest yellow regardless of side: at a hairpin the nearest yellow is often on the far
        branch and the midpoint would cut through the inner cones and turn the car around.
        Unpaired cones are offset by half the track width along their chain's normal."""
        # the orange start cones stand on the boundary lines: for driving they count as the
        # colour of the side they are on
        cones = [(x, y, (BLUE if y > 0.0 else YELLOW) if c == ORANGE else c) for x, y, c in cones]
        left = self.chain([c for c in cones if c[2] == BLUE and c[0] > -1.0], self.p["chain_step"])
        right = self.chain([c for c in cones if c[2] == YELLOW and c[0] > -1.0], self.p["chain_step"])
        half = self.p["track_width"] / 2.0

        def tangents(chain):
            out = []
            for i, c in enumerate(chain):
                a = chain[max(i - 1, 0)]
                b = chain[min(i + 1, len(chain) - 1)]
                tx, ty = b[0] - a[0], b[1] - a[1]
                n = math.hypot(tx, ty)
                out.append((tx / n, ty / n) if n > 1e-6 else (1.0, 0.0))
            return out

        lt, rt = tangents(left), tangents(right)
        points, used = [], set()
        for i, l in enumerate(left):
            nx, ny = lt[i][1], -lt[i][0]                       # right-hand normal of the blue chain
            best, best_score = None, None
            for j, r in enumerate(right):
                dx, dy = r[0] - l[0], r[1] - l[1]
                d = math.hypot(dx, dy)
                across = dx * nx + dy * ny                       # how far to the right
                along = abs(dx * lt[i][0] + dy * lt[i][1])       # how far along the track
                if j in used or d > 1.3 * self.p["track_width"] or across < 0.4 * d:
                    continue
                score = along + 0.5 * d
                if best_score is None or score < best_score:
                    best, best_score = j, score
            if best is None:
                points.append((l[0] + half * nx, l[1] + half * ny))
            else:
                used.add(best)
                points.append(((l[0] + right[best][0]) / 2.0, (l[1] + right[best][1]) / 2.0))
        for j, r in enumerate(right):
            if j not in used:
                nx, ny = -rt[j][1], rt[j][0]                     # left-hand normal of the yellow chain
                points.append((r[0] + half * nx, r[1] + half * ny))
        return sorted([q for q in points if q[0] > 0.0], key=lambda q: math.hypot(*q))

    def follow_local(self, clusters, cones, dt):
        p = self.p
        line = self.local_centreline([(x, y, c) for x, y, c, w in cones])
        target = next((q for q in line if math.hypot(*q) >= p["lookahead"]), line[-1] if line else None)
        if target is None:
            self.integral = 0.0
            return self.steer * 0.5, 0.0
        ld = max(math.hypot(*target), 0.3)
        alpha = math.atan2(target[1], target[0])
        wanted = math.atan(2.0 * WHEELBASE * math.sin(alpha) / ld)
        for x, y in clusters:
            if 0.0 < x < p["avoid_range"] and abs(y) < 0.5:
                wanted -= math.copysign(0.6, y) * (1.0 - x / p["avoid_range"])
        wanted = float(np.clip(wanted, -MAX_STEER, MAX_STEER))
        step = (wanted - self.steer) * min(1.0, dt / p["steer_tau"])
        steer = self.steer + float(np.clip(step, -MAX_STEER_RATE * dt, MAX_STEER_RATE * dt))
        v_goal = max(p["map_min_speed"], p["map_speed"] * (1.0 - 0.7 * abs(steer) / MAX_STEER))
        return steer, self.throttle_law(v_goal, dt)

    def throttle_law(self, v_t, dt):
        p = self.p
        ff = v_t / p["speed_per_throttle"]
        err = v_t - self.speed
        self.integral = float(np.clip(self.integral + err * dt, -0.8, 0.8))
        bound = ff if self.speed < 0.3 else 0.5 * ff
        trim = float(np.clip(p["throttle_kp"] * err + p["throttle_ki"] * self.integral, -bound, bound))
        wanted = float(np.clip(ff + trim, 0.0, 1.0))
        slew = p["throttle_slew"] * dt
        return float(np.clip(wanted, self.throttle - slew, self.throttle + slew)) if self.throttle > 0.0 else wanted

    # ------------------------------------------------------------ racing

    def mpc_warm_reset(self):
        if self.mpc is not None:
            self.mpc.warm = None
        self.uprev = np.array([self.delta, 0.0])

    def localise(self, z_rel, colours):
        corrected, matched = self.slam.localise(self.pose, z_rel, colours)
        shift = float(np.linalg.norm(corrected - self.pose))
        if matched >= 3:
            self.pose = 0.5 * self.pose + 0.5 * corrected      # trust the map, but no jumps
        if self.last_t - self.loc_log_t > 2.0:
            self.loc_log_t = self.last_t
            self.get_logger().info("localise: %d cones in view, %d matched, correction %.2f m, speed %.1f, mpc %.0f ms" % (len(z_rel), matched, shift, self.speed, self.solve_ms))
        # lap timing on the raceline index wrap
        n = len(self.raceline)
        i = self.nearest_index()
        if self.race_idx > 0.8 * n and i < 0.2 * n:
            lap = self.last_t - self.lap_start_t
            self.lap_start_t = self.last_t
            self.lap_times.append(lap)
            self.get_logger().info("lap %d: %.2f s (mpc %.0f ms/solve)" % (len(self.lap_times) - 1, lap, self.solve_ms))
        self.race_idx = i
        return matched

    def nearest_index(self):
        n = len(self.raceline)
        window = np.arange(self.race_idx - 10, self.race_idx + 40) % n
        d = np.linalg.norm(self.raceline[window] - self.pose, axis=1)
        if d.min() > 2.0:                                   # far from where we thought: search everywhere
            return int(np.argmin(np.linalg.norm(self.raceline - self.pose, axis=1)))
        return int(window[int(np.argmin(d))])

    def reference(self, i0, v_now):
        """Raceline points ahead, one per MPC stage, spaced by the profile speed times DT."""
        N, DT = self.mpc.N, self.mpc.DT
        n = len(self.raceline)
        s = self.race_s[i0]
        ref = np.zeros((4, N))
        v = max(v_now, 0.5)
        idx = i0
        for k in range(N):
            v = self.race_v[idx]
            s += v * DT
            idx = (i0 + int(round((s - self.race_s[i0]) / self.p["sample_step"]))) % n
            ref[0, k], ref[1, k] = self.raceline[idx]
            ref[2, k] = self.race_psi[idx]
            ref[3, k] = self.race_v[idx]
        return ref

    def race_step(self, dt):
        i = self.race_idx
        if self.mpc is not None:
            # delay compensation: where the car will be when this command bites
            x, y = self.pose
            psi, v, d = self.yaw, self.odo_speed, self.delta
            for _ in range(3):
                h = STEER_DELAY / 3.0
                x += v * math.cos(psi) * h
                y += v * math.sin(psi) * h
                psi += v * math.tan(d) / WHEELBASE * h
            z0 = np.array([x, y, psi, v, 0.0, self.yaw_rate, d])
            ref = self.reference(i, v)
            # unwrap the reference heading around the current one
            ref[2] = psi + np.array([wrap(a - psi) for a in ref[2]])
            t0 = time.time()
            out = self.mpc.solve(z0, ref, self.uprev)
            self.solve_ms = 0.8 * self.solve_ms + 0.2 * 1000 * (time.time() - t0)
            if out is not None:
                u0, Z = out
                self.uprev = u0
                self.publish_prediction(Z)
                return float(np.clip(u0[0], -MAX_STEER, MAX_STEER)), float(np.clip(u0[1], 0.0, 1.0))
        return self.pursuit_step(dt)

    def pursuit_step(self, dt, speed=None):
        """Pure pursuit on the raceline: the fallback when the MPC is unavailable or fails, and
        the way back when the car has lost sight of the cones."""
        i = self.race_idx = self.nearest_index()
        n = len(self.raceline)
        j = i
        while np.linalg.norm(self.raceline[j] - self.pose) < self.p["pursuit_lookahead"]:
            j = (j + 1) % n
            if j == i:
                break
        tx, ty = self.raceline[j] - self.pose
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        lx, ly = c * tx - s * ty, s * tx + c * ty
        ld = max(math.hypot(lx, ly), 0.3)
        wanted = float(np.clip(math.atan(2.0 * WHEELBASE * ly / (ld * ld)), -MAX_STEER, MAX_STEER))
        step = (wanted - self.steer) * min(1.0, dt / self.p["steer_tau"])
        steer = self.steer + float(np.clip(step, -MAX_STEER_RATE * dt, MAX_STEER_RATE * dt))
        return steer, self.throttle_law(float(self.race_v[i]) if speed is None else speed, dt)

    # ------------------------------------------------------------ debug topics

    def publish_cones(self, header, cones):
        arr = PoseArray(header=header)
        for x, y, c, w in cones:
            q = Pose()
            q.position.x, q.position.y = float(x), float(y)
            q.orientation.w = float(c)
            arr.poses.append(q)
        self.pub_cones.publish(arr)

    def publish_map(self):
        arr = PoseArray()
        arr.header.frame_id = "map"
        for (x, y), c in zip(self.slam.lhat, self.slam.colour):
            q = Pose()
            q.position.x, q.position.y, q.orientation.w = float(x), float(y), float(c)
            arr.poses.append(q)
        self.pub_map.publish(arr)
        path = Path()
        path.header.frame_id = "map"
        for (x, y), v in zip(self.raceline, self.race_v):
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(v)
            path.poses.append(ps)
        self.pub_line.publish(path)

    def publish_pose(self, header):
        ps = PoseStamped()
        ps.header.stamp = header.stamp
        ps.header.frame_id = "map"
        ps.pose.position.x, ps.pose.position.y = float(self.pose[0]), float(self.pose[1])
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.pub_pose.publish(ps)

    def publish_prediction(self, Z):
        path = Path()
        path.header.frame_id = "map"
        for k in range(Z.shape[1]):
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(Z[0, k]), float(Z[1, k]), float(Z[3, k])
            path.poses.append(ps)
        self.pub_pred.publish(path)


def main():
    rclpy.init()
    node = Racer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_throttle.publish(Float32(data=0.0))
        node.pub_steering.publish(Float32(data=0.0))
        node.destroy_node()
