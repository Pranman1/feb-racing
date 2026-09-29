"""Cone-track racer: map the track on lap 1, then race a raceline with MPC.

Lap 1 (MAPPING): the simple cone follower drives (perception -> local centreline -> pure
pursuit, slowly) while GraphSLAM builds a map of the cones from odometry (wheel speed + IMU
heading) and the coloured cones seen each scan. When the car is back at its start the map is
frozen, the boundaries ordered, the centreline sampled, a minimum-curvature raceline and a
speed profile computed.
Laps 2+ (RACING): the body's speed comes from the wheels through the measured tyre curve, the
position from dead reckoning corrected against the frozen map (ICP of the cones in view), and
an MPC on the car's measured model tracks the raceline (controller.py, mpc.py, vehicle.py).

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
from std_msgs.msg import Float32, String

from .controller import RaceController, wrap
from .graphslam import GraphSLAM
from .params import RACE
from .perception import BLUE, ORANGE, UNKNOWN, YELLOW, Perception
from .raceline import min_curvature, speed_plan
from .track import build_track, repair_by_path, track_from_rungs
from .vehicle import AXLE_TO_LIDAR, BodySpeed

try:                                     # the team's cone ordering, if its package is in the stack
    from feb_cone_ordering.srv import OrderCones
except ImportError:
    OrderCones = None

NS = "/autodrive/roboracer_1/"
QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=10)
# the raceline is published once, when the track is ready: latched, so anything that attaches
# later (Foxglove, a probe) still receives it
QOS_LATCHED = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, depth=1)
WHEELBASE = 0.324
WHEEL_RADIUS = 0.059
MAX_STEER = 0.5236
MAX_STEER_RATE = 3.2

DEFAULTS = dict(
    # perception (as feb_cone_driver)
    max_range=6.0, cluster_gap=0.15, cone_max_width=0.35, cone_max_points=80, match_px_frac=0.04,
    min_blob_px=3, camera_hfov_deg=57.1, track_gate=0.4, track_memory=40, side_guess_range=2.5, track_width=2.2, side_override_x=1.5, side_override_y=0.6,
    camera_ahead=-0.66, camera_lateral=-0.033, camera_col_bias=-1.0, camera_lag_s=0.073,   # camera vs lidar, fitted on this car
    hsv_blue=[105, 220, 8, 135, 255, 255], hsv_yellow=[18, 150, 8, 40, 255, 255], hsv_orange=[0, 150, 15, 15, 255, 255],
    band_top=0.20, band_bottom=0.36, band_floor=0.21, orange_min_px=35, blob_min_value=25,
    # lap-1 follower
    map_speed=1.2, map_min_speed=0.8, lookahead=1.0, follow_range=3.5, throttle_start=0.07, chain_step=1.8, avoid_range=0.9, local_path_lookahead=1.4, local_path_min_reach=2.0, rung_max_age=1.0,
    speed_per_throttle=24.95, speed_tau=0.8, throttle_kp=0.02, throttle_ki=0.03, throttle_slew=0.8, speed_window=0.25, steer_tau=0.15,
    push_throttle=0.16, recover_for=8.0,
    # slam
    keyframe_dist=0.4, slam_range=6.0, loc_range=6.0, loc_corridor=2.5, dx_weight=2.0, z_weight=1.0, new_landmark_dist=0.6, icp_gate=1.5,
    solve_every=3, min_lap_length=15.0, order_timeout=10.0, closure_landmarks=10, lap_close_dist=2.0, min_seen=2, icp_min_matches=5, snap_radius=10.0, snap_gate=6.0,
    # racing: where the car is
    loc_gain_along=0.3,       # share of the map match's correction taken along the car's heading, per scan
    loc_gain_across=0.5,      # and across it
    loc_max_step=0.25,        # m, the most a single scan may move the estimate
    lost_after=1.5,           # s without cones in view, or without a map match, before the plan is slowed
    loc_lost_after=3.0,       # s without a map match before the car is looked for anywhere on the map
    blind_speed=0.5,          # share of the plan's speed driven while blind or unmatched
    give_up_after=8.0,        # s blind or unmatched before the car stops
    start_ramp=2.5,           # s over which the racing laps come up to the plan's pace once the first corner is behind the car
    pin_turn=0.8,             # rad the raceline has to turn from the race start before the car's place along the track counts as pinned
    debug=False,
    # racing: raceline, speed plan and MPC (params.py)
    **RACE,
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
            from .mpc import KinematicMPC
            # the MPC may not plan faster than the speed plan it is following
            p["v_cap"] = min(p["v_cap"], p["v_max"] * p["race_speed_scale"] + 0.3)
            self.mpc = KinematicMPC(p)
            if not self.mpc.ok:
                self.mpc = None
        except Exception as e:                      # noqa: BLE001
            self.get_logger().warn("MPC unavailable (%s); racing with pure pursuit. Run install_deps.sh." % e)
            self.mpc = None
        if self.mpc is None:
            self.get_logger().warn("CasADi not importable: mapping and raceline still run, control falls back to pure pursuit")
        self.ctl = RaceController(p, self.mpc)
        self.body = BodySpeed()                  # the body's speed while racing, from the wheels through the tyre

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
        self.enc_now = {}                # latest (stamp, angle) of each wheel encoder
        self.enc_prev = None             # (stamp, mean angle) at the previous racing scan
        self.race_yaw = None             # heading at the previous racing scan
        self.race_s = None               # progress along the raceline at the previous racing scan
        self.icp_track = []              # (time, position the map match gave): is the car going anywhere?
        self.matched_run = 0             # racing scans in a row with a good map match
        self.race_ready_t = None         # when the map match was first confirmed on the racing laps
        self.still_since = None          # since when the lidar scene has stood still
        # control state
        self.steer = 0.0
        self.throttle = 0.0
        self.integral = 0.0
        self.last_t = None
        self.lap_times = []
        self.lap_start_t = None
        self.solve_ms = 0.0
        self.prev_clusters = None        # last scan's cone clusters, to tell whether the world moves past us
        self.clusters_now = []
        self.last_dt = 0.1
        self.odo_speed = 0.0
        self.v_cmd = 0.0                         # the speed asked of the throttle law, rate limited
        self.v_goal = None                       # the follower's own speed, smoothed
        self.seen_pose, self.recovering = np.zeros(2), False          # where cones were last in view
        self.loc_log_t = 0.0
        self.last_seen_t = 0.0
        self.no_target = False
        self.snap_t = -1e9
        self.target_t = -1e9
        self.good_loc_t = 0.0
        self.reloc_t = 0.0
        self.stalled_since = None
        self.halted = False
        self.push_until = None
        self.on_local_path = False
        self.lap1_target = (float("nan"), float("nan"))
        self.path_reach = 0.0
        self.pushed = False
        self.path_reach = 0.0

        self.pub_throttle = self.create_publisher(Float32, NS + "throttle_command", QOS)
        self.pub_steering = self.create_publisher(Float32, NS + "steering_command", QOS)
        self.pub_cones = self.create_publisher(PoseArray, "/feb/cones", 10)
        self.pub_map = self.create_publisher(PoseArray, "/feb/map", 10)
        # the team's cone ordering, live on the growing map: lap one follows its local path
        self.rungs = {"blue": None, "yellow": None, "t": -1e9}
        self.create_subscription(PoseArray, "/feb/cone_order/blue", lambda m: self.on_rungs("blue", m), 10)
        self.create_subscription(PoseArray, "/feb/cone_order/yellow", lambda m: self.on_rungs("yellow", m), 10)
        self.lap_one_scans = [0, 0]              # scans of lap one on the team's local path, on the reactive follower
        self.order_client = self.create_client(OrderCones, "/feb/order_cones") if OrderCones is not None else None
        self.order_future = None
        self.pub_line = self.create_publisher(Path, "/feb/raceline", QOS_LATCHED)
        self.pub_pose = self.create_publisher(PoseStamped, "/feb/pose", 10)
        self.pub_status = self.create_publisher(String, "/feb/status", 10)      # phase, what steers, active fallbacks
        self.status_t = -1e9
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
        if hist and angle < hist[-1][1] - 50.0:      # the counter was reset (a small decrease is the car reversing)
            hist.clear()
        hist.append((t, angle))
        self.enc_now[msg.header.frame_id] = (t, angle)
        while len(hist) > 2 and t - hist[0][0] > self.p["speed_window"]:
            hist.pop(0)
        if len(hist) < 2 or t - hist[0][0] < 1e-3:
            return
        v = (angle - hist[0][1]) / (t - hist[0][0]) * WHEEL_RADIUS
        if -5.0 < v < 25.0:                          # signed: backing up must move the dead reckoning backwards too
            self.speed = 0.5 * self.speed + 0.5 * v

    # ------------------------------------------------------------ main loop, per scan

    def on_scan(self, msg):
        p = self.p
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = min(max(t - self.last_t, 0.01), 0.5) if self.last_t else 0.1
        self.last_t = t
        if self.yaw is None:
            return
        clusters = self.perception.clusters(msg)
        self.clusters_now, self.last_dt = clusters, dt
        cones = self.perception.fuse(clusters, self.speed, self.yaw, dt, self.yaw_rate)
        self.publish_cones(msg.header, cones)

        if self.mode == "RACING":
            steer, throttle = self.race_scan(t, dt, clusters, cones)
            if self.halted:
                steer, throttle = 0.0, 0.0
            self.steer, self.throttle = steer, throttle
            if t - self.status_t > 0.5:
                self.status_t = t
                self.pub_status.publish(String(data=self.status_line(t, True)))
            self.pub_throttle.publish(Float32(data=float(throttle)))
            self.pub_steering.publish(Float32(data=float(steer / MAX_STEER)))
            self.publish_pose(msg.header)
            return

        # the mapping lap. Dead reckoning: wheel speed, slew-limited to what the car can do, and
        # only while the lidar scene actually moves past us (at the mapping lap's pace the
        # wheels and the body agree, and a car stopped against a cone must not map on)
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
        # "cones in view" counts what the lidar sees, coloured or not: at a hairpin the camera can
        # look at nothing for seconds while the lidar has the whole corner
        if len(cones) >= 2 or sum(1 for x, y in clusters if x > -0.5 and math.hypot(x, y) < 6.0) >= 2:
            self.last_seen_t = t
        if self.mode == "ORDERING":
            # the wait is driven as the lap was: on the local path, whose rungs do not age now
            # that the map is closed. (The reactive follower alone swerved here on a track whose
            # lanes run side by side, and put the car on a cone before the race began.)
            steer, throttle = self.follow_rungs(clusters, dt, p["order_timeout"]) or self.follow_local(clusters, cones, dt)
            self.ordering_step()
        elif self.mode == "MAPPING":
            self.mapping_step(z_rel, colours, weights)
            if len(self.slam.lhat) >= 4:
                self.pub_map.publish(self.map_message())      # the growing map, every scan: the live ordering starts at the car
            local = self.follow_rungs(clusters, dt)
            self.lap_one_scans[0 if local is not None else 1] += 1
            self.on_local_path = local is not None
            steer, throttle = local if local is not None else self.follow_local(clusters, cones, dt)
            if self.p["debug"]:
                self.get_logger().info("lap one: %s, target (%.1f, %.1f) in the car frame, reach %.1f m, steer %.2f, throttle %.2f"
                                       % ("local path" if local is not None else "reactive", *self.lap1_target, self.path_reach, steer, throttle))
            if t - self.last_seen_t > p["lost_after"]:
                steer, throttle = self.recover(t, dt)
            else:
                self.seen_pose, self.recovering = self.pose.copy(), False
        # stuck on a cone (wheels may spin, so watch the lidar scene): push once, then stop
        if moving and not self.no_target:
            self.stalled_since = None
            self.pushed = False
        if self.push_until is not None:
            if t < self.push_until:
                throttle = max(throttle, self.p["push_throttle"])
            else:
                self.push_until = None
        elif self.throttle > 0.02 or throttle > 0.02 or self.no_target:
            # stuck on a cone, or the follower stopped with nothing to follow: one push (a cone
            # under the bumper gives way), and if that does not free the car it stops, as the
            # real car would; it cannot back up
            self.stalled_since = self.stalled_since or t      # sticky: a throttle dip does not reset it
            if t - self.stalled_since > 2.0 and self.push_until is None and not self.pushed:
                self.get_logger().warn("not moving: pushing harder for a moment")
                self.push_until = t + 1.5
                self.pushed = True
            elif t - self.stalled_since > 6.0:
                self.halt("nothing to follow" if self.no_target else "stuck on something")
        if self.halted:
            steer, throttle = 0.0, 0.0
        self.steer, self.throttle = steer, throttle
        if t - self.status_t > 0.5:
            self.status_t = t
            self.pub_status.publish(String(data=self.status_line(t, moving)))
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
        snap_radius = max(p["snap_radius"], 0.06 * self.travelled)       # the drift grows with the lap
        if self.travelled > p["min_lap_length"] + 20.0 and np.linalg.norm(self.pose - self.start[0]) < snap_radius and len(z_rel) >= 4:
            # near the start again after a lap: a unique translation search of the cones in view
            # against the landmarks mapped around the start closes the loop whatever the drift
            # (an ICP with a wide net settles on a wrong local fit when the drift is metres; the
            # search only answers when one placement fits clearly better than every other)
            guess = self.pose + self.since_key
            found = self.slam.relocalise(z_rel, colours, near=self.start[0], radius=snap_radius)
            if found is not None and np.linalg.norm(found - guess) > 0.05:
                t_snap = found - guess
                self.since_key = self.since_key + t_snap
                self.snap_t = self.last_t
                self.get_logger().info("loop closure at the start gate: pose corrected by %.2f m, %.1f m from the start"
                                       % (np.linalg.norm(t_snap), np.linalg.norm(found - self.start[0])))
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
        # back at the start? Either the pose says so, or (the real loop closure) the cones in
        # view are the very first ones mapped: then the drift of a long lap does not matter
        d0 = np.linalg.norm(self.pose - self.start[0])
        close = p["lap_close_dist"] * (2.5 if self.last_t - self.snap_t < 3.0 else 1.0)   # the gate match itself says we are here
        early = sum(1 for k in self.slam.last_matched if k < p["closure_landmarks"])
        if self.travelled > p["min_lap_length"] + 20.0 and early >= 3 and d0 < max(p["snap_radius"], 0.06 * self.travelled) and math.cos(self.yaw - self.start[1]) > 0.5:
            self.get_logger().info("loop closure: %d of the first cones in view again, %.1f m from the start" % (early, d0))
            self.finish_mapping()
        elif self.travelled > p["min_lap_length"] and d0 < close and math.cos(self.yaw - self.start[1]) > 0.5:
            self.finish_mapping()
        elif self.keyframes % 10 == 0:
            self.get_logger().info("mapping: %d keyframes, %d landmarks, travelled %.1f m" % (self.keyframes, len(self.slam.lhat), self.travelled))

    def finish_mapping(self):
        """The map is complete: freeze it, repair the colours from the mapping-lap path, and
        ask the team's cone ordering for the track. The answer comes back on a later scan
        (the car keeps following the local line meanwhile); without the service, or after
        `order_timeout`, the built-in boundary walk takes over."""
        p = self.p
        self.slam.freeze(int(p["min_seen"]))
        self.lap_times.append(self.last_t - self.lap_start_t)
        self.slam.colour_override = repair_by_path(self.slam.lhat, self.slam.colour, self.slam.xhat)
        self.map_closed_t = time.time()
        if self.order_client is not None and self.order_client.service_is_ready():
            req = OrderCones.Request()
            req.car.position.x, req.car.position.y = float(self.pose[0]), float(self.pose[1])
            req.car.orientation.z, req.car.orientation.w = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
            req.map = self.map_message()
            self.order_future = self.order_client.call_async(req)
            self.order_deadline = self.last_t + p["order_timeout"]
            self.mode = "ORDERING"
            a, b = self.lap_one_scans
            self.get_logger().info("map closed after %.1f s: %d cones, lap one %.0f%% on the team's local path; asking the team's cone ordering"
                                   % (self.lap_times[-1], len(self.slam.lhat), 100.0 * a / max(a + b, 1)))
        else:
            self.get_logger().info("map closed after %.1f s: %d cones; no cone ordering service, using the boundary walk" % (self.lap_times[-1], len(self.slam.lhat)))
            self.complete_track(None)

    def ordering_step(self):
        """Waiting for the cone ordering: take its answer when it lands, or give up on it."""
        if self.order_future.done():
            res = self.order_future.result()
            rungs = None
            if res is not None and len(res.blue.poses) >= 8:
                b = np.array([[q.position.x, q.position.y] for q in res.blue.poses])
                y = np.array([[q.position.x, q.position.y] for q in res.yellow.poses])
                n = min(len(b), len(y))
                mid = (b[:n] + y[:n]) / 2.0
                covered = float(np.sum(np.linalg.norm(np.diff(mid, axis=0), axis=1)))
                # a closed loop of rungs round the whole lap: as long as the distance driven, or
                # as long as the map itself says a lap is (the distance driven includes any
                # excursion on the way, so it can overstate the lap by half)
                if res.closed and covered >= 0.7 * min(self.travelled, self.map_lap_length()):
                    rungs = (b, y)
                    self.get_logger().info("cone ordering: %d rungs over %.0f m, closed track" % (n, covered))
                else:
                    self.get_logger().warn("cone ordering gave %s covering %.0f m of a %.0f m lap; using the boundary walk"
                                           % ("a closed loop" if res.closed else "an open path", covered, self.travelled))
            else:
                self.get_logger().warn("cone ordering returned nothing usable; using the boundary walk")
            self.complete_track(rungs)
        elif self.last_t > self.order_deadline:
            self.get_logger().warn("cone ordering did not answer in time; using the boundary walk")
            self.complete_track(None)

    def map_lap_length(self):
        """The lap as the map tells it: cones of one colour stand one spacing apart along their
        boundary, so half the cones times the typical spacing between a cone and its nearest
        neighbour of the same colour is the length of a boundary."""
        L, C = self.slam.lhat, self.slam.colour
        spacings = []
        for c in (BLUE, YELLOW):
            P = L[C == c]
            if len(P) >= 4:
                d = np.linalg.norm(P[:, None, :] - P[None, :, :], axis=2)
                np.fill_diagonal(d, np.inf)
                spacings.append(np.median(d.min(axis=1)) * len(P))
        return float(np.mean(spacings)) if spacings else self.travelled

    def complete_track(self, rungs):
        p = self.p
        lap = self.lap_times[-1]
        track = track_from_rungs(rungs[0], rungs[1], self.slam.colour_override, p["sample_step"], cones=self.slam.lhat) if rungs is not None else None
        if track is not None:
            why = self.track_fault(track)
            if why:
                self.get_logger().warn("the ordering's track %s; using the boundary walk" % why)
                track, rungs = None, None
        if track is None:
            track = build_track(self.slam.lhat, self.slam.colour, self.start[0], (math.cos(self.start[1]), math.sin(self.start[1])), p["sample_step"],
                                path=self.slam.xhat)
            if track is None:
                self.get_logger().error("map has too few cones of a colour; keep following the local line")
                self.slam.frozen = False
                self.slam.colour_override = None
                self.mode = "MAPPING"
                return
            self.slam.colour_override = track["colour"]
            n_b, n_y = int(np.sum(track["colour"] == BLUE)), int(np.sum(track["colour"] == YELLOW))
            if len(track["left"]) < 0.9 * n_b or len(track["right"]) < 0.9 * n_y:
                self.get_logger().warn("boundaries leave cones out: blue %d of %d, yellow %d of %d" % (len(track["left"]), n_b, len(track["right"]), n_y))
        t0 = self.map_closed_t
        line, _ = min_curvature(track["centre"], track["normal"], track["half_width"], p["car_half_width"], p["margin"], p["curvature_reg"],
                                cones=self.slam.lhat)
        v, _ = speed_plan(line, p["v_max"] * p["race_speed_scale"], p["a_lat"] * p["corner_scale"] ** 2, p["a_acc"], p["a_brake"],
                          curv_window=p["curv_window"])
        self.track, self.raceline, self.race_v = track, line, v
        self.ctl.reset()
        self.ctl.set_raceline(line, v)
        self.body.v = max(self.speed, 0.0)
        self.enc_prev, self.race_yaw, self.race_s, self.icp_track = None, self.yaw, None, []
        self.matched_run, self.race_ready_t, self.still_since = 0, None, None
        self.pinned_t, self.start_heading, self.corner_pace = None, None, float(np.min(v))
        self.mode = "RACING"
        self.lap_start_t = self.last_t
        self.good_loc_t = self.last_seen_t = self.reloc_t = self.last_t
        plan_lap = float(np.sum(np.diff(self.ctl.line.s) / np.maximum(v, 0.2)))      # what driving the plan exactly would take
        self.get_logger().info("track ready %.0f ms after the map closed (lap %.1f s): %s, raceline %.1f m, %d points, v %.1f..%.1f m/s, the plan is a %.2f s lap -> RACING with %s"
                               % (1000 * (time.time() - t0), lap, "team cone ordering" if rungs is not None else "boundary walk",
                                  self.ctl.line.length, len(line), v.min(), v.max(), plan_lap, "MPC" if self.mpc else "pure pursuit"))
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

    def chain_step(self, cones):
        """Largest gap between consecutive cones of one colour when ordering them: twice the
        spacing of the cones in view (one may be missing), never below the configured minimum.
        Tracks are laid out with anything from 1 m to 5 m between cones."""
        gaps = []
        for i, (x, y, c) in enumerate(cones):
            same = [math.hypot(x - u, y - v) for j, (u, v, d) in enumerate(cones) if j != i and d == c]
            if same:
                gaps.append(min(same))
        if len(gaps) < 2:
            return self.p["chain_step"]
        return float(np.clip(2.0 * np.median(gaps), self.p["chain_step"], 6.0))

    def local_centreline(self, cones):
        """The middle of the corridor in view, as a dense line of points: the blue chain is
        walked every 0.3 m and each step is paired with the nearest point of the yellow
        chain that lies across the track to its RIGHT (right of the chain's direction of
        travel), never the nearest yellow regardless of side: at a hairpin the nearest yellow
        is often on the far branch and the midpoint would cut through the inner cones and turn
        the car around. Walking the chain rather than pairing cone to cone matters at a
        corner: with one midpoint per cone the line stays straight until the cone past the
        corner, and the car turns a length too late. Where one chain has no partner, its points
        are offset by half the track width."""
        # the orange start cones stand on the boundary lines: for driving they count as the
        # colour of the side they are on
        cones = [(x, y, (BLUE if y > 0.0 else YELLOW) if c == ORANGE else c) for x, y, c in cones]
        # only the cones nearby: a cone's colour is reliable within a few metres and not
        # beyond (a far wall of cones read as the wrong colour turned the chains the wrong way
        # at a corner), and the target is one lookahead away
        cones = [c for c in cones if c[0] > -1.0 and math.hypot(c[0], c[1]) < self.p["follow_range"]]
        step = self.chain_step(cones)
        left = self.chain([c for c in cones if c[2] == BLUE], step)
        right = self.chain([c for c in cones if c[2] == YELLOW], step)
        w, half = self.p["track_width"], self.p["track_width"] / 2.0
        L, R = self.walk_chain(left), self.walk_chain(right)
        # walked from both sides: the inner chain of a corner is a sharp vertex and the outer
        # one a wide arc, and midpoints made from one side alone follow that side's shape
        points = self.midpoints(L, R, 1.0, w, half) + self.midpoints(R, L, -1.0, w, half)
        return sorted([q for q in points if q[0] > 0.0], key=lambda q: math.hypot(*q))

    @staticmethod
    def midpoints(A, B, side, w, half):
        """Midpoints between the walked chain A and the walked chain B, one per step of A: the
        step is paired with the nearest point of B that lies across the track to its right
        (side +1, A blue) or left (side -1, A yellow); a step with no partner is offset by half
        the track width."""
        points = []
        for (px, py), (tx, ty) in A:
            nx, ny = side * ty, -side * tx                      # normal towards the other chain
            best, best_score = None, None
            for (qx, qy), _ in B:
                dx, dy = qx - px, qy - py
                d = math.hypot(dx, dy)
                across = dx * nx + dy * ny                      # how far across the track
                along = abs(dx * tx + dy * ty)                   # how far along it
                if d > 1.3 * w or across < 0.4 * d:
                    continue
                score = along + 0.5 * d
                if best_score is None or score < best_score:
                    best, best_score = (qx, qy), score
            points.append((px + half * nx, py + half * ny) if best is None else ((px + best[0]) / 2.0, (py + best[1]) / 2.0))
        return points

    @staticmethod
    def walk_chain(chain, step=0.3):
        """Points every `step` along a chain of cones, each with the chain's direction there."""
        if not chain:
            return []
        if len(chain) == 1:
            return [((chain[0][0], chain[0][1]), (1.0, 0.0))]
        out = []
        for a, b in zip(chain, chain[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            d = math.hypot(dx, dy)
            tx, ty = dx / d, dy / d
            n = max(int(d / step), 1)
            out += [((a[0] + tx * d * k / n, a[1] + ty * d * k / n), (tx, ty)) for k in range(n)]
        out.append(((chain[-1][0], chain[-1][1]), out[-1][1]))
        return out

    def on_rungs(self, side, msg):
        self.rungs[side] = np.array([[q.position.x, q.position.y] for q in msg.poses]).reshape(-1, 2)
        self.rungs["t"] = self.last_t if self.last_t is not None else -1e9

    def follow_rungs(self, clusters, dt, max_age=None):
        """Lap one the way the car does it: the cone ordering runs live on the growing map and
        its rungs give a local path (the rung midpoints, map frame). Pure pursuit on the first
        midpoint ahead of the car beyond the lookahead. None when the rungs are stale or empty,
        and the reactive follower takes over."""
        p = self.p
        b, y = self.rungs["blue"], self.rungs["yellow"]
        if b is None or y is None or self.last_t - self.rungs["t"] > (max_age or p["rung_max_age"]):
            return None
        n = self.corridor_prefix(b, y)
        if n < 2:
            return None
        mid = (b[:n] + y[:n]) / 2.0 - self.pose
        c, s_ = math.cos(-self.yaw), math.sin(-self.yaw)
        local = np.column_stack([c * mid[:, 0] - s_ * mid[:, 1], s_ * mid[:, 0] + c * mid[:, 1]])   # car frame
        ahead = [q for q in local if q[0] > 0.2]
        if not ahead:
            return None
        far = [q for q in ahead if math.hypot(*q) >= p["local_path_lookahead"]]
        target = far[0] if far else max(ahead, key=lambda q: q[0])     # the path may end just ahead on a young map
        reach = max(q[0] for q in ahead)                               # how far the known path goes
        self.path_reach = reach
        if target[0] < 0.8 or reach < p["local_path_min_reach"]:
            return None                                                # too short to steer by: the reactive follower knows better here
        self.no_target = False
        self.target_t = self.last_t
        self.lap1_target = (float(target[0]), float(target[1]))
        ld = max(math.hypot(*target), 0.3)
        alpha = math.atan2(target[1], target[0])
        wanted = self.push_from_cones(math.atan(2.0 * WHEELBASE * math.sin(alpha) / ld), clusters)
        step = (wanted - self.steer) * min(1.0, dt / p["steer_tau"])
        steer = self.steer + float(np.clip(step, -MAX_STEER_RATE * dt, MAX_STEER_RATE * dt))
        v_goal = max(p["map_min_speed"], p["map_speed"] * (1.0 - 0.7 * abs(steer) / MAX_STEER) * min(1.0, reach / 4.0))
        return steer, self.throttle_law(self.smooth_speed(v_goal, dt), dt)

    def push_from_cones(self, wanted, clusters):
        """A cone dead ahead pushes the wheel away from it. Only dead ahead, and never past
        straight: the inner cones of a tight bend sit ahead and to the side, and a push that
        turned the car the other way sent it out through the outer line."""
        p = self.p
        pushed = wanted
        for x, y in clusters:
            if 0.0 < x < p["avoid_range"] and abs(y) < 0.3:
                pushed -= math.copysign(0.4, y) * (1.0 - x / p["avoid_range"])
        if wanted * pushed < 0.0:
            pushed = 0.0
        return float(np.clip(pushed, -MAX_STEER, MAX_STEER))

    @staticmethod
    def at_distance(a, b, r):
        """The point of segment a-b at distance r from the origin (b is beyond r, a within)."""
        ax, ay = a
        dx, dy = b[0] - ax, b[1] - ay
        qa, qb, qc = dx * dx + dy * dy, 2.0 * (ax * dx + ay * dy), ax * ax + ay * ay - r * r
        disc = qb * qb - 4.0 * qa * qc
        if qa < 1e-9 or disc < 0.0:
            return b
        t = float(np.clip((-qb + math.sqrt(disc)) / (2.0 * qa), 0.0, 1.0))
        return (ax + t * dx, ay + t * dy)

    def recover(self, t, dt):
        """Nothing in view on the mapping lap: the car has nosed out of the corridor. It turns
        back towards the last place the cones were in view and crawls there. That is ordinary
        forward driving, which the real car can do; it never reverses, and if the cones do not
        come back within `recover_for` it stops and says so."""
        p = self.p
        back = self.seen_pose - self.pose
        c, s_ = math.cos(-self.yaw), math.sin(-self.yaw)
        lx, ly = c * back[0] - s_ * back[1], s_ * back[0] + c * back[1]
        if t - self.last_seen_t > p["lost_after"] + p["recover_for"] or math.hypot(lx, ly) < 0.3:
            self.halt("nothing in view for %.0f s on the mapping lap" % (t - self.last_seen_t))
            return 0.0, 0.0
        if not self.recovering:
            self.recovering = True
            self.get_logger().warn("no cones in view: turning back towards them, %.1f m away" % math.hypot(lx, ly))
        # towards the spot when it is ahead, otherwise turn as hard as the car can, still forwards
        wanted = math.atan2(ly, lx) if lx > 0.3 else math.copysign(MAX_STEER, ly if ly != 0.0 else 1.0)
        wanted = float(np.clip(wanted, -MAX_STEER, MAX_STEER))
        step = (wanted - self.steer) * min(1.0, dt / p["steer_tau"])
        steer = self.steer + float(np.clip(step, -MAX_STEER_RATE * dt, MAX_STEER_RATE * dt))
        return steer, self.throttle_law(p["map_min_speed"], dt)

    def corridor_prefix(self, b, y):
        """How many rungs, from the car on, still look like the track: the ordering is built
        for a complete map, and on the growing map its path past the last well-mapped cones
        fans out (rungs several track widths long, reaching across to another section) or
        zigzags (the slope field has too few cones to point one way). Rungs are trusted up to
        the first one that is much longer or shorter than the track, or whose midpoint turns
        more than the car could between two rungs."""
        w = self.p["track_width"]
        n = min(len(b), len(y))
        mid = (b[:n] + y[:n]) / 2.0
        # the direction of the path is judged over a metre at a time (the rungs come every
        # 0.2 m and their ends jitter, so neighbouring midpoints say nothing about direction)
        back = 0
        for i in range(n):
            if not 0.5 * w <= np.linalg.norm(b[i] - y[i]) <= 1.6 * w:
                return i
            while back < i and np.linalg.norm(mid[i] - mid[back]) > 1.0:
                back += 1
            if back > 0 and np.linalg.norm(mid[i] - mid[back]) > 0.8:
                d1, d0 = mid[i] - mid[back], mid[back] - mid[max(back - (i - back), 0)]
                if np.linalg.norm(d0) > 0.8:
                    turn = math.atan2(d1[0] * d0[1] - d1[1] * d0[0], d1 @ d0)
                    if abs(turn) > math.radians(100):
                        return i
        return n

    def follow_local(self, clusters, cones, dt):
        p = self.p
        line = self.local_centreline([(x, y, c) for x, y, c, w in cones])
        if p["debug"]:
            self.get_logger().info("reactive: cones %s | line %s" % (" ".join("%s(%.1f,%.1f,%.1f)" % ("?BYO"[int(c)], x, y, w) for x, y, c, w in cones),
                                                                    " ".join("(%.1f,%.1f)" % q for q in line[:8])))
        # the target is the point of the centreline exactly one lookahead away, interpolated
        # between its (cone-spaced) points: the first point beyond the lookahead can be three
        # metres out at a corner, and pure pursuit on it turns far too little
        # the lookahead grows with speed (a fixed short one weaves at 1.2 m/s), never below
        # the configured one, which is what turns the car in at a tight corner
        la = float(np.clip(1.2 * max(self.speed, 0.0), p["lookahead"], 1.8))
        target, prev = None, (0.0, 0.0)
        for q in line:
            if math.hypot(*q) >= la:
                target = self.at_distance(prev, q, la)
                break
            prev = q
        if target is None and line and math.hypot(*line[-1]) > 0.5:
            target = line[-1]                      # a line that ends within reach of the bumper is no target
        if target is None:
            # a scan with nothing ahead (the big start cones fill the view, the cones of a corner
            # not yet in the camera's view): hold the wheel and crawl for a few seconds before
            # giving up; the corner's cones come into view as the car turns
            if self.last_t - self.target_t < 4.0:
                self.no_target = False
                return self.steer, self.throttle_law(p["map_min_speed"], dt)
            self.no_target = True
            self.lap1_target = (float("nan"), float("nan"))
            self.integral = 0.0
            return self.steer * 0.5, 0.0
        self.no_target = False
        self.target_t = self.last_t
        self.lap1_target = (float(target[0]), float(target[1]))
        ld = max(math.hypot(*target), 0.3)
        alpha = math.atan2(target[1], target[0])
        wanted = self.push_from_cones(math.atan(2.0 * WHEELBASE * math.sin(alpha) / ld), clusters)
        step = (wanted - self.steer) * min(1.0, dt / p["steer_tau"])
        steer = self.steer + float(np.clip(step, -MAX_STEER_RATE * dt, MAX_STEER_RATE * dt))
        v_goal = max(p["map_min_speed"], p["map_speed"] * (1.0 - 0.7 * abs(steer) / MAX_STEER))
        return steer, self.throttle_law(self.smooth_speed(v_goal, dt), dt)

    def smooth_speed(self, v_goal, dt):
        """Slow down at once, speed up gently. The mapping lap's speed is scaled by how far the
        known path reaches and by how hard the wheel is turned, and both jump from scan to scan
        as rungs come and go; a car that chases those jumps lurches forward and coasts. Braking
        is never delayed, so the car still slows the moment the path ahead shortens."""
        if self.v_goal is None or v_goal < self.v_goal:
            self.v_goal = v_goal
        else:
            self.v_goal += (v_goal - self.v_goal) * min(1.0, dt / self.p["speed_tau"])
        return self.v_goal

    def throttle_law(self, v_t, dt):
        p = self.p
        # the target may only change as fast as the car can accelerate or brake. The follower's
        # speed jumps whenever the known path shortens or the wheel turns, and a 0.4 m/s step
        # asks for 4 m/s^2 in one scan: the car surges, overshoots, coasts into the motor's
        # idle brake and surges again, which is the lurching of the mapping lap.
        if v_t > 0.0:
            v_t = float(np.clip(v_t, self.v_cmd - p["a_brake"] * dt, self.v_cmd + p["a_acc"] * dt))
        self.v_cmd = max(v_t, 0.0)
        ff = v_t / p["speed_per_throttle"]
        err = v_t - max(self.speed, 0.0)
        self.integral = float(np.clip(self.integral + err * dt, -0.8, 0.8))
        bound = ff if self.speed < 0.3 else 0.5 * ff
        trim = float(np.clip(p["throttle_kp"] * err + p["throttle_ki"] * self.integral, -bound, bound))
        wanted = float(np.clip(ff + trim, 0.0, 1.0))
        if v_t > 0.0:
            # below about 0.03 the motor brakes rather than coasts, so a trim that dips there
            # while a positive speed is wanted takes speed off the car for no reason
            wanted = max(wanted, 0.7 * ff)
        if v_t > 0.0 and self.speed < 0.25:
            # getting rolling from rest needs more than the steady-state throttle, and more
            # again with the wheels turned (a crawl at full lock stalls on 0.03). It fades out
            # well below the crawl speed, so it never fights the regulator at its own setpoint
            floor = p["throttle_start"] * (1.0 + 0.5 * abs(self.steer) / MAX_STEER)
            wanted = max(wanted, floor * float(np.clip((0.25 - self.speed) / 0.2, 0.0, 1.0)))
        slew = p["throttle_slew"] * dt
        return float(np.clip(wanted, self.throttle - slew, self.throttle + slew)) if self.throttle > 0.0 else wanted

    # ------------------------------------------------------------ racing

    def wheel_speed(self):
        """The wheels' speed over the last scan, from the encoders' own angles and stamps."""
        if len(self.enc_now) < 2:
            return max(self.speed, 0.0)
        stamp = max(e[0] for e in self.enc_now.values())
        angle = float(np.mean([e[1] for e in self.enc_now.values()]))
        prev, self.enc_prev = self.enc_prev, (stamp, angle)
        if prev is None or stamp - prev[0] < 1e-3 or abs(angle - prev[1]) > 200.0:
            return max(self.speed, 0.0)
        return (angle - prev[1]) / (stamp - prev[0]) * WHEEL_RADIUS

    def race_scan(self, t, dt, clusters, cones):
        """One scan of a racing lap: where the car is, then what it should do."""
        p = self.p
        # speed: the wheels, put through the tyre. The encoders count the wheels, and the
        # throttle sets the wheel speed at once, so on their own they are the command coming
        # back; the tyre curve says how the body follows (vehicle.BodySpeed)
        v = self.body.update(self.wheel_speed(), dt, self.yaw_rate, self.delta)
        self.odo_speed = v
        # dead reckoning: the rear axle moves along the heading, and the lidar, which is the
        # point the map match places, swings round it as the car turns
        mid = self.race_yaw + 0.5 * wrap(self.yaw - self.race_yaw)
        ahead = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        self.pose = (self.pose - AXLE_TO_LIDAR * np.array([math.cos(self.race_yaw), math.sin(self.race_yaw)])
                     + v * dt * np.array([math.cos(mid), math.sin(mid)]) + AXLE_TO_LIDAR * ahead)
        self.race_yaw = self.yaw
        self.travelled += v * dt
        # the map: every lidar cone within range counts, coloured or not
        obs = [(x, y, c) for x, y, c, w in cones] + [(x, y, UNKNOWN) for x, y in self.perception.unknown]
        obs = [o for o in obs if math.hypot(o[0], o[1]) < p["loc_range"]]
        z_rel = np.array([[ahead[0] * x - ahead[1] * y, ahead[1] * x + ahead[0] * y] for x, y, c in obs]).reshape(-1, 2)
        colours = np.array([c for _, _, c in obs], dtype=int)
        matched = self.localise(t, z_rel, colours)
        self.matched_run = self.matched_run + 1 if matched >= 4 else 0
        if self.race_ready_t is None and self.matched_run >= 3:
            self.race_ready_t = t
            self.get_logger().info("map match confirmed: %d of %d cones, the pace of the slowest corner from here" % (matched, len(z_rel)))
        self.still_since = (self.still_since or t) if self.scene_still() and self.body.v > 0.5 else None
        if len(cones) >= 2 or sum(1 for x, y in clusters if x > -0.5 and math.hypot(x, y) < 6.0) >= 2:
            self.last_seen_t = t
        if matched >= 3 or len(z_rel) < 3:
            self.good_loc_t = t                     # matched, or too little in view to judge
        blind, lost = t - self.last_seen_t, t - self.good_loc_t
        if t - self.loc_log_t > 2.0:
            self.loc_log_t = t
            self.get_logger().info("racing: %d cones in view, %d matched, speed %.1f of %.1f m/s, %.2f m off the line, mpc %.0f ms"
                                   % (len(z_rel), matched, v, self.ctl.info.get("v_plan", 0.0), self.ctl.info.get("off", 0.0), self.ctl.solve_ms))
        if max(blind, lost) > p["give_up_after"]:
            self.halt("no cones in view for %.0f s" % blind if blind > lost else "the map has not matched for %.0f s" % lost)
            return 0.0, 0.0
        if self.stuck(t):
            self.halt("not moving with the wheels turning")
            return 0.0, 0.0
        searching = self.race_ready_t is None and matched < 3       # the race has not found itself on the map yet
        if (searching or lost > p["loc_lost_after"]) and len(z_rel) >= 4 and t - self.reloc_t > (0.3 if searching else 1.0):
            self.reloc_t = t                        # look for the car on the map: near where it thinks it is first
            found = self.slam.relocalise(z_rel, colours, near=self.pose, radius=3.0) if searching else self.slam.relocalise(z_rel, colours)
            if found is not None:
                self.get_logger().info("relocalised: map position moved %.1f m" % np.linalg.norm(found - self.pose))
                self.pose, self.good_loc_t = found, t
                self.ctl.s = None
        # how much of the plan to ask for. The mapping lap's pace until the map match is
        # confirmed (the mapping lap can end beside a cone, a little off the map). Then the pace
        # of the plan's slowest corner, which needs no braking point, until the first corner is
        # behind the car: where the lap closes the map has a seam, and on a straight of evenly
        # spaced cones the match sits as well one cone along, so the car may believe it is a
        # cone's spacing from where it is, and at racing pace it then brakes that much late. A
        # corner pins it. From there up to all of the plan over `start_ramp`; a share of it while
        # the car drives on dead reckoning alone
        if self.race_ready_t is None:
            self.ctl.speed_cap = p["map_speed"]
        elif self.pinned_t is None:
            self.ctl.speed_cap = self.corner_pace
        else:
            ramp = float(np.clip((t - self.pinned_t) / p["start_ramp"], 0.0, 1.0))
            self.ctl.speed_cap = self.corner_pace + ramp * (p["v_max"] - self.corner_pace) if ramp < 1.0 else 99.0
        self.ctl.speed_scale = 1.0 if max(blind, lost) < p["lost_after"] else p["blind_speed"]
        rear = self.pose - AXLE_TO_LIDAR * ahead
        steer, throttle = self.ctl.step(t, rear[0], rear[1], self.yaw, v, self.delta)
        if self.ctl.info.get("Z") is not None:
            self.publish_prediction(self.ctl.info["Z"])
        s, length = self.ctl.s, self.ctl.line.length
        if self.race_s is not None and self.race_s > 0.8 * length and s < 0.2 * length:
            self.lap_times.append(t - self.lap_start_t)
            self.lap_start_t = t
            self.get_logger().info("lap %d: %.2f s (mpc %.0f ms/solve)" % (len(self.lap_times) - 1, self.lap_times[-1], self.ctl.solve_ms))
        self.race_s = s
        if self.pinned_t is None and self.race_ready_t is not None:
            heading = float(self.ctl.line.at(s)[2])
            self.start_heading = heading if self.start_heading is None else self.start_heading
            if abs(wrap(heading - self.start_heading)) > p["pin_turn"]:
                self.pinned_t = t
                self.get_logger().info("first corner behind the car, racing pace from here")
        return steer, throttle

    def localise(self, t, z_rel, colours):
        """Correct the dead-reckoned position against the frozen map. Cones line the track, so a
        match says where the car is across the track precisely and along it less so (slide the
        estimate one cone along and the boundary fits as well); dead reckoning is the opposite.
        Each scan takes a share of the correction, a smaller one along the heading, and never
        more than `loc_max_step`: the car cannot teleport."""
        p = self.p
        corrected, matched = self.slam.localise(self.pose, z_rel, colours, subset=self.landmarks_near())
        if matched >= 3:
            delta = corrected - self.pose
            c, sn = math.cos(self.yaw), math.sin(self.yaw)
            along = p["loc_gain_along"] * (delta[0] * c + delta[1] * sn)
            across = p["loc_gain_across"] * (-delta[0] * sn + delta[1] * c)
            step = np.array([along * c - across * sn, along * sn + across * c])
            n = float(np.linalg.norm(step))
            if n > p["loc_max_step"]:
                step *= p["loc_max_step"] / n
            self.pose = self.pose + step
            self.icp_track.append((t, corrected))
            self.icp_track = [e for e in self.icp_track if t - e[0] < 4.0]
        return matched

    def scene_still(self):
        """True when the lidar sees the same cones in the same places as a scan ago: most of
        them within 4 cm. (Matching cones between scans says little about how fast the car
        moves, at speed one cone is taken for the next; that nothing has moved at all is a
        statement it can make.)"""
        cur = np.array(self.clusters_now, float).reshape(-1, 2)
        prev, self.prev_clusters = self.prev_clusters, cur
        if prev is None or len(prev) < 3 or len(cur) < 3:
            return False
        near = np.min(np.linalg.norm(cur[:, None, :] - prev[None, :, :], axis=2), axis=1)
        return bool(np.sum(near < 0.04) >= max(3, 0.7 * len(cur)))

    def stuck(self, t):
        """True when the car is against something: the wheels turn, and either the lidar scene
        has stood still for three seconds, or the map match has placed the car within 0.3 m of
        the same spot for that long."""
        if self.body.v < 0.5 and self.throttle < 0.05:
            return False
        if self.still_since is not None and t - self.still_since > 3.0:
            return True
        old = [e for e in self.icp_track if t - e[0] > 3.0]
        recent = [e for e in self.icp_track if t - e[0] <= 3.0]
        if not old or len(recent) < 10:
            return False
        pts = np.array([e[1] for e in recent] + [old[-1][1]])
        return bool(np.max(np.linalg.norm(pts - pts[-1], axis=1)) < 0.3)

    def landmarks_near(self):
        """The landmarks within reach of the stretch of raceline around the car: on a layout
        whose sections run side by side, the cones of the neighbouring section must not be
        candidates, or the match slides across to them. Everything when the map match is lost."""
        if self.ctl.s is None or self.last_t - self.good_loc_t > self.p["loc_lost_after"]:
            return None
        n = len(self.raceline)
        i0 = int(self.ctl.s / self.p["sample_step"])
        idx = np.arange(i0 - int(8.0 / self.p["sample_step"]), i0 + int(15.0 / self.p["sample_step"])) % n
        stretch = self.raceline[idx]
        d = np.min(np.linalg.norm(self.slam.lhat[:, None, :] - stretch[None, :, :], axis=2), axis=1)
        near = np.flatnonzero(d < self.p["loc_corridor"])
        return near if len(near) >= 4 else None

    # ------------------------------------------------------------ debug topics

    def publish_cones(self, header, cones):
        arr = PoseArray(header=header)
        for x, y, c, w in cones:
            q = Pose()
            q.position.x, q.position.y = float(x), float(y)
            q.orientation.w = float(c)
            arr.poses.append(q)
        self.pub_cones.publish(arr)

    @staticmethod
    def track_fault(track):
        """What is wrong with a track built from the ordering's rungs, or None: the rungs may
        reach across to another section (absurd width), leave a pinch, or zigzag (a kink the
        MPC cannot follow). The boundary walk is the fallback in those cases."""
        half = track["half_width"]
        if not 0.5 <= float(np.median(half)) <= 2.5:
            return "has an absurd width (median half width %.1f m)" % float(np.median(half))
        if float(np.min(half)) < 0.25:
            return "pinches to %.2f m" % float(np.min(half))
        c = track["centre"]
        d = np.roll(c, -1, axis=0) - c
        h = np.arctan2(d[:, 1], d[:, 0])
        turn = np.abs(np.angle(np.exp(1j * np.diff(np.concatenate([h, h[:1]])))))
        if float(np.max(turn)) > math.radians(60):
            return "zigzags (%.0f degrees between samples)" % math.degrees(float(np.max(turn)))
        return None

    def halt(self, why):
        """Stop for good and say why: the real car cannot back up, so a car that is stuck or
        has lost the cones is a DNF, and the log says what happened."""
        if not self.halted:
            self.halted = True
            self.get_logger().error("stopped: %s" % why)

    def status_line(self, t, moving):
        """One line for /feb/status: phase, what is steering, and every fallback that is active."""
        if self.mode == "MAPPING":
            what = "mapping lap on %s" % ("the team's local path" if self.on_local_path else "the reactive follower")
            detail = "%d cones, %.0f m" % (len(self.slam.lhat), self.travelled)
        elif self.mode == "ORDERING":
            what, detail = "map closed, waiting for the cone ordering", "%d cones" % len(self.slam.lhat)
        else:
            what = "racing on " + ("MPC" if self.ctl.info.get("ok") else "pure pursuit")
            detail = "lap %d | %.1f of %.1f m/s | %.2f m off the line" % (max(len(self.lap_times) - 1, 0), self.odo_speed,
                                                                          self.ctl.info.get("v_plan", 0.0), self.ctl.info.get("off", 0.0))
        flags = []
        if self.halted:
            flags.append("stopped")
        if t - self.last_seen_t > self.p["lost_after"]:
            flags.append("no cones in view")
        if self.mode == "RACING" and t - self.good_loc_t > self.p["lost_after"]:
            flags.append("map match lost")
        if not moving and self.throttle > 0.02:
            flags.append("not moving")
        return "%s | %s%s" % (what, detail, (" | " + ", ".join(flags)) if flags else "")

    def map_message(self):
        arr = PoseArray()
        arr.header.frame_id = "map"
        for (x, y), c in zip(self.slam.lhat, self.slam.colour):
            q = Pose()
            q.position.x, q.position.y, q.orientation.w = float(x), float(y), float(c)
            arr.poses.append(q)
        return arr

    def publish_map(self):
        self.pub_map.publish(self.map_message())
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
