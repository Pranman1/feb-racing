"""Longitudinal identification of the simulator's car: hold a throttle from rest, straight
ahead, and watch the ground truth. The wheel encoders slip under power, so speed comes from
the position stream only.

The reset command latches: while it is true the simulator holds the body kinematic, so it has
to be published false again, and kept there, or the car never moves.
"""
import rclpy, numpy as np, time, pickle, sys
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy

rclpy.init(); n = Node("longid")
q = QoSProfile(depth=10); q.reliability = ReliabilityPolicy.RELIABLE
pub_t = n.create_publisher(Float32, "/autodrive/roboracer_1/throttle_command", q)
pub_s = n.create_publisher(Float32, "/autodrive/roboracer_1/steering_command", q)
pub_r = n.create_publisher(Bool, "/autodrive/reset_command", q)
G = {"x": 0.0, "y": 0.0, "hits": 0}
n.create_subscription(Point, "/autodrive/roboracer_1/ips", lambda m: G.update(x=m.x, y=m.y), q)
n.create_subscription(Int32, "/autodrive/roboracer_1/collision_count", lambda m: G.update(hits=m.data), q)

def spin(sec, tau, reset=False, rec=None):
    t0 = time.time()
    while time.time() - t0 < sec:
        pub_r.publish(Bool(data=reset))
        pub_s.publish(Float32(data=0.0))
        pub_t.publish(Float32(data=float(tau)))
        rclpy.spin_once(n, timeout_sec=0.005)
        if rec is not None: rec.append((time.time(), G["x"], G["y"], float(tau), G["hits"]))
        time.sleep(0.01)

def restart():
    spin(0.8, 0.0, reset=True)
    spin(1.2, 0.0, reset=False)

bursts = []
for tau in [0.03, 0.05, 0.08, 0.12, 0.16, 0.20, 0.25, 0.30, 0.40, 0.60, 1.00]:
    restart(); rec = []
    spin(3.5, tau, rec=rec)          # accelerate from rest
    spin(2.0, 0.0, rec=rec)          # throttle zero: the simulator brakes all four wheels
    bursts.append((tau, rec))
    print("tau %.2f: %d samples, moved %.1f m, %d hits"
          % (tau, len(rec), np.hypot(rec[-1][1] - rec[0][1], rec[-1][2] - rec[0][2]), rec[-1][4] - rec[0][4]), flush=True)
for hold in [0.0, 0.005, 0.02, 0.05, 0.10, 0.15]:
    restart(); rec = []
    spin(2.2, 0.30, rec=rec)         # up to speed
    spin(3.0, hold, rec=rec)         # then hold a small throttle: coast, or brake?
    bursts.append(("hold%.3f" % hold, rec))
    print("hold %.3f done" % hold, flush=True)
spin(0.5, 0.0)
pickle.dump(bursts, open(sys.argv[1], "wb"))
print("saved")
