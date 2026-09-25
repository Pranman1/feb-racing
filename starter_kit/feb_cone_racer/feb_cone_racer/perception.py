"""Cone perception: lidar clusters + camera colour, fused by bearing, colour carried between scans.

Same method as feb_cone_driver (the simple driver), packaged as a class so the racer can call
it per scan. Frames: car frame, x forward, y left, origin at the lidar. Images are decoded with
numpy (no cv_bridge) so the package has no dependency beyond numpy and OpenCV.
"""
import math

import cv2
import numpy as np

BLUE, YELLOW, ORANGE = 1, 2, 3          # orange: the big start-line cones


class Perception:
    def __init__(self, p):
        self.p = p
        self.blobs = []                 # [(colour, u_centre_px, top_row_px, area)] from the latest image
        self.image_width = None
        self.prev = []                  # last scan's cones: (x, y, colour, age)
        self.prev_yaw = None

    # ------------------------------------------------------------ camera

    def on_image(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]
        self.image_width = msg.width
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_BGR2HSV)
        # HSV bounds per colour (h_lo, s_lo, v_lo, h_hi, s_hi, v_hi), calibrated on this camera:
        # the cones are dark in this scene, so the value floor is low and colour comes from hue
        # and saturation; the sky is a desaturated blue, so blue needs high saturation
        bounds = {BLUE: self.p["hsv_blue"], YELLOW: self.p["hsv_yellow"], ORANGE: self.p["hsv_orange"]}
        top, bottom, floor = self.p["band_top"], self.p["band_bottom"], self.p["band_floor"]
        blobs = []
        for colour, b in bounds.items():
            mask = cv2.inRange(hsv, tuple(int(v) for v in b[:3]), tuple(int(v) for v in b[3:]))
            mask[: int(top * h), :] = 0              # sky
            mask[int(bottom * h):, :] = 0            # the car's own body and the floor
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if not self.p["min_blob_px"] <= area <= 0.02 * w * h:
                    continue
                x, y, bw, bh = cv2.boundingRect(c)
                if y + bh < floor * h:                   # entirely above the horizon: sky
                    continue
                # the start cones are big: a small orange-hued blob is a shaded yellow cone
                label = YELLOW if colour == ORANGE and area < self.p["orange_min_px"] else colour
                blobs.append((label, x + bw / 2.0, float(y), area))   # colour, column, top row, area
        self.blobs = blobs

    # ------------------------------------------------------------ lidar

    def clusters(self, msg):
        """Small tight clusters within max_range: cone candidates as (x, y) in the car frame."""
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        ok = np.isfinite(ranges) & (ranges > 0.3) & (ranges < self.p["max_range"])
        xs, ys = ranges * np.cos(angles), ranges * np.sin(angles)
        groups, cur = [], []
        for i in np.flatnonzero(ok):
            if cur and math.hypot(xs[i] - xs[cur[-1]], ys[i] - ys[cur[-1]]) > self.p["cluster_gap"]:
                groups.append(cur)
                cur = []
            cur.append(i)
        if cur:
            groups.append(cur)
        out = []
        for g in groups:
            if len(g) > self.p["cone_max_points"]:
                continue
            px, py = xs[g], ys[g]
            if math.hypot(px.max() - px.min(), py.max() - py.min()) > self.p["cone_max_width"]:
                continue
            out.append((float(px.mean()), float(py.mean())))
        return out

    # ------------------------------------------------------------ fusion

    def pixel_column(self, x, y, yaw_rate=0.0):
        """Image column where a point (x forward, y left, lidar frame) lands; NaN if behind the
        camera. Camera pose relative to the lidar, focal length, column bias and the image's lag
        behind the scan (the car keeps turning between the two) are calibrated parameters."""
        w = self.image_width
        fx = (w / 2.0) / math.tan(math.radians(self.p["camera_hfov_deg"]) / 2.0)
        cx = x - self.p["camera_ahead"]
        cy = y - self.p["camera_lateral"]
        if cx < 0.2:
            return float("nan")
        bearing = math.atan2(cy, cx) + yaw_rate * self.p["camera_lag_s"]     # where it was when the image was taken
        return w / 2.0 - fx * math.tan(bearing) + self.p["camera_col_bias"]

    def assign(self, clusters, yaw_rate=0.0):
        """One-to-one matching of lidar cones to camera blobs by image column, closest pairs
        first, so two cones cannot claim the same blob. Returns per cluster (colour or None,
        ambiguous) where ambiguous means another cone sits within a few pixels of its column."""
        n = len(clusters)
        cols = [self.pixel_column(x, y, yaw_rate) if self.image_width else float("nan") for x, y in clusters]
        colour, taken = [None] * n, set()
        if self.blobs and self.image_width:
            tol = self.p["match_px_frac"] * self.image_width
            pairs = sorted((abs(b[1] - cols[i]), i, k) for i in range(n) if not math.isnan(cols[i])
                           for k, b in enumerate(self.blobs) if abs(b[1] - cols[i]) < tol)
            done = set()
            for d, i, k in pairs:
                if i in done or k in taken:
                    continue
                colour[i], done.add(i), taken.add(k)
                colour[i] = self.blobs[k][0]
        amb_px = 0.065 * self.image_width if self.image_width else 12.0
        ambiguous = [not math.isnan(cols[i]) and any(j != i and not math.isnan(cols[j]) and abs(cols[j] - cols[i]) < amb_px for j in range(n)) for i in range(n)]
        return colour, ambiguous

    def fuse(self, clusters, speed, yaw, dt, yaw_rate=0.0):
        """Colour each lidar cone: camera first, then the colour it had last scan (moved into
        this scan's frame), then a side guess for near cones. Returns [(x, y, colour, weight)]: weight
        1.0 when the camera matched this cone this scan and no other cone shares its bearing,
        0.3 when the colour was carried from an earlier scan, 0.0 for a side guess."""
        dyaw = 0.0
        if yaw is not None and self.prev_yaw is not None:
            dyaw = math.atan2(math.sin(yaw - self.prev_yaw), math.cos(yaw - self.prev_yaw))
        self.prev_yaw = yaw
        ds = speed * dt
        c, s = math.cos(dyaw), math.sin(dyaw)
        moved = []
        for x, y, colour, age in self.prev:
            x0, y0 = x - ds, y
            moved.append((c * x0 + s * y0, -s * x0 + c * y0, colour, age + 1))
        cam_colour, ambiguous = self.assign(clusters, yaw_rate)
        out, result = [], []
        for i, (x, y) in enumerate(clusters):
            colour, age = cam_colour[i], 0
            weight = 0.0 if colour is None else (0.3 if ambiguous[i] else 1.0)
            if colour is None:
                best, best_d = None, self.p["track_gate"]
                for px, py, pc, page in moved:
                    d = math.hypot(px - x, py - y)
                    if d < best_d and page < self.p["track_memory"]:
                        best, best_d = (pc, page), d
                if best is not None:
                    colour, age = best
                    weight = 0.3
            if colour is None and 0.0 < x < self.p["side_guess_range"] and abs(y) < self.p["track_width"]:
                colour, age, weight = (BLUE if y > 0.0 else YELLOW), self.p["track_memory"] - 1, 0.0
            # a cone right beside the car is that side's boundary as long as the car is between
            # the lines: its side beats a colour remembered from when it was far ahead (the camera
            # cannot see it here). Such a cone drives the car but does not vote for the map.
            if colour is not None and x < self.p["side_override_x"] and abs(y) > self.p["side_override_y"] and abs(y) < self.p["track_width"]:
                side = BLUE if y > 0.0 else YELLOW
                if colour != side and colour != ORANGE:
                    colour, weight = side, 0.0
            out.append((x, y, colour, age))
            if colour is not None:
                result.append((x, y, colour, weight))
        self.prev = out
        return result
