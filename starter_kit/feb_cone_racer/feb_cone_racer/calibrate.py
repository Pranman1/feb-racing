"""Camera calibration for the cone racer, from a bag: the numbers in racer.yaml that are about
this car's camera, fitted from what the lidar and the camera saw together.

    ros2 run feb_cone_racer calibrate_camera <bag> [--track track.json] [--params racer.yaml]

What it fits, from lidar cones and camera blobs that clearly belong together (one blob near
the cone's expected column, no other cone on that bearing):
  camera geometry   camera_hfov_deg, camera_ahead, camera_lateral, camera_col_bias (where a
                    point in front of the lidar lands in the image), camera_lag_s (how far the
                    image lags the scan, from the error against yaw rate)
  image band        band_top, band_bottom, band_floor (the rows cones live in)
  brightness gate   blob_min_value (a hue blob darker than this is a speck, not a cone)
With --track (the true cone positions, and the bag's IPS ground truth as on the simulator)
it also fits the HSV bounds per colour from the pixels of cones of known colour, and reports
the colour accuracy by range. On the real car there is no truth: the geometry, band and
brightness fits still work, the HSV bounds are reported as they stand.

The output is a yaml snippet to paste into config/racer.yaml, with the fit quality beside
each number so a bad bag (too few cones, the car standing still) is obvious.
"""
import argparse
import json
import math
import sys

import cv2
import numpy as np

from .perception import BLUE, ORANGE, YELLOW, Perception
from .racer import DEFAULTS


def load_params(path):
    p = dict(DEFAULTS)
    if path:
        import yaml
        doc = yaml.safe_load(open(path))
        node = next(iter(doc.values()))
        p.update(node.get("ros__parameters", node))
    return p


def read_bag(path, per, truth, image_hsv):
    """One row per lidar cone that the camera could see: car-frame position, the blob that was
    nearest its column (if any), the truth colour (if known), yaw rate, and the image's HSV."""
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import Image, Imu, LaserScan
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id="sqlite3"), ConverterOptions("", ""))
    yaw = ips = None
    yaw_rate = 0.0
    hsv = None
    rows = []
    scans = 0
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic.endswith("/imu"):
            m = deserialize_message(data, Imu)
            q = m.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            yaw_rate = m.angular_velocity.z
        elif topic.endswith("/ips"):
            m = deserialize_message(data, Point)
            ips = np.array([m.x, m.y])
        elif topic.endswith("/front_camera"):
            m = deserialize_message(data, Image)
            per.on_image(m)
            if image_hsv:
                img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
                hsv = cv2.cvtColor(np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img), cv2.COLOR_BGR2HSV)
        elif topic.endswith("/lidar") and yaw is not None and per.blobs and (truth is None or ips is not None):
            scans += 1
            clusters = per.clusters(deserialize_message(data, LaserScan))
            cols = [per.pixel_column(x, y, yaw_rate) for x, y in clusters]
            for i, (x, y) in enumerate(clusters):
                if math.isnan(cols[i]):
                    continue
                lonely = all(j == i or math.isnan(cols[j]) or abs(cols[j] - cols[i]) > 0.065 * per.image_width for j in range(len(clusters)))
                near = [b for b in per.blobs if abs(b[1] - cols[i]) < 0.06 * per.image_width]
                blob = min(near, key=lambda b: abs(b[1] - cols[i])) if near else None
                tc = None
                if truth is not None:
                    c, s = math.cos(yaw), math.sin(yaw)
                    w = ips + 0.32 * np.array([c, s]) + np.array([c * x - s * y, s * x + c * y])
                    d = np.linalg.norm(truth[0] - w, axis=1)
                    k = int(np.argmin(d))
                    tc = int(truth[1][k]) if d[k] < 0.4 else None
                rows.append(dict(x=x, y=y, col=cols[i], lonely=lonely, blob=blob, truth=tc, yaw_rate=yaw_rate, hsv=hsv))
    return rows, scans


def fit_geometry(rows, W, p, truth=False):
    """Column model u = W/2 - fx * (y - dy) / (x - x0) + bias, then the lag from the error's
    slope against yaw rate. Uses cones with one clear blob and nothing else on the bearing."""
    S = [(r["x"], r["y"], r["blob"][1], r["yaw_rate"]) for r in rows if r["lonely"] and r["blob"] is not None
         and abs(r["blob"][1] - r["col"]) < 0.04 * W and 1.0 < math.hypot(r["x"], r["y"]) < 6.0
         and (r["truth"] == r["blob"][0] if truth else True)]
    if len(S) < 200:
        return None, "only %d clear cone-blob pairs (need a few hundred: drive past more cones)" % len(S)
    A = np.array(S)
    keep = np.ones(len(A), dtype=bool)
    # u - W/2 = -fx * (y - dy) / (x - x0) + bias - fx * lag * yaw_rate: linear in fx, fx*dy,
    # bias and fx*lag once x0 is fixed, so x0 is scanned and the rest solved by least squares;
    # then the pairs that do not fit (a wrong blob) are dropped and it is done again
    for _ in range(3):
        x, y, u, yr = A[keep].T
        best = None
        for x0 in np.arange(-1.0, 0.61, 0.02):
            cx = x - x0
            m = cx > 0.2
            if m.sum() < 100:
                continue
            M = np.c_[y[m] / cx[m], 1.0 / cx[m], np.ones(int(m.sum())), yr[m]]
            coef, *_ = np.linalg.lstsq(M, u[m] - W / 2, rcond=None)
            err = u[m] - W / 2 - M @ coef
            if best is None or err.std() < best[0]:
                best = (err.std(), x0, coef)
        sd, x0, coef = best
        fx, bias = -coef[0], coef[2]
        dy = coef[1] / fx if fx else 0.0
        lag = -coef[3] / fx if fx else 0.0
        cx = A[:, 0] - x0
        pred = W / 2 - fx * (A[:, 1] - dy) / np.maximum(cx, 0.2) + bias - fx * lag * A[:, 3]
        e_all = A[:, 2] - pred
        keep = (cx > 0.2) & (np.abs(e_all - np.median(e_all[keep])) < 3.0 * max(sd, 1.0))
    hfov = 2 * math.degrees(math.atan((W / 2) / fx))
    return dict(camera_hfov_deg=round(hfov, 1), camera_ahead=round(x0, 2), camera_lateral=round(dy, 3),
                camera_col_bias=round(bias, 1), camera_lag_s=round(lag, 3)), \
        "%d pairs kept of %d, residual %.2f px" % (int(keep.sum()), len(A), e_all[keep].std())


def fit_band(rows, H):
    tops = np.array([r["blob"][2] for r in rows if r["blob"] is not None and r["lonely"]])
    bottoms = np.array([r["blob"][3] for r in rows if r["blob"] is not None and r["lonely"]])
    if len(tops) < 100:
        return None, "too few blobs"
    return dict(band_top=round(max(np.percentile(tops, 1) / H - 0.03, 0.0), 2),
                band_bottom=round(min(np.percentile(bottoms, 99) / H + 0.03, 1.0), 2),
                band_floor=round(np.percentile(bottoms, 1) / H, 2)), "%d blobs; cone tops at rows %.0f-%.0f, bottoms %.0f-%.0f of %d" % (
        len(tops), np.percentile(tops, 1), np.percentile(tops, 99), np.percentile(bottoms, 1), np.percentile(bottoms, 99), H)


def blob_value(r):
    """Mean HSV value of the blob's pixels (its column +-3, its rows)."""
    b, hsv = r["blob"], r["hsv"]
    u0, u1 = int(max(b[1] - 3, 0)), int(min(b[1] + 4, hsv.shape[1]))
    v0, v1 = int(b[2]), int(max(b[3], b[2] + 1))
    return float(np.mean(hsv[v0:v1, u0:u1, 2]))


def fit_brightness(rows, truth):
    M = [r for r in rows if r["blob"] is not None and r["hsv"] is not None and 2.0 < math.hypot(r["x"], r["y"]) < 4.5]
    if len(M) < 100:
        return None, "too few blobs"
    vals = np.array([blob_value(r) for r in M])
    if not truth:
        return dict(blob_min_value=int(np.percentile(vals, 5))), "%d blobs, value 5%%/50%%/95%%: %.0f / %.0f / %.0f (no truth: the 5th percentile keeps nearly every blob; raise it if far cones get the wrong colour)" % (
            len(M), *np.percentile(vals, [5, 50, 95]))
    right = np.array([blob_value(r) for r in M if r["truth"] is not None and r["truth"] == r["blob"][0]])
    wrong = np.array([blob_value(r) for r in M if r["truth"] is not None and r["truth"] not in (None, r["blob"][0]) and r["truth"] != ORANGE])
    if len(wrong) < 20:
        return dict(blob_min_value=int(np.percentile(right, 3))), "%d right blobs, too few wrong ones to weigh against" % len(right)
    best = max(range(5, 120, 5), key=lambda t: np.mean(right >= t) - np.mean(wrong >= t))
    return dict(blob_min_value=best), "right blobs %d (value median %.0f), wrong %d (median %.0f); at %d it keeps %.0f%% of right and %.0f%% of wrong" % (
        len(right), np.median(right), len(wrong), np.median(wrong), best, 100 * np.mean(right >= best), 100 * np.mean(wrong >= best))


def fit_hsv(rows):
    """HSV bounds per colour from the pixels of blobs on cones of known colour."""
    out, notes = {}, []
    for name, c in (("hsv_blue", BLUE), ("hsv_yellow", YELLOW), ("hsv_orange", ORANGE)):
        px = []
        for r in rows:
            b = r["blob"]
            if b is None or r["hsv"] is None or r["truth"] != c or b[0] != c or not r["lonely"]:
                continue
            u0, u1 = int(max(b[1] - 2, 0)), int(min(b[1] + 3, r["hsv"].shape[1]))
            win = r["hsv"][int(b[2]):int(max(b[3], b[2] + 1)), u0:u1].reshape(-1, 3)
            px.append(win[win[:, 1] > 80])           # the cone's pixels are the saturated ones; the rest is ground
        if not px:
            notes.append("%s: no cones of this colour seen" % name)
            continue
        P = np.vstack(px)
        # the hue range: the 90% of pixels around the commonest hue, widened a little; the
        # saturation and value floors from the darkest, palest cone pixels
        hist = np.bincount(P[:, 0], minlength=180)
        mode = int(np.argmax(hist))
        lo_h, hi_h = mode, mode
        while hist[lo_h:hi_h + 1].sum() < 0.9 * len(P) and (lo_h > 0 or hi_h < 179):
            if lo_h > 0 and (hi_h >= 179 or hist[lo_h - 1] >= hist[hi_h + 1]):
                lo_h -= 1
            else:
                hi_h += 1
        lo, hi = np.percentile(P, 2, axis=0), np.percentile(P, 98, axis=0)
        lo[0], hi[0] = max(lo_h - 5, 0), min(hi_h + 5, 179)
        out[name] = [int(lo[0]), int(lo[1]), int(max(lo[2] - 5, 0)), int(hi[0]), 255, 255]
        notes.append("%s: %d pixels, hue %d-%d, saturation from %d, value from %d" % (name, len(P), lo[0], hi[0], lo[1], lo[2]))
    return out, "; ".join(notes)


def colour_report(rows):
    bands = [(0, 2), (2, 4), (4, 6.5)]
    lines = []
    for a, b in bands:
        R = [r for r in rows if r["blob"] is not None and r["truth"] not in (None, ORANGE) and a <= math.hypot(r["x"], r["y"]) < b]
        if R:
            wrong = sum(1 for r in R if r["blob"][0] != r["truth"])
            lines.append("  %.0f-%.1f m: %d readings, %.1f%% wrong" % (a, b, len(R), 100.0 * wrong / len(R)))
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description="fit the racer's camera parameters from a bag")
    ap.add_argument("bag")
    ap.add_argument("--track", help="track.json with the true cones (simulator bags with IPS): also fits the HSV bounds and reports colour accuracy")
    ap.add_argument("--params", help="racer.yaml to start from (default: the built-in defaults)")
    args = ap.parse_args(argv)
    p = load_params(args.params)
    per = Perception(p)
    truth = None
    if args.track:
        t = json.load(open(args.track))
        truth = (np.array([[c["x"], c["y"]] for c in t["cones"]]),
                 np.array([{"blue": BLUE, "yellow": YELLOW}.get(c["color"], ORANGE) for c in t["cones"]]))
    rows, scans = read_bag(args.bag, per, truth, image_hsv=True)
    if not rows:
        sys.exit("nothing usable in the bag: it needs lidar, front_camera and imu (and ips with --track)")
    W = per.image_width
    H = int(round(max(r["blob"][3] for r in rows if r["blob"] is not None) / max(p["band_bottom"], 0.01))) if any(r["blob"] for r in rows) else 0
    H = rows[0]["hsv"].shape[0] if rows[0]["hsv"] is not None else H
    print("bag: %d scans with an image, %d lidar cones in the camera's view, image %dx%d" % (scans, len(rows), W, H))
    out = {}
    for name, (fit, note) in (("camera geometry", fit_geometry(rows, W, p, truth is not None)), ("image band", fit_band(rows, H)),
                              ("brightness gate", fit_brightness(rows, truth is not None))):
        print("%s: %s" % (name, note))
        if fit:
            out.update(fit)
    if truth is not None:
        fit, note = fit_hsv(rows)
        print("hsv bounds: %s" % note)
        out.update(fit)
        print("colour accuracy with the current parameters, camera readings against the true cones:\n" + colour_report(rows))
    print("\n# paste into config/racer.yaml (ros__parameters)")
    for k, v in out.items():
        print("    %s: %s" % (k, v))


if __name__ == "__main__":
    main()
