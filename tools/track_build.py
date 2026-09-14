#!/usr/bin/env python3
"""Turn a track corridor PNG into the geometry the FEB simulator loads at runtime.

A track is a folder::

    tracks/<name>/
        map.png       white = drivable corridor, anything else = not drivable
        track.yaml    metadata (see below)
        track.json    OUTPUT: walls, centreline, checkpoints, spawn, cones
        preview.png   OUTPUT: picture of the result

track.yaml::

    name: FEB Loop 1
    resolution: 0.05          # metres per pixel
    origin: [0.0, 0.0]        # map-frame position of the bottom-left pixel (m)
    direction: ccw            # driving direction, ccw or cw
    checkpoints: 20           # number of lap checkpoints (checkpoint 0 = finish line)
    start: [3.0, 1.0]         # optional finish-line position (m); default: longest straight
    walls:
      diameter: 0.33          # air-duct diameter (m); omit the key for no walls
      color: "#9a9a9a"        # optional duct colour (hex), default mid grey
    cones:                    # optional; omit the key for no cones
      spacing: 1.0            # metres between cones along each boundary

Coordinates in track.json are in the map frame: x right, y up, metres.
The simulator converts to its own axes when loading.
"""
import argparse
import json
import math
import pathlib

import cv2
import numpy as np
import yaml

FREE_THRESHOLD = 250      # grey level at or above which a pixel is drivable (ROS maps: 254 free, 205 unknown, 0 wall)
MIN_ISLAND_AREA = 1.0     # square metres; smaller holes in the corridor are map noise and get filled
SIMPLIFY_PX = 0.75        # polyline simplification tolerance in pixels
CENTRELINE_SPACING = 0.10  # metres between centreline samples
STRAIGHT_WINDOW = 3.0     # metres used to find the straightest section for the start
SPAWN_BEHIND_START = 0.3  # metres the car spawns behind the finish line
MIN_WALL_LENGTH = 1.0     # metres; shorter wall loops are specks in the map, not walls


class Frame:
    """Pixel <-> map-frame conversion."""

    def __init__(self, resolution, origin, height_px):
        self.res = resolution
        self.ox, self.oy = origin
        self.h = height_px

    def to_map(self, px):
        px = np.asarray(px, dtype=float).reshape(-1, 2)
        x = self.ox + (px[:, 0] + 0.5) * self.res
        y = self.oy + (self.h - px[:, 1] - 0.5) * self.res
        return np.stack([x, y], axis=1)

    def to_px(self, xy):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        c = (xy[:, 0] - self.ox) / self.res - 0.5
        r = self.h - (xy[:, 1] - self.oy) / self.res - 0.5
        return np.stack([c, r], axis=1)


# ---------------------------------------------------------------- raster helpers

def corridor_mask(png_path, resolution):
    """Largest white connected component of the PNG with noise holes filled, as a 0/255 mask."""
    img = cv2.imread(str(png_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"cannot read {png_path}")
    free = (img >= FREE_THRESHOLD).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(free, connectivity=4)
    if n < 2:
        raise SystemExit("no drivable (white) area found")
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    mask = np.where(labels == largest, 255, 0).astype(np.uint8)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    noise = [c for c, h in zip(contours, hierarchy[0])
             if h[3] >= 0 and cv2.contourArea(c) * resolution ** 2 < MIN_ISLAND_AREA]
    cv2.drawContours(mask, noise, -1, 255, -1)
    return mask


def boundaries(mask):
    """(outer contour, list of hole contours) of a mask, each as (N, 2) float pixel arrays."""
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    hierarchy = hierarchy[0]
    outer = [c for c, h in zip(contours, hierarchy) if h[3] < 0]
    holes = [c for c, h in zip(contours, hierarchy) if h[3] >= 0]
    outer = max(outer, key=cv2.contourArea)
    return outer.reshape(-1, 2).astype(float), [h.reshape(-1, 2).astype(float) for h in holes]


def distance_to(contour, shape):
    """Distance transform (pixels) to a contour drawn as a 1-px line."""
    canvas = np.full(shape, 255, dtype=np.uint8)
    cv2.polylines(canvas, [np.round(contour).astype(np.int32)], True, 0, 1)
    return cv2.distanceTransform(canvas, cv2.DIST_L2, 5)


def simplify(contour, tolerance_px=SIMPLIFY_PX):
    approx = cv2.approxPolyDP(contour.astype(np.float32).reshape(-1, 1, 2), tolerance_px, True)
    return approx.reshape(-1, 2).astype(float)


# ---------------------------------------------------------------- centreline

def resample_closed(points, spacing):
    """Resample a closed polyline at uniform arc-length spacing."""
    pts = np.vstack([points, points[:1]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(8, int(round(s[-1] / spacing)))
    t = np.linspace(0.0, s[-1], n, endpoint=False)
    return np.stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])], axis=1)


def smooth_closed(points, window):
    kernel = np.ones(window) / window
    pad = window // 2
    ext = np.vstack([points[-pad:], points, points[:pad]])
    return np.stack([np.convolve(ext[:, i], kernel, mode="valid") for i in range(2)], axis=1)


def centreline_px(mask, outer, hole, d_outer, d_hole):
    """Walk outward from the hole boundary until equidistant from both walls."""
    h, w = mask.shape
    pts = resample_closed(hole, spacing=4.0)
    tangents = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    tangents /= np.linalg.norm(tangents, axis=1, keepdims=True)
    normals = np.stack([-tangents[:, 1], tangents[:, 0]], axis=1)

    def inside(p):
        c, r = int(round(p[0])), int(round(p[1]))
        return 0 <= c < w and 0 <= r < h and mask[r, c] > 0

    # pick the normal sign that points into the corridor
    probe = pts + 3.0 * normals
    if np.mean([inside(p) for p in probe]) < 0.5:
        normals = -normals

    centre = []
    for p, n in zip(pts, normals):
        q = p.copy()
        for _ in range(int(max(h, w))):
            q += 0.5 * n
            if not inside(q):
                break
            if d_outer[int(round(q[1])), int(round(q[0]))] <= d_hole[int(round(q[1])), int(round(q[0]))]:
                centre.append(q.copy())
                break
    return np.array(centre)


def signed_area(xy):
    x, y = xy[:, 0], xy[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def straightest_index(xy, window_m, spacing):
    """Index of the point starting the straightest window (least heading change)."""
    k = max(2, int(window_m / spacing))
    d = np.roll(xy, -1, axis=0) - xy
    heading = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    turn = np.abs(np.roll(heading, -k) - heading)
    turn[-k:] = np.abs(heading[:k] + 2 * math.pi * round((heading[-1] - heading[0]) / (2 * math.pi)) - heading[-k:])
    return int(np.argmin(turn))


def yaw_at(xy, i):
    d = xy[(i + 1) % len(xy)] - xy[i - 1]
    return math.atan2(d[1], d[0])


# ---------------------------------------------------------------- features

def build_walls(mask, radius_px, frame):
    """Wall centre polylines (metres): the corridor boundary pushed out by the duct radius."""
    k = 2 * int(math.ceil(radius_px)) + 1
    grown = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    outer, holes = boundaries(grown)
    loops = [frame.to_map(simplify(c)) for c in [outer] + holes]
    return [w for w in loops if cv2.arcLength(w.astype(np.float32), True) >= MIN_WALL_LENGTH]


def build_cones(outer, hole, ccw, spacing_m, frame):
    """Cones along both corridor edges: blue on the driver's left, yellow on the right."""
    spacing_px = spacing_m / frame.res
    left, right = (hole, outer) if ccw else (outer, hole)
    cones = []
    for contour, colour in ((left, "blue"), (right, "yellow")):
        for x, y in frame.to_map(resample_closed(contour, spacing_px)):
            cones.append({"x": round(float(x), 3), "y": round(float(y), 3), "color": colour})
    return cones


def build_track(folder):
    folder = pathlib.Path(folder)
    cfg = yaml.safe_load((folder / "track.yaml").read_text())
    resolution = float(cfg["resolution"])
    mask = corridor_mask(folder / "map.png", resolution)
    frame = Frame(resolution, cfg.get("origin", [0.0, 0.0]), mask.shape[0])

    outer, holes = boundaries(mask)
    if len(holes) != 1:
        raise SystemExit(f"expected a loop with exactly one island, found {len(holes)} holes")
    hole = holes[0]
    d_outer, d_hole = distance_to(outer, mask.shape), distance_to(hole, mask.shape)

    centre = centreline_px(mask, outer, hole, d_outer, d_hole)
    spacing_px = CENTRELINE_SPACING / frame.res
    centre = resample_closed(smooth_closed(resample_closed(centre, spacing_px), window=9), spacing_px)
    xy = frame.to_map(centre)
    ccw = cfg.get("direction", "ccw") == "ccw"
    if (signed_area(xy) > 0) != ccw:
        xy, centre = xy[::-1], centre[::-1]

    if "start" in cfg:
        start = int(np.argmin(np.linalg.norm(xy - np.asarray(cfg["start"], float), axis=1)))
    else:
        start = straightest_index(xy, STRAIGHT_WINDOW, CENTRELINE_SPACING)
    xy, centre = np.roll(xy, -start, axis=0), np.roll(centre, -start, axis=0)

    n_cp = int(cfg.get("checkpoints", 20))
    checkpoints = []
    for i in np.linspace(0, len(xy), n_cp, endpoint=False).astype(int):
        c, r = np.round(centre[i]).astype(int)
        checkpoints.append({
            "x": round(float(xy[i, 0]), 3), "y": round(float(xy[i, 1]), 3),
            "yaw": round(yaw_at(xy, i), 4),
            "width": round(float(d_outer[r, c] + d_hole[r, c]) * frame.res, 3),
        })

    back = int(round(SPAWN_BEHIND_START / CENTRELINE_SPACING))
    spawn = {"x": round(float(xy[-back, 0]), 3), "y": round(float(xy[-back, 1]), 3),
             "yaw": round(yaw_at(xy, len(xy) - back), 4)}

    track = {
        "version": 1,
        "name": cfg.get("name", folder.name),
        "direction": "ccw" if ccw else "cw",
        "length": round(float(len(xy) * CENTRELINE_SPACING), 2),
        "centreline": xy.round(3).flatten().tolist(),
        "checkpoints": checkpoints,
        "spawn": spawn,
        "walls": [],
        "cones": [],
    }
    if "walls" in cfg:
        diameter = float(cfg["walls"].get("diameter", 0.33))
        track["wall_diameter"] = diameter
        track["wall_color"] = str(cfg["walls"].get("color", "#9a9a9a"))
        track["walls"] = [{"points": w.round(3).flatten().tolist()}
                          for w in build_walls(mask, diameter / 2 / frame.res, frame)]
    if "cones" in cfg:
        track["cones"] = build_cones(outer, hole, ccw, float(cfg["cones"].get("spacing", 1.0)), frame)
    return track, mask, frame


# ---------------------------------------------------------------- preview

def draw_preview(track, mask, frame, path):
    img = cv2.cvtColor(np.where(mask > 0, 235, 40).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    blue, gold, white = (135, 70, 28), (21, 181, 253), (255, 255, 255)   # BGR
    px = lambda xy: np.round(frame.to_px(xy)).astype(np.int32)
    for w in track["walls"]:
        cv2.polylines(img, [px(w["points"])], True, blue, max(1, int(track["wall_diameter"] / frame.res)))
    cv2.polylines(img, [px(track["centreline"])], True, gold, 1)
    for cp in track["checkpoints"]:
        n = np.array([-math.sin(cp["yaw"]), math.cos(cp["yaw"])]) * cp["width"] / 2
        a, b = px([cp["x"] - n[0], cp["y"] - n[1]]), px([cp["x"] + n[0], cp["y"] + n[1]])
        cv2.line(img, tuple(a[0]), tuple(b[0]), white, 2 if cp is track["checkpoints"][0] else 1)
    for c in track["cones"]:
        cv2.circle(img, tuple(px([c["x"], c["y"]])[0]), 2, blue if c["color"] == "blue" else gold, -1)
    s = track["spawn"]
    tip = px([s["x"] + 0.3 * math.cos(s["yaw"]), s["y"] + 0.3 * math.sin(s["yaw"])])[0]
    cv2.arrowedLine(img, tuple(px([s["x"], s["y"]])[0]), tuple(tip), (0, 200, 0), 2, tipLength=0.5)
    cv2.imwrite(str(path), img)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="+", type=pathlib.Path, help="track folder(s) containing map.png and track.yaml")
    args = ap.parse_args()
    for folder in args.folders:
        track, mask, frame = build_track(folder)
        (folder / "track.json").write_text(json.dumps(track, separators=(",", ":")))
        draw_preview(track, mask, frame, folder / "preview.png")
        print(f"{track['name']}: {track['length']} m, {len(track['checkpoints'])} checkpoints, "
              f"{len(track['walls'])} wall loops, {len(track['cones'])} cones -> {folder / 'track.json'}")


if __name__ == "__main__":
    main()
