#!/usr/bin/env python3
"""Make a cone track from a real Formula Student layout, cones exactly where they stand.

Reads the cone lists of a FSSIM track (AMZ, `cones_left`/`cones_right`/`cones_orange_big` in
a yaml) or an EUFS track (`tag,x,y,...` csv with blue/yellow/big_orange rows), scales them so
the corridor is our standard width, writes the cones to cones.json, rasterises the corridor between the two boundaries into
map.png (so centreline, checkpoints and the asphalt follow the real layout), and a track.yaml
that uses those cones.

    tools/track_from_cones.py FSG.yaml tracks/fsg --name "FSG trackdrive" --width 2.2
    tools/track_from_cones.py small_track.csv tracks/eufs_small --name "EUFS small"

Then, as for any track:  tools/track_build.py tracks/fsg
"""
import argparse
import csv
import json
import math
import pathlib

import numpy as np
import yaml

ap = argparse.ArgumentParser()
ap.add_argument("source")
ap.add_argument("folder")
ap.add_argument("--name", required=True)
ap.add_argument("--width", type=float, default=2.2, help="corridor width after scaling (m)")
ap.add_argument("--category", default="fsae")
ap.add_argument("--direction", default=None, help="ccw or cw; default: as laid out")
args = ap.parse_args()

src = pathlib.Path(args.source)
if src.suffix == ".yaml":
    t = yaml.safe_load(src.read_text())
    xy = lambda rows: np.array([r[:2] for r in rows], float).reshape(-1, 2)
    blue, yellow, orange = xy(t["cones_left"]), xy(t["cones_right"]), xy(t.get("cones_orange_big") or [])
else:
    rows = list(csv.DictReader(open(src)))
    pick = lambda tag: np.array([[float(r["x"]), float(r["y"])] for r in rows if r["tag"] == tag], float).reshape(-1, 2)
    blue, yellow, orange = pick("blue"), pick("yellow"), pick("big_orange")

# scale so the median distance from a blue cone to the nearest yellow one is the corridor width
width_now = float(np.median(np.min(np.linalg.norm(blue[:, None] - yellow[None], axis=2), axis=1)))
scale = args.width / width_now
blue, yellow, orange = blue * scale, yellow * scale, orange * scale
centre = (blue + yellow) / 2.0 if len(blue) == len(yellow) else None


def order(pts):
    """Greedy nearest-neighbour walk from the first cone: the source lists are nearly in order."""
    rest, out = list(range(1, len(pts))), [0]
    while rest:
        d = np.linalg.norm(pts[rest] - pts[out[-1]], axis=1)
        j = rest.pop(int(np.argmin(d)))
        out.append(j)
    return pts[out]


blue_o, yellow_o = order(blue), order(yellow)
# shift everything so the drawing starts near the origin
shift = np.min(np.vstack([blue, yellow]), axis=0) - 3.0
blue, yellow, orange, blue_o, yellow_o = blue - shift, yellow - shift, orange - shift, blue_o - shift, yellow_o - shift
# the corridor is the ring between the two ordered boundaries, rasterised straight from the cones
# (no fixed width): the centreline, checkpoints and the asphalt then follow the real layout
res = 0.05
hi = np.max(np.vstack([blue, yellow]), axis=0) + 3.0
W, H = int(hi[0] / res) + 1, int(hi[1] / res) + 1
import cv2
canvas = np.zeros((H, W), np.uint8)
px = lambda pts: np.round(np.column_stack([pts[:, 0] / res, (H - 1) - pts[:, 1] / res])).astype(np.int32)
cv2.fillPoly(canvas, [px(blue_o)], 255)
cv2.fillPoly(canvas, [px(yellow_o)], 0) if cv2.contourArea(px(blue_o).astype(np.float32)) > cv2.contourArea(px(yellow_o).astype(np.float32)) else None
if cv2.contourArea(px(blue_o).astype(np.float32)) <= cv2.contourArea(px(yellow_o).astype(np.float32)):
    canvas[:] = 0
    cv2.fillPoly(canvas, [px(yellow_o)], 255)
    cv2.fillPoly(canvas, [px(blue_o)], 0)
kept = np.array([(b + yellow[np.argmin(np.linalg.norm(yellow - b, axis=1))]) / 2.0 for b in blue_o])   # only for the length estimate

folder = pathlib.Path(args.folder)
folder.mkdir(parents=True, exist_ok=True)
cones = [{"x": round(float(x), 3), "y": round(float(y), 3), "color": "blue"} for x, y in blue]
cones += [{"x": round(float(x), 3), "y": round(float(y), 3), "color": "yellow"} for x, y in yellow]
cones += [{"x": round(float(x), 3), "y": round(float(y), 3), "color": "orange"} for x, y in orange]
(folder / "cones.json").write_text(json.dumps(cones))
cv2.imwrite(str(folder / "map.png"), canvas)
for f in ("design.yaml",):
    (folder / f).unlink(missing_ok=True)
gate = orange.mean(axis=0) if len(orange) else kept[0]
# driving direction: blue cones stand on the left of travel, so if the blue boundary is the inner
# one the lap runs counter-clockwise, otherwise clockwise
K = np.array(kept)
area = 0.5 * np.sum(K[:, 0] * np.roll(K[:, 1], -1) - np.roll(K[:, 0], -1) * K[:, 1])      # positive = ccw polygon
ccw_poly = area > 0
inner_blue = np.mean([np.min(np.linalg.norm(K - b, axis=1)) for b in blue]) < np.mean([np.min(np.linalg.norm(K - y, axis=1)) for y in yellow])
# with a ccw walk the inside is on the left; if blue is nearer the centroid it is the inner boundary
centroid = K.mean(axis=0)
inner_blue = np.mean(np.linalg.norm(blue - centroid, axis=1)) < np.mean(np.linalg.norm(yellow - centroid, axis=1))
direction = args.direction or ("ccw" if inner_blue else "cw")
(folder / "track.yaml").write_text(
    f"name: {args.name}\ncategory: {args.category}\nresolution: {res}\norigin: [0.0, 0.0]\ndirection: {direction}\ncheckpoints: 30\n"
    f"qualifying: false\nstart: [{gate[0]:.2f}, {gate[1]:.2f}]\ncones:\n  from: cones.json\n")
seg = np.linalg.norm(np.diff(np.vstack([kept, kept[:1]]), axis=0), axis=1).sum()
print(f"{args.name}: {len(blue)} blue, {len(yellow)} yellow, {len(orange)} orange, scale {scale:.3f}, about {seg:.0f} m -> {folder}")
