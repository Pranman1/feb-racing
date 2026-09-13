#!/usr/bin/env python3
"""Draw a track corridor PNG from a few waypoints.

Input is a YAML file with the corridor centreline as control points (metres) and
a width. A closed smooth spline is fitted through the points and painted white
onto a black image. The result is the ``map.png`` that ``track_build.py``
turns into simulator geometry, so a new track is: edit YAML, run this, run
track_build.

    tools/track_design.py tracks/loop/design.yaml

design.yaml::

    width: 2.0                 # corridor width in metres
    resolution: 0.05           # metres per pixel (default 0.05)
    margin: 1.0                # black border around the track (default 1.0)
    points:                    # closed loop, any number >= 3
      - [0, 0]
      - [8, 0]
      - [8, 5]
      - [0, 5]
"""
import argparse
import pathlib

import cv2
import numpy as np
import yaml


def closed_spline(points, samples_per_segment=40):
    """Catmull-Rom spline through the points, closed. Returns (N, 2) array."""
    p = np.asarray(points, dtype=float)
    n = len(p)
    out = []
    for i in range(n):
        p0, p1, p2, p3 = p[(i - 1) % n], p[i], p[(i + 1) % n], p[(i + 2) % n]
        t = np.linspace(0.0, 1.0, samples_per_segment, endpoint=False)[:, None]
        out.append(0.5 * ((2 * p1) + (-p0 + p2) * t
                          + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2
                          + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    return np.vstack(out)


def render(design):
    width = float(design["width"])
    res = float(design.get("resolution", 0.05))
    margin = float(design.get("margin", 1.0))
    line = closed_spline(design["points"])

    lo = line.min(axis=0) - width / 2 - margin
    hi = line.max(axis=0) + width / 2 + margin
    size = np.ceil((hi - lo) / res).astype(int)
    img = np.zeros((size[1], size[0]), dtype=np.uint8)

    # image rows grow downwards, map y grows upwards
    px = np.stack([(line[:, 0] - lo[0]) / res, size[1] - (line[:, 1] - lo[1]) / res], axis=1)
    cv2.polylines(img, [np.round(px).astype(np.int32)], isClosed=True, color=255,
                  thickness=max(1, int(round(width / res))), lineType=cv2.LINE_AA)
    _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    return img, res, lo


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("design", type=pathlib.Path, help="design.yaml")
    ap.add_argument("-o", "--out", type=pathlib.Path, help="output PNG (default: map.png next to the design)")
    args = ap.parse_args()

    design = yaml.safe_load(args.design.read_text())
    img, res, origin = render(design)
    out = args.out or args.design.with_name("map.png")
    cv2.imwrite(str(out), img)

    # keep the metric frame so track_build places the track where it was designed
    meta = args.design.with_name("track.yaml")
    track = yaml.safe_load(meta.read_text()) if meta.exists() else {}
    track.update(resolution=res, origin=[float(origin[0]), float(origin[1])])
    meta.write_text(yaml.safe_dump(track, sort_keys=False))
    print(f"{out}: {img.shape[1]}x{img.shape[0]} px at {res} m/px; origin {origin.round(3).tolist()}")


if __name__ == "__main__":
    main()
