#!/usr/bin/env python3
"""Make a track design from a RoboRacer/F1TENTH racetrack centreline CSV.

The f1tenth_racetracks repository (github.com/f1tenth/f1tenth_racetracks) ships real
circuits at 1:10 scale as `<Name>_centerline.csv` with columns x_m, y_m, w_tr_right_m,
w_tr_left_m. This writes a design.yaml for track_design.py with our standard corridor
width, so the circuit gets air-duct walls like every other FEB track.

    tools/track_from_centerline.py Spielberg_centerline.csv tracks/spielberg --width 2.2 --scale 1.0 --step 2.0
"""
import argparse
import math
import pathlib

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("folder")
ap.add_argument("--width", type=float, default=2.2, help="corridor width in metres")
ap.add_argument("--scale", type=float, default=1.0, help="multiply the centreline coordinates")
ap.add_argument("--step", type=float, default=2.0, help="metres between design points")
args = ap.parse_args()

points = []
for line in pathlib.Path(args.csv).read_text().splitlines():
    if not line or line.startswith("#"):
        continue
    x, y = (float(v) for v in line.split(",")[:2])
    points.append((x * args.scale, y * args.scale))
kept = [points[0]]
for p in points[1:]:
    if math.dist(p, kept[-1]) >= args.step:
        kept.append(p)
if math.dist(kept[-1], kept[0]) < args.step / 2:
    kept.pop()

folder = pathlib.Path(args.folder)
folder.mkdir(parents=True, exist_ok=True)
(folder / "design.yaml").write_text(
    f"# from {pathlib.Path(args.csv).name}, scale {args.scale}, every {args.step} m\n"
    f"width: {args.width}\nresolution: 0.05\npoints:\n" + "".join(f"  - [{x:.2f}, {y:.2f}]\n" for x, y in kept))
print(f"{len(kept)} design points -> {folder / 'design.yaml'}")
