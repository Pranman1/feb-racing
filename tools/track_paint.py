#!/usr/bin/env python3
"""The asphalt of a track, from the truth: where its cones or its walls actually stand.

    tools/track_paint.py tracks/loop_cones            write paint_left / paint_right into track.json
    tools/track_paint.py --all                        every track in ./tracks
    tools/track_paint.py --all --preview out.png      and a sheet of pictures to look at

The simulator paints the road between two lines. They used to be guessed inside the app from
whichever cones lay within three metres of each centreline point, which jumps from cone to cone
and gave the road a sawtooth edge. Here the two lines are made once, from the boundaries
themselves, and written into the track file as pairs: point i of `paint_left` faces point i of
`paint_right`, and the app joins each pair to the next with nothing to decide.

  * On a cone track each edge is a smooth envelope round that side's cones. Every cone has a
    place along the centreline and a distance out from it; the edge is that distance, taken as
    the largest within a metre and a half so that a cone standing out of line is still on the
    road, smoothed, plus a quarter of a metre so that the cones stand on the asphalt and not on
    its edge.
  * On a walled track the edges are the two walls' own polylines (the tube covers the edge),
    zipped together: both are walked in the direction of travel, always advancing the side
    that gives the shorter line across the road, so every corner of both walls is used.

`check` verifies the result: every cone and the whole centreline on the asphalt, nothing folded.
"""
import argparse
import json
import pathlib
import sys

import numpy as np
from scipy.ndimage import maximum_filter1d, uniform_filter1d

STEP = 0.25            # m between points of an edge
CONE_MARGIN = 0.25     # m of asphalt beyond the cones
REACH = 1.5            # m along the track over which a cone sets the width
ROOT = pathlib.Path(__file__).resolve().parent.parent


def closed_lengths(p):
    return np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1)


def resample_closed(p, step):
    s = np.concatenate([[0.0], np.cumsum(closed_lengths(p))])
    n = max(int(round(s[-1] / step)), 8)
    u = np.arange(n) * s[-1] / n
    q = np.vstack([p, p[:1]])
    return np.column_stack([np.interp(u, s, q[:, 0]), np.interp(u, s, q[:, 1])]), s[-1]


class Centre:
    """The centreline, resampled every 0.1 m, with its left normals."""

    def __init__(self, xy):
        self.p, self.length = resample_closed(np.asarray(xy, float).reshape(-1, 2), 0.1)
        self.n = len(self.p)
        self.ds = self.length / self.n
        k = 8                                                   # tangents over a metre and a half: a kink in the line must not flip a side
        t = np.roll(self.p, -k, axis=0) - np.roll(self.p, k, axis=0)
        t /= np.linalg.norm(t, axis=1)[:, None] + 1e-12
        self.left = np.column_stack([-t[:, 1], t[:, 0]])

    def nearest(self, q, side=0, around=None, back=2.0, ahead=10.0):
        """Index of the centreline point nearest to q: on the given side of the line if `side`
        is +1 (q to its left) or -1, and within a window round `around` if that is given."""
        if around is None:
            idx = np.arange(self.n)
        else:
            idx = np.arange(around - int(back / self.ds), around + int(ahead / self.ds) + 1) % self.n
        d = q - self.p[idx]
        dist = np.linalg.norm(d, axis=1)
        if side:
            wrong = np.sign(np.einsum("ij,ij->i", d, self.left[idx])) != side
            dist = np.where(wrong, dist + 100.0, dist)
        return int(idx[int(np.argmin(dist))])


def orderings(p, centre, side):
    """Candidate orders of a boundary's points round the loop: as given, by station, and by
    walking from each point to its nearest unvisited neighbour."""
    yield np.arange(len(p))
    yield np.argsort([centre.nearest(q, side) for q in p], kind="stable")
    left, walk = list(range(1, len(p))), [0]
    while left:
        d = np.linalg.norm(p[left] - p[walk[-1]], axis=1)
        walk.append(left.pop(int(np.argmin(d))))
    yield np.array(walk)


def chain(p, centre, side):
    """The boundary's points in order round the loop, in the direction of travel and from the
    lowest station, with the centreline index of each. Of the candidate orders the right one
    is the shortest way round (a wrong one doubles back); the indices are then found one after
    the other, each near the last, so a point is never placed on a neighbouring leg of the
    track."""
    p = np.asarray(p, float)
    p = p[min(orderings(p, centre, side), key=lambda o: closed_lengths(p[o]).sum())]
    first = np.diff([centre.nearest(q, side) for q in p[: min(6, len(p))]])
    if np.sum((first + centre.n // 2) % centre.n - centre.n // 2) < 0:             # the short way round, summed
        p = p[::-1]
    i = [centre.nearest(p[0], side)]
    for q, gap in zip(p[1:], np.linalg.norm(np.diff(p, axis=0), axis=1)):
        i.append(centre.nearest(q, side, around=i[-1], ahead=max(10.0, 2.5 * gap)))
    start = int(np.argmin(i))
    p, i = np.roll(p, -start, axis=0), np.roll(np.array(i), -start)
    lap = np.flatnonzero(np.diff(i) < -centre.n // 2)                             # past the finish line: the next lap
    if len(lap):
        i[lap[0] + 1:] += centre.n
    return p, np.maximum.accumulate(i)


# ---------------------------------------------------------------- cones: an envelope

def envelope(points, centre, side, stations):
    """How far out the edge is at each station, m from the centreline."""
    p, i = chain(points, centre, side)
    out = np.linalg.norm(p - centre.p[i % centre.n], axis=1)
    s = i * centre.ds
    s3 = np.concatenate([s - centre.length, s, s + centre.length])                # a lap either side, to read round the join
    d = np.interp(stations, s3, np.tile(out, 3))
    k = int(round(REACH / STEP))
    hit = np.zeros(len(stations))                                                  # each cone counts at its own station, whatever the sampling
    np.maximum.at(hit, np.round(s / STEP).astype(int) % len(stations), out)
    d = maximum_filter1d(np.maximum(d, hit), size=2 * k + 1, mode="wrap")
    d = uniform_filter1d(d, size=2 * k + 1, mode="wrap")
    return uniform_filter1d(d, size=5, mode="wrap") + CONE_MARGIN


def paint_cones(track, centre):
    cones = track["cones"]
    xy = np.array([[c["x"], c["y"]] for c in cones], float)
    side = np.array([+1 if c["color"] == "blue" else -1 if c["color"] == "yellow" else 0 for c in cones])
    for k in np.flatnonzero(side == 0):                         # the start cones belong to the side they stand on
        j = centre.nearest(xy[k])
        side[k] = +1 if (xy[k] - centre.p[j]) @ centre.left[j] > 0 else -1
    n = int(round(centre.length / STEP))
    stations = np.arange(n) * centre.length / n
    j = np.round(stations / centre.ds).astype(int) % centre.n
    c, normal = centre.p[j], centre.left[j]
    left, right = envelope(xy[side > 0], centre, +1, stations), envelope(xy[side < 0], centre, -1, stations)
    forward = np.column_stack([normal[:, 1], -normal[:, 0]])
    return unfold(c + left[:, None] * normal, forward), unfold(c - right[:, None] * normal, forward)


def unfold(edge, forward):
    """On the inside of a corner tighter than the road is wide, the edge runs backwards for a
    few points (they are past the corner's centre). Those points are held at the corner, so
    the road fans out from it instead of folding over itself."""
    edge = edge.copy()
    start = int(np.argmax(np.einsum("ij,ij->i", np.roll(edge, -1, axis=0) - edge, forward)))      # begin where the edge runs clearly forwards
    order = (start + np.arange(len(edge) + 1)) % len(edge)
    for a, b in zip(order[:-1], order[1:]):
        if (edge[b] - edge[a]) @ forward[b] <= 0.0:
            edge[b] = edge[a]
    return edge


# ---------------------------------------------------------------- walls: the polylines, zipped

def dense(p, i, centre):
    """Points every STEP along the closed polyline p, corners kept, with the station of each."""
    s = i * centre.ds
    n, out, st = len(p), [], []
    for k in range(n):
        p1, p2 = p[k], p[(k + 1) % n]
        s1, s2 = s[k], (s[k + 1] if k + 1 < n else s[0] + centre.length)
        m = max(int(np.ceil(np.linalg.norm(p2 - p1) / STEP)), 1)
        f = np.arange(m) / m
        out.append(p1 + f[:, None] * (p2 - p1))
        st.append(s1 + f * (s2 - s1))
    return np.vstack(out), np.concatenate(st)


def zip_up(a, sa, b, sb, skew=4.0):
    """a and b as facing pairs all the way round: from the first point of each, one side
    advances at a time, the one whose new line across the road is the shorter, and neither side
    gets more than `skew` metres of station ahead of the other."""
    i, j, pairs = 0, 0, [(0, 0)]
    while i < len(a) - 1 or j < len(b) - 1:
        can_a, can_b = i < len(a) - 1, j < len(b) - 1
        if can_a and can_b:
            if sa[i + 1] - sb[j] > skew:
                can_a = False
            elif sb[j + 1] - sa[i] > skew:
                can_b = False
        if can_a and (not can_b or np.linalg.norm(a[i + 1] - b[j]) <= np.linalg.norm(a[i] - b[j + 1])):
            i += 1
        else:
            j += 1
        pairs.append((i, j))
    k = np.array(pairs)
    return a[k[:, 0]], b[k[:, 1]]


def paint_walls(track, centre):
    walls = [np.array(w["points"], float).reshape(-1, 2) for w in track["walls"]]
    if len(walls) != 2:
        raise ValueError("%d walls: a track has an outer wall and an inner one" % len(walls))
    sides = []
    for w in walls:
        votes = [np.sign((q - centre.p[j]) @ centre.left[j]) for q in w[:: max(len(w) // 40, 1)] for j in [centre.nearest(q)]]
        sides.append(+1 if np.sum(votes) > 0 else -1)
    if sorted(sides) != [-1, 1]:
        raise ValueError("the two walls are on the same side of the centreline")
    left, right = (walls[0], walls[1]) if sides[0] > 0 else (walls[1], walls[0])
    a, sa = dense(*chain(left, centre, +1), centre)
    b, sb = dense(*chain(right, centre, -1), centre)
    return zip_up(a, sa, b, sb)


def paint(track):
    """(left, right): the two edges of the asphalt as facing pairs, (N, 2) each."""
    centre = Centre(track["centreline"])
    return paint_cones(track, centre) if track.get("cones") else paint_walls(track, centre)


# ---------------------------------------------------------------- checking and looking

def triangles(left, right):
    a, b, c, d = left, right, np.roll(left, -1, axis=0), np.roll(right, -1, axis=0)
    return np.concatenate([np.stack([a, b, c], axis=1), np.stack([b, d, c], axis=1)])


def inside(points, tri):
    """Which of the points lie on one of the triangles."""
    u, e0, e1 = tri[:, 0], tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    den = e0[:, 0] * e1[:, 1] - e0[:, 1] * e1[:, 0]
    ok = np.abs(den) > 1e-12
    u, e0, e1, den = u[ok], e0[ok], e1[ok], den[ok]
    on = np.zeros(len(points), dtype=bool)
    for k in range(0, len(points), 200):
        e2 = points[k:k + 200, None, :] - u[None, :, :]
        s = (e2[:, :, 0] * e1[None, :, 1] - e2[:, :, 1] * e1[None, :, 0]) / den
        t = (e0[None, :, 0] * e2[:, :, 1] - e0[None, :, 1] * e2[:, :, 0]) / den
        on[k:k + 200] = np.any((s >= -1e-6) & (t >= -1e-6) & (s + t <= 1 + 1e-6), axis=1)
    return on


def check(track, left, right):
    """What is wrong with a paint job, as a list of sentences; empty when nothing is."""
    faults, tri = [], triangles(left, right)
    centre = np.asarray(track["centreline"], float).reshape(-1, 2)[::3]
    off = int(np.sum(~inside(centre, tri)))
    if off:
        faults.append("%d of %d centreline points are not on the asphalt" % (off, len(centre)))
    cones = np.array([[c["x"], c["y"]] for c in track.get("cones") or []], float).reshape(-1, 2)
    if len(cones):
        off = int(np.sum(~inside(cones, tri)))
        if off:
            faults.append("%d of %d cones are not on the asphalt" % (off, len(cones)))
        # between cones nothing hides a fold, so there must be none. (Between walls a thin spur
        # of wall standing in the road is walked round, and the triangles that reach past it
        # overlap: the app draws every triangle face up, so it shows as plain asphalt.)
        area = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        folded = int(np.sum(area * np.sign(np.sum(area)) < -1e-4))
        if folded:
            faults.append("%d triangles are folded over" % folded)
    width = np.linalg.norm(left - right, axis=1)
    if width.min() < 0.8:
        faults.append("the road is %.2f m wide at its narrowest" % width.min())
    step = max(closed_lengths(left).max(), closed_lengths(right).max())
    if step > 1.5:
        faults.append("an edge jumps %.2f m between points" % step)
    return faults


def draw(ax, track, left, right):
    from matplotlib.collections import PolyCollection
    ax.add_collection(PolyCollection(triangles(left, right), facecolors="#2b2d31", edgecolors="#2b2d31", linewidths=0.3))
    for e in (left, right):
        q = np.vstack([e, e[:1]])
        ax.plot(q[:, 0], q[:, 1], color="white", lw=0.7)
    for w in track.get("walls") or []:
        q = np.array(w["points"], float).reshape(-1, 2)
        q = np.vstack([q, q[:1]])
        ax.plot(q[:, 0], q[:, 1], color="#c9c9c9", lw=1.4)
    colours = {"blue": "#2f6fd6", "yellow": "#f2c200", "orange": "#ff6a00"}
    cones = track.get("cones") or []
    if cones:
        ax.scatter([c["x"] for c in cones], [c["y"] for c in cones], s=3.0, c=[colours.get(c["color"], "k") for c in cones], zorder=3, linewidths=0)
    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.margins(0.04)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_facecolor("#8c949f")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tracks", nargs="*", help="track folders")
    ap.add_argument("--all", action="store_true", help="every track in ./tracks")
    ap.add_argument("--preview", help="write a sheet of pictures of the result")
    ap.add_argument("--dry", action="store_true", help="do not write track.json")
    a = ap.parse_args()
    folders = [pathlib.Path(t) for t in a.tracks]
    if a.all:
        folders += sorted(p.parent for p in (ROOT / "tracks").glob("*/track.json"))
    if not folders:
        ap.error("no tracks given")
    done, bad = [], 0
    for folder in folders:
        f = folder / "track.json"
        track = json.loads(f.read_text())
        left, right = paint(track)
        faults = check(track, left, right)
        bad += bool(faults)
        width = np.linalg.norm(left - right, axis=1)
        print("%-36s %4d pairs, road %.2f to %.2f m wide  %s" % (folder.name, len(left), width.min(), width.max(), "; ".join(faults) or "ok"))
        done.append((folder.name, track, left, right, faults))
        if not a.dry:
            track.pop("edges", None)                            # nested lists: the app's JSON reader never could read them
            track["paint_left"] = left.round(3).flatten().tolist()
            track["paint_right"] = right.round(3).flatten().tolist()
            f.write_text(json.dumps(track, separators=(",", ":")))
    if a.preview:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        cols = min(4, len(done))
        rows = int(np.ceil(len(done) / cols))
        fig, axes = plt.subplots(rows, cols, figsize=(5.0 * cols, 4.2 * rows), squeeze=False)
        for ax in axes.flat:
            ax.axis("off")
        for ax, (name, track, left, right, faults) in zip(axes.flat, done):
            ax.axis("on")
            draw(ax, track, left, right)
            ax.set_title(name + ("" if not faults else "  (FAULT)"), fontsize=10, color="black" if not faults else "red")
        fig.tight_layout()
        fig.savefig(a.preview, dpi=130)
        print("wrote", a.preview)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
