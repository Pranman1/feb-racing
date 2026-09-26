"""From a cone map to a track: ordered boundaries, centreline, widths. Pure numpy.

Boundaries are closed loops of cones of one colour. Ordering is a nearest-neighbour walk that
prefers to keep going the way it was going (the same greedy chain as the simple driver, run
around the whole loop). The centreline is sampled along the blue boundary: for every sample
the nearest point of the yellow boundary is found and the midpoint taken; the half distance
between them is the local half width.
"""
import numpy as np

from .perception import BLUE, ORANGE, YELLOW


def _walk(pts, i0, h, max_step):
    rest = list(range(len(pts)))
    order = [i0]
    rest.remove(i0)
    while rest:
        last = pts[order[-1]]
        best, best_score = None, -np.inf
        for j in rest:
            d = pts[j] - last
            dist = np.linalg.norm(d)
            if dist > max_step or dist < 1e-6:
                continue
            score = float(d @ h) / dist - 0.15 * dist        # onward, and near
            if score > best_score:
                best, best_score = j, score
        if best is None:
            break
        d = pts[best] - last
        h = 0.5 * h + 0.5 * d / (np.linalg.norm(d) + 1e-9)
        h /= np.linalg.norm(h) + 1e-9
        order.append(best)
        rest.remove(best)
    return order


def order_loop(points, start, heading, max_step=None):
    """Greedy walk through the points starting nearest `start`, heading roughly `heading`,
    preferring to continue straight. If the walk dies early it is retried the other way round
    and the longer result kept. The longest step allowed follows the cone spacing (2.4 times
    the median nearest-neighbour distance, at least 3.5 m) so one cone missing from the map
    does not cut the loop, and 4 times the spacing if the loop still stays open. Cones farther
    than that from everything are left out."""
    pts = np.asarray(points, float)
    if len(pts) < 3:
        return pts
    spacing = float(np.median(np.sort(np.linalg.norm(pts[:, None] - pts[None], axis=2), axis=1)[:, 1]))
    steps = [max_step] if max_step else [max(3.5, 2.4 * spacing), max(3.5, 4.0 * spacing)]
    i0 = int(np.argmin(np.linalg.norm(pts - start, axis=1)))
    h = np.asarray(heading, float) / (np.linalg.norm(heading) + 1e-9)
    best = []
    for step in steps:                      # a wider step only if the loop stays open otherwise
        fwd = _walk(pts, i0, h, step)
        if len(fwd) < len(pts) - 1:
            back = _walk(pts, i0, -h, step)
            if len(back) > len(fwd):
                fwd = back[::-1]
        if len(fwd) > len(best):
            best = fwd
        closed = np.linalg.norm(pts[best[-1]] - pts[best[0]]) <= step
        if closed and len(best) >= 0.95 * len(pts):
            break
    return pts[best]


def resample_closed(poly, step):
    """Resample a closed polyline at roughly `step` metres. Returns (points, cumulative s)."""
    poly = np.asarray(poly, float)
    closed = np.vstack([poly, poly[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n = max(int(round(total / step)), 8)
    ss = np.linspace(0.0, total, n, endpoint=False)
    xs = np.interp(ss, s, closed[:, 0])
    ys = np.interp(ss, s, closed[:, 1])
    return np.column_stack([xs, ys]), ss


def nearest_on_polyline(poly_closed, q, max_seg=None):
    """Nearest point on a closed polyline to q. With max_seg, segments longer than that (a
    bridge across missing cones, or the closing segment of a loop that did not close) do not
    count."""
    P = np.vstack([poly_closed, poly_closed[:1]])
    a, b = P[:-1], P[1:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", q - a, ab) / (np.einsum("ij,ij->i", ab, ab) + 1e-12), 0.0, 1.0)
    proj = a + t[:, None] * ab
    dist = np.linalg.norm(proj - q, axis=1)
    if max_seg is not None:
        dist[np.linalg.norm(ab, axis=1) > max_seg] = np.inf
    k = int(np.argmin(dist))
    return proj[k]


def side_of_path(path, q):
    """Signed distance of q from the closed polyline `path`: positive on its left."""
    P = np.vstack([path, path[:1]])
    a, b = P[:-1], P[1:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", q - a, ab) / (np.einsum("ij,ij->i", ab, ab) + 1e-12), 0.0, 1.0)
    proj = a + t[:, None] * ab
    k = int(np.argmin(np.linalg.norm(proj - q, axis=1)))
    cross = ab[k, 0] * (q[1] - proj[k, 1]) - ab[k, 1] * (q[0] - proj[k, 0])
    return float(np.sign(cross) * np.linalg.norm(q - proj[k]))


def repair_by_path(lhat, colour, path):
    """Cone colours by which side of the car's own mapping-lap path they lie on: the local
    follower keeps that path near the middle, so a cone clearly to its left is blue and one to
    its right is yellow, whatever the camera said. The orange gate cones get their side too."""
    colour = np.array(colour, dtype=int).copy()
    if path is None or len(path) < 8:
        return colour
    path = np.asarray(path, float)
    width = _usual_width(lhat, colour)
    for i, q in enumerate(lhat):
        d = side_of_path(path, q)
        if 0.35 * width < abs(d) < 1.6 * width:
            colour[i] = BLUE if d > 0 else YELLOW
    return colour


def track_from_rungs(blue, yellow, colour, step=0.25, cones=None):
    """The track from the team's cone ordering: blue[i] and yellow[i] are the two ends of one
    rung across the track, in order along it. The centreline is the rung midpoints, the half
    width half the rung length, both resampled evenly; the boundaries are the rung ends."""
    blue, yellow = np.asarray(blue, float).reshape(-1, 2), np.asarray(yellow, float).reshape(-1, 2)
    n = min(len(blue), len(yellow))
    if n < 8:
        return None
    blue, yellow = blue[:n], yellow[:n]
    centre = (blue + yellow) / 2.0
    half = np.linalg.norm(blue - yellow, axis=1) / 2.0
    # the rungs come every 0.2 m and the odd one is rotated or out of step at a hairpin: drop
    # midpoints that double back on their neighbours, then smooth over about a metre
    for _ in range(4):
        m = len(centre)
        keep = np.ones(m, dtype=bool)
        for i in range(m):
            a, b = centre[i] - centre[i - 1], centre[(i + 1) % m] - centre[i]
            if np.dot(a, b) < 0 and np.linalg.norm(a) > 1e-6 and np.linalg.norm(b) > 1e-6:
                keep[i] = False
        if np.all(keep):
            break
        centre, half = centre[keep], half[keep]
    k = 4
    pad = np.vstack([centre[-k:], centre, centre[:k]])
    ker = np.ones(2 * k + 1) / (2 * k + 1)
    centre = np.column_stack([np.convolve(pad[:, 0], ker, mode="valid"), np.convolve(pad[:, 1], ker, mode="valid")])
    seg = np.linalg.norm(np.vstack([np.diff(centre, axis=0), centre[:1] - centre[-1:]]), axis=1)
    s_in = np.concatenate([[0.0], np.cumsum(seg)])
    centre, s = resample_closed(centre, step)
    half = np.interp(s, s_in[:-1], half)
    # never wider than the nearest mapped cone allows (the rung ends are projections and bunch
    # up at a hairpin, so they are not used as a boundary line)
    if cones is not None and len(cones):
        cones = np.asarray(cones, float).reshape(-1, 2)
        half = np.minimum(half, np.min(np.linalg.norm(centre[:, None, :] - cones[None, :, :], axis=2), axis=1))
    track = _finish(centre, half, s, step)
    track.update(left=blue, right=yellow, colour=np.array(colour, dtype=int))
    return track


def build_track(lhat, colour, start_xy, start_heading, step=0.25, repair_colours=True, path=None):
    """Ordered boundaries and a sampled centreline from the map.
    Returns dict(centre (N,2), normal (N,2), half_width (N,), s (N,), left (M,2), right (K,2),
    colour (repaired)) or None if a boundary is missing. Colours are repaired before building:
    `path` is the car's own mapping-lap track, which the local follower keeps near the middle,
    so a cone clearly to its left is blue and one to its right is yellow, whatever the camera
    said; then (repair_colours) a cone sitting on the other colour's boundary line is flipped
    too, and the track rebuilt once."""
    colour = repair_by_path(lhat, colour, path)
    # the orange start cones belong to whichever boundary they stand on: give each the colour
    # of its nearest blue or yellow neighbour, then treat them like any other boundary cone
    for i in np.flatnonzero(colour == ORANGE):
        others = np.flatnonzero((colour == BLUE) | (colour == YELLOW))
        if len(others):
            colour[i] = colour[others[int(np.argmin(np.linalg.norm(lhat[others] - lhat[i], axis=1)))]]
    track = _build(lhat, colour, start_xy, start_heading, step, path)
    if track is None or not repair_colours:
        return track
    fixed = np.array(colour, dtype=int).copy()
    # a cone sitting on the other colour's boundary line (built without it) has the wrong colour:
    # a true yellow cone labelled blue lies within centimetres of the segment joining its two
    # yellow neighbours, while a real blue cone is a track width away from that line
    for _ in range(2):
        left, right = order_loop(lhat[fixed == BLUE], start_xy, start_heading), order_loop(lhat[fixed == YELLOW], start_xy, start_heading)
        seg = lambda P: 2.0 * np.median(np.linalg.norm(np.roll(P, -1, axis=0) - P, axis=1))   # only real cone-to-cone segments count
        changed = False
        for i, q in enumerate(lhat):
            other = right if fixed[i] == BLUE else left
            if len(other) >= 3 and np.linalg.norm(nearest_on_polyline(other, q, seg(other)) - q) < 0.35:
                fixed[i] = YELLOW if fixed[i] == BLUE else BLUE
                changed = True
        if not changed:
            break
    if np.array_equal(fixed, colour):
        track["colour"] = fixed
        return track
    track2 = _build(lhat, fixed, start_xy, start_heading, step, path)
    if track2 is None:
        track["colour"] = np.array(colour, dtype=int)
        return track
    track2["colour"] = fixed
    return track2


def _usual_width(lhat, colour):
    """Median distance from a blue cone to the nearest yellow one: the track width."""
    b, y = lhat[colour == BLUE], lhat[colour == YELLOW]
    if len(b) < 3 or len(y) < 3:
        return 3.0
    return float(np.median(np.min(np.linalg.norm(b[:, None] - y[None], axis=2), axis=1)))


def _build(lhat, colour, start_xy, start_heading, step, path=None):
    left = order_loop(lhat[colour == BLUE], start_xy, start_heading)
    right = order_loop(lhat[colour == YELLOW], start_xy, start_heading)
    if len(left) < 4 or len(right) < 4:
        return None
    lp, _ = resample_closed(left, step)
    rp = np.array([nearest_on_polyline(right, q) for q in lp])
    centre, half = (lp + rp) / 2.0, np.linalg.norm(lp - rp, axis=1) / 2.0
    # A cone track has a nearly constant width. Where the two boundaries come much closer or
    # farther than usual, one of them is wrong there (a cone missing at a hairpin makes its
    # boundary cut across the other one), so those samples take the usual width, measured
    # sideways from the blue boundary instead.
    tang = np.roll(lp, -1, axis=0) - np.roll(lp, 1, axis=0)
    tang /= np.linalg.norm(tang, axis=1)[:, None] + 1e-9
    nrm = np.column_stack([-tang[:, 1], tang[:, 0]])
    h_med = float(np.median(half))
    bad = np.abs(half - h_med) > 0.4 * h_med
    if np.any(bad) and not np.all(bad):
        if path is not None and len(path) >= 8:          # the car's own lap ran near the middle
            centre[bad] = np.array([nearest_on_polyline(np.asarray(path, float), q) for q in lp[bad]])
        else:
            side = np.sign(np.mean(np.einsum("ij,ij->i", rp[~bad] - lp[~bad], nrm[~bad])))
            centre[bad] = lp[bad] + side * h_med * nrm[bad]
        half[bad] = h_med
    # smooth the centreline lightly (closed) and resample evenly. The window is short on
    # purpose: a long one cuts into the inside of a hairpin. The half width is then measured
    # again from the smoothed centre to the nearer boundary line, so it is true where the
    # centre moved.
    k = 2
    pad = np.vstack([centre[-k:], centre, centre[:k]])
    ker = np.ones(2 * k + 1) / (2 * k + 1)
    sm = np.column_stack([np.convolve(pad[:, 0], ker, mode="valid"), np.convolve(pad[:, 1], ker, mode="valid")])
    centre, s = resample_closed(sm, step)
    seg_l, seg_r = 2.0 * np.median(np.linalg.norm(np.roll(left, -1, axis=0) - left, axis=1)), 2.0 * np.median(np.linalg.norm(np.roll(right, -1, axis=0) - right, axis=1))
    half = np.array([min(np.linalg.norm(nearest_on_polyline(left, q, seg_l) - q), np.linalg.norm(nearest_on_polyline(right, q, seg_r) - q)) for q in centre])
    half = np.minimum(half, h_med)
    track = _finish(centre, half, s, step, left)
    track.update(left=left, right=right)
    return track


def _finish(centre, half, s, step, left=None):
    """Tangents and left-of-travel normals for a sampled centreline. With `left` (the blue
    boundary) the loop is flipped if blue is not on the +normal side."""
    nxt = np.roll(centre, -1, axis=0)
    prv = np.roll(centre, 1, axis=0)
    tang = nxt - prv
    tang /= np.linalg.norm(tang, axis=1)[:, None] + 1e-9
    normal = np.column_stack([-tang[:, 1], tang[:, 0]])          # left of travel
    if left is not None:
        lq = np.array([nearest_on_polyline(left, c) for c in centre[:: max(len(centre) // 12, 1)]])
        side = np.mean(np.einsum("ij,ij->i", lq - centre[:: max(len(centre) // 12, 1)], normal[:: max(len(centre) // 12, 1)]))
        if side < 0:
            centre, half, normal = centre[::-1], half[::-1], -normal[::-1]
            s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(centre, axis=0), axis=1))])
    return dict(centre=centre, normal=normal, half_width=half, s=s)
