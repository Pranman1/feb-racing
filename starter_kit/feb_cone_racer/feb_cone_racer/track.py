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


def order_loop(points, start, heading, max_step=3.5):
    """Greedy walk through the points starting nearest `start`, heading roughly `heading`,
    preferring to continue straight. If the walk dies early it is retried the other way round
    and the longer result kept. Outliers farther than max_step from everything are left out."""
    pts = np.asarray(points, float)
    if len(pts) < 3:
        return pts
    i0 = int(np.argmin(np.linalg.norm(pts - start, axis=1)))
    h = np.asarray(heading, float) / (np.linalg.norm(heading) + 1e-9)
    fwd = _walk(pts, i0, h, max_step)
    if len(fwd) < len(pts) - 1:
        back = _walk(pts, i0, -h, max_step)
        if len(back) > len(fwd):
            fwd = back[::-1]
    return pts[fwd]


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


def nearest_on_polyline(poly_closed, q):
    """Nearest point on a closed polyline to q."""
    P = np.vstack([poly_closed, poly_closed[:1]])
    a, b = P[:-1], P[1:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", q - a, ab) / (np.einsum("ij,ij->i", ab, ab) + 1e-12), 0.0, 1.0)
    proj = a + t[:, None] * ab
    k = int(np.argmin(np.linalg.norm(proj - q, axis=1)))
    return proj[k]


def build_track(lhat, colour, start_xy, start_heading, step=0.25, repair_colours=True):
    """Ordered boundaries and a sampled centreline from the map.
    Returns dict(centre (N,2), normal (N,2), half_width (N,), s (N,), left (M,2), right (K,2),
    colour (repaired)) or None if a boundary is missing. With repair_colours the map's colours
    are corrected once by which side of the first centreline each cone lies on (blue is left of
    travel), which fixes the odd cone the camera mislabelled, and the track is rebuilt."""
    colour = np.array(colour, dtype=int).copy()
    # the orange start cones belong to whichever boundary they stand on: give each the colour
    # of its nearest blue or yellow neighbour, then treat them like any other boundary cone
    for i in np.flatnonzero(colour == ORANGE):
        others = np.flatnonzero((colour == BLUE) | (colour == YELLOW))
        if len(others):
            colour[i] = colour[others[int(np.argmin(np.linalg.norm(lhat[others] - lhat[i], axis=1)))]]
    track = _build(lhat, colour, start_xy, start_heading, step)
    if track is None or not repair_colours:
        return track
    fixed = np.array(colour, dtype=int).copy()
    # a cone sitting on the other colour's boundary line (built without it) has the wrong colour:
    # a true yellow cone labelled blue lies within centimetres of the segment joining its two
    # yellow neighbours, while a real blue cone is a track width away from that line
    for _ in range(2):
        left, right = order_loop(lhat[fixed == BLUE], start_xy, start_heading), order_loop(lhat[fixed == YELLOW], start_xy, start_heading)
        changed = False
        for i, q in enumerate(lhat):
            other = right if fixed[i] == BLUE else left
            if len(other) >= 3 and np.linalg.norm(nearest_on_polyline(other, q) - q) < 0.35:
                fixed[i] = YELLOW if fixed[i] == BLUE else BLUE
                changed = True
        if not changed:
            break
    if np.array_equal(fixed, colour):
        track["colour"] = fixed
        return track
    track2 = _build(lhat, fixed, start_xy, start_heading, step)
    if track2 is None:
        track["colour"] = np.array(colour, dtype=int)
        return track
    track2["colour"] = fixed
    return track2


def _build(lhat, colour, start_xy, start_heading, step):
    left = order_loop(lhat[colour == BLUE], start_xy, start_heading)
    right = order_loop(lhat[colour == YELLOW], start_xy, start_heading)
    if len(left) < 4 or len(right) < 4:
        return None
    lp, _ = resample_closed(left, step)
    centre, half = [], []
    for q in lp:
        r = nearest_on_polyline(right, q)
        centre.append((q + r) / 2.0)
        half.append(np.linalg.norm(q - r) / 2.0)
    centre, half = np.array(centre), np.array(half)
    # smooth the centreline lightly (closed) and resample evenly
    k = 5
    pad = np.vstack([centre[-k:], centre, centre[:k]])
    ker = np.ones(2 * k + 1) / (2 * k + 1)
    sm = np.column_stack([np.convolve(pad[:, 0], ker, mode="valid"), np.convolve(pad[:, 1], ker, mode="valid")])
    centre, s = resample_closed(sm, step)
    half = np.interp(s, np.linspace(0, s[-1] + step, len(half), endpoint=False), half)
    nxt = np.roll(centre, -1, axis=0)
    prv = np.roll(centre, 1, axis=0)
    tang = nxt - prv
    tang /= np.linalg.norm(tang, axis=1)[:, None] + 1e-9
    normal = np.column_stack([-tang[:, 1], tang[:, 0]])          # left of travel
    # make sure the blue (left) boundary really is on the +normal side; flip the loop otherwise
    lq = np.array([nearest_on_polyline(left, c) for c in centre[:: max(len(centre) // 12, 1)]])
    side = np.mean(np.einsum("ij,ij->i", lq - centre[:: max(len(centre) // 12, 1)], normal[:: max(len(centre) // 12, 1)]))
    if side < 0:
        centre, half, normal = centre[::-1], half[::-1], -normal[::-1]
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(centre, axis=0), axis=1))])
    return dict(centre=centre, normal=normal, half_width=half, s=s, left=left, right=right)
