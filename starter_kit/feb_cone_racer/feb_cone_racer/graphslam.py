"""GraphSLAM with cones as landmarks, ported from the team's FSAE stack (graphslam_global).

The graph is linear: heading comes from the IMU, so every pose is (x, y) and every landmark is
(x, y). Two kinds of edges, each a pair of rows in a sparse system A x = b:
  odometry   x_k - x_{k-1} = dx          (weight dx_weight)
  landmark   l_j - x_k     = z           (weight z_weight; z is the cone seen from pose k, in
                                          world orientation)
Solving the normal equations gives every pose and landmark at once. Data association follows
the FSAE recipe: align the current observations to the map with a few ICP iterations, then
match each cone to the nearest landmark; anything farther than new_landmark_dist becomes a new
landmark. Colour is a vote: every observation adds to its landmark's blue or yellow count and
the majority wins, so one wrong camera reading does not create a ghost of the other colour.
When the car comes back to cones it has seen, the same matching closes the loop.

After the first lap the map is frozen and the same ICP alignment localises the car against it.
"""
import numpy as np
import scipy.sparse as sps
import scipy.sparse.linalg as spla

from .perception import UNKNOWN

BLUE, YELLOW, ORANGE = 1, 2, 3


class GraphSLAM:
    def __init__(self, x0=(0.0, 0.0), dx_weight=2.0, z_weight=1.0, new_landmark_dist=0.6,
                 icp_gate=1.5, icp_iterations=5, icp_min_matches=5, icp_max_shift=1.0, icp_max_turn=0.26):
        self.dx_weight, self.z_weight = dx_weight, z_weight
        self.icp_min_matches, self.icp_max_shift, self.icp_max_turn = icp_min_matches, icp_max_shift, icp_max_turn
        self.icp_max_residual = 0.5
        self.icp_gate_frac = 0.45               # gate never more than this fraction of the local landmark spacing
        self.new_landmark_dist, self.icp_gate, self.icp_iterations = new_landmark_dist, icp_gate, icp_iterations
        self.rows = []               # (row, col, value) triplets of A
        self.b = []
        self.nvars, self.neqns = 0, 0
        self.x = []                  # variable index of each pose
        self.l = []                  # variable index of each landmark
        self.xhat = np.zeros((0, 2))
        self.lhat = np.zeros((0, 2))
        self.votes = np.zeros((0, 4))          # per landmark: [unused, blue, yellow, orange] weighted colour votes
        self.count = np.zeros(0, dtype=int)    # per landmark: observations
        self.x.append(0)
        self.nvars = 2
        self._eq([(0, 1.0)], x0[0])
        self._eq([(1, 1.0)], x0[1])
        self.xhat = np.array([[x0[0], x0[1]]], dtype=float)
        self.frozen = False
        self.colour_override = None            # colours repaired by the track builder, once frozen

    @property
    def colour(self):
        if self.colour_override is not None:
            return self.colour_override
        if len(self.votes) == 0:
            return np.zeros(0, dtype=int)
        return np.argmax(self.votes[:, 1:], axis=1) + 1

    @property
    def seen(self):
        return self.count

    def _eq(self, terms, rhs):
        for col, val in terms:
            self.rows.append((self.neqns, col, val))
        self.b.append(rhs)
        self.neqns += 1

    # ------------------------------------------------------------ data association

    def icp(self, z, colour):
        """Rigid transform (t, R) that best lays the world-frame cones z over the map, by
        mutual nearest neighbours of the same colour; identity if there is nothing to match."""
        t, R = np.zeros(2), np.eye(2)
        if len(z) < 2 or len(self.lhat) < 2:
            return t, R
        lcol = self.colour
        # where landmarks stand close together (two track sections side by side, a hairpin's
        # inner cones) the gate shrinks to a fraction of the local spacing, so a cone is never
        # associated across to its neighbour's section
        gate = np.full(len(self.lhat), self.icp_gate)
        if self.icp_gate_frac > 0 and len(self.lhat) >= 2:
            dd = np.linalg.norm(self.lhat[:, None, :] - self.lhat[None, :, :], axis=2)
            np.fill_diagonal(dd, np.inf)
            gate = np.minimum(gate, self.icp_gate_frac * dd.min(axis=1))
        for _ in range(self.icp_iterations):
            zz = z @ R.T + t
            X, Y = [], []
            for c in np.unique(colour):
                zi = np.flatnonzero(colour == c)
                li = np.arange(len(lcol)) if c == UNKNOWN else np.flatnonzero(lcol == c)
                if not len(zi) or not len(li):
                    continue
                d = np.linalg.norm(zz[zi][:, None, :] - self.lhat[li][None, :, :], axis=2)
                fwd, bwd = np.argmin(d, axis=1), np.argmin(d, axis=0)
                for i, j in enumerate(fwd):
                    if bwd[j] == i and d[i, j] < gate[li[j]]:
                        X.append(z[zi[i]])
                        Y.append(self.lhat[li[j]])
            if len(X) < self.icp_min_matches:
                return np.zeros(2), np.eye(2)
            X, Y = np.array(X), np.array(Y)
            xm, ym = X.mean(axis=0), Y.mean(axis=0)
            U, _, Vt = np.linalg.svd((X - xm).T @ (Y - ym))
            Rn = (U @ Vt).T
            if np.linalg.det(Rn) < 0:
                Vt[-1, :] *= -1
                Rn = (U @ Vt).T
            R, t = Rn, ym - Rn @ xm
        residual = np.median(np.linalg.norm(X @ R.T + t - Y, axis=1))
        if np.linalg.norm(t) > self.icp_max_shift or abs(np.arctan2(R[1, 0], R[0, 0])) > self.icp_max_turn or residual > self.icp_max_residual:
            return np.zeros(2), np.eye(2)
        return t, R

    def relocalise(self, z_rel, colour, min_matches=4, near=None, radius=3.0):
        """Find the car anywhere on the frozen map from the cones in view, heading known (the
        IMU heading is absolute, so only a translation is unknown). Every pairing of a seen cone
        with a landmark of its colour is a candidate translation; the one that lays the most
        seen cones onto landmarks wins, if it is clear and unique. Returns (x, y) or None."""
        z_rel, colour = np.asarray(z_rel, float).reshape(-1, 2), np.asarray(colour, int)
        n = len(z_rel)
        if n < min_matches or len(self.lhat) < min_matches:
            return None
        lcol = self.colour
        cands = []
        for i in range(n):
            li = np.arange(len(lcol)) if colour[i] == UNKNOWN else np.flatnonzero(lcol == colour[i])
            if len(li):
                cands.append(self.lhat[li] - z_rel[i])
        if not cands:
            return None
        T = np.vstack(cands)                                                   # (C, 2)
        if near is not None:                                                   # only positions close to the guess
            T = T[np.linalg.norm(T - np.asarray(near, float), axis=1) < radius]
            if len(T) == 0:
                return None
        d = np.linalg.norm((z_rel[None, :, None, :] + T[:, None, None, :]) - self.lhat[None, None, :, :], axis=3)   # (C, n, L)
        hit = d.min(axis=2) < 0.4
        score = hit.sum(axis=1)
        best = int(np.argmax(score))
        if score[best] < max(min_matches, 0.8 * n):
            return None
        others = np.linalg.norm(T - T[best], axis=1) > 0.6
        if np.any(others) and score[others].max() >= score[best] - 2:
            return None                # another place fits nearly as well (cones every metre look alike one cone along)
        return T[best].copy()

    def snap(self, z, colour, centre, radius, gate):
        """Loop closure with a wide net: align the world-frame cones z to the landmarks within
        `radius` of `centre` (the start area) allowing matches up to `gate` apart. Used when the
        orange start cones come back into view after a lap, when odometry drift may exceed the
        normal association gate. Returns the shift (t) and rotation (R), identity if unsure."""
        near = np.linalg.norm(self.lhat - centre, axis=1) < radius
        if near.sum() < 4 or len(z) < 4:
            return np.zeros(2), np.eye(2)
        saved = (self.lhat, self.votes, self.count, self.colour_override, self.icp_gate, self.icp_max_shift, self.icp_min_matches)
        try:
            self.lhat, self.votes, self.count = self.lhat[near], self.votes[near], self.count[near]
            self.colour_override = None if saved[3] is None else saved[3][near]
            self.icp_gate, self.icp_max_shift, self.icp_min_matches = gate, gate, 4
            return self.icp(z, colour)
        finally:
            self.lhat, self.votes, self.count, self.colour_override, self.icp_gate, self.icp_max_shift, self.icp_min_matches = saved

    # ------------------------------------------------------------ building the graph

    def add(self, dx, z_rel, colour, weight=None):
        """One keyframe: the car moved dx (world frame) since the last one and now sees cones
        at z_rel (world orientation, relative to the car) with colours `colour`; `weight` is how
        much each colour reading counts (1 camera-confirmed, less for a remembered colour)."""
        dx, z_rel, colour = np.asarray(dx, float), np.asarray(z_rel, float).reshape(-1, 2), np.asarray(colour, int)
        weight = np.ones(len(colour)) if weight is None else np.asarray(weight, float)
        prev = self.x[-1]
        self.x.append(self.nvars)
        self.nvars += 2
        w = self.dx_weight
        self._eq([(self.x[-1], w), (prev, -w)], dx[0] * w)
        self._eq([(self.x[-1] + 1, w), (prev + 1, -w)], dx[1] * w)
        guess = self.xhat[-1] + dx
        self.xhat = np.vstack([self.xhat, guess])
        if len(z_rel) == 0:
            return
        zw = z_rel + guess
        t, R = self.icp(zw, colour)
        zw = zw @ R.T + t
        self.xhat[-1] = R @ guess + t
        for i in range(len(zw)):
            j = None
            if len(self.lhat):
                d = np.linalg.norm(self.lhat - zw[i], axis=1)
                k = int(np.argmin(d))
                if d[k] < self.new_landmark_dist:
                    j = k
            if j is None:
                j = len(self.lhat)
                self.lhat = np.vstack([self.lhat, zw[i]])
                self.votes = np.vstack([self.votes, np.zeros(4)])
                self.count = np.append(self.count, 0)
                self.l.append(self.nvars)
                self.nvars += 2
            self.votes[j, colour[i]] += weight[i]
            self.count[j] += 1
            w = self.z_weight
            self._eq([(self.l[j], w), (self.x[-1], -w)], z_rel[i, 0] * w)
            self._eq([(self.l[j] + 1, w), (self.x[-1] + 1, -w)], z_rel[i, 1] * w)

    def solve(self):
        r, c, v = zip(*self.rows)
        A = sps.csr_matrix((v, (r, c)), shape=(self.neqns, self.nvars))
        b = np.asarray(self.b)
        sol = spla.spsolve((A.T @ A).tocsc(), A.T @ b)
        for k, i in enumerate(self.x):
            self.xhat[k] = sol[i:i + 2]
        for k, i in enumerate(self.l):
            self.lhat[k] = sol[i:i + 2]

    # ------------------------------------------------------------ frozen map

    def freeze(self, min_seen=2):
        """Stop mapping; drop landmarks seen fewer than min_seen times and merge any two of the
        same colour closer than half the association gate (duplicates from drift)."""
        self.solve()
        keep = self.seen >= min_seen
        lhat, votes, count = self.lhat[keep], self.votes[keep], self.count[keep]
        col = np.argmax(votes[:, 1:], axis=1) + 1
        merged = np.ones(len(lhat), dtype=bool)
        for i in range(len(lhat)):
            if not merged[i]:
                continue
            for j in range(i + 1, len(lhat)):
                if merged[j] and col[i] == col[j] and np.linalg.norm(lhat[i] - lhat[j]) < 0.5 * self.new_landmark_dist + 0.2:
                    wi, wj = count[i], count[j]
                    lhat[i] = (wi * lhat[i] + wj * lhat[j]) / (wi + wj + 1e-9)
                    votes[i] += votes[j]
                    count[i] += count[j]
                    merged[j] = False
        self.lhat, self.votes, self.count = lhat[merged], votes[merged], count[merged]
        self.frozen = True

    def localise(self, pose_guess, z_rel, colour, wide=True, subset=None):
        """Correct a dead-reckoned position against the frozen map. Returns the corrected (x, y)
        and how many cones matched. With `wide`, a failed match is retried as a unique translation
        search within 3 m."""
        pose_guess = np.asarray(pose_guess, float)
        z_rel, colour = np.asarray(z_rel, float).reshape(-1, 2), np.asarray(colour, int)
        if len(z_rel) < 3:
            return pose_guess, 0
        if subset is not None:                  # only the landmarks along the stretch the car is on
            saved = (self.lhat, self.votes, self.count, self.colour_override)
            try:
                self.lhat, self.votes, self.count = self.lhat[subset], self.votes[subset], self.count[subset]
                self.colour_override = None if saved[3] is None else saved[3][subset]
                return self.localise(pose_guess, z_rel, colour, wide=wide)
            finally:
                self.lhat, self.votes, self.count, self.colour_override = saved
        zw = z_rel + pose_guess
        saved_min = self.icp_min_matches
        self.icp_min_matches = 3                 # a hairpin shows only a few cones; three are enough to hold position
        try:
            t, R = self.icp(zw, colour)
        finally:
            self.icp_min_matches = saved_min
        if not np.any(t) and wide:
            # nothing within the usual gate: the car may have drifted more than that. A wider
            # ICP net would happily lock one cone along on a regular layout, so instead search
            # the translations within 3 m and accept only a clear, unique fit.
            found = self.relocalise(z_rel, colour, near=pose_guess, radius=3.0)
            if found is None:
                return pose_guess, 0
            t, R = found - pose_guess, np.eye(2)
        if not np.any(t):
            return pose_guess, 0
        zz = zw @ R.T + t
        d = np.linalg.norm(zz[:, None, :] - self.lhat[None, :, :], axis=2)
        matched = int(np.sum(d.min(axis=1) < 0.5))
        return R @ pose_guess + t, matched
