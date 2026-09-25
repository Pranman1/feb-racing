"""Raceline: minimum-curvature path inside the track, then a speed profile.

Minimum curvature (the usual first-order approximation of minimum time on a narrow track):
every centreline sample i may move sideways by alpha_i along its normal, within the track less
a margin for the car's half width. Curvature is approximated by the second difference of the
positions, which is linear in alpha, so the objective is a quadratic in alpha and the problem
is a box-constrained QP. CasADi's qrqp solves it in milliseconds; ipopt is the fallback.

Speed profile: the lateral-acceleration limit caps speed at each point from the curvature;
a forward pass limits acceleration and a backward pass limits braking, both run twice around
the loop so the closure is consistent.
"""
import numpy as np


def min_curvature(centre, normal, half_width, car_half_width=0.135, margin=0.12, reg=0.02):
    """Minimum-curvature offsets alpha along the normals, a box-constrained QP. The Hessian is
    pentadiagonal (second differences), so it is built sparse: a 1300-point track solves in
    well under a second instead of minutes. CasADi's qrqp first, scipy's L-BFGS-B as fallback."""
    import scipy.sparse as sps
    N = len(centre)
    bound = np.maximum(half_width - car_half_width - margin, 0.03)
    idx = np.arange(N)
    ds = float(np.median(np.linalg.norm(np.roll(centre, -1, axis=0) - centre, axis=1)))
    # second difference divided by ds^2 approximates curvature, so the objective (and reg) mean
    # the same thing whatever the sampling
    D = sps.csr_matrix((np.concatenate([-2.0 * np.ones(N), np.ones(N), np.ones(N)]) / (ds * ds),
                        (np.concatenate([idx, idx, idx]), np.concatenate([idx, (idx + 1) % N, (idx - 1) % N]))), shape=(N, N))
    Cx, Cy = D @ centre[:, 0], D @ centre[:, 1]
    Nx, Ny = D @ sps.diags(normal[:, 0]), D @ sps.diags(normal[:, 1])
    H = (2.0 * (Nx.T @ Nx + Ny.T @ Ny) + 2.0 * reg * sps.eye(N)).tocsc()
    H.sum_duplicates()
    H.sort_indices()                      # CasADi requires sorted compressed-column storage
    g = 2.0 * (Nx.T @ Cx + Ny.T @ Cy)
    alpha = None
    try:
        import casadi as ca
        a = ca.MX.sym("a", N)
        Hc = ca.DM(H)
        qp = {"x": a, "f": 0.5 * ca.dot(a, ca.mtimes(Hc, a)) + ca.dot(ca.DM(g), a)}
        solver = ca.qpsol("qp", "qrqp", qp, {"print_iter": False, "print_header": False, "error_on_fail": False})
        sol = solver(lbx=-bound, ubx=bound, x0=np.zeros(N))
        alpha = np.asarray(sol["x"]).flatten()
        if not np.all(np.isfinite(alpha)):
            alpha = None
    except Exception:
        alpha = None
    if alpha is None:
        from scipy.optimize import minimize
        res = minimize(lambda x: 0.5 * x @ (H @ x) + g @ x, np.zeros(N), jac=lambda x: H @ x + g,
                       method="L-BFGS-B", bounds=list(zip(-bound, bound)), options={"maxiter": 500})
        alpha = res.x
    return centre + alpha[:, None] * normal, alpha


def curvature(path):
    nxt, prv = np.roll(path, -1, axis=0), np.roll(path, 1, axis=0)
    d1 = (nxt - prv) / 2.0
    d2 = nxt - 2.0 * path + prv
    num = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    den = (d1[:, 0] ** 2 + d1[:, 1] ** 2) ** 1.5 + 1e-9
    return num / den


def speed_profile(path, v_max, a_lat, a_acc, a_brake, v_min=0.5):
    ds = np.linalg.norm(np.roll(path, -1, axis=0) - path, axis=1)
    kappa = np.abs(curvature(path))
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(kappa, 1e-6)))
    N = len(v)
    for _ in range(2):
        for i in range(N):                                   # forward: acceleration limit
            j = (i + 1) % N
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * a_acc * ds[i]))
        for i in range(N - 1, -1, -1):                       # backward: braking limit
            j = (i - 1) % N
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * a_brake * ds[j]))
    return np.maximum(v, v_min), kappa


def heading_along(path):
    nxt, prv = np.roll(path, -1, axis=0), np.roll(path, 1, axis=0)
    d = nxt - prv
    return np.arctan2(d[:, 1], d[:, 0])
