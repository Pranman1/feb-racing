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
    N = len(centre)
    bound = np.maximum(half_width - car_half_width - margin, 0.03)
    # p_i = c_i + a_i n_i ; second difference D p ; minimise |D p|^2 + reg |a|^2
    idx = np.arange(N)
    D = np.zeros((N, N))
    D[idx, idx] = -2.0
    D[idx, (idx + 1) % N] = 1.0
    D[idx, (idx - 1) % N] = 1.0
    Cx, Cy = D @ centre[:, 0], D @ centre[:, 1]
    Nx, Ny = D * normal[:, 0][None, :], D * normal[:, 1][None, :]
    H = 2.0 * (Nx.T @ Nx + Ny.T @ Ny) + 2.0 * reg * np.eye(N)
    g = 2.0 * (Nx.T @ Cx + Ny.T @ Cy)
    try:
        import casadi as ca
        a = ca.MX.sym("a", N)
        qp = {"x": a, "f": 0.5 * ca.dot(a, ca.mtimes(ca.DM(H), a)) + ca.dot(ca.DM(g), a)}
        try:
            solver = ca.qpsol("qp", "qrqp", qp, {"print_iter": False, "print_header": False, "error_on_fail": False})
        except Exception:
            solver = ca.nlpsol("qp", "ipopt", qp, {"ipopt.print_level": 0, "print_time": False})
        sol = solver(lbx=-bound, ubx=bound, x0=np.zeros(N))
        alpha = np.asarray(sol["x"]).flatten()
    except Exception:
        # projected gradient fallback, no CasADi
        alpha = np.zeros(N)
        L = np.linalg.eigvalsh(H).max()
        for _ in range(400):
            alpha = np.clip(alpha - (H @ alpha + g) / L, -bound, bound)
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
