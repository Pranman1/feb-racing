"""Nonlinear MPC on a dynamic bicycle model, in CasADi.

State  z = [x, y, psi, vx, vy, r, delta]      (world position, heading, body velocities, yaw
                                               rate, steering angle at the wheels)
Input  u = [delta_cmd, tau]                    (commanded steering angle, throttle 0..1)

Steering actuator: the wheels follow the command through a first-order lag,
    ddelta = (delta_cmd - delta) / tau_s,
close to what the simulator's servo does (about 0.19 s to respond, 3.2 rad/s at most).

Longitudinal: the simulator's throttle behaves like a first-order motor,
    dvx = a*tau - b*vx - c*brake(tau) + vy*r,   brake(tau) ~ 1 when tau is near zero
with a, b, c from the starter kit's sysid fit (throttle 0 is a hard brake on this car).
Lateral: Pacejka-style tyres on front and rear, slip angles from the body velocities,
    Fy = D sin(C atan(B alpha)),  dvy = (Fyf cos(delta) + Fyr)/m - vx r,
    dr  = (lf Fyf cos(delta) - lr Fyr)/Iz.
Direct multiple shooting, RK4, horizon N steps of DT. Cost: distance to the raceline points
advanced along the speed profile, heading error, speed error, input rates. IPOPT solves it,
warm-started from the last solution; a few tens of milliseconds for N = 12.
"""
import numpy as np

try:
    import casadi as ca
except ImportError:                  # the node reports this clearly at startup
    ca = None


class BicycleMPC:
    def __init__(self, p):
        self.p = p
        self.N, self.DT = int(p["mpc_horizon"]), float(p["mpc_dt"])
        self.ok = ca is not None
        if not self.ok:
            return
        self.build()
        self.warm = None

    # ------------------------------------------------------------ model

    def dynamics(self, z, u):
        p = self.p
        x, y, psi, vx, vy, r, d = z[0], z[1], z[2], z[3], z[4], z[5], z[6]
        dcmd, tau = u[0], u[1]
        dd = (dcmd - d) / p["mpc_steer_tau"]          # first-order servo lag (the rate limit is close to this at tau 0.15 s)
        m, Iz, lf, lr = p["mass"], p["inertia_z"], p["lf"], p["lr"]
        vxs = ca.fmax(vx, 0.3)                                  # no slip-angle blow-up at rest
        af = d - ca.atan((vy + lf * r) / vxs)
        ar = -ca.atan((vy - lr * r) / vxs)
        Ff = p["tyre_D"] * ca.sin(p["tyre_C"] * ca.atan(p["tyre_B"] * af))
        Fr = p["tyre_D"] * ca.sin(p["tyre_C"] * ca.atan(p["tyre_B"] * ar))
        ax = p["long_a"] * tau - p["long_b"] * vx - p["idle_brake"] * (1.0 - ca.tanh(40.0 * tau))   # idle brake fades in as the throttle closes
        return ca.vertcat(vx * ca.cos(psi) - vy * ca.sin(psi),
                          vx * ca.sin(psi) + vy * ca.cos(psi),
                          r,
                          ax + vy * r,
                          (Ff * ca.cos(d) + Fr) / m - vx * r,
                          (lf * Ff * ca.cos(d) - lr * Fr) / Iz,
                          dd)

    def step(self, z, u):
        """One MPC interval, integrated in a few RK4 substeps: the yaw dynamics of a 4 kg car
        are fast (small inertia), so one 0.1 s step would be inaccurate and hard to solve."""
        n = int(self.p["mpc_substeps"])
        h = self.DT / n
        for _ in range(n):
            k1 = self.dynamics(z, u)
            k2 = self.dynamics(z + h / 2 * k1, u)
            k3 = self.dynamics(z + h / 2 * k2, u)
            k4 = self.dynamics(z + h * k3, u)
            z = z + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return z

    # ------------------------------------------------------------ problem

    def build(self):
        p, N = self.p, self.N
        Z = ca.SX.sym("Z", 7, N + 1)
        U = ca.SX.sym("U", 2, N)
        z0 = ca.SX.sym("z0", 7)
        ref = ca.SX.sym("ref", 4, N)          # x, y, psi, v per stage
        uprev = ca.SX.sym("uprev", 2)
        g, lbg, ubg = [], [], []
        g.append(Z[:, 0] - z0)
        lbg += [0] * 7
        ubg += [0] * 7
        cost = 0
        for k in range(N):
            zn = self.step(Z[:, k], U[:, k])
            g.append(Z[:, k + 1] - zn)
            lbg += [0] * 7
            ubg += [0] * 7
            # throttle may only change so fast: a hard bound, so the solution cannot chatter
            g.append(U[1, k] - (uprev[1] if k == 0 else U[1, k - 1]))
            lbg += [-p["dtau_max"] * self.DT]
            ubg += [p["dtau_max"] * self.DT]
            zk = Z[:, k + 1]
            ex, ey = zk[0] - ref[0, k], zk[1] - ref[1, k]
            cost += p["w_pos"] * (ex ** 2 + ey ** 2)
            cost += p["w_head"] * (1 - ca.cos(zk[2] - ref[2, k]))
            cost += p["w_speed"] * (zk[3] - ref[3, k]) ** 2
            cost += p["w_vy"] * zk[4] ** 2
            prev = uprev if k == 0 else U[:, k - 1]
            cost += p["w_dsteer"] * ((U[0, k] - prev[0]) / self.DT) ** 2
            cost += p["w_dtau"] * ((U[1, k] - prev[1]) / self.DT) ** 2
            cost += p["w_tau"] * U[1, k] ** 2
        w = ca.vertcat(ca.vec(Z), ca.vec(U))
        params = ca.vertcat(z0, ca.vec(ref), uprev)
        nlp = {"x": w, "f": cost, "g": ca.vertcat(*g), "p": params}
        opts = {"ipopt.print_level": 0, "print_time": False, "ipopt.max_iter": int(p["mpc_max_iter"]),
                "ipopt.tol": 1e-3, "ipopt.acceptable_tol": 1e-2, "ipopt.warm_start_init_point": "yes",
                "ipopt.mu_strategy": "adaptive"}
        self.solver = ca.nlpsol("mpc", "ipopt", nlp, opts)
        self.lbg, self.ubg = np.array(lbg, float), np.array(ubg, float)
        # variable bounds
        lbz = np.tile([-np.inf, -np.inf, -np.inf, 0.0, -5.0, -10.0, -p["max_steer"]], N + 1)
        ubz = np.tile([np.inf, np.inf, np.inf, p["v_cap"], 5.0, 10.0, p["max_steer"]], N + 1)
        lbu = np.tile([-p["max_steer"], 0.0], N)
        ubu = np.tile([p["max_steer"], p["tau_max"]], N)
        self.lbx, self.ubx = np.concatenate([lbz, lbu]), np.concatenate([ubz, ubu])

    def solve(self, z0, ref, uprev):
        """z0 (7,), ref (4, N), uprev (2,). Returns (u0 (2,), predicted states (7, N+1)) or None."""
        N = self.N
        if self.warm is None:
            Zg = np.repeat(np.asarray(z0, float)[:, None], N + 1, axis=1)
            Zg[0, 1:], Zg[1, 1:], Zg[2, 1:], Zg[3, 1:] = ref[0], ref[1], ref[2], ref[3]
            Ug = np.zeros((2, N))
            Ug[0, :] = z0[6]
            Ug[1, :] = uprev[1]
            x0 = np.concatenate([Zg.flatten(order="F"), Ug.flatten(order="F")])
        else:
            x0 = self.warm
        pvec = np.concatenate([np.asarray(z0, float), np.asarray(ref, float).flatten(order="F"), np.asarray(uprev, float)])
        try:
            sol = self.solver(x0=x0, p=pvec, lbx=self.lbx, ubx=self.ubx, lbg=self.lbg, ubg=self.ubg)
        except Exception:
            self.warm = None
            return None
        w = np.asarray(sol["x"]).flatten()
        stat = self.solver.stats()
        if not stat.get("success", False) and stat.get("return_status", "") not in ("Solve_Succeeded", "Solved_To_Acceptable_Level", "Maximum_Iterations_Exceeded"):
            self.warm = None
            return None
        Z = w[: 7 * (N + 1)].reshape((7, N + 1), order="F")
        U = w[7 * (N + 1):].reshape((2, N), order="F")
        # shift for the next warm start
        Zs = np.hstack([Z[:, 1:], Z[:, -1:]])
        Us = np.hstack([U[:, 1:], U[:, -1:]])
        self.warm = np.concatenate([Zs.flatten(order="F"), Us.flatten(order="F")])
        return U[:, 0], Z
