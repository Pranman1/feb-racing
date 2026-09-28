"""Closed-loop bench for the racer's controller, without the simulator.

    ros2 run feb_cone_racer mpc_bench tracks/fs_comp_2021/track.json [-p w_dsteer:=2.0 ...]

The controller under test is the real `BicycleMPC` and the real reference builder; what it
drives is a plant with the things the MPC's own model leaves out, because those are what make
a controller oscillate on the car and not in the plan:

  * the steering servo is rate limited (3.2 rad/s) as well as lagged, so a command the MPC
    cannot physically execute shows up as the error it really is;
  * the command reaches the wheels one `STEER_DELAY` later;
  * the loop runs at the bridge's rate (10 Hz), not at the plant's;
  * the plant integrates at 1 ms.

It reports what matters for driving and not only for lap time: how hard the wheel is worked
(peak and RMS steering rate), how often it reverses direction, and how far the car runs from
the line. A controller that oscillates scores hundreds of reversals a lap; a controller that
drives scores a handful per corner.
"""
import argparse
import json
import math
import sys
import time

import numpy as np

from .mpc import BicycleMPC
from .raceline import heading_along, speed_profile
from .racer import DEFAULTS, MAX_STEER, MAX_STEER_RATE, STEER_DELAY, WHEELBASE
from .track import resample_closed


class Plant:
    """The car. Same equations as the MPC's model, plus the actuator it does not model."""

    def __init__(self, p, z0):
        self.p = p
        self.z = np.array(z0, float)          # x, y, psi, vx, vy, r, delta

    def deriv(self, z, dcmd, tau):
        p = self.p
        _, _, psi, vx, vy, r, d = z
        rate = np.clip((dcmd - d) / p["mpc_steer_tau"], -MAX_STEER_RATE, MAX_STEER_RATE)
        vxs = max(vx, 0.3)
        af = d - math.atan((vy + p["lf"] * r) / vxs)
        ar = -math.atan((vy - p["lr"] * r) / vxs)
        Ff = p["tyre_D"] * math.sin(p["tyre_C"] * math.atan(p["tyre_B"] * af))
        Fr = p["tyre_D"] * math.sin(p["tyre_C"] * math.atan(p["tyre_B"] * ar))
        ax = p["long_a"] * tau - p["long_b"] * vx - p["idle_brake"] * (1.0 - math.tanh(40.0 * tau))
        return np.array([vx * math.cos(psi) - vy * math.sin(psi),
                         vx * math.sin(psi) + vy * math.cos(psi),
                         r,
                         ax + vy * r,
                         (Ff * math.cos(d) + Fr) / p["mass"] - vx * r,
                         (p["lf"] * Ff * math.cos(d) - p["lr"] * Fr) / p["inertia_z"],
                         rate])

    def step(self, dcmd, tau, dt, h=0.001):
        for _ in range(max(int(round(dt / h)), 1)):
            k1 = self.deriv(self.z, dcmd, tau)
            k2 = self.deriv(self.z + h / 2 * k1, dcmd, tau)
            k3 = self.deriv(self.z + h / 2 * k2, dcmd, tau)
            k4 = self.deriv(self.z + h * k3, dcmd, tau)
            self.z = self.z + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            self.z[3] = max(self.z[3], 0.0)
            self.z[6] = float(np.clip(self.z[6], -MAX_STEER, MAX_STEER))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def build_line(track_json, p):
    """A raceline from a track's centreline: the geometry is the real track's, which is what
    the controller is being judged on; the line itself is the centre, not the optimum."""
    t = json.load(open(track_json))
    c = np.array(t["centreline"], float).reshape(-1, 2)          # track.json stores it flat
    line, s = resample_closed(c, p["sample_step"])
    v, _ = speed_profile(line, p["v_max"] * p["race_speed_scale"],
                         p["a_lat"] * p["corner_scale"] ** 2, p["a_acc"], p["a_brake"])
    return line, s, v, heading_along(line)


def reference(line, race_s, race_v, race_psi, i0, N, DT, step):
    """Mirrors Racer.reference: raceline points ahead, one per stage, spaced by profile speed."""
    n = len(line)
    s = race_s[i0]
    ref = np.zeros((4, N))
    idx = i0
    for k in range(N):
        s += race_v[idx] * DT
        idx = (i0 + int(round((s - race_s[i0]) / step))) % n
        ref[0, k], ref[1, k] = line[idx]
        ref[2, k] = race_psi[idx]
        ref[3, k] = race_v[idx]
    return ref


def run(track_json, p, rate=10.0, laps=1.0, slip=0.0, speed_source="wheel", jitter=0.0, seed=1):
    line, s_line, race_v, race_psi = build_line(track_json, p)
    n = len(line)
    mpc = BicycleMPC(p)
    if not mpc.ok:
        sys.exit("CasADi is missing: install it (starter_kit/feb_cone_racer/install_deps.sh)")
    z = np.array([line[0, 0], line[0, 1], race_psi[0], max(race_v[0], 0.5), 0.0, 0.0, 0.0])
    plant = Plant(p, z)
    dt = 1.0 / rate
    rng = np.random.default_rng(seed)
    delay = [(0.0, 0.0)] * max(int(round(STEER_DELAY / dt)), 1)
    uprev = np.array([0.0, 0.0])
    lap_len = float(s_line[-1])
    travelled, t, idx_prev = 0.0, 0.0, 0
    steers, rates, errs, speeds, solves, ripple = [], [], [], [], [], []
    last_cmd, last_tau, wheel = 0.0, 0.0, 0.0
    last_pos = plant.z[:2].copy()
    while travelled < laps * lap_len and t < 4.0 * laps * lap_len:
        x, y, psi, vx, vy, r, d = plant.z
        # what the controller is told the speed is. The wheels over-read when they slip, which
        # they do under torque, and the racer feeds that straight to the MPC; `pose` is what a
        # map match gives instead, the displacement between scans.
        wheel = 0.5 * wheel + 0.5 * (vx + slip * last_tau)
        v_meas = wheel if speed_source == "wheel" else float(np.linalg.norm(plant.z[:2] - last_pos)) / dt
        last_pos = plant.z[:2].copy()
        i = int(np.argmin(np.linalg.norm(line - np.array([x, y]), axis=1)))
        # the racer's own delay compensation, kinematic over STEER_DELAY
        px, py, ppsi = x, y, psi
        for _ in range(3):
            h = STEER_DELAY / 3.0
            px += vx * math.cos(ppsi) * h
            py += vx * math.sin(ppsi) * h
            ppsi += vx * math.tan(d) / WHEELBASE * h
        ref = reference(line, s_line, race_v, race_psi, i, mpc.N, mpc.DT, p["sample_step"])
        ref[2] = ppsi + np.array([wrap(a - ppsi) for a in ref[2]])
        t0 = time.time()
        out = mpc.solve(np.array([px, py, ppsi, max(v_meas, 0.0), 0.0, r, d]), ref, uprev)
        solves.append(1000.0 * (time.time() - t0))
        if out is None:
            cmd, tau = last_cmd, 0.0
        else:
            uprev = out[0]
            cmd, tau = float(np.clip(out[0][0], -MAX_STEER, MAX_STEER)), float(np.clip(out[0][1], 0.0, p["tau_max"]))
        last_tau = tau
        delay.append((cmd, tau))
        applied = delay.pop(0)
        # the bridge does not deliver on a metronome: the command is held for however long the
        # next scan takes, while the MPC planned it for exactly mpc_dt
        held = float(np.clip(dt + jitter * rng.standard_normal(), 0.4 * dt, 2.0 * dt)) if jitter > 0 else dt
        plant.step(applied[0], applied[1], held)
        rates.append((cmd - last_cmd) / dt)
        last_cmd = cmd
        steers.append(cmd)
        errs.append(float(np.min(np.linalg.norm(line - plant.z[:2], axis=1))))
        speeds.append(float(plant.z[3]))
        ripple.append(float(plant.z[3]))
        step_s = s_line[i] - s_line[idx_prev]
        travelled += step_s + (lap_len if step_s < -0.5 * lap_len else 0.0)
        idx_prev = i
        t += held if jitter > 0 else dt
        if errs[-1] > 3.0:
            break
    rates = np.array(rates)
    big = np.abs(rates) > 0.5                       # a real steering move, not numerical dust
    sign = np.sign(rates[big]) if big.any() else np.array([0.0])
    reversals = int(np.sum(sign[1:] != sign[:-1]))
    laps_done = travelled / lap_len
    out = dict(lap_time=t / max(laps_done, 1e-6), laps=laps_done, finished=errs[-1] <= 3.0,
               err_rms=float(np.sqrt(np.mean(np.square(errs)))), err_max=float(np.max(errs)),
               rate_rms=float(np.sqrt(np.mean(np.square(rates)))), rate_max=float(np.max(np.abs(rates))),
               reversals=reversals, reversals_per_100m=100.0 * reversals / max(travelled, 1.0),
               v_mean=float(np.mean(speeds)), v_max=float(np.max(speeds)), solve_ms=float(np.mean(solves)),
               v_ripple=float(np.std(np.array(ripple) - np.convolve(ripple, np.ones(11) / 11, mode="same"))),
               saturated=float(np.mean(np.abs(np.array(steers)) > 0.95 * MAX_STEER)))
    return out


def crawl(p, v_target=1.2, secs=25.0, rate=10.0, vary=0.0, period=1.0):
    """The lap-one speed law against the same longitudinal model: does the crawl hold its
    speed, or surge and coast? `throttle_law` is called exactly as the node calls it. With
    `vary` the target steps between v_target and v_target - vary every `period` seconds, the
    way the follower's does when the known path shortens or the wheel turns."""
    from .racer import Racer
    shim = type("S", (), {})()
    shim.p, shim.integral, shim.throttle, shim.speed, shim.steer, shim.v_cmd = p, 0.0, 0.0, 0.0, 0.0, 0.0
    dt, vx, t = 1.0 / rate, 0.0, 0.0
    vs, taus = [], []
    while t < secs:
        v_t = v_target - (vary if int(t / period) % 2 else 0.0)
        tau = Racer.throttle_law(shim, v_t, dt)
        shim.throttle = tau
        for _ in range(int(round(dt / 0.001))):
            dv = p["long_a"] * tau - p["long_b"] * vx - p["idle_brake"] * (1.0 - math.tanh(40.0 * tau))
            vx = max(vx + 0.001 * dv, 0.0)
        shim.speed = 0.5 * shim.speed + 0.5 * vx        # the encoder smoothing the node does
        t += held if jitter > 0 else dt
        if t > 5.0:
            vs.append(vx)
            taus.append(tau)
    vs, taus = np.array(vs), np.array(taus)
    braking = float(np.mean(taus < 0.03))
    return dict(v_mean=float(vs.mean()), v_ripple=float(vs.max() - vs.min()), v_std=float(vs.std()),
                tau_mean=float(taus.mean()), tau_min=float(taus.min()), tau_max=float(taus.max()), braking=braking)


def report(name, m):
    print("%-22s %s lap %6.1f s (%.2f laps) | line error rms %.2f max %.2f m | steering rate rms %5.2f max %5.2f rad/s | "
          "reversals %4d | v mean %.2f ripple %.2f m/s | mpc %.0f ms"
          % (name, "  " if m["finished"] else "LOST", m["lap_time"], m["laps"], m["err_rms"], m["err_max"],
             m["rate_rms"], m["rate_max"], m["reversals"], m["v_mean"], m["v_ripple"], m["solve_ms"]))


def main(argv=None):
    ap = argparse.ArgumentParser(description="closed-loop bench for the racer's MPC, no simulator")
    ap.add_argument("track", nargs="?", help="a track.json (its centreline is the line to follow)")
    ap.add_argument("--laps", type=float, default=1.0)
    ap.add_argument("--jitter", type=float, default=0.0, help="standard deviation of the control period, seconds (the bridge shows 0.024)")
    ap.add_argument("--slip", type=float, default=0.0, help="wheel over-read per unit throttle, m/s (the simulator shows about 3.5)")
    ap.add_argument("--speed-source", choices=["wheel", "pose"], default="wheel", help="what the controller is told the speed is")
    ap.add_argument("--crawl", action="store_true", help="bench the lap-one speed law instead of the MPC")
    ap.add_argument("--period", type=float, default=1.0, help="with --vary: how often the target dips, seconds")
    ap.add_argument("--vary", type=float, default=0.0, help="with --crawl: how far the target dips every second (the follower slows for steering and a short path)")
    ap.add_argument("--rate", type=float, default=10.0, help="control rate, Hz (the bridge runs at 10)")
    ap.add_argument("-p", "--param", action="append", default=[], metavar="name:=value",
                    help="override a racer parameter, e.g. -p w_dsteer:=2.0")
    args = ap.parse_args(argv)
    p = dict(DEFAULTS)
    for o in args.param:
        k, _, v = o.partition(":=")
        p[k] = type(p[k])(float(v)) if isinstance(p[k], (int, float)) and not isinstance(p[k], bool) else v
    if args.crawl:
        c = crawl(p, p["map_speed"], vary=args.vary, period=args.period)
        print("lap-one crawl at %.1f m/s: speed mean %.2f, ripple %.2f, std %.3f m/s | throttle %.3f (%.3f..%.3f), "
              "in the idle brake %.0f%% of the time" % (p["map_speed"], c["v_mean"], c["v_ripple"], c["v_std"],
                                                        c["tau_mean"], c["tau_min"], c["tau_max"], 100 * c["braking"]))
        return
    report("baseline", run(args.track, p, args.rate, args.laps, args.slip, args.speed_source, args.jitter))


if __name__ == "__main__":
    main()
