"""Fit the longitudinal model to the measured bursts. The car's position updates at 12 Hz, so
distance travelled is the reliable signal, not a differentiated speed: simulate each burst and
match the distance curve."""
import pickle, numpy as np
from scipy.optimize import least_squares

B = pickle.load(open(__import__("sys").argv[1] if len(__import__("sys").argv)>1 else "longid.pkl", "rb"))

def clean(rec):
    a = np.array([r[:5] for r in rec], dtype=float)
    t = a[:, 0] - a[0, 0]; x, y, tau = a[:, 1], a[:, 2], a[:, 3]
    keep = [0] + [i for i in range(1, len(t)) if (x[i] != x[i - 1] or y[i] != y[i - 1])]
    t, x, y, tau = t[keep], x[keep], y[keep], tau[keep]
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return t, s, tau

runs = [clean(r) for _, r in B]

def simulate(p, t, tau):
    a, b, sat, brake = p
    v = 0.0; s = 0.0; out = np.empty(len(t))
    for i in range(len(t)):
        out[i] = s
        if i + 1 == len(t): break
        h = (t[i + 1] - t[i]) / 4.0
        for _ in range(4):
            ax = min(a * tau[i] - b * v, sat)
            if tau[i] <= 1e-9: ax -= brake         # the simulator brakes all four wheels at exactly zero
            v = max(v + ax * h, 0.0)
            s += v * h
    return out

def resid(p):
    if min(p) <= 0: return np.full(sum(len(t) for t, _, _ in runs), 1e3)
    return np.concatenate([simulate(p, t, tau) - s for t, s, tau in runs])

best = least_squares(resid, [35.0, 1.7, 6.0, 3.0], bounds=([5, 0.2, 1.0, 0.0], [200, 12, 20, 30]))
a, b, sat, brake = best.x
r = resid(best.x)
print("fit:  long_a %.1f   long_b %.2f   ax_sat %.2f   extra brake at zero throttle %.2f m/s^2" % (a, b, sat, brake))
print("      steady speed per unit throttle a/b = %.1f m/s,  time constant 1/b = %.2f s" % (a / b, 1 / b))
print("      distance residual rms %.2f m over %d samples" % (np.sqrt(np.mean(r ** 2)), len(r)))
print("\nold model for comparison (a 152, b 6.09, sat 9.8, brake 16.5):")
r0 = resid([152.0, 6.09, 9.8, 16.5])
print("      distance residual rms %.2f m" % np.sqrt(np.mean(r0 ** 2)))
print("\npredicted vs measured speed at 3.0-3.25 s, throttle held from rest:")
print("  tau   measured  new model  old model")
for (t, s, tau), (lab, _) in zip(runs, B):
    if not isinstance(lab, float): continue
    def vat(sim):
        m = (t > 3.0) & (t < 3.3)
        return (sim[m][-1] - sim[m][0]) / (t[m][-1] - t[m][0]) if m.sum() > 2 else np.nan
    print("  %.2f    %6.2f    %6.2f    %6.2f" % (lab, vat(s), vat(simulate(best.x, t, tau)), vat(simulate([152.0, 6.09, 9.8, 16.5], t, tau))))
print("\nhow long to reach 90%% of steady speed at throttle 0.25:  new %.2f s   old %.2f s"
      % (np.log(10) / b, np.log(10) / 6.09))
