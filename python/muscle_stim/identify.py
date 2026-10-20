"""System identification and model-based controller tuning.

Workflow (the same functions are used on simulated data and on real session logs):

1. Staircase test (open loop, short holds, rests between) -> static recruitment curve
   -> look-up table (LUT) for the inverse recruitment in the firmware.
2. Small force-demand step test with the LUT loaded -> fit a second-order-plus-dead-time
   (SOPDT) model  G(s) = K exp(-theta s) / ((tau1 s + 1)(tau2 s + 1)).
3. SIMC rules (Skogestad) turn the SOPDT parameters into PI / PID gains.

Command line:
    python -m muscle_stim.identify recruitment session.csv --out lut.txt
    python -m muscle_stim.identify dynamics   session.csv --t0 12 --t1 18
where session.csv is a log from log_serial.py (columns t_ms, u, v_N, force_N, ...).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .controllers import PidGains


# --------------------------------------------------------------------------- #
# 1. static recruitment curve from a staircase test
# --------------------------------------------------------------------------- #
def recruitment_from_staircase(t, u, force, min_plateau_s: float = 1.0, settle_frac: float = 0.5):
    """Extract (u_i, F_i) from a staircase test.

    A plateau is a stretch where u is constant. The force on each plateau is the mean over the
    last (1 - settle_frac) of the plateau, so the activation transient is ignored.
    Returns u_i and F_i (N), sorted by u and made monotone non-decreasing.
    F at u = 0 is used as the zero offset of the sensor and subtracted.
    """
    t = np.asarray(t, float)
    u = np.asarray(u, float)
    f = np.asarray(force, float)
    change = np.flatnonzero(np.abs(np.diff(u)) > 1e-6) + 1
    edges = np.concatenate(([0], change, [len(u)]))
    pts = {}
    for a, b in zip(edges[:-1], edges[1:]):
        if t[b - 1] - t[a] < min_plateau_s:
            continue
        a2 = a + int(settle_frac * (b - a))
        key = round(float(np.median(u[a:b])), 4)
        pts.setdefault(key, []).append(float(np.mean(f[a2:b])))
    if len(pts) < 5:
        raise ValueError(f"only {len(pts)} usable plateaus found, need at least 5")
    uu = np.array(sorted(pts))
    ff = np.array([np.mean(pts[k]) for k in uu])
    if uu[0] > 0.0:
        raise ValueError("staircase must include u = 0 (rest) to remove the sensor offset")
    ff = np.maximum(ff - ff[0], 0.0)
    ff = _isotonic(ff)
    # Where the curve is flat at the top, extra intensity gives no extra force (saturation, or
    # fatigue during the test). Keep only the first point of the flat top, so the usable range
    # of u ends where force stops increasing.
    gain = np.diff(ff)
    keep = len(ff)
    for i in range(len(gain) - 1, -1, -1):
        if gain[i] > 0.02:
            break
        keep = i + 1
    return uu[:keep], ff[:keep]


def _isotonic(y):
    """Monotone non-decreasing least-squares fit (pool adjacent violators)."""
    blocks = []                       # each block: [sum, count]
    for val in y:
        blocks.append([float(val), 1])
        while len(blocks) > 1 and blocks[-2][0] / blocks[-2][1] > blocks[-1][0] / blocks[-1][1]:
            s, c = blocks.pop()
            blocks[-1][0] += s
            blocks[-1][1] += c
    out = []
    for s, c in blocks:
        out.extend([s / c] * c)
    return np.array(out)


def lut_text(u, f, name_u="RECRUIT_U", name_f="RECRUIT_F_N") -> str:
    """C++ arrays to paste into firmware/muscle_stim_pid/config.h."""
    fu = ", ".join(f"{x:.4f}f" for x in u)
    ff = ", ".join(f"{x:.4f}f" for x in f)
    n = len(u)
    return (f"constexpr int RECRUIT_N = {n};\n"
            f"constexpr float {name_u}[RECRUIT_N] = {{ {fu} }};\n"
            f"constexpr float {name_f}[RECRUIT_N] = {{ {ff} }};\n")


# --------------------------------------------------------------------------- #
# 2. dynamic model: second-order plus dead time
# --------------------------------------------------------------------------- #
@dataclass
class Sopdt:
    k: float        # N/N  static gain (force demand -> force)
    tau1: float     # s, slower lag
    tau2: float     # s, faster lag
    theta: float    # s, dead time
    rmse: float     # N, fit residual
    r2: float       # -, coefficient of determination


def simulate_sopdt(t, v, k, tau1, tau2, theta, v0=0.0):
    """Response of K exp(-theta s)/((tau1 s+1)(tau2 s+1)) to input v on a uniform time grid."""
    t = np.asarray(t, float)
    dt = float(np.mean(np.diff(t)))
    vd = np.interp(t - theta, t, np.asarray(v, float) - v0, left=0.0)
    a1 = 1.0 - np.exp(-dt / tau1)
    a2 = 1.0 - np.exp(-dt / tau2)
    y1 = 0.0
    y2 = 0.0
    out = np.empty_like(vd)
    for i, x in enumerate(vd):
        y1 += a1 * (x - y1)
        y2 += a2 * (y1 - y2)
        out[i] = k * y2
    return out


def fit_sopdt(t, v, force, n_pre: int | None = None) -> Sopdt:
    """Fit an SOPDT model to a step test.

    t, v, force : uniformly sampled arrays covering a short window that starts with a few
                  samples of rest/baseline BEFORE the step. n_pre = number of samples before the
                  step used to estimate the baselines (default: first 10 % of the window).
    """
    t = np.asarray(t, float)
    v = np.asarray(v, float)
    y = np.asarray(force, float)
    n_pre = n_pre or max(5, len(t) // 10)
    v0 = float(np.mean(v[:n_pre]))
    y0 = float(np.mean(y[:n_pre]))
    yy = y - y0
    dv = float(np.mean(v[-n_pre:]) - v0)
    if abs(dv) < 1e-6:
        raise ValueError("no step found in the window")

    def resid(p):
        k, t1, t2, th = p
        return simulate_sopdt(t, v, k, t1, t2, th, v0) - yy

    best = None
    for t1_0, t2_0 in [(0.15, 0.05), (0.3, 0.1), (0.08, 0.04)]:
        r = least_squares(resid, x0=[1.0, t1_0, t2_0, 0.03],
                          bounds=([0.05, 0.01, 0.005, 0.0], [5.0, 3.0, 3.0, 0.5]))
        if best is None or r.cost < best.cost:
            best = r
    k, t1, t2, th = best.x
    if t2 > t1:
        t1, t2 = t2, t1
    res = resid([k, t1, t2, th])
    ss_res = float(np.sum(res ** 2))
    ss_tot = float(np.sum((yy - np.mean(yy)) ** 2))
    return Sopdt(k=float(k), tau1=float(t1), tau2=float(t2), theta=float(th),
                 rmse=float(np.sqrt(np.mean(res ** 2))), r2=1.0 - ss_res / max(ss_tot, 1e-12))


# --------------------------------------------------------------------------- #
# 3. SIMC tuning rules
# --------------------------------------------------------------------------- #
def simc_pi(m: Sopdt, tau_c: float) -> PidGains:
    """SIMC PI from an SOPDT model using the half rule (second lag + half of tau2 added to dead time)."""
    tau = m.tau1 + 0.5 * m.tau2
    theta = m.theta + 0.5 * m.tau2
    kc = tau / (m.k * (tau_c + theta))
    ti = min(tau, 4.0 * (tau_c + theta))
    return PidGains(kp=kc, ki=kc / ti, kd=0.0)


def simc_pid(m: Sopdt, tau_c: float, ts: float = 0.01, n_filter: float = 10.0) -> PidGains:
    """SIMC PID (series form converted to parallel form)."""
    kc = m.tau1 / (m.k * (tau_c + m.theta))
    ti = min(m.tau1, 4.0 * (tau_c + m.theta))
    td = m.tau2
    kp = kc * (1.0 + td / ti)
    ki = kc / ti
    kd = kc * td
    tf = max(2.0 * ts, td / n_filter)
    return PidGains(kp=kp, ki=ki, kd=kd, tf=tf)


# --------------------------------------------------------------------------- #
# command line
# --------------------------------------------------------------------------- #
def _load(path):
    import pandas as pd
    d = pd.read_csv(path, comment="#")
    d["t_s"] = d["t_ms"] / 1000.0
    return d


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("recruitment", help="LUT from a staircase test")
    a.add_argument("csv")
    a.add_argument("--out", default="recruitment_lut.txt")
    b = sub.add_parser("dynamics", help="SOPDT fit + SIMC gains from a force-demand step test")
    b.add_argument("csv")
    b.add_argument("--t0", type=float, required=True, help="window start (s), a little BEFORE the step")
    b.add_argument("--t1", type=float, required=True, help="window end (s)")
    b.add_argument("--tau-c", type=float, default=None, help="closed-loop time constant (s); default 1.0 x effective dead time (the value used in the simulations)")
    args = ap.parse_args(argv)

    d = _load(args.csv)
    if args.cmd == "recruitment":
        u, f = recruitment_from_staircase(d["t_s"], d["u"], d["force_N"])
        txt = lut_text(u, f)
        with open(args.out, "w") as fh:
            fh.write(txt)
        print(txt)
        print(f"written to {args.out}  (paste into config.h)")
    else:
        w = d[(d["t_s"] >= args.t0) & (d["t_s"] <= args.t1)]
        m = fit_sopdt(w["t_s"].to_numpy(), w["v_N"].to_numpy(), w["force_N"].to_numpy())
        print(f"SOPDT: K={m.k:.3f} N/N  tau1={m.tau1:.3f} s  tau2={m.tau2:.3f} s  theta={m.theta:.3f} s"
              f"  (RMSE {m.rmse:.3f} N, R2 {m.r2:.3f})")
        tc = args.tau_c if args.tau_c is not None else 1.0 * (m.theta + 0.5 * m.tau2)
        pi = simc_pi(m, tc)
        pid = simc_pid(m, tc)
        print(f"tau_c = {tc:.3f} s")
        print(f"PI : Kp={pi.kp:.3f}  Ki={pi.ki:.3f} 1/s")
        print(f"PID: Kp={pid.kp:.3f}  Ki={pid.ki:.3f} 1/s  Kd={pid.kd:.4f} s  Tf={pid.tf:.3f} s")
        print("Send to the device with:  SET KP <v>  SET KI <v>  SET KD <v>")


if __name__ == "__main__":
    main()
