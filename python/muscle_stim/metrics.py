"""Performance metrics shared by the simulator and by analyze_session.py (real data).

All metrics take plain numpy arrays so they work on simulated and on measured data.
In simulation they are computed on the TRUE force; on hardware only the measured
(sensor) force exists.
"""

from __future__ import annotations

import numpy as np


def iae(t, ref, y) -> float:
    """Integral of absolute error, N*s."""
    return float(np.trapezoid(np.abs(np.asarray(ref) - np.asarray(y)), t))


def rmse(ref, y) -> float:
    """Root-mean-square tracking error, N."""
    d = np.asarray(ref) - np.asarray(y)
    return float(np.sqrt(np.mean(d ** 2)))


def hold_stats(t, ref, y, windows, settle: float = 1.5):
    """Per-plateau statistics.

    windows : list of (t_start, t_end) of constant-reference plateaus
    settle  : seconds at the start of each plateau ignored for bias/ripple
              (the response is still settling)

    Returns dict with
      bias_N       mean(y - ref) after the settling time, averaged over plateaus
      ripple_N     standard deviation of y in the last 2 s of each plateau, averaged
      overshoot_pct  max(y - ref)/ref_plateau*100 inside the plateau (0 if never above)
    """
    t = np.asarray(t)
    ref = np.asarray(ref)
    y = np.asarray(y)
    bias, ripple, over = [], [], []
    for a, b in windows:
        amp = float(np.max(ref[(t >= a) & (t <= b)]))
        m_all = (t >= a) & (t <= b)
        m_set = (t >= a + settle) & (t <= b)
        m_end = (t >= b - 2.0) & (t <= b)
        bias.append(float(np.mean(y[m_set] - ref[m_set])))
        ripple.append(float(np.std(y[m_end])))
        over.append(max(0.0, float(np.max(y[m_all] - amp)) / amp * 100.0))
    return {
        "bias_N": float(np.mean(bias)),
        "ripple_N": float(np.mean(ripple)),
        "overshoot_pct": float(np.mean(over)),
    }


def effort(t, u, u_max: float = 1.0) -> dict:
    """Actuator effort: mean |du/dt| (1/s) and the fraction of time spent at the limit."""
    t = np.asarray(t)
    u = np.asarray(u)
    du = np.abs(np.diff(u)) / np.diff(t)
    return {
        "mean_abs_du_per_s": float(np.mean(du)),
        "pct_time_at_limit": float(np.mean(u >= 0.999 * u_max) * 100.0),
    }


def step_metrics(t, y, y0: float, y1: float, t_step: float) -> dict:
    """Rise time (10-90 %) and peak overshoot of a step from y0 to y1 applied at t_step."""
    t = np.asarray(t)
    y = np.asarray(y)
    m = t >= t_step
    ts_, ys = t[m] - t_step, y[m]
    span = y1 - y0
    lo, hi = y0 + 0.1 * span, y0 + 0.9 * span
    try:
        t10 = ts_[np.argmax(ys >= lo)]
        t90 = ts_[np.argmax(ys >= hi)]
        rise = float(t90 - t10)
    except ValueError:
        rise = float("nan")
    over = max(0.0, (float(np.max(ys)) - y1) / span * 100.0)
    return {"rise_time_s": rise, "overshoot_pct": over}


def fatigue_stats(t, ref, y, hold_start: float, hold_end: float, win: float = 5.0) -> dict:
    """Force held at the start and at the end of one long contraction.

    Returns mean force in the first and last `win` seconds and their ratio.
    """
    t = np.asarray(t)
    y = np.asarray(y)
    a = float(np.mean(y[(t >= hold_start + 2.0) & (t <= hold_start + 2.0 + win)]))
    b = float(np.mean(y[(t >= hold_end - win) & (t <= hold_end)]))
    return {"force_start_N": a, "force_end_N": b, "end_over_start_pct": b / a * 100.0}
