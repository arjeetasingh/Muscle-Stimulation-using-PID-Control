"""Performance report and plot for one logged session (hardware or simulated).

    python -m muscle_stim.analyze_session session_01.csv --out session_01.png

Only rows in state RUNNING (2) are analysed. Plateaus of the reference (constant for >= 2 s and
above 0.5 N) are found automatically and used for bias, ripple and overshoot. If a plateau lasts
at least 20 s the fatigue statistics (force at start vs end) are reported too.
On hardware only the MEASURED force exists, so all numbers are computed on it.
"""

from __future__ import annotations

import argparse

import numpy as np
import pandas as pd

from . import metrics


def find_plateaus(t, ref, min_len=2.0, min_ref=0.5, tol=1e-3):
    t = np.asarray(t)
    ref = np.asarray(ref)
    out = []
    start = None
    for i in range(1, len(t) + 1):
        steady = i < len(t) and abs(ref[i] - ref[i - 1]) < tol and ref[i] >= min_ref
        if steady and start is None:
            start = i - 1
        if not steady and start is not None:
            if t[i - 1] - t[start] >= min_len:
                out.append((float(t[start]), float(t[i - 1])))
            start = None
    return out


def analyse(df: pd.DataFrame) -> dict:
    d = df[df["state"] == 2].copy()
    if d.empty:
        raise ValueError("no RUNNING rows (state = 2) in this file")
    d["t_s"] = (d["t_ms"] - d["t_ms"].iloc[0]) / 1000.0
    t, ref, y, u = d["t_s"].to_numpy(), d["ref_N"].to_numpy(), d["force_N"].to_numpy(), d["u"].to_numpy()
    rep = {"duration_s": float(t[-1]), "rmse_N": metrics.rmse(ref, y), "iae_Ns": metrics.iae(t, ref, y)}
    rep.update(metrics.effort(t, u))
    pl = find_plateaus(t, ref)
    if pl:
        rep.update(metrics.hold_stats(t, ref, y, pl))
        rep["n_plateaus"] = len(pl)
        long_ = [p for p in pl if p[1] - p[0] >= 20.0]
        if long_:
            rep.update(metrics.fatigue_stats(t, ref, y, long_[0][0], long_[0][1]))
    for col in ("i_peak_mA", "t_skin_C", "v_rail_V", "hr_bpm"):
        if col in d and d[col].notna().any():
            rep[col + "_max"] = float(d[col].max())
    rep["_t"], rep["_ref"], rep["_y"], rep["_u"] = t, ref, y, u
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--out", default=None, help="PNG file for the plot")
    args = ap.parse_args(argv)
    df = pd.read_csv(args.csv, comment="#")
    rep = analyse(df)
    for k, v in rep.items():
        if not k.startswith("_"):
            print(f"{k:22s} {v:.4f}" if isinstance(v, float) else f"{k:22s} {v}")
    if args.out:
        from .plotting import use_style, STYLES
        import matplotlib.pyplot as plt
        use_style()
        fig, ax = plt.subplots(2, 1, figsize=(10, 5), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]})
        ax[0].plot(rep["_t"], rep["_ref"], label="Reference", **STYLES["Reference"])
        ax[0].plot(rep["_t"], rep["_y"], label="Measured force", **STYLES["PID"])
        ax[0].set_ylabel("Force (N)")
        ax[0].legend()
        ax[1].plot(rep["_t"], rep["_u"], color="black", lw=1.2)
        ax[1].set_ylabel("Intensity u (-)")
        ax[1].set_xlabel("Time (s)")
        fig.tight_layout()
        fig.savefig(args.out, bbox_inches="tight")


if __name__ == "__main__":
    main()
