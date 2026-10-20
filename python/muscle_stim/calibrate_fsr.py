"""Calibrate the FSR force sensor in newtons and produce the look-up table for the firmware.

How to collect the data (see docs/hardware.md, "FSR calibration"):
  * Mount the FSR exactly as it will be used (same rigid backing, same force-spreading pad).
  * Type the serial command  ADC  on the device: it prints the averaged raw ADC counts
    (0..4095) of the force sensor. Put a known mass on the pad, wait 5 s, send ADC and note the
    value. Repeat for at least 8 different masses from 0 g up to about 1 kg (about 10 N), and
    more than once each if you can.
  * Write them to a CSV with two columns:  mass_g,adc

Usage:
    python -m muscle_stim.calibrate_fsr fsr_points.csv --out fsr_lut.txt --plot fsr_calibration.png

Two physically motivated models are fitted (c = raw counts, c0 = counts with no load):
    power    : F = a * (c - c0)^b
    divider  : F = a * ((c - c0)/(cmax - (c - c0)))^b      (voltage-divider saturation)
The model with the lower leave-one-out RMSE is used to write a 16-point table that the firmware
interpolates linearly. The leave-one-out error is the honest estimate of the calibration accuracy;
an FSR is typically only good to about +/-10 % even after calibration (drift, hysteresis).
"""

from __future__ import annotations

import argparse

import numpy as np
from scipy.optimize import curve_fit

G = 9.80665  # m/s^2


def _power(x, a, b):
    return a * np.power(np.maximum(x, 0.0), b)


def _divider(x, a, b, cmax):
    x = np.maximum(x, 0.0)
    return a * np.power(x / np.maximum(cmax - x, 1e-9), b)


def _fit(model, x, y):
    if model == "power":
        p0, bounds = [0.01, 1.0], ([1e-9, 0.2], [1e3, 3.0])
        f = _power
    else:
        xm = float(np.max(x))
        p0, bounds = [1.0, 1.0, xm * 1.3], ([1e-9, 0.2, xm * 1.001], [1e4, 3.0, xm * 20.0])
        f = _divider
    popt, _ = curve_fit(f, x, y, p0=p0, bounds=bounds, maxfev=20000)
    return f, popt


def calibrate(mass_g, adc, n_lut: int = 16):
    """Return dict with the chosen model, parameters, errors and the (adc, N) table."""
    mass = np.asarray(mass_g, float)
    c = np.asarray(adc, float)
    f_n = mass / 1000.0 * G
    zero = mass <= 0.0
    if not np.any(zero):
        raise ValueError("include at least one zero-load row (mass_g = 0)")
    c0 = float(np.mean(c[zero]))
    x = np.maximum(c - c0, 0.0)
    use = ~zero
    if use.sum() < 6:
        raise ValueError("need at least 6 loaded points")
    results = {}
    for name in ("power", "divider"):
        try:
            f, p = _fit(name, x[use], f_n[use])
        except Exception:                              # model could not be fitted
            continue
        pred = f(x[use], *p)
        # leave-one-out cross validation
        idx = np.flatnonzero(use)
        loo = []
        for j in range(len(idx)):
            m = np.ones(len(idx), bool)
            m[j] = False
            try:
                f2, p2 = _fit(name, x[idx][m], f_n[idx][m])
                loo.append(f2(x[idx][j], *p2) - f_n[idx][j])
            except Exception:
                loo.append(np.nan)
        results[name] = {
            "f": f, "p": p,
            "rmse": float(np.sqrt(np.mean((pred - f_n[use]) ** 2))),
            "loo_rmse": float(np.sqrt(np.nanmean(np.square(loo)))),
            "max_abs": float(np.max(np.abs(pred - f_n[use]))),
            "r2": float(1 - np.sum((pred - f_n[use]) ** 2) / np.sum((f_n[use] - f_n[use].mean()) ** 2)),
        }
    if not results:
        raise RuntimeError("no model could be fitted to these points")
    best = min(results, key=lambda k: results[k]["loo_rmse"])
    r = results[best]
    top = float(np.max(c))
    grid = np.linspace(c0, top, n_lut)
    f_grid = r["f"](np.maximum(grid - c0, 0.0), *r["p"])
    f_grid = np.maximum.accumulate(f_grid)
    f_grid[0] = 0.0
    return {"best": best, "results": results, "c0": c0, "adc_grid": grid, "force_grid": f_grid,
            "x": x, "f_n": f_n, "zero": zero}


def lut_text(adc_grid, force_grid) -> str:
    a = ", ".join(f"{v:.1f}f" for v in adc_grid)
    f = ", ".join(f"{v:.4f}f" for v in force_grid)
    n = len(adc_grid)
    return (f"constexpr int FSR_LUT_N = {n};\n"
            f"constexpr float FSR_LUT_ADC[FSR_LUT_N] = {{ {a} }};\n"
            f"constexpr float FSR_LUT_N_FORCE[FSR_LUT_N] = {{ {f} }};\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="CSV with columns mass_g,adc")
    ap.add_argument("--out", default="fsr_lut.txt")
    ap.add_argument("--plot", default=None, help="optional PNG with the fit")
    args = ap.parse_args(argv)
    import pandas as pd
    d = pd.read_csv(args.csv)
    d = d.groupby("mass_g", as_index=False)["adc"].mean()      # average repeated readings
    res = calibrate(d["mass_g"].to_numpy(), d["adc"].to_numpy())
    for name, r in res["results"].items():
        tag = "  <== used" if name == res["best"] else ""
        print(f"{name:8s} params={np.round(r['p'], 5)}  RMSE={r['rmse']:.3f} N  "
              f"leave-one-out RMSE={r['loo_rmse']:.3f} N  max|err|={r['max_abs']:.3f} N  R2={r['r2']:.4f}{tag}")
    txt = lut_text(res["adc_grid"], res["force_grid"])
    with open(args.out, "w") as fh:
        fh.write(txt)
    print(txt)
    print(f"written to {args.out}  (paste into firmware/muscle_stim_pid/config.h)")
    if args.plot:
        from .plotting import use_style
        import matplotlib.pyplot as plt
        use_style()
        fig, ax = plt.subplots(1, 2, figsize=(9, 3.4))
        c = d["adc"].to_numpy()
        ax[0].plot(c, d["mass_g"] / 1000 * G, "o", mfc="white", mec="black", label="Known masses")
        ax[0].plot(res["adc_grid"], res["force_grid"], color="black", label=f"Table ({res['best']} fit)")
        ax[0].set_xlabel("ADC counts")
        ax[0].set_ylabel("Force (N)")
        ax[0].legend()
        r = res["results"][res["best"]]
        use = ~res["zero"]
        ax[1].axhline(0, color="black", lw=0.8)
        ax[1].plot(res["f_n"][use], r["f"](res["x"][use], *r["p"]) - res["f_n"][use], "o", mfc="white", mec="black")
        ax[1].set_xlabel("Applied force (N)")
        ax[1].set_ylabel("Fit minus applied (N)")
        fig.tight_layout()
        fig.savefig(args.plot, bbox_inches="tight")


if __name__ == "__main__":
    main()
